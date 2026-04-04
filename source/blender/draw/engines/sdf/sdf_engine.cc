/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw_engine
 *
 * SDF draw engine: analytical sphere tracer — evaluates SDF primitives directly per-pixel.
 */

#include <algorithm>

#include "BLI_math_matrix.h"
#include "BLI_math_matrix.hh"
#include "BLI_math_rotation.h"
#include "BLI_math_vector.h"
#include "BLI_math_vector.hh"

#include "BKE_global.hh"
#include "BKE_main.hh"
#include "BKE_object.hh"
#include "BKE_object_types.hh"
#include "BKE_sdf.hh"
#include "BKE_sdf_group.hh"
#include "BKE_studiolight.h"

#include "DEG_depsgraph_query.hh"

#include "DNA_modifier_types.h"
#include "DNA_object_types.h"
#include "DNA_sdf_group_types.h"
#include "DNA_sdf_types.h"
#include "DNA_userdef_types.h"
#include "DNA_view3d_enums.h"
#include "DNA_view3d_types.h"

#include "ED_view3d.hh"

#include "IMB_imbuf_types.hh"

#include "MEM_guardedalloc.h"

#include "GPU_batch.hh"
#include "GPU_framebuffer.hh"
#include "GPU_shader.hh"
#include "GPU_shader_builtin.hh"
#include "GPU_state.hh"
#include "GPU_storage_buffer.hh"
#include "GPU_texture.hh"
#include "GPU_compute.hh"
#include "GPU_uniform_buffer.hh"

#include "draw_defines.hh"
#include "draw_manager.hh"
#include "draw_view.hh"
#include "draw_view_data.hh"

#include "sdf_cpu_eval.hh"
#include "sdf_private.hh"

#include "sdf_engine.h"
#include "sdf_meshing.hh"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>

namespace blender::draw::sdf {

using namespace draw;

/* Static state shared with overlay/selection code */
static int s_object_count = 0;
static gpu::StorageBuf *s_object_ssbo = nullptr;
static gpu::StorageBuf *s_modifier_ssbo = nullptr;
static gpu::StorageBuf *s_polygon_ssbo = nullptr;
static gpu::StorageBuf *s_group_ssbo = nullptr;
static gpu::StorageBuf *s_bvh_ssbo = nullptr;
static int s_bvh_root = -1;
static int s_group_count = 0;
static Vector<int> s_depsgraph_to_sorted;
static gpu::Texture *s_depth_tx = nullptr;
static gpu::Texture *s_gbuf_color_tx = nullptr;
static int2 s_render_size = {0, 0};
static int2 s_texture_size = {0, 0};
static const SDFObjectGPU *s_objects_cpu = nullptr;
static int s_objects_cpu_count = 0;
static const SDFPolygonPointGPU *s_polygon_pts_cpu = nullptr;
static int s_polygon_pts_count = 0;
static const SDFModifierGPU *s_modifiers_cpu = nullptr;
static int s_modifier_count = 0;

/* Static shader cache — survives engine instance destruction (mode switches). */
enum ShaderIndex {
  SH_TRACE_COMP = 0,
  SH_TRACE_TILE_COMP,
  SH_AABB_PROJECT_COMP,
  SH_TILE_CULL_COMP,
  SH_CONE_MARCH_COMP,
  SH_COLOR_RESOLVE_COMP,
  SH_NORMAL_COMP,
  SH_SHADE_COMP,
  SH_BLIT,
  SH_FXAA,
  SH_COUNT,
};

static constexpr const char *s_shader_info_names[SH_COUNT] = {
    "sdf_trace_comp",
    "sdf_trace_tile_comp",
    "sdf_aabb_project_comp",
    "sdf_tile_cull_comp",
    "sdf_cone_march_comp",
    "sdf_color_resolve_comp",
    "sdf_normal_comp",
    "sdf_shade_comp",
    "sdf_blit",
    "sdf_fxaa",
};

static gpu::Shader *s_shader_cache[SH_COUNT] = {};
static bool s_shaders_compiled = false;

static void sdf_shaders_ensure()
{
  if (s_shaders_compiled) {
    return;
  }
  auto t0 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < SH_COUNT; i++) {
    if (s_shader_cache[i] == nullptr) {
      auto ts = std::chrono::high_resolution_clock::now();
      s_shader_cache[i] = GPU_shader_create_from_info_name(s_shader_info_names[i]);
      auto te = std::chrono::high_resolution_clock::now();
      printf("SDF shader[%d] '%s' => %.0f ms\n", i, s_shader_info_names[i],
             std::chrono::duration<double, std::milli>(te - ts).count());
    }
  }
  auto t1 = std::chrono::high_resolution_clock::now();
  printf("SDF total shader compile: %.0f ms\n",
         std::chrono::duration<double, std::milli>(t1 - t0).count());
  s_shaders_compiled = true;
}

void sdf_shaders_free()
{
  for (int i = 0; i < SH_COUNT; i++) {
    GPU_SHADER_FREE_SAFE(s_shader_cache[i]);
  }
  s_shaders_compiled = false;
}

class Instance : public DrawEngine {
 private:
  Vector<SDFObjectGPU> objects_;
  Vector<SDFGroup *> object_group_ptrs_;
  Vector<Object *> object_ptrs_;

  float3 scene_min_ = float3(1e30f);
  float3 scene_max_ = float3(-1e30f);

  bool fxaa_enabled_ = true;
  float max_blend_ = 0.0f;
  float max_shell_distance_ = 0.0f;
  float step_factor_ = 0.85f;
  bool needs_upload_ = true;
  bool depth_mode_ = false;

  /* Aliases into static shader cache */
  gpu::Shader *&trace_comp_sh_ = s_shader_cache[SH_TRACE_COMP];
  gpu::Shader *&trace_tile_sh_ = s_shader_cache[SH_TRACE_TILE_COMP];
  gpu::Shader *&aabb_project_sh_ = s_shader_cache[SH_AABB_PROJECT_COMP];
  gpu::Shader *&tile_cull_sh_ = s_shader_cache[SH_TILE_CULL_COMP];
  gpu::Shader *&cone_march_sh_ = s_shader_cache[SH_CONE_MARCH_COMP];
  gpu::Shader *&color_resolve_sh_ = s_shader_cache[SH_COLOR_RESOLVE_COMP];
  gpu::Shader *&normal_comp_sh_ = s_shader_cache[SH_NORMAL_COMP];
  gpu::Shader *&shade_comp_sh_ = s_shader_cache[SH_SHADE_COMP];
  gpu::Shader *&blit_sh_ = s_shader_cache[SH_BLIT];
  gpu::Shader *&fxaa_sh_ = s_shader_cache[SH_FXAA];

  gpu::Texture *comp_color_tx_ = nullptr;
  gpu::Texture *comp_depth_tx_ = nullptr;
  gpu::Texture *gbuf_pos_tx_ = nullptr;
  gpu::Texture *gbuf_color_tx_ = nullptr;
  gpu::Texture *gbuf_normal_tx_ = nullptr;
  gpu::Texture *march_color_tx_ = nullptr;
  gpu::FrameBuffer *march_fb_ = nullptr;
  int2 render_size_ = int2(0);
  int2 prev_render_size_ = int2(0);
  int2 texture_size_ = int2(0);
  int2 viewport_size_ = int2(0);
  int2 fxaa_size_ = int2(0);
  float resolution_scale_ = 1.0f;
  bool adaptive_resolution_ = false;
  bool scene_changed_ = false;
  bool view_changed_ = false;
  bool mesh_changed_ = false;
  int scroll_cooldown_ = 0;
  int idle_frames_ = 0;
  bool compute_valid_ = false;
  uint64_t prev_data_hash_ = 0;
  uint64_t prev_mesh_hash_ = 0;
  uint64_t prev_shading_hash_ = 0;
  Vector<float4x4> mesh_transforms_;
  float4x4 prev_viewmat_ = float4x4::identity();
  float4x4 prev_winmat_ = float4x4::identity();

  gpu::StorageBuf *object_ssbo_ = nullptr;
  int object_ssbo_count_ = 0;

  Vector<SDFObjectAABB> object_aabbs_;
  gpu::StorageBuf *object_aabb_ssbo_ = nullptr;
  int object_aabb_ssbo_count_ = 0;

  Vector<SDFModifierGPU> modifiers_;
  gpu::StorageBuf *modifier_ssbo_ = nullptr;
  int modifier_ssbo_count_ = 0;

  Vector<SDFPolygonPointGPU> polygon_points_;
  gpu::StorageBuf *polygon_ssbo_ = nullptr;
  int polygon_ssbo_count_ = 0;

  Vector<SDFGroupGPU> groups_gpu_;
  gpu::StorageBuf *group_ssbo_ = nullptr;
  int group_ssbo_count_ = 0;

  SdfAabbTree bvh_tree_;
  gpu::StorageBuf *bvh_nodes_ssbo_ = nullptr;
  int bvh_nodes_ssbo_count_ = 0;

  gpu::StorageBuf *cone_hit_ssbo_ = nullptr;
  gpu::StorageBuf *tile_far_hint_ssbo_ = nullptr;
  gpu::StorageBuf *screen_aabbs_ssbo_ = nullptr;
  int screen_aabbs_ssbo_count_ = 0;

  gpu::StorageBuf *tile_prim_counts_ssbo_ = nullptr;
  int tile_prim_counts_ssbo_tiles_ = 0;
  gpu::StorageBuf *tile_prim_lists_ssbo_ = nullptr;

  gpu::Batch *fullscreen_batch_ = nullptr;

  const DRWContext *draw_ctx_ = nullptr;

  gpu::Texture *matcap_tx_ = nullptr;
  std::string current_matcap_;
  int lighting_type_ = V3D_LIGHTING_STUDIO;
  int use_specular_ = 0;
  int use_matcap_flip_ = 0;
  int use_bvh_ = 1;
  int debug_bvh_views_ = 0;
  int debug_fd_normals_ = 0;
  int use_cone_trace_ = 0;
  int sdf_max_steps_ = 256;
  float sdf_ray_epsilon_ = 0.001f;
  float sdf_over_relaxation_ = 1.2f;
  float sdf_cone_aperture_ = 1.25f;
  int sdf_cone_steps_ = 16;

  float4 frustum_planes_[6];
  bool frustum_valid_ = false;
  bool use_frustum_cull_ = true;

  float4 studio_light_dir_[4] = {};
  float4 studio_light_col_[4] = {};
  float4 studio_light_spec_[4] = {};
  float3 studio_ambient_ = float3(0.0f);

 public:
  Instance() {}

  blender::StringRefNull name_get() final
  {
    return "SDF";
  }

  void init() final
  {
    draw_ctx_ = DRW_context_get();

    sync_sdf_settings();

    sdf_shaders_ensure();
  }

  void begin_sync() final
  {
    depth_mode_ = draw_ctx_->is_depth();
    if (depth_mode_) {
      return;
    }

    objects_.clear();
    object_group_ptrs_.clear();
    object_ptrs_.clear();
    modifiers_.clear();
    polygon_points_.clear();
    groups_gpu_.clear();
    s_depsgraph_to_sorted.clear();
    bvh_tree_.clear();
    mesh_transforms_.clear();
    scene_min_ = float3(1e30f);
    scene_max_ = float3(-1e30f);
    max_blend_ = 0.0f;
    max_shell_distance_ = 0.0f;
    step_factor_ = 0.85f;
    frustum_valid_ = false;
  }

  void object_sync(ObjectRef &ob_ref, Manager & /*manager*/) final
  {
    if (depth_mode_) {
      return;
    }

    Object *ob = ob_ref.object;

    if (ob->type == OB_MESH) {
      mesh_transforms_.append(ob->object_to_world());
      return;
    }

    if (ob->type != OB_SDF) {
      return;
    }

    const SDF *sdf_data = id_cast<const SDF *>(ob->data);
    if (sdf_data == nullptr) {
      return;
    }

    if (!frustum_valid_) {
      const View &view = View::default_get();
      float4x4 vp = view.winmat() * view.viewmat();
      for (int i = 0; i < 4; i++) {
        frustum_planes_[0][i] = vp[i][3] + vp[i][0];
        frustum_planes_[1][i] = vp[i][3] - vp[i][0];
        frustum_planes_[2][i] = vp[i][3] + vp[i][1];
        frustum_planes_[3][i] = vp[i][3] - vp[i][1];
        frustum_planes_[4][i] = vp[i][3] + vp[i][2];
        frustum_planes_[5][i] = vp[i][3] - vp[i][2];
      }
      for (int p = 0; p < 6; p++) {
        frustum_planes_[p] /= math::length(float3(frustum_planes_[p]));
      }
      frustum_valid_ = true;
    }

    const float4x4 &mat = ob->object_to_world();

    float3 scale;
    scale.x = math::length(float3(mat[0]));
    scale.y = math::length(float3(mat[1]));
    scale.z = math::length(float3(mat[2]));

    float4x4 rot_mat = mat;
    if (scale.x > 0.0f) {
      rot_mat[0] = float4(float3(mat[0]) / scale.x, 0.0f);
    }
    if (scale.y > 0.0f) {
      rot_mat[1] = float4(float3(mat[1]) / scale.y, 0.0f);
    }
    if (scale.z > 0.0f) {
      rot_mat[2] = float4(float3(mat[2]) / scale.z, 0.0f);
    }
    rot_mat[3] = float4(0.0f, 0.0f, 0.0f, 1.0f);

    float4x4 inv_rot = math::invert(rot_mat);

    SDFObjectGPU gpu_obj = {};
    gpu_obj.inverse_matrix = inv_rot;
    gpu_obj.position = float4(mat[3].x, mat[3].y, mat[3].z, 0.0f);

    gpu_obj.sdf_size = float4(sdf_data->size[0] * scale.x,
                              sdf_data->size[1] * scale.y,
                              sdf_data->size[2] * scale.z,
                              0.0f);

    float bevel = 0.0f;
    for (const ModifierData *bmd = static_cast<const ModifierData *>(ob->modifiers.first); bmd; bmd = bmd->next) {
      if ((bmd->mode & eModifierMode_Realtime) && bmd->type == eModifierType_SDFBevel) {
        bevel += reinterpret_cast<const SDFBevelModifierData *>(bmd)->radius;
      }
    }
    bevel *= math::reduce_min(scale);
    gpu_obj.bevel = bevel;

    gpu_obj.blend = sdf_data->blend;
    gpu_obj.sdf_type = sdf_data->sdf_type;
    gpu_obj.blend_type = sdf_data->blend_type;
    gpu_obj.csg_operation = sdf_data->csg_operation;
    gpu_obj.shell_distance = sdf_data->shell_distance;
    gpu_obj.shell_mode = sdf_data->shell_mode;
    gpu_obj.shell_op = sdf_data->shell_op;
    gpu_obj.shell_blend_top = sdf_data->shell_blend_top;
    gpu_obj.shell_blend_bottom = sdf_data->shell_blend_bottom;
    gpu_obj.chamfer_k2 = sdf_data->chamfer_k2;
    gpu_obj.chamfer_k3 = sdf_data->chamfer_k3;

    /* Early AABB + frustum cull (before expensive polygon/modifier work) */
    {
      float shell_expand = (sdf_data->csg_operation == SDF_CSG_SHELL) ?
                               fabsf(sdf_data->shell_distance) :
                               0.0f;
      float blend_pad = (sdf_data->blend_type != 0) ? sdf_data->blend : 0.0f;
      float aabb_pad = blend_pad + shell_expand;
      float3 local_extent;
      float3 sz = float3(gpu_obj.sdf_size);
      switch (sdf_data->sdf_type) {
        case SDF_TYPE_SPHERE:
          local_extent = sz + float3(bevel + aabb_pad);
          break;
        case SDF_TYPE_CAPSULE: {
          float r = sz.x;
          float h = math::max(sz.y - bevel, 0.0f);
          local_extent = float3(r + aabb_pad, r + aabb_pad, h + r + aabb_pad);
          break;
        }
        case SDF_TYPE_CYLINDER:
          local_extent = sz + float3(bevel + aabb_pad);
          break;
        case SDF_TYPE_CONE: {
          float r = sz.x;
          float h = sz.y;
          local_extent = float3(r + bevel + aabb_pad, r + bevel + aabb_pad, h + bevel + aabb_pad);
          break;
        }
        case SDF_TYPE_TORUS: {
          float outer = sz.x + sz.y;
          local_extent = float3(outer + aabb_pad, outer + aabb_pad, sz.y + aabb_pad);
          break;
        }
        case SDF_TYPE_NGON: {
          float r = sz.x;
          local_extent = float3(r + bevel + aabb_pad, r + bevel + aabb_pad, sz.z + bevel + aabb_pad);
          break;
        }
        case SDF_TYPE_POLYGON: {
          float ps = math::min(scale.x, scale.y);
          float max_xy = 0.0f;
          for (const SDFPolygonPoint *pt = static_cast<const SDFPolygonPoint *>(sdf_data->polygon_points.first); pt; pt = pt->next) {
            max_xy = math::max(max_xy, fabsf(pt->co[0]) * ps);
            max_xy = math::max(max_xy, fabsf(pt->co[1]) * ps);
          }
          local_extent = float3(
              max_xy + bevel + aabb_pad, max_xy + bevel + aabb_pad, sz.z + bevel + aabb_pad);
          break;
        }
        default:
          local_extent = sz + float3(bevel + aabb_pad);
          break;
      }

      for (int mi = gpu_obj.modifier_start;
           mi < gpu_obj.modifier_start + gpu_obj.modifier_count; mi++)
      {
        const SDFModifierGPU &mod = modifiers_[mi];
        int mtype = mod.header.x;
        int mflags = mod.header.y;
        switch (mtype) {
          case SDF_MOD_MIRROR: {
            float offset = fabsf(mod.params.x);
            float3 local_org = float3(mod.params.y, mod.params.z, mod.params.w);
            if ((mflags & SDF_MOD_MIRROR_X) != 0) {
              float3 N = float3(inv_rot[0]);
              float extent_along = math::dot(local_extent, math::abs(N));
              float disp = fmax(2.0f * fabsf(math::dot(local_org, N)) + offset, extent_along);
              local_extent += math::abs(N) * disp;
            }
            if ((mflags & SDF_MOD_MIRROR_Y) != 0) {
              float3 N = float3(inv_rot[1]);
              float extent_along = math::dot(local_extent, math::abs(N));
              float disp = fmax(2.0f * fabsf(math::dot(local_org, N)) + offset, extent_along);
              local_extent += math::abs(N) * disp;
            }
            if ((mflags & SDF_MOD_MIRROR_Z) != 0) {
              float3 N = float3(inv_rot[2]);
              float extent_along = math::dot(local_extent, math::abs(N));
              float disp = fmax(2.0f * fabsf(math::dot(local_org, N)) + offset, extent_along);
              local_extent += math::abs(N) * disp;
            }
            break;
          }
          case SDF_MOD_ELONGATE:
            local_extent += float3(mod.params.x, mod.params.y, mod.params.z);
            break;
          case SDF_MOD_SOLIDIFY:
          case SDF_MOD_ONION:
            break;
          case SDF_MOD_ROUND:
          case SDF_MOD_BEVEL:
            local_extent = math::max(local_extent + float3(mod.params.x), float3(0.0f));
            break;
          case SDF_MOD_TWIST: {
            float xy = math::sqrt(local_extent.x * local_extent.x +
                                  local_extent.y * local_extent.y);
            local_extent.x = xy;
            local_extent.y = xy;
            break;
          }
          case SDF_MOD_BEND: {
            float k = mod.params.x;
            int axis = int(mod.params.y);
            float3 origin = float3(mod.params.z, mod.params.w, mod.params2.x);
            if (fabsf(k) > 0.0001f) {
              float3 new_ext = float3(0.0f);
              for (int c = 0; c < 8; c++) {
                float3 pt = float3((c & 1) ? local_extent.x : -local_extent.x,
                                   (c & 2) ? local_extent.y : -local_extent.y,
                                   (c & 4) ? local_extent.z : -local_extent.z);
                pt -= origin;
                float drive, curve, free_val;
                if (axis == 1) { drive = pt.y; curve = pt.z; free_val = pt.x; }
                else if (axis == 2) { drive = pt.z; curve = pt.x; free_val = pt.y; }
                else { drive = pt.x; curve = pt.y; free_val = pt.z; }
                float a = k * drive;
                float nd = cosf(a) * drive - sinf(a) * curve;
                float nc = sinf(a) * drive + cosf(a) * curve;
                float3 bent;
                if (axis == 1) { bent = float3(free_val, nd, nc); }
                else if (axis == 2) { bent = float3(nc, free_val, nd); }
                else { bent = float3(nd, nc, free_val); }
                bent += origin;
                new_ext = math::max(new_ext, math::abs(bent));
              }
              local_extent = new_ext;
            }
            break;
          }
          case SDF_MOD_ARRAY: {
            float count = mod.params.x;
            if (count > 0.5f) {
              if (mflags == SDF_MOD_ARRAY_LINEAR) {
                float3 arr_offset = float3(mod.params.y, mod.params.z, mod.params.w);
                local_extent += math::abs(arr_offset) * (count - 1.0f);
              }
              else if (mflags == SDF_MOD_ARRAY_RADIAL) {
                float radius = mod.params.y;
                local_extent.x += radius;
                local_extent.y += radius;
                float3 rot_off(mod.params2.y, mod.params2.z, mod.params2.w);
                if (math::length(rot_off) > 0.0001f) {
                  float r = math::length(local_extent);
                  local_extent = float3(r);
                }
              }
            }
            break;
          }
          default:
            break;
        }
      }

      float3 world_min = float3(1e30f);
      float3 world_max = float3(-1e30f);
      for (int corner = 0; corner < 8; corner++) {
        float3 lc = float3((corner & 1) ? local_extent.x : -local_extent.x,
                           (corner & 2) ? local_extent.y : -local_extent.y,
                           (corner & 4) ? local_extent.z : -local_extent.z);
        float3 wc = float3(rot_mat * float4(lc, 0.0f)) +
                     float3(mat[3].x, mat[3].y, mat[3].z);
        world_min = math::min(world_min, wc);
        world_max = math::max(world_max, wc);
      }

      gpu_obj.bbox_min = float4(world_min, 0.0f);
      gpu_obj.bbox_max = float4(world_max, 0.0f);
      gpu_obj.orig_bbox_min = gpu_obj.bbox_min;
      gpu_obj.orig_bbox_max = gpu_obj.bbox_max;

      float sphere_radius = math::length(local_extent);
      float3 obj_center = float3(mat[3].x, mat[3].y, mat[3].z);
      scene_min_ = math::min(scene_min_, obj_center - float3(sphere_radius));
      scene_max_ = math::max(scene_max_, obj_center + float3(sphere_radius));

      /* Frustum cull deferred to end_sync where modifier-expanded AABBs are available */
    }

    gpu_obj.group_id = -1;
    gpu_obj.original_index = sdf_data->sdf_index;

    gpu_obj.color = float4(
        sdf_data->color[0], sdf_data->color[1], sdf_data->color[2], sdf_data->color[3]);

    /* Shape-specific data */
    if (sdf_data->sdf_type == SDF_TYPE_NGON) {
      float ngon_taper = sdf_data->ngon_taper;
      gpu_obj.box_corners = float4(sdf_data->ngon_corner, sdf_data->ngon_star, 0.0f, 0.0f);
      gpu_obj.box_edges = float4(sdf_data->ngon_edge_top,
                                 sdf_data->ngon_edge_bottom,
                                 math::max(ngon_taper, 0.0f),
                                 math::max(-ngon_taper, 0.0f));
      gpu_obj.box_modes = int4(0, sdf_data->ngon_edge_mode, sdf_data->ngon_sides, 0);
    }
    else if (sdf_data->sdf_type == SDF_TYPE_POLYGON) {
      float poly_taper = sdf_data->polygon_taper;
      gpu_obj.polygon_point_start = int(polygon_points_.size());
      gpu_obj.polygon_point_count = 0;

      /* Collect original points (uniform XY scale preserves aspect ratio) */
      float poly_scale = math::min(scale.x, scale.y);
      Vector<float2> pts;
      Vector<float> crn;
      for (const SDFPolygonPoint *pt = static_cast<const SDFPolygonPoint *>(sdf_data->polygon_points.first); pt; pt = pt->next) {
        pts.append(float2(pt->co[0] * poly_scale, pt->co[1] * poly_scale));
        crn.append(pt->corner * poly_scale);
      }
      int pc = int(pts.size());

      float max_corner = 0.0f;
      for (int i = 0; i < pc; i++) {
        max_corner = math::max(max_corner, crn[i]);
      }

      /* Radius clamping pre-pass */
      Vector<float> half_angles(pc);
      Vector<float> tan_halfs(pc);
      for (int i = 0; i < pc; i++) {
        int ip = (i - 1 + pc) % pc;
        int j = (i + 1) % pc;
        float2 to_prev = math::normalize(pts[ip] - pts[i]);
        float2 to_next = math::normalize(pts[j] - pts[i]);
        float cos_theta = math::clamp(math::dot(to_prev, to_next), -0.999f, 0.999f);
        half_angles[i] = acosf(cos_theta) * 0.5f;
        tan_halfs[i] = tanf(half_angles[i]);
      }

      for (int i = 0; i < pc; i++) {
        int j = (i + 1) % pc;
        float edge_len = math::length(pts[j] - pts[i]);
        if (edge_len < 1e-6f) {
          continue;
        }
        float t_i = (tan_halfs[i] > 1e-6f) ? crn[i] / tan_halfs[i] : 0.0f;
        float t_j = (tan_halfs[j] > 1e-6f) ? crn[j] / tan_halfs[j] : 0.0f;
        if (t_i + t_j > edge_len) {
          float scale = edge_len / (t_i + t_j);
          crn[i] *= scale;
          crn[j] *= scale;
        }
      }

      /* Precompute per-edge data */
      for (int i = 0; i < pc; i++) {
        int j = (i + 1) % pc;
        int ip = (i - 1 + pc) % pc;

        float2 edge = pts[j] - pts[i];
        float edge_len = math::length(edge);

        SDFPolygonPointGPU gpu_pt = {};
        gpu_pt.vi_edge = float4(pts[i].x, pts[i].y, edge.x, edge.y);

        float R_signed = 0.0f;
        float2 C = pts[i];
        float t_trim_start = 0.0f;
        float t_trim_end = 1.0f;
        float ang_mid = 0.0f;
        float ang_half = 0.0f;

        /* Recompute half-angle/tan for clamped radii */
        float2 to_prev = math::normalize(pts[ip] - pts[i]);
        float2 to_next = math::normalize(pts[j] - pts[i]);
        float cos_theta = math::clamp(math::dot(to_prev, to_next), -0.999f, 0.999f);
        float half = acosf(cos_theta) * 0.5f;
        float sin_half = sinf(half);
        float tan_half = tanf(half);

        if (crn[i] > 0.001f && pc >= 3 && sin_half > 0.01f && tan_half > 1e-6f) {
          float2 bisector = math::normalize(to_prev + to_next);
          float cross_val = to_prev.x * to_next.y - to_prev.y * to_next.x;

          float inset_dist = crn[i] / sin_half;
          C = pts[i] + bisector * inset_dist;

          R_signed = (cross_val > 0.0f) ? -crn[i] : crn[i];

          /* Trim start on this edge */
          if (edge_len > 1e-6f) {
            t_trim_start = crn[i] / (tan_half * edge_len);
          }

          /* Arc angular bounds */
          float2 tangent_start = pts[i] + math::normalize(edge) * (crn[i] / tan_half);
          float2 dir_start = math::normalize(tangent_start - C);
          float2 tangent_prev_end = pts[i] + to_prev * (crn[i] / tan_half);
          float2 dir_prev_end = math::normalize(tangent_prev_end - C);

          float a1 = atan2f(dir_prev_end.y, dir_prev_end.x);
          float a2 = atan2f(dir_start.y, dir_start.x);

          float diff = a2 - a1;
          diff -= 6.2831853f * floorf((diff + 3.1415927f) / 6.2831853f);
          ang_mid = a1 + diff * 0.5f;
          ang_half = fabsf(diff) * 0.5f;
        }

        /* Trim end on this edge (from next vertex's rounding) */
        if (crn[j] > 0.001f && pc >= 3) {
          float2 to_prev_j = math::normalize(pts[i] - pts[j]);
          float2 to_next_j = math::normalize(pts[(j + 1) % pc] - pts[j]);
          float cos_theta_j = math::clamp(math::dot(to_prev_j, to_next_j), -0.999f, 0.999f);
          float tan_half_j = tanf(acosf(cos_theta_j) * 0.5f);
          if (edge_len > 1e-6f && tan_half_j > 1e-6f) {
            t_trim_end = 1.0f - crn[j] / (tan_half_j * edge_len);
          }
        }

        gpu_pt.arc_data = float4(R_signed, C.x, C.y, t_trim_start);
        gpu_pt.arc_bounds = float4(t_trim_end, ang_mid, ang_half, 0.0f);

        polygon_points_.append(gpu_pt);
        gpu_obj.polygon_point_count++;
      }
      gpu_obj.box_corners = float4(max_corner, 0.0f, 0.0f, 0.0f);
      gpu_obj.box_edges = float4(sdf_data->polygon_edge_top,
                                 sdf_data->polygon_edge_bottom,
                                 math::max(poly_taper, 0.0f),
                                 math::max(-poly_taper, 0.0f));
      gpu_obj.box_modes = int4(0, sdf_data->polygon_edge_mode, 0, 0);
    }
    else if (sdf_data->sdf_type == SDF_TYPE_TORUS) {
      float angle_rad = sdf_data->torus_angle;
      float half_rad = angle_rad * 0.5f;
      gpu_obj.box_corners = float4(sinf(half_rad), cosf(half_rad), 0.0f, 0.0f);
      gpu_obj.box_edges = float4(0.0f);
      gpu_obj.box_modes = int4(0, 0, 0, (angle_rad < (float(M_PI) * 2.0f) - 0.001f) ? 1 : 0);
    }
    else {
      gpu_obj.box_corners = float4(sdf_data->box_corners[0],
                                   sdf_data->box_corners[1],
                                   sdf_data->box_corners[2],
                                   sdf_data->box_corners[3]);
      float taper = sdf_data->box_taper;
      gpu_obj.box_edges = float4(sdf_data->box_edge_top,
                                 sdf_data->box_edge_bottom,
                                 math::max(taper, 0.0f),
                                 math::max(-taper, 0.0f));
      gpu_obj.box_modes = int4(sdf_data->box_corner_mode, sdf_data->box_edge_mode, 0, 0);
    }

    /* Pack modifiers from native modifier stack (old internal SDFModifier list is deprecated) */
    gpu_obj.modifier_start = int(modifiers_.size());
    gpu_obj.modifier_count = 0;

    /* Native Blender modifier stack */
    for (const ModifierData *md = static_cast<const ModifierData *>(ob->modifiers.first); md; md = md->next) {
      if (!(md->mode & eModifierMode_Realtime)) {
        continue;
      }
      SDFModifierGPU gpu_mod = {};
      bool valid = false;
      switch (md->type) {
        case eModifierType_SDFMirror: {
          const auto &m = *reinterpret_cast<const SDFMirrorModifierData *>(md);
          float3 obj_pos = float3(mat[3]);
          float3 mirror_pos(0.0f);
          if (m.mirror_object != nullptr) {
            mirror_pos = float3(m.mirror_object->object_to_world()[3]);
          }
          float3 world_origin = mirror_pos - obj_pos;
          float3 local_origin = float3(inv_rot * float4(world_origin, 0.0f));
          /* Side: fold toward the object center (stable across rotation). */
          int sides = ((obj_pos.x >= mirror_pos.x) ? 1 : 0) |
                      ((obj_pos.y >= mirror_pos.y) ? 2 : 0) |
                      ((obj_pos.z >= mirror_pos.z) ? 4 : 0);
          gpu_mod.header = int4(SDF_MOD_MIRROR, m.flag, m.blend_type, sides);
          gpu_mod.params = float4(m.offset_distance, local_origin.x, local_origin.y, local_origin.z);
          gpu_mod.params2 = float4(m.blend, 0.0f, float(m.blend_type), 0.0f);
          valid = true;
          break;
        }
        case eModifierType_SDFTwist: {
          const auto &m = *reinterpret_cast<const SDFTwistModifierData *>(md);
          gpu_mod.header = int4(SDF_MOD_TWIST, 0, 0, 0);
          gpu_mod.params = float4(m.strength, float(m.axis), 0.0f, 0.0f);
          valid = true;
          break;
        }
        case eModifierType_SDFBend: {
          const auto &m = *reinterpret_cast<const SDFBendModifierData *>(md);
          gpu_mod.header = int4(SDF_MOD_BEND, 0, 0, 0);
          gpu_mod.params = float4(m.strength, float(m.axis), m.origin[0], m.origin[1]);
          gpu_mod.params2 = float4(m.origin[2], 0.0f, 0.0f, 0.0f);
          valid = true;
          break;
        }
        case eModifierType_SDFElongate: {
          const auto &m = *reinterpret_cast<const SDFElongateModifierData *>(md);
          gpu_mod.header = int4(SDF_MOD_ELONGATE, 0, 0, 0);
          gpu_mod.params = float4(m.elongation[0], m.elongation[1], m.elongation[2], 0.0f);
          valid = true;
          break;
        }
        case eModifierType_SDFSolidify: {
          const auto &m = *reinterpret_cast<const SDFSolidifyModifierData *>(md);
          float inner_scale = 1.0f;
          if (m.mode == SDF_SOLIDIFY_OPEN) {
            int ax = math::clamp(m.axis, 0, 2);
            float sz = math::max(sdf_data->size[ax] * scale[ax], 0.001f);
            inner_scale = math::max(1.0f - 2.0f * m.thickness / sz, 0.1f);
          }
          gpu_mod.header = int4(SDF_MOD_SOLIDIFY, m.mode, m.axis, 0);
          gpu_mod.params = float4(m.thickness, inner_scale, m.bevel, 0.0f);
          valid = true;
          break;
        }
        case eModifierType_SDFRound: {
          const auto &m = *reinterpret_cast<const SDFRoundModifierData *>(md);
          gpu_mod.header = int4(SDF_MOD_ROUND, 0, 0, 0);
          gpu_mod.params = float4(m.offset, 0.0f, 0.0f, 0.0f);
          valid = true;
          break;
        }
        case eModifierType_SDFOnion: {
          const auto &m = *reinterpret_cast<const SDFOnionModifierData *>(md);
          int layers = math::max(m.layers, 1);
          float3 sz = float3(gpu_obj.sdf_size);
          float min_ext = math::min(sz.x, math::min(sz.y, sz.z));
          gpu_mod.header = int4(SDF_MOD_ONION, layers, 0, 0);
          gpu_mod.params = float4(m.gap, min_ext, 0.0f, 0.0f);
          valid = true;
          break;
        }
        case eModifierType_SDFBevel: {
          const auto &m = *reinterpret_cast<const SDFBevelModifierData *>(md);
          gpu_mod.header = int4(SDF_MOD_BEVEL, 0, 0, 0);
          gpu_mod.params = float4(m.radius, 0.0f, 0.0f, 0.0f);
          valid = true;
          break;
        }
        case eModifierType_SDFArray: {
          const auto &m = *reinterpret_cast<const SDFArrayModifierData *>(md);
          gpu_mod.header = int4(SDF_MOD_ARRAY, m.array_type, m.blend_type, 0);
          if (m.array_type == MOD_SDF_ARRAY_RADIAL) {
            gpu_mod.params = float4(float(m.count), m.array_radius, 0.0f, 0.0f);
            gpu_mod.params2 = float4(m.blend,
                                     m.rotation_offset[0],
                                     m.rotation_offset[1],
                                     m.rotation_offset[2]);
          }
          else {
            float3 sz = float3(sdf_data->size);
            float3 offset(0.0f);
            if (m.use_relative_offset) {
              offset += float3(m.relative_offset[0], m.relative_offset[1], m.relative_offset[2]) *
                        sz * 2.0f;
            }
            if (m.use_constant_offset) {
              offset += float3(m.constant_offset[0], m.constant_offset[1], m.constant_offset[2]);
            }
            gpu_mod.params = float4(float(m.count), offset.x, offset.y, offset.z);
            gpu_mod.params2 = float4(m.blend, 0.0f, float(m.blend_type), 0.0f);
          }
          valid = true;
          break;
        }
        default:
          break;
      }
      if (valid) {
        modifiers_.append(gpu_mod);
        gpu_obj.modifier_count++;
      }
    }

    /* Compute AABB */
    float shell_expand = (sdf_data->csg_operation == SDF_CSG_SHELL) ?
                             fabsf(sdf_data->shell_distance) :
                             0.0f;
    float blend_pad = (sdf_data->blend_type != 0) ? sdf_data->blend : 0.0f;
    float pad = blend_pad + shell_expand;
    float3 local_extent;
    float3 sz = float3(gpu_obj.sdf_size);
    switch (sdf_data->sdf_type) {
      case SDF_TYPE_CAPSULE: {
        float r = sz.x;
        float h = math::max(sz.y - bevel, 0.0f);
        local_extent = float3(r + pad, r + pad, h + r + pad);
        break;
      }
      case SDF_TYPE_CYLINDER: {
        local_extent = sz + float3(bevel + pad);
        break;
      }
      case SDF_TYPE_CONE: {
        float r = sz.x;
        float h = sz.y;
        local_extent = float3(r + bevel + pad, r + bevel + pad, h + bevel + pad);
        break;
      }
      case SDF_TYPE_TORUS: {
        float outer = sz.x + sz.y;
        local_extent = float3(outer + pad, outer + pad, sz.y + pad);
        break;
      }
      case SDF_TYPE_NGON: {
        float r = sz.x;
        local_extent = float3(r + bevel + pad, r + bevel + pad, sz.z + bevel + pad);
        break;
      }
      case SDF_TYPE_POLYGON: {
        float ps = math::min(scale.x, scale.y);
        float max_xy = 0.0f;
        for (const SDFPolygonPoint *pt = static_cast<const SDFPolygonPoint *>(sdf_data->polygon_points.first); pt; pt = pt->next) {
          max_xy = math::max(max_xy, fabsf(pt->co[0]) * ps);
          max_xy = math::max(max_xy, fabsf(pt->co[1]) * ps);
        }
        local_extent = float3(
            max_xy + bevel + pad, max_xy + bevel + pad, sz.z + bevel + pad);
        break;
      }
      default:
        local_extent = sz + float3(bevel + pad);
        break;
    }

    /* Expand for domain modifiers (read from packed GPU modifiers) */
    for (int mi = gpu_obj.modifier_start;
         mi < gpu_obj.modifier_start + gpu_obj.modifier_count; mi++)
    {
      const SDFModifierGPU &mod = modifiers_[mi];
      int mtype = mod.header.x;
      int mflags = mod.header.y;
      switch (mtype) {
        case SDF_MOD_MIRROR: {
          float offset = fabsf(mod.params.x);
          float3 local_org = float3(mod.params.y, mod.params.z, mod.params.w);
          float4x4 inv_rot = gpu_obj.inverse_matrix;
          if (mflags & SDF_MOD_MIRROR_X) {
            float3 N = float3(inv_rot[0][0], inv_rot[0][1], inv_rot[0][2]);
            float ea = math::dot(local_extent, math::abs(N));
            float disp = fmax(2.0f * fabsf(math::dot(local_org, N)) + offset, ea);
            local_extent += math::abs(N) * disp;
          }
          if (mflags & SDF_MOD_MIRROR_Y) {
            float3 N = float3(inv_rot[1][0], inv_rot[1][1], inv_rot[1][2]);
            float ea = math::dot(local_extent, math::abs(N));
            float disp = fmax(2.0f * fabsf(math::dot(local_org, N)) + offset, ea);
            local_extent += math::abs(N) * disp;
          }
          if (mflags & SDF_MOD_MIRROR_Z) {
            float3 N = float3(inv_rot[2][0], inv_rot[2][1], inv_rot[2][2]);
            float ea = math::dot(local_extent, math::abs(N));
            float disp = fmax(2.0f * fabsf(math::dot(local_org, N)) + offset, ea);
            local_extent += math::abs(N) * disp;
          }
          break;
        }
        case SDF_MOD_ELONGATE:
          local_extent += float3(mod.params.x, mod.params.y, mod.params.z);
          break;
        case SDF_MOD_SOLIDIFY:
        case SDF_MOD_ONION:
          local_extent += float3(mod.params.x);
          break;
        case SDF_MOD_ROUND:
        case SDF_MOD_BEVEL:
          local_extent = math::max(local_extent + float3(mod.params.x), float3(0.0f));
          break;
        case SDF_MOD_TWIST: {
          float xy = math::sqrt(local_extent.x * local_extent.x +
                                local_extent.y * local_extent.y);
          local_extent.x = xy;
          local_extent.y = xy;
          break;
        }
        case SDF_MOD_BEND: {
          float k = mod.params.x;
          int axis = int(mod.params.y);
          float3 origin = float3(mod.params.z, mod.params.w, mod.params2.x);
          if (fabsf(k) > 0.0001f) {
            float3 new_ext = float3(0.0f);
            for (int c = 0; c < 8; c++) {
              float3 pt = float3((c & 1) ? local_extent.x : -local_extent.x,
                                 (c & 2) ? local_extent.y : -local_extent.y,
                                 (c & 4) ? local_extent.z : -local_extent.z);
              pt -= origin;
              float drive, curve, free_val;
              if (axis == 1) { drive = pt.y; curve = pt.z; free_val = pt.x; }
              else if (axis == 2) { drive = pt.z; curve = pt.x; free_val = pt.y; }
              else { drive = pt.x; curve = pt.y; free_val = pt.z; }
              float a = k * drive;
              float nd = cosf(a) * drive - sinf(a) * curve;
              float nc = sinf(a) * drive + cosf(a) * curve;
              float3 bent;
              if (axis == 1) { bent = float3(free_val, nd, nc); }
              else if (axis == 2) { bent = float3(nc, free_val, nd); }
              else { bent = float3(nd, nc, free_val); }
              bent += origin;
              new_ext = math::max(new_ext, math::abs(bent));
            }
            local_extent = new_ext;
          }
          break;
        }
        case SDF_MOD_ARRAY: {
          float count = mod.params.x;
          if (count > 0.5f) {
            if (mflags == SDF_MOD_ARRAY_LINEAR) {
              float3 arr_offset = float3(mod.params.y, mod.params.z, mod.params.w);
              local_extent += math::abs(arr_offset) * (count - 1.0f);
            }
            else if (mflags == SDF_MOD_ARRAY_RADIAL) {
              float radius = mod.params.y;
              local_extent.x += radius;
              local_extent.y += radius;
              /* Spherical AABB when rotation offset or Y bias is active */
              float r = math::length(local_extent);
              local_extent = float3(r);
            }
          }
          break;
        }
        default:
          break;
      }
    }

    /* Transform local AABB to world */
    float3 world_min = float3(1e30f);
    float3 world_max = float3(-1e30f);
    for (int corner = 0; corner < 8; corner++) {
      float3 local_corner = float3((corner & 1) ? local_extent.x : -local_extent.x,
                                   (corner & 2) ? local_extent.y : -local_extent.y,
                                   (corner & 4) ? local_extent.z : -local_extent.z);
      float3 world_corner = float3(rot_mat * float4(local_corner, 0.0f)) +
                            float3(mat[3].x, mat[3].y, mat[3].z);
      world_min = math::min(world_min, world_corner);
      world_max = math::max(world_max, world_corner);
    }

    gpu_obj.bbox_min = float4(world_min, 0.0f);
    gpu_obj.bbox_max = float4(world_max, 0.0f);
    gpu_obj.orig_bbox_min = gpu_obj.bbox_min;
    gpu_obj.orig_bbox_max = gpu_obj.bbox_max;

    /* Early frustum cull — skip objects entirely outside the viewport */
    if (use_frustum_cull_) {
      if (!frustum_valid_) {
        const View &view = View::default_get();
        float4x4 vp = view.winmat() * view.viewmat();
        for (int i = 0; i < 4; i++) {
          frustum_planes_[0][i] = vp[i][3] + vp[i][0];
          frustum_planes_[1][i] = vp[i][3] - vp[i][0];
          frustum_planes_[2][i] = vp[i][3] + vp[i][1];
          frustum_planes_[3][i] = vp[i][3] - vp[i][1];
          frustum_planes_[4][i] = vp[i][3] + vp[i][2];
          frustum_planes_[5][i] = vp[i][3] - vp[i][2];
        }
        for (int p = 0; p < 6; p++) {
          frustum_planes_[p] /= math::length(float3(frustum_planes_[p]));
        }
        frustum_valid_ = true;
      }

      float group_pad = 0.0f;
      if (sdf_data->sdf_group != nullptr) {
        const SDFGroup *grp = sdf_data->sdf_group;
        float gb = (grp->csg_operation == SDF_CSG_SHELL)
                       ? std::max(grp->shell_blend_top, grp->shell_blend_bottom)
                       : grp->blend;
        group_pad = gb + fabsf(grp->shell_distance);
      }
      float3 pad_min = world_min - float3(group_pad);
      float3 pad_max = world_max + float3(group_pad);
      bool outside = false;
      for (int p = 0; p < 6; p++) {
        float3 n(frustum_planes_[p]);
        float d = frustum_planes_[p].w;
        float3 pos_vertex(n.x > 0 ? pad_max.x : pad_min.x,
                          n.y > 0 ? pad_max.y : pad_min.y,
                          n.z > 0 ? pad_max.z : pad_min.z);
        if (math::dot(n, pos_vertex) + d < 0.0f) {
          outside = true;
          break;
        }
      }
      if (outside) {
        return;
      }
    }

    float sphere_radius = math::length(local_extent);
    float3 obj_center = float3(mat[3].x, mat[3].y, mat[3].z);
    scene_min_ = math::min(scene_min_, obj_center - float3(sphere_radius));
    scene_max_ = math::max(scene_max_, obj_center + float3(sphere_radius));

    max_blend_ = math::max(max_blend_, gpu_obj.blend);
    if (gpu_obj.blend > 0.001f &&
        (gpu_obj.blend_type == SDF_BLEND_ROUND || gpu_obj.blend_type == SDF_BLEND_CHAMFER))
    {
      step_factor_ = min_ff(step_factor_, 0.65f);
    }
    if (sdf_data->csg_operation == SDF_CSG_SHELL) {
      max_shell_distance_ = math::max(max_shell_distance_, fabsf(sdf_data->shell_distance));
    }

    object_group_ptrs_.append(sdf_data->sdf_group);
    object_ptrs_.append(ob);
    objects_.append(gpu_obj);
  }

  void end_sync() final
  {
    if (depth_mode_) {
      return;
    }

    if (objects_.is_empty()) {
      return;
    }

    /* Build group GPU data */
    {
      Main *bmain = DEG_get_bmain(draw_ctx_->depsgraph);

      for (SDFGroup *group = static_cast<SDFGroup *>(bmain->sdf_groups.first); group; group = reinterpret_cast<SDFGroup *>(group->id.next)) {
        BKE_sdf_group_cleanup_null_members(group);
      }

      groups_gpu_.clear();
      Map<SDFGroup *, int> group_index_map;

      struct GroupMembership {
        int group_id;
        int group_order;
      };
      Map<Object *, GroupMembership> object_membership_map;

      int g_idx = 0;
      int obj_offset = 0;
      for (SDFGroup *group = static_cast<SDFGroup *>(bmain->sdf_groups.first); group; group = reinterpret_cast<SDFGroup *>(group->id.next)) {
        SDFGroupGPU gpu_grp = {};
        gpu_grp.csg_operation = group->csg_operation;
        gpu_grp.blend_type = group->blend_type;
        gpu_grp.blend = group->blend;
        gpu_grp.shell_distance = group->shell_distance;
        gpu_grp.shell_mode = group->shell_mode;
        gpu_grp.shell_op = group->shell_op;
        gpu_grp.shell_blend_top = group->shell_blend_top;
        gpu_grp.shell_blend_bottom = group->shell_blend_bottom;
        gpu_grp.chamfer_k2 = group->chamfer_k2;
        gpu_grp.chamfer_k3 = group->chamfer_k3;
        gpu_grp.first_object = obj_offset;
        gpu_grp.object_count = group->totmember;
        gpu_grp.color = float4(
            group->color[0], group->color[1], group->color[2], group->color[3]);
        float grp_eff_blend = (gpu_grp.csg_operation == SDF_CSG_SHELL)
                                  ? std::max(gpu_grp.shell_blend_top, gpu_grp.shell_blend_bottom)
                                  : gpu_grp.blend;
        if (grp_eff_blend > 0.001f &&
            (gpu_grp.blend_type == SDF_BLEND_ROUND || gpu_grp.blend_type == SDF_BLEND_CHAMFER))
        {
          step_factor_ = min_ff(step_factor_, 0.65f);
        }
        groups_gpu_.append(gpu_grp);
        group_index_map.add(group, g_idx);

        int member_order = 0;
        for (SDFGroupMember *member = static_cast<SDFGroupMember *>(group->members.first); member; member = member->next) {
          if (member->object) {
            object_membership_map.add_overwrite(member->object, {g_idx, member_order});
          }
          member_order++;
        }

        g_idx++;
        obj_offset += group->totmember;
      }

      BLI_assert(int(object_ptrs_.size()) == int(objects_.size()));
      Vector<int> group_orders(int(objects_.size()), 0);
      for (int i = 0; i < int(objects_.size()); i++) {
        Object *eval_ob = object_ptrs_[i];
        Object *orig_ob = DEG_get_original(eval_ob);
        const GroupMembership *membership = object_membership_map.lookup_ptr(orig_ob);
        if (membership) {
          objects_[i].group_id = membership->group_id;
          group_orders[i] = membership->group_order;
        }
      }

      /* Sort objects by (group_id, group_order) */
      {
        const int n = int(objects_.size());
        if (n > 0) {
          Vector<std::pair<int64_t, int>> sort_pairs(n);
          for (int i = 0; i < n; i++) {
            if (objects_[i].group_id >= 0) {
              sort_pairs[i] = {int64_t(objects_[i].group_id) * 1000000LL + group_orders[i], i};
            }
            else {
              sort_pairs[i] = {int64_t(10000000) + objects_[i].original_index, i};
            }
          }
          std::stable_sort(sort_pairs.begin(), sort_pairs.end());

          Vector<SDFObjectGPU> sorted(n);
          Vector<int> old_to_new(n);
          for (int new_idx = 0; new_idx < n; new_idx++) {
            int old_idx = sort_pairs[new_idx].second;
            sorted[new_idx] = objects_[old_idx];
            old_to_new[old_idx] = new_idx;
          }
          objects_ = std::move(sorted);
          s_depsgraph_to_sorted = std::move(old_to_new);
        }
      }

      /* Fix first_object/object_count per group */
      for (int gi = 0; gi < int(groups_gpu_.size()); gi++) {
        int first = -1;
        int count = 0;
        for (int i = 0; i < int(objects_.size()); i++) {
          if (objects_[i].group_id == gi) {
            if (first == -1) {
              first = i;
            }
            count++;
          }
        }
        groups_gpu_[gi].first_object = (first >= 0) ? first : 0;
        groups_gpu_[gi].object_count = count;
      }

      /* Compute max blend per group, store in each member's _pad1 */
      for (int gi = 0; gi < int(groups_gpu_.size()); gi++) {
        float grp_blend = (groups_gpu_[gi].csg_operation == SDF_CSG_SHELL)
                              ? std::max(groups_gpu_[gi].shell_blend_top,
                                         groups_gpu_[gi].shell_blend_bottom)
                              : fabsf(groups_gpu_[gi].blend);
        float max_blend = grp_blend;
        int start = groups_gpu_[gi].first_object;
        int cnt = groups_gpu_[gi].object_count;
        for (int m = start; m < start + cnt; m++) {
          float b = (objects_[m].blend_type == 0) ? 0.0f : objects_[m].blend;
          max_blend = std::max(max_blend, b + fabsf(objects_[m].shell_distance));
        }
        for (int m = start; m < start + cnt; m++) {
          objects_[m].max_group_blend = max_blend;
        }
      }
      /* Ungrouped objects: store own blend in _pad1 */
      for (int i = 0; i < int(objects_.size()); i++) {
        if (objects_[i].group_id < 0) {
          float b = (objects_[i].blend_type == 0) ? 0.0f : objects_[i].blend;
          b += fabsf(objects_[i].shell_distance);
          objects_[i].max_group_blend = b;
        }
      }
    }

    /* Early frustum cull on pre-expansion AABBs with conservative padding */
    {
      float max_expansion = max_blend_ + max_shell_distance_;
      for (int g = 0; g < int(groups_gpu_.size()); g++) {
        float gb = (groups_gpu_[g].csg_operation == SDF_CSG_SHELL)
                       ? std::max(groups_gpu_[g].shell_blend_top,
                                  groups_gpu_[g].shell_blend_bottom)
                       : groups_gpu_[g].blend;
        max_expansion = std::max(max_expansion, gb + fabsf(groups_gpu_[g].shell_distance));
      }

      const View &view = View::default_get();
      float4x4 vp = view.winmat() * view.viewmat();
      float4 planes[6];
      for (int i = 0; i < 4; i++) {
        planes[0][i] = vp[i][3] + vp[i][0];
        planes[1][i] = vp[i][3] - vp[i][0];
        planes[2][i] = vp[i][3] + vp[i][1];
        planes[3][i] = vp[i][3] - vp[i][1];
        planes[4][i] = vp[i][3] + vp[i][2];
        planes[5][i] = vp[i][3] - vp[i][2];
      }
      for (int p = 0; p < 6; p++) {
        planes[p] /= math::length(float3(planes[p]));
      }

      for (int i = 0; i < int(objects_.size()); i++) {
        float3 bmin = float3(objects_[i].bbox_min) - float3(max_expansion);
        float3 bmax = float3(objects_[i].bbox_max) + float3(max_expansion);
        bool visible = true;
        for (int p = 0; p < 6; p++) {
          float3 n(planes[p]);
          float d = planes[p].w;
          float3 pos_vertex(n.x > 0 ? bmax.x : bmin.x,
                            n.y > 0 ? bmax.y : bmin.y,
                            n.z > 0 ? bmax.z : bmin.z);
          if (math::dot(n, pos_vertex) + d < 0.0f) {
            visible = false;
            break;
          }
        }
        objects_[i]._pad2 = visible ? 1 : 0;
      }

      /* Group propagation: if any member visible, keep entire group */
      for (int g = 0; g < int(groups_gpu_.size()); g++) {
        int start = groups_gpu_[g].first_object;
        int count = groups_gpu_[g].object_count;
        bool any_visible = false;
        for (int i = start; i < start + count; i++) {
          if (objects_[i]._pad2 != 0) {
            any_visible = true;
            break;
          }
        }
        if (any_visible) {
          for (int i = start; i < start + count; i++) {
            objects_[i]._pad2 = 1;
          }
        }
      }

      /* Compact visible objects */
      int visible_count = 0;
      for (int i = 0; i < int(objects_.size()); i++) {
        if (objects_[i]._pad2 != 0) {
          visible_count++;
        }
      }

      if (visible_count < int(objects_.size())) {
        Vector<SDFObjectGPU> compact_objects;
        Vector<SDFModifierGPU> compact_modifiers;
        Vector<SDFPolygonPointGPU> compact_polygon_points;
        Vector<int> obj_remap(objects_.size(), -1);
        compact_objects.reserve(visible_count);

        for (int i = 0; i < int(objects_.size()); i++) {
          if (objects_[i]._pad2 == 0) {
            continue;
          }
          obj_remap[i] = int(compact_objects.size());
          SDFObjectGPU obj = objects_[i];

          int new_mod_start = int(compact_modifiers.size());
          for (int m = 0; m < obj.modifier_count; m++) {
            compact_modifiers.append(modifiers_[obj.modifier_start + m]);
          }
          obj.modifier_start = new_mod_start;

          int new_poly_start = int(compact_polygon_points.size());
          for (int p = 0; p < obj.polygon_point_count; p++) {
            compact_polygon_points.append(polygon_points_[obj.polygon_point_start + p]);
          }
          obj.polygon_point_start = new_poly_start;

          compact_objects.append(obj);
        }

        Vector<SDFGroupGPU> compact_groups;
        Vector<int> group_remap(groups_gpu_.size(), -1);
        for (int g = 0; g < int(groups_gpu_.size()); g++) {
          int old_start = groups_gpu_[g].first_object;
          int old_count = groups_gpu_[g].object_count;
          int new_first = -1;
          int new_count = 0;
          for (int i = old_start; i < old_start + old_count; i++) {
            if (obj_remap[i] >= 0) {
              if (new_first < 0) {
                new_first = obj_remap[i];
              }
              new_count++;
            }
          }
          if (new_count > 0) {
            group_remap[g] = int(compact_groups.size());
            SDFGroupGPU grp = groups_gpu_[g];
            grp.first_object = new_first;
            grp.object_count = new_count;
            compact_groups.append(grp);
          }
        }

        for (auto &obj : compact_objects) {
          if (obj.group_id >= 0) {
            obj.group_id = group_remap[obj.group_id];
          }
        }

        for (int &idx : s_depsgraph_to_sorted) {
          idx = (idx >= 0 && idx < int(obj_remap.size())) ? obj_remap[idx] : -1;
        }

        objects_ = std::move(compact_objects);
        modifiers_ = std::move(compact_modifiers);
        polygon_points_ = std::move(compact_polygon_points);
        groups_gpu_ = std::move(compact_groups);
      }
    }

    /* Expand AABBs for blend-aware CSG (now runs on visible set only) */
    Vector<float3> new_mins(objects_.size());
    Vector<float3> new_maxs(objects_.size());
    for (int i = 0; i < int(objects_.size()); i++) {
      float blend_radius = (objects_[i].blend_type == 0) ? 0.0f : objects_[i].blend;
      float exp = blend_radius + fabsf(objects_[i].shell_distance);
      new_mins[i] = float3(objects_[i].bbox_min) - float3(exp);
      new_maxs[i] = float3(objects_[i].bbox_max) + float3(exp);
    }

    int ungrouped_start = int(objects_.size());
    for (int i = 0; i < int(objects_.size()); i++) {
      if (objects_[i].group_id == -1) {
        ungrouped_start = i;
        break;
      }
    }

    for (int i = int(objects_.size()) - 1; i >= ungrouped_start; i--) {
      SDFObjectGPU &obj = objects_[i];
      for (int j = i + 1; j < int(objects_.size()); j++) {
        float sub_blend_radius = (objects_[j].blend_type == 0) ? 0.0f : objects_[j].blend;
        float sub_blend = sub_blend_radius + fabsf(objects_[j].shell_distance);
        if (sub_blend > 0.0f) {
          float3 obj_exp_min = float3(obj.bbox_min) - float3(sub_blend);
          float3 obj_exp_max = float3(obj.bbox_max) + float3(sub_blend);
          float3 sub_exp_min = new_mins[j];
          float3 sub_exp_max = new_maxs[j];
          float3 ix_min = math::max(obj_exp_min, sub_exp_min);
          float3 ix_max = math::min(obj_exp_max, sub_exp_max);
          if (ix_min.x <= ix_max.x && ix_min.y <= ix_max.y && ix_min.z <= ix_max.z) {
            new_mins[i] = math::min(new_mins[i], ix_min);
            new_maxs[i] = math::max(new_maxs[i], ix_max);
          }
        }
      }
    }

    Vector<float3> group_mins(groups_gpu_.size(), float3(1e10f));
    Vector<float3> group_maxs(groups_gpu_.size(), float3(-1e10f));

    for (int g = int(groups_gpu_.size()) - 1; g >= 0; g--) {
      SDFGroupGPU &grp = groups_gpu_[g];
      int start_idx = grp.first_object;
      int end_idx = grp.first_object + grp.object_count - 1;

      if (grp.object_count > 0 && start_idx >= 0 && end_idx < int(objects_.size())) {
        for (int i = end_idx; i >= start_idx; i--) {
          SDFObjectGPU &obj = objects_[i];

          for (int j = i + 1; j <= end_idx; j++) {
            float sub_blend_radius = (objects_[j].blend_type == 0) ? 0.0f : objects_[j].blend;
            float sub_blend = sub_blend_radius + fabsf(objects_[j].shell_distance);
            if (sub_blend > 0.0f) {
              float3 obj_exp_min = float3(obj.bbox_min) - float3(sub_blend);
              float3 obj_exp_max = float3(obj.bbox_max) + float3(sub_blend);
              float3 sub_exp_min = new_mins[j];
              float3 sub_exp_max = new_maxs[j];
              float3 ix_min = math::max(obj_exp_min, sub_exp_min);
              float3 ix_max = math::min(obj_exp_max, sub_exp_max);
              if (ix_min.x <= ix_max.x && ix_min.y <= ix_max.y && ix_min.z <= ix_max.z) {
                new_mins[i] = math::min(new_mins[i], ix_min);
                new_maxs[i] = math::max(new_maxs[i], ix_max);
              }
            }
          }

          for (int sub_g = g + 1; sub_g < int(groups_gpu_.size()); sub_g++) {
            float sg_blend = (groups_gpu_[sub_g].csg_operation == SDF_CSG_SHELL)
                                 ? std::max(groups_gpu_[sub_g].shell_blend_top,
                                            groups_gpu_[sub_g].shell_blend_bottom)
                                 : groups_gpu_[sub_g].blend;
            float sub_blend = sg_blend + fabsf(groups_gpu_[sub_g].shell_distance);
            if (sub_blend > 0.0f) {
              float3 obj_exp_min = float3(obj.bbox_min) - float3(sub_blend);
              float3 obj_exp_max = float3(obj.bbox_max) + float3(sub_blend);
              float3 sub_exp_min = group_mins[sub_g];
              float3 sub_exp_max = group_maxs[sub_g];
              float3 ix_min = math::max(obj_exp_min, sub_exp_min);
              float3 ix_max = math::min(obj_exp_max, sub_exp_max);
              if (ix_min.x <= ix_max.x && ix_min.y <= ix_max.y && ix_min.z <= ix_max.z) {
                new_mins[i] = math::min(new_mins[i], ix_min);
                new_maxs[i] = math::max(new_maxs[i], ix_max);
              }
            }
          }

          for (int j = ungrouped_start; j < int(objects_.size()); j++) {
            float sub_blend_radius = (objects_[j].blend_type == 0) ? 0.0f : objects_[j].blend;
            float sub_blend = sub_blend_radius + fabsf(objects_[j].shell_distance);
            if (sub_blend > 0.0f) {
              float3 obj_exp_min = float3(obj.bbox_min) - float3(sub_blend);
              float3 obj_exp_max = float3(obj.bbox_max) + float3(sub_blend);
              float3 sub_exp_min = new_mins[j];
              float3 sub_exp_max = new_maxs[j];
              float3 ix_min = math::max(obj_exp_min, sub_exp_min);
              float3 ix_max = math::min(obj_exp_max, sub_exp_max);
              if (ix_min.x <= ix_max.x && ix_min.y <= ix_max.y && ix_min.z <= ix_max.z) {
                new_mins[i] = math::min(new_mins[i], ix_min);
                new_maxs[i] = math::max(new_maxs[i], ix_max);
              }
            }
          }

          group_mins[g] = math::min(group_mins[g], new_mins[i]);
          group_maxs[g] = math::max(group_maxs[g], new_maxs[i]);
        }
      }
    }

    for (int i = 0; i < int(objects_.size()); i++) {
      objects_[i].bbox_min = float4(new_mins[i], 0.0f);
      objects_[i].bbox_max = float4(new_maxs[i], 0.0f);
    }

    /* Fine frustum cull on expanded AABBs (before intersection expansion) */
    {
      const View &view = View::default_get();
      float4x4 vp = view.winmat() * view.viewmat();
      float4 planes[6];
      for (int i = 0; i < 4; i++) {
        planes[0][i] = vp[i][3] + vp[i][0];
        planes[1][i] = vp[i][3] - vp[i][0];
        planes[2][i] = vp[i][3] + vp[i][1];
        planes[3][i] = vp[i][3] - vp[i][1];
        planes[4][i] = vp[i][3] + vp[i][2];
        planes[5][i] = vp[i][3] - vp[i][2];
      }
      for (int p = 0; p < 6; p++) {
        planes[p] /= math::length(float3(planes[p]));
      }
      for (int i = 0; i < int(objects_.size()); i++) {
        float3 bmin = float3(objects_[i].bbox_min);
        float3 bmax = float3(objects_[i].bbox_max);
        bool visible = true;
        for (int p = 0; p < 6; p++) {
          float3 n(planes[p]);
          float d = planes[p].w;
          float3 pos_vertex(n.x > 0 ? bmax.x : bmin.x,
                            n.y > 0 ? bmax.y : bmin.y,
                            n.z > 0 ? bmax.z : bmin.z);
          if (math::dot(n, pos_vertex) + d < 0.0f) {
            visible = false;
            break;
          }
        }
        objects_[i]._pad2 = visible ? 1 : 0;
      }
    }

    /* Intersection/Push: expand to visible scene bounds.
     * Runs after frustum cull so only visible objects contribute. */
    {
      float3 smin = float3(1e30f), smax = float3(-1e30f);
      for (int i = 0; i < int(objects_.size()); i++) {
        if (objects_[i]._pad2 == 0) { continue; }
        smin = math::min(smin, float3(objects_[i].bbox_min));
        smax = math::max(smax, float3(objects_[i].bbox_max));
      }
      for (int i = 0; i < int(objects_.size()); i++) {
        if (objects_[i].csg_operation == SDF_CSG_INTERSECT ||
            objects_[i].csg_operation == SDF_CSG_PUSH)
        {
          objects_[i].bbox_min = float4(smin, 0.0f);
          objects_[i].bbox_max = float4(smax, 0.0f);
          objects_[i]._pad2 = 1;
        }
      }
      for (int g = 0; g < int(groups_gpu_.size()); g++) {
        int grp_op = groups_gpu_[g].csg_operation;
        if (grp_op == SDF_CSG_INTERSECT || grp_op == SDF_CSG_PUSH) {
          int start = groups_gpu_[g].first_object;
          int count = groups_gpu_[g].object_count;
          for (int i = start; i < start + count; i++) {
            objects_[i].bbox_min = float4(smin, 0.0f);
            objects_[i].bbox_max = float4(smax, 0.0f);
            objects_[i]._pad2 = 1;
          }
        }
      }
    }

    /* Recompute scene AABB and build BVH from visible objects */
    scene_min_ = float3(1e30f);
    scene_max_ = float3(-1e30f);
    bvh_tree_.clear();
    for (int i = 0; i < int(objects_.size()); i++) {
      scene_min_ = math::min(scene_min_, float3(objects_[i].bbox_min));
      scene_max_ = math::max(scene_max_, float3(objects_[i].bbox_max));

      SdfAabb bounds;
      bounds.min = float3(objects_[i].bbox_min);
      bounds.max = float3(objects_[i].bbox_max);
      bvh_tree_.create_proxy(bounds, i);
    }

    /* Hot AABB buffer (must be after compaction + AABB expansion) */
    {
      const int n = int(objects_.size());
      object_aabbs_.resize(n);
      for (int i = 0; i < n; i++) {
        SDFObjectAABB &a = object_aabbs_[i];
        a.bbox_min = objects_[i].bbox_min;
        a.bbox_max = objects_[i].bbox_max;
        a.group_id = objects_[i].group_id;
        a.max_group_blend = objects_[i].max_group_blend;
        a._pad0 = 0;
        a._pad1 = 0;
      }
    }

    needs_upload_ = true;

  }

  void draw(Manager & /*manager*/) final
  {
    if (objects_.is_empty()) {
      /* Clear stale output so previous frame's SDF doesn't persist */
      if (comp_color_tx_) {
        float clear[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        GPU_texture_clear(comp_color_tx_, GPU_DATA_FLOAT, clear);
      }
      if (comp_depth_tx_) {
        float clear[4] = {1.0f, 1.0f, 1.0f, 1.0f};
        GPU_texture_clear(comp_depth_tx_, GPU_DATA_FLOAT, clear);
      }
      compute_valid_ = false;
      return;
    }

    ensure_shaders();
    if (trace_comp_sh_ == nullptr) {
      return;
    }

    sync_shading();

    /* Detect shading changes (matcap, lighting, studiolight) */
    bool shading_changed = false;
    {
      uint64_t sh = 0xcbf29ce484222325ULL;
      auto hash_shading = [&](const void *data, size_t size) {
        const uint8_t *p = static_cast<const uint8_t *>(data);
        for (size_t i = 0; i < size; i++) {
          sh ^= p[i];
          sh *= 0x100000001b3ULL;
        }
      };
      hash_shading(&lighting_type_, sizeof(lighting_type_));
      hash_shading(&use_specular_, sizeof(use_specular_));
      hash_shading(&use_matcap_flip_, sizeof(use_matcap_flip_));
      hash_shading(studio_light_dir_, sizeof(studio_light_dir_));
      hash_shading(studio_light_col_, sizeof(studio_light_col_));
      hash_shading(studio_light_spec_, sizeof(studio_light_spec_));
      hash_shading(&studio_ambient_, sizeof(studio_ambient_));
      hash_shading(current_matcap_.data(), current_matcap_.size());
      hash_shading(&debug_bvh_views_, sizeof(debug_bvh_views_));
      hash_shading(&use_cone_trace_, sizeof(use_cone_trace_));
      hash_shading(&sdf_max_steps_, sizeof(sdf_max_steps_));
      hash_shading(&sdf_ray_epsilon_, sizeof(sdf_ray_epsilon_));
      hash_shading(&sdf_over_relaxation_, sizeof(sdf_over_relaxation_));
      hash_shading(&sdf_cone_aperture_, sizeof(sdf_cone_aperture_));
      hash_shading(&sdf_cone_steps_, sizeof(sdf_cone_steps_));
      hash_shading(&use_frustum_cull_, sizeof(use_frustum_cull_));
      hash_shading(&fxaa_enabled_, sizeof(fxaa_enabled_));
      shading_changed = (sh != prev_shading_hash_);
      prev_shading_hash_ = sh;
    }

    DRW_submission_start();

    /* Detect scene and view changes for adaptive resolution */
    {
      uint64_t h = 0xcbf29ce484222325ULL;
      auto hash_bytes = [&](const void *data, size_t size) {
        const uint8_t *p = static_cast<const uint8_t *>(data);
        for (size_t i = 0; i < size; i++) {
          h ^= p[i];
          h *= 0x100000001b3ULL;
        }
      };
      hash_bytes(objects_.data(), objects_.size() * sizeof(SDFObjectGPU));
      hash_bytes(modifiers_.data(), modifiers_.size() * sizeof(SDFModifierGPU));
      hash_bytes(groups_gpu_.data(), groups_gpu_.size() * sizeof(SDFGroupGPU));
      hash_bytes(
          polygon_points_.data(), polygon_points_.size() * sizeof(SDFPolygonPointGPU));
      scene_changed_ = (h != prev_data_hash_);
      prev_data_hash_ = h;

      uint64_t mh = 0xcbf29ce484222325ULL;
      auto hash_mesh = [&](const void *data, size_t size) {
        const uint8_t *p = static_cast<const uint8_t *>(data);
        for (size_t i = 0; i < size; i++) {
          mh ^= p[i];
          mh *= 0x100000001b3ULL;
        }
      };
      hash_mesh(mesh_transforms_.data(), mesh_transforms_.size() * sizeof(float4x4));
      mesh_changed_ = (mh != prev_mesh_hash_);
      bool mesh_changed = mesh_changed_;
      prev_mesh_hash_ = mh;

      const View &view = View::default_get();
      const float4x4 &cur_viewmat = view.viewmat();
      const float4x4 &cur_winmat = view.winmat();
      {
        const float eps = 1e-6f;
        bool vdiff = false;
        for (int c = 0; c < 4 && !vdiff; c++) {
          for (int r = 0; r < 4 && !vdiff; r++) {
            if (math::abs(cur_viewmat[c][r] - prev_viewmat_[c][r]) > eps ||
                math::abs(cur_winmat[c][r] - prev_winmat_[c][r]) > eps)
            {
              vdiff = true;
            }
          }
        }
        view_changed_ = vdiff;
      }
      prev_viewmat_ = cur_viewmat;
      prev_winmat_ = cur_winmat;

      if (scene_changed_ || view_changed_ || mesh_changed || shading_changed) {
        scroll_cooldown_ = 3;
        idle_frames_ = 0;
      }
      else if (scroll_cooldown_ > 0) {
        scroll_cooldown_--;
        idle_frames_ = 0;
      }
      else {
        idle_frames_++;
      }
    }

    if (needs_upload_ && (scene_changed_ || mesh_changed_ || !compute_valid_)) {
      GPU_debug_group_begin("SDF Upload");
      upload_objects();
      GPU_debug_group_end();
      needs_upload_ = false;
    }

    ensure_compute_targets();

    bool res_changed = (render_size_ != prev_render_size_);
    bool force_compute = (G.debug & G_DEBUG_GPU_SDF) != 0;
    bool need_compute = force_compute || !compute_valid_ || scene_changed_ || view_changed_ ||
                         res_changed || shading_changed;

    if (need_compute) {
      GPU_debug_group_begin("SDF AABB Project");
      draw_aabb_project();
      GPU_debug_group_end();

      GPU_memory_barrier(GPU_BARRIER_SHADER_STORAGE);

      GPU_debug_group_begin("SDF Tile Cull");
      draw_tile_cull();
      GPU_debug_group_end();

      GPU_memory_barrier(GPU_BARRIER_SHADER_STORAGE);

      GPU_debug_group_begin("SDF Cone March");
      draw_cone_march();
      GPU_debug_group_end();

      GPU_memory_barrier(GPU_BARRIER_SHADER_STORAGE);

      GPU_debug_group_begin("SDF Trace");
      draw_trace();
      GPU_debug_group_end();

      GPU_memory_barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS);

      GPU_debug_group_begin("SDF Color Resolve");
      draw_color_resolve();
      GPU_debug_group_end();

      GPU_memory_barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS);

      GPU_debug_group_begin("SDF Normal");
      draw_normal();
      GPU_debug_group_end();

      GPU_memory_barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS);

      GPU_debug_group_begin("SDF Shade");
      draw_shade();
      GPU_debug_group_end();

      GPU_memory_barrier(GPU_BARRIER_TEXTURE_FETCH);

      compute_valid_ = true;
    }

    GPU_debug_group_begin("SDF Blit");
    draw_blit();
    GPU_debug_group_end();

    GPU_debug_group_begin("SDF FXAA");
    draw_fxaa();
    GPU_debug_group_end();

    prev_render_size_ = render_size_;

    DRW_submission_end();

    if ((G.debug & G_DEBUG_GPU_SDF) ||
        (adaptive_resolution_ && render_size_ != texture_size_))
    {
      DRW_viewport_request_redraw();
    }

    /* Update static globals for overlay */
    s_object_count = int(objects_.size());
    s_object_ssbo = object_ssbo_;
    s_modifier_ssbo = modifier_ssbo_;
    s_polygon_ssbo = polygon_ssbo_;
    s_group_ssbo = group_ssbo_;
    s_bvh_ssbo = bvh_nodes_ssbo_;
    s_bvh_root = bvh_tree_.root();
    s_depth_tx = comp_depth_tx_;
    s_gbuf_color_tx = gbuf_color_tx_;
    s_render_size = render_size_;
    s_texture_size = texture_size_;
    s_objects_cpu = objects_.data();
    s_objects_cpu_count = int(objects_.size());
    s_polygon_pts_cpu = polygon_points_.data();
    s_polygon_pts_count = int(polygon_points_.size());
    s_modifiers_cpu = modifiers_.data();
    s_modifier_count = int(modifiers_.size());
    s_group_count = int(groups_gpu_.size());
  }

 private:
  void sync_sdf_settings()
  {
    const View3D *v3d = draw_ctx_->v3d;
    if (v3d == nullptr) {
      return;
    }

    const View3DShading &s = v3d->shading;

    use_bvh_ = 1;
    /* Values 1-5 are GPU debug views, 6+ are overlay-only. */
    debug_bvh_views_ = (s.sdf_bvh_debug_view <= 5) ? s.sdf_bvh_debug_view : 0;
    debug_fd_normals_ = s.sdf_fd_normals ? 1 : 0;
    use_cone_trace_ = s.sdf_use_cone_trace ? 1 : 0;
    sdf_max_steps_ = s.sdf_max_steps > 0 ? s.sdf_max_steps : 512;
    sdf_ray_epsilon_ = s.sdf_ray_epsilon > 0.0f ? s.sdf_ray_epsilon : 0.0001f;
    sdf_over_relaxation_ = s.sdf_over_relaxation >= 1.0f ? s.sdf_over_relaxation : 1.3f;
    sdf_cone_aperture_ = s.sdf_cone_aperture > 0.0f ? s.sdf_cone_aperture : 0.5f;
    sdf_cone_steps_ = s.sdf_cone_steps > 0 ? s.sdf_cone_steps : 64;

    float scale_pct = s.sdf_resolution_scale;
    resolution_scale_ = (scale_pct >= 20.0f) ? scale_pct / 100.0f : 1.0f;
    adaptive_resolution_ = s.sdf_adaptive_resolution != 0;
    use_frustum_cull_ = s.sdf_frustum_cull != 0;

    bool new_fxaa = (U.sdf_fxaa != 0);
    if (!new_fxaa && fxaa_enabled_) {
      if (march_color_tx_) {
        GPU_texture_free(march_color_tx_);
        march_color_tx_ = nullptr;
      }
      if (march_fb_) {
        GPU_framebuffer_free(march_fb_);
        march_fb_ = nullptr;
      }
      fxaa_size_ = int2(0);
    }
    fxaa_enabled_ = new_fxaa;
  }

  void sync_shading()
  {
    const View3D *v3d = draw_ctx_->v3d;
    if (v3d == nullptr) {
      lighting_type_ = V3D_LIGHTING_FLAT;
      use_specular_ = 0;
      use_matcap_flip_ = 0;
      return;
    }

    const View3DShading &shading = v3d->shading;
    lighting_type_ = int(shading.light);
    use_specular_ = 0;

    if (lighting_type_ == V3D_LIGHTING_FLAT) {
      return;
    }

    if (lighting_type_ == V3D_LIGHTING_STUDIO) {
      StudioLight *sl = BKE_studiolight_find(shading.studio_light, STUDIOLIGHT_TYPE_STUDIO);
      if (sl == nullptr) {
        sl = BKE_studiolight_find_default(STUDIOLIGHT_TYPE_STUDIO);
      }

      float4x4 world_shading_rotation = float4x4::identity();
      if (shading.flag & V3D_SHADING_WORLD_ORIENTATION) {
        float4x4 V = blender::draw::View::default_get().viewmat();
        float R[4][4];
        axis_angle_to_mat4_single(R, 'Z', -shading.studiolight_rot_z);
        mul_m4_m4m4(R, V.ptr(), R);
        swap_v3_v3(R[2], R[1]);
        negate_v3(R[2]);
        world_shading_rotation = float4x4(R);
      }

      if (sl != nullptr) {
        use_specular_ = ((shading.flag & V3D_SHADING_SPECULAR_HIGHLIGHT) &&
                         (sl->flag & STUDIOLIGHT_SPECULAR_HIGHLIGHT_PASS)) ?
                            1 :
                            0;
        for (int i = 0; i < 4; i++) {
          const SolidLight &light = sl->light[i];
          if (light.flag) {
            float3 dir = math::transform_direction(world_shading_rotation,
                                                   float3(light.vec));
            studio_light_dir_[i] = float4(dir, 0.0f);
            studio_light_col_[i] = float4(light.col[0], light.col[1], light.col[2], light.smooth);
            studio_light_spec_[i] = float4(light.spec[0], light.spec[1], light.spec[2], 0.0f);
          }
          else {
            studio_light_dir_[i] = float4(0.0f);
            studio_light_col_[i] = float4(0.0f);
            studio_light_spec_[i] = float4(0.0f);
          }
        }
        studio_ambient_ = float3(sl->light_ambient[0], sl->light_ambient[1], sl->light_ambient[2]);
      }
      else {
        float3 fallback_dir = math::transform_direction(
            world_shading_rotation, math::normalize(float3(0.5f, 0.7f, 1.0f)));
        studio_light_dir_[0] = float4(fallback_dir, 0.0f);
        studio_light_col_[0] = float4(0.8f, 0.8f, 0.8f, 0.0f);
        studio_light_spec_[0] = float4(1.0f, 1.0f, 1.0f, 0.0f);
        for (int i = 1; i < 4; i++) {
          studio_light_dir_[i] = float4(0.0f);
          studio_light_col_[i] = float4(0.0f);
          studio_light_spec_[i] = float4(0.0f);
        }
        studio_ambient_ = float3(0.15f);
      }
    }
    else {
      StudioLight *sl = BKE_studiolight_find(shading.matcap, STUDIOLIGHT_TYPE_MATCAP);
      if (sl == nullptr) {
        sl = BKE_studiolight_find_default(STUDIOLIGHT_TYPE_MATCAP);
      }
      if (sl == nullptr) {
        lighting_type_ = V3D_LIGHTING_FLAT;
        return;
      }

      use_specular_ = ((shading.flag & V3D_SHADING_SPECULAR_HIGHLIGHT) &&
                       (sl->flag & STUDIOLIGHT_SPECULAR_HIGHLIGHT_PASS)) ?
                          1 :
                          0;
      use_matcap_flip_ = (shading.flag & V3D_SHADING_MATCAP_FLIP_X) ? 1 : 0;

      if (std::string(sl->name) != current_matcap_) {
        BKE_studiolight_ensure_flag(
            sl, STUDIOLIGHT_MATCAP_DIFFUSE_GPUTEXTURE | STUDIOLIGHT_MATCAP_SPECULAR_GPUTEXTURE);

        ImBuf *diffuse_ibuf = sl->matcap_diffuse.ibuf;
        ImBuf *specular_ibuf = sl->matcap_specular.ibuf;

        if (diffuse_ibuf && diffuse_ibuf->float_buffer.data) {
          int w = diffuse_ibuf->x;
          int h = diffuse_ibuf->y;
          int pixel_count = w * h * 4;
          int layers = 1;
          float *buffer = diffuse_ibuf->float_buffer.data;
          Vector<float> combined;

          if (specular_ibuf && specular_ibuf->float_buffer.data) {
            combined.extend(diffuse_ibuf->float_buffer.data, pixel_count);
            combined.extend(specular_ibuf->float_buffer.data, pixel_count);
            buffer = combined.begin();
            layers = 2;
          }

          if (matcap_tx_) {
            GPU_texture_free(matcap_tx_);
          }

          matcap_tx_ = GPU_texture_create_2d_array("sdf_matcap",
                                                   w,
                                                   h,
                                                   layers,
                                                   1,
                                                   gpu::TextureFormat::SFLOAT_16_16_16_16,
                                                   GPU_TEXTURE_USAGE_SHADER_READ,
                                                   buffer);
          if (matcap_tx_) {
            GPU_texture_filter_mode(matcap_tx_, true);
          }
        }

        current_matcap_ = sl->name;
      }

      if (matcap_tx_ == nullptr) {
        float white[4] = {1.0f, 1.0f, 1.0f, 1.0f};
        matcap_tx_ = GPU_texture_create_2d_array("sdf_matcap_fallback",
                                                 1,
                                                 1,
                                                 1,
                                                 1,
                                                 gpu::TextureFormat::SFLOAT_16_16_16_16,
                                                 GPU_TEXTURE_USAGE_SHADER_READ,
                                                 white);
      }
    }
  }

  void ensure_shaders()
  {
    sdf_shaders_ensure();
  }

  void upload_objects()
  {
    const int count = int(objects_.size());
    const size_t buf_size = count * sizeof(SDFObjectGPU);

    if (object_ssbo_ != nullptr && object_ssbo_count_ != count) {
      GPU_storagebuf_free(object_ssbo_);
      object_ssbo_ = nullptr;
    }

    if (object_ssbo_ == nullptr) {
      object_ssbo_ = GPU_storagebuf_create_ex(
          buf_size, objects_.data(), GPU_USAGE_DYNAMIC, "sdf_objects_ssbo");
      object_ssbo_count_ = count;
    }
    else {
      GPU_storagebuf_update(object_ssbo_, objects_.data());
    }

    /* AABB hot buffer */
    const size_t aabb_buf_size = count * sizeof(SDFObjectAABB);
    if (object_aabb_ssbo_ != nullptr && object_aabb_ssbo_count_ != count) {
      GPU_storagebuf_free(object_aabb_ssbo_);
      object_aabb_ssbo_ = nullptr;
    }
    if (object_aabb_ssbo_ == nullptr) {
      object_aabb_ssbo_ = GPU_storagebuf_create_ex(
          aabb_buf_size, object_aabbs_.data(), GPU_USAGE_DYNAMIC, "sdf_object_aabbs_ssbo");
      object_aabb_ssbo_count_ = count;
    }
    else {
      GPU_storagebuf_update(object_aabb_ssbo_, object_aabbs_.data());
    }

    const int mod_count = math::max(int(modifiers_.size()), 1);
    const size_t mod_buf_size = mod_count * sizeof(SDFModifierGPU);

    if (modifier_ssbo_ != nullptr && modifier_ssbo_count_ != mod_count) {
      GPU_storagebuf_free(modifier_ssbo_);
      modifier_ssbo_ = nullptr;
    }

    if (modifier_ssbo_ == nullptr) {
      if (modifiers_.is_empty()) {
        SDFModifierGPU dummy = {};
        modifier_ssbo_ = GPU_storagebuf_create_ex(
            mod_buf_size, &dummy, GPU_USAGE_DYNAMIC, "sdf_modifiers_ssbo");
      }
      else {
        modifier_ssbo_ = GPU_storagebuf_create_ex(
            mod_buf_size, modifiers_.data(), GPU_USAGE_DYNAMIC, "sdf_modifiers_ssbo");
      }
      modifier_ssbo_count_ = mod_count;
    }
    else {
      if (!modifiers_.is_empty()) {
        GPU_storagebuf_update(modifier_ssbo_, modifiers_.data());
      }
    }

    /* Polygon points SSBO */
    const int poly_count = math::max(int(polygon_points_.size()), 1);
    const size_t poly_buf_size = poly_count * sizeof(SDFPolygonPointGPU);

    if (polygon_ssbo_ != nullptr && polygon_ssbo_count_ != poly_count) {
      GPU_storagebuf_free(polygon_ssbo_);
      polygon_ssbo_ = nullptr;
    }

    if (polygon_ssbo_ == nullptr) {
      if (polygon_points_.is_empty()) {
        SDFPolygonPointGPU dummy = {};
        polygon_ssbo_ = GPU_storagebuf_create_ex(
            poly_buf_size, &dummy, GPU_USAGE_DYNAMIC, "sdf_polygon_points_ssbo");
      }
      else {
        polygon_ssbo_ = GPU_storagebuf_create_ex(
            poly_buf_size, polygon_points_.data(), GPU_USAGE_DYNAMIC, "sdf_polygon_points_ssbo");
      }
      polygon_ssbo_count_ = poly_count;
    }
    else {
      if (!polygon_points_.is_empty()) {
        GPU_storagebuf_update(polygon_ssbo_, polygon_points_.data());
      }
    }

    const int grp_count = math::max(int(groups_gpu_.size()), 1);
    const size_t grp_buf_size = grp_count * sizeof(SDFGroupGPU);

    if (group_ssbo_ != nullptr && group_ssbo_count_ != grp_count) {
      GPU_storagebuf_free(group_ssbo_);
      group_ssbo_ = nullptr;
    }

    if (group_ssbo_ == nullptr) {
      if (groups_gpu_.is_empty()) {
        SDFGroupGPU dummy = {};
        group_ssbo_ = GPU_storagebuf_create_ex(
            grp_buf_size, &dummy, GPU_USAGE_DYNAMIC, "sdf_groups_ssbo");
      }
      else {
        group_ssbo_ = GPU_storagebuf_create_ex(
            grp_buf_size, groups_gpu_.data(), GPU_USAGE_DYNAMIC, "sdf_groups_ssbo");
      }
      group_ssbo_count_ = grp_count;
    }
    else {
      if (!groups_gpu_.is_empty()) {
        GPU_storagebuf_update(group_ssbo_, groups_gpu_.data());
      }
    }

    Vector<SdfAabbNodeGPU> gpu_nodes = bvh_tree_.build_gpu_nodes(SdfAabbTree::fat_bounds_radius);
    const int bvh_count = math::max(int(gpu_nodes.size()), 1);
    const size_t bvh_buf_size = bvh_count * sizeof(SdfAabbNodeGPU);

    if (bvh_nodes_ssbo_ != nullptr && bvh_nodes_ssbo_count_ != bvh_count) {
      GPU_storagebuf_free(bvh_nodes_ssbo_);
      bvh_nodes_ssbo_ = nullptr;
    }

    if (bvh_nodes_ssbo_ == nullptr) {
      if (gpu_nodes.is_empty()) {
        SdfAabbNodeGPU dummy = {};
        bvh_nodes_ssbo_ = GPU_storagebuf_create_ex(
            bvh_buf_size, &dummy, GPU_USAGE_DYNAMIC, "sdf_bvh_ssbo");
      }
      else {
        bvh_nodes_ssbo_ = GPU_storagebuf_create_ex(
            bvh_buf_size, gpu_nodes.data(), GPU_USAGE_DYNAMIC, "sdf_bvh_ssbo");
      }
      bvh_nodes_ssbo_count_ = bvh_count;
    }
    else {
      if (!gpu_nodes.is_empty()) {
        GPU_storagebuf_update(bvh_nodes_ssbo_, gpu_nodes.data());
      }
    }
  }

  void ensure_compute_targets()
  {
    const int2 vp = int2(draw_ctx_->viewport_size_get());
    viewport_size_ = vp;

    /* Stay at low res until idle for 2+ frames (avoids GPU stall on re-grab) */
    float scale = resolution_scale_;
    bool is_interacting = scroll_cooldown_ > 0 || idle_frames_ < 2;
    if (adaptive_resolution_ && is_interacting) {
      scale *= 0.25f;
      scale = math::max(scale, 0.1f);
    }
    render_size_ = int2(math::max(int(vp.x * scale), 1),
                        math::max(int(vp.y * scale), 1));

    /* Textures sized to full viewport — only reallocate on viewport resize */
    int2 tex_size = int2(math::max(int(vp.x * resolution_scale_), 1),
                         math::max(int(vp.y * resolution_scale_), 1));
    if (comp_color_tx_ != nullptr && texture_size_ == tex_size) {
      return;
    }
    compute_valid_ = false;

    if (comp_color_tx_) {
      GPU_texture_free(comp_color_tx_);
    }
    if (comp_depth_tx_) {
      GPU_texture_free(comp_depth_tx_);
    }
    if (gbuf_pos_tx_) {
      GPU_texture_free(gbuf_pos_tx_);
    }
    if (gbuf_color_tx_) {
      GPU_texture_free(gbuf_color_tx_);
    }
    if (gbuf_normal_tx_) {
      GPU_texture_free(gbuf_normal_tx_);
    }
    texture_size_ = tex_size;

    eGPUTextureUsage usage = GPU_TEXTURE_USAGE_SHADER_READ | GPU_TEXTURE_USAGE_SHADER_WRITE |
                             GPU_TEXTURE_USAGE_ATTACHMENT;
    eGPUTextureUsage usage_readback = usage | GPU_TEXTURE_USAGE_HOST_READ;
    comp_color_tx_ = GPU_texture_create_2d("sdf_comp_color", tex_size.x, tex_size.y, 1, gpu::TextureFormat::SFLOAT_16_16_16_16, usage, nullptr);
    comp_depth_tx_ = GPU_texture_create_2d("sdf_comp_depth", tex_size.x, tex_size.y, 1, gpu::TextureFormat::SFLOAT_32, usage_readback, nullptr);
    gbuf_pos_tx_ = GPU_texture_create_2d("sdf_gbuf_pos", tex_size.x, tex_size.y, 1, gpu::TextureFormat::SFLOAT_32_32_32_32, usage, nullptr);
    gbuf_color_tx_ = GPU_texture_create_2d("sdf_gbuf_color", tex_size.x, tex_size.y, 1, gpu::TextureFormat::SFLOAT_16_16_16_16, usage_readback, nullptr);
    gbuf_normal_tx_ = GPU_texture_create_2d("sdf_gbuf_normal", tex_size.x, tex_size.y, 1, gpu::TextureFormat::SFLOAT_16_16_16_16, usage, nullptr);
  }

  void bind_ssbos(gpu::Shader *sh)
  {
    if (object_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "objects");
      GPU_storagebuf_bind(object_ssbo_, slot);
    }
    if (modifier_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "sdf_modifiers");
      GPU_storagebuf_bind(modifier_ssbo_, slot);
    }
    if (polygon_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "polygon_points");
      if (slot >= 0) {
        GPU_storagebuf_bind(polygon_ssbo_, slot);
      }
    }
    if (group_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "groups");
      GPU_storagebuf_bind(group_ssbo_, slot);
    }
    if (bvh_nodes_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "aabb_nodes");
      if (slot >= 0) {
        GPU_storagebuf_bind(bvh_nodes_ssbo_, slot);
      }
    }
    if (object_aabb_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "object_aabbs");
      if (slot >= 0) {
        GPU_storagebuf_bind(object_aabb_ssbo_, slot);
      }
    }
  }

  static constexpr int kMaxTileObjects = 128;

  void ensure_screen_aabbs_ssbo(int obj_count)
  {
    if (screen_aabbs_ssbo_ != nullptr && screen_aabbs_ssbo_count_ != obj_count) {
      GPU_storagebuf_free(screen_aabbs_ssbo_);
      screen_aabbs_ssbo_ = nullptr;
    }
    if (screen_aabbs_ssbo_ == nullptr && obj_count > 0) {
      screen_aabbs_ssbo_ = GPU_storagebuf_create_ex(
          obj_count * sizeof(int[4]), nullptr, GPU_USAGE_DYNAMIC, "sdf_screen_aabbs");
      screen_aabbs_ssbo_count_ = obj_count;
    }
  }

  void ensure_tile_ssbos(int total_tiles)
  {
    /* Grow-only: allocate at peak tile count to avoid thrashing on resolution transitions */
    if (total_tiles <= tile_prim_counts_ssbo_tiles_) {
      return;
    }

    if (tile_prim_counts_ssbo_) {
      GPU_storagebuf_free(tile_prim_counts_ssbo_);
    }
    if (tile_prim_lists_ssbo_) {
      GPU_storagebuf_free(tile_prim_lists_ssbo_);
    }
    if (cone_hit_ssbo_) {
      GPU_storagebuf_free(cone_hit_ssbo_);
    }
    if (tile_far_hint_ssbo_) {
      GPU_storagebuf_free(tile_far_hint_ssbo_);
    }

    tile_prim_counts_ssbo_ = GPU_storagebuf_create_ex(
        total_tiles * sizeof(int), nullptr, GPU_USAGE_DYNAMIC, "sdf_tile_prim_counts");
    tile_prim_lists_ssbo_ = GPU_storagebuf_create_ex(
        total_tiles * kMaxTileObjects * sizeof(int),
        nullptr,
        GPU_USAGE_DYNAMIC,
        "sdf_tile_prim_lists");
    cone_hit_ssbo_ = GPU_storagebuf_create_ex(
        total_tiles * sizeof(float[4]), nullptr, GPU_USAGE_DYNAMIC, "sdf_cone_hit_ssbo");
    tile_far_hint_ssbo_ = GPU_storagebuf_create_ex(
        total_tiles * sizeof(float), nullptr, GPU_USAGE_DYNAMIC, "sdf_tile_far_hint");

    tile_prim_counts_ssbo_tiles_ = total_tiles;
  }

  void draw_aabb_project()
  {
    if (!aabb_project_sh_ || objects_.is_empty()) {
      return;
    }
    int obj_count = int(objects_.size());
    ensure_screen_aabbs_ssbo(obj_count);
    gpu::Shader *sh = aabb_project_sh_;
    GPU_shader_bind(sh);
    if (object_ssbo_) {
      GPU_storagebuf_bind(object_ssbo_, GPU_shader_get_ssbo_binding(sh, "objects"));
    }
    if (screen_aabbs_ssbo_) {
      GPU_storagebuf_bind(screen_aabbs_ssbo_,
                          GPU_shader_get_ssbo_binding(sh, "screen_aabbs"));
    }
    GPU_shader_uniform_1i(sh, "object_count", obj_count);
    GPU_shader_uniform_2iv(sh, "screen_size", &render_size_.x);
    View &view = View::default_get();
    view.matrices_ubo_get().push_update();
    GPU_uniformbuf_bind(view.matrices_ubo_get(), DRW_VIEW_UBO_SLOT);
    GPU_compute_dispatch(sh, (obj_count + 63) / 64, 1, 1);
    GPU_shader_unbind();
  }

  void draw_tile_cull()
  {
    if (!tile_cull_sh_ || use_bvh_ == 0) {
      return;
    }
    ensure_compute_targets();
    int tiles_x = (render_size_.x + 7) / 8;
    int tiles_y = (render_size_.y + 7) / 8;
    int total_tiles = tiles_x * tiles_y;
    ensure_tile_ssbos(total_tiles);

    gpu::Shader *sh = tile_cull_sh_;
    GPU_shader_bind(sh);
    if (object_ssbo_) {
      GPU_storagebuf_bind(object_ssbo_, GPU_shader_get_ssbo_binding(sh, "objects"));
    }
    if (screen_aabbs_ssbo_) {
      GPU_storagebuf_bind(screen_aabbs_ssbo_,
                          GPU_shader_get_ssbo_binding(sh, "screen_aabbs"));
    }
    if (tile_prim_counts_ssbo_) {
      GPU_storagebuf_bind(tile_prim_counts_ssbo_,
                          GPU_shader_get_ssbo_binding(sh, "tile_prim_counts"));
    }
    if (tile_prim_lists_ssbo_) {
      GPU_storagebuf_bind(tile_prim_lists_ssbo_,
                          GPU_shader_get_ssbo_binding(sh, "tile_prim_lists"));
    }
    GPU_shader_uniform_1i(sh, "object_count", int(objects_.size()));
    GPU_shader_uniform_2iv(sh, "screen_size", &render_size_.x);
    GPU_compute_dispatch(sh, tiles_x, tiles_y, 1);
    GPU_shader_unbind();
  }

  void draw_cone_march()
  {
    if (use_cone_trace_ == 0 || !cone_march_sh_) {
      return;
    }

    int tiles_x = (render_size_.x + 7) / 8;
    int tiles_y = (render_size_.y + 7) / 8;

    gpu::Shader *sh = cone_march_sh_;
    GPU_shader_bind(sh);

    if (object_ssbo_) {
      GPU_storagebuf_bind(object_ssbo_, GPU_shader_get_ssbo_binding(sh, "objects"));
    }
    if (modifier_ssbo_) {
      GPU_storagebuf_bind(modifier_ssbo_, GPU_shader_get_ssbo_binding(sh, "sdf_modifiers"));
    }
    if (polygon_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "polygon_points");
      if (slot >= 0) {
        GPU_storagebuf_bind(polygon_ssbo_, slot);
      }
    }
    if (group_ssbo_) {
      GPU_storagebuf_bind(group_ssbo_, GPU_shader_get_ssbo_binding(sh, "groups"));
    }
    if (object_aabb_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "object_aabbs");
      if (slot >= 0) {
        GPU_storagebuf_bind(object_aabb_ssbo_, slot);
      }
    }
    if (cone_hit_ssbo_) {
      GPU_storagebuf_bind(cone_hit_ssbo_, GPU_shader_get_ssbo_binding(sh, "tile_hit_pos"));
    }
    if (tile_prim_counts_ssbo_) {
      GPU_storagebuf_bind(tile_prim_counts_ssbo_,
                          GPU_shader_get_ssbo_binding(sh, "tile_prim_counts"));
    }
    if (tile_prim_lists_ssbo_) {
      GPU_storagebuf_bind(tile_prim_lists_ssbo_,
                          GPU_shader_get_ssbo_binding(sh, "tile_prim_lists"));
    }
    if (tile_far_hint_ssbo_) {
      GPU_storagebuf_bind(tile_far_hint_ssbo_,
                          GPU_shader_get_ssbo_binding(sh, "tile_far_hint"));
    }

    GPU_shader_uniform_1i(sh, "object_count", int(objects_.size()));
    GPU_shader_uniform_1i(sh, "group_count", int(groups_gpu_.size()));
    GPU_shader_uniform_1f(sh, "sdf_ray_epsilon", sdf_ray_epsilon_);
    GPU_shader_uniform_3fv(sh, "scene_aabb_min", scene_min_);
    GPU_shader_uniform_3fv(sh, "scene_aabb_max", scene_max_);
    GPU_shader_uniform_1f(sh, "sdf_cone_aperture", sdf_cone_aperture_);
    GPU_shader_uniform_1i(sh, "sdf_cone_steps", sdf_cone_steps_);
    GPU_shader_uniform_2iv(sh, "screen_size", &render_size_.x);

    View &view = View::default_get();
    view.matrices_ubo_get().push_update();
    GPU_uniformbuf_bind(view.matrices_ubo_get(), DRW_VIEW_UBO_SLOT);

    int dispatch_x = (tiles_x + 7) / 8;
    int dispatch_y = (tiles_y + 7) / 8;
    GPU_compute_dispatch(sh, dispatch_x, dispatch_y, 1);
    GPU_shader_unbind();
  }

  void draw_trace()
  {
    ensure_compute_targets();

    gpu::Shader *sh = (use_bvh_ != 0 && trace_tile_sh_) ? trace_tile_sh_ : trace_comp_sh_;
    if (!sh) {
      return;
    }

    GPU_shader_bind(sh);
    bind_ssbos(sh);

    /* Bind tile SSBOs (tile-culled path) */
    if (cone_hit_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "tile_hit_pos");
      if (slot >= 0) {
        GPU_storagebuf_bind(cone_hit_ssbo_, slot);
      }
    }
    if (tile_prim_counts_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "tile_prim_counts");
      if (slot >= 0) {
        GPU_storagebuf_bind(tile_prim_counts_ssbo_, slot);
      }
    }
    if (tile_prim_lists_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "tile_prim_lists");
      if (slot >= 0) {
        GPU_storagebuf_bind(tile_prim_lists_ssbo_, slot);
      }
    }
    if (tile_far_hint_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(sh, "tile_far_hint");
      if (slot >= 0) {
        GPU_storagebuf_bind(tile_far_hint_ssbo_, slot);
      }
    }

    /* Bind images: out_color/out_depth for debug views, G-buffer for hit data */
    GPU_texture_image_bind(comp_color_tx_, GPU_shader_get_sampler_binding(sh, "out_color_img"));
    GPU_texture_image_bind(comp_depth_tx_, GPU_shader_get_sampler_binding(sh, "out_depth_img"));
    GPU_texture_image_bind(gbuf_pos_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_pos_img"));
    GPU_texture_image_bind(gbuf_color_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_color_img"));

    GPU_shader_uniform_1i(sh, "object_count", int(objects_.size()));
    GPU_shader_uniform_1i(sh, "group_count", int(groups_gpu_.size()));
    GPU_shader_uniform_1i(sh, "use_bvh", use_bvh_);
    GPU_shader_uniform_1i(sh, "use_cone_trace", use_cone_trace_);
    GPU_shader_uniform_1i(sh, "debug_bvh_views", debug_bvh_views_);
    GPU_shader_uniform_1i(sh, "sdf_max_steps", sdf_max_steps_);
    GPU_shader_uniform_1f(sh, "sdf_ray_epsilon", sdf_ray_epsilon_);
    GPU_shader_uniform_1f(sh, "sdf_over_relaxation", sdf_over_relaxation_);
    GPU_shader_uniform_1f(sh, "sdf_step_factor", step_factor_);
    GPU_shader_uniform_1i(sh, "bvh_root", bvh_tree_.root());
    GPU_shader_uniform_2iv(sh, "screen_size", &render_size_.x);
    GPU_shader_uniform_3fv(sh, "scene_aabb_min", scene_min_);
    GPU_shader_uniform_3fv(sh, "scene_aabb_max", scene_max_);

    View &view = View::default_get();
    view.matrices_ubo_get().push_update();
    GPU_uniformbuf_bind(view.matrices_ubo_get(), DRW_VIEW_UBO_SLOT);

    int dispatch_x = (render_size_.x + 7) / 8;
    int dispatch_y = (render_size_.y + 7) / 8;
    GPU_compute_dispatch(sh, dispatch_x, dispatch_y, 1);

    GPU_texture_image_unbind(comp_color_tx_);
    GPU_texture_image_unbind(comp_depth_tx_);
    GPU_texture_image_unbind(gbuf_pos_tx_);
    GPU_texture_image_unbind(gbuf_color_tx_);
    GPU_shader_unbind();
  }

  void draw_color_resolve()
  {
    if (debug_bvh_views_ != 0) {
      return;
    }
    gpu::Shader *sh = color_resolve_sh_;
    if (!sh) {
      return;
    }
    GPU_shader_bind(sh);

    GPU_texture_image_bind(gbuf_pos_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_pos_img"));
    GPU_texture_image_bind(gbuf_color_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_color_img"));

    bind_ssbos(sh);

    if (tile_prim_counts_ssbo_) {
      GPU_storagebuf_bind(tile_prim_counts_ssbo_,
                          GPU_shader_get_ssbo_binding(sh, "tile_prim_counts"));
    }
    if (tile_prim_lists_ssbo_) {
      GPU_storagebuf_bind(tile_prim_lists_ssbo_,
                          GPU_shader_get_ssbo_binding(sh, "tile_prim_lists"));
    }

    GPU_shader_uniform_1i(sh, "object_count", int(objects_.size()));
    GPU_shader_uniform_1i(sh, "group_count", int(groups_gpu_.size()));
    GPU_shader_uniform_1f(sh, "sdf_ray_epsilon", sdf_ray_epsilon_);
    GPU_shader_uniform_2iv(sh, "screen_size", &render_size_.x);

    int dispatch_x = (render_size_.x + 7) / 8;
    int dispatch_y = (render_size_.y + 7) / 8;
    GPU_compute_dispatch(sh, dispatch_x, dispatch_y, 1);

    GPU_texture_image_unbind(gbuf_pos_tx_);
    GPU_texture_image_unbind(gbuf_color_tx_);
    GPU_shader_unbind();
  }

  void draw_normal()
  {
    if (debug_bvh_views_ != 0) {
      return;
    }
    gpu::Shader *sh = normal_comp_sh_;
    if (!sh) {
      return;
    }
    GPU_shader_bind(sh);

    GPU_texture_image_bind(gbuf_pos_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_pos_img"));
    GPU_texture_image_bind(gbuf_normal_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_normal_img"));
    GPU_texture_image_bind(gbuf_color_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_color_img"));

    bind_ssbos(sh);

    if (tile_prim_counts_ssbo_) {
      GPU_storagebuf_bind(tile_prim_counts_ssbo_,
                          GPU_shader_get_ssbo_binding(sh, "tile_prim_counts"));
    }
    if (tile_prim_lists_ssbo_) {
      GPU_storagebuf_bind(tile_prim_lists_ssbo_,
                          GPU_shader_get_ssbo_binding(sh, "tile_prim_lists"));
    }

    GPU_shader_uniform_1i(sh, "object_count", int(objects_.size()));
    GPU_shader_uniform_1i(sh, "group_count", int(groups_gpu_.size()));
    GPU_shader_uniform_1f(sh, "sdf_ray_epsilon", sdf_ray_epsilon_);
    GPU_shader_uniform_1i(sh, "debug_fd_normals", debug_fd_normals_);
    GPU_shader_uniform_2iv(sh, "screen_size", &render_size_.x);

    int dispatch_x = (render_size_.x + 7) / 8;
    int dispatch_y = (render_size_.y + 7) / 8;
    GPU_compute_dispatch(sh, dispatch_x, dispatch_y, 1);

    GPU_texture_image_unbind(gbuf_pos_tx_);
    GPU_texture_image_unbind(gbuf_normal_tx_);
    GPU_texture_image_unbind(gbuf_color_tx_);
    GPU_shader_unbind();
  }

  void draw_shade()
  {
    if (debug_bvh_views_ != 0) {
      return;
    }

    gpu::Shader *sh = shade_comp_sh_;
    if (!sh) {
      return;
    }

    GPU_shader_bind(sh);

    GPU_texture_image_bind(gbuf_pos_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_pos_img"));
    GPU_texture_image_bind(gbuf_color_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_color_img"));
    GPU_texture_image_bind(comp_color_tx_, GPU_shader_get_sampler_binding(sh, "out_color_img"));
    GPU_texture_image_bind(comp_depth_tx_, GPU_shader_get_sampler_binding(sh, "out_depth_img"));
    GPU_texture_image_bind(gbuf_normal_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_normal_img"));

    if (matcap_tx_) {
      int matcap_slot = GPU_shader_get_sampler_binding(sh, "matcap_tx");
      GPU_texture_bind(matcap_tx_, matcap_slot);
    }

    GPU_shader_uniform_1i(sh, "lighting_type", lighting_type_);
    GPU_shader_uniform_1i(sh, "use_specular", use_specular_);
    GPU_shader_uniform_1i(sh, "use_matcap_flip", use_matcap_flip_);
    GPU_shader_uniform_1f(sh, "sdf_ray_epsilon", sdf_ray_epsilon_);
    GPU_shader_uniform_2iv(sh, "screen_size", &render_size_.x);

    GPU_shader_uniform_4fv(sh, "studio_light0", studio_light_dir_[0]);
    GPU_shader_uniform_4fv(sh, "studio_light1", studio_light_dir_[1]);
    GPU_shader_uniform_4fv(sh, "studio_light2", studio_light_dir_[2]);
    GPU_shader_uniform_4fv(sh, "studio_light3", studio_light_dir_[3]);
    GPU_shader_uniform_4fv(sh, "studio_color0", studio_light_col_[0]);
    GPU_shader_uniform_4fv(sh, "studio_color1", studio_light_col_[1]);
    GPU_shader_uniform_4fv(sh, "studio_color2", studio_light_col_[2]);
    GPU_shader_uniform_4fv(sh, "studio_color3", studio_light_col_[3]);
    GPU_shader_uniform_4fv(sh, "studio_spec0", studio_light_spec_[0]);
    GPU_shader_uniform_4fv(sh, "studio_spec1", studio_light_spec_[1]);
    GPU_shader_uniform_4fv(sh, "studio_spec2", studio_light_spec_[2]);
    GPU_shader_uniform_4fv(sh, "studio_spec3", studio_light_spec_[3]);
    GPU_shader_uniform_3fv(sh, "studio_ambient", studio_ambient_);

    View &view = View::default_get();
    GPU_uniformbuf_bind(view.matrices_ubo_get(), DRW_VIEW_UBO_SLOT);

    int dispatch_x = (render_size_.x + 7) / 8;
    int dispatch_y = (render_size_.y + 7) / 8;
    GPU_compute_dispatch(sh, dispatch_x, dispatch_y, 1);

    GPU_texture_image_unbind(gbuf_pos_tx_);
    GPU_texture_image_unbind(gbuf_color_tx_);
    GPU_texture_image_unbind(comp_color_tx_);
    GPU_texture_image_unbind(comp_depth_tx_);
    GPU_texture_image_unbind(gbuf_normal_tx_);
    if (matcap_tx_) {
      GPU_texture_unbind(matcap_tx_);
    }
    GPU_shader_unbind();
  }

  void draw_blit()
  {
    DefaultFramebufferList *dfbl = draw_ctx_->viewport_framebuffer_list_get();
    if (draw_ctx_->is_depth() || (draw_ctx_->v3d && draw_ctx_->v3d->shading.type == OB_RENDER)) {
      GPU_framebuffer_bind(dfbl->depth_only_fb);
    }
    else if (fxaa_enabled_) {
      ensure_fxaa_target();
      GPU_framebuffer_bind(march_fb_);
      float clear_color[4] = {0.0f, 0.0f, 0.0f, 0.0f};
      GPU_framebuffer_clear_color(march_fb_, clear_color);
    }
    else {
      GPU_framebuffer_bind(dfbl->default_fb);
    }

    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
    GPU_depth_mask(true);
    GPU_blend(GPU_BLEND_NONE);
    GPU_face_culling(GPU_CULL_NONE);
    GPU_stencil_test(GPU_STENCIL_NONE);

    GPU_shader_bind(blit_sh_);
    GPU_shader_uniform_1i(blit_sh_, "debug_bvh_views", debug_bvh_views_);

    float bg[3] = {0.0f, 0.0f, 0.0f};
    if (draw_ctx_->scene && draw_ctx_->v3d) {
      ED_view3d_background_color_get(draw_ctx_->scene, draw_ctx_->v3d, bg);
    }
    GPU_shader_uniform_3fv(blit_sh_, "bg_color", bg);

    float uv_sc[2] = {float(render_size_.x) / float(texture_size_.x),
                       float(render_size_.y) / float(texture_size_.y)};
    GPU_shader_uniform_2fv(blit_sh_, "uv_scale", uv_sc);

    int color_slot = GPU_shader_get_sampler_binding(blit_sh_, "color_tx");
    GPU_texture_filter_mode(comp_color_tx_, false);
    GPU_texture_bind(comp_color_tx_, color_slot);
    int depth_slot = GPU_shader_get_sampler_binding(blit_sh_, "depth_tx");
    GPU_texture_filter_mode(comp_depth_tx_, false);
    GPU_texture_bind(comp_depth_tx_, depth_slot);

    if (fullscreen_batch_ == nullptr) {
      fullscreen_batch_ = GPU_batch_create_procedural(GPU_PRIM_TRIS, 3);
    }
    GPU_batch_set_shader(fullscreen_batch_, blit_sh_);
    GPU_batch_draw(fullscreen_batch_);

    GPU_texture_unbind(comp_color_tx_);
    GPU_texture_unbind(comp_depth_tx_);
    GPU_shader_unbind();
  }

  /* FXAA post-process */

  void ensure_fxaa_target()
  {
    const int2 vp = int2(draw_ctx_->viewport_size_get());
    if (march_color_tx_ != nullptr && fxaa_size_ == vp) {
      return;
    }

    if (march_color_tx_) {
      GPU_texture_free(march_color_tx_);
      march_color_tx_ = nullptr;
    }
    if (march_fb_) {
      GPU_framebuffer_free(march_fb_);
      march_fb_ = nullptr;
    }

    fxaa_size_ = vp;

    eGPUTextureUsage usage = GPU_TEXTURE_USAGE_SHADER_READ | GPU_TEXTURE_USAGE_ATTACHMENT;
    march_color_tx_ = GPU_texture_create_2d(
        "sdf_march_color", vp.x, vp.y, 1, gpu::TextureFormat::SFLOAT_16_16_16_16, usage, nullptr);

    DefaultTextureList *dtxl = draw_ctx_->viewport_texture_list_get();
    march_fb_ = GPU_framebuffer_create("sdf_march_fb");
    GPU_framebuffer_texture_attach(march_fb_, march_color_tx_, 0, 0);
    GPU_framebuffer_texture_attach(march_fb_, dtxl->depth, 0, 0);
  }

  void draw_fxaa()
  {
    if (!fxaa_enabled_ || fxaa_sh_ == nullptr) {
      return;
    }

    if (draw_ctx_->is_depth() || (draw_ctx_->v3d && draw_ctx_->v3d->shading.type == OB_RENDER)) {
      return;
    }

    DefaultFramebufferList *dfbl = draw_ctx_->viewport_framebuffer_list_get();
    GPU_framebuffer_bind(dfbl->default_fb);

    GPU_depth_test(GPU_DEPTH_NONE);
    GPU_depth_mask(false);
    GPU_blend(GPU_BLEND_ALPHA);
    GPU_face_culling(GPU_CULL_NONE);
    GPU_stencil_test(GPU_STENCIL_NONE);

    GPU_shader_bind(fxaa_sh_);

    int color_slot = GPU_shader_get_sampler_binding(fxaa_sh_, "color_tx");
    GPU_texture_filter_mode(march_color_tx_, true);
    GPU_texture_bind(march_color_tx_, color_slot);

    float2 rcp = float2(1.0f / float(viewport_size_.x), 1.0f / float(viewport_size_.y));
    GPU_shader_uniform_2fv(fxaa_sh_, "rcpFrame", rcp);

    if (fullscreen_batch_ == nullptr) {
      fullscreen_batch_ = GPU_batch_create_procedural(GPU_PRIM_TRIS, 3);
    }
    GPU_batch_set_shader(fullscreen_batch_, fxaa_sh_);
    GPU_batch_draw(fullscreen_batch_);

    GPU_texture_unbind(march_color_tx_);
    GPU_shader_unbind();

    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
    GPU_depth_mask(true);
    GPU_blend(GPU_BLEND_NONE);
  }

 public:
  ~Instance() override
  {
    /* Shaders live in static cache — do NOT free them here. */

    s_object_ssbo = nullptr;
    s_modifier_ssbo = nullptr;
    s_polygon_ssbo = nullptr;
    s_group_ssbo = nullptr;
    s_group_count = 0;
    s_bvh_ssbo = nullptr;
    s_bvh_root = -1;
    s_depth_tx = nullptr;
    s_gbuf_color_tx = nullptr;
    s_render_size = {0, 0};
    s_texture_size = {0, 0};
    s_object_count = 0;

    if (comp_color_tx_) {
      GPU_texture_free(comp_color_tx_);
    }
    if (comp_depth_tx_) {
      GPU_texture_free(comp_depth_tx_);
    }
    if (gbuf_pos_tx_) {
      GPU_texture_free(gbuf_pos_tx_);
    }
    if (gbuf_color_tx_) {
      GPU_texture_free(gbuf_color_tx_);
    }
    if (gbuf_normal_tx_) {
      GPU_texture_free(gbuf_normal_tx_);
    }
    if (march_color_tx_) {
      GPU_texture_free(march_color_tx_);
    }
    if (march_fb_) {
      GPU_framebuffer_free(march_fb_);
    }
    if (matcap_tx_) {
      GPU_texture_free(matcap_tx_);
    }
    if (object_ssbo_) {
      GPU_storagebuf_free(object_ssbo_);
    }
    if (object_aabb_ssbo_) {
      GPU_storagebuf_free(object_aabb_ssbo_);
    }
    if (modifier_ssbo_) {
      GPU_storagebuf_free(modifier_ssbo_);
    }
    if (polygon_ssbo_) {
      GPU_storagebuf_free(polygon_ssbo_);
    }
    if (group_ssbo_) {
      GPU_storagebuf_free(group_ssbo_);
    }
    if (bvh_nodes_ssbo_) {
      GPU_storagebuf_free(bvh_nodes_ssbo_);
    }
    if (cone_hit_ssbo_) {
      GPU_storagebuf_free(cone_hit_ssbo_);
    }
    if (tile_far_hint_ssbo_) {
      GPU_storagebuf_free(tile_far_hint_ssbo_);
    }
    if (screen_aabbs_ssbo_) {
      GPU_storagebuf_free(screen_aabbs_ssbo_);
    }
    if (tile_prim_counts_ssbo_) {
      GPU_storagebuf_free(tile_prim_counts_ssbo_);
    }
    if (tile_prim_lists_ssbo_) {
      GPU_storagebuf_free(tile_prim_lists_ssbo_);
    }
    if (fullscreen_batch_) {
      GPU_batch_discard(fullscreen_batch_);
    }
  }
};

/* Public API */

int sdf_object_count_get()
{
  return s_object_count;
}

int sdf_group_count_get()
{
  return s_group_count;
}

const int *sdf_depsgraph_to_sorted_get(int *out_count)
{
  if (s_depsgraph_to_sorted.is_empty()) {
    *out_count = 0;
    return nullptr;
  }
  *out_count = int(s_depsgraph_to_sorted.size());
  return s_depsgraph_to_sorted.data();
}

gpu::Texture *sdf_depth_texture_get()
{
  return s_depth_tx;
}

gpu::Texture *sdf_gbuf_color_texture_get()
{
  return s_gbuf_color_tx;
}

float2 sdf_uv_scale_get()
{
  if (s_texture_size.x == 0 || s_texture_size.y == 0) {
    return float2(1.0f);
  }
  return float2(float(s_render_size.x) / float(s_texture_size.x),
                float(s_render_size.y) / float(s_texture_size.y));
}

/* Debug: near-surface points from last bbox computation. */
static Vector<float3> s_bbox_debug_points;
static float4x4 s_bbox_debug_rot;
static float3 s_bbox_debug_pos;

void sdf_bbox_debug_points_get(const float3 **pts, int *count,
                               float4x4 *rot, float3 *pos)
{
  *pts = s_bbox_debug_points.data();
  *count = int(s_bbox_debug_points.size());
  *rot = s_bbox_debug_rot;
  *pos = s_bbox_debug_pos;
}

bool sdf_object_bbox_get(int sdf_index, float3 &out_min, float3 &out_max,
                         float4x4 &out_rot, float3 &out_pos)
{
  s_bbox_debug_points.clear();

  for (int i = 0; i < s_objects_cpu_count; i++) {
    if (s_objects_cpu[i].original_index == sdf_index) {
      const SDFObjectGPU &obj = s_objects_cpu[i];
      float3 sz(obj.sdf_size);
      float3 pos = float3(obj.position);
      float4x4 rot = math::transpose(obj.inverse_matrix);

      /* Rotation-stable search region: compute in local space from primitive. */
      float3 ext;
      switch (obj.sdf_type) {
        case SDF_TYPE_CONE: ext = float3(sz.x, sz.x, sz.y); break;
        case SDF_TYPE_CAPSULE: ext = float3(sz.x, sz.x, sz.y + sz.x); break;
        case SDF_TYPE_TORUS: ext = float3(sz.x + sz.y, sz.x + sz.y, sz.y); break;
        case SDF_TYPE_NGON: ext = float3(sz.x, sz.x, sz.z); break;
        default: ext = sz; break;
      }
      /* Search region: expand by non-mirror modifiers only.
       * Mirror bbox copies are drawn by the overlay's copy system. */
      float3 search_ext = ext;
      for (int mi = obj.modifier_start; mi < obj.modifier_start + obj.modifier_count; mi++) {
        if (mi < 0 || mi >= s_modifier_count) { break; }
        const SDFModifierGPU &mod = s_modifiers_cpu[mi];
        int mt = mod.header.x;
        if (mt == SDF_MOD_ELONGATE) {
          search_ext += float3(mod.params.x, mod.params.y, mod.params.z);
        }
        else if (mt == SDF_MOD_TWIST || mt == SDF_MOD_BEND) {
          float diag = math::length(search_ext);
          search_ext = float3(diag);
        }
        else if (mt == SDF_MOD_SOLIDIFY || mt == SDF_MOD_ONION) {
          /* Never expand — these only carve inward. */
        }
        else if (mt == SDF_MOD_ROUND || mt == SDF_MOD_BEVEL) {
          search_ext = math::max(search_ext + float3(mod.params.x), float3(0.0f));
        }
      }
      search_ext *= 1.1f;
      float3 search_min = -search_ext;
      float3 search_max = search_ext;

      constexpr int RES = 20;
      float3 cell = (search_max - search_min) / float(RES);
      float threshold = std::max(cell.x, std::max(cell.y, cell.z)) * 0.6f;
      float3 bb_min(1e30f), bb_max(-1e30f);
      bool found = false;

      s_bbox_debug_rot = rot;
      s_bbox_debug_pos = pos;

      for (int iz = 0; iz <= RES; iz++) {
        for (int iy = 0; iy <= RES; iy++) {
          for (int ix = 0; ix <= RES; ix++) {
            float3 lp = search_min + float3(float(ix), float(iy), float(iz)) * cell;
            float d = sdf_cpu::evalObjectSDF(obj, s_modifiers_cpu, lp, true, true);
            if (d >= 0.0f && d < threshold) {
              bb_min = math::min(bb_min, lp);
              bb_max = math::max(bb_max, lp);
              s_bbox_debug_points.append(lp);
              found = true;
            }
          }
        }
      }

      if (!found) {
        bb_min = -ext;
        bb_max = ext;
      }
      else {
        bb_min -= cell * 0.25f;
        bb_max += cell * 0.25f;
      }

      out_min = bb_min;
      out_max = bb_max;
      out_rot = rot;
      out_pos = pos;
      return true;
    }
  }
  return false;
}

DrawEngine *Engine::create_instance()
{
  return new Instance();
}

std::string sdf_dual_contour_to_mesh(int grid_res,
                                      Vector<float3> &out_positions,
                                      Vector<float3> &out_normals,
                                      Vector<int3> &out_tris,
                                      Vector<float4> &out_colors,
                                      int *out_vert_count,
                                      int *out_tri_count)
{
  *out_vert_count = 0;
  *out_tri_count = 0;

  if (s_object_count == 0 || !s_object_ssbo) {
    return "No SDF data. Render viewport first.";
  }

  const float cell_size = 1.0f / float(grid_res);
  const int CHUNK = 256;
  const int PAD = 1;
  const int PADDED = CHUNK + 2 * PAD;
  const int PADDED_GV = PADDED + 1;

  /* Scene AABB */
  Vector<SDFObjectGPU> objs(s_object_count);
  GPU_storagebuf_read(s_object_ssbo, objs.data());

  float3 scene_min(1e30f), scene_max(-1e30f);
  for (int i = 0; i < s_object_count; i++) {
    scene_min = math::min(scene_min, float3(objs[i].bbox_min));
    scene_max = math::max(scene_max, float3(objs[i].bbox_max));
  }
  scene_min -= float3(cell_size * 2.0f);
  scene_max += float3(cell_size * 2.0f);

  int3 grid_cells;
  grid_cells.x = int(ceilf((scene_max.x - scene_min.x) / cell_size));
  grid_cells.y = int(ceilf((scene_max.y - scene_min.y) / cell_size));
  grid_cells.z = int(ceilf((scene_max.z - scene_min.z) / cell_size));
  float3 grid_origin = scene_min;

  int3 n_chunks;
  n_chunks.x = (grid_cells.x + CHUNK - 1) / CHUNK;
  n_chunks.y = (grid_cells.y + CHUNK - 1) / CHUNK;
  n_chunks.z = (grid_cells.z + CHUNK - 1) / CHUNK;

  /* Compile shaders (cached) */
  gpu::Shader *grid_sh = GPU_shader_create_from_info_name("sdf_grid_eval_comp");
  gpu::Shader *dc_sh = GPU_shader_create_from_info_name("sdf_dc_contour_comp");
  gpu::Shader *tri_sh = GPU_shader_create_from_info_name("sdf_dc_triangulate_comp");
  gpu::Shader *color_sh = GPU_shader_create_from_info_name("sdf_dc_vertex_color_comp");
  if (!grid_sh || !dc_sh || !tri_sh || !color_sh) return "Shader compile failed";

  /* Global output buffers. Readback = verts + tris transferred from GPU→CPU.
   * 4M verts (128 MB) + 8M tris (128 MB) = 256 MB. PCIe transfer ~20ms.
   * The GPU sync (waiting for compute to finish) dominates readback time. */
  const int global_max_verts = 4 * 1024 * 1024;
  const int global_max_tris = 8 * 1024 * 1024;

  gpu::StorageBuf *vert_ssbo = GPU_storagebuf_create_ex(
      global_max_verts * sizeof(DCVertexGPU), nullptr, GPU_USAGE_DYNAMIC, "dc_vertices");
  gpu::StorageBuf *counter_ssbo = GPU_storagebuf_create_ex(
      2 * sizeof(int), nullptr, GPU_USAGE_DYNAMIC, "dc_counters");
  GPU_storagebuf_clear_to_zero(counter_ssbo);
  gpu::StorageBuf *tri_ssbo = GPU_storagebuf_create_ex(
      global_max_tris * sizeof(int) * 4, nullptr, GPU_USAGE_DYNAMIC, "dc_triangles");

  /* Per-chunk buffers (reused) */
  int padded_grid_total = PADDED_GV * PADDED_GV * PADDED_GV;
  int padded_cells_total = PADDED * PADDED * PADDED;
  gpu::StorageBuf *grid_ssbo = GPU_storagebuf_create_ex(
      padded_grid_total * sizeof(float), nullptr, GPU_USAGE_DYNAMIC, "dc_grid_values");
  gpu::StorageBuf *cell_ssbo = GPU_storagebuf_create_ex(
      padded_cells_total * sizeof(int), nullptr, GPU_USAGE_DYNAMIC, "dc_cell_verts");

  /* Color output buffer (one float4 per vertex) */
  gpu::StorageBuf *color_ssbo = GPU_storagebuf_create_ex(
      global_max_verts * sizeof(float4), nullptr, GPU_USAGE_DYNAMIC, "dc_colors");

  auto cleanup = [&]() {
    GPU_storagebuf_free(grid_ssbo);
    GPU_storagebuf_free(vert_ssbo);
    GPU_storagebuf_free(counter_ssbo);
    GPU_storagebuf_free(cell_ssbo);
    GPU_storagebuf_free(tri_ssbo);
    GPU_storagebuf_free(color_ssbo);
    GPU_shader_free(grid_sh);
    GPU_shader_free(dc_sh);
    GPU_shader_free(tri_sh);
    GPU_shader_free(color_sh);
  };

  /* Pre-bind SSBO slots that don't change between chunks */
  int grid_obj_slot = GPU_shader_get_ssbo_binding(grid_sh, "objects");
  int grid_mod_slot = GPU_shader_get_ssbo_binding(grid_sh, "sdf_modifiers");
  int grid_grp_slot = GPU_shader_get_ssbo_binding(grid_sh, "groups");
  int grid_poly_slot = s_polygon_ssbo ?
                           GPU_shader_get_ssbo_binding(grid_sh, "polygon_points") :
                           -1;
  int grid_val_slot = GPU_shader_get_ssbo_binding(grid_sh, "grid_values");
  int grid_bvh_slot = GPU_shader_get_ssbo_binding(grid_sh, "aabb_nodes");
  int has_bvh = (s_bvh_ssbo && s_bvh_root >= 0) ? 1 : 0;

  int dc_gv_slot = GPU_shader_get_ssbo_binding(dc_sh, "grid_values");
  int dc_v_slot = GPU_shader_get_ssbo_binding(dc_sh, "dc_vertices");
  int dc_c_slot = GPU_shader_get_ssbo_binding(dc_sh, "dc_counters");
  int dc_cv_slot = GPU_shader_get_ssbo_binding(dc_sh, "dc_cell_verts");

  int tr_gv_slot = GPU_shader_get_ssbo_binding(tri_sh, "grid_values");
  int tr_t_slot = GPU_shader_get_ssbo_binding(tri_sh, "dc_triangles");
  int tr_c_slot = GPU_shader_get_ssbo_binding(tri_sh, "dc_counters");
  int tr_cv_slot = GPU_shader_get_ssbo_binding(tri_sh, "dc_cell_verts");

  int gd = (PADDED_GV + 3) / 4;
  int cd = (PADDED + 3) / 4;
  int chunk_done = 0, chunk_skipped = 0;

  for (int cz = 0; cz < n_chunks.z; cz++) {
    for (int cy = 0; cy < n_chunks.y; cy++) {
      for (int cx = 0; cx < n_chunks.x; cx++) {
        int3 cell_start = int3(cx, cy, cz) * CHUNK - int3(PAD);
        float3 chunk_origin = grid_origin + float3(cell_start) * cell_size;
        float3 chunk_min = chunk_origin;
        float3 chunk_max = chunk_origin + float3(float(PADDED_GV)) * cell_size;

        /* AABB cull: skip chunks with no nearby SDF objects */
        bool has_overlap = false;
        for (int i = 0; i < s_object_count; i++) {
          float3 obj_min = float3(objs[i].bbox_min);
          float3 obj_max = float3(objs[i].bbox_max);
          if (chunk_min.x <= obj_max.x && chunk_max.x >= obj_min.x &&
              chunk_min.y <= obj_max.y && chunk_max.y >= obj_min.y &&
              chunk_min.z <= obj_max.z && chunk_max.z >= obj_min.z)
          {
            has_overlap = true;
            break;
          }
        }
        if (!has_overlap) {
          chunk_skipped++;
          chunk_done++;
          continue;
        }

        GPU_storagebuf_clear(cell_ssbo, 0xFFFFFFFFu);

        /* Grid eval */
        GPU_shader_bind(grid_sh);
        if (s_object_ssbo) GPU_storagebuf_bind(s_object_ssbo, grid_obj_slot);
        if (s_modifier_ssbo) GPU_storagebuf_bind(s_modifier_ssbo, grid_mod_slot);
        if (s_group_ssbo) GPU_storagebuf_bind(s_group_ssbo, grid_grp_slot);
        if (grid_poly_slot >= 0) GPU_storagebuf_bind(s_polygon_ssbo, grid_poly_slot);
        GPU_storagebuf_bind(grid_ssbo, grid_val_slot);
        if (has_bvh && grid_bvh_slot >= 0) GPU_storagebuf_bind(s_bvh_ssbo, grid_bvh_slot);
        GPU_shader_uniform_1i(grid_sh, "object_count", s_object_count);
        GPU_shader_uniform_1i(grid_sh, "group_count", s_group_count);
        GPU_shader_uniform_1i(grid_sh, "grid_verts", PADDED_GV);
        GPU_shader_uniform_3fv(grid_sh, "grid_origin", chunk_origin);
        GPU_shader_uniform_1f(grid_sh, "cell_size", cell_size);
        GPU_shader_uniform_1i(grid_sh, "use_bvh", has_bvh);
        GPU_shader_uniform_1i(grid_sh, "bvh_root", s_bvh_root);
        GPU_compute_dispatch(grid_sh, gd, gd, gd);
        GPU_memory_barrier(GPU_BARRIER_SHADER_STORAGE);

        /* DC contour → writes to GLOBAL vert buffer + local cell map */
        GPU_shader_bind(dc_sh);
        GPU_storagebuf_bind(grid_ssbo, dc_gv_slot);
        GPU_storagebuf_bind(vert_ssbo, dc_v_slot);
        GPU_storagebuf_bind(counter_ssbo, dc_c_slot);
        GPU_storagebuf_bind(cell_ssbo, dc_cv_slot);
        GPU_shader_uniform_1i(dc_sh, "grid_verts", PADDED_GV);
        GPU_shader_uniform_3fv(dc_sh, "grid_origin", chunk_origin);
        GPU_shader_uniform_1f(dc_sh, "cell_size", cell_size);
        GPU_shader_uniform_1i(dc_sh, "max_verts", global_max_verts);
        GPU_compute_dispatch(dc_sh, cd, cd, cd);
        GPU_memory_barrier(GPU_BARRIER_SHADER_STORAGE);

        /* Triangulation → only inner cells, writes to GLOBAL tri buffer */
        GPU_shader_bind(tri_sh);
        GPU_storagebuf_bind(grid_ssbo, tr_gv_slot);
        GPU_storagebuf_bind(tri_ssbo, tr_t_slot);
        GPU_storagebuf_bind(counter_ssbo, tr_c_slot);
        GPU_storagebuf_bind(cell_ssbo, tr_cv_slot);
        GPU_shader_uniform_1i(tri_sh, "grid_verts", PADDED_GV);
        GPU_shader_uniform_1i(tri_sh, "inner_start", PAD);
        GPU_shader_uniform_1i(tri_sh, "inner_end", PAD + CHUNK);
        GPU_shader_uniform_1i(tri_sh, "max_tris", global_max_tris);
        GPU_compute_dispatch(tri_sh, cd, cd, cd);
        GPU_memory_barrier(GPU_BARRIER_SHADER_STORAGE);

        chunk_done++;
      }
    }
  }
  GPU_shader_unbind();

  /* Single readback at end */
  Vector<int> counters(2);
  GPU_storagebuf_read(counter_ssbo, counters.data());
  int vert_count = math::min(counters[0], global_max_verts);
  int tri_count = math::min(counters[1], global_max_tris);

  if (vert_count == 0) {
    cleanup();
    return "DC produced 0 vertices";
  }

  /* Vertex color pass: evaluate SDF color at each DC vertex position */
  {
    GPU_shader_bind(color_sh);
    int col_obj_slot = GPU_shader_get_ssbo_binding(color_sh, "objects");
    int col_mod_slot = GPU_shader_get_ssbo_binding(color_sh, "sdf_modifiers");
    int col_grp_slot = GPU_shader_get_ssbo_binding(color_sh, "groups");
    int col_poly_slot = s_polygon_ssbo ?
                            GPU_shader_get_ssbo_binding(color_sh, "polygon_points") :
                            -1;
    int col_pos_slot = GPU_shader_get_ssbo_binding(color_sh, "dc_positions");
    int col_out_slot = GPU_shader_get_ssbo_binding(color_sh, "dc_colors");
    int col_bvh_slot = GPU_shader_get_ssbo_binding(color_sh, "aabb_nodes");

    if (s_object_ssbo) GPU_storagebuf_bind(s_object_ssbo, col_obj_slot);
    if (s_modifier_ssbo) GPU_storagebuf_bind(s_modifier_ssbo, col_mod_slot);
    if (s_group_ssbo) GPU_storagebuf_bind(s_group_ssbo, col_grp_slot);
    if (col_poly_slot >= 0) GPU_storagebuf_bind(s_polygon_ssbo, col_poly_slot);
    GPU_storagebuf_bind(vert_ssbo, col_pos_slot);
    GPU_storagebuf_bind(color_ssbo, col_out_slot);
    if (has_bvh && col_bvh_slot >= 0) GPU_storagebuf_bind(s_bvh_ssbo, col_bvh_slot);

    GPU_shader_uniform_1i(color_sh, "object_count", s_object_count);
    GPU_shader_uniform_1i(color_sh, "group_count", s_group_count);
    GPU_shader_uniform_1i(color_sh, "vert_count", vert_count);
    GPU_shader_uniform_1i(color_sh, "use_bvh", has_bvh);
    GPU_shader_uniform_1i(color_sh, "bvh_root", s_bvh_root);

    int color_groups = (vert_count + 63) / 64;
    GPU_compute_dispatch(color_sh, color_groups, 1, 1);
    GPU_memory_barrier(GPU_BARRIER_SHADER_STORAGE);
    GPU_shader_unbind();
  }

  Vector<DCVertexGPU> gpu_verts(global_max_verts);
  GPU_storagebuf_read(vert_ssbo, gpu_verts.data());

  out_positions.resize(vert_count);
  out_normals.resize(vert_count);
  for (int i = 0; i < vert_count; i++) {
    out_positions[i] = float3(gpu_verts[i].position);
    out_normals[i] = float3(gpu_verts[i].normal);
  }

  /* Read back vertex colors */
  Vector<float4> gpu_colors(global_max_verts);
  GPU_storagebuf_read(color_ssbo, gpu_colors.data());
  out_colors.resize(vert_count);
  for (int i = 0; i < vert_count; i++) {
    out_colors[i] = gpu_colors[i];
  }

  Vector<int4> gpu_tris(global_max_tris);
  GPU_storagebuf_read(tri_ssbo, gpu_tris.data());

  out_tris.reserve(tri_count);
  for (int i = 0; i < tri_count; i++) {
    int a = gpu_tris[i].x, b = gpu_tris[i].y, c = gpu_tris[i].z;
    if (a >= 0 && a < vert_count && b >= 0 && b < vert_count && c >= 0 && c < vert_count) {
      out_tris.append(int3(a, b, c));
    }
  }

  *out_vert_count = vert_count;
  *out_tri_count = out_tris.size();

  cleanup();
  return "";
}

}  // namespace blender::draw::sdf
