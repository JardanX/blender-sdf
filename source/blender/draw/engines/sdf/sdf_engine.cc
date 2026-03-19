/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw_engine
 *
 * SDF draw engine: analytical sphere tracer — evaluates SDF primitives directly per-pixel.
 */

#include <algorithm>
#include <chrono>

#include "BLI_math_matrix.h"
#include "BLI_math_matrix.hh"
#include "BLI_math_rotation.h"
#include "BLI_math_vector.h"
#include "BLI_math_vector.hh"

#include "BKE_main.hh"
#include "BKE_object.hh"
#include "BKE_object_types.hh"
#include "BKE_sdf.hh"
#include "BKE_sdf_group.hh"
#include "BKE_studiolight.h"

#include "DEG_depsgraph_query.hh"

#include "DNA_object_types.h"
#include "DNA_sdf_group_types.h"
#include "DNA_sdf_types.h"
#include "DNA_userdef_types.h"
#include "DNA_view3d_enums.h"
#include "DNA_view3d_types.h"

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

#include "sdf_private.hh"

#include "sdf_engine.h"

#include <cmath>
#include <cstdio>
#include <string>

namespace blender::draw::sdf {

using namespace draw;

/* Static state shared with overlay/selection code */
static int s_object_count = 0;
static gpu::StorageBuf *s_object_ssbo = nullptr;
static gpu::StorageBuf *s_modifier_ssbo = nullptr;
static gpu::StorageBuf *s_group_ssbo = nullptr;
static int s_group_count = 0;
static Vector<int> s_depsgraph_to_sorted;

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

  enum ShaderIndex {
    SH_TRACE_COMP = 0,
    SH_TRACE_TILE_COMP,
    SH_TILE_CULL_COMP,
    SH_CONE_MARCH_COMP,
    SH_SHADE_COMP,
    SH_BLIT,
    SH_FXAA,
    SH_COUNT,
  };

  static constexpr const char *shader_info_names_[SH_COUNT] = {
      "sdf_trace_comp",
      "sdf_trace_tile_comp",
      "sdf_tile_cull_comp",
      "sdf_cone_march_comp",
      "sdf_shade_comp",
      "sdf_blit",
      "sdf_fxaa",
  };

  gpu::Shader *shaders_[SH_COUNT] = {};

  gpu::Shader *&trace_comp_sh_ = shaders_[SH_TRACE_COMP];
  gpu::Shader *&trace_tile_sh_ = shaders_[SH_TRACE_TILE_COMP];
  gpu::Shader *&tile_cull_sh_ = shaders_[SH_TILE_CULL_COMP];
  gpu::Shader *&cone_march_sh_ = shaders_[SH_CONE_MARCH_COMP];
  gpu::Shader *&shade_comp_sh_ = shaders_[SH_SHADE_COMP];
  gpu::Shader *&blit_sh_ = shaders_[SH_BLIT];
  gpu::Shader *&fxaa_sh_ = shaders_[SH_FXAA];

  BatchHandle shader_compile_batch_ = 0;
  bool shaders_compiled_ = false;

  gpu::Texture *comp_color_tx_ = nullptr;
  gpu::Texture *comp_depth_tx_ = nullptr;
  gpu::Texture *gbuf_pos_tx_ = nullptr;
  gpu::Texture *gbuf_color_tx_ = nullptr;
  gpu::Texture *gbuf_normal_tx_ = nullptr;
  gpu::Texture *march_color_tx_ = nullptr;
  gpu::FrameBuffer *march_fb_ = nullptr;
  int2 render_size_ = int2(0);

  gpu::StorageBuf *object_ssbo_ = nullptr;
  int object_ssbo_count_ = 0;

  Vector<SDFModifierGPU> modifiers_;
  gpu::StorageBuf *modifier_ssbo_ = nullptr;
  int modifier_ssbo_count_ = 0;

  Vector<SDFGroupGPU> groups_gpu_;
  gpu::StorageBuf *group_ssbo_ = nullptr;
  int group_ssbo_count_ = 0;

  SdfAabbTree bvh_tree_;
  gpu::StorageBuf *bvh_nodes_ssbo_ = nullptr;
  int bvh_nodes_ssbo_count_ = 0;

  gpu::StorageBuf *cone_hit_ssbo_ = nullptr;
  int cone_hit_ssbo_count_ = 0;

  gpu::StorageBuf *tile_prim_counts_ssbo_ = nullptr;
  int tile_prim_counts_ssbo_tiles_ = 0;
  gpu::StorageBuf *tile_prim_lists_ssbo_ = nullptr;
  int tile_prim_lists_ssbo_tiles_ = 0;

  gpu::Batch *fullscreen_batch_ = nullptr;

  const DRWContext *draw_ctx_ = nullptr;

  gpu::Texture *matcap_tx_ = nullptr;
  std::string current_matcap_;
  int lighting_type_ = V3D_LIGHTING_STUDIO;
  int use_specular_ = 0;
  int use_matcap_flip_ = 0;
  int use_bvh_ = 1;
  int debug_bvh_views_ = 0;
  int use_cone_trace_ = 0;
  int sdf_max_steps_ = 128;
  float sdf_ray_epsilon_ = 0.001f;
  float sdf_over_relaxation_ = 1.2f;
  float sdf_cone_aperture_ = 1.25f;
  int sdf_cone_steps_ = 16;

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

    if (!shaders_compiled_ && shader_compile_batch_ == 0) {
      const GPUShaderCreateInfo *infos[SH_COUNT];
      for (int i = 0; i < SH_COUNT; i++) {
        infos[i] = GPU_shader_create_info_get(shader_info_names_[i]);
      }
      shader_compile_batch_ = GPU_shader_batch_create_from_infos({infos, SH_COUNT},
                                                                  CompilationPriority::High);
    }
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
    groups_gpu_.clear();
    s_depsgraph_to_sorted.clear();
    bvh_tree_.clear();
    scene_min_ = float3(1e30f);
    scene_max_ = float3(-1e30f);
    max_blend_ = 0.0f;
    max_shell_distance_ = 0.0f;
    step_factor_ = 0.85f;
  }

  void object_sync(ObjectRef &ob_ref, Manager & /*manager*/) final
  {
    if (depth_mode_) {
      return;
    }

    Object *ob = ob_ref.object;

    if (ob->type != OB_SDF) {
      return;
    }

    const SDF *sdf_data = static_cast<const SDF *>(ob->data);
    if (sdf_data == nullptr) {
      return;
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
    LISTBASE_FOREACH (const SDFModifier *, bmod, &sdf_data->modifiers) {
      if (bmod->show_viewport && bmod->type == SDF_MOD_BEVEL) {
        bevel += bmod->params[0];
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

    gpu_obj.group_id = -1;
    gpu_obj.group_first = 0;
    gpu_obj.group_order = sdf_data->group_order;
    gpu_obj.original_index = int(objects_.size());
    object_group_ptrs_.append(sdf_data->sdf_group);
    object_ptrs_.append(ob);

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

    /* Pack modifiers */
    gpu_obj.modifier_start = int(modifiers_.size());
    gpu_obj.modifier_count = 0;
    LISTBASE_FOREACH (const SDFModifier *, mod, &sdf_data->modifiers) {
      if (!mod->show_viewport) {
        continue;
      }
      SDFModifierGPU gpu_mod = {};
      gpu_mod.header = int4(mod->type, mod->flag, 0, 0);
      if (mod->type == SDF_MOD_MIRROR) {
        float3 world_origin;
        if (mod->mirror_ob != nullptr) {
          const float4x4 &mirror_mat = mod->mirror_ob->object_to_world();
          world_origin = float3(mirror_mat[3]) - float3(mat[3]);
        }
        else {
          world_origin = float3(mod->params[1], mod->params[2], mod->params[3]);
        }
        float3 local_origin = float3(inv_rot * float4(world_origin, 0.0f));
        gpu_mod.params = float4(mod->params[0], local_origin.x, local_origin.y, local_origin.z);
      }
      else {
        gpu_mod.params = float4(mod->params[0], mod->params[1], mod->params[2], mod->params[3]);
      }
      gpu_mod.params2 = float4(mod->params[4], mod->params[5], mod->params[6], mod->params[7]);
      modifiers_.append(gpu_mod);
      gpu_obj.modifier_count++;
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
      default:
        local_extent = sz + float3(bevel + pad);
        break;
    }

    /* Expand for domain modifiers */
    LISTBASE_FOREACH (const SDFModifier *, mod, &sdf_data->modifiers) {
      if (!mod->show_viewport) {
        continue;
      }
      switch (mod->type) {
        case SDF_MOD_MIRROR: {
          float offset = fabsf(mod->params[0]);
          float3 world_org;
          if (mod->mirror_ob != nullptr) {
            const float4x4 &mirror_mat = mod->mirror_ob->object_to_world();
            world_org = float3(mirror_mat[3]) - float3(mat[3]);
          }
          else {
            world_org = float3(mod->params[1], mod->params[2], mod->params[3]);
          }
          float3 local_org = float3(inv_rot * float4(world_org, 0.0f));
          if ((mod->flag & SDF_MOD_MIRROR_X) != 0) {
            float3 N = float3(inv_rot[0]);
            float disp = 2.0f * fabsf(math::dot(local_org, N)) + offset;
            local_extent += math::abs(N) * disp;
          }
          if ((mod->flag & SDF_MOD_MIRROR_Y) != 0) {
            float3 N = float3(inv_rot[1]);
            float disp = 2.0f * fabsf(math::dot(local_org, N)) + offset;
            local_extent += math::abs(N) * disp;
          }
          if ((mod->flag & SDF_MOD_MIRROR_Z) != 0) {
            float3 N = float3(inv_rot[2]);
            float disp = 2.0f * fabsf(math::dot(local_org, N)) + offset;
            local_extent += math::abs(N) * disp;
          }
          break;
        }
        case SDF_MOD_ELONGATE:
          local_extent += float3(mod->params[0], mod->params[1], mod->params[2]);
          break;
        case SDF_MOD_HOLLOW:
        case SDF_MOD_ONION:
          local_extent += float3(mod->params[0]);
          break;
        case SDF_MOD_ROUND:
          local_extent += float3(mod->params[0]);
          break;
        case SDF_MOD_TWIST: {
          float xy = math::sqrt(local_extent.x * local_extent.x +
                                local_extent.y * local_extent.y);
          local_extent.x = xy;
          local_extent.y = xy;
          break;
        }
        case SDF_MOD_BEND: {
          float k = mod->params[0];
          int axis = (int)mod->params[1];
          if (fabsf(k) > 0.0001f) {
            float R = 1.0f / k;
            float3 new_ext = float3(0.0f);
            for (int c = 0; c < 8; c++) {
              float cx = (c & 1) ? local_extent.x : -local_extent.x;
              float cy = (c & 2) ? local_extent.y : -local_extent.y;
              float cz = (c & 4) ? local_extent.z : -local_extent.z;
              float drive, curve, free_val;
              if (axis == 1) {
                drive = cy;
                curve = cz;
                free_val = cx;
              }
              else if (axis == 2) {
                drive = cz;
                curve = cx;
                free_val = cy;
              }
              else {
                drive = cx;
                curve = cy;
                free_val = cz;
              }
              float theta = k * drive;
              float bd = (R + curve) * sinf(theta);
              float bc = -R + (R + curve) * cosf(theta);
              float3 bent;
              if (axis == 1) {
                bent = float3(fabsf(free_val), fabsf(bd), fabsf(bc));
              }
              else if (axis == 2) {
                bent = float3(fabsf(bc), fabsf(free_val), fabsf(bd));
              }
              else {
                bent = float3(fabsf(bd), fabsf(bc), fabsf(free_val));
              }
              new_ext = math::max(new_ext, bent);
            }
            float drive_ext = (axis == 0) ? local_extent.x :
                              (axis == 1) ? local_extent.y :
                                            local_extent.z;
            float curve_ext = (axis == 0) ? local_extent.y :
                              (axis == 1) ? local_extent.z :
                                            local_extent.x;
            float max_angle = fabsf(k) * drive_ext;
            if (max_angle > float(M_PI_2)) {
              float outer_r = fabsf(R) + curve_ext;
              if (axis == 0) {
                new_ext.x = math::max(new_ext.x, outer_r);
              }
              else if (axis == 1) {
                new_ext.y = math::max(new_ext.y, outer_r);
              }
              else {
                new_ext.z = math::max(new_ext.z, outer_r);
              }
            }
            local_extent = new_ext;
          }
          break;
        }
        case SDF_MOD_ARRAY: {
          float count = mod->params[0];
          if (count > 0.5f) {
            if (mod->flag == SDF_MOD_ARRAY_LINEAR) {
              float3 offset = float3(mod->params[1], mod->params[2], mod->params[3]);
              local_extent += math::abs(offset) * (count - 1.0f);
            }
            else if (mod->flag == SDF_MOD_ARRAY_RADIAL) {
              float radius = mod->params[1];
              local_extent.x += radius;
              local_extent.y += radius;
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

    float sphere_radius = math::length(local_extent);
    float3 obj_center = float3(mat[3].x, mat[3].y, mat[3].z);
    scene_min_ = math::min(scene_min_, obj_center - float3(sphere_radius));
    scene_max_ = math::max(scene_max_, obj_center + float3(sphere_radius));

    max_blend_ = math::max(max_blend_, gpu_obj.blend);
    if (gpu_obj.blend > 0.001f && gpu_obj.blend_type == SDF_BLEND_ROUND) {
      step_factor_ = min_ff(step_factor_, 0.65f);
    }
    if (sdf_data->csg_operation == SDF_CSG_SHELL) {
      max_shell_distance_ = math::max(max_shell_distance_, fabsf(sdf_data->shell_distance));
    }

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

    auto start_time = std::chrono::high_resolution_clock::now();

    /* Build group GPU data */
    {
      Main *bmain = DEG_get_bmain(draw_ctx_->depsgraph);

      LISTBASE_FOREACH (SDFGroup *, group, &bmain->sdf_groups) {
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
      LISTBASE_FOREACH (SDFGroup *, group, &bmain->sdf_groups) {
        SDFGroupGPU gpu_grp = {};
        gpu_grp.csg_operation = group->csg_operation;
        gpu_grp.blend_type = group->blend_type;
        gpu_grp.blend = group->blend;
        gpu_grp.shell_distance = group->shell_distance;
        gpu_grp.shell_mode = group->shell_mode;
        gpu_grp.first_object = obj_offset;
        gpu_grp.object_count = group->totmember;
        gpu_grp.color = float4(
            group->color[0], group->color[1], group->color[2], group->color[3]);
        if (gpu_grp.blend > 0.001f && gpu_grp.blend_type == SDF_BLEND_ROUND) {
          step_factor_ = min_ff(step_factor_, 0.65f);
        }
        groups_gpu_.append(gpu_grp);
        group_index_map.add(group, g_idx);

        int member_order = 0;
        LISTBASE_FOREACH (SDFGroupMember *, member, &group->members) {
          if (member->object) {
            object_membership_map.add_overwrite(member->object, {g_idx, member_order});
          }
          member_order++;
        }

        g_idx++;
        obj_offset += group->totmember;
      }

      BLI_assert(int(object_ptrs_.size()) == int(objects_.size()));
      for (int i = 0; i < int(objects_.size()); i++) {
        Object *eval_ob = object_ptrs_[i];
        Object *orig_ob = DEG_get_original(eval_ob);
        const GroupMembership *membership = object_membership_map.lookup_ptr(orig_ob);
        if (membership) {
          objects_[i].group_id = membership->group_id;
          objects_[i].group_order = membership->group_order;
        }
      }

      /* Sort objects by (group_id, group_order) */
      {
        const int n = int(objects_.size());
        if (n > 0) {
          Vector<std::pair<int64_t, int>> sort_pairs(n);
          for (int i = 0; i < n; i++) {
            if (objects_[i].group_id >= 0) {
              sort_pairs[i] = {int64_t(objects_[i].group_id) * 100000 + objects_[i].group_order,
                               i};
            }
            else {
              sort_pairs[i] = {int64_t(1000000) + i, i};
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

      /* Resolve group_first and fix first_object/object_count */
      for (int gi = 0; gi < int(groups_gpu_.size()); gi++) {
        int first = -1;
        int count = 0;
        for (int i = 0; i < int(objects_.size()); i++) {
          if (objects_[i].group_id == gi) {
            if (first == -1) {
              first = i;
              objects_[i].group_first = 1;
            }
            count++;
          }
        }
        groups_gpu_[gi].first_object = (first >= 0) ? first : 0;
        groups_gpu_[gi].object_count = count;
      }

      /* Compute max blend per group, store in each member's _pad1 */
      for (int gi = 0; gi < int(groups_gpu_.size()); gi++) {
        float max_blend = fabsf(groups_gpu_[gi].blend);
        int start = groups_gpu_[gi].first_object;
        int cnt = groups_gpu_[gi].object_count;
        for (int m = start; m < start + cnt; m++) {
          float b = (objects_[m].blend_type == 0) ? 0.0f : objects_[m].blend;
          max_blend = std::max(max_blend, b + fabsf(objects_[m].shell_distance));
        }
        for (int m = start; m < start + cnt; m++) {
          memcpy(&objects_[m]._pad1, &max_blend, sizeof(float));
        }
      }
      /* Ungrouped objects: store own blend in _pad1 */
      for (int i = 0; i < int(objects_.size()); i++) {
        if (objects_[i].group_id < 0) {
          float b = (objects_[i].blend_type == 0) ? 0.0f : objects_[i].blend;
          b += fabsf(objects_[i].shell_distance);
          memcpy(&objects_[i]._pad1, &b, sizeof(float));
        }
      }
    }

    /* Expand AABBs for blend-aware CSG */
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

    /* Ungrouped objects */
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

    /* Group bounding boxes */
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
            float sub_blend = groups_gpu_[sub_g].blend + fabsf(groups_gpu_[sub_g].shell_distance);
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

    /* Frustum culling: mark objects outside camera frustum via _pad0 flag */
    {
      const View &view = View::default_get();
      float4x4 vp = view.winmat() * view.viewmat();
      float4 planes[6];
      /* Extract frustum planes from view-projection matrix */
      for (int i = 0; i < 4; i++) {
        planes[0][i] = vp[i][3] + vp[i][0]; /* left */
        planes[1][i] = vp[i][3] - vp[i][0]; /* right */
        planes[2][i] = vp[i][3] + vp[i][1]; /* bottom */
        planes[3][i] = vp[i][3] - vp[i][1]; /* top */
        planes[4][i] = vp[i][3] + vp[i][2]; /* near */
        planes[5][i] = vp[i][3] - vp[i][2]; /* far */
      }
      for (int p = 0; p < 6; p++) {
        planes[p] /= math::length(float3(planes[p]));
      }
      for (int i = 0; i < int(objects_.size()); i++) {
        float3 bmin = new_mins[i];
        float3 bmax = new_maxs[i];
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
        objects_[i]._pad0 = visible ? 1 : 0;
      }
    }

    /* Recompute scene AABB from final expanded bounds */
    scene_min_ = float3(1e30f);
    scene_max_ = float3(-1e30f);
    for (int i = 0; i < int(objects_.size()); i++) {
      scene_min_ = math::min(scene_min_, float3(objects_[i].bbox_min));
      scene_max_ = math::max(scene_max_, float3(objects_[i].bbox_max));
    }

    bvh_tree_.clear();
    for (int i = 0; i < int(objects_.size()); i++) {
      SdfAabb bounds;
      bounds.min = new_mins[i];
      bounds.max = new_maxs[i];
      bvh_tree_.create_proxy(bounds, i);
    }

    needs_upload_ = true;

    auto end_time = std::chrono::high_resolution_clock::now();
    GPU_profile_add_group_cpu(
        "SDF BVH",
        std::chrono::duration_cast<std::chrono::nanoseconds>(start_time.time_since_epoch()).count(),
        std::chrono::duration_cast<std::chrono::nanoseconds>(end_time.time_since_epoch()).count());
  }

  void draw(Manager & /*manager*/) final
  {
    if (objects_.is_empty()) {
      return;
    }

    ensure_shaders();
    if (trace_comp_sh_ == nullptr) {
      return;
    }

    sync_shading();

    DRW_submission_start();

    if (needs_upload_) {
      GPU_debug_group_begin("SDF Upload");
      upload_objects();
      GPU_debug_group_end();
      needs_upload_ = false;
    }

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

    GPU_debug_group_begin("SDF Shade");
    draw_shade();
    GPU_debug_group_end();

    GPU_debug_group_begin("SDF Blit");
    draw_blit();
    GPU_debug_group_end();

    GPU_debug_group_begin("SDF FXAA");
    draw_fxaa();
    GPU_debug_group_end();

    DRW_submission_end();

    /* Update static globals for overlay */
    s_object_count = int(objects_.size());
    s_object_ssbo = object_ssbo_;
    s_modifier_ssbo = modifier_ssbo_;
    s_group_ssbo = group_ssbo_;
    s_group_count = int(groups_gpu_.size());
  }

 private:
  void sync_sdf_settings()
  {
    const View3D *v3d = draw_ctx_->v3d;
    if (v3d == nullptr) {
      return;
    }

    /* Apply defaults for unversioned viewports (new DNA fields default to 0) */
    View3D *v3d_mut = const_cast<View3D *>(v3d);
    if (v3d_mut->shading.sdf_max_steps == 0) {
      v3d_mut->shading.sdf_max_steps = 128;
    }
    if (v3d_mut->shading.sdf_ray_epsilon == 0.0f) {
      v3d_mut->shading.sdf_ray_epsilon = 0.001f;
    }
    if (v3d_mut->shading.sdf_over_relaxation == 0.0f) {
      v3d_mut->shading.sdf_over_relaxation = 1.2f;
    }
    if (v3d_mut->shading.sdf_cone_aperture == 0.0f) {
      v3d_mut->shading.sdf_cone_aperture = 1.25f;
    }
    if (v3d_mut->shading.sdf_cone_steps == 0) {
      v3d_mut->shading.sdf_cone_steps = 16;
    }
    use_bvh_ = 1;
    debug_bvh_views_ = v3d->shading.sdf_bvh_debug_view;
    use_cone_trace_ = v3d->shading.sdf_use_cone_trace ? 1 : 0;
    sdf_max_steps_ = v3d->shading.sdf_max_steps;
    sdf_ray_epsilon_ = v3d->shading.sdf_ray_epsilon;
    sdf_over_relaxation_ = v3d->shading.sdf_over_relaxation;
    sdf_cone_aperture_ = v3d->shading.sdf_cone_aperture;
    sdf_cone_steps_ = v3d->shading.sdf_cone_steps;

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
      render_size_ = int2(0);
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
    if (shaders_compiled_) {
      return;
    }

    if (shader_compile_batch_ != 0) {
      Vector<gpu::Shader *> result = GPU_shader_batch_finalize(shader_compile_batch_);
      for (int i = 0; i < SH_COUNT; i++) {
        shaders_[i] = result[i];
      }
      shaders_compiled_ = true;
      return;
    }

    for (int i = 0; i < SH_COUNT; i++) {
      if (shaders_[i] == nullptr) {
        shaders_[i] = GPU_shader_create_from_info_name(shader_info_names_[i]);
      }
    }
    shaders_compiled_ = true;
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
    if (comp_color_tx_ != nullptr && render_size_ == vp) {
      return;
    }

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
    render_size_ = vp;

    eGPUTextureUsage usage = GPU_TEXTURE_USAGE_SHADER_READ | GPU_TEXTURE_USAGE_SHADER_WRITE;
    comp_color_tx_ = GPU_texture_create_2d("sdf_comp_color", vp.x, vp.y, 1, gpu::TextureFormat::SFLOAT_16_16_16_16, usage, nullptr);
    comp_depth_tx_ = GPU_texture_create_2d("sdf_comp_depth", vp.x, vp.y, 1, gpu::TextureFormat::SFLOAT_32, usage, nullptr);
    gbuf_pos_tx_ = GPU_texture_create_2d("sdf_gbuf_pos", vp.x, vp.y, 1, gpu::TextureFormat::SFLOAT_32_32_32_32, usage, nullptr);
    gbuf_color_tx_ = GPU_texture_create_2d("sdf_gbuf_color", vp.x, vp.y, 1, gpu::TextureFormat::SFLOAT_16_16_16_16, usage, nullptr);
    gbuf_normal_tx_ = GPU_texture_create_2d("sdf_gbuf_normal", vp.x, vp.y, 1, gpu::TextureFormat::SFLOAT_16_16_16_16, usage, nullptr);
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
  }

  void ensure_tile_ssbos(int total_tiles)
  {
    if (tile_prim_counts_ssbo_ != nullptr && tile_prim_counts_ssbo_tiles_ != total_tiles) {
      GPU_storagebuf_free(tile_prim_counts_ssbo_);
      tile_prim_counts_ssbo_ = nullptr;
    }
    if (tile_prim_counts_ssbo_ == nullptr && total_tiles > 0) {
      tile_prim_counts_ssbo_ = GPU_storagebuf_create_ex(
          total_tiles * sizeof(int), nullptr, GPU_USAGE_DYNAMIC, "sdf_tile_prim_counts");
      tile_prim_counts_ssbo_tiles_ = total_tiles;
    }

    if (tile_prim_lists_ssbo_ != nullptr && tile_prim_lists_ssbo_tiles_ != total_tiles) {
      GPU_storagebuf_free(tile_prim_lists_ssbo_);
      tile_prim_lists_ssbo_ = nullptr;
    }
    if (tile_prim_lists_ssbo_ == nullptr && total_tiles > 0) {
      tile_prim_lists_ssbo_ = GPU_storagebuf_create_ex(
          total_tiles * 512 * sizeof(int), nullptr, GPU_USAGE_DYNAMIC, "sdf_tile_prim_lists");
      tile_prim_lists_ssbo_tiles_ = total_tiles;
    }

    if (cone_hit_ssbo_ != nullptr && cone_hit_ssbo_count_ != total_tiles) {
      GPU_storagebuf_free(cone_hit_ssbo_);
      cone_hit_ssbo_ = nullptr;
    }
    if (cone_hit_ssbo_ == nullptr && total_tiles > 0) {
      cone_hit_ssbo_ = GPU_storagebuf_create_ex(
          total_tiles * sizeof(float[4]), nullptr, GPU_USAGE_DYNAMIC, "sdf_cone_hit_ssbo");
      cone_hit_ssbo_count_ = total_tiles;
    }
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

    View &view = View::default_get();
    view.matrices_ubo_get().push_update();
    GPU_uniformbuf_bind(view.matrices_ubo_get(), DRW_VIEW_UBO_SLOT);

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
    if (group_ssbo_) {
      GPU_storagebuf_bind(group_ssbo_, GPU_shader_get_ssbo_binding(sh, "groups"));
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

    /* Bind images: out_color/out_depth for debug views, G-buffer for hit data */
    GPU_texture_image_bind(comp_color_tx_, GPU_shader_get_sampler_binding(sh, "out_color_img"));
    GPU_texture_image_bind(comp_depth_tx_, GPU_shader_get_sampler_binding(sh, "out_depth_img"));
    GPU_texture_image_bind(gbuf_pos_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_pos_img"));
    GPU_texture_image_bind(gbuf_color_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_color_img"));
    GPU_texture_image_bind(gbuf_normal_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_normal_img"));

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
    GPU_texture_image_unbind(gbuf_normal_tx_);
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

    /* SSBOs for brute-force normal computation */
    if (object_ssbo_) {
      GPU_storagebuf_bind(object_ssbo_, GPU_shader_get_ssbo_binding(sh, "objects"));
    }
    if (modifier_ssbo_) {
      GPU_storagebuf_bind(modifier_ssbo_, GPU_shader_get_ssbo_binding(sh, "sdf_modifiers"));
    }
    if (group_ssbo_) {
      GPU_storagebuf_bind(group_ssbo_, GPU_shader_get_ssbo_binding(sh, "groups"));
    }

    /* G-buffer as read, final output as write */
    GPU_texture_image_bind(gbuf_pos_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_pos_img"));
    GPU_texture_image_bind(gbuf_color_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_color_img"));
    GPU_texture_image_bind(comp_color_tx_, GPU_shader_get_sampler_binding(sh, "out_color_img"));
    GPU_texture_image_bind(comp_depth_tx_, GPU_shader_get_sampler_binding(sh, "out_depth_img"));
    GPU_texture_image_bind(gbuf_normal_tx_, GPU_shader_get_sampler_binding(sh, "gbuf_normal_img"));

    if (matcap_tx_) {
      int matcap_slot = GPU_shader_get_sampler_binding(sh, "matcap_tx");
      GPU_texture_bind(matcap_tx_, matcap_slot);
    }

    GPU_shader_uniform_1i(sh, "object_count", int(objects_.size()));
    GPU_shader_uniform_1i(sh, "group_count", int(groups_gpu_.size()));
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

    GPU_depth_test(GPU_DEPTH_ALWAYS);
    GPU_depth_mask(true);
    GPU_blend(GPU_BLEND_NONE);
    GPU_face_culling(GPU_CULL_NONE);
    GPU_stencil_test(GPU_STENCIL_NONE);

    GPU_shader_bind(blit_sh_);
    GPU_shader_uniform_1i(blit_sh_, "debug_bvh_views", debug_bvh_views_);
    int color_slot = GPU_shader_get_sampler_binding(blit_sh_, "color_tx");
    GPU_texture_bind(comp_color_tx_, color_slot);
    int depth_slot = GPU_shader_get_sampler_binding(blit_sh_, "depth_tx");
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
    if (march_color_tx_ != nullptr && render_size_ == vp) {
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

    render_size_ = vp;

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

    float2 rcp = float2(1.0f / float(render_size_.x), 1.0f / float(render_size_.y));
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
    if (shader_compile_batch_ != 0) {
      GPU_shader_batch_cancel(shader_compile_batch_);
    }
    for (int i = 0; i < SH_COUNT; i++) {
      GPU_SHADER_FREE_SAFE(shaders_[i]);
    }

    s_object_ssbo = nullptr;
    s_modifier_ssbo = nullptr;
    s_group_ssbo = nullptr;
    s_group_count = 0;

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
    if (modifier_ssbo_) {
      GPU_storagebuf_free(modifier_ssbo_);
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

void sdf_atlas_params_get(
    float *voxel_size, float3 *origin, float3 *extent, int3 *grid_resolution, int *bricks_per_axis)
{
  *voxel_size = 0.01f;
  *origin = float3(0);
  *extent = float3(0);
  *grid_resolution = int3(0);
  *bricks_per_axis = 0;
}

int sdf_object_count_get()
{
  return s_object_count;
}

gpu::StorageBuf *sdf_objects_ssbo_get()
{
  return s_object_ssbo;
}

gpu::StorageBuf *sdf_modifiers_ssbo_get()
{
  return s_modifier_ssbo;
}

gpu::StorageBuf *sdf_groups_ssbo_get()
{
  return s_group_ssbo;
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

DrawEngine *Engine::create_instance()
{
  return new Instance();
}

}  // namespace blender::draw::sdf
