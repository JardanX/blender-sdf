/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw_engine
 *
 * SDF draw engine: bakes SDF objects into a sparse brick atlas, then ray-marches it.
 */

#include <algorithm>

#include "BLI_map.hh"
#include "BLI_math_matrix.h"
#include "BLI_math_matrix.hh"
#include "BLI_math_rotation.h"
#include "BLI_math_vector.h"
#include "BLI_math_vector.hh"
#include "BLI_set.hh"

#include "BKE_geometry_set.hh"
#include "BKE_idprop.hh"
#include "BKE_main.hh"
#include "BKE_object.hh"
#include "BKE_object_types.hh"
#include "BKE_sdf.hh"
#include "BKE_sdf_group.hh"
#include "BKE_studiolight.h"
#include "BKE_volume.hh"
#include "BKE_volume_render.hh"

#include "DEG_depsgraph_query.hh"

#include "DNA_modifier_types.h"
#include "DNA_node_tree_interface_types.h"
#include "DNA_node_types.h"
#include "DNA_object_types.h"
#include "DNA_sdf_group_types.h"
#include "DNA_sdf_types.h"
#include "DNA_userdef_types.h"
#include "DNA_view3d_enums.h"
#include "DNA_view3d_types.h"

#include "IMB_imbuf_types.hh"

#include "MEM_guardedalloc.h"

#include "GPU_batch.hh"
#include "GPU_compute.hh"
#include "GPU_framebuffer.hh"
#include "GPU_shader.hh"
#include "GPU_shader_builtin.hh"
#include "GPU_state.hh"
#include "GPU_storage_buffer.hh"
#include "GPU_texture.hh"
#include "GPU_uniform_buffer.hh"

#include "draw_defines.hh"
#include "draw_manager.hh"
#include "draw_view.hh"
#include "draw_view_data.hh"

#include "sdf_private.hh"

#include "sdf_engine.h" /* Own include. */

#include "BLI_time.h"

#include <cmath>
#include <cstdio>
#include <string>

namespace blender::draw::sdf {

using namespace draw;

/* Global sparse brick atlas pipeline: classify → bake → march. */

/* Performance overlay state */

static constexpr int PERF_PASS_COUNT = 5;
static constexpr int PERF_PASS_CLASSIFY = 0;
static constexpr int PERF_PASS_BAKE = 1;
static constexpr int PERF_PASS_GRID = 2;
static constexpr int PERF_PASS_MARCH = 3;
static constexpr int PERF_PASS_FXAA = 4;
static constexpr int PERF_FPS_SAMPLES = 8;

static char s_perf_text[SDF_PERF_BUF_SIZE] = "";
static bool s_perf_active = false;

/* Static atlas state */
static gpu::Texture *s_compact_atlas = nullptr;
static gpu::Texture *s_indirection = nullptr;

static float s_voxel_size = 0.0f;
static float3 s_atlas_origin = float3(0);
static float3 s_atlas_extent = float3(0);
static int3 s_grid_resolution = int3(0);
static int s_bricks_per_axis = 0;
static int s_object_count = 0;
static gpu::StorageBuf *s_object_ssbo = nullptr;
static gpu::StorageBuf *s_modifier_ssbo = nullptr;
static gpu::StorageBuf *s_group_ssbo = nullptr;
static int s_group_count = 0;
/* Depsgraph-order -> sorted-order index map. */
static Vector<int> s_depsgraph_to_sorted;

class Instance : public DrawEngine {
 private:
  Vector<SDFObjectGPU> objects_;
  Vector<SDFGroup *> object_group_ptrs_;
  Vector<Object *> object_ptrs_;

  float3 scene_min_ = float3(1e30f);
  float3 scene_max_ = float3(-1e30f);
  float3 padded_min_ = float3(0);
  float3 padded_max_ = float3(0);

  gpu::Texture *indirection_tx_ = nullptr;
  gpu::Texture *compact_atlas_tx_ = nullptr;
  gpu::StorageBuf *brick_counter_ = nullptr;
  gpu::StorageBuf *active_bricks_ = nullptr;
  int active_bricks_capacity_ = 0;

  float3 atlas_origin_ = float3(0.0f);
  float3 atlas_extent_ = float3(1.0f);
  float voxel_size_ = 1.0f / 256.0f;

  int3 grid_res_ = int3(32);
  int sdf_resolution_ = 128;
  int active_brick_count_ = 0;
  int prev_active_brick_count_ = 0;
  int atlas_capacity_ = 0;
  int bricks_per_axis_ = 1;
  int debug_grid_ = 0;
  bool fxaa_enabled_ = true;
  float surface_margin_ = 1.0f;
  float max_blend_ = 0.0f;
  float max_shell_distance_ = 0.0f;

  uint64_t scene_hash_ = 0;
  uint64_t grid_hash_ = 0;
  uint64_t spatial_hash_ = 0;
  bool needs_bake_ = true;
  bool needs_upload_ = true;
  bool needs_bvh_rebuild_ = true;
  bool depth_mode_ = false;

  /* Incremental rebake state. */
  Vector<uint64_t> prev_object_hashes_;
  Vector<float4> prev_bbox_mins_;
  Vector<float4> prev_bbox_maxs_;
  int prev_object_count_ = 0;
  float3 prev_atlas_origin_ = float3(0);
  int3 prev_grid_res_ = int3(0);
  float prev_voxel_size_ = 0.0f;
  uint64_t prev_group_structure_hash_ = 0;
  float prev_max_blend_ = 0.0f;
  float prev_max_shell_distance_ = 0.0f;
  bool prev_had_grid_objects_ = false;
  bool incremental_bake_ = false;
  int3 dirty_brick_min_ = int3(0);
  int3 dirty_brick_max_ = int3(0);
  int total_allocated_slots_ = 0;

  struct PendingGridObject {
    const Object *ob;
  };
  Vector<PendingGridObject> pending_grid_objects_;

  struct GridObject {
    gpu::Texture *texture = nullptr; /* R32F 3D texture */
    float4x4 world_to_texture;       /* Combined transform: world -> [0,1]^3 */
    float4 color;                    /* Object display color */
    float blend;                     /* Blend amount from modifier */
    int blend_type;                  /* Blend function type (eSDFBlendType) */
    int csg_operation;               /* CSG operation (eSDFCSGOperation) */
    float shell_distance;            /* Shell/extrusion thickness */
    int shell_mode;                  /* Shell sub-mode (eSDFShellMode) */
  };
  Vector<GridObject> grid_objects_;

  enum ShaderIndex {
    SH_CLASSIFY = 0,
    SH_AUGMENT_GRIDS,
    SH_BAKE,
    SH_MARCH,
    SH_GRID_BLEND,
    SH_FXAA,
    SH_COUNT,
  };

  static constexpr const char *shader_info_names_[SH_COUNT] = {
      "sdf_classify",
      "sdf_augment_grids",
      "sdf_bake",
      "sdf_march",
      "sdf_grid_blend",
      "sdf_fxaa",
  };

  gpu::Shader *shaders_[SH_COUNT] = {};

  gpu::Shader *&classify_sh_ = shaders_[SH_CLASSIFY];
  gpu::Shader *&augment_grids_sh_ = shaders_[SH_AUGMENT_GRIDS];
  gpu::Shader *&bake_sh_ = shaders_[SH_BAKE];
  gpu::Shader *&march_sh_ = shaders_[SH_MARCH];
  gpu::Shader *&grid_blend_sh_ = shaders_[SH_GRID_BLEND];
  gpu::Shader *&fxaa_sh_ = shaders_[SH_FXAA];

  /** Async shader compilation. */
  BatchHandle shader_compile_batch_ = 0;
  bool shaders_compiled_ = false;

  /** FXAA offscreen target: march renders here, then FXAA composites to default FB. */
  gpu::Texture *march_color_tx_ = nullptr;
  gpu::FrameBuffer *march_fb_ = nullptr;
  int2 fxaa_viewport_size_ = int2(0);

  /** Object SSBO. */
  gpu::StorageBuf *object_ssbo_ = nullptr;
  int object_ssbo_count_ = 0;

  /** Modifier data for GPU upload. */
  Vector<SDFModifierGPU> modifiers_;
  gpu::StorageBuf *modifier_ssbo_ = nullptr;
  int modifier_ssbo_count_ = 0;

  /** Group data for GPU upload. */
  Vector<SDFGroupGPU> groups_gpu_;
  gpu::StorageBuf *group_ssbo_ = nullptr;
  int group_ssbo_count_ = 0;

  /** BVH for object AABB culling on GPU. */
  Vector<BVHNodeGPU> bvh_nodes_;
  gpu::StorageBuf *bvh_ssbo_ = nullptr;
  int bvh_ssbo_count_ = 0;

  /** Per-object dirty flags for incremental bake (1=changed, 0=clean). */
  Vector<int> dirty_flags_;
  gpu::StorageBuf *dirty_flags_ssbo_ = nullptr;
  int dirty_flags_ssbo_count_ = 0;

  /** Fullscreen triangle batch (cached). */
  gpu::Batch *fullscreen_batch_ = nullptr;

  /** Debug grid wireframe batch (GPU_PRIM_LINES). */
  gpu::Batch *grid_batch_ = nullptr;
  int grid_batch_mode_ = 0;
  int3 grid_batch_res_ = int3(0);

  /** Debug BVH wireframe batch (GPU_PRIM_LINES, per-vertex color). */
  gpu::Batch *bvh_batch_ = nullptr;

  /** Debug scene bounds wireframe batch (GPU_PRIM_LINES, per-vertex color). */
  gpu::Batch *scene_bounds_batch_ = nullptr;

  const DRWContext *draw_ctx_ = nullptr;

  /** Matcap / shading state. */
  gpu::Texture *matcap_tx_ = nullptr;
  std::string current_matcap_;
  int lighting_type_ = V3D_LIGHTING_STUDIO;
  int use_specular_ = 0;
  int use_matcap_flip_ = 0;

  /** Studio light data (4 directional lights + ambient). */
  float4 studio_light_dir_[4] = {};
  float4 studio_light_col_[4] = {};
  float4 studio_light_spec_[4] = {};
  float3 studio_ambient_ = float3(0.0f);

  /** Performance overlay state. */
  bool perf_enabled_ = false;
  bool perf_queries_created_ = false;
  /** Per-pass wall-clock start timestamps (set by perf_begin_pass). */
  double perf_pass_start_[PERF_PASS_COUNT] = {};
  /** Which passes were active this frame. */
  bool perf_pass_active_[PERF_PASS_COUNT] = {};
  /** Whether the current frame is baking. */
  bool perf_currently_baking_ = false;
  /** Whether the DISPLAYED data is from a bake frame. */
  bool perf_prev_baked_ = false;
  /** Timing results (from previous frame's queries). */
  double perf_classify_ms_ = 0.0;
  double perf_bake_ms_ = 0.0;
  double perf_grid_ms_ = 0.0;
  double perf_march_ms_ = 0.0;
  double perf_fxaa_ms_ = 0.0;
  double perf_frame_ms_ = 0.0;
  double perf_fps_ = 0.0;
  /** Persistent last-bake timing (updated only when a bake frame is read). */
  double perf_last_classify_ms_ = 0.0;
  double perf_last_bake_ms_ = 0.0;
  double perf_last_grid_ms_ = 0.0;
  /** Whether we have ever read valid bake timing data. */
  bool perf_has_bake_data_ = false;
  /** Wall-clock time of last draw() call for FPS. */
  double perf_last_draw_time_ = 0.0;
  /** Circular buffer for FPS smoothing. */
  double perf_fps_samples_[PERF_FPS_SAMPLES] = {};
  int perf_fps_sample_idx_ = 0;
  int perf_fps_sample_count_ = 0;

 public:
  Instance()
  {
    if (G.debug & G_DEBUG_GPU) {
      printf("[SDF] engine instance created (default res=%d)\n", sdf_resolution_);
    }
  }

  blender::StringRefNull name_get() final
  {
    return "SDF";
  }

  void init() final
  {
    draw_ctx_ = DRW_context_get();

    /* Read SDF settings early (before end_sync computes grid params and hash).
     * Previously this ran in draw(), after end_sync had already computed the
     * grid using stale sdf_resolution_ / surface_margin_ defaults. */
    sync_sdf_settings();

    /* Kick off async shader compilation on first init so the GPU backend can
     * compile in a subprocess while we do sync work (object_sync, end_sync).
     * By the time draw() calls ensure_shaders(), binaries are often ready. */
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
    /* In depth mode, preserve all cached state so depth passes don't
     * invalidate the bake cache with a different object subset. */
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
    pending_grid_objects_.clear();
    scene_min_ = float3(1e30f);
    scene_max_ = float3(-1e30f);
    max_blend_ = 0.0f;
    max_shell_distance_ = 0.0f;
  }

  void object_sync(ObjectRef &ob_ref, Manager & /*manager*/) final
  {
    if (depth_mode_) {
      return;
    }

    Object *ob = ob_ref.object;

    /* Detect mesh objects with volume grids (from "Mesh to SDF Grid" modifier). */
    if (ob->type == OB_MESH && ob->runtime && ob->runtime->geometry_set_eval) {
      const bke::GeometrySet &geo = *ob->runtime->geometry_set_eval;
      if (geo.has_volume()) {
        const Volume *volume = geo.get_volume();
        if (volume && BKE_volume_num_grids(volume) > 0) {
          pending_grid_objects_.append({ob});

          /* Accumulate AABB from the object's local bounds transformed to world. */
          const std::optional<Bounds<float3>> bounds = BKE_object_boundbox_get(ob);
          if (bounds) {
            /* Transform all 8 corners of the local AABB to world space. */
            const float3 &lo = bounds->min;
            const float3 &hi = bounds->max;
            for (int c = 0; c < 8; c++) {
              float3 corner = float3(
                  (c & 1) ? hi.x : lo.x, (c & 2) ? hi.y : lo.y, (c & 4) ? hi.z : lo.z);
              float3 wc = math::transform_point(ob->object_to_world(), corner);
              scene_min_ = math::min(scene_min_, wc);
              scene_max_ = math::max(scene_max_, wc);
            }
          }
        }
      }
    }

    if (ob->type != OB_SDF) {
      return;
    }

    const SDF *sdf_data = static_cast<const SDF *>(ob->data);
    if (sdf_data == nullptr) {
      return;
    }

    /* Decompose object_to_world into rotation + scale.
     * Scale is baked into sdf_size; inverse matrix is rotation-only.
     * If the SDF belongs to a group with a non-identity transform, compose it first. */
    const float4x4 &mat = ob->object_to_world();

    float3 scale;
    scale.x = math::length(float3(mat[0]));
    scale.y = math::length(float3(mat[1]));
    scale.z = math::length(float3(mat[2]));

    /* Build rotation-only matrix. */
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

    /* Pack object data. */
    SDFObjectGPU gpu_obj = {};
    gpu_obj.inverse_matrix = inv_rot;
    gpu_obj.position = float4(mat[3].x, mat[3].y, mat[3].z, 0.0f);

    /* Bake scale into SDF size. */
    gpu_obj.sdf_size = float4(sdf_data->size[0] * scale.x,
                              sdf_data->size[1] * scale.y,
                              sdf_data->size[2] * scale.z,
                              0.0f);

    /* Compute bevel from modifier stack (sum of all enabled bevel modifiers). */
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

    /* Group membership: store pointer for resolution in end_sync.
     * group_id is set to -1 here and resolved after groups_gpu_ is built. */
    gpu_obj.group_id = -1;
    gpu_obj.group_first = 0;
    gpu_obj.group_order = sdf_data->group_order;
    gpu_obj.original_index = int(objects_.size());
    object_group_ptrs_.append(sdf_data->sdf_group);
    object_ptrs_.append(ob);

    gpu_obj.color = float4(
        sdf_data->color[0], sdf_data->color[1], sdf_data->color[2], sdf_data->color[3]);

    /* Shape-specific data (box, ngon, torus share GPU struct fields). */
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
      /* Precompute sin/cos of half-angle for capped torus. */
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

    /* Pack modifiers for this object. */
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

    /* Compute shape-specific local extent, then add blend/shell padding.
     * Each primitive has different actual extents from its size parameters. */
    float shell_expand = (sdf_data->csg_operation == SDF_CSG_SHELL) ?
                             fabsf(sdf_data->shell_distance) :
                             0.0f;
    float pad = sdf_data->blend + shell_expand;
    float3 local_extent;
    float3 sz = float3(gpu_obj.sdf_size);
    switch (sdf_data->sdf_type) {
      case SDF_TYPE_CAPSULE: {
        /* Capsule: radius sz.x in XY, half-height sz.y along Z, caps add radius. */
        float r = sz.x;
        float h = math::max(sz.y - bevel, 0.0f);
        local_extent = float3(r + pad, r + pad, h + r + pad);
        break;
      }
      case SDF_TYPE_CYLINDER: {
        /* Elliptical cylinder: XY radii + bevel + pad, Z half-height + bevel + pad. */
        local_extent = sz + float3(bevel + pad);
        break;
      }
      case SDF_TYPE_CONE: {
        /* Cone: base radius sz.x in XY, half-height sz.y along Z. */
        float r = sz.x;
        float h = sz.y;
        local_extent = float3(r + bevel + pad, r + bevel + pad, h + bevel + pad);
        break;
      }
      case SDF_TYPE_TORUS: {
        /* Torus: major radius sz.x in XY, minor sz.y is tube radius along Z. */
        float outer = sz.x + sz.y;
        local_extent = float3(outer + pad, outer + pad, sz.y + pad);
        break;
      }
      case SDF_TYPE_NGON: {
        /* N-Gon: circumradius sz.x in XY, half-height sz.z along Z. */
        float r = sz.x;
        local_extent = float3(r + bevel + pad, r + bevel + pad, sz.z + bevel + pad);
        break;
      }
      default:
        /* Box / Sphere: symmetric extent = size + bevel + pad. */
        local_extent = sz + float3(bevel + pad);
        break;
    }

    /* Expand AABB for domain modifiers. */
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
            /* Also check arc apex when bend exceeds 90 degrees. */
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

    /* Transform local AABB corners to world to get tight world AABB. */
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

    /* Intersection objects must be evaluated everywhere: outside the intersector
     * the result should be "outside" (large positive), clipping the base geometry.
     * Expand their AABB to cover the entire scene so they're always BVH candidates. */
    if (sdf_data->csg_operation == SDF_CSG_INTERSECT) {
      gpu_obj.bbox_min = float4(-1e10f, -1e10f, -1e10f, 0.0f);
      gpu_obj.bbox_max = float4(1e10f, 1e10f, 1e10f, 0.0f);
    }
    else {
      gpu_obj.bbox_min = float4(world_min, 0.0f);
      gpu_obj.bbox_max = float4(world_max, 0.0f);
    }

    /* Rotation-invariant scene extent: circumscribed sphere of local AABB.
     * radius = length(local_extent) is the tightest rotation-invariant bound.
     * Grid resolution stays constant regardless of object rotation. */
    float sphere_radius = math::length(local_extent);
    float3 obj_center = float3(mat[3].x, mat[3].y, mat[3].z);
    scene_min_ = math::min(scene_min_, obj_center - float3(sphere_radius));
    scene_max_ = math::max(scene_max_, obj_center + float3(sphere_radius));

    max_blend_ = math::max(max_blend_, gpu_obj.blend);
    if (sdf_data->csg_operation == SDF_CSG_SHELL) {
      max_shell_distance_ = math::max(max_shell_distance_, fabsf(sdf_data->shell_distance));
    }

    objects_.append(gpu_obj);
  }

  void end_sync() final
  {
    /* In depth mode, reuse cached atlas — don't recompute anything. */
    if (depth_mode_) {
      needs_bake_ = false;
      return;
    }

    /* Compute a lightweight hash from pending grid objects BEFORE doing the
     * expensive dense-float extraction. This lets us skip grid processing
     * entirely when nothing changed (saves 40ms+ per frame at fine voxel sizes).
     *
     * Hash includes: object count, transforms, bounds (changes with voxel_size),
     * color, and blend from modifier. */
    uint64_t grid_hash = uint64_t(pending_grid_objects_.size()) * 997;
    for (const PendingGridObject &pending : pending_grid_objects_) {
      const Object *ob = pending.ob;
      const float4x4 &mat = ob->object_to_world();

      /* Hash object transform (covers move/rotate/scale). */
      for (int i = 0; i < 4; i++) {
        grid_hash = grid_hash * 6364136223846793005ULL + float_as_uint(mat[i][0]);
        grid_hash = grid_hash * 6364136223846793005ULL + float_as_uint(mat[i][1]);
        grid_hash = grid_hash * 6364136223846793005ULL + float_as_uint(mat[i][2]);
      }
      /* Hash bounds (changes when voxel_size or mesh changes). */
      const std::optional<Bounds<float3>> bounds = BKE_object_boundbox_get(ob);
      if (bounds) {
        grid_hash = grid_hash * 6364136223846793005ULL + float_as_uint(bounds->min.x);
        grid_hash = grid_hash * 6364136223846793005ULL + float_as_uint(bounds->max.x);
        grid_hash = grid_hash * 6364136223846793005ULL + float_as_uint(bounds->min.y);
        grid_hash = grid_hash * 6364136223846793005ULL + float_as_uint(bounds->max.y);
      }
      /* Hash object color. */
      grid_hash = grid_hash * 6364136223846793005ULL + float_as_uint(ob->color[0]);
      grid_hash = grid_hash * 6364136223846793005ULL + float_as_uint(ob->color[1]);
      /* Hash session UID (detect object add/remove/replace). */
      grid_hash = grid_hash * 6364136223846793005ULL + uint64_t(ob->id.session_uid);
    }

    bool grids_changed = (grid_hash != grid_hash_);
    grid_hash_ = grid_hash;

    if (grids_changed) {
      /* Grid data changed: re-extract dense floats and create GPU textures. */
      free_grid_objects();
      for (const PendingGridObject &pending : pending_grid_objects_) {
        process_grid_object(pending.ob);
      }
    }

    if (objects_.is_empty() && grid_objects_.is_empty()) {
      needs_bake_ = false;
      return;
    }

    /* Voxel size defines the resolution. To maintain consistent visual quality
     * with older versions, we calculate voxel_size using a fixed legacy margin 
     * (max(scene_size * 0.5, 2.0)). This prevents the object resolution from 
     * randomly jumping when the user adds/removes objects. */
    float3 scene_size = scene_max_ - scene_min_;
    float legacy_margin = math::max(math::reduce_max(scene_size) * 0.5f, 2.0f);
    float legacy_max_extent = math::reduce_max(scene_size) + 2.0f * legacy_margin;
    voxel_size_ = legacy_max_extent / float(sdf_resolution_);

    /* But we tightly bound the actual grid to the physical SDF limits!
     * This eliminates hundreds of millions of empty padding bricks from the
     * indirection grid, massively speeding up classify. */
    float brick_world = float(SDF_BRICK_SIZE) * voxel_size_;
    float tight_margin = max_blend_ + max_shell_distance_ + brick_world;
    float3 padded_min = scene_min_ - float3(tight_margin);
    float3 padded_max = scene_max_ + float3(tight_margin);
    padded_min_ = padded_min;
    padded_max_ = padded_max;
    float max_extent = math::reduce_max(padded_max - padded_min);
    voxel_size_ = max_extent / float(sdf_resolution_);

    float chunk_size = float(SDF_BRICK_SIZE) * voxel_size_;

    /* Snap to world-aligned chunk boundaries. */
    float3 grid_min = math::floor(padded_min / chunk_size) * chunk_size;
    float3 grid_max = math::ceil(padded_max / chunk_size) * chunk_size;

    /* Per-axis grid resolution in bricks. */
    int3 new_grid_res = int3(math::round((grid_max - grid_min) / chunk_size));
    new_grid_res = math::clamp(new_grid_res, int3(1), int3(SDF_MAX_GRID_RES));

    atlas_origin_ = grid_min;
    atlas_extent_ = grid_max - grid_min;

    /* Lazy reallocation: only free textures when the new grid EXCEEDS the
     * allocated size. Shrinking reuses existing textures (larger than needed
     * but avoids the lag spike from GPU texture reallocation). */
    if (new_grid_res != grid_res_) {
      bool needs_realloc = false;
      if (indirection_tx_) {
        if (new_grid_res.x > grid_res_.x || new_grid_res.y > grid_res_.y ||
            new_grid_res.z > grid_res_.z)
        {
          needs_realloc = true;
        }
      }
      else {
        needs_realloc = true;
      }

      if (needs_realloc) {
        if (indirection_tx_) {
          GPU_texture_free(indirection_tx_);
          indirection_tx_ = nullptr;
        }
        if (compact_atlas_tx_) {
          GPU_texture_free(compact_atlas_tx_);
          compact_atlas_tx_ = nullptr;
        }
      }
      grid_res_ = new_grid_res;
      atlas_capacity_ = 0;
      prev_active_brick_count_ = 0;
    }

    /* Build group GPU data from Main's sdf_groups list.
     * Also build maps to resolve per-object group_id and group_order values
     * directly from the original group member lists (not from evaluated SDF
     * copies, which may have stale group_order values). */
    {
      Main *bmain = DEG_get_bmain(draw_ctx_->depsgraph);

      /* Clean up zombie group members (object was deleted but member struct
       * was left behind by the remap system).  Without this, thousands of
       * null-pointer entries accumulate and waste memory + iteration time. */
      LISTBASE_FOREACH (SDFGroup *, group, &bmain->sdf_groups) {
        BKE_sdf_group_cleanup_null_members(group);
      }

      groups_gpu_.clear();
      Map<SDFGroup *, int> group_index_map;

      /* Map from original Object* → (group_id, group_order).
       * Built by iterating the original group member lists, so the order
       * always matches the outliner display order. */
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
        gpu_grp.color = float4(group->color[0], group->color[1], group->color[2], group->color[3]);
        groups_gpu_.append(gpu_grp);
        group_index_map.add(group, g_idx);

        /* Record each member's (group_id, group_order) from the original list order. */
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

      /* Resolve group_id and group_order on objects using the membership map.
       * Uses the original Object pointer (via DEG_get_original) to look up
       * the correct order from the original group member list. */
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

      /* Sort objects so that within each group, members appear in ascending
       * group_order. This ensures the GPU evaluates CSG operations in the
       * correct user-specified order. Ungrouped objects are placed after
       * all grouped objects in their original relative order. */
      {
        const int n = int(objects_.size());
        if (n > 0) {
          /* Build (sort_key, original_index) pairs for stable sort. */
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

          /* Reorder objects and build old→new index mapping. */
          Vector<SDFObjectGPU> sorted(n);
          Vector<int> old_to_new(n);
          for (int new_idx = 0; new_idx < n; new_idx++) {
            int old_idx = sort_pairs[new_idx].second;
            sorted[new_idx] = objects_[old_idx];
            old_to_new[old_idx] = new_idx;
          }
          objects_ = std::move(sorted);

          /* Export depsgraph→sorted mapping for overlay outline code. */
          s_depsgraph_to_sorted = std::move(old_to_new);
        }
      }

      /* Resolve group_first and fix first_object/object_count in groups_gpu_.
       * After sorting, group members are contiguous and in group_order,
       * so the first encountered member per group is the base shape. */
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

    }

    /* Compute tighter per-object blend-aware AABB expansions.
     * We iterate backwards. Each object/group maintains its expanded AABB.
     * An object's AABB is expanded by its own intrinsic blend radius,
     * plus the intersection of its blend-expanded bounds with the bounds
     * of any subsequent operation that blends with it.
     * This avoids the O(N^2) volume blowup, keeping BVH culling efficient. */
    Vector<float3> new_mins(objects_.size());
    Vector<float3> new_maxs(objects_.size());
    for (int i = 0; i < int(objects_.size()); i++) {
      float blend_radius = (objects_[i].blend_type == 0 /* SDF_BLEND_LINEAR */) ? 0.0f : objects_[i].blend;
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

    /* 1. Ungrouped objects (evaluated last). */
    for (int i = int(objects_.size()) - 1; i >= ungrouped_start; i--) {
      SDFObjectGPU &obj = objects_[i];

      for (int j = i + 1; j < int(objects_.size()); j++) {
        float sub_blend = objects_[j].blend + fabsf(objects_[j].shell_distance);
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

    /* Group bounding boxes. */
    Vector<float3> group_mins(groups_gpu_.size(), float3(1e10f));
    Vector<float3> group_maxs(groups_gpu_.size(), float3(-1e10f));

    /* 2. Groups (evaluated in order from 0 to group_count - 1). */
    for (int g = int(groups_gpu_.size()) - 1; g >= 0; g--) {
      SDFGroupGPU &grp = groups_gpu_[g];
      int start_idx = grp.first_object;
      int end_idx = grp.first_object + grp.object_count - 1;

      if (grp.object_count > 0 && start_idx >= 0 && end_idx < int(objects_.size())) {
        for (int i = end_idx; i >= start_idx; i--) {
          SDFObjectGPU &obj = objects_[i];

          /* Blend with subsequent objects IN THE SAME GROUP. */
          for (int j = i + 1; j <= end_idx; j++) {
            float sub_blend_radius = (objects_[j].blend_type == 0 /* SDF_BLEND_LINEAR */) ? 0.0f : objects_[j].blend;
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

          /* Blend with subsequent groups. */
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

          /* Blend with ungrouped objects (evaluated after all groups). */
          for (int j = ungrouped_start; j < int(objects_.size()); j++) {
            float sub_blend_radius = (objects_[j].blend_type == 0 /* SDF_BLEND_LINEAR */) ? 0.0f : objects_[j].blend;
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

          /* Update this group's bounding box. */
          group_mins[g] = math::min(group_mins[g], new_mins[i]);
          group_maxs[g] = math::max(group_maxs[g], new_maxs[i]);
        }
      }
    }

    /* Finally, write the new AABBs back. */
    for (int i = 0; i < int(objects_.size()); i++) {
      objects_[i].bbox_min = float4(new_mins[i], 0.0f);
      objects_[i].bbox_max = float4(new_maxs[i], 0.0f);
    }

    /* ---- Incremental rebake detection ----
     * Compute per-object hashes AFTER sorting and AABB expansion so that
     * any change (intrinsic property, group membership, expansion) is captured.
     * Then compare with previous frame to find dirty objects and compute
     * the minimal brick region that needs reclassification and rebaking. */
    {
      const int n = int(objects_.size());
      Vector<uint64_t> current_hashes(n);
      for (int i = 0; i < n; i++) {
        current_hashes[i] = compute_object_hash(i);
      }

      /* Compute group structure hash (membership, order, group-level params). */
      uint64_t group_struct_hash = uint64_t(groups_gpu_.size()) * 997;
      for (const SDFGroupGPU &grp : groups_gpu_) {
        group_struct_hash = group_struct_hash * 6364136223846793005ULL + uint64_t(grp.csg_operation);
        group_struct_hash = group_struct_hash * 6364136223846793005ULL + uint64_t(grp.blend_type);
        group_struct_hash = group_struct_hash * 6364136223846793005ULL + float_as_uint(grp.blend);
        group_struct_hash =
            group_struct_hash * 6364136223846793005ULL + float_as_uint(grp.shell_distance);
        group_struct_hash = group_struct_hash * 6364136223846793005ULL + uint64_t(grp.shell_mode);
        group_struct_hash = group_struct_hash * 6364136223846793005ULL + uint64_t(grp.first_object);
        group_struct_hash = group_struct_hash * 6364136223846793005ULL + uint64_t(grp.object_count);
        group_struct_hash = group_struct_hash * 6364136223846793005ULL + float_as_uint(grp.color.x);
        group_struct_hash = group_struct_hash * 6364136223846793005ULL + float_as_uint(grp.color.y);
        group_struct_hash = group_struct_hash * 6364136223846793005ULL + float_as_uint(grp.color.z);
        group_struct_hash = group_struct_hash * 6364136223846793005ULL + float_as_uint(grp.color.w);
      }

      incremental_bake_ = false;
      dirty_flags_.reinitialize(n);
      dirty_flags_.fill(0);

      /* Incremental is possible when the grid geometry and structure are stable
       * and only a subset of objects changed their intrinsic properties. */
      if (prev_object_count_ == n &&                     /* Same object count. */
          prev_atlas_origin_ == atlas_origin_ &&          /* Grid didn't shift. */
          prev_grid_res_ == grid_res_ &&                  /* Grid didn't resize. */
          prev_voxel_size_ == voxel_size_ &&              /* Voxel density unchanged. */
          prev_group_structure_hash_ == group_struct_hash && /* Group structure stable. */
          !grids_changed &&                               /* No grid object changes. */
          grid_objects_.is_empty() &&                      /* No grid objects this frame. */
          !prev_had_grid_objects_ &&                       /* No grid objects last frame. */
          compact_atlas_tx_ != nullptr &&                  /* Atlas exists. */
          indirection_tx_ != nullptr &&                    /* Indirection exists. */
          total_allocated_slots_ > 0)                      /* Had a previous bake. */
      {
        /* Find which objects changed. */
        Vector<int> dirty_indices;
        for (int i = 0; i < n; i++) {
          if (current_hashes[i] != prev_object_hashes_[i]) {
            dirty_indices.append(i);
            dirty_flags_[i] = 1;
          }
        }

        if (!dirty_indices.is_empty() && dirty_indices.size() < n) {
          /* Compute dirty AABB: union of old+new AABBs for all dirty objects. */
          float3 dirty_min = float3(1e30f);
          float3 dirty_max = float3(-1e30f);

          for (int idx : dirty_indices) {
            dirty_min = math::min(dirty_min, float3(objects_[idx].bbox_min));
            dirty_max = math::max(dirty_max, float3(objects_[idx].bbox_max));
            dirty_min = math::min(dirty_min, float3(prev_bbox_mins_[idx]));
            dirty_max = math::max(dirty_max, float3(prev_bbox_maxs_[idx]));
          }

          /* Convert to brick coordinates. */
          float chunk = float(SDF_BRICK_SIZE) * voxel_size_;
          dirty_brick_min_ = int3(math::floor((dirty_min - atlas_origin_) / chunk));
          dirty_brick_max_ = int3(math::ceil((dirty_max - atlas_origin_) / chunk));

          dirty_brick_min_ = math::clamp(dirty_brick_min_, int3(0), grid_res_);
          dirty_brick_max_ = math::clamp(dirty_brick_max_, int3(0), grid_res_);

          int3 dirty_size = dirty_brick_max_ - dirty_brick_min_;
          int dirty_volume = dirty_size.x * dirty_size.y * dirty_size.z;

          /* Per-brick dirty flags let the shaders skip clean bricks even
           * when the dirty AABB is large, so no volume threshold needed.
           * Only check atlas capacity and fragmentation. */
          if (dirty_volume >= 0) {
            int max_atlas_slots = bricks_per_axis_ * bricks_per_axis_ * bricks_per_axis_;
            if (total_allocated_slots_ + dirty_volume <= max_atlas_slots) {
              if (total_allocated_slots_ <= prev_active_brick_count_ * 3 ||
                  prev_active_brick_count_ == 0 || dirty_volume == 0)
              {
                incremental_bake_ = true;

                if (G.debug & G_DEBUG_GPU) {
                  printf(
                      "[SDF] incremental rebake: %d dirty objects, dirty region "
                      "[%d,%d,%d]-[%d,%d,%d] (%d bricks)\n",
                      int(dirty_indices.size()),
                      dirty_brick_min_.x,
                      dirty_brick_min_.y,
                      dirty_brick_min_.z,
                      dirty_brick_max_.x,
                      dirty_brick_max_.y,
                      dirty_brick_max_.z,
                      dirty_volume);
                }
              }
            }
          }
        }
      }

      /* Save non-expanded AABBs for next frame's comparison. */
      Vector<float4> cur_mins(n);
      Vector<float4> cur_maxs(n);
      for (int i = 0; i < n; i++) {
        cur_mins[i] = objects_[i].bbox_min;
        cur_maxs[i] = objects_[i].bbox_max;
      }

      /* Expand dirty objects' AABBs to cover old positions so the BVH
       * routes bricks near the previous position to the moved object.
       * Without this, bricks at the old position wouldn't detect the
       * object moved away and would keep stale atlas data. */
      if (incremental_bake_) {
        for (int i = 0; i < n; i++) {
          if (dirty_flags_[i]) {
            objects_[i].bbox_min = float4(
                math::min(float3(objects_[i].bbox_min), float3(prev_bbox_mins_[i])), 0);
            objects_[i].bbox_max = float4(
                math::max(float3(objects_[i].bbox_max), float3(prev_bbox_maxs_[i])), 0);
          }
        }
      }

      prev_object_hashes_ = std::move(current_hashes);
      prev_bbox_mins_ = std::move(cur_mins);
      prev_bbox_maxs_ = std::move(cur_maxs);
      prev_object_count_ = n;
      prev_atlas_origin_ = atlas_origin_;
      prev_grid_res_ = grid_res_;
      prev_voxel_size_ = voxel_size_;
      prev_group_structure_hash_ = group_struct_hash;
      prev_max_blend_ = max_blend_;
      prev_max_shell_distance_ = max_shell_distance_;
      prev_had_grid_objects_ = !grid_objects_.is_empty();
    }

    /* Scene hash: reuse per-object + group hashes already computed above. */
    uint64_t hash = uint64_t(objects_.size()) + grid_hash;
    for (int i = 0; i < int(prev_object_hashes_.size()); i++) {
      hash = hash * 6364136223846793005ULL + prev_object_hashes_[i];
    }
    hash = hash * 6364136223846793005ULL + prev_group_structure_hash_;
    hash = hash * 6364136223846793005ULL + float_as_uint(surface_margin_);
    hash = hash * 6364136223846793005ULL + float_as_uint(atlas_origin_.x);
    hash = hash * 6364136223846793005ULL + float_as_uint(atlas_origin_.y);
    hash = hash * 6364136223846793005ULL + float_as_uint(atlas_origin_.z);
    hash = hash * 6364136223846793005ULL + uint64_t(grid_res_.x);
    hash = hash * 6364136223846793005ULL + uint64_t(grid_res_.y);
    hash = hash * 6364136223846793005ULL + uint64_t(grid_res_.z);

    /* Spatial hash: only positions, sizes, and object count.
     * BVH topology depends on centroids, so only rebuild when these change. */
    uint64_t sp_hash = uint64_t(objects_.size());
    for (const SDFObjectGPU &obj : objects_) {
      sp_hash = sp_hash * 6364136223846793005ULL + float_as_uint(obj.position.x);
      sp_hash = sp_hash * 6364136223846793005ULL + float_as_uint(obj.position.y);
      sp_hash = sp_hash * 6364136223846793005ULL + float_as_uint(obj.position.z);
      sp_hash = sp_hash * 6364136223846793005ULL + float_as_uint(obj.sdf_size.x);
      sp_hash = sp_hash * 6364136223846793005ULL + float_as_uint(obj.sdf_size.y);
      sp_hash = sp_hash * 6364136223846793005ULL + float_as_uint(obj.sdf_size.z);
    }

    if (hash != scene_hash_) {
      if (G.debug & G_DEBUG_GPU) {
        printf("[SDF] scene hash changed: %016llx -> %016llx  (objs=%d grids=%d res=%d)\n",
               (unsigned long long)scene_hash_,
               (unsigned long long)hash,
               int(objects_.size()),
               int(grid_objects_.size()),
               sdf_resolution_);
      }
      scene_hash_ = hash;
      needs_bake_ = true;
      needs_upload_ = true;
      needs_bvh_rebuild_ = (sp_hash != spatial_hash_);
      spatial_hash_ = sp_hash;
    }
    else {
      needs_bake_ = false;
    }

  }

  void draw(Manager & /*manager*/) final
  {
    if (objects_.is_empty() && grid_objects_.is_empty()) {
      return;
    }

    /* In depth mode, skip bake — only march with cached atlas for depth. */
    if (depth_mode_) {
      if (compact_atlas_tx_ == nullptr) {
        return; /* No atlas yet — nothing to march. */
      }
      sync_shading();
      ensure_shaders();
      if (march_sh_ == nullptr) {
        return;
      }
      DRW_submission_start();
      draw_march();
      DRW_submission_end();
      return;
    }

    sync_shading();
    ensure_shaders();

    perf_enabled_ = draw_ctx_->v3d && (draw_ctx_->v3d->overlay.flag & V3D_OVERLAY_SDF_PERF) &&
                    !(draw_ctx_->v3d->flag2 & V3D_HIDE_OVERLAYS);
    if (perf_enabled_) {
      perf_init();
      perf_begin_frame();
    }

    if (march_sh_ == nullptr) {
      return;
    }
    if (!objects_.is_empty() && (classify_sh_ == nullptr || bake_sh_ == nullptr)) {
      return;
    }
    if (!grid_objects_.is_empty() && grid_blend_sh_ == nullptr) {
      return;
    }
    DRW_submission_start();

    if (!objects_.is_empty() && needs_upload_) {
      upload_objects();
      if (needs_bvh_rebuild_) {
        build_bvh();
      }
      else if (!bvh_nodes_.is_empty()) {
        update_bvh_aabbs();
      }
      upload_bvh();
      needs_upload_ = false;
    }

    if (perf_enabled_) {
      for (int i = 0; i < PERF_PASS_COUNT; i++) {
        perf_pass_active_[i] = false;
      }
      perf_currently_baking_ = needs_bake_;
    }

    if (needs_bake_) {
      if (grid_batch_) {
        GPU_batch_discard(grid_batch_);
        grid_batch_ = nullptr;
      }
      if (bvh_batch_) {
        GPU_batch_discard(bvh_batch_);
        bvh_batch_ = nullptr;
      }
      if (scene_bounds_batch_) {
        GPU_batch_discard(scene_bounds_batch_);
        scene_bounds_batch_ = nullptr;
      }

      if (incremental_bake_) {
        /* Incremental pipeline. */
        if (perf_enabled_) {
          perf_begin_pass(PERF_PASS_CLASSIFY);
        }
        ensure_indirection();
        if (!objects_.is_empty()) {
          dispatch_classify();
        }
        if (perf_enabled_) {
          perf_end_pass(PERF_PASS_CLASSIFY);
        }

        if (perf_enabled_) {
          perf_begin_pass(PERF_PASS_BAKE);
        }
        if (!objects_.is_empty() && active_brick_count_ > 0) {
          dispatch_bake();
        }
        if (perf_enabled_) {
          perf_end_pass(PERF_PASS_BAKE);
        }

        if (perf_enabled_) {
          perf_begin_pass(PERF_PASS_GRID);
          perf_end_pass(PERF_PASS_GRID);
        }
      }
      else {
        /* Full pipeline. */
        if (perf_enabled_) {
          perf_begin_pass(PERF_PASS_CLASSIFY);
        }
        ensure_indirection();
        if (!objects_.is_empty()) {
          int32_t clear_val = -1;
          GPU_texture_clear(indirection_tx_, GPU_DATA_INT, &clear_val);
          dispatch_classify();
        }
        else {
          clear_indirection();
        }
        if (perf_enabled_) {
          perf_end_pass(PERF_PASS_CLASSIFY);
        }

        if (!grid_objects_.is_empty()) {
          augment_indirection_for_grids();
        }
        {
          int estimated = math::max(active_brick_count_, prev_active_brick_count_);
          int capacity = math::max(estimated + estimated / 2, 64);
          if (capacity > atlas_capacity_) {
            atlas_capacity_ = capacity;
          }
          else if (atlas_capacity_ > capacity * 2) {
            atlas_capacity_ = math::max(capacity, atlas_capacity_ / 2);
          }
          int new_bpa = int(std::ceil(std::cbrt(double(atlas_capacity_))));
          if (new_bpa < 1) {
            new_bpa = 1;
          }
          bricks_per_axis_ = new_bpa;
        }

        ensure_compact_atlas();

        clear_compact_atlas();

        if (perf_enabled_) {
          perf_begin_pass(PERF_PASS_BAKE);
        }
        if (!objects_.is_empty()) {
          dispatch_bake();

          if (active_brick_count_ > atlas_capacity_) {
            atlas_capacity_ = active_brick_count_ + active_brick_count_ / 2;
            int new_bpa = int(std::ceil(std::cbrt(double(atlas_capacity_))));
            if (new_bpa < 1) {
              new_bpa = 1;
            }
            bricks_per_axis_ = new_bpa;
            ensure_compact_atlas();
            clear_compact_atlas();
            dispatch_bake();
          }
          prev_active_brick_count_ = active_brick_count_;
        }
        if (perf_enabled_) {
          perf_end_pass(PERF_PASS_BAKE);
        }

        if (perf_enabled_) {
          perf_begin_pass(PERF_PASS_GRID);
        }
        if (!grid_objects_.is_empty()) {
          dispatch_grid_blends();
        }
        if (perf_enabled_) {
          perf_end_pass(PERF_PASS_GRID);
        }
      }
    }

    if (perf_enabled_) {
      perf_begin_pass(PERF_PASS_MARCH);
    }
    draw_march();
    if (perf_enabled_) {
      perf_end_pass(PERF_PASS_MARCH);
    }

    if (perf_enabled_) {
      perf_begin_pass(PERF_PASS_FXAA);
    }
    draw_fxaa();
    if (perf_enabled_) {
      perf_end_pass(PERF_PASS_FXAA);
    }

    if (!draw_ctx_->v3d || draw_ctx_->v3d->shading.type != OB_RENDER) {
      draw_debug_grid();
      draw_debug_scene_bounds();
      draw_debug_bvh();
    }

    DRW_submission_end();

    s_compact_atlas = compact_atlas_tx_;
    s_indirection = indirection_tx_;
    s_voxel_size = voxel_size_;
    s_atlas_origin = atlas_origin_;
    s_atlas_extent = atlas_extent_;
    s_grid_resolution = grid_res_;
    s_bricks_per_axis = bricks_per_axis_;
    s_object_count = int(objects_.size());
    s_object_ssbo = object_ssbo_;
    s_modifier_ssbo = modifier_ssbo_;
    s_group_ssbo = group_ssbo_;
    s_group_count = int(groups_gpu_.size());

    if (perf_enabled_) {
      perf_end_frame(needs_bake_);
    }
    else {
      s_perf_active = false;
    }
  }

 private:
  uint64_t compute_object_hash(int obj_idx) const
  {
    const SDFObjectGPU &obj = objects_[obj_idx];
    uint64_t h = 14695981039346656037ULL;
    auto mix = [&](uint64_t v) {
      h ^= v;
      h *= 1099511628211ULL;
    };
    mix(float_as_uint(obj.position.x));
    mix(float_as_uint(obj.position.y));
    mix(float_as_uint(obj.position.z));
    mix(float_as_uint(obj.sdf_size.x));
    mix(float_as_uint(obj.sdf_size.y));
    mix(float_as_uint(obj.sdf_size.z));
    mix(float_as_uint(obj.bevel));
    mix(float_as_uint(obj.blend));
    mix(uint64_t(obj.blend_type));
    mix(uint64_t(obj.sdf_type));
    mix(uint64_t(obj.csg_operation));
    mix(float_as_uint(obj.shell_distance));
    mix(uint64_t(obj.shell_mode));
    mix(float_as_uint(obj.color.x));
    mix(float_as_uint(obj.color.y));
    mix(float_as_uint(obj.color.z));
    mix(float_as_uint(obj.color.w));
    mix(float_as_uint(obj.box_corners.x));
    mix(float_as_uint(obj.box_corners.y));
    mix(float_as_uint(obj.box_corners.z));
    mix(float_as_uint(obj.box_corners.w));
    mix(float_as_uint(obj.box_edges.x));
    mix(float_as_uint(obj.box_edges.y));
    mix(float_as_uint(obj.box_edges.z));
    mix(float_as_uint(obj.box_edges.w));
    mix(uint64_t(obj.box_modes.x));
    mix(uint64_t(obj.box_modes.y));
    mix(uint64_t(obj.box_modes.z));
    mix(float_as_uint(obj.inverse_matrix[0][0]));
    mix(float_as_uint(obj.inverse_matrix[1][1]));
    mix(float_as_uint(obj.inverse_matrix[2][2]));
    mix(uint64_t(uint32_t(obj.group_id + 1)));
    mix(uint64_t(obj.group_first));
    mix(uint64_t(uint32_t(obj.group_order + 1)));
    for (int m = obj.modifier_start; m < obj.modifier_start + obj.modifier_count; m++) {
      if (m >= 0 && m < int(modifiers_.size())) {
        const SDFModifierGPU &mod = modifiers_[m];
        mix(uint64_t(mod.header.x));
        mix(uint64_t(mod.header.y));
        mix(float_as_uint(mod.params.x));
        mix(float_as_uint(mod.params.y));
        mix(float_as_uint(mod.params.z));
        mix(float_as_uint(mod.params.w));
        mix(float_as_uint(mod.params2.x));
        mix(float_as_uint(mod.params2.y));
        mix(float_as_uint(mod.params2.z));
        mix(float_as_uint(mod.params2.w));
      }
    }
    return h;
  }

  void sync_sdf_settings()
  {
    const View3D *v3d = draw_ctx_->v3d;
    if (v3d == nullptr) {
      return;
    }

    const View3DShading &shading = v3d->shading;

    int new_level = int(shading.sdf_resolution);
    if (new_level < 1 || new_level > 4) {
      new_level = 2; /* Default level 2 is 1024 */
    }
    int new_res = 256 << new_level;

    if (new_res != sdf_resolution_) {
      if (G.debug & G_DEBUG_GPU) {
        printf("[SDF] resolution changed: %d -> %d\n", sdf_resolution_, new_res);
      }
      sdf_resolution_ = new_res;
      needs_bake_ = true;
      needs_upload_ = true;
      scene_hash_ = 0;
      atlas_capacity_ = 0;
      prev_active_brick_count_ = 0;
    }

    debug_grid_ = int(shading.sdf_debug_grid);
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
      fxaa_viewport_size_ = int2(0);
    }
    fxaa_enabled_ = new_fxaa;

    int margin_pct = int(shading.sdf_surface_margin);
    if (margin_pct <= 0) {
      margin_pct = 100;
    }
    surface_margin_ = float(margin_pct) / 100.0f;
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

  void ensure_indirection()
  {
    if (indirection_tx_ != nullptr) {
      return;
    }
    eGPUTextureUsage usage = GPU_TEXTURE_USAGE_SHADER_READ | GPU_TEXTURE_USAGE_SHADER_WRITE;
    indirection_tx_ = GPU_texture_create_3d("sdf_indirection",
                                            grid_res_.x,
                                            grid_res_.y,
                                            grid_res_.z,
                                            1,
                                            gpu::TextureFormat::SINT_32,
                                            usage,
                                            nullptr);
  }

  void ensure_compact_atlas()
  {
    if (bricks_per_axis_ < 1) {
      bricks_per_axis_ = 1;
    }
    int atlas_dim = bricks_per_axis_ * SDF_BRICK_STORAGE;

    if (compact_atlas_tx_ != nullptr) {
      int existing_dim = GPU_texture_width(compact_atlas_tx_);
      if (existing_dim == atlas_dim) {
        return;
      }
      GPU_texture_free(compact_atlas_tx_);
      compact_atlas_tx_ = nullptr;
    }

    eGPUTextureUsage usage = GPU_TEXTURE_USAGE_SHADER_READ | GPU_TEXTURE_USAGE_SHADER_WRITE;
    compact_atlas_tx_ = GPU_texture_create_3d("sdf_compact_atlas",
                                              atlas_dim,
                                              atlas_dim,
                                              atlas_dim,
                                              1,
                                              gpu::TextureFormat::SFLOAT_16_16_16_16,
                                              usage,
                                              nullptr);
    GPU_texture_filter_mode(compact_atlas_tx_, true);
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

    /* Dirty flags SSBO. */
    const int df_count = math::max(int(dirty_flags_.size()), 1);
    const size_t df_buf_size = df_count * sizeof(int);

    if (dirty_flags_ssbo_ != nullptr && dirty_flags_ssbo_count_ != df_count) {
      GPU_storagebuf_free(dirty_flags_ssbo_);
      dirty_flags_ssbo_ = nullptr;
    }

    if (dirty_flags_ssbo_ == nullptr) {
      if (dirty_flags_.is_empty()) {
        int dummy = 0;
        dirty_flags_ssbo_ = GPU_storagebuf_create_ex(
            df_buf_size, &dummy, GPU_USAGE_DYNAMIC, "sdf_dirty_flags_ssbo");
      }
      else {
        dirty_flags_ssbo_ = GPU_storagebuf_create_ex(
            df_buf_size, dirty_flags_.data(), GPU_USAGE_DYNAMIC, "sdf_dirty_flags_ssbo");
      }
      dirty_flags_ssbo_count_ = df_count;
    }
    else {
      if (!dirty_flags_.is_empty()) {
        GPU_storagebuf_update(dirty_flags_ssbo_, dirty_flags_.data());
      }
    }
  }

  /* SAH BVH */

  void build_bvh()
  {
    bvh_nodes_.clear();
    const int n = int(objects_.size());
    if (n == 0) {
      return;
    }

    Vector<int> indices(n);
    for (int i = 0; i < n; i++) {
      indices[i] = i;
    }

    Vector<float3> centroids(n);
    for (int i = 0; i < n; i++) {
      centroids[i] = (float3(objects_[i].bbox_min) + float3(objects_[i].bbox_max)) * 0.5f;
    }

    bvh_nodes_.reserve(2 * n);

    build_bvh_recursive(indices.data(), n, centroids);
  }

  int build_bvh_recursive(int *indices, int count, const Vector<float3> &centroids)
  {
    float3 node_min = float3(1e30f);
    float3 node_max = float3(-1e30f);
    for (int i = 0; i < count; i++) {
      const SDFObjectGPU &obj = objects_[indices[i]];
      node_min = math::min(node_min, float3(obj.bbox_min));
      node_max = math::max(node_max, float3(obj.bbox_max));
    }

    if (count == 1) {
      int node_idx = int(bvh_nodes_.size());
      BVHNodeGPU node = {};
      node.min_and_left = float4(node_min, 0.0f);
      node.max_and_right = float4(node_max, 0.0f);
      reinterpret_cast<int &>(node.min_and_left.w) = -1;
      reinterpret_cast<int &>(node.max_and_right.w) = indices[0];
      bvh_nodes_.append(node);
      return node_idx;
    }

    float3 centroid_min = float3(1e30f);
    float3 centroid_max = float3(-1e30f);
    for (int i = 0; i < count; i++) {
      centroid_min = math::min(centroid_min, centroids[indices[i]]);
      centroid_max = math::max(centroid_max, centroids[indices[i]]);
    }

    float3 centroid_extent = centroid_max - centroid_min;

    if (math::reduce_max(centroid_extent) < 1e-6f) {
      int mid = count / 2;
      int node_idx = int(bvh_nodes_.size());
      bvh_nodes_.append({});

      int left = build_bvh_recursive(indices, mid, centroids);
      int right = build_bvh_recursive(indices + mid, count - mid, centroids);

      BVHNodeGPU &node = bvh_nodes_[node_idx];
      node.min_and_left = float4(node_min, 0.0f);
      node.max_and_right = float4(node_max, 0.0f);
      reinterpret_cast<int &>(node.min_and_left.w) = left;
      reinterpret_cast<int &>(node.max_and_right.w) = right;
      return node_idx;
    }

    constexpr int NUM_BINS = 8;
    float best_cost = 1e30f;
    int best_axis = 0;
    int best_split = -1;

    auto surface_area = [](float3 lo, float3 hi) -> float {
      float3 d = hi - lo;
      return 2.0f * (d.x * d.y + d.y * d.z + d.z * d.x);
    };

    float parent_area = surface_area(node_min, node_max);

    for (int axis = 0; axis < 3; axis++) {
      if (centroid_extent[axis] < 1e-6f) {
        continue;
      }

      struct Bin {
        float3 lo = float3(1e30f);
        float3 hi = float3(-1e30f);
        int count = 0;
      };
      Bin bins[NUM_BINS];

      float inv_extent = float(NUM_BINS) / centroid_extent[axis];
      for (int i = 0; i < count; i++) {
        int b = int((centroids[indices[i]][axis] - centroid_min[axis]) * inv_extent);
        b = math::clamp(b, 0, NUM_BINS - 1);
        bins[b].lo = math::min(bins[b].lo, float3(objects_[indices[i]].bbox_min));
        bins[b].hi = math::max(bins[b].hi, float3(objects_[indices[i]].bbox_max));
        bins[b].count++;
      }

      float3 left_min[NUM_BINS - 1];
      float3 left_max[NUM_BINS - 1];
      int left_count[NUM_BINS - 1];
      {
        float3 running_min = float3(1e30f);
        float3 running_max = float3(-1e30f);
        int running_count = 0;
        for (int i = 0; i < NUM_BINS - 1; i++) {
          running_min = math::min(running_min, bins[i].lo);
          running_max = math::max(running_max, bins[i].hi);
          running_count += bins[i].count;
          left_min[i] = running_min;
          left_max[i] = running_max;
          left_count[i] = running_count;
        }
      }

      {
        float3 running_min = float3(1e30f);
        float3 running_max = float3(-1e30f);
        int running_count = 0;
        for (int i = NUM_BINS - 1; i >= 1; i--) {
          running_min = math::min(running_min, bins[i].lo);
          running_max = math::max(running_max, bins[i].hi);
          running_count += bins[i].count;

          if (left_count[i - 1] > 0 && running_count > 0) {
            float left_area = surface_area(left_min[i - 1], left_max[i - 1]);
            float right_area = surface_area(running_min, running_max);
            float cost = 1.0f + (left_count[i - 1] * left_area + running_count * right_area) /
                                    parent_area;
            if (cost < best_cost) {
              best_cost = cost;
              best_axis = axis;
              best_split = i;
            }
          }
        }
      }
    }

    int mid;
    if (best_split < 0) {
      mid = count / 2;
    }
    else {
      float inv_extent = float(NUM_BINS) / centroid_extent[best_axis];
      int left_end = 0;
      for (int i = 0; i < count; i++) {
        int b = int((centroids[indices[i]][best_axis] - centroid_min[best_axis]) * inv_extent);
        b = math::clamp(b, 0, NUM_BINS - 1);
        if (b < best_split) {
          int tmp = indices[left_end];
          indices[left_end] = indices[i];
          indices[i] = tmp;
          left_end++;
        }
      }
      mid = left_end;
      if (mid == 0) {
        mid = 1;
      }
      if (mid == count) {
        mid = count - 1;
      }
    }

    int node_idx = int(bvh_nodes_.size());
    bvh_nodes_.append({});

    int left = build_bvh_recursive(indices, mid, centroids);
    int right = build_bvh_recursive(indices + mid, count - mid, centroids);

    BVHNodeGPU &node = bvh_nodes_[node_idx];
    node.min_and_left = float4(node_min, 0.0f);
    node.max_and_right = float4(node_max, 0.0f);
    reinterpret_cast<int &>(node.min_and_left.w) = left;
    reinterpret_cast<int &>(node.max_and_right.w) = right;
    return node_idx;
  }

  void update_bvh_aabbs()
  {
    /* Update AABBs bottom-up without changing topology. */
    const int n = int(bvh_nodes_.size());
    for (int i = n - 1; i >= 0; i--) {
      BVHNodeGPU &node = bvh_nodes_[i];
      int left = reinterpret_cast<int &>(node.min_and_left.w);
      int right = reinterpret_cast<int &>(node.max_and_right.w);

      if (left == -1) {
        /* Leaf: update from object bbox. */
        int obj_idx = right;
        if (obj_idx >= 0 && obj_idx < int(objects_.size())) {
          node.min_and_left = float4(float3(objects_[obj_idx].bbox_min), node.min_and_left.w);
          node.max_and_right = float4(float3(objects_[obj_idx].bbox_max), node.max_and_right.w);
        }
      }
      else {
        /* Interior: union of children. */
        const BVHNodeGPU &l = bvh_nodes_[left];
        const BVHNodeGPU &r = bvh_nodes_[right];
        float3 lo = math::min(float3(l.min_and_left), float3(r.min_and_left));
        float3 hi = math::max(float3(l.max_and_right), float3(r.max_and_right));
        node.min_and_left = float4(lo, node.min_and_left.w);
        node.max_and_right = float4(hi, node.max_and_right.w);
      }
    }
  }

  void upload_bvh()
  {
    const int count = int(bvh_nodes_.size());
    if (count == 0) {
      return;
    }
    const size_t buf_size = count * sizeof(BVHNodeGPU);

    if (bvh_ssbo_ != nullptr && bvh_ssbo_count_ != count) {
      GPU_storagebuf_free(bvh_ssbo_);
      bvh_ssbo_ = nullptr;
    }

    if (bvh_ssbo_ == nullptr) {
      bvh_ssbo_ = GPU_storagebuf_create_ex(
          buf_size, bvh_nodes_.data(), GPU_USAGE_DYNAMIC, "sdf_bvh_ssbo");
      bvh_ssbo_count_ = count;
    }
    else {
      GPU_storagebuf_update(bvh_ssbo_, bvh_nodes_.data());
    }
  }

  void dispatch_classify()
  {
    const bool incremental = incremental_bake_;

    BrickCounter init_counter = {};
    init_counter.count = 0;
    if (incremental) {
      init_counter.next_slot = uint(total_allocated_slots_);
    }
    else {
      init_counter.next_slot = 0;
    }
    init_counter._pad1 = 0;
    init_counter._pad2 = 0;

    if (brick_counter_ == nullptr) {
      brick_counter_ = GPU_storagebuf_create_ex(
          sizeof(BrickCounter), &init_counter, GPU_USAGE_DYNAMIC, "sdf_brick_counter");
    }
    else {
      GPU_storagebuf_update(brick_counter_, &init_counter);
    }

    int max_bricks;
    if (incremental) {
      int3 dirty_size = dirty_brick_max_ - dirty_brick_min_;
      max_bricks = math::max(dirty_size.x * dirty_size.y * dirty_size.z, 1);
    }
    else {
      max_bricks = grid_res_.x * grid_res_.y * grid_res_.z;
      if (max_bricks < 1) {
        max_bricks = 1;
      }
    }
    max_bricks = math::min(max_bricks, SDF_MAX_BRICKS);
    if (active_bricks_ == nullptr || active_bricks_capacity_ < max_bricks) {
      if (active_bricks_) {
        GPU_storagebuf_free(active_bricks_);
      }
      active_bricks_ = GPU_storagebuf_create_ex(
          max_bricks * sizeof(ActiveBrick), nullptr, GPU_USAGE_DYNAMIC, "sdf_active_bricks");
      active_bricks_capacity_ = max_bricks;
    }

    GPU_shader_bind(classify_sh_);

    int obj_slot = GPU_shader_get_ssbo_binding(classify_sh_, "objects");
    GPU_storagebuf_bind(object_ssbo_, obj_slot);

    int counter_slot = GPU_shader_get_ssbo_binding(classify_sh_, "brick_counter");
    GPU_storagebuf_bind(brick_counter_, counter_slot);

    int ab_slot = GPU_shader_get_ssbo_binding(classify_sh_, "active_bricks");
    GPU_storagebuf_bind(active_bricks_, ab_slot);

    if (bvh_ssbo_) {
      int bvh_slot = GPU_shader_get_ssbo_binding(classify_sh_, "bvh_nodes");
      GPU_storagebuf_bind(bvh_ssbo_, bvh_slot);
    }

    if (modifier_ssbo_) {
      int mod_slot = GPU_shader_get_ssbo_binding(classify_sh_, "sdf_modifiers");
      GPU_storagebuf_bind(modifier_ssbo_, mod_slot);
    }

    if (group_ssbo_) {
      int grp_slot = GPU_shader_get_ssbo_binding(classify_sh_, "groups");
      GPU_storagebuf_bind(group_ssbo_, grp_slot);
    }

    if (dirty_flags_ssbo_) {
      int df_slot = GPU_shader_get_ssbo_binding(classify_sh_, "dirty_flags");
      GPU_storagebuf_bind(dirty_flags_ssbo_, df_slot);
    }

    GPU_texture_image_bind(indirection_tx_, 0);

    GPU_shader_uniform_1i(classify_sh_, "object_count", int(objects_.size()));
    GPU_shader_uniform_1f(classify_sh_, "voxel_size", voxel_size_);
    GPU_shader_uniform_3fv(classify_sh_, "atlas_origin", atlas_origin_);
    GPU_shader_uniform_3iv(classify_sh_, "grid_resolution", grid_res_);

    float brick_half_diag = float(SDF_BRICK_SIZE) * voxel_size_ * 0.866025f; /* sqrt(3)/2 */
    brick_half_diag *= surface_margin_;
    GPU_shader_uniform_1f(classify_sh_, "brick_half_diag", brick_half_diag);
    GPU_shader_uniform_1i(classify_sh_, "bvh_node_count", int(bvh_nodes_.size()));
    GPU_shader_uniform_1i(classify_sh_, "group_count", int(groups_gpu_.size()));

    GPU_shader_uniform_1i(classify_sh_, "incremental_mode", incremental ? 1 : 0);
    GPU_shader_uniform_3iv(classify_sh_, "dirty_brick_min", dirty_brick_min_);
    GPU_shader_uniform_3iv(classify_sh_, "dirty_brick_max", dirty_brick_max_);
    GPU_shader_uniform_1i(
        classify_sh_, "has_dirty_flags", (incremental && !dirty_flags_.is_empty()) ? 1 : 0);
    GPU_shader_uniform_1i(classify_sh_, "max_active_bricks", max_bricks);

    if (incremental) {
      int3 dirty_size = dirty_brick_max_ - dirty_brick_min_;
      if (dirty_size.x > 0 && dirty_size.y > 0 && dirty_size.z > 0) {
        GPU_compute_dispatch(classify_sh_,
                             divide_ceil_u(dirty_size.x, 4),
                             divide_ceil_u(dirty_size.y, 4),
                             divide_ceil_u(dirty_size.z, 4));
      }
    }
    else {
      GPU_compute_dispatch(classify_sh_,
                           divide_ceil_u(grid_res_.x, 4),
                           divide_ceil_u(grid_res_.y, 4),
                           divide_ceil_u(grid_res_.z, 4));
    }

    GPU_memory_barrier(GPU_BARRIER_TEXTURE_FETCH | GPU_BARRIER_SHADER_STORAGE);
    GPU_texture_image_unbind(indirection_tx_);
    GPU_shader_unbind();

    {
      BrickCounter readback = {};
      GPU_storagebuf_read(brick_counter_, &readback);
      active_brick_count_ = int(readback.count);

      if (incremental) {
        total_allocated_slots_ = int(readback.next_slot);
      }
      else {
        total_allocated_slots_ = active_brick_count_;
      }
    }
  }

  void dispatch_bake()
  {
    int dispatch_count = active_brick_count_;
    if (dispatch_count <= 0) {
      return;
    }

    GPU_shader_bind(bake_sh_);

    int ssbo_slot = GPU_shader_get_ssbo_binding(bake_sh_, "objects");
    GPU_storagebuf_bind(object_ssbo_, ssbo_slot);

    int ab_slot = GPU_shader_get_ssbo_binding(bake_sh_, "active_bricks");
    GPU_storagebuf_bind(active_bricks_, ab_slot);

    if (bvh_ssbo_) {
      int bvh_slot = GPU_shader_get_ssbo_binding(bake_sh_, "bvh_nodes");
      GPU_storagebuf_bind(bvh_ssbo_, bvh_slot);
    }

    int counter_slot = GPU_shader_get_ssbo_binding(bake_sh_, "brick_counter");
    GPU_storagebuf_bind(brick_counter_, counter_slot);

    /* Bind modifier SSBO. */
    if (modifier_ssbo_) {
      int mod_slot = GPU_shader_get_ssbo_binding(bake_sh_, "sdf_modifiers");
      GPU_storagebuf_bind(modifier_ssbo_, mod_slot);
    }

    /* Bind group SSBO. */
    if (group_ssbo_) {
      int grp_slot = GPU_shader_get_ssbo_binding(bake_sh_, "groups");
      GPU_storagebuf_bind(group_ssbo_, grp_slot);
    }

    if (dirty_flags_ssbo_) {
      int df_slot = GPU_shader_get_ssbo_binding(bake_sh_, "dirty_flags");
      GPU_storagebuf_bind(dirty_flags_ssbo_, df_slot);
    }

    /* Bind atlas images. */
    GPU_texture_image_bind(compact_atlas_tx_, 0);

    /* Push constants. */
    GPU_shader_uniform_1i(bake_sh_, "object_count", int(objects_.size()));
    GPU_shader_uniform_1f(bake_sh_, "voxel_size", voxel_size_);
    GPU_shader_uniform_3fv(bake_sh_, "atlas_origin", atlas_origin_);
    GPU_shader_uniform_1i(bake_sh_, "bricks_per_axis", bricks_per_axis_);
    GPU_shader_uniform_1i(bake_sh_, "bvh_node_count", int(bvh_nodes_.size()));
    GPU_shader_uniform_1i(bake_sh_, "group_count", int(groups_gpu_.size()));
    GPU_shader_uniform_1i(
        bake_sh_, "has_dirty_flags", (incremental_bake_ && !dirty_flags_.is_empty()) ? 1 : 0);
    GPU_shader_uniform_1f(bake_sh_, "max_blend", max_blend_);

    /* Over-dispatch: one workgroup per capacity slot. Surplus workgroups
     * early-exit after reading brick_counter.count from SSBO. */
    uint bake_x = uint(math::min(dispatch_count, 65535));
    uint bake_y = uint(divide_ceil_u(dispatch_count, 65535));
    GPU_shader_uniform_1i(bake_sh_, "dispatch_width", int(bake_x));
    GPU_compute_dispatch(bake_sh_, bake_x, bake_y, 1);

    GPU_memory_barrier(GPU_BARRIER_TEXTURE_FETCH | GPU_BARRIER_SHADER_STORAGE);
    GPU_texture_image_unbind(compact_atlas_tx_);
    GPU_shader_unbind();
    /* active_brick_count_ was already set by dispatch_classify() readback. */
  }

  void draw_march()
  {
    DefaultFramebufferList *dfbl = draw_ctx_->viewport_framebuffer_list_get();

    if (draw_ctx_->is_depth() || (draw_ctx_->v3d && draw_ctx_->v3d->shading.type == OB_RENDER)) {
      /* Depth-only: either a depth prepass, or rendered mode where Cycles
       * provides the color and we only contribute depth for the overlay grid. */
      GPU_framebuffer_bind(dfbl->depth_only_fb);
    }
    else if (fxaa_enabled_) {
      /* Solid / wireframe with FXAA: render to offscreen texture for post-processing.
       * Depth is shared with the viewport so depth writes still land correctly. */
      ensure_fxaa_target();
      GPU_framebuffer_bind(march_fb_);
      float clear_color[4] = {0.0f, 0.0f, 0.0f, 0.0f};
      GPU_framebuffer_clear_color(march_fb_, clear_color);
    }
    else {
      /* Solid / wireframe without FXAA: render directly to default framebuffer. */
      GPU_framebuffer_bind(dfbl->default_fb);
    }

    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
    GPU_depth_mask(true);
    GPU_blend(GPU_BLEND_NONE);
    GPU_face_culling(GPU_CULL_NONE);
    GPU_stencil_test(GPU_STENCIL_NONE);

    /* Scissor rect: project atlas AABB to screen to skip pixels outside the SDF scene. */
    bool scissor_set = false;
    {
      float3 aabb_min = atlas_origin_;
      float3 aabb_max = atlas_origin_ + float3(grid_res_ * SDF_BRICK_SIZE) * voxel_size_;

      View &scissor_view = View::default_get();
      float4x4 vp_mat = scissor_view.winmat() * scissor_view.viewmat();

      float2 screen_min(1e30f);
      float2 screen_max(-1e30f);
      bool any_behind = false;

      for (int c = 0; c < 8; c++) {
        float3 corner((c & 1) ? aabb_max.x : aabb_min.x,
                       (c & 2) ? aabb_max.y : aabb_min.y,
                       (c & 4) ? aabb_max.z : aabb_min.z);
        float4 clip = vp_mat * float4(corner, 1.0f);
        if (clip.w <= 0.0f) {
          any_behind = true;
          break;
        }
        float2 ndc = float2(clip.x, clip.y) / clip.w;
        screen_min = math::min(screen_min, ndc);
        screen_max = math::max(screen_max, ndc);
      }

      if (!any_behind) {
        const int2 vp_size = int2(draw_ctx_->viewport_size_get());
        int x0 = int(floorf((screen_min.x * 0.5f + 0.5f) * float(vp_size.x))) - 16;
        int y0 = int(floorf((screen_min.y * 0.5f + 0.5f) * float(vp_size.y))) - 16;
        int x1 = int(ceilf((screen_max.x * 0.5f + 0.5f) * float(vp_size.x))) + 16;
        int y1 = int(ceilf((screen_max.y * 0.5f + 0.5f) * float(vp_size.y))) + 16;

        x0 = math::max(x0, 0);
        y0 = math::max(y0, 0);
        x1 = math::min(x1, vp_size.x);
        y1 = math::min(y1, vp_size.y);

        if (x1 > x0 && y1 > y0 && (x1 - x0) < vp_size.x && (y1 - y0) < vp_size.y) {
          GPU_scissor_test(true);
          GPU_scissor(x0, y0, x1 - x0, y1 - y0);
          scissor_set = true;
        }
      }
    }

    GPU_shader_bind(march_sh_);

    /* Bind compact atlas as sampler. */
    int atlas_slot = GPU_shader_get_sampler_binding(march_sh_, "compact_atlas");
    if (compact_atlas_tx_) {
      GPU_texture_bind(compact_atlas_tx_, atlas_slot);
    }

    /* Bind indirection as sampler. */
    int indir_slot = GPU_shader_get_sampler_binding(march_sh_, "indirection_tx");
    if (indirection_tx_) {
      GPU_texture_bind(indirection_tx_, indir_slot);
    }

    if (bvh_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(march_sh_, "bvh_nodes");
      GPU_storagebuf_bind(bvh_ssbo_, slot);
    }
    if (modifier_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(march_sh_, "sdf_modifiers");
      GPU_storagebuf_bind(modifier_ssbo_, slot);
    }
    if (object_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(march_sh_, "objects");
      GPU_storagebuf_bind(object_ssbo_, slot);
    }
    if (group_ssbo_) {
      int slot = GPU_shader_get_ssbo_binding(march_sh_, "groups");
      GPU_storagebuf_bind(group_ssbo_, slot);
    }

    /* Push constants. */
    GPU_shader_uniform_1f(march_sh_, "voxel_size", voxel_size_);
    GPU_shader_uniform_3fv(march_sh_, "atlas_origin", atlas_origin_);
    GPU_shader_uniform_3fv(march_sh_, "atlas_extent", atlas_extent_);
    GPU_shader_uniform_3iv(march_sh_, "grid_resolution", grid_res_);
    GPU_shader_uniform_1i(march_sh_, "bricks_per_axis", bricks_per_axis_);
    GPU_shader_uniform_1i(march_sh_, "lighting_type", lighting_type_);
    GPU_shader_uniform_1i(march_sh_, "use_specular", use_specular_);
    GPU_shader_uniform_1i(march_sh_, "use_matcap_flip", use_matcap_flip_);
    GPU_shader_uniform_1i(march_sh_, "use_instanced", 0);
    GPU_shader_uniform_1i(march_sh_, "instance_count", 0);
    GPU_shader_uniform_1i(march_sh_, "bvh_node_count", int(bvh_nodes_.size()));
    GPU_shader_uniform_1i(march_sh_, "object_count", int(objects_.size()));
    GPU_shader_uniform_1i(march_sh_, "group_count", int(groups_gpu_.size()));

    GPU_shader_uniform_4fv(march_sh_, "studio_light0", studio_light_dir_[0]);
    GPU_shader_uniform_4fv(march_sh_, "studio_light1", studio_light_dir_[1]);
    GPU_shader_uniform_4fv(march_sh_, "studio_light2", studio_light_dir_[2]);
    GPU_shader_uniform_4fv(march_sh_, "studio_light3", studio_light_dir_[3]);
    GPU_shader_uniform_4fv(march_sh_, "studio_color0", studio_light_col_[0]);
    GPU_shader_uniform_4fv(march_sh_, "studio_color1", studio_light_col_[1]);
    GPU_shader_uniform_4fv(march_sh_, "studio_color2", studio_light_col_[2]);
    GPU_shader_uniform_4fv(march_sh_, "studio_color3", studio_light_col_[3]);
    GPU_shader_uniform_4fv(march_sh_, "studio_spec0", studio_light_spec_[0]);
    GPU_shader_uniform_4fv(march_sh_, "studio_spec1", studio_light_spec_[1]);
    GPU_shader_uniform_4fv(march_sh_, "studio_spec2", studio_light_spec_[2]);
    GPU_shader_uniform_4fv(march_sh_, "studio_spec3", studio_light_spec_[3]);
    GPU_shader_uniform_3fv(march_sh_, "studio_ambient", studio_ambient_);

    /* Bind matcap texture. */
    if (matcap_tx_) {
      int matcap_slot = GPU_shader_get_sampler_binding(march_sh_, "matcap_tx");
      GPU_texture_bind(matcap_tx_, matcap_slot);
    }

    /* Bind the view UBO so the fragment shader can access camera matrices.
     * Must call push_update() before binding — this uploads the current frame's
     * matrices to the GPU. Without it, the GPU-side UBO contains stale data from
     * the previous frame, causing incorrect gl_FragDepth during camera movement.
     * View::bind() does this internally but is protected; we call push_update()
     * directly on the public UBO reference. */
    View &view = View::default_get();
    view.matrices_ubo_get().push_update();
    GPU_uniformbuf_bind(view.matrices_ubo_get(), DRW_VIEW_UBO_SLOT);

    if (fullscreen_batch_ == nullptr) {
      fullscreen_batch_ = GPU_batch_create_procedural(GPU_PRIM_TRIS, 3);
    }
    GPU_batch_set_shader(fullscreen_batch_, march_sh_);
    GPU_batch_draw(fullscreen_batch_);

    if (scissor_set) {
      GPU_scissor_test(false);
    }

    if (compact_atlas_tx_) {
      GPU_texture_unbind(compact_atlas_tx_);
    }
    if (indirection_tx_) {
      GPU_texture_unbind(indirection_tx_);
    }
    if (matcap_tx_) {
      GPU_texture_unbind(matcap_tx_);
    }
    GPU_shader_unbind();
  }

  /* ---- FXAA post-process ---- */

  /** Ensure the offscreen color texture and framebuffer for march → FXAA exist
   * and match the current viewport size. */
  void ensure_fxaa_target()
  {
    const int2 vp = int2(draw_ctx_->viewport_size_get());
    if (march_color_tx_ != nullptr && fxaa_viewport_size_ == vp) {
      return;
    }

    /* (Re)create on size change. */
    if (march_color_tx_) {
      GPU_texture_free(march_color_tx_);
      march_color_tx_ = nullptr;
    }
    if (march_fb_) {
      GPU_framebuffer_free(march_fb_);
      march_fb_ = nullptr;
    }

    fxaa_viewport_size_ = vp;

    eGPUTextureUsage usage = GPU_TEXTURE_USAGE_SHADER_READ | GPU_TEXTURE_USAGE_ATTACHMENT;
    march_color_tx_ = GPU_texture_create_2d(
        "sdf_march_color", vp.x, vp.y, 1, gpu::TextureFormat::SFLOAT_16_16_16_16, usage, nullptr);

    /* Build framebuffer: offscreen color + viewport depth (shared). */
    DefaultTextureList *dtxl = draw_ctx_->viewport_texture_list_get();
    march_fb_ = GPU_framebuffer_create("sdf_march_fb");
    GPU_framebuffer_texture_attach(march_fb_, march_color_tx_, 0, 0);
    GPU_framebuffer_texture_attach(march_fb_, dtxl->depth, 0, 0);
  }

  /** Apply FXAA to the offscreen march result and composite to the default FB. */
  void draw_fxaa()
  {
    if (!fxaa_enabled_ || fxaa_sh_ == nullptr) {
      return;
    }

    /* Skip in depth-only or Cycles rendered mode (matches draw_march behavior). */
    if (draw_ctx_->is_depth() || (draw_ctx_->v3d && draw_ctx_->v3d->shading.type == OB_RENDER)) {
      return;
    }

    DefaultFramebufferList *dfbl = draw_ctx_->viewport_framebuffer_list_get();
    GPU_framebuffer_bind(dfbl->default_fb);

    /* FXAA is screen-space color only — depth was already written by march. */
    GPU_depth_test(GPU_DEPTH_NONE);
    GPU_depth_mask(false);
    GPU_blend(GPU_BLEND_ALPHA);
    GPU_face_culling(GPU_CULL_NONE);
    GPU_stencil_test(GPU_STENCIL_NONE);

    GPU_shader_bind(fxaa_sh_);

    int color_slot = GPU_shader_get_sampler_binding(fxaa_sh_, "color_tx");
    GPU_texture_filter_mode(march_color_tx_, true);
    GPU_texture_bind(march_color_tx_, color_slot);

    float2 rcp = float2(1.0f / float(fxaa_viewport_size_.x), 1.0f / float(fxaa_viewport_size_.y));
    GPU_shader_uniform_2fv(fxaa_sh_, "rcpFrame", rcp);

    if (fullscreen_batch_ == nullptr) {
      fullscreen_batch_ = GPU_batch_create_procedural(GPU_PRIM_TRIS, 3);
    }
    GPU_batch_set_shader(fullscreen_batch_, fxaa_sh_);
    GPU_batch_draw(fullscreen_batch_);

    GPU_texture_unbind(march_color_tx_);
    GPU_shader_unbind();

    /* Restore depth state for subsequent passes (debug grid, overlays). */
    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
    GPU_depth_mask(true);
    GPU_blend(GPU_BLEND_NONE);
  }

  /* ---- Debug grid wireframe ---- */

  gpu::Batch *create_line_batch(const float3 *positions, int vert_count)
  {
    if (vert_count <= 0) {
      return nullptr;
    }

    GPUVertFormat format = {};
    GPU_vertformat_attr_add(&format, "pos", gpu::VertAttrType::SFLOAT_32_32_32);

    gpu::VertBuf *vbo = GPU_vertbuf_create_with_format(format);
    GPU_vertbuf_data_alloc(*vbo, uint(vert_count));
    GPU_vertbuf_attr_fill(vbo, 0, positions);

    return GPU_batch_create_ex(GPU_PRIM_LINES, vbo, nullptr, GPU_BATCH_OWNS_VBO);
  }

  /** Create a GPU_PRIM_LINES batch with per-vertex position and color. */
  gpu::Batch *create_colored_line_batch(const float3 *positions,
                                        const float4 *colors,
                                        int vert_count)
  {
    if (vert_count <= 0) {
      return nullptr;
    }

    GPUVertFormat format = {};
    uint pos_attr = GPU_vertformat_attr_add(&format, "pos", gpu::VertAttrType::SFLOAT_32_32_32);
    uint col_attr = GPU_vertformat_attr_add(
        &format, "color", gpu::VertAttrType::SFLOAT_32_32_32_32);

    gpu::VertBuf *vbo = GPU_vertbuf_create_with_format(format);
    GPU_vertbuf_data_alloc(*vbo, uint(vert_count));
    GPU_vertbuf_attr_fill(vbo, pos_attr, positions);
    GPU_vertbuf_attr_fill(vbo, col_attr, colors);

    return GPU_batch_create_ex(GPU_PRIM_LINES, vbo, nullptr, GPU_BATCH_OWNS_VBO);
  }

  void rebuild_grid_batch_active()
  {
    if (!indirection_tx_) {
      return;
    }

    int32_t *data = static_cast<int32_t *>(GPU_texture_read(indirection_tx_, GPU_DATA_INT, 0));
    if (!data) {
      return;
    }

    /* Texture may be larger than grid_res_ due to lazy reallocation.
     * Use actual texture dimensions for the stride. */
    int3 n = grid_res_;
    int tex_w = GPU_texture_width(indirection_tx_);
    int tex_h = GPU_texture_height(indirection_tx_);
    float brick_world = float(SDF_BRICK_SIZE) * voxel_size_;

    int active_count = 0;
    for (int z = 0; z < n.z; z++) {
      for (int y = 0; y < n.y; y++) {
        for (int x = 0; x < n.x; x++) {
          int idx = x + y * tex_w + z * tex_w * tex_h;
          if (data[idx] >= 0) {
            active_count++;
          }
        }
      }
    }

    if (active_count == 0) {
      MEM_freeN(data);
      return;
    }

    Vector<float3> positions(active_count * 24);
    int vi = 0;

    for (int z = 0; z < n.z; z++) {
      for (int y = 0; y < n.y; y++) {
        for (int x = 0; x < n.x; x++) {
          int idx = x + y * tex_w + z * tex_w * tex_h;
          if (data[idx] < 0) {
            continue;
          }

          float3 lo = atlas_origin_ + float3(float(x), float(y), float(z)) * brick_world;
          float3 hi = lo + float3(brick_world);

          positions[vi++] = float3(lo.x, lo.y, lo.z);
          positions[vi++] = float3(hi.x, lo.y, lo.z);
          positions[vi++] = float3(hi.x, lo.y, lo.z);
          positions[vi++] = float3(hi.x, hi.y, lo.z);
          positions[vi++] = float3(hi.x, hi.y, lo.z);
          positions[vi++] = float3(lo.x, hi.y, lo.z);
          positions[vi++] = float3(lo.x, hi.y, lo.z);
          positions[vi++] = float3(lo.x, lo.y, lo.z);

          positions[vi++] = float3(lo.x, lo.y, hi.z);
          positions[vi++] = float3(hi.x, lo.y, hi.z);
          positions[vi++] = float3(hi.x, lo.y, hi.z);
          positions[vi++] = float3(hi.x, hi.y, hi.z);
          positions[vi++] = float3(hi.x, hi.y, hi.z);
          positions[vi++] = float3(lo.x, hi.y, hi.z);
          positions[vi++] = float3(lo.x, hi.y, hi.z);
          positions[vi++] = float3(lo.x, lo.y, hi.z);

          positions[vi++] = float3(lo.x, lo.y, lo.z);
          positions[vi++] = float3(lo.x, lo.y, hi.z);
          positions[vi++] = float3(hi.x, lo.y, lo.z);
          positions[vi++] = float3(hi.x, lo.y, hi.z);
          positions[vi++] = float3(hi.x, hi.y, lo.z);
          positions[vi++] = float3(hi.x, hi.y, hi.z);
          positions[vi++] = float3(lo.x, hi.y, lo.z);
          positions[vi++] = float3(lo.x, hi.y, hi.z);
        }
      }
    }

    MEM_freeN(data);
    grid_batch_ = create_line_batch(positions.data(), vi);
  }

  /** Emit 12 wireframe edges (24 vertices) for an axis-aligned box. */
  static void emit_box_edges(Vector<float3> &positions,
                             Vector<float4> &colors,
                             const float3 &lo,
                             const float3 &hi,
                             const float4 &col)
  {
    /* Bottom face. */
    positions.append(float3(lo.x, lo.y, lo.z));
    colors.append(col);
    positions.append(float3(hi.x, lo.y, lo.z));
    colors.append(col);
    positions.append(float3(hi.x, lo.y, lo.z));
    colors.append(col);
    positions.append(float3(hi.x, hi.y, lo.z));
    colors.append(col);
    positions.append(float3(hi.x, hi.y, lo.z));
    colors.append(col);
    positions.append(float3(lo.x, hi.y, lo.z));
    colors.append(col);
    positions.append(float3(lo.x, hi.y, lo.z));
    colors.append(col);
    positions.append(float3(lo.x, lo.y, lo.z));
    colors.append(col);

    /* Top face. */
    positions.append(float3(lo.x, lo.y, hi.z));
    colors.append(col);
    positions.append(float3(hi.x, lo.y, hi.z));
    colors.append(col);
    positions.append(float3(hi.x, lo.y, hi.z));
    colors.append(col);
    positions.append(float3(hi.x, hi.y, hi.z));
    colors.append(col);
    positions.append(float3(hi.x, hi.y, hi.z));
    colors.append(col);
    positions.append(float3(lo.x, hi.y, hi.z));
    colors.append(col);
    positions.append(float3(lo.x, hi.y, hi.z));
    colors.append(col);
    positions.append(float3(lo.x, lo.y, hi.z));
    colors.append(col);

    /* Vertical edges. */
    positions.append(float3(lo.x, lo.y, lo.z));
    colors.append(col);
    positions.append(float3(lo.x, lo.y, hi.z));
    colors.append(col);
    positions.append(float3(hi.x, lo.y, lo.z));
    colors.append(col);
    positions.append(float3(hi.x, lo.y, hi.z));
    colors.append(col);
    positions.append(float3(hi.x, hi.y, lo.z));
    colors.append(col);
    positions.append(float3(hi.x, hi.y, hi.z));
    colors.append(col);
    positions.append(float3(lo.x, hi.y, lo.z));
    colors.append(col);
    positions.append(float3(lo.x, hi.y, hi.z));
    colors.append(col);
  }

  /** HSV to RGB (H in [0,1], S in [0,1], V in [0,1]). */
  static float3 hsv_to_rgb(float h, float s, float v)
  {
    float c = v * s;
    float x = c * (1.0f - fabsf(fmodf(h * 6.0f, 2.0f) - 1.0f));
    float m = v - c;
    float3 rgb;
    if (h < 1.0f / 6.0f) {
      rgb = float3(c, x, 0);
    }
    else if (h < 2.0f / 6.0f) {
      rgb = float3(x, c, 0);
    }
    else if (h < 3.0f / 6.0f) {
      rgb = float3(0, c, x);
    }
    else if (h < 4.0f / 6.0f) {
      rgb = float3(0, x, c);
    }
    else if (h < 5.0f / 6.0f) {
      rgb = float3(x, 0, c);
    }
    else {
      rgb = float3(c, 0, x);
    }
    return rgb + float3(m);
  }

  /** Build a colored wireframe batch from the BVH tree and per-object OBBs. */
  void rebuild_bvh_batch()
  {
    if (bvh_nodes_.is_empty()) {
      return;
    }

    const int node_count = int(bvh_nodes_.size());
    const int obj_count = int(objects_.size());

    /* --- Compute depth for each node via iterative stack-based DFS --- */
    Vector<int> depth(node_count, 0);
    int max_depth = 0;

    struct StackEntry {
      int node_idx;
      int node_depth;
    };
    Vector<StackEntry> stack;
    stack.append({0, 0});

    while (!stack.is_empty()) {
      StackEntry entry = stack.last();
      stack.remove_last();

      if (entry.node_idx < 0 || entry.node_idx >= node_count) {
        continue;
      }

      depth[entry.node_idx] = entry.node_depth;
      max_depth = max_ii(max_depth, entry.node_depth);

      const BVHNodeGPU &node = bvh_nodes_[entry.node_idx];
      int left = reinterpret_cast<const int &>(node.min_and_left.w);
      int right = reinterpret_cast<const int &>(node.max_and_right.w);

      if (left >= 0) {
        /* Interior node: push children. */
        stack.append({left, entry.node_depth + 1});
        stack.append({right, entry.node_depth + 1});
      }
      /* Leaf: left == -1, nothing to push. */
    }

    /* Preallocate: 24 verts per BVH node + 24 verts per object OBB. */
    Vector<float3> positions;
    Vector<float4> colors;
    positions.reserve((node_count + obj_count) * 24);
    colors.reserve((node_count + obj_count) * 24);

    /* --- BVH node AABBs: depth-colored red→blue --- */
    float depth_range = max_ii(max_depth, 1);
    for (int i = 0; i < node_count; i++) {
      const BVHNodeGPU &node = bvh_nodes_[i];
      float3 lo = float3(node.min_and_left);
      float3 hi = float3(node.max_and_right);

      float t = float(depth[i]) / depth_range;
      float hue = t * 0.67f; /* 0 = red, 0.67 = blue. */
      float3 rgb = hsv_to_rgb(hue, 0.85f, 0.9f);
      float4 col = float4(rgb, 0.85f);

      emit_box_edges(positions, colors, lo, hi, col);
    }

    /* --- Per-object OBBs: white, rotated wireframes --- */
    float4 white = float4(1.0f, 1.0f, 1.0f, 0.9f);
    for (int i = 0; i < obj_count; i++) {
      const SDFObjectGPU &obj = objects_[i];

      /* Reconstruct rotation matrix from inverse_matrix.
       * inverse_matrix is the inverse of the rotation-only matrix.
       * For pure rotation: inverse = transpose, so rot = transpose(inv). */
      float3 rot_col0 = float3(
          obj.inverse_matrix[0][0], obj.inverse_matrix[1][0], obj.inverse_matrix[2][0]);
      float3 rot_col1 = float3(
          obj.inverse_matrix[0][1], obj.inverse_matrix[1][1], obj.inverse_matrix[2][1]);
      float3 rot_col2 = float3(
          obj.inverse_matrix[0][2], obj.inverse_matrix[1][2], obj.inverse_matrix[2][2]);

      float3 pos = float3(obj.position);
      float3 extent = float3(obj.sdf_size) + float3(obj.bevel + obj.blend);

      /* Generate 8 OBB corners. */
      float3 corners[8];
      for (int c = 0; c < 8; c++) {
        float3 local = float3((c & 1) ? extent.x : -extent.x,
                              (c & 2) ? extent.y : -extent.y,
                              (c & 4) ? extent.z : -extent.z);
        corners[c] = rot_col0 * local.x + rot_col1 * local.y + rot_col2 * local.z + pos;
      }

      /* 12 edges of a box: bottom(4), top(4), vertical(4).
       * Corner indices: bottom = 0,1,3,2  top = 4,5,7,6 */
      const int edges[12][2] = {
          {0, 1},
          {1, 3},
          {3, 2},
          {2, 0}, /* bottom */
          {4, 5},
          {5, 7},
          {7, 6},
          {6, 4}, /* top */
          {0, 4},
          {1, 5},
          {3, 7},
          {2, 6}, /* vertical */
      };
      for (int e = 0; e < 12; e++) {
        positions.append(corners[edges[e][0]]);
        colors.append(white);
        positions.append(corners[edges[e][1]]);
        colors.append(white);
      }
    }

    bvh_batch_ = create_colored_line_batch(positions.data(), colors.data(), int(positions.size()));
  }

  void rebuild_grid_batch()
  {
    if (grid_batch_) {
      GPU_batch_discard(grid_batch_);
      grid_batch_ = nullptr;
    }
    if (bvh_batch_) {
      GPU_batch_discard(bvh_batch_);
      bvh_batch_ = nullptr;
    }
    if (scene_bounds_batch_) {
      GPU_batch_discard(scene_bounds_batch_);
      scene_bounds_batch_ = nullptr;
    }

    grid_batch_mode_ = debug_grid_;
    grid_batch_res_ = grid_res_;

    if (debug_grid_ == 1) {
      rebuild_grid_batch_active();
    }
    else if (debug_grid_ == 2) {
      rebuild_scene_bounds_batch();
    }
    else if (debug_grid_ == 3) {
      rebuild_bvh_batch();
    }
  }

  void draw_debug_grid()
  {
    if (debug_grid_ != 1) {
      return;
    }

    /* Rebuild batch if settings changed. */
    if (grid_batch_ == nullptr || grid_batch_mode_ != debug_grid_ || grid_batch_res_ != grid_res_)
    {
      rebuild_grid_batch();
    }
    if (grid_batch_ == nullptr) {
      return;
    }

    /* Use builtin 3D uniform color shader. */
    gpu::Shader *shader = GPU_shader_get_builtin_shader(GPU_SHADER_3D_UNIFORM_COLOR);
    GPU_shader_bind(shader);

    /* MVP = persmat (lines are in world space, no model transform). */
    View &view = View::default_get();
    float4x4 mvp = view.persmat();
    GPU_shader_uniform_mat4(shader, "ModelViewProjectionMatrix", mvp.ptr());

    GPU_blend(GPU_BLEND_ALPHA);
    GPU_depth_mask(false); /* Grid lines don't write depth. */

    GPU_batch_set_shader(grid_batch_, shader);

    /* --- Pass 1: Occluded lines (behind SDF surface) ---
     * Drawn first so front lines overdraw on top. Faint ghost
     * lines give spatial context without visual clutter. */
    GPU_depth_test(GPU_DEPTH_GREATER);
    GPU_line_width(1.0f);
    GPU_shader_uniform_4f(shader, "color", 0.3f, 0.5f, 0.7f, 0.08f);
    GPU_batch_draw(grid_batch_);

    /* --- Pass 2: Front lines (in front of SDF surface) ---
     * Bright and slightly thicker for clear readability. */
    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
    GPU_line_width(1.5f);
    GPU_shader_uniform_4f(shader, "color", 0.4f, 0.65f, 0.9f, 0.6f);
    GPU_batch_draw(grid_batch_);

    /* Restore state. */
    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
    GPU_depth_mask(true);
    GPU_blend(GPU_BLEND_NONE);
    GPU_line_width(1.0f);
    GPU_shader_unbind();
  }

  void rebuild_scene_bounds_batch()
  {
    Vector<float3> positions;
    Vector<float4> colors;

    /* Scene AABB (green). */
    float4 scene_col(0.2f, 0.9f, 0.2f, 0.8f);
    emit_box_edges(positions, colors, scene_min_, scene_max_, scene_col);

    /* Padded AABB (yellow). */
    float4 padded_col(0.9f, 0.9f, 0.2f, 0.6f);
    emit_box_edges(positions, colors, padded_min_, padded_max_, padded_col);

    /* Atlas grid bounds (cyan). */
    float4 atlas_col(0.2f, 0.8f, 0.9f, 0.6f);
    float3 atlas_max = atlas_origin_ + atlas_extent_;
    emit_box_edges(positions, colors, atlas_origin_, atlas_max, atlas_col);

    scene_bounds_batch_ = create_colored_line_batch(
        positions.data(), colors.data(), int(positions.size()));
  }

  void draw_debug_scene_bounds()
  {
    if (debug_grid_ != 2) {
      return;
    }

    if (scene_bounds_batch_ == nullptr || grid_batch_mode_ != debug_grid_ ||
        grid_batch_res_ != grid_res_)
    {
      rebuild_grid_batch();
    }
    if (scene_bounds_batch_ == nullptr) {
      return;
    }

    gpu::Shader *shader = GPU_shader_get_builtin_shader(GPU_SHADER_3D_FLAT_COLOR);
    GPU_shader_bind(shader);

    View &view = View::default_get();
    float4x4 mvp = view.persmat();
    GPU_shader_uniform_mat4(shader, "ModelViewProjectionMatrix", mvp.ptr());

    GPU_blend(GPU_BLEND_ALPHA);
    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
    GPU_depth_mask(false);
    GPU_line_width(2.0f);

    GPU_batch_set_shader(scene_bounds_batch_, shader);
    GPU_batch_draw(scene_bounds_batch_);

    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
    GPU_depth_mask(true);
    GPU_blend(GPU_BLEND_NONE);
    GPU_line_width(1.0f);
    GPU_shader_unbind();
  }

  void draw_debug_bvh()
  {
    if (debug_grid_ != 3) {
      return;
    }

    /* Rebuild batch if settings changed or batch was invalidated. */
    if (bvh_batch_ == nullptr || grid_batch_mode_ != debug_grid_) {
      rebuild_grid_batch();
    }
    if (bvh_batch_ == nullptr) {
      return;
    }

    /* Use builtin flat-color shader (per-vertex color). */
    gpu::Shader *shader = GPU_shader_get_builtin_shader(GPU_SHADER_3D_FLAT_COLOR);
    GPU_shader_bind(shader);

    /* MVP = persmat (lines are in world space). */
    View &view = View::default_get();
    float4x4 mvp = view.persmat();
    GPU_shader_uniform_mat4(shader, "ModelViewProjectionMatrix", mvp.ptr());

    GPU_blend(GPU_BLEND_ALPHA);
    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
    GPU_depth_mask(false);
    GPU_line_width(1.5f);

    GPU_batch_set_shader(bvh_batch_, shader);
    GPU_batch_draw(bvh_batch_);

    /* Restore state. */
    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
    GPU_depth_mask(true);
    GPU_blend(GPU_BLEND_NONE);
    GPU_line_width(1.0f);
    GPU_shader_unbind();
  }

  void process_grid_object(const Object *ob)
  {
    if (!ob->runtime || !ob->runtime->geometry_set_eval) {
      return;
    }

    const bke::GeometrySet &geo = *ob->runtime->geometry_set_eval;
    const Volume *volume = geo.get_volume();
    if (!volume || BKE_volume_num_grids(volume) == 0) {
      return;
    }

    const bke::VolumeGridData *vgrid = BKE_volume_grid_get(volume, 0);
    if (!vgrid) {
      return;
    }

    /* Extract dense float voxels. */
    DenseFloatVolumeGrid dense = {};
    if (!BKE_volume_grid_dense_floats(volume, vgrid, &dense)) {
      return;
    }

    int w = dense.resolution[0];
    int h = dense.resolution[1];
    int d = dense.resolution[2];
    if (w <= 0 || h <= 0 || d <= 0) {
      BKE_volume_dense_float_grid_clear(&dense);
      return;
    }

    /* Copy the texture-to-object matrix before we free the dense data. */
    float4x4 tex_to_obj;
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        tex_to_obj[i][j] = dense.texture_to_object[i][j];
      }
    }

    /* Create GPU 3D texture from dense data (R32F for full precision). */
    eGPUTextureUsage usage = GPU_TEXTURE_USAGE_SHADER_READ;
    gpu::Texture *tex = GPU_texture_create_3d(
        "sdf_grid", w, h, d, 1, gpu::TextureFormat::SFLOAT_32, usage, dense.voxels);
    BKE_volume_dense_float_grid_clear(&dense);

    if (!tex) {
      return;
    }
    GPU_texture_filter_mode(tex, true);

    /* Compute world_to_texture transform.
     * DenseFloatVolumeGrid::texture_to_object maps normalized [0,1]^3 -> object space.
     * We need: world_to_texture = inverse(object_to_world * texture_to_object) */
    float4x4 tex_to_world = ob->object_to_world() * tex_to_obj;
    float4x4 world_to_tex = math::invert(tex_to_world);

    /* Scene AABB is accumulated in object_sync() via BKE_object_boundbox_get().
     * Do NOT expand it here — this function only runs when grids change,
     * so expanding here creates AABB inconsistency between bake frames (larger)
     * and cached frames (smaller), causing objects to appear at wrong sizes. */

    /* Read blend from the "Mesh to SDF Grid" modifier's IDProperties. */
    float blend_val = 0.0f;
    LISTBASE_FOREACH (ModifierData *, md, &ob->modifiers) {
      if (md->type != eModifierType_Nodes) {
        continue;
      }
      NodesModifierData *nmd = reinterpret_cast<NodesModifierData *>(md);
      if (!nmd->node_group) {
        continue;
      }
      /* Check if this is our "Mesh to SDF Grid" node group. */
      const char *group_name = nmd->node_group->id.name + 2; /* Skip "NT" prefix. */
      if (std::strcmp(group_name, "Mesh to SDF Grid") != 0) {
        continue;
      }
      /* Find the "Blend" socket and read its value from IDProperties. */
      if (nmd->settings.properties) {
        /* The socket identifier for the Blend input. We iterate interface_inputs
         * to find it, but as a simpler approach, just search all properties for
         * one named with the identifier that has a float type. The Blend socket
         * is the 4th input (index 3), but socket identifiers are assigned by
         * Blender and may vary. Search by iterating all float properties and
         * matching the node group's interface. */
        const bNodeTreeInterface &iface = nmd->node_group->tree_interface;
        iface.foreach_item([&](const bNodeTreeInterfaceItem &item) {
          if (item.item_type != NODE_INTERFACE_SOCKET) {
            return true; /* Continue iteration. */
          }
          const auto &socket = reinterpret_cast<const bNodeTreeInterfaceSocket &>(item);
          if (StringRef(socket.name) == "Blend" &&
              StringRef(socket.socket_type) == "NodeSocketFloat")
          {
            IDProperty *prop = IDP_GetPropertyFromGroup(nmd->settings.properties,
                                                        socket.identifier);
            if (prop && prop->type == IDP_FLOAT) {
              blend_val = IDP_float_get(prop);
            }
            else if (prop && prop->type == IDP_DOUBLE) {
              blend_val = float(IDP_double_get(prop));
            }
            return false; /* Stop iteration. */
          }
          return true; /* Continue iteration. */
        });
      }
      break;
    }

    /* Read object color. */
    float4 color = float4(ob->color[0], ob->color[1], ob->color[2], ob->color[3]);

    GridObject grid_obj = {};
    grid_obj.texture = tex;
    grid_obj.world_to_texture = world_to_tex;
    grid_obj.color = color;
    grid_obj.blend = blend_val;
    grid_obj.blend_type = 0;    /* Default to linear for grid objects. */
    grid_obj.csg_operation = 0; /* Default to union for grid objects. */
    grid_obj.shell_distance = 0.0f;
    grid_obj.shell_mode = 0;

    /* If the object has SDF data, use its blend_type and csg_operation. */
    if (ob->type == OB_SDF && ob->data) {
      const SDF *sdf_data = static_cast<const SDF *>(ob->data);
      grid_obj.blend_type = sdf_data->blend_type;
      grid_obj.csg_operation = sdf_data->csg_operation;
      grid_obj.shell_distance = sdf_data->shell_distance;
      grid_obj.shell_mode = sdf_data->shell_mode;
    }

    grid_objects_.append(grid_obj);
  }

  void free_grid_objects()
  {
    for (GridObject &grid : grid_objects_) {
      if (grid.texture) {
        GPU_texture_free(grid.texture);
      }
    }
    grid_objects_.clear();
  }

  void clear_indirection()
  {
    /* Initialize all bricks as outside (-1).
     * Use GPU_texture_clear instead of allocating a CPU-side vector
     * (which could be up to 8MB at grid_res 128^3). */
    int neg_one = -1;
    GPU_texture_clear(indirection_tx_, GPU_DATA_INT, &neg_one);
    GPU_memory_barrier(GPU_BARRIER_TEXTURE_FETCH);
    active_brick_count_ = 0;
  }

  void augment_indirection_for_grids()
  {
    if (augment_grids_sh_ == nullptr || indirection_tx_ == nullptr || active_bricks_ == nullptr) {
      return;
    }

    float brick_world = voxel_size_ * float(SDF_BRICK_SIZE);

    GPU_shader_bind(augment_grids_sh_);

    int counter_slot = GPU_shader_get_ssbo_binding(augment_grids_sh_, "brick_counter");
    GPU_storagebuf_bind(brick_counter_, counter_slot);

    int ab_slot = GPU_shader_get_ssbo_binding(augment_grids_sh_, "active_bricks");
    GPU_storagebuf_bind(active_bricks_, ab_slot);

    GPU_texture_image_bind(indirection_tx_, 0);

    GPU_shader_uniform_3iv(augment_grids_sh_, "grid_resolution", grid_res_);
    GPU_shader_uniform_1i(augment_grids_sh_, "max_active_bricks", active_bricks_capacity_);

    for (const GridObject &grid : grid_objects_) {
      float4x4 tex_to_world = math::invert(grid.world_to_texture);
      float3 gmin = float3(1e30f);
      float3 gmax = float3(-1e30f);
      for (int c = 0; c < 8; c++) {
        float3 tc = float3(
            (c & 1) ? 1.0f : 0.0f, (c & 2) ? 1.0f : 0.0f, (c & 4) ? 1.0f : 0.0f);
        float3 wc = math::transform_point(tex_to_world, tc);
        gmin = math::min(gmin, wc);
        gmax = math::max(gmax, wc);
      }

      int3 bmin = int3(math::floor((gmin - atlas_origin_) / brick_world));
      int3 bmax = int3(math::ceil((gmax - atlas_origin_) / brick_world));
      bmin = math::max(bmin, int3(0));
      bmax = math::min(bmax, grid_res_);

      int3 range = bmax - bmin;
      if (range.x <= 0 || range.y <= 0 || range.z <= 0) {
        continue;
      }

      GPU_shader_uniform_3iv(augment_grids_sh_, "grid_brick_min", bmin);
      GPU_shader_uniform_3iv(augment_grids_sh_, "grid_brick_max", bmax);

      GPU_compute_dispatch(augment_grids_sh_,
                           divide_ceil_u(range.x, 4),
                           divide_ceil_u(range.y, 4),
                           divide_ceil_u(range.z, 4));

      GPU_memory_barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS | GPU_BARRIER_SHADER_STORAGE);
    }

    GPU_texture_image_unbind(indirection_tx_);
    GPU_shader_unbind();

    /* Read back only the 16-byte counter to get updated active_brick_count_. */
    BrickCounter readback = {};
    GPU_storagebuf_read(brick_counter_, &readback);
    active_brick_count_ = int(readback.count);
    total_allocated_slots_ = active_brick_count_;
  }

  void clear_compact_atlas()
  {
    /* For grid-only scenes, initialize the compact atlas to large distance (1e10)
     * so that grid blend's min-union works correctly.
     * R = 1e10 (large distance), GBA = 0 (no color). */
    if (!compact_atlas_tx_) {
      return;
    }
    float clear_val[4] = {1e10f, 0.0f, 0.0f, 0.0f};
    GPU_texture_clear(compact_atlas_tx_, GPU_DATA_FLOAT, clear_val);
    GPU_memory_barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS);
  }

  void dispatch_grid_blends()
  {
    if (grid_blend_sh_ == nullptr || compact_atlas_tx_ == nullptr || active_brick_count_ <= 0 ||
        active_bricks_ == nullptr)
    {
      return;
    }

    GPU_shader_bind(grid_blend_sh_);

    /* Cache binding slots (avoids repeated string lookups). */
    int ab_slot = GPU_shader_get_ssbo_binding(grid_blend_sh_, "active_bricks");
    int grid_sampler_slot = GPU_shader_get_sampler_binding(grid_blend_sh_, "sdf_grid");

    /* Bind resources that stay constant across all grids. */
    GPU_storagebuf_bind(active_bricks_, ab_slot);
    if (modifier_ssbo_) {
      int mod_slot = GPU_shader_get_ssbo_binding(grid_blend_sh_, "sdf_modifiers");
      GPU_storagebuf_bind(modifier_ssbo_, mod_slot);
    }
    GPU_texture_image_bind(compact_atlas_tx_, 0);

    /* Invariant push constants. */
    GPU_shader_uniform_1f(grid_blend_sh_, "voxel_size", voxel_size_);
    GPU_shader_uniform_3fv(grid_blend_sh_, "atlas_origin", atlas_origin_);
    GPU_shader_uniform_1i(grid_blend_sh_, "bricks_per_axis", bricks_per_axis_);
    GPU_shader_uniform_1i(grid_blend_sh_, "active_brick_count", active_brick_count_);

    uint gb_x = uint(math::min(active_brick_count_, 65535));
    uint gb_y = uint(divide_ceil_u(active_brick_count_, 65535));
    GPU_shader_uniform_1i(grid_blend_sh_, "dispatch_width", int(gb_x));

    for (const GridObject &grid : grid_objects_) {
      GPU_texture_bind(grid.texture, grid_sampler_slot);

      /* Per-grid push constants only. */
      GPU_shader_uniform_mat4(
          grid_blend_sh_, "grid_world_to_texture", grid.world_to_texture.ptr());
      GPU_shader_uniform_4fv(grid_blend_sh_, "grid_color", grid.color);
      GPU_shader_uniform_1f(grid_blend_sh_, "grid_blend", grid.blend);
      GPU_shader_uniform_1i(grid_blend_sh_, "grid_blend_type", grid.blend_type);
      GPU_shader_uniform_1i(grid_blend_sh_, "grid_csg_operation", grid.csg_operation);
      GPU_shader_uniform_1f(grid_blend_sh_, "grid_shell_distance", grid.shell_distance);
      GPU_shader_uniform_1i(grid_blend_sh_, "grid_shell_mode", grid.shell_mode);

      GPU_compute_dispatch(grid_blend_sh_, gb_x, gb_y, 1);

      GPU_memory_barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS);

      GPU_texture_unbind(grid.texture);
    }

    GPU_texture_image_unbind(compact_atlas_tx_);
    GPU_memory_barrier(GPU_BARRIER_TEXTURE_FETCH);
    GPU_shader_unbind();
  }

  /* ---- Performance overlay helpers ---- */

  /** Initialize perf tracking state (idempotent). */
  void perf_init()
  {
    if (perf_queries_created_) {
      return;
    }
    perf_queries_created_ = true;
  }

  /** Update wall-clock FPS rolling average. */
  void perf_begin_frame()
  {
    double now = BLI_time_now_seconds();
    if (perf_last_draw_time_ > 0.0) {
      double dt = now - perf_last_draw_time_;
      if (dt > 0.0) {
        double instant_fps = 1.0 / dt;
        perf_fps_samples_[perf_fps_sample_idx_] = instant_fps;
        perf_fps_sample_idx_ = (perf_fps_sample_idx_ + 1) % PERF_FPS_SAMPLES;
        if (perf_fps_sample_count_ < PERF_FPS_SAMPLES) {
          perf_fps_sample_count_++;
        }
        double sum = 0.0;
        for (int i = 0; i < perf_fps_sample_count_; i++) {
          sum += perf_fps_samples_[i];
        }
        perf_fps_ = sum / double(perf_fps_sample_count_);
        perf_frame_ms_ = dt * 1000.0;
      }
    }
    perf_last_draw_time_ = now;
  }

  /** Begin timing for the given pass. Snapshots wall-clock time. */
  void perf_begin_pass(int pass)
  {
    GPU_finish();
    perf_pass_start_[pass] = BLI_time_now_seconds();
    perf_pass_active_[pass] = true;
  }

  /** End timing for the given pass. GPU_finish ensures the pass completed. */
  void perf_end_pass(int pass)
  {
    GPU_finish();
    double elapsed_ms = (BLI_time_now_seconds() - perf_pass_start_[pass]) * 1000.0;
    switch (pass) {
      case PERF_PASS_CLASSIFY:
        perf_classify_ms_ = elapsed_ms;
        break;
      case PERF_PASS_BAKE:
        perf_bake_ms_ = elapsed_ms;
        break;
      case PERF_PASS_GRID:
        perf_grid_ms_ = elapsed_ms;
        break;
      case PERF_PASS_MARCH:
        perf_march_ms_ = elapsed_ms;
        break;
      case PERF_PASS_FXAA:
        perf_fxaa_ms_ = elapsed_ms;
        break;
    }
  }

  /** Format results and update bake cache. */
  void perf_end_frame(bool /*currently_baking*/)
  {
    perf_prev_baked_ = perf_currently_baking_;
    if (perf_prev_baked_) {
      perf_last_classify_ms_ = perf_classify_ms_;
      perf_last_bake_ms_ = perf_bake_ms_;
      perf_last_grid_ms_ = perf_grid_ms_;
      perf_has_bake_data_ = true;
    }

    /* Format the text. */
    int total_bricks = grid_res_.x * grid_res_.y * grid_res_.z;
    float brick_pct = (total_bricks > 0) ?
                          100.0f * float(active_brick_count_) / float(total_bricks) :
                          0.0f;

    /* Display logic:
     * - perf_prev_baked_: previous frame's readback had bake data → show live timing.
     * - !perf_prev_baked_ && perf_has_bake_data_: idle but have cached values.
     * - Neither: never baked yet → dashes. */
    char classify_str[48], bake_str[48], grid_str[48];
    if (perf_prev_baked_) {
      /* Live data from the frame we just read back. */
      std::snprintf(classify_str, sizeof(classify_str), "%.2f ms", perf_classify_ms_);
      std::snprintf(bake_str, sizeof(bake_str), "%.2f ms", perf_bake_ms_);
      std::snprintf(grid_str, sizeof(grid_str), "%.2f ms", perf_grid_ms_);
    }
    else if (perf_has_bake_data_) {
      /* Idle: show last bake cost so user can still see it. */
      std::snprintf(classify_str, sizeof(classify_str), "%.2f ms", perf_last_classify_ms_);
      std::snprintf(bake_str, sizeof(bake_str), "%.2f ms", perf_last_bake_ms_);
      std::snprintf(grid_str, sizeof(grid_str), "%.2f ms", perf_last_grid_ms_);
    }
    else {
      std::snprintf(classify_str, sizeof(classify_str), "%s", "\xe2\x80\x94");
      std::snprintf(bake_str, sizeof(bake_str), "%s", "\xe2\x80\x94");
      std::snprintf(grid_str, sizeof(grid_str), "%s", "\xe2\x80\x94");
    }

    std::snprintf(s_perf_text,
                  SDF_PERF_BUF_SIZE,
                  "SDF Performance\n"
                  "  fps: %.1f  frame: %.1f ms\n"
                  "  classify: %s\n"
                  "  bake: %s\n"
                  "  grid blend: %s\n"
                  "  march: %.2f ms\n"
                  "  fxaa: %.2f ms\n"
                  "  bricks: %d / %d (%.1f%%)",
                  perf_fps_,
                  perf_frame_ms_,
                  classify_str,
                  bake_str,
                  grid_str,
                  perf_march_ms_,
                  perf_fxaa_ms_,
                  active_brick_count_,
                  total_bricks,
                  brick_pct);
    s_perf_active = true;
  }

  /** Reset perf state. */
  void perf_cleanup()
  {
    perf_queries_created_ = false;
    s_perf_active = false;
    s_perf_text[0] = '\0';
  }

 public:
  ~Instance() override
  {
    /* Cancel any in-flight async shader compilation. */
    if (shader_compile_batch_ != 0) {
      GPU_shader_batch_cancel(shader_compile_batch_);
    }
    for (int i = 0; i < SH_COUNT; i++) {
      GPU_SHADER_FREE_SAFE(shaders_[i]);
    }

    perf_cleanup();
    free_grid_objects();

    /* Clear static pointers BEFORE freeing resources to prevent
     * dangling access from overlay/selection code. */
    s_compact_atlas = nullptr;
    s_indirection = nullptr;

    s_object_ssbo = nullptr;
    s_modifier_ssbo = nullptr;
    s_group_ssbo = nullptr;
    s_group_count = 0;
    s_perf_active = false;
    if (march_color_tx_) {
      GPU_texture_free(march_color_tx_);
    }
    if (march_fb_) {
      GPU_framebuffer_free(march_fb_);
    }
    if (indirection_tx_) {
      GPU_texture_free(indirection_tx_);
    }
    if (compact_atlas_tx_) {
      GPU_texture_free(compact_atlas_tx_);
    }

    if (brick_counter_) {
      GPU_storagebuf_free(brick_counter_);
    }
    if (active_bricks_) {
      GPU_storagebuf_free(active_bricks_);
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
    if (bvh_ssbo_) {
      GPU_storagebuf_free(bvh_ssbo_);
    }
    if (dirty_flags_ssbo_) {
      GPU_storagebuf_free(dirty_flags_ssbo_);
    }
    if (fullscreen_batch_) {
      GPU_batch_discard(fullscreen_batch_);
    }
    if (grid_batch_) {
      GPU_batch_discard(grid_batch_);
    }
    if (bvh_batch_) {
      GPU_batch_discard(bvh_batch_);
    }
    if (scene_bounds_batch_) {
      GPU_batch_discard(scene_bounds_batch_);
    }
  }
};

/* ---- Public perf API ---- */

const char *sdf_perf_info_get()
{
  return s_perf_active ? s_perf_text : nullptr;
}

bool sdf_perf_active()
{
  return s_perf_active;
}

/* ---- Public atlas API ---- */

gpu::Texture *sdf_atlas_get()
{
  return s_compact_atlas;
}

gpu::Texture *sdf_indirection_get()
{
  return s_indirection;
}

void sdf_atlas_params_get(
    float *voxel_size, float3 *origin, float3 *extent, int3 *grid_resolution, int *bricks_per_axis)
{
  *voxel_size = s_voxel_size;
  *origin = s_atlas_origin;
  *extent = s_atlas_extent;
  *grid_resolution = s_grid_resolution;
  *bricks_per_axis = s_bricks_per_axis;
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
