/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw_engine
 *
 * SDF draw engine: bakes SDF objects into a sparse brick atlas, then ray-marches it.
 */

#include "BLI_math_matrix.hh"
#include "BLI_math_vector.hh"

#include "BKE_geometry_set.hh"
#include "BKE_idprop.hh"
#include "BKE_object.hh"
#include "BKE_object_types.hh"
#include "BKE_studiolight.h"
#include "BKE_volume.hh"
#include "BKE_volume_render.hh"

#include "DNA_modifier_types.h"
#include "DNA_node_tree_interface_types.h"
#include "DNA_node_types.h"
#include "DNA_object_types.h"
#include "DNA_sdf_types.h"
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

#include "GPU_context.hh"

#include <epoxy/gl.h>

#include <cmath>
#include <cstdio>
#include <string>

namespace blender::draw::sdf {

using namespace draw;

/* ---- Performance overlay static state ---- */

/** Number of per-pass elapsed-time queries: classify, bake, grid blend, march. */
static constexpr int PERF_PASS_COUNT = 4;
static constexpr int PERF_PASS_CLASSIFY = 0;
static constexpr int PERF_PASS_BAKE = 1;
static constexpr int PERF_PASS_GRID = 2;
static constexpr int PERF_PASS_MARCH = 3;
/** Number of FPS samples for smoothing. */
static constexpr int PERF_FPS_SAMPLES = 8;

/** Formatted performance text, shared with the overlay drawing code. */
static char s_perf_text[SDF_PERF_BUF_SIZE] = "";
/** Whether the performance data is valid for display. */
static bool s_perf_active = false;

class Instance : public DrawEngine {
 private:
  /** Collected SDF objects for this frame. */
  Vector<SDFObjectGPU> objects_;

  /** Scene AABB accumulated from all SDF objects. */
  float3 scene_min_ = float3(1e30f);
  float3 scene_max_ = float3(-1e30f);

  /** Brick grid indirection texture (R32I, grid_res^3). */
  gpu::Texture *indirection_tx_ = nullptr;
  /** Compact atlas (RGBA16F, sized for active bricks only). */
  gpu::Texture *compact_atlas_tx_ = nullptr;
  /** Brick counter SSBO (1 uint + padding). */
  gpu::StorageBuf *brick_counter_ = nullptr;

  /** Atlas parameters. */
  float3 atlas_origin_ = float3(0.0f);
  float3 atlas_extent_ = float3(1.0f);
  float voxel_size_ = 1.0f / 256.0f;

  /** Grid resolution in bricks per axis (sdf_resolution / BRICK_SIZE). */
  int grid_res_ = 32;
  /** Total voxel resolution from UI. */
  int sdf_resolution_ = 256;
  /** Active brick count after classify pass. */
  int active_brick_count_ = 0;
  /** Bricks per axis in compact atlas layout. */
  int bricks_per_axis_ = 1;
  /** Debug grid mode from UI. */
  int debug_grid_ = 0;
  /** Surface margin multiplier (1.0 = default, from UI percentage). */
  float surface_margin_ = 1.0f;
  /** Max blend radius across all objects (computed in classify, used in bake). */
  float max_blend_ = 0.0f;

  /** Dirty tracking: hash of object data for the current frame. */
  uint64_t scene_hash_ = 0;
  /** Separate hash for grid objects (computed from lightweight data before
   * the expensive dense-float extraction, so we can skip it when unchanged). */
  uint64_t grid_hash_ = 0;
  bool needs_bake_ = true;

  /** Mesh-to-SDF grid objects pending processing. */
  struct PendingGridObject {
    const Object *ob;
  };
  Vector<PendingGridObject> pending_grid_objects_;

  /** Processed grid objects ready for GPU dispatch. */
  struct GridObject {
    gpu::Texture *texture = nullptr; /* R32F 3D texture */
    float4x4 world_to_texture;      /* Combined transform: world -> [0,1]^3 */
    float4 color;                    /* Object display color */
    float blend;                     /* Blend amount from modifier */
  };
  Vector<GridObject> grid_objects_;

  /** Cached shaders. */
  gpu::Shader *classify_sh_ = nullptr;
  gpu::Shader *bake_sh_ = nullptr;
  gpu::Shader *march_sh_ = nullptr;
  gpu::Shader *grid_blend_sh_ = nullptr;

  /** Object SSBO. */
  gpu::StorageBuf *object_ssbo_ = nullptr;
  int object_ssbo_count_ = 0;

  /** Fullscreen triangle batch (cached). */
  gpu::Batch *fullscreen_batch_ = nullptr;

  /** Debug grid wireframe batch (GPU_PRIM_LINES). */
  gpu::Batch *grid_batch_ = nullptr;
  int grid_batch_mode_ = 0;
  int grid_batch_res_ = 0;

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
  /** Double-buffered GL_TIME_ELAPSED queries: [frame_idx][pass_idx]. */
  GLuint perf_queries_[2][PERF_PASS_COUNT] = {};
  /** Whether queries have been issued for each frame slot. */
  bool perf_queries_valid_[2] = {false, false};
  /** Which passes were active (issued) on each frame slot. */
  bool perf_pass_active_[2][PERF_PASS_COUNT] = {};
  /** Current frame index (alternates 0/1). */
  int perf_frame_idx_ = 0;
  /** Whether bake passes ran on the frame whose results we're reading. */
  bool perf_prev_baked_ = false;
  /** Timing results (from previous frame's queries). */
  double perf_classify_ms_ = 0.0;
  double perf_bake_ms_ = 0.0;
  double perf_grid_ms_ = 0.0;
  double perf_march_ms_ = 0.0;
  double perf_frame_ms_ = 0.0;
  double perf_fps_ = 0.0;
  /** Wall-clock time of last draw() call for FPS. */
  double perf_last_draw_time_ = 0.0;
  /** Circular buffer for FPS smoothing. */
  double perf_fps_samples_[PERF_FPS_SAMPLES] = {};
  int perf_fps_sample_idx_ = 0;
  int perf_fps_sample_count_ = 0;

 public:
  blender::StringRefNull name_get() final
  {
    return "SDF";
  }

  void init() final
  {
    draw_ctx_ = DRW_context_get();
  }

  void begin_sync() final
  {
    objects_.clear();
    pending_grid_objects_.clear();
    scene_min_ = float3(1e30f);
    scene_max_ = float3(-1e30f);
  }

  void object_sync(ObjectRef &ob_ref, Manager & /*manager*/) final
  {
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
              float3 corner = float3((c & 1) ? hi.x : lo.x,
                                     (c & 2) ? hi.y : lo.y,
                                     (c & 4) ? hi.z : lo.z);
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
     * Scale is baked into sdf_size; inverse matrix is rotation-only. */
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

    float bevel = sdf_data->bevel * math::reduce_min(scale);
    gpu_obj.bevel = bevel;

    gpu_obj.blend = sdf_data->blend;

    gpu_obj.color = float4(
        sdf_data->color[0], sdf_data->color[1], sdf_data->color[2], sdf_data->color[3]);

    /* Compute world AABB from scaled size + bevel + blend.
     * Smooth union with factor k can push the iso-surface outward by up to k
     * from either object's surface, so we expand by the full blend value. */
    float3 local_extent = float3(gpu_obj.sdf_size) + float3(bevel + sdf_data->blend);

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

    gpu_obj.bbox_min = float4(world_min, 0.0f);
    gpu_obj.bbox_max = float4(world_max, 0.0f);

    /* Accumulate scene AABB. */
    scene_min_ = math::min(scene_min_, world_min);
    scene_max_ = math::max(scene_max_, world_max);

    objects_.append(gpu_obj);
  }

  void end_sync() final
  {
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
        grid_hash = grid_hash * 6364136223846793005ULL +
                    *reinterpret_cast<const uint32_t *>(&mat[i][0]);
        grid_hash = grid_hash * 6364136223846793005ULL +
                    *reinterpret_cast<const uint32_t *>(&mat[i][1]);
        grid_hash = grid_hash * 6364136223846793005ULL +
                    *reinterpret_cast<const uint32_t *>(&mat[i][2]);
      }
      /* Hash bounds (changes when voxel_size or mesh changes). */
      const std::optional<Bounds<float3>> bounds = BKE_object_boundbox_get(ob);
      if (bounds) {
        grid_hash = grid_hash * 6364136223846793005ULL +
                    *reinterpret_cast<const uint32_t *>(&bounds->min.x);
        grid_hash = grid_hash * 6364136223846793005ULL +
                    *reinterpret_cast<const uint32_t *>(&bounds->max.x);
        grid_hash = grid_hash * 6364136223846793005ULL +
                    *reinterpret_cast<const uint32_t *>(&bounds->min.y);
        grid_hash = grid_hash * 6364136223846793005ULL +
                    *reinterpret_cast<const uint32_t *>(&bounds->max.y);
      }
      /* Hash object color. */
      grid_hash = grid_hash * 6364136223846793005ULL +
                  *reinterpret_cast<const uint32_t *>(&ob->color[0]);
      grid_hash = grid_hash * 6364136223846793005ULL +
                  *reinterpret_cast<const uint32_t *>(&ob->color[1]);
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

    /* Compute scene hash for dirty tracking (analytic + grid combined). */
    uint64_t hash = uint64_t(objects_.size()) + grid_hash;
    for (const SDFObjectGPU &obj : objects_) {
      hash = hash * 6364136223846793005ULL + *reinterpret_cast<const uint32_t *>(&obj.position.x);
      hash = hash * 6364136223846793005ULL + *reinterpret_cast<const uint32_t *>(&obj.position.y);
      hash = hash * 6364136223846793005ULL + *reinterpret_cast<const uint32_t *>(&obj.position.z);
      hash = hash * 6364136223846793005ULL + *reinterpret_cast<const uint32_t *>(&obj.sdf_size.x);
      hash = hash * 6364136223846793005ULL + *reinterpret_cast<const uint32_t *>(&obj.sdf_size.y);
      hash = hash * 6364136223846793005ULL + *reinterpret_cast<const uint32_t *>(&obj.sdf_size.z);
      hash = hash * 6364136223846793005ULL + *reinterpret_cast<const uint32_t *>(&obj.bevel);
      hash = hash * 6364136223846793005ULL + *reinterpret_cast<const uint32_t *>(&obj.blend);
      hash = hash * 6364136223846793005ULL +
             *reinterpret_cast<const uint32_t *>(&obj.inverse_matrix[0][0]);
      hash = hash * 6364136223846793005ULL +
             *reinterpret_cast<const uint32_t *>(&obj.inverse_matrix[1][1]);
      hash = hash * 6364136223846793005ULL +
             *reinterpret_cast<const uint32_t *>(&obj.inverse_matrix[2][2]);
    }
    /* Include surface margin so changes trigger rebake. */
    hash = hash * 6364136223846793005ULL + *reinterpret_cast<const uint32_t *>(&surface_margin_);
    /* Include scene AABB so any bounding box change triggers rebake.
     * This catches cases where object bounds differ between frames
     * (e.g., modifier evaluation changes, object added/removed). */
    hash = hash * 6364136223846793005ULL + *reinterpret_cast<const uint32_t *>(&scene_min_.x);
    hash = hash * 6364136223846793005ULL + *reinterpret_cast<const uint32_t *>(&scene_min_.y);
    hash = hash * 6364136223846793005ULL + *reinterpret_cast<const uint32_t *>(&scene_min_.z);
    hash = hash * 6364136223846793005ULL + *reinterpret_cast<const uint32_t *>(&scene_max_.x);
    hash = hash * 6364136223846793005ULL + *reinterpret_cast<const uint32_t *>(&scene_max_.y);
    hash = hash * 6364136223846793005ULL + *reinterpret_cast<const uint32_t *>(&scene_max_.z);

    if (hash != scene_hash_) {
      scene_hash_ = hash;
      needs_bake_ = true;
    }
    else {
      needs_bake_ = false;
    }

    /* Compute atlas parameters from total voxel resolution. */
    int total_res = grid_res_ * SDF_BRICK_SIZE;

    float3 scene_center = (scene_min_ + scene_max_) * 0.5f;
    float3 scene_size = scene_max_ - scene_min_;
    float margin = math::reduce_max(scene_size) * 0.1f;
    margin = math::max(margin, 0.5f);
    scene_size += float3(margin * 2.0f);

    float max_axis = math::reduce_max(scene_size);
    voxel_size_ = max_axis / float(total_res);
    atlas_origin_ = scene_center - float3(max_axis * 0.5f);
    atlas_extent_ = float3(max_axis);
  }

  void draw(Manager & /*manager*/) final
  {
    if (objects_.is_empty() && grid_objects_.is_empty()) {
      return;
    }

    sync_sdf_settings();
    sync_shading();
    ensure_shaders();

    /* Check if perf overlay is enabled. Only issue GL queries when active. */
    perf_enabled_ = draw_ctx_->v3d &&
                    (draw_ctx_->v3d->overlay.flag & V3D_OVERLAY_SDF_PERF) &&
                    !(draw_ctx_->v3d->flag2 & V3D_HIDE_OVERLAYS);
    if (perf_enabled_) {
      perf_ensure_queries();
      perf_begin_frame();
    }

    /* Bail if critical shaders failed. */
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

    if (!objects_.is_empty()) {
      upload_objects();
    }

    /* Reset per-frame pass activity flags. */
    if (perf_enabled_) {
      for (int i = 0; i < PERF_PASS_COUNT; i++) {
        perf_pass_active_[perf_frame_idx_][i] = false;
      }
    }

    if (needs_bake_) {
      /* Invalidate debug grid batch: atlas geometry may have changed. */
      if (grid_batch_) {
        GPU_batch_discard(grid_batch_);
        grid_batch_ = nullptr;
      }

      /* Phase 1: Classify analytic SDFs (sparse brick allocation). */
      if (perf_enabled_) {
        perf_begin_pass(PERF_PASS_CLASSIFY);
      }
      ensure_indirection();
      if (!objects_.is_empty()) {
        dispatch_classify();
      }
      else {
        clear_indirection();
      }
      if (!grid_objects_.is_empty()) {
        augment_indirection_for_grids();
      }
      if (perf_enabled_) {
        perf_end_pass(PERF_PASS_CLASSIFY);
      }

      ensure_compact_atlas();

      /* Phase 2: Bake analytic SDFs into all active bricks. */
      if (perf_enabled_) {
        perf_begin_pass(PERF_PASS_BAKE);
      }
      if (!objects_.is_empty()) {
        dispatch_bake();
      }
      else {
        clear_compact_atlas();
      }
      if (perf_enabled_) {
        perf_end_pass(PERF_PASS_BAKE);
      }

      /* Blend grid objects into atlas. */
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

    /* Ray march (runs every frame, even when cached). */
    if (perf_enabled_) {
      perf_begin_pass(PERF_PASS_MARCH);
    }
    draw_march();
    if (perf_enabled_) {
      perf_end_pass(PERF_PASS_MARCH);
    }

    /* Skip debug grid in Cycles rendered view — it draws on top of the
     * path-traced image since the SDF draw engine only provides depth. */
    if (!draw_ctx_->v3d || draw_ctx_->v3d->shading.type != OB_RENDER) {
      draw_debug_grid();
    }

    DRW_submission_end();

    if (perf_enabled_) {
      perf_end_frame(needs_bake_);
    }
    else {
      s_perf_active = false;
    }
  }

 private:
  void sync_sdf_settings()
  {
    const View3D *v3d = draw_ctx_->v3d;
    if (v3d == nullptr) {
      return;
    }

    const View3DShading &shading = v3d->shading;

    /* Read resolution setting. Default to 256 if unset (0). */
    int new_res = int(shading.sdf_resolution);
    if (new_res == 0) {
      new_res = 256;
    }
    new_res = math::clamp(new_res, 64, 512);

    int new_grid_res = new_res / SDF_BRICK_SIZE;

    if (new_grid_res != grid_res_) {
      grid_res_ = new_grid_res;
      sdf_resolution_ = new_res;
      /* Resolution changed: free resources and force rebake. */
      if (indirection_tx_) {
        GPU_texture_free(indirection_tx_);
        indirection_tx_ = nullptr;
      }
      if (compact_atlas_tx_) {
        GPU_texture_free(compact_atlas_tx_);
        compact_atlas_tx_ = nullptr;
      }
      needs_bake_ = true;
      scene_hash_ = 0;
    }

    debug_grid_ = int(shading.sdf_debug_grid);

    /* Surface margin: percentage → multiplier. Treat 0 (unset) as 100%. */
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
      if (sl != nullptr) {
        use_specular_ = ((shading.flag & V3D_SHADING_SPECULAR_HIGHLIGHT) &&
                         (sl->flag & STUDIOLIGHT_SPECULAR_HIGHLIGHT_PASS)) ?
                            1 :
                            0;
        for (int i = 0; i < 4; i++) {
          const SolidLight &light = sl->light[i];
          if (light.flag) {
            studio_light_dir_[i] = float4(light.vec[0], light.vec[1], light.vec[2], 0.0f);
            studio_light_col_[i] = float4(
                light.col[0], light.col[1], light.col[2], light.smooth);
            studio_light_spec_[i] = float4(light.spec[0], light.spec[1], light.spec[2], 0.0f);
          }
          else {
            studio_light_dir_[i] = float4(0.0f);
            studio_light_col_[i] = float4(0.0f);
            studio_light_spec_[i] = float4(0.0f);
          }
        }
        studio_ambient_ = float3(
            sl->light_ambient[0], sl->light_ambient[1], sl->light_ambient[2]);
      }
      else {
        studio_light_dir_[0] = float4(math::normalize(float3(0.5f, 0.7f, 1.0f)), 0.0f);
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
    if (classify_sh_ == nullptr) {
      classify_sh_ = GPU_shader_create_from_info_name("sdf_classify");
    }
    if (bake_sh_ == nullptr) {
      bake_sh_ = GPU_shader_create_from_info_name("sdf_bake");
    }
    if (march_sh_ == nullptr) {
      march_sh_ = GPU_shader_create_from_info_name("sdf_march");
    }
    if (grid_blend_sh_ == nullptr) {
      grid_blend_sh_ = GPU_shader_create_from_info_name("sdf_grid_blend");
    }
  }

  void ensure_indirection()
  {
    if (indirection_tx_ != nullptr) {
      return;
    }
    eGPUTextureUsage usage = GPU_TEXTURE_USAGE_SHADER_READ | GPU_TEXTURE_USAGE_SHADER_WRITE;
    indirection_tx_ = GPU_texture_create_3d("sdf_indirection",
                                            grid_res_,
                                            grid_res_,
                                            grid_res_,
                                            1,
                                            gpu::TextureFormat::SINT_32,
                                            usage,
                                            nullptr);
  }

  void ensure_compact_atlas()
  {
    if (compact_atlas_tx_) {
      GPU_texture_free(compact_atlas_tx_);
    }
    compact_atlas_tx_ = nullptr;

    if (active_brick_count_ <= 0) {
      active_brick_count_ = 0;
      bricks_per_axis_ = 1;
      eGPUTextureUsage usage = GPU_TEXTURE_USAGE_SHADER_READ | GPU_TEXTURE_USAGE_SHADER_WRITE;
      compact_atlas_tx_ = GPU_texture_create_3d("sdf_compact_atlas",
                                                SDF_BRICK_STORAGE,
                                                SDF_BRICK_STORAGE,
                                                SDF_BRICK_STORAGE,
                                                1,
                                                gpu::TextureFormat::SFLOAT_16_16_16_16,
                                                usage,
                                                nullptr);
      GPU_texture_filter_mode(compact_atlas_tx_, true);
      return;
    }

    bricks_per_axis_ = int(std::ceil(std::cbrt(double(active_brick_count_))));
    if (bricks_per_axis_ < 1) {
      bricks_per_axis_ = 1;
    }
    int atlas_dim = bricks_per_axis_ * SDF_BRICK_STORAGE;

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
  }

  void dispatch_classify()
  {
    /* Create / reset brick counter SSBO. */
    BrickCounter zero_counter = {};
    zero_counter.count = 0;
    zero_counter._pad0 = 0;
    zero_counter._pad1 = 0;
    zero_counter._pad2 = 0;

    if (brick_counter_ == nullptr) {
      brick_counter_ = GPU_storagebuf_create_ex(
          sizeof(BrickCounter), &zero_counter, GPU_USAGE_DYNAMIC, "sdf_brick_counter");
    }
    else {
      GPU_storagebuf_update(brick_counter_, &zero_counter);
    }

    GPU_shader_bind(classify_sh_);

    /* Bind SSBOs. */
    int obj_slot = GPU_shader_get_ssbo_binding(classify_sh_, "objects");
    GPU_storagebuf_bind(object_ssbo_, obj_slot);

    int counter_slot = GPU_shader_get_ssbo_binding(classify_sh_, "brick_counter");
    GPU_storagebuf_bind(brick_counter_, counter_slot);

    /* Bind indirection texture as image. */
    GPU_texture_image_bind(indirection_tx_, 0);

    /* Push constants. */
    GPU_shader_uniform_1i(classify_sh_, "object_count", int(objects_.size()));
    GPU_shader_uniform_1f(classify_sh_, "voxel_size", voxel_size_);
    GPU_shader_uniform_3fv(classify_sh_, "atlas_origin", atlas_origin_);
    int3 grid_res_v = int3(grid_res_);
    GPU_shader_uniform_3iv(classify_sh_, "grid_resolution", grid_res_v);

    /* Compute max blend across all objects. Smooth union pushes the
     * iso-surface outward by up to k/4 from either object's hard surface. */
    max_blend_ = 0.0f;
    for (const SDFObjectGPU &obj : objects_) {
      max_blend_ = math::max(max_blend_, obj.blend);
    }

    /* Brick half-diagonal: conservative surface test distance.
     * Multiplied by surface_margin_ (UI "Surface Margin" percentage) to
     * widen the band of bricks classified as active near the surface.
     * Add max_blend * 0.25 to account for smooth union outward push. */
    float brick_half_diag = float(SDF_BRICK_SIZE) * voxel_size_ * 0.866025f; /* sqrt(3)/2 */
    brick_half_diag *= surface_margin_;
    brick_half_diag += max_blend_ * 0.25f;
    GPU_shader_uniform_1f(classify_sh_, "brick_half_diag", brick_half_diag);
    GPU_shader_uniform_1f(classify_sh_, "max_blend", max_blend_);

    /* Dispatch: one thread per brick, local group size is 4x4x4. */
    GPU_compute_dispatch(classify_sh_,
                         divide_ceil_u(grid_res_, 4),
                         divide_ceil_u(grid_res_, 4),
                         divide_ceil_u(grid_res_, 4));

    GPU_memory_barrier(GPU_BARRIER_TEXTURE_FETCH | GPU_BARRIER_SHADER_STORAGE);
    GPU_texture_image_unbind(indirection_tx_);
    GPU_shader_unbind();

    /* Readback active brick count from SSBO. */
    BrickCounter readback = {};
    GPU_storagebuf_read(brick_counter_, &readback);
    active_brick_count_ = int(readback.count);
  }

  void dispatch_bake()
  {
    if (active_brick_count_ <= 0) {
      return;
    }

    GPU_shader_bind(bake_sh_);

    /* Bind object SSBO. */
    int ssbo_slot = GPU_shader_get_ssbo_binding(bake_sh_, "objects");
    GPU_storagebuf_bind(object_ssbo_, ssbo_slot);

    /* Bind indirection as sampler. */
    int indir_slot = GPU_shader_get_sampler_binding(bake_sh_, "indirection_tx");
    GPU_texture_bind(indirection_tx_, indir_slot);

    /* Bind compact atlas as image. */
    GPU_texture_image_bind(compact_atlas_tx_, 0);

    /* Push constants. */
    GPU_shader_uniform_1i(bake_sh_, "object_count", int(objects_.size()));
    GPU_shader_uniform_1f(bake_sh_, "voxel_size", voxel_size_);
    GPU_shader_uniform_3fv(bake_sh_, "atlas_origin", atlas_origin_);
    int3 grid_res_v = int3(grid_res_);
    GPU_shader_uniform_3iv(bake_sh_, "grid_resolution", grid_res_v);
    GPU_shader_uniform_1i(bake_sh_, "bricks_per_axis", bricks_per_axis_);
    GPU_shader_uniform_1f(bake_sh_, "max_blend", max_blend_);

    /* Dispatch: one workgroup per brick in the grid. */
    GPU_compute_dispatch(bake_sh_, grid_res_, grid_res_, grid_res_);

    GPU_memory_barrier(GPU_BARRIER_TEXTURE_FETCH);
    GPU_texture_image_unbind(compact_atlas_tx_);
    GPU_texture_unbind(indirection_tx_);
    GPU_shader_unbind();
  }

  void draw_march()
  {
    DefaultFramebufferList *dfbl = draw_ctx_->viewport_framebuffer_list_get();

    if (draw_ctx_->is_depth() ||
        (draw_ctx_->v3d && draw_ctx_->v3d->shading.type == OB_RENDER))
    {
      /* Depth-only: either a depth prepass, or rendered mode where Cycles
       * provides the color and we only contribute depth for the overlay grid. */
      GPU_framebuffer_bind(dfbl->depth_only_fb);
    }
    else {
      /* Solid / wireframe: write both color and depth. */
      GPU_framebuffer_bind(dfbl->default_fb);
    }

    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
    GPU_depth_mask(true);
    GPU_blend(GPU_BLEND_NONE);
    GPU_face_culling(GPU_CULL_NONE);
    GPU_stencil_test(GPU_STENCIL_NONE);

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

    /* Push constants. */
    GPU_shader_uniform_1f(march_sh_, "voxel_size", voxel_size_);
    GPU_shader_uniform_3fv(march_sh_, "atlas_origin", atlas_origin_);
    GPU_shader_uniform_3fv(march_sh_, "atlas_extent", atlas_extent_);
    int3 grid_res_v = int3(grid_res_);
    GPU_shader_uniform_3iv(march_sh_, "grid_resolution", grid_res_v);
    GPU_shader_uniform_1i(march_sh_, "bricks_per_axis", bricks_per_axis_);
    GPU_shader_uniform_1i(march_sh_, "lighting_type", lighting_type_);
    GPU_shader_uniform_1i(march_sh_, "use_specular", use_specular_);
    GPU_shader_uniform_1i(march_sh_, "use_matcap_flip", use_matcap_flip_);

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

  void rebuild_grid_batch_active()
  {
    if (!indirection_tx_) {
      return;
    }

    /* Read back indirection texture to find active bricks. */
    int32_t *data = static_cast<int32_t *>(
        GPU_texture_read(indirection_tx_, GPU_DATA_INT, 0));
    if (!data) {
      return;
    }

    int n = grid_res_;
    float brick_world = float(SDF_BRICK_SIZE) * voxel_size_;

    /* Count active bricks. */
    int total = n * n * n;
    int active_count = 0;
    for (int i = 0; i < total; i++) {
      if (data[i] >= 0) {
        active_count++;
      }
    }

    if (active_count == 0) {
      MEM_freeN(data);
      return;
    }

    /* 12 edges per cube, 2 vertices per edge = 24 vertices per active brick. */
    Vector<float3> positions(active_count * 24);
    int vi = 0;

    for (int z = 0; z < n; z++) {
      for (int y = 0; y < n; y++) {
        for (int x = 0; x < n; x++) {
          int idx = z * n * n + y * n + x;
          if (data[idx] < 0) {
            continue;
          }

          float3 lo = atlas_origin_ + float3(float(x), float(y), float(z)) * brick_world;
          float3 hi = lo + float3(brick_world);

          /* Bottom face edges. */
          positions[vi++] = float3(lo.x, lo.y, lo.z);
          positions[vi++] = float3(hi.x, lo.y, lo.z);
          positions[vi++] = float3(hi.x, lo.y, lo.z);
          positions[vi++] = float3(hi.x, hi.y, lo.z);
          positions[vi++] = float3(hi.x, hi.y, lo.z);
          positions[vi++] = float3(lo.x, hi.y, lo.z);
          positions[vi++] = float3(lo.x, hi.y, lo.z);
          positions[vi++] = float3(lo.x, lo.y, lo.z);

          /* Top face edges. */
          positions[vi++] = float3(lo.x, lo.y, hi.z);
          positions[vi++] = float3(hi.x, lo.y, hi.z);
          positions[vi++] = float3(hi.x, lo.y, hi.z);
          positions[vi++] = float3(hi.x, hi.y, hi.z);
          positions[vi++] = float3(hi.x, hi.y, hi.z);
          positions[vi++] = float3(lo.x, hi.y, hi.z);
          positions[vi++] = float3(lo.x, hi.y, hi.z);
          positions[vi++] = float3(lo.x, lo.y, hi.z);

          /* Vertical edges. */
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

  void rebuild_grid_batch()
  {
    if (grid_batch_) {
      GPU_batch_discard(grid_batch_);
      grid_batch_ = nullptr;
    }

    grid_batch_mode_ = debug_grid_;
    grid_batch_res_ = grid_res_;

    if (debug_grid_ != 0) {
      rebuild_grid_batch_active();
    }
  }

  void draw_debug_grid()
  {
    if (debug_grid_ == 0) {
      return;
    }

    /* Rebuild batch if settings changed. */
    if (grid_batch_ == nullptr || grid_batch_mode_ != debug_grid_ ||
        grid_batch_res_ != grid_res_)
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
    GPU_shader_uniform_4f(shader, "color", 0.0f, 0.6f, 0.0f, 0.1f);
    GPU_batch_draw(grid_batch_);

    /* --- Pass 2: Front lines (in front of SDF surface) ---
     * Bright and slightly thicker for clear readability. */
    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
    GPU_line_width(1.5f);
    GPU_shader_uniform_4f(shader, "color", 0.0f, 1.0f, 0.0f, 0.8f);
    GPU_batch_draw(grid_batch_);

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
    /* Initialize all bricks as outside (-1). */
    int total = grid_res_ * grid_res_ * grid_res_;
    Vector<int32_t> data(total, -1);
    GPU_texture_update(indirection_tx_, GPU_DATA_INT, data.data());
    GPU_memory_barrier(GPU_BARRIER_TEXTURE_FETCH);
    active_brick_count_ = 0;
  }

  void augment_indirection_for_grids()
  {
    /* Read back indirection texture, mark ALL bricks overlapping each grid
     * object's world AABB as active, re-upload. No per-voxel sampling needed —
     * the grid blend shader already skips out-of-bounds and background voxels.
     *
     * Previous approach sampled 9 points per brick which missed most surface
     * bricks on detailed meshes (causing holes). It also read back the entire
     * dense grid from GPU which stalled the pipeline. */
    int32_t *data = static_cast<int32_t *>(
        GPU_texture_read(indirection_tx_, GPU_DATA_INT, 0));

    float brick_world = voxel_size_ * float(SDF_BRICK_SIZE);

    for (const GridObject &grid : grid_objects_) {
      /* Compute grid's world AABB from the inverse of world_to_texture. */
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

      /* Convert world AABB to brick coordinates. */
      int3 bmin = int3(math::floor((gmin - atlas_origin_) / brick_world));
      int3 bmax = int3(math::floor((gmax - atlas_origin_) / brick_world));
      bmin = math::max(bmin, int3(0));
      bmax = math::min(bmax, int3(grid_res_ - 1));

      /* Activate all bricks within the grid's AABB. */
      for (int bz = bmin.z; bz <= bmax.z; bz++) {
        for (int by = bmin.y; by <= bmax.y; by++) {
          for (int bx = bmin.x; bx <= bmax.x; bx++) {
            int idx = bx + by * grid_res_ + bz * grid_res_ * grid_res_;
            if (data[idx] >= 0) {
              continue; /* Already active from analytic classify. */
            }
            data[idx] = active_brick_count_++;
          }
        }
      }
    }

    /* Re-upload modified indirection. */
    GPU_texture_update(indirection_tx_, GPU_DATA_INT, data);
    GPU_memory_barrier(GPU_BARRIER_TEXTURE_FETCH);

    MEM_freeN(data);
  }

  void clear_compact_atlas()
  {
    /* For grid-only scenes, initialize the compact atlas to large distance (1e10)
     * so that grid blend's min-union works correctly. */
    if (!compact_atlas_tx_) {
      return;
    }

    int atlas_dim = bricks_per_axis_ * SDF_BRICK_STORAGE;
    int total_voxels = atlas_dim * atlas_dim * atlas_dim;
    Vector<float> clear_data(total_voxels * 4);
    for (int i = 0; i < total_voxels; i++) {
      clear_data[i * 4 + 0] = 1e10f;  /* distance */
      clear_data[i * 4 + 1] = 0.0f;   /* color.r */
      clear_data[i * 4 + 2] = 0.0f;   /* color.g */
      clear_data[i * 4 + 3] = 0.0f;   /* color.b */
    }
    GPU_texture_update(compact_atlas_tx_, GPU_DATA_FLOAT, clear_data.data());
    GPU_memory_barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS);
  }

  void dispatch_grid_blends()
  {
    if (grid_blend_sh_ == nullptr || compact_atlas_tx_ == nullptr || indirection_tx_ == nullptr) {
      return;
    }

    GPU_shader_bind(grid_blend_sh_);

    for (const GridObject &grid : grid_objects_) {
      /* Bind compact atlas as read-write image. */
      GPU_texture_image_bind(compact_atlas_tx_, 0);

      /* Bind indirection as sampler. */
      int indir_slot = GPU_shader_get_sampler_binding(grid_blend_sh_, "indirection_tx");
      GPU_texture_bind(indirection_tx_, indir_slot);

      /* Bind grid texture as sampler. */
      int grid_slot = GPU_shader_get_sampler_binding(grid_blend_sh_, "sdf_grid");
      GPU_texture_bind(grid.texture, grid_slot);

      /* Push constants. */
      GPU_shader_uniform_1f(grid_blend_sh_, "voxel_size", voxel_size_);
      GPU_shader_uniform_3fv(grid_blend_sh_, "atlas_origin", atlas_origin_);
      int3 grid_res_v = int3(grid_res_);
      GPU_shader_uniform_3iv(grid_blend_sh_, "grid_resolution", grid_res_v);
      GPU_shader_uniform_1i(grid_blend_sh_, "bricks_per_axis", bricks_per_axis_);
      GPU_shader_uniform_mat4(
          grid_blend_sh_, "grid_world_to_texture", grid.world_to_texture.ptr());
      GPU_shader_uniform_4fv(grid_blend_sh_, "grid_color", grid.color);
      GPU_shader_uniform_1f(grid_blend_sh_, "grid_blend", grid.blend);

      /* Dispatch: one workgroup per brick in the grid. */
      GPU_compute_dispatch(grid_blend_sh_, grid_res_, grid_res_, grid_res_);

      /* Barrier between dispatches so next grid reads the updated atlas. */
      GPU_memory_barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS);

      GPU_texture_unbind(grid.texture);
      GPU_texture_unbind(indirection_tx_);
      GPU_texture_image_unbind(compact_atlas_tx_);
    }

    GPU_memory_barrier(GPU_BARRIER_TEXTURE_FETCH);
    GPU_shader_unbind();
  }

  /* ---- Performance overlay helpers ---- */

  /** Lazily create GL elapsed-time query objects. Only on OpenGL backend. */
  void perf_ensure_queries()
  {
    if (perf_queries_created_) {
      return;
    }
    if (GPU_backend_get_type() != GPU_BACKEND_OPENGL) {
      return;
    }
    for (int f = 0; f < 2; f++) {
      glGenQueries(PERF_PASS_COUNT, perf_queries_[f]);
    }
    perf_queries_created_ = true;
  }

  /** Read back the previous frame's query results (non-blocking) and update FPS. */
  void perf_begin_frame()
  {
    /* Compute wall-clock FPS. */
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

    if (!perf_queries_created_) {
      return;
    }

    /* Read back previous frame's elapsed-time results (non-blocking). */
    int prev = 1 - perf_frame_idx_;
    if (!perf_queries_valid_[prev]) {
      return;
    }

    /* Check if the march query (always issued) is ready. */
    GLint available = 0;
    glGetQueryObjectiv(
        perf_queries_[prev][PERF_PASS_MARCH], GL_QUERY_RESULT_AVAILABLE, &available);
    if (!available) {
      return;
    }

    /* Read each pass that was active on the previous frame. */
    auto read_pass = [&](int pass, double &out_ms) {
      if (perf_pass_active_[prev][pass]) {
        GLuint64 elapsed_ns = 0;
        glGetQueryObjectui64v(perf_queries_[prev][pass], GL_QUERY_RESULT, &elapsed_ns);
        out_ms = double(elapsed_ns) * 1e-6;
      }
      else {
        out_ms = 0.0;
      }
    };
    read_pass(PERF_PASS_CLASSIFY, perf_classify_ms_);
    read_pass(PERF_PASS_BAKE, perf_bake_ms_);
    read_pass(PERF_PASS_GRID, perf_grid_ms_);
    read_pass(PERF_PASS_MARCH, perf_march_ms_);
  }

  /** Begin a GL_TIME_ELAPSED query for the given pass. */
  void perf_begin_pass(int pass)
  {
    if (!perf_queries_created_) {
      return;
    }
    glBeginQuery(GL_TIME_ELAPSED, perf_queries_[perf_frame_idx_][pass]);
    perf_pass_active_[perf_frame_idx_][pass] = true;
  }

  /** End the current GL_TIME_ELAPSED query. */
  void perf_end_pass(int /*pass*/)
  {
    if (!perf_queries_created_) {
      return;
    }
    glEndQuery(GL_TIME_ELAPSED);
  }

  /** Format results and swap frame index. */
  void perf_end_frame(bool baked)
  {
    perf_queries_valid_[perf_frame_idx_] = true;
    perf_prev_baked_ = baked;
    perf_frame_idx_ = 1 - perf_frame_idx_;

    /* Format the text. */
    int total_bricks = grid_res_ * grid_res_ * grid_res_;
    float brick_pct = (total_bricks > 0) ?
                          100.0f * float(active_brick_count_) / float(total_bricks) :
                          0.0f;

    char classify_str[32], bake_str[32], grid_str[32];
    if (perf_prev_baked_) {
      std::snprintf(classify_str, sizeof(classify_str), "%.2f ms", perf_classify_ms_);
      std::snprintf(bake_str, sizeof(bake_str), "%.2f ms", perf_bake_ms_);
      std::snprintf(grid_str, sizeof(grid_str), "%.2f ms", perf_grid_ms_);
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
                  "  bricks: %d / %d (%.1f%%)",
                  perf_fps_,
                  perf_frame_ms_,
                  classify_str,
                  bake_str,
                  grid_str,
                  perf_march_ms_,
                  active_brick_count_,
                  total_bricks,
                  brick_pct);
    s_perf_active = true;
  }

  /** Delete GL query objects. */
  void perf_cleanup()
  {
    if (!perf_queries_created_) {
      return;
    }
    for (int f = 0; f < 2; f++) {
      glDeleteQueries(PERF_PASS_COUNT, perf_queries_[f]);
    }
    perf_queries_created_ = false;
    s_perf_active = false;
    s_perf_text[0] = '\0';
  }

 public:
  ~Instance() override
  {
    perf_cleanup();
    free_grid_objects();
    if (indirection_tx_) {
      GPU_texture_free(indirection_tx_);
    }
    if (compact_atlas_tx_) {
      GPU_texture_free(compact_atlas_tx_);
    }
    if (brick_counter_) {
      GPU_storagebuf_free(brick_counter_);
    }
    if (matcap_tx_) {
      GPU_texture_free(matcap_tx_);
    }
    if (object_ssbo_) {
      GPU_storagebuf_free(object_ssbo_);
    }
    if (fullscreen_batch_) {
      GPU_batch_discard(fullscreen_batch_);
    }
    if (grid_batch_) {
      GPU_batch_discard(grid_batch_);
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

DrawEngine *Engine::create_instance()
{
  return new Instance();
}

}  // namespace blender::draw::sdf
