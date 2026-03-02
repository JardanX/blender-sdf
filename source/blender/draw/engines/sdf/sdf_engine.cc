/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw_engine
 *
 * SDF draw engine: bakes SDF objects into a 3D atlas, then ray-marches it.
 */

#include "BLI_math_matrix.hh"
#include "BLI_math_vector.hh"

#include "BKE_studiolight.h"

#include "DNA_object_types.h"
#include "DNA_sdf_types.h"
#include "DNA_view3d_enums.h"
#include "DNA_view3d_types.h"

#include "IMB_imbuf_types.hh"

#include "GPU_batch.hh"
#include "GPU_compute.hh"
#include "GPU_framebuffer.hh"
#include "GPU_shader.hh"
#include "GPU_state.hh"
#include "GPU_storage_buffer.hh"
#include "GPU_texture.hh"
#include "GPU_uniform_buffer.hh"

#include "draw_manager.hh"
#include "draw_view.hh"
#include "draw_view_data.hh"

#include "sdf_private.hh"

#include "sdf_engine.h" /* Own include. */

#include <cstdio>
#include <string>

namespace blender::draw::sdf {

using namespace draw;

class Instance : public DrawEngine {
 private:
  /** Collected SDF objects for this frame. */
  Vector<SDFObjectGPU> objects_;

  /** Scene AABB accumulated from all SDF objects. */
  float3 scene_min_ = float3(1e30f);
  float3 scene_max_ = float3(-1e30f);

  /** Dense 3D atlas (R16F). */
  gpu::Texture *atlas_tx_ = nullptr;

  /** Atlas parameters. */
  float3 atlas_origin_ = float3(0.0f);
  float3 atlas_extent_ = float3(1.0f);
  float voxel_size_ = 1.0f / SDF_ATLAS_RES;

  /** Dirty tracking: hash of object data for the current frame. */
  uint64_t scene_hash_ = 0;
  bool needs_bake_ = true;

  /** Cached shaders. */
  gpu::Shader *bake_sh_ = nullptr;
  gpu::Shader *march_sh_ = nullptr;

  /** Object SSBO. */
  gpu::StorageBuf *object_ssbo_ = nullptr;
  int object_ssbo_count_ = 0;

  /** Fullscreen triangle batch (cached). */
  gpu::Batch *fullscreen_batch_ = nullptr;

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
    scene_min_ = float3(1e30f);
    scene_max_ = float3(-1e30f);
  }

  void object_sync(ObjectRef &ob_ref, Manager & /*manager*/) final
  {
    Object *ob = ob_ref.object;
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

    /* Compute world AABB from scaled size + bevel. */
    float3 local_extent = float3(gpu_obj.sdf_size) + float3(bevel);

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
    if (objects_.is_empty()) {
      needs_bake_ = false;
      return;
    }

    /* Compute scene hash for dirty tracking. */
    uint64_t hash = uint64_t(objects_.size());
    for (const SDFObjectGPU &obj : objects_) {
      /* Hash the essential fields that affect the baked atlas. */
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

    if (hash != scene_hash_) {
      scene_hash_ = hash;
      needs_bake_ = true;
    }
    else {
      needs_bake_ = false;
    }

    /* Compute atlas parameters. */
    float3 scene_center = (scene_min_ + scene_max_) * 0.5f;
    float3 scene_size = scene_max_ - scene_min_;
    /* Add margin so objects at the boundary are fully captured. */
    float margin = math::reduce_max(scene_size) * 0.1f;
    margin = math::max(margin, 0.5f);
    scene_size += float3(margin * 2.0f);

    /* Use uniform voxel size based on the largest axis. */
    float max_axis = math::reduce_max(scene_size);
    voxel_size_ = max_axis / float(SDF_ATLAS_RES);
    atlas_origin_ = scene_center - float3(max_axis * 0.5f);
    atlas_extent_ = float3(max_axis);
  }

  void draw(Manager & /*manager*/) final
  {
    if (objects_.is_empty()) {
      return;
    }

    sync_shading();
    ensure_shaders();

    /* Bail if shader compilation failed. */
    if (bake_sh_ == nullptr || march_sh_ == nullptr) {
      std::printf("[SDF] Shader compilation FAILED: bake=%p march=%p\n",
                  (void *)bake_sh_,
                  (void *)march_sh_);
      return;
    }

    if (needs_bake_) {
      std::printf("[SDF] draw(): %d objects, voxel=%.4f, "
                  "origin=(%.2f,%.2f,%.2f), extent=(%.2f,%.2f,%.2f)\n",
                  int(objects_.size()),
                  voxel_size_,
                  atlas_origin_.x,
                  atlas_origin_.y,
                  atlas_origin_.z,
                  atlas_extent_.x,
                  atlas_extent_.y,
                  atlas_extent_.z);
    }

    DRW_submission_start();

    ensure_atlas();
    upload_objects();

    if (needs_bake_) {
      dispatch_bake();
      std::printf("[SDF] Bake dispatched (%d x %d x %d)\n",
                  SDF_ATLAS_RES,
                  SDF_ATLAS_RES,
                  SDF_ATLAS_RES);
    }

    draw_march();

    DRW_submission_end();
  }

 private:
  void sync_shading()
  {
    const View3D *v3d = draw_ctx_->v3d;
    if (v3d == nullptr) {
      /* No 3D viewport (e.g. final render): default to flat. */
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
      /* STUDIO: extract 4 directional lights + ambient from studio light.
       * Light directions (sl->vec) are in view space (camera-relative). The
       * shader transforms the world-space normal to view space to match. */
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
            /* Pack wrap factor (smooth) into color.w for the shader. */
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
        /* No studio light: use a reasonable default. */
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
      /* MATCAP: find matcap studio light and build 2-layer array texture. */
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

      /* Build matcap texture if the matcap changed. */
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

          /* Free old texture. */
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

      /* Ensure a fallback texture exists for matcap mode. */
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
    if (bake_sh_ == nullptr) {
      bake_sh_ = GPU_shader_create_from_info_name("sdf_bake");
    }
    if (march_sh_ == nullptr) {
      march_sh_ = GPU_shader_create_from_info_name("sdf_march");
    }
  }

  void ensure_atlas()
  {
    if (atlas_tx_ != nullptr) {
      return;
    }
    eGPUTextureUsage usage = GPU_TEXTURE_USAGE_SHADER_READ | GPU_TEXTURE_USAGE_SHADER_WRITE;
    atlas_tx_ = GPU_texture_create_3d("sdf_atlas",
                                      SDF_ATLAS_RES,
                                      SDF_ATLAS_RES,
                                      SDF_ATLAS_RES,
                                      1,
                                      gpu::TextureFormat::SFLOAT_16_16_16_16,
                                      usage,
                                      nullptr);
    GPU_texture_filter_mode(atlas_tx_, true);
  }

  void upload_objects()
  {
    const int count = int(objects_.size());
    const size_t buf_size = count * sizeof(SDFObjectGPU);

    /* Recreate SSBO if count changed (update only works within same size). */
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

  void dispatch_bake()
  {
    GPU_shader_bind(bake_sh_);

    /* Bind object SSBO. */
    int ssbo_slot = GPU_shader_get_ssbo_binding(bake_sh_, "objects");
    GPU_storagebuf_bind(object_ssbo_, ssbo_slot);

    /* Bind atlas as image (slot 0 from ShaderCreateInfo). */
    GPU_texture_image_bind(atlas_tx_, 0);

    /* Push constants. */
    GPU_shader_uniform_1i(bake_sh_, "object_count", int(objects_.size()));
    GPU_shader_uniform_1f(bake_sh_, "voxel_size", voxel_size_);
    GPU_shader_uniform_3fv(bake_sh_, "atlas_origin", atlas_origin_);
    int3 res = int3(SDF_ATLAS_RES);
    GPU_shader_uniform_3iv(bake_sh_, "atlas_resolution", res);

    /* Dispatch. */
    GPU_compute_dispatch(bake_sh_,
                         divide_ceil_u(SDF_ATLAS_RES, 8),
                         divide_ceil_u(SDF_ATLAS_RES, 8),
                         divide_ceil_u(SDF_ATLAS_RES, 4));

    GPU_memory_barrier(GPU_BARRIER_TEXTURE_FETCH);
    GPU_texture_image_unbind(atlas_tx_);
    GPU_shader_unbind();
  }

  void draw_march()
  {
    DefaultFramebufferList *dfbl = draw_ctx_->viewport_framebuffer_list_get();

    if (draw_ctx_->is_depth()) {
      /* Depth-only mode (3D cursor, orbit pivot, etc.): write depth only.
       * Uses the same viewport depth texture as default_fb. */
      GPU_framebuffer_bind(dfbl->depth_only_fb);
    }
    else {
      /* Normal rendering: write both color and depth. */
      GPU_framebuffer_bind(dfbl->default_fb);
    }

    /* Explicitly reset all GPU state that Workbench may have left dirty. */
    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
    GPU_depth_mask(true);
    GPU_blend(GPU_BLEND_NONE);
    GPU_face_culling(GPU_CULL_NONE);
    GPU_stencil_test(GPU_STENCIL_NONE);

    GPU_shader_bind(march_sh_);

    /* Bind atlas as sampler. */
    int atlas_slot = GPU_shader_get_sampler_binding(march_sh_, "sdf_atlas");
    GPU_texture_bind(atlas_tx_, atlas_slot);

    /* NOTE: We intentionally don't bind depth_tx for reading because the
     * framebuffer's depth attachment IS the same texture — simultaneous
     * read+write is a texture feedback loop (UB in OpenGL).
     * TODO: Copy depth to a separate texture for proper occlusion. */

    /* Push constants. */
    GPU_shader_uniform_1f(march_sh_, "voxel_size", voxel_size_);
    GPU_shader_uniform_3fv(march_sh_, "atlas_origin", atlas_origin_);
    GPU_shader_uniform_3fv(march_sh_, "atlas_extent", atlas_extent_);
    int3 res = int3(SDF_ATLAS_RES);
    GPU_shader_uniform_3iv(march_sh_, "atlas_resolution", res);
    GPU_shader_uniform_1i(march_sh_, "lighting_type", lighting_type_);
    GPU_shader_uniform_1i(march_sh_, "use_specular", use_specular_);
    GPU_shader_uniform_1i(march_sh_, "use_matcap_flip", use_matcap_flip_);

    /* Studio light data (used when lighting_type == STUDIO). */
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

    /* Bind matcap texture (used when lighting_type == MATCAP). */
    if (matcap_tx_) {
      int matcap_slot = GPU_shader_get_sampler_binding(march_sh_, "matcap_tx");
      GPU_texture_bind(matcap_tx_, matcap_slot);
    }

    /* Bind the view UBO so the fragment shader can access camera matrices. */
    View &view = View::default_get();
    GPU_uniformbuf_bind(view.matrices_ubo_get(), DRW_VIEW_UBO_SLOT);

    /* Draw fullscreen triangle (3 vertices, vertex shader generates positions from gl_VertexID). */
    if (fullscreen_batch_ == nullptr) {
      fullscreen_batch_ = GPU_batch_create_procedural(GPU_PRIM_TRIS, 3);
    }
    GPU_batch_set_shader(fullscreen_batch_, march_sh_);
    GPU_batch_draw(fullscreen_batch_);

    /* NOTE: No GPU sync needed here. gl_FragDepth framebuffer writes are
     * automatically coherent with subsequent texture reads in OpenGL.
     * GPU_flush() and GPU_finish() were both tested and did NOT fix the
     * grid-on-top-during-zoom issue — root cause is not synchronization. */

    GPU_texture_unbind(atlas_tx_);
    if (matcap_tx_) {
      GPU_texture_unbind(matcap_tx_);
    }
    GPU_shader_unbind();
  }

 public:
  ~Instance() override
  {
    if (atlas_tx_) {
      GPU_texture_free(atlas_tx_);
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
  }
};

DrawEngine *Engine::create_instance()
{
  return new Instance();
}

}  // namespace blender::draw::sdf
