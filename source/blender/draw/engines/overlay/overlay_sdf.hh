/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "BLI_assert.h"

#include "DNA_object_types.h"

#include "GPU_batch.hh"
#include "GPU_shader.hh"
#include "GPU_state.hh"
#include "GPU_storage_buffer.hh"
#include "GPU_texture.hh"
#include "GPU_uniform_buffer.hh"

#include "draw_view.hh"

#include "overlay_base.hh"

#include "engines/sdf/sdf_engine.h"
#include "engines/select/select_defines.hh"

namespace blender::draw::overlay {

/**
 * SDF overlay for GPU picking.
 * Draws a fullscreen march through the SDF engine's baked atlas and object ID atlas.
 * On hit, reads the object ID and outputs the corresponding select::ID.
 * Only active during selection draws.
 */
class Sdfs : Overlay {
 private:
  const SelectionType selection_type_;

  /** Maps SDF object index (bake order) -> select::ID for the selection buffer. */
  Vector<uint32_t> select_id_map_;

  /** Cached raw GPU buffer pointers to selection system buffers (from Resources in end_sync). */
  gpu::UniformBuf *select_info_buf_ = nullptr;
  gpu::StorageBuf *select_output_buf_ = nullptr;

  gpu::Shader *select_march_sh_ = nullptr;
  gpu::Batch *fullscreen_batch_ = nullptr;

  /** Temporary SSBO for select_id_map upload (recreated per frame). */
  gpu::StorageBuf *map_ssbo_ = nullptr;

 public:
  Sdfs(const SelectionType selection_type) : selection_type_(selection_type) {};

  void begin_sync(Resources & /*res*/, const State &state) final
  {
    enabled_ = state.is_space_v3d() && (selection_type_ != SelectionType::DISABLED);
    if (!enabled_) {
      return;
    }
    select_id_map_.clear();
  }

  void object_sync(Manager & /*manager*/,
                   const ObjectRef &ob_ref,
                   Resources &res,
                   const State & /*state*/) final
  {
    if (!enabled_) {
      return;
    }
    if (ob_ref.object->type != OB_SDF) {
      return;
    }

    /* Append select ID in same order as SDF engine's objects_ vector.
     * Both iterate depsgraph objects filtering OB_SDF. */
    const select::ID sel_id = res.select_id(ob_ref);
    select_id_map_.append(sel_id.get());
  }

  void end_sync(Resources &res, const State & /*state*/) final
  {
    if (!enabled_) {
      return;
    }
    if (select_id_map_.is_empty()) {
      return;
    }

    /* Check if SDF atlas is available. */
    if (sdf::sdf_atlas_get() == nullptr) {
      return;
    }

    /* Verify ordering matches the SDF engine. */
    BLI_assert(int(select_id_map_.size()) == sdf::sdf_object_count_get());

    if (!select_march_sh_) {
      select_march_sh_ = GPU_shader_create_from_info_name("sdf_select_march");
    }
    if (!select_march_sh_) {
      return;
    }

    /* Capture selection buffer pointers for use in draw().
     * Use implicit conversion operators to get raw GPU pointers. */
    select_info_buf_ = static_cast<gpu::UniformBuf *>(res.info_buf);
    select_output_buf_ = static_cast<gpu::StorageBuf *>(res.select_output_buf);

    /* Create/update the select ID map SSBO. */
    if (map_ssbo_) {
      GPU_storagebuf_free(map_ssbo_);
    }
    map_ssbo_ = GPU_storagebuf_create_ex(select_id_map_.size() * sizeof(uint32_t),
                                         select_id_map_.data(),
                                         GPU_USAGE_STATIC,
                                         "sdf_select_id_map");
  }

  void draw(Framebuffer &framebuffer, Manager & /*manager*/, View &view) final
  {
    if (!enabled_ || select_id_map_.is_empty() || !select_march_sh_) {
      return;
    }
    if (sdf::sdf_atlas_get() == nullptr || !map_ssbo_) {
      return;
    }

    GPU_framebuffer_bind(framebuffer);

    GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
    GPU_depth_mask(true);
    GPU_blend(GPU_BLEND_NONE);

    GPU_shader_bind(select_march_sh_);

    /* Bind SDF atlas textures. */
    int atlas_slot = GPU_shader_get_sampler_binding(select_march_sh_, "compact_atlas");
    GPU_texture_bind(sdf::sdf_atlas_get(), atlas_slot);

    int indir_slot = GPU_shader_get_sampler_binding(select_march_sh_, "indirection_tx");
    GPU_texture_bind(sdf::sdf_indirection_get(), indir_slot);

    int objid_slot = GPU_shader_get_sampler_binding(select_march_sh_, "object_id_tx");
    GPU_texture_bind(sdf::sdf_object_id_atlas_get(), objid_slot);

    /* Bind select ID map SSBO. */
    int map_slot = GPU_shader_get_ssbo_binding(select_march_sh_, "select_id_map_buf");
    GPU_storagebuf_bind(map_ssbo_, map_slot);

    /* Bind SDF objects SSBO (for ray-AABB cycling). */
    gpu::StorageBuf *obj_ssbo = sdf::sdf_objects_ssbo_get();
    if (obj_ssbo) {
      int obj_slot = GPU_shader_get_ssbo_binding(select_march_sh_, "sdf_objects");
      GPU_storagebuf_bind(obj_ssbo, obj_slot);
    }

    /* Bind selection system UBO and output SSBO. */
    if (select_info_buf_) {
      GPU_uniformbuf_bind(select_info_buf_, SELECT_DATA);
    }
    if (select_output_buf_) {
      GPU_storagebuf_bind(select_output_buf_, SELECT_ID_OUT);
    }

    /* Push atlas params. */
    float voxel_size;
    float3 origin, extent;
    int3 grid_res;
    int bpa;
    sdf::sdf_atlas_params_get(&voxel_size, &origin, &extent, &grid_res, &bpa);
    GPU_shader_uniform_1f(select_march_sh_, "voxel_size", voxel_size);
    GPU_shader_uniform_3fv(select_march_sh_, "atlas_origin", origin);
    GPU_shader_uniform_3fv(select_march_sh_, "atlas_extent", extent);
    GPU_shader_uniform_3iv(select_march_sh_, "grid_resolution", grid_res);
    GPU_shader_uniform_1i(select_march_sh_, "bricks_per_axis", bpa);
    GPU_shader_uniform_1i(select_march_sh_, "object_count", sdf::sdf_object_count_get());

    /* Bind the view UBO for camera matrices. */
    view.matrices_ubo_get().push_update();
    GPU_uniformbuf_bind(view.matrices_ubo_get(), DRW_VIEW_UBO_SLOT);

    /* Draw fullscreen triangle. */
    if (!fullscreen_batch_) {
      fullscreen_batch_ = GPU_batch_create_procedural(GPU_PRIM_TRIS, 3);
    }
    GPU_batch_set_shader(fullscreen_batch_, select_march_sh_);
    GPU_batch_draw(fullscreen_batch_);

    /* Cleanup. */
    GPU_texture_unbind(sdf::sdf_atlas_get());
    GPU_texture_unbind(sdf::sdf_indirection_get());
    GPU_texture_unbind(sdf::sdf_object_id_atlas_get());
    GPU_shader_unbind();
  }
};

}  // namespace blender::draw::overlay
