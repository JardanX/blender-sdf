/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "DNA_sdf_types.h"

#include "overlay_base.hh"

namespace blender::draw::overlay {

/**
 * SDF overlay for GPU picking.
 * Draws a solid ray-march bounding cube per SDF object so the selection system
 * can sphere-trace the analytical SDF primitive for pixel-perfect picking with exact depth.
 * Only active during selection draws (no visible output).
 */
class Sdfs : Overlay {
 private:
  const SelectionType selection_type_;

  /* Solid ray-march pass for picking/depth. */
  PassSimple solid_ps_ = {"SDF_solid"};

 public:
  Sdfs(const SelectionType selection_type) : selection_type_(selection_type) {};

  void begin_sync(Resources &res, const State &state) final
  {
    enabled_ = state.is_space_v3d() &&
               (selection_type_ != SelectionType::DISABLED);

    if (!enabled_) {
      return;
    }

    /* Set up solid pass fully here so we can record draws in object_sync.
     * PassSimple requires shader_set() before draw() calls. */
    solid_ps_.init();
    solid_ps_.state_set(DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS_EQUAL,
                        state.clipping_plane_count);
    solid_ps_.shader_set(res.shaders->sdf_pick.get());
    solid_ps_.bind_ubo(OVERLAY_GLOBALS_SLOT, &res.globals_buf);
    solid_ps_.bind_ubo(DRW_CLIPPING_UBO_SLOT, &res.clip_planes_buf);
    res.select_bind(solid_ps_);
  }

  void object_sync(Manager &manager,
                   const ObjectRef &ob_ref,
                   Resources &res,
                   const State & /*state*/) final
  {
    if (!enabled_) {
      return;
    }

    const Object *ob = ob_ref.object;
    const SDF *sdf = static_cast<const SDF *>(ob->data);
    if (!sdf) {
      return;
    }

    const select::ID select_id = res.select_id(ob_ref);

    /* Compute bounding extent that encloses the SDF primitive. */
    const float3 half = float3(sdf->size[0], sdf->size[1], sdf->size[2]) + float3(sdf->bevel);
    float3 bound = half;
    if (eSDFType(sdf->sdf_type) == SDF_TYPE_TORUS) {
      bound = float3(half.x + half.y, half.y + sdf->bevel, half.z + half.y);
    }

    /* Use the object's unscaled transform for the resource handle so that
     * drw_point_world_to_object() in the fragment shader gives SDF-local coordinates.
     * The bounding cube is scaled in the vertex shader via sdf_bound push constant. */
    ResourceHandle handle = manager.resource_handle(float4x4(ob->object_to_world()));

    /* Push constants must come before the draw call. */
    solid_ps_.push_constant("sdf_type", int(sdf->sdf_type));
    solid_ps_.push_constant("sdf_size", float3(sdf->size));
    solid_ps_.push_constant("sdf_bevel", sdf->bevel);
    solid_ps_.push_constant("sdf_bound", bound);
    solid_ps_.draw(
        res.shapes.cube_solid.get(), ResourceHandleRange(handle), select_id.get());
  }

  void end_sync(Resources & /*res*/, const State & /*state*/) final {}

  void draw(Framebuffer &framebuffer, Manager &manager, View &view) final
  {
    if (!enabled_) {
      return;
    }

    GPU_framebuffer_bind(framebuffer);
    manager.submit(solid_ps_, view);
  }
};

}  // namespace blender::draw::overlay
