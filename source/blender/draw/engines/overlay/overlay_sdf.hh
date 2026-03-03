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
 * Draws a solid bounding cube per SDF object using the standard extra_shape instanced
 * pattern (same as speakers/empties). Only active during selection draws.
 */
class Sdfs : Overlay {
  using SdfInstanceBuf = ShapeInstanceBuf<ExtraInstanceData>;

 private:
  const SelectionType selection_type_;

  PassSimple solid_ps_ = {"SDF_solid"};
  SdfInstanceBuf pick_buf_;

 public:
  Sdfs(const SelectionType selection_type)
      : selection_type_(selection_type), pick_buf_(selection_type, "sdf_pick_buf") {};

  void begin_sync(Resources & /*res*/, const State &state) final
  {
    enabled_ = state.is_space_v3d() && (selection_type_ != SelectionType::DISABLED);

    if (!enabled_) {
      return;
    }

    pick_buf_.clear();
  }

  void object_sync(Manager & /*manager*/,
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

    /* Bake SDF bounding extent into the model matrix so the unit cube [-1,1]
     * maps to the SDF bounding volume in world space. */
    float4x4 mat = float4x4(ob->object_to_world());
    mat[0] *= bound.x;
    mat[1] *= bound.y;
    mat[2] *= bound.z;

    const float4 color(0.0f);
    pick_buf_.append({mat, color, 1.0f}, select_id);
  }

  void end_sync(Resources &res, const State &state) final
  {
    if (!enabled_) {
      return;
    }

    solid_ps_.init();
    solid_ps_.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH |
                            DRW_STATE_DEPTH_LESS_EQUAL,
                        state.clipping_plane_count);
    solid_ps_.shader_set(res.shaders->extra_shape.get());
    solid_ps_.bind_ubo(OVERLAY_GLOBALS_SLOT, &res.globals_buf);
    solid_ps_.bind_ubo(DRW_CLIPPING_UBO_SLOT, &res.clip_planes_buf);
    res.select_bind(solid_ps_);

    pick_buf_.end_sync(solid_ps_, res.shapes.cube_solid.get());
  }

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
