/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw_engine
 *
 * SDF (Signed Distance Field) draw engine.
 * Analytical sphere tracer — evaluates SDF primitives directly per-pixel.
 */

#pragma once

#include "BLI_math_vector_types.hh"

#include "GPU_storage_buffer.hh"
#include "GPU_texture.hh"

#include "DRW_render.hh"

#include "sdf_bvh.hh"

namespace blender::draw::sdf {

struct Engine : public DrawEngine::Pointer {
  DrawEngine *create_instance() final;
};

void sdf_atlas_params_get(float *voxel_size,
                          float3 *origin,
                          float3 *extent,
                          int3 *grid_resolution,
                          int *bricks_per_axis);

int sdf_object_count_get();
gpu::StorageBuf *sdf_objects_ssbo_get();
gpu::StorageBuf *sdf_modifiers_ssbo_get();
gpu::StorageBuf *sdf_groups_ssbo_get();
int sdf_group_count_get();

/* Maps depsgraph iteration order to sorted sdf_objects[] order. */
const int *sdf_depsgraph_to_sorted_get(int *out_count);

}  // namespace blender::draw::sdf
