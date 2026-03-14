/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw_engine
 *
 * SDF (Signed Distance Field) draw engine.
 * Bakes SDF objects into a dense 3D atlas and ray-marches it.
 */

#pragma once

#include "BLI_math_vector_types.hh"

#include "GPU_storage_buffer.hh"
#include "GPU_texture.hh"

#include "DRW_render.hh"

#define SDF_PERF_BUF_SIZE 512

namespace blender::draw::sdf {

struct Engine : public DrawEngine::Pointer {
  DrawEngine *create_instance() final;
};

const char *sdf_perf_info_get();
bool sdf_perf_active();

gpu::Texture *sdf_atlas_get();
gpu::Texture *sdf_indirection_get();
gpu::Texture *sdf_object_id_atlas_get();

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
