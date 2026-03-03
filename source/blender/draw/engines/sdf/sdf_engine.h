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

#include "GPU_texture.hh"

#include "DRW_render.hh"

/** Size of the formatted performance text buffer. */
#define SDF_PERF_BUF_SIZE 512

namespace blender::draw::sdf {

struct Engine : public DrawEngine::Pointer {
  DrawEngine *create_instance() final;
};

/**
 * Returns the formatted SDF performance text for the overlay.
 * The returned pointer is valid until the next call to this function or engine destruction.
 * Returns nullptr if no performance data is available.
 */
const char *sdf_perf_info_get();

/** Returns true if performance data is valid and should be displayed. */
bool sdf_perf_active();

/** Returns the compact SDF atlas texture, or nullptr if not baked. */
gpu::Texture *sdf_atlas_get();

/** Returns the indirection texture, or nullptr. */
gpu::Texture *sdf_indirection_get();

/** Returns the object ID atlas texture, or nullptr. */
gpu::Texture *sdf_object_id_atlas_get();

/** Returns current atlas parameters. */
void sdf_atlas_params_get(float *voxel_size,
                          float3 *origin,
                          float3 *extent,
                          int3 *grid_resolution,
                          int *bricks_per_axis);

/** Returns the number of SDF objects in the current bake. */
int sdf_object_count_get();

}  // namespace blender::draw::sdf
