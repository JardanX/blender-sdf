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

#include <string>

#include "BLI_math_vector_types.hh"

#include "GPU_storage_buffer.hh"
#include "GPU_texture.hh"

#include "sdf_shader_shared.hh"

#include "DRW_render.hh"

#include "sdf_bvh.hh"

namespace blender::draw::sdf {

struct Engine : public DrawEngine::Pointer {
  DrawEngine *create_instance() final;
};

int sdf_object_count_get();
int sdf_group_count_get();
const SDFObjectGPU *sdf_objects_cpu_get();

const int *sdf_depsgraph_to_sorted_get(int *out_count);

gpu::Texture *sdf_depth_texture_get();
gpu::Texture *sdf_gbuf_color_texture_get();

float2 sdf_uv_scale_get();

/* Get local-space bbox + rotation/position for an SDF object.
 * hint_pos: world position of the object to disambiguate modifier copies.
 * Returns false if not found. */
bool sdf_object_bbox_get(int sdf_index, const float3 &hint_pos,
                         float3 &out_min, float3 &out_max,
                         float4x4 &out_rot, float3 &out_pos);

/* Debug: get near-surface grid points from last bbox computation. */
void sdf_bbox_debug_points_get(const float3 **pts, int *count,
                               float4x4 *rot, float3 *pos);

/* Free static shader cache (call on application exit). */
void sdf_shaders_free();

/* Frame profiling */
void sdf_profile_request();
bool sdf_profile_is_ready();
bool sdf_profile_is_pending();
std::string sdf_profile_format_text();

}  // namespace blender::draw::sdf
