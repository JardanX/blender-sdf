/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_math_vector_types.hh"
#include "BLI_string_ref.hh"
#include "BLI_vector.hh"

namespace blender::draw::sdf {

/**
 * Bake the current SDF scene to a triangle mesh via the CPU DC-SDD
 * (Dual Contouring of Signed Distance Data) mesher.
 * Returns empty string on success, or an error description on failure.
 */
std::string sdf_bake_to_mesh(int grid_res,
                             Vector<float3> &out_positions,
                             Vector<float3> &out_normals,
                             Vector<int3> &out_tris,
                             Vector<float4> &out_colors,
                             int *out_vert_count,
                             int *out_tri_count);

}  // namespace blender::draw::sdf
