/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Dense per-mesh SDF volume bake: one thread per voxel.
 *
 * Bakes signed distance (narrow band, fp16), smooth normals (barycentric
 * corner-normal interpolation — exactly the base mesh's shading, including
 * sharp edges / autosmooth, 2x fp16) and barycentric corner colors (RGBA8)
 * into three shared append-only voxel pools (bake_dist / bake_nrm / bake_col).
 * All coordinates are in the payload's local UNSCALED mesh space
 * (mesh_scale = 1), matching the analytic mesh eval convention.
 *
 * Grid convention: voxel center of (i,j,k) is
 *   origin + (vec3(i,j,k) + 0.5) * voxel_size,
 * the grid covers the payload bounds expanded by (band + voxel_size) on every
 * side. Voxel linear index: vi = base + i + res.x * (j + res.y * k), shared by
 * all three pools (bake_nrm stores two uints per voxel at 2*vi / 2*vi+1).
 *
 * Re-dispatching a bake for the same record just rewrites its voxels
 * (idempotent). Runtime sampling consumes these pools in a later phase. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_mesh_bake_comp)

#include "sdf_lp_common.glsl"

/* Manual IEEE-754 float32 -> float16 bits conversion (round to nearest even;
 * overflow -> half inf, underflow -> signed zero). Written longhand instead
 * of relying on packHalf2x16: the bit layout is identical, but the builtin
 * has no other user in the tree and its Metal translation is unverified.
 * Distance/normal magnitudes here are all well within half range. */
uint sdf_bake_f32_to_f16_bits(float v)
{
  uint b = floatBitsToUint(v);
  uint sign = (b >> 16u) & 0x8000u;
  uint mag = b & 0x7FFFFFFFu;

  /* NaN/Inf, or too large for a half -> half Inf. */
  if (mag >= 0x7F800000u || mag > 0x477FEFFFu) {
    return sign | 0x7C00u;
  }
  /* Too small for a half subnormal -> signed zero. */
  if (mag <= 0x33000000u) {
    return sign;
  }
  /* Half subnormal. */
  if (mag < 0x38800000u) {
    uint shift = 126u - (mag >> 23u); /* 14..24 */
    uint man = (mag & 0x7FFFFFu) | 0x800000u;
    uint half_man = man >> shift;
    uint remainder = man & ((1u << shift) - 1u);
    uint halfway = 1u << (shift - 1u);
    if (remainder > halfway || (remainder == halfway && (half_man & 1u) != 0u)) {
      half_man += 1u;
    }
    return sign + half_man;
  }
  /* Normal half: rebias the exponent (127 -> 15), round the mantissa to
   * nearest even; a mantissa overflow carries into the exponent. */
  uint half_bits = (mag >> 13u) - 0x1C000u;
  uint remainder = mag & 0x1FFFu;
  if (remainder > 0x1000u || (remainder == 0x1000u && (half_bits & 1u) != 0u)) {
    half_bits += 1u;
  }
  return sign + half_bits;
}

/* packHalf2x16(vec2(x, y)) equivalent (x in the low half-word). */
uint sdf_bake_pack_half2(float x, float y)
{
  return sdf_bake_f32_to_f16_bits(x) | (sdf_bake_f32_to_f16_bits(y) << 16u);
}

/* packUnorm4x8 equivalent, matching the CPU-side packing in
 * BKE_sdf (sdf_mesh.cc sdf_mesh_pack_color: R in the low byte, A high). */
uint sdf_bake_pack_rgba8(float4 c)
{
  uint4 b = uint4(clamp(c, 0.0f, 1.0f) * 255.0f + 0.5f);
  return b.x | (b.y << 8u) | (b.z << 16u) | (b.w << 24u);
}

void main()
{
  int3 voxel = int3(gl_GlobalInvocationID);
  /* Progressive bake: the manager dispatches one z-slice range per call;
   * z_offset is the absolute z of this dispatch's first slice. */
  voxel.z += z_offset;
  int3 res = res_and_base.xyz;
  if (voxel.x >= res.x || voxel.y >= res.y || voxel.z >= res.z) {
    return;
  }

  float3 p = origin + (float3(voxel) + 0.5f) * voxel_size;

  /* Miss defaults: far-field distance, arbitrary unit normal, white. */
  float d = band;
  float3 nrm = float3(0.0f, 0.0f, 1.0f);
  uint col = 0xFFFFFFFFu;

  float distance_squared;
  int triangle_index;
  float3 closest_point;
  float3 barycentric;
  float3 feature_normal;
  if (lp_mesh_nearest(mesh_data,
                      mesh_node_count,
                      float4(1.0f),
                      p,
                      distance_squared,
                      triangle_index,
                      closest_point,
                      barycentric,
                      feature_normal))
  {
    float sign_value = dot(p - closest_point, feature_normal) < 0.0f ? -1.0f : 1.0f;
    d = clamp(sqrt(max(distance_squared, 0.0f)) * sign_value, -band, band);

    SDFMeshTriangleGPU triangle = lp_mesh_triangle_load(mesh_data, triangle_index);
    float3 corner_nrm = lp_mesh_unpack_normal(triangle.corner_normals.x) * barycentric.x +
                        lp_mesh_unpack_normal(triangle.corner_normals.y) * barycentric.y +
                        lp_mesh_unpack_normal(triangle.corner_normals.z) * barycentric.z;
    /* Degenerate interpolation (opposing normals canceling) falls back to the
     * winning feature's normal. */
    nrm = (dot(corner_nrm, corner_nrm) > 1e-12f) ? normalize(corner_nrm) : feature_normal;

    if (has_colors != 0) {
      uint4 packed_colors = mesh_color_buf[color_start + triangle_index];
      float4 c = unpackUnorm4x8(packed_colors.x) * barycentric.x +
                 unpackUnorm4x8(packed_colors.y) * barycentric.y +
                 unpackUnorm4x8(packed_colors.z) * barycentric.z;
      col = sdf_bake_pack_rgba8(c);
    }
  }

  int vi = res_and_base.w + voxel.x + res.x * (voxel.y + res.y * voxel.z);
  if (dist_only != 0) {
    /* Coarse far-field level: full fp32 distance (1 uint = float bits). The
     * coarse grid's unclamped field must stay smooth far out — fp16
     * quantization at blend-reach scale bands wide blend zones. The normal
     * and color ranges of coarse records are never read at runtime. */
    bake_dist[vi] = floatBitsToUint(clamp(d, -60000.0f, 60000.0f));
    return;
  }
  bake_dist[vi] = sdf_bake_pack_half2(d, 0.0f);
  bake_nrm[2 * vi] = sdf_bake_pack_half2(nrm.x, nrm.y);
  bake_nrm[2 * vi + 1] = sdf_bake_pack_half2(nrm.z, 0.0f);
  bake_col[vi] = col;
}
