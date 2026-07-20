/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/* Mesh-to-SDF is bake-only at runtime: the analytic BVH evaluation path was
 * removed (too slow per ray-step, smooth normals never robust). Mesh objects
 * sample the baked voxel volume (sdf_mesh_bake_comp) via the samplers below;
 * objects whose bake is still pending evaluate to a huge distance (invisible)
 * until their record flips ready. */

/* Local UNSCALED mesh-space position of the last baked volume sample (set by
 * sdTriangleMesh); consumed by sdfMeshLastWorldNormals to fetch the baked
 * normal at the hit. */
float3 g_sdf_mesh_last_baked_pos;
/* Raw FINE-grid distance of the last baked volume sample (unscaled local
 * units; 1e30 when the sample fell outside the fine grid). Drives the
 * baked-normal -> field-gradient cross-fade in the color resolve: near the
 * surface the baked smooth normal is authoritative, deep in a blend zone
 * (|d| towards the narrow band and beyond) only the field gradient is
 * meaningful. NOTE: FD stencil taps (sdfAnalyticWorldNormals) re-evaluate
 * the mesh and overwrite this — read it before calling them. */
float g_sdf_mesh_last_baked_fine_dist;

/* ------------------------------------------------------------------ */
/** \name Baked volume sampling (SDF_LP_MESH_FLAG_BAKED)
 *
 * Trilinear sampling of the per-mesh baked voxel pools (bake_dist/bake_nrm/
 * bake_col, written by sdf_mesh_bake_comp). All coordinates are in the
 * payload's UNSCALED local mesh space; the grid convention (voxel centers,
 * linear index) matches the bake shader exactly.
 * \{ */

/* Manual IEEE-754 float16 -> float32 decode (bit layout mirrors
 * sdf_bake_f32_to_f16_bits in sdf_mesh_bake_comp.glsl; written longhand
 * because unpackHalf2x16 has no other user in the tree and its Metal
 * translation is unverified). */
float sdfBakedF16(uint h)
{
  uint mag = h & 0x7FFFu;
  float f;
  if (mag == 0u) {
    f = 0.0f;
  }
  else if (mag >= 0x7C00u) {
    /* Half inf/nan: never produced by the bake (values are band-clamped). */
    f = 1e30f;
  }
  else if ((mag >> 10u) == 0u) {
    /* Subnormal half: mag * 2^-24 (exactly representable). */
    f = float(mag) * 5.9604644775390625e-8f;
  }
  else {
    /* Normal half: rebias the exponent (15 -> 127), shift the mantissa. */
    f = uintBitsToFloat((((mag >> 10u) + 112u) << 23u) | ((mag & 0x3FFu) << 13u));
  }
  return ((h & 0x8000u) != 0u) ? -f : f;
}

/* Maps p to the containing trilinear cell. Returns false when p is outside
 * the voxel-center range [-0.5, res-0.5] on any axis (the far path). On
 * success `vi` is the linear index of the (0,0,0) corner WITHOUT the pool
 * base, and f the fractional position inside the cell. */
bool sdfBakedGridCoords(float3 origin,
                        float voxel_size,
                        int3 res,
                        float3 p,
                        out int vi,
                        out int stride_y,
                        out int stride_z,
                        out float3 f)
{
  float3 uvw = (p - origin) / voxel_size - 0.5f;
  float3 resf = float3(res);
  if (any(lessThan(uvw, float3(-0.5f))) || any(greaterThan(uvw, resf - 0.5f))) {
    return false;
  }
  int3 i0 = clamp(int3(floor(uvw)), int3(0), res - 2);
  f = clamp(uvw - float3(i0), float3(0.0f), float3(1.0f));
  stride_y = res.x;
  stride_z = res.x * res.y;
  vi = i0.x + i0.y * stride_y + i0.z * stride_z;
  return true;
}

/* Distance decode for one pool entry: fine grids store fp16 in the low
 * half-word, the coarse far-field grid stores full fp32 (its unclamped field
 * must stay smooth far out — fp16 quantization at blend-reach scale bands
 * wide blend zones). */
float sdfBakedDistAt(int vi, bool fp32)
{
  uint u = bake_dist[vi];
  return fp32 ? uintBitsToFloat(u) : sdfBakedF16(u & 0xFFFFu);
}

/* Trilinear sample of ONE baked grid (fine or coarse), with the sign-flip
 * guard: the baked sign comes from the closest feature's normal, so across a
 * Voronoi boundary between opposing-orientation features (the gap between
 * two parallel sheets) the field jumps from +a to -b with a, b large. The
 * analytic field jumps without crossing zero (harmless), but trilinear
 * interpolation ramps through zero and paints a phantom surface across the
 * gap. A real surface crossing a cell always puts SOME corner within half
 * the cell diagonal (sqrt(3)/2*voxel) of it, so a sign disagreement with
 * every corner farther than voxel_size is always such a phantom: return the
 * conservative positive lower bound (d(p) >= min|d_corner| - sqrt(3)/2*voxel
 * for any p in the cell); genuine interiors (all corners negative) are
 * untouched. Returns false when p is outside the voxel-center range. */
bool sdfBakedGridSample(
    float3 origin, float voxel_size, int3 res, int base, bool fp32, float3 p, out float d)
{
  int vi;
  int stride_y;
  int stride_z;
  float3 f;
  if (!sdfBakedGridCoords(origin, voxel_size, res, p, vi, stride_y, stride_z, f)) {
    return false;
  }
  vi += base;
  float d000 = sdfBakedDistAt(vi, fp32);
  float d100 = sdfBakedDistAt(vi + 1, fp32);
  float d010 = sdfBakedDistAt(vi + stride_y, fp32);
  float d110 = sdfBakedDistAt(vi + stride_y + 1, fp32);
  float d001 = sdfBakedDistAt(vi + stride_z, fp32);
  float d101 = sdfBakedDistAt(vi + stride_z + 1, fp32);
  float d011 = sdfBakedDistAt(vi + stride_y + stride_z, fp32);
  float d111 = sdfBakedDistAt(vi + stride_y + stride_z + 1, fp32);
  float min_abs = min(min(min(abs(d000), abs(d100)), min(abs(d010), abs(d110))),
                      min(min(abs(d001), abs(d101)), min(abs(d011), abs(d111))));
  if (min_abs > voxel_size) {
    bool neg0 = d000 < 0.0f;
    bool disagree = (neg0 != (d100 < 0.0f)) || (neg0 != (d010 < 0.0f)) ||
                    (neg0 != (d110 < 0.0f)) || (neg0 != (d001 < 0.0f)) ||
                    (neg0 != (d101 < 0.0f)) || (neg0 != (d011 < 0.0f)) ||
                    (neg0 != (d111 < 0.0f));
    if (disagree) {
      d = min_abs - 0.87f * voxel_size;
      return true;
    }
  }
  d = mix(mix(mix(d000, d100, f.x), mix(d010, d110, f.x), f.y),
          mix(mix(d001, d101, f.x), mix(d011, d111, f.x), f.y),
          f.z);
  return true;
}

/* Baked signed distance at p (unscaled local mesh space), 3-way:
 * 1. fine grid (narrow band, full detail) while its value is NOT clamped;
 * 2. coarse grid (unclamped fp32, out to the scene blend reach) everywhere
 *    else inside it — CSG blends of any radius see a true distance field,
 *    and deferring to it at the fine clamp keeps the field continuous: a
 *    clamp step here would paint the fine volume's box skin into wide color
 *    blends;
 * 3. otherwise the point-to-MESH-BOUNDS distance (the fine volume box
 *    shrunk by pad = band + voxel_size on every side) — a conservative
 *    lower bound on the true distance that, unlike the volume-box distance,
 *    never vanishes along a ray outside the grid (the volume-box distance
 *    drops to zero exactly on the volume skin and sphere tracing
 *    stalls/hits a phantom shell there).
 * Both grid reads are scaled by 0.95 as a Lipschitz safety margin for
 * interpolation error. */
float sdfBakedSample(float3 origin,
                     float voxel_size,
                     float band,
                     int3 res,
                     int base,
                     float3 corigin,
                     float cvoxel,
                     int3 cres,
                     int cbase,
                     float3 p,
                     out float fine_d)
{
  float d;
  const bool have_fine = sdfBakedGridSample(origin, voxel_size, res, base, false, p, d);
  fine_d = have_fine ? d : 1e30f;
  if (have_fine && abs(d) < 0.99f * band) {
    return d * 0.95f;
  }
  float dc;
  if (cres.x > 0 && sdfBakedGridSample(corigin, cvoxel, cres, cbase, true, p, dc)) {
    return dc * 0.95f;
  }
  if (have_fine) {
    /* No coarse level (scene has no blends): clamped fine value. */
    return d * 0.95f;
  }
  float pad = band + voxel_size;
  float3 inner_min = origin + float3(pad);
  float3 inner_max = origin + float3(res) * voxel_size - float3(pad);
  float3 delta = max(inner_min - p, max(p - inner_max, float3(0.0f)));
  return length(delta);
}

/* Baked smooth (corner-normal) shading normal at p. No far path: only
 * called at/near the traced surface, which lies inside the grid. */
float3 sdfBakedNormal(float3 origin, float voxel_size, int3 res, int base, float3 p)
{
  int vi;
  int stride_y;
  int stride_z;
  float3 f;
  if (!sdfBakedGridCoords(origin, voxel_size, res, p, vi, stride_y, stride_z, f)) {
    return float3(0.0f, 0.0f, 1.0f);
  }
  vi += base;
  int c000 = 2 * vi;
  int c100 = 2 * (vi + 1);
  int c010 = 2 * (vi + stride_y);
  int c110 = 2 * (vi + stride_y + 1);
  int c001 = 2 * (vi + stride_z);
  int c101 = 2 * (vi + stride_z + 1);
  int c011 = 2 * (vi + stride_y + stride_z);
  int c111 = 2 * (vi + stride_y + stride_z + 1);
  float3 n000 = float3(sdfBakedF16(bake_nrm[c000] & 0xFFFFu),
                       sdfBakedF16(bake_nrm[c000] >> 16u),
                       sdfBakedF16(bake_nrm[c000 + 1] & 0xFFFFu));
  float3 n100 = float3(sdfBakedF16(bake_nrm[c100] & 0xFFFFu),
                       sdfBakedF16(bake_nrm[c100] >> 16u),
                       sdfBakedF16(bake_nrm[c100 + 1] & 0xFFFFu));
  float3 n010 = float3(sdfBakedF16(bake_nrm[c010] & 0xFFFFu),
                       sdfBakedF16(bake_nrm[c010] >> 16u),
                       sdfBakedF16(bake_nrm[c010 + 1] & 0xFFFFu));
  float3 n110 = float3(sdfBakedF16(bake_nrm[c110] & 0xFFFFu),
                       sdfBakedF16(bake_nrm[c110] >> 16u),
                       sdfBakedF16(bake_nrm[c110 + 1] & 0xFFFFu));
  float3 n001 = float3(sdfBakedF16(bake_nrm[c001] & 0xFFFFu),
                       sdfBakedF16(bake_nrm[c001] >> 16u),
                       sdfBakedF16(bake_nrm[c001 + 1] & 0xFFFFu));
  float3 n101 = float3(sdfBakedF16(bake_nrm[c101] & 0xFFFFu),
                       sdfBakedF16(bake_nrm[c101] >> 16u),
                       sdfBakedF16(bake_nrm[c101 + 1] & 0xFFFFu));
  float3 n011 = float3(sdfBakedF16(bake_nrm[c011] & 0xFFFFu),
                       sdfBakedF16(bake_nrm[c011] >> 16u),
                       sdfBakedF16(bake_nrm[c011 + 1] & 0xFFFFu));
  float3 n111 = float3(sdfBakedF16(bake_nrm[c111] & 0xFFFFu),
                       sdfBakedF16(bake_nrm[c111] >> 16u),
                       sdfBakedF16(bake_nrm[c111 + 1] & 0xFFFFu));
  return mix(mix(mix(n000, n100, f.x), mix(n010, n110, f.x), f.y),
             mix(mix(n001, n101, f.x), mix(n011, n111, f.x), f.y),
             f.z);
}

/* Baked barycentric corner color (RGBA8, linear) at p. Same near-surface
 * assumption as sdfBakedNormal. */
float4 sdfBakedColor(float3 origin, float voxel_size, int3 res, int base, float3 p)
{
  int vi;
  int stride_y;
  int stride_z;
  float3 f;
  if (!sdfBakedGridCoords(origin, voxel_size, res, p, vi, stride_y, stride_z, f)) {
    return float4(1.0f);
  }
  vi += base;
  float4 c000 = unpackUnorm4x8(bake_col[vi]);
  float4 c100 = unpackUnorm4x8(bake_col[vi + 1]);
  float4 c010 = unpackUnorm4x8(bake_col[vi + stride_y]);
  float4 c110 = unpackUnorm4x8(bake_col[vi + stride_y + 1]);
  float4 c001 = unpackUnorm4x8(bake_col[vi + stride_z]);
  float4 c101 = unpackUnorm4x8(bake_col[vi + stride_z + 1]);
  float4 c011 = unpackUnorm4x8(bake_col[vi + stride_y + stride_z]);
  float4 c111 = unpackUnorm4x8(bake_col[vi + stride_y + stride_z + 1]);
  return mix(mix(mix(c000, c100, f.x), mix(c010, c110, f.x), f.y),
             mix(mix(c001, c101, f.x), mix(c011, c111, f.x), f.y),
             f.z);
}

/** \} */

float sdTriangleMesh(float3 p, SDFObjectGPU obj)
{
  if ((obj.mesh_settings.y & SDF_LP_MESH_FLAG_BAKED) == 0) {
    /* Bake pending: the object stays invisible until its record flips ready
     * (the analytic BVH runtime path was removed). */
    g_sdf_mesh_last_baked_fine_dist = 1e30f;
    return 1e30f;
  }
  /* p is the rotation-only local position; dividing by the scale maps into
   * the unscaled volume frame. The min-scale factor converts the sampled
   * distance back to the scaled frame (exact for uniform scale,
   * conservative otherwise). */
  float3 p_unscaled = p / obj.obj_scale.xyz;
  g_sdf_mesh_last_baked_pos = p_unscaled;
  float d = sdfBakedSample(obj.bake_origin.xyz,
                           obj.bake_origin.w,
                           obj.bake_params.x,
                           obj.bake_grid.xyz,
                           obj.bake_grid.w,
                           obj.bake_coarse_origin.xyz,
                           obj.bake_coarse_origin.w,
                           obj.bake_coarse_grid.xyz,
                           obj.bake_coarse_grid.w,
                           p_unscaled,
                           g_sdf_mesh_last_baked_fine_dist);
  return d * min(min(obj.obj_scale.x, obj.obj_scale.y), obj.obj_scale.z);
}

bool sdfMeshLastWorldNormals(SDFObjectGPU obj,
                             out float3 shading_normal,
                             out float3 geometric_normal)
{
  if ((obj.mesh_settings.y & SDF_LP_MESH_FLAG_BAKED) == 0) {
    shading_normal = float3(0.0f);
    geometric_normal = float3(0.0f);
    return false;
  }
  /* Near-surface hits only: the baked smooth normal is defined by the
   * fine grid. When the last sample fell outside it (blend-zone hits,
   * whose distance came from the coarse far grid or the box fallback)
   * there is no baked normal — return false so the caller falls back to
   * the field gradient (sdfAnalyticWorldNormals) like the analytic
   * primitives, instead of shading with the constant out-of-grid
   * fallback. */
  int vi;
  int stride_y;
  int stride_z;
  float3 f;
  if (!sdfBakedGridCoords(obj.bake_origin.xyz,
                          obj.bake_origin.w,
                          obj.bake_grid.xyz,
                          g_sdf_mesh_last_baked_pos,
                          vi,
                          stride_y,
                          stride_z,
                          f))
  {
    shading_normal = float3(0.0f);
    geometric_normal = float3(0.0f);
    return false;
  }
  /* Baked volume: fetch the baked smooth normal at the last sampled
   * position and return it as both shading and geometric normal. */
  float3 n = sdfBakedNormal(obj.bake_origin.xyz,
                            obj.bake_origin.w,
                            obj.bake_grid.xyz,
                            obj.bake_grid.w,
                            g_sdf_mesh_last_baked_pos);
  float3x3 normal_to_world = transpose(to_float3x3(obj.inverse_matrix));
  float3 world_n = normal_to_world * (n / obj.obj_scale.xyz);
  float len_squared = dot(world_n, world_n);
  if (len_squared <= 1e-12f || any(isnan(world_n))) {
    shading_normal = float3(0.0f);
    geometric_normal = float3(0.0f);
    return false;
  }
  world_n *= inversesqrt(len_squared);
  shading_normal = world_n;
  geometric_normal = world_n;
  return true;
}
