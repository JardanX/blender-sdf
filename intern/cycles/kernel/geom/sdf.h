/* SPDX-FileCopyrightText: 2025 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

/* SDF ray marching for Cycles kernel.
 * Two-level DDA: brick-level traversal skips empty space, then
 * voxel-level DDA within active bricks finds exact surface hits.
 *
 * Ported from the draw engine's sdf_march_frag.glsl.
 * Based on "Ray Tracing of Signed Distance Function Grids"
 * (Hansson-Soderlund, Evans, Akenine-Moller 2022). */

#pragma once

#include "kernel/geom/sdf_lib.h"

CCL_NAMESPACE_BEGIN

#define SDF_BRICK_SIZE 8
#define SDF_BRICK_STORAGE 12 /* 8 interior + 2-voxel border on each side (matches draw engine). */
#define SDF_BRICK_BORDER 2
#define SDF_MAX_BRICK_STEPS 256
#define SDF_MAX_VOXEL_STEPS 24

/* uint16 distance encoding: maps [-SDF_DIST16_RANGE, +SDF_DIST16_RANGE] voxel-units
 * to [0, 65535].  Range of 12 covers the full brick storage diagonal
 * (12 * sqrt(3)/2 ≈ 10.4 voxels).  65536 levels over 24 voxel-units gives
 * ~0.000366 voxels per quantization step — negligible precision loss. */
#define SDF_DIST16_RANGE 12.0f

/* -------------------------------------------------------------------- */
/* Flat array access helpers.
 *
 * SDF data is stored in flat device arrays (sdf_indirection, sdf_atlas,
 * sdf_matid) and accessed via kernel_data_fetch with linearized 3D indices.
 * Each SDF object stores an offset into these arrays. */

/* Fetch integer value from indirection array at brick coordinate. */
ccl_device_inline int sdf_fetch_indirection(KernelGlobals kg,
                                             const int offset,
                                             const int bx,
                                             const int by,
                                             const int bz,
                                             const int grid_res)
{
  const int idx = offset + bz * grid_res * grid_res + by * grid_res + bx;
  return kernel_data_fetch(sdf_indirection, idx);
}

/* Non-cubic variant for per-shape grids (instanced mode). */
ccl_device_inline int sdf_fetch_shape_indirection(KernelGlobals kg,
                                                   const int offset,
                                                   const int bx,
                                                   const int by,
                                                   const int bz,
                                                   const int grid_res_x,
                                                   const int grid_res_y)
{
  const int idx = offset + bz * grid_res_x * grid_res_y + by * grid_res_x + bx;
  return kernel_data_fetch(sdf_shape_indirection, idx);
}

/* Fetch float4 value from atlas array at given texel. */
ccl_device_inline float4 sdf_fetch_atlas(KernelGlobals kg,
                                          const int offset,
                                          const int x,
                                          const int y,
                                          const int z,
                                          const int atlas_dim)
{
  const int idx = offset + z * atlas_dim * atlas_dim + y * atlas_dim + x;
  return kernel_data_fetch(sdf_atlas, idx);
}

/* Fetch the SDF distance at a compact atlas coordinate. */
ccl_device_inline float sdf_fetch_distance(KernelGlobals kg,
                                            const int offset,
                                            const int x,
                                            const int y,
                                            const int z,
                                            const int atlas_dim)
{
  return sdf_fetch_atlas(kg, offset, x, y, z, atlas_dim).x;
}

/* Fetch material ID from matid array at given texel. */
ccl_device_inline int sdf_fetch_matid(KernelGlobals kg,
                                       const int offset,
                                       const int x,
                                       const int y,
                                       const int z,
                                       const int atlas_dim)
{
  const int idx = offset + z * atlas_dim * atlas_dim + y * atlas_dim + x;
  return kernel_data_fetch(sdf_matid, idx);
}

/* Compute the atlas-space base coordinate for a brick slot.
 * This involves 3 integer divides + 3 modulos, so call once per brick
 * and reuse the result for all voxel fetches within that brick. */
ccl_device_inline int3 sdf_slot_origin(const int brick_slot, const int bpa)
{
  const int sx = brick_slot % bpa;
  const int sy = (brick_slot / bpa) % bpa;
  const int sz = brick_slot / (bpa * bpa);
  return make_int3(sx * SDF_BRICK_STORAGE + SDF_BRICK_BORDER,
                   sy * SDF_BRICK_STORAGE + SDF_BRICK_BORDER,
                   sz * SDF_BRICK_STORAGE + SDF_BRICK_BORDER);
}

/* Convert grid-space local voxel to compact atlas coordinate using
 * pre-computed slot origin (from sdf_slot_origin). */
ccl_device_inline int3 sdf_grid_to_compact(const int3 local_voxel,
                                            const int3 slot_org)
{
  return make_int3(slot_org.x + local_voxel.x,
                   slot_org.y + local_voxel.y,
                   slot_org.z + local_voxel.z);
}

/* Fetch 8 corner SDF values for a voxel cell.
 * slot_org is the pre-computed atlas base from sdf_slot_origin(). */
ccl_device void sdf_fetch_corners(KernelGlobals kg,
                                   const int atlas_offset,
                                   const int3 local_cell,
                                   const int3 slot_org,
                                   const int atlas_dim,
                                   float s[8])
{
  const int3 base = sdf_grid_to_compact(local_cell, slot_org);
  s[0] = sdf_fetch_distance(kg, atlas_offset, base.x, base.y, base.z, atlas_dim);
  s[1] = sdf_fetch_distance(kg, atlas_offset, base.x + 1, base.y, base.z, atlas_dim);
  s[2] = sdf_fetch_distance(kg, atlas_offset, base.x, base.y + 1, base.z, atlas_dim);
  s[3] = sdf_fetch_distance(kg, atlas_offset, base.x + 1, base.y + 1, base.z, atlas_dim);
  s[4] = sdf_fetch_distance(kg, atlas_offset, base.x, base.y, base.z + 1, atlas_dim);
  s[5] = sdf_fetch_distance(kg, atlas_offset, base.x + 1, base.y, base.z + 1, atlas_dim);
  s[6] = sdf_fetch_distance(kg, atlas_offset, base.x, base.y + 1, base.z + 1, atlas_dim);
  s[7] = sdf_fetch_distance(
      kg, atlas_offset, base.x + 1, base.y + 1, base.z + 1, atlas_dim);
}

/* Decode a uint16 distance value to local-space float.
 * Reverses the encoding in sdf.cpp: uint16 -> [-RANGE, +RANGE] voxel-units -> * voxel_size. */
ccl_device_inline float sdf_decode_dist16(const uint16_t raw, const float voxel_size)
{
  return ((float(raw) * (1.0f / 65535.0f)) * 2.0f * SDF_DIST16_RANGE - SDF_DIST16_RANGE) *
         voxel_size;
}

/* Fetch 8 corner SDF values from the per-shape atlas (instanced mode).
 * Same layout as sdf_fetch_corners but reads uint16 from sdf_shape_atlas
 * and decodes to local-space float via sdf_decode_dist16. */
ccl_device void sdf_fetch_corners_shape(KernelGlobals kg,
                                         const int atlas_offset,
                                         const int3 local_cell,
                                         const int3 slot_org,
                                         const int atlas_dim,
                                         const float voxel_size,
                                         float s[8])
{
  const int3 base = sdf_grid_to_compact(local_cell, slot_org);
  const int zy0 = atlas_offset + base.z * atlas_dim * atlas_dim + base.y * atlas_dim;
  const int zy1 = zy0 + atlas_dim;           /* base.y + 1 */
  const int zy2 = zy0 + atlas_dim * atlas_dim; /* base.z + 1 */
  const int zy3 = zy2 + atlas_dim;           /* base.y + 1, base.z + 1 */
  s[0] = sdf_decode_dist16(kernel_data_fetch(sdf_shape_atlas, zy0 + base.x), voxel_size);
  s[1] = sdf_decode_dist16(kernel_data_fetch(sdf_shape_atlas, zy0 + base.x + 1), voxel_size);
  s[2] = sdf_decode_dist16(kernel_data_fetch(sdf_shape_atlas, zy1 + base.x), voxel_size);
  s[3] = sdf_decode_dist16(kernel_data_fetch(sdf_shape_atlas, zy1 + base.x + 1), voxel_size);
  s[4] = sdf_decode_dist16(kernel_data_fetch(sdf_shape_atlas, zy2 + base.x), voxel_size);
  s[5] = sdf_decode_dist16(kernel_data_fetch(sdf_shape_atlas, zy2 + base.x + 1), voxel_size);
  s[6] = sdf_decode_dist16(kernel_data_fetch(sdf_shape_atlas, zy3 + base.x), voxel_size);
  s[7] = sdf_decode_dist16(kernel_data_fetch(sdf_shape_atlas, zy3 + base.x + 1), voxel_size);
}

/* -------------------------------------------------------------------- */
/* Normal computation using dual voxel method
 * (Section 3.2, Hansson-Soderlund et al. JCGT 2022).
 *
 * Fetches a 3x3x3 neighborhood (27 values) around the dual voxel center,
 * evaluates analytic trilinear gradients in each of the 8 overlapping voxels,
 * normalizes each, and trilinearly blends for C0-continuous normals across
 * voxel boundaries. */

ccl_device float3 sdf_compute_normal(KernelGlobals kg,
                                      const int atlas_offset,
                                      const float3 grid_pos_in_brick,
                                      const int3 slot_org,
                                      const int atlas_dim)
{
  /* Dual voxel center: nearest grid corner to the hit point. */
  const float3 shifted = grid_pos_in_brick + make_float3(0.5f, 0.5f, 0.5f);
  const int3 dc = make_int3((int)floorf(shifted.x),
                             (int)floorf(shifted.y),
                             (int)floorf(shifted.z));

  /* Fetch 3x3x3 neighborhood centered on the dual voxel. */
  float vals[27];
  const int3 base = sdf_grid_to_compact(make_int3(dc.x - 1, dc.y - 1, dc.z - 1), slot_org);
  for (int k = 0; k < 3; k++) {
    for (int j = 0; j < 3; j++) {
      for (int i = 0; i < 3; i++) {
        vals[k * 9 + j * 3 + i] = sdf_fetch_distance(
            kg, atlas_offset, base.x + i, base.y + j, base.z + k, atlas_dim);
      }
    }
  }

  return sdf_dual_voxel_normal(vals, grid_pos_in_brick, dc);
}

/* -------------------------------------------------------------------- */
/* Per-brick intersection (used by OptiX hardware BVH path).
 *
 * When OptiX builds one AABB per active brick, the hardware BVH handles
 * brick-level traversal. The intersection program only needs to march
 * voxels within the single brick that was hit.
 * This reduces worst-case from 256+24 DDA steps to just 24 steps. */

ccl_device bool sdf_intersect_brick(KernelGlobals kg,
                                     ccl_private const Ray *ray,
                                     ccl_private Intersection *isect,
                                     const int sdf_index,
                                     const int brick_linear,
                                     const int brick_slot)
{
  const KernelSDF ksdf = kernel_data_fetch(sdf_objects, sdf_index);
  const int grid_res = ksdf.grid_res;
  const float voxel_size = ksdf.voxel_size;
  const float3 origin = make_float3(ksdf.origin.x, ksdf.origin.y, ksdf.origin.z);
  const int bpa = ksdf.bricks_per_axis;
  const int atlas_off = ksdf.atlas_offset;
  const int atlas_dim = bpa * SDF_BRICK_STORAGE;

  /* Decode brick cell from linear index. */
  const int3 brick_cell = make_int3(brick_linear % grid_res,
                                     (brick_linear / grid_res) % grid_res,
                                     brick_linear / (grid_res * grid_res));

  /* Fully-inside brick: immediate hit at AABB entry. */
  if (brick_slot == -2) {
    /* Clip ray to brick AABB. */
    const float brick_world = float(SDF_BRICK_SIZE) * voxel_size;
    const float3 brick_min = origin + make_float3(float(brick_cell.x),
                                                    float(brick_cell.y),
                                                    float(brick_cell.z)) *
                                          brick_world;
    const float3 brick_max = brick_min + make_float3(brick_world, brick_world, brick_world);

    const float3 inv_dir = safe_divide(one_float3(), ray->D);
    const float3 t0 = (brick_min - ray->P) * inv_dir;
    const float3 t1 = (brick_max - ray->P) * inv_dir;
    const float3 t_lo = min(t0, t1);
    float t_enter = max(max(t_lo.x, t_lo.y), t_lo.z);

    const float t_self_offset = voxel_size;
    t_enter = max(t_enter, t_self_offset);

    if (t_enter >= isect->t) {
      return false;
    }

    isect->t = t_enter;
    isect->prim = sdf_index;
    isect->object = ksdf.object_id;
    isect->type = PRIMITIVE_SDF;
    isect->u = __int_as_float(brick_linear);
    isect->v = __int_as_float(-2);
    return true;
  }

  /* Active brick: voxel-level DDA only. */
  const float inv_voxel = 1.0f / voxel_size;
  const int3 slot_org = sdf_slot_origin(brick_slot, bpa);

  const float3 brick_origin = origin + make_float3(float(brick_cell.x * SDF_BRICK_SIZE),
                                                     float(brick_cell.y * SDF_BRICK_SIZE),
                                                     float(brick_cell.z * SDF_BRICK_SIZE)) *
                                            voxel_size;

  /* Clip ray to brick AABB. */
  const float3 brick_min = brick_origin;
  const float3 brick_max = brick_origin + make_float3(float(SDF_BRICK_SIZE),
                                                        float(SDF_BRICK_SIZE),
                                                        float(SDF_BRICK_SIZE)) *
                                               voxel_size;

  const float3 inv_dir = safe_divide(one_float3(), ray->D);
  const float3 t0 = (brick_min - ray->P) * inv_dir;
  const float3 t1 = (brick_max - ray->P) * inv_dir;
  const float3 t_lo = min(t0, t1);
  const float3 t_hi = max(t0, t1);
  float t_enter = max(max(t_lo.x, t_lo.y), t_lo.z);
  float t_brick_exit = min(min(t_hi.x, t_hi.y), t_hi.z);

  const float t_self_offset = voxel_size;
  t_enter = max(t_enter, t_self_offset);
  t_brick_exit = min(t_brick_exit, isect->t);

  if (t_enter >= t_brick_exit) {
    return false;
  }

  /* Voxel-level DDA setup. */
  const float3 V = (ray->P - brick_origin) * inv_voxel;
  const float3 VD = ray->D * inv_voxel;

  const float3 V_enter = V + t_enter * VD;
  int3 vcell = make_int3(
      (int)floorf(V_enter.x), (int)floorf(V_enter.y), (int)floorf(V_enter.z));
  vcell.x = clamp(vcell.x, 0, SDF_BRICK_SIZE - 1);
  vcell.y = clamp(vcell.y, 0, SDF_BRICK_SIZE - 1);
  vcell.z = clamp(vcell.z, 0, SDF_BRICK_SIZE - 1);

  const int3 vstep = make_int3(VD.x > 0.0f ? 1 : (VD.x < 0.0f ? -1 : 0),
                                VD.y > 0.0f ? 1 : (VD.y < 0.0f ? -1 : 0),
                                VD.z > 0.0f ? 1 : (VD.z < 0.0f ? -1 : 0));

  const float3 vtDelta = make_float3(VD.x != 0.0f ? fabsf(1.0f / VD.x) : 1e30f,
                                      VD.y != 0.0f ? fabsf(1.0f / VD.y) : 1e30f,
                                      VD.z != 0.0f ? fabsf(1.0f / VD.z) : 1e30f);

  const float3 vbound = make_float3(VD.x > 0.0f ? float(vcell.x + 1) : float(vcell.x),
                                     VD.y > 0.0f ? float(vcell.y + 1) : float(vcell.y),
                                     VD.z > 0.0f ? float(vcell.z + 1) : float(vcell.z));

  float3 vtMax = make_float3(VD.x != 0.0f ? (vbound.x - V.x) / VD.x : 1e30f,
                               VD.y != 0.0f ? (vbound.y - V.y) / VD.y : 1e30f,
                               VD.z != 0.0f ? (vbound.z - V.z) / VD.z : 1e30f);

  float vt_current = t_enter;

  for (int vs = 0; vs < SDF_MAX_VOXEL_STEPS; vs++) {
    if (vcell.x < 0 || vcell.y < 0 || vcell.z < 0 || vcell.x > SDF_BRICK_SIZE - 1 ||
        vcell.y > SDF_BRICK_SIZE - 1 || vcell.z > SDF_BRICK_SIZE - 1)
    {
      break;
    }

    float vt_cell_exit = min(min(vtMax.x, vtMax.y), vtMax.z);
    vt_cell_exit = min(vt_cell_exit, t_brick_exit);

    /* Fetch 8 corners. */
    float s[8];
    sdf_fetch_corners(kg, atlas_off, vcell, slot_org, atlas_dim, s);

    float smin = min(min(min(s[0], s[1]), min(s[2], s[3])),
                     min(min(s[4], s[5]), min(s[6], s[7])));
    float smax = max(max(max(s[0], s[1]), max(s[2], s[3])),
                     max(max(s[4], s[5]), max(s[6], s[7])));

    if (smin <= 0.0f) {
      if (smax < 0.0f) {
        /* All negative: immediate hit. */
        isect->t = vt_current;
        isect->prim = sdf_index;
        isect->object = ksdf.object_id;
        isect->type = PRIMITIVE_SDF;
        isect->u = __int_as_float(brick_linear);
        isect->v = __int_as_float(brick_slot);
        return true;
      }

      /* Mixed signs: cubic solve. */
      float T_max = vt_cell_exit - vt_current;
      if (T_max > 1e-8f) {
        float k[8];
        sdf_trilinear_coeffs(s, k);

        float3 o_local = V + vt_current * VD - make_float3(float(vcell.x),
                                                              float(vcell.y),
                                                              float(vcell.z));
        o_local = clamp(o_local, zero_float3(), one_float3());
        float3 d_scaled = VD * T_max;

        float c[4];
        sdf_cubic_coeffs(k, o_local, d_scaled, c);

        if (c[0] < -1e-5f) {
          isect->t = vt_current;
          isect->prim = sdf_index;
          isect->object = ksdf.object_id;
          isect->type = PRIMITIVE_SDF;
          isect->u = __int_as_float(brick_linear);
          isect->v = __int_as_float(brick_slot);
          return true;
        }

        float u_hit = sdf_solve_cubic(c, 1.0f);
        if (u_hit >= 0.0f) {
          float hit_t = vt_current + u_hit * T_max;
          if (hit_t < isect->t) {
            isect->t = hit_t;
            isect->prim = sdf_index;
            isect->object = ksdf.object_id;
            isect->type = PRIMITIVE_SDF;
            isect->u = __int_as_float(brick_linear);
            isect->v = __int_as_float(brick_slot);
            return true;
          }
        }
      }
    }

    /* Advance voxel DDA. */
    if (vtMax.x < vtMax.y) {
      if (vtMax.x < vtMax.z) {
        vt_current = vtMax.x;
        vcell.x += vstep.x;
        vtMax.x += vtDelta.x;
      }
      else {
        vt_current = vtMax.z;
        vcell.z += vstep.z;
        vtMax.z += vtDelta.z;
      }
    }
    else {
      if (vtMax.y < vtMax.z) {
        vt_current = vtMax.y;
        vcell.y += vstep.y;
        vtMax.y += vtDelta.y;
      }
      else {
        vt_current = vtMax.z;
        vcell.z += vstep.z;
        vtMax.z += vtDelta.z;
      }
    }

    if (vt_current >= t_brick_exit) {
      break;
    }
  }

  return false;
}

/* -------------------------------------------------------------------- */
/* Per-brick shadow intersection (OptiX path).
 * Skips Newton-Raphson refinement — only detects root existence. */

ccl_device bool sdf_intersect_brick_shadow(KernelGlobals kg,
                                            ccl_private const Ray *ray,
                                            const float t_max,
                                            const int sdf_index,
                                            const int brick_linear,
                                            const int brick_slot)
{
  const KernelSDF ksdf = kernel_data_fetch(sdf_objects, sdf_index);
  const int grid_res = ksdf.grid_res;
  const float voxel_size = ksdf.voxel_size;
  const float3 origin = make_float3(ksdf.origin.x, ksdf.origin.y, ksdf.origin.z);
  const int bpa = ksdf.bricks_per_axis;
  const int atlas_off = ksdf.atlas_offset;
  const int atlas_dim = bpa * SDF_BRICK_STORAGE;

  /* Decode brick cell from linear index. */
  const int3 brick_cell = make_int3(brick_linear % grid_res,
                                     (brick_linear / grid_res) % grid_res,
                                     brick_linear / (grid_res * grid_res));

  /* Fully-inside brick: immediate shadow hit. */
  if (brick_slot == -2) {
    const float brick_world = float(SDF_BRICK_SIZE) * voxel_size;
    const float3 brick_min = origin + make_float3(float(brick_cell.x),
                                                    float(brick_cell.y),
                                                    float(brick_cell.z)) *
                                          brick_world;
    const float3 brick_max = brick_min + make_float3(brick_world, brick_world, brick_world);
    const float3 inv_dir = safe_divide(one_float3(), ray->D);
    const float3 t0 = (brick_min - ray->P) * inv_dir;
    const float3 t1 = (brick_max - ray->P) * inv_dir;
    const float3 t_lo = min(t0, t1);
    float t_enter = max(max(t_lo.x, t_lo.y), t_lo.z);
    t_enter = max(t_enter, voxel_size);
    return t_enter < t_max;
  }

  /* Active brick: voxel-level DDA. */
  const float inv_voxel = 1.0f / voxel_size;
  const int3 slot_org = sdf_slot_origin(brick_slot, bpa);
  const float3 brick_origin = origin + make_float3(float(brick_cell.x * SDF_BRICK_SIZE),
                                                     float(brick_cell.y * SDF_BRICK_SIZE),
                                                     float(brick_cell.z * SDF_BRICK_SIZE)) *
                                            voxel_size;

  const float3 brick_min = brick_origin;
  const float3 brick_max = brick_origin + make_float3(float(SDF_BRICK_SIZE),
                                                        float(SDF_BRICK_SIZE),
                                                        float(SDF_BRICK_SIZE)) *
                                               voxel_size;

  const float3 inv_dir = safe_divide(one_float3(), ray->D);
  const float3 t0 = (brick_min - ray->P) * inv_dir;
  const float3 t1 = (brick_max - ray->P) * inv_dir;
  const float3 t_lo = min(t0, t1);
  const float3 t_hi = max(t0, t1);
  float t_enter = max(max(t_lo.x, t_lo.y), t_lo.z);
  float t_brick_exit = min(min(t_hi.x, t_hi.y), t_hi.z);

  t_enter = max(t_enter, voxel_size);
  t_brick_exit = min(t_brick_exit, t_max);

  if (t_enter >= t_brick_exit) {
    return false;
  }

  /* Voxel-level DDA. */
  const float3 V = (ray->P - brick_origin) * inv_voxel;
  const float3 VD = ray->D * inv_voxel;

  const float3 V_enter = V + t_enter * VD;
  int3 vcell = make_int3(
      (int)floorf(V_enter.x), (int)floorf(V_enter.y), (int)floorf(V_enter.z));
  vcell.x = clamp(vcell.x, 0, SDF_BRICK_SIZE - 1);
  vcell.y = clamp(vcell.y, 0, SDF_BRICK_SIZE - 1);
  vcell.z = clamp(vcell.z, 0, SDF_BRICK_SIZE - 1);

  const int3 vstep = make_int3(VD.x > 0.0f ? 1 : (VD.x < 0.0f ? -1 : 0),
                                VD.y > 0.0f ? 1 : (VD.y < 0.0f ? -1 : 0),
                                VD.z > 0.0f ? 1 : (VD.z < 0.0f ? -1 : 0));

  const float3 vtDelta = make_float3(VD.x != 0.0f ? fabsf(1.0f / VD.x) : 1e30f,
                                      VD.y != 0.0f ? fabsf(1.0f / VD.y) : 1e30f,
                                      VD.z != 0.0f ? fabsf(1.0f / VD.z) : 1e30f);

  const float3 vbound = make_float3(VD.x > 0.0f ? float(vcell.x + 1) : float(vcell.x),
                                     VD.y > 0.0f ? float(vcell.y + 1) : float(vcell.y),
                                     VD.z > 0.0f ? float(vcell.z + 1) : float(vcell.z));

  float3 vtMax = make_float3(VD.x != 0.0f ? (vbound.x - V.x) / VD.x : 1e30f,
                               VD.y != 0.0f ? (vbound.y - V.y) / VD.y : 1e30f,
                               VD.z != 0.0f ? (vbound.z - V.z) / VD.z : 1e30f);

  float vt_current = t_enter;

  for (int vs = 0; vs < SDF_MAX_VOXEL_STEPS; vs++) {
    if (vcell.x < 0 || vcell.y < 0 || vcell.z < 0 || vcell.x > SDF_BRICK_SIZE - 1 ||
        vcell.y > SDF_BRICK_SIZE - 1 || vcell.z > SDF_BRICK_SIZE - 1)
    {
      break;
    }

    float vt_cell_exit = min(min(vtMax.x, vtMax.y), vtMax.z);
    vt_cell_exit = min(vt_cell_exit, t_brick_exit);

    float s[8];
    sdf_fetch_corners(kg, atlas_off, vcell, slot_org, atlas_dim, s);

    float smin = min(min(min(s[0], s[1]), min(s[2], s[3])),
                     min(min(s[4], s[5]), min(s[6], s[7])));
    float smax = max(max(max(s[0], s[1]), max(s[2], s[3])),
                     max(max(s[4], s[5]), max(s[6], s[7])));

    if (smin <= 0.0f) {
      if (smax < 0.0f) {
        return true; /* All negative: inside surface. */
      }

      /* Mixed signs: check root existence without NR. */
      float T_max = vt_cell_exit - vt_current;
      if (T_max > 1e-8f) {
        float k[8];
        sdf_trilinear_coeffs(s, k);

        float3 o_local = V + vt_current * VD - make_float3(float(vcell.x),
                                                              float(vcell.y),
                                                              float(vcell.z));
        o_local = clamp(o_local, zero_float3(), one_float3());
        float3 d_scaled = VD * T_max;

        float c[4];
        sdf_cubic_coeffs(k, o_local, d_scaled, c);

        if (c[0] < -1e-5f) {
          return true; /* Inside surface at ray entry. */
        }

        if (sdf_has_cubic_root(c, 1.0f)) {
          return true; /* Root exists — shadow is blocked. */
        }
      }
    }

    /* Advance voxel DDA. */
    if (vtMax.x < vtMax.y) {
      if (vtMax.x < vtMax.z) {
        vt_current = vtMax.x;
        vcell.x += vstep.x;
        vtMax.x += vtDelta.x;
      }
      else {
        vt_current = vtMax.z;
        vcell.z += vstep.z;
        vtMax.z += vtDelta.z;
      }
    }
    else {
      if (vtMax.y < vtMax.z) {
        vt_current = vtMax.y;
        vcell.y += vstep.y;
        vtMax.y += vtDelta.y;
      }
      else {
        vt_current = vtMax.z;
        vcell.z += vstep.z;
        vtMax.z += vtDelta.z;
      }
    }

    if (vt_current >= t_brick_exit) {
      break;
    }
  }

  return false;
}

/* -------------------------------------------------------------------- */
/* Per-shape brick intersection for TLAS/BLAS instanced mode.
 * Ray is already in the shape's local space (OptiX transforms automatically).
 * Uses KernelSDFShape for atlas parameters instead of KernelSDF. */

ccl_device bool sdf_intersect_brick_shape(KernelGlobals kg,
                                           ccl_private const Ray *ray,
                                           ccl_private Intersection *isect,
                                           const KernelSDFShape kshape,
                                           const KernelSDFInstance kinst,
                                           const int brick_linear,
                                           const int brick_slot)
{
  const int3 grid_res = make_int3(kshape.grid_res_x, kshape.grid_res_y, kshape.grid_res_z);
  const float voxel_size = kshape.voxel_size;
  const float3 origin = make_float3(kshape.origin.x, kshape.origin.y, kshape.origin.z);
  const int bpa = kshape.bricks_per_axis;
  const int atlas_off = kshape.atlas_offset;
  const int atlas_dim = bpa * SDF_BRICK_STORAGE;

  /* Decode brick cell from linear index (non-cubic grid). */
  const int3 brick_cell = make_int3(brick_linear % grid_res.x,
                                     (brick_linear / grid_res.x) % grid_res.y,
                                     brick_linear / (grid_res.x * grid_res.y));

  /* Fully-inside brick: immediate hit at AABB entry. */
  if (brick_slot == -2) {
    const float brick_world = float(SDF_BRICK_SIZE) * voxel_size;
    const float3 brick_min = origin + make_float3(float(brick_cell.x),
                                                    float(brick_cell.y),
                                                    float(brick_cell.z)) *
                                          brick_world;
    const float3 brick_max = brick_min + make_float3(brick_world, brick_world, brick_world);
    const float3 inv_dir = safe_divide(one_float3(), ray->D);
    const float3 t0 = (brick_min - ray->P) * inv_dir;
    const float3 t1 = (brick_max - ray->P) * inv_dir;
    const float3 t_lo = min(t0, t1);
    float t_enter = max(max(t_lo.x, t_lo.y), t_lo.z);
    t_enter = max(t_enter, voxel_size);
    if (t_enter >= isect->t) {
      return false;
    }
    isect->t = t_enter;
    isect->prim = 0;
    isect->object = kinst.object_id;
    isect->type = PRIMITIVE_SDF;
    isect->u = __int_as_float(brick_linear);
    isect->v = __int_as_float(-2);
    return true;
  }

  /* Active brick: voxel-level DDA. */
  const float inv_voxel = 1.0f / voxel_size;
  const int3 slot_org = sdf_slot_origin(brick_slot, bpa);
  const float3 brick_origin = origin + make_float3(float(brick_cell.x * SDF_BRICK_SIZE),
                                                     float(brick_cell.y * SDF_BRICK_SIZE),
                                                     float(brick_cell.z * SDF_BRICK_SIZE)) *
                                            voxel_size;

  const float3 brick_min = brick_origin;
  const float3 brick_max = brick_origin + make_float3(float(SDF_BRICK_SIZE),
                                                        float(SDF_BRICK_SIZE),
                                                        float(SDF_BRICK_SIZE)) *
                                               voxel_size;

  const float3 inv_dir = safe_divide(one_float3(), ray->D);
  const float3 t0 = (brick_min - ray->P) * inv_dir;
  const float3 t1 = (brick_max - ray->P) * inv_dir;
  const float3 t_lo = min(t0, t1);
  const float3 t_hi = max(t0, t1);
  float t_enter = max(max(t_lo.x, t_lo.y), t_lo.z);
  float t_brick_exit = min(min(t_hi.x, t_hi.y), t_hi.z);
  t_enter = max(t_enter, voxel_size);
  t_brick_exit = min(t_brick_exit, isect->t);

  if (t_enter >= t_brick_exit) {
    return false;
  }

  const float3 V = (ray->P - brick_origin) * inv_voxel;
  const float3 VD = ray->D * inv_voxel;
  const float3 V_enter = V + t_enter * VD;
  int3 vcell = make_int3(
      (int)floorf(V_enter.x), (int)floorf(V_enter.y), (int)floorf(V_enter.z));
  vcell.x = clamp(vcell.x, 0, SDF_BRICK_SIZE - 1);
  vcell.y = clamp(vcell.y, 0, SDF_BRICK_SIZE - 1);
  vcell.z = clamp(vcell.z, 0, SDF_BRICK_SIZE - 1);

  const int3 vstep = make_int3(VD.x > 0.0f ? 1 : (VD.x < 0.0f ? -1 : 0),
                                VD.y > 0.0f ? 1 : (VD.y < 0.0f ? -1 : 0),
                                VD.z > 0.0f ? 1 : (VD.z < 0.0f ? -1 : 0));
  const float3 vtDelta = make_float3(VD.x != 0.0f ? fabsf(1.0f / VD.x) : 1e30f,
                                      VD.y != 0.0f ? fabsf(1.0f / VD.y) : 1e30f,
                                      VD.z != 0.0f ? fabsf(1.0f / VD.z) : 1e30f);
  const float3 vbound = make_float3(VD.x > 0.0f ? float(vcell.x + 1) : float(vcell.x),
                                     VD.y > 0.0f ? float(vcell.y + 1) : float(vcell.y),
                                     VD.z > 0.0f ? float(vcell.z + 1) : float(vcell.z));
  float3 vtMax = make_float3(VD.x != 0.0f ? (vbound.x - V.x) / VD.x : 1e30f,
                               VD.y != 0.0f ? (vbound.y - V.y) / VD.y : 1e30f,
                               VD.z != 0.0f ? (vbound.z - V.z) / VD.z : 1e30f);

  float vt_current = t_enter;

  for (int vs = 0; vs < SDF_MAX_VOXEL_STEPS; vs++) {
    if (vcell.x < 0 || vcell.y < 0 || vcell.z < 0 || vcell.x > SDF_BRICK_SIZE - 1 ||
        vcell.y > SDF_BRICK_SIZE - 1 || vcell.z > SDF_BRICK_SIZE - 1)
    {
      break;
    }

    float vt_cell_exit = min(min(vtMax.x, vtMax.y), vtMax.z);
    vt_cell_exit = min(vt_cell_exit, t_brick_exit);

    float s[8];
    sdf_fetch_corners_shape(kg, atlas_off, vcell, slot_org, atlas_dim, voxel_size, s);

    float smin = min(min(min(s[0], s[1]), min(s[2], s[3])),
                     min(min(s[4], s[5]), min(s[6], s[7])));
    float smax = max(max(max(s[0], s[1]), max(s[2], s[3])),
                     max(max(s[4], s[5]), max(s[6], s[7])));

    if (smin <= 0.0f) {
      if (smax < 0.0f) {
        isect->t = vt_current;
        isect->prim = 0;
        isect->object = kinst.object_id;
        isect->type = PRIMITIVE_SDF;
        isect->u = __int_as_float(brick_linear);
        isect->v = __int_as_float(brick_slot);
        return true;
      }

      float T_max = vt_cell_exit - vt_current;
      if (T_max > 1e-8f) {
        float k[8];
        sdf_trilinear_coeffs(s, k);

        float3 o_local = V + vt_current * VD - make_float3(float(vcell.x),
                                                              float(vcell.y),
                                                              float(vcell.z));
        o_local = clamp(o_local, zero_float3(), one_float3());
        float3 d_scaled = VD * T_max;

        float c[4];
        sdf_cubic_coeffs(k, o_local, d_scaled, c);

        if (c[0] < -1e-5f) {
          isect->t = vt_current;
          isect->prim = 0;
          isect->object = kinst.object_id;
          isect->type = PRIMITIVE_SDF;
          isect->u = __int_as_float(brick_linear);
          isect->v = __int_as_float(brick_slot);
          return true;
        }

        float u_hit = sdf_solve_cubic(c, 1.0f);
        if (u_hit >= 0.0f) {
          float hit_t = vt_current + u_hit * T_max;
          if (hit_t < isect->t) {
            isect->t = hit_t;
            isect->prim = 0;
            isect->object = kinst.object_id;
            isect->type = PRIMITIVE_SDF;
            isect->u = __int_as_float(brick_linear);
            isect->v = __int_as_float(brick_slot);
            return true;
          }
        }
      }
    }

    /* Advance voxel DDA. */
    if (vtMax.x < vtMax.y) {
      if (vtMax.x < vtMax.z) {
        vt_current = vtMax.x; vcell.x += vstep.x; vtMax.x += vtDelta.x;
      }
      else {
        vt_current = vtMax.z; vcell.z += vstep.z; vtMax.z += vtDelta.z;
      }
    }
    else {
      if (vtMax.y < vtMax.z) {
        vt_current = vtMax.y; vcell.y += vstep.y; vtMax.y += vtDelta.y;
      }
      else {
        vt_current = vtMax.z; vcell.z += vstep.z; vtMax.z += vtDelta.z;
      }
    }
    if (vt_current >= t_brick_exit) {
      break;
    }
  }
  return false;
}

/* Per-shape shadow intersection. */
ccl_device bool sdf_intersect_brick_shape_shadow(KernelGlobals kg,
                                                  ccl_private const Ray *ray,
                                                  const float t_max,
                                                  const KernelSDFShape kshape,
                                                  const int brick_linear,
                                                  const int brick_slot)
{
  const int3 grid_res = make_int3(kshape.grid_res_x, kshape.grid_res_y, kshape.grid_res_z);
  const float voxel_size = kshape.voxel_size;
  const float3 origin = make_float3(kshape.origin.x, kshape.origin.y, kshape.origin.z);
  const int bpa = kshape.bricks_per_axis;
  const int atlas_off = kshape.atlas_offset;
  const int atlas_dim = bpa * SDF_BRICK_STORAGE;

  const int3 brick_cell = make_int3(brick_linear % grid_res.x,
                                     (brick_linear / grid_res.x) % grid_res.y,
                                     brick_linear / (grid_res.x * grid_res.y));

  if (brick_slot == -2) {
    const float brick_world = float(SDF_BRICK_SIZE) * voxel_size;
    const float3 brick_min = origin + make_float3(float(brick_cell.x),
                                                    float(brick_cell.y),
                                                    float(brick_cell.z)) *
                                          brick_world;
    const float3 brick_max = brick_min + make_float3(brick_world, brick_world, brick_world);
    const float3 inv_dir = safe_divide(one_float3(), ray->D);
    const float3 t0 = (brick_min - ray->P) * inv_dir;
    const float3 t1 = (brick_max - ray->P) * inv_dir;
    const float3 t_lo = min(t0, t1);
    float t_enter = max(max(t_lo.x, t_lo.y), t_lo.z);
    t_enter = max(t_enter, voxel_size);
    return t_enter < t_max;
  }

  const float inv_voxel = 1.0f / voxel_size;
  const int3 slot_org = sdf_slot_origin(brick_slot, bpa);
  const float3 brick_origin = origin + make_float3(float(brick_cell.x * SDF_BRICK_SIZE),
                                                     float(brick_cell.y * SDF_BRICK_SIZE),
                                                     float(brick_cell.z * SDF_BRICK_SIZE)) *
                                            voxel_size;
  const float3 brick_min = brick_origin;
  const float3 brick_max = brick_origin + make_float3(float(SDF_BRICK_SIZE),
                                                        float(SDF_BRICK_SIZE),
                                                        float(SDF_BRICK_SIZE)) *
                                               voxel_size;

  const float3 inv_dir = safe_divide(one_float3(), ray->D);
  const float3 t0 = (brick_min - ray->P) * inv_dir;
  const float3 t1 = (brick_max - ray->P) * inv_dir;
  const float3 t_lo = min(t0, t1);
  const float3 t_hi = max(t0, t1);
  float t_enter = max(max(t_lo.x, t_lo.y), t_lo.z);
  float t_brick_exit = min(min(t_hi.x, t_hi.y), t_hi.z);
  t_enter = max(t_enter, voxel_size);
  t_brick_exit = min(t_brick_exit, t_max);

  if (t_enter >= t_brick_exit) {
    return false;
  }

  const float3 V = (ray->P - brick_origin) * inv_voxel;
  const float3 VD = ray->D * inv_voxel;
  const float3 V_enter = V + t_enter * VD;
  int3 vcell = make_int3(
      (int)floorf(V_enter.x), (int)floorf(V_enter.y), (int)floorf(V_enter.z));
  vcell.x = clamp(vcell.x, 0, SDF_BRICK_SIZE - 1);
  vcell.y = clamp(vcell.y, 0, SDF_BRICK_SIZE - 1);
  vcell.z = clamp(vcell.z, 0, SDF_BRICK_SIZE - 1);

  const int3 vstep = make_int3(VD.x > 0.0f ? 1 : (VD.x < 0.0f ? -1 : 0),
                                VD.y > 0.0f ? 1 : (VD.y < 0.0f ? -1 : 0),
                                VD.z > 0.0f ? 1 : (VD.z < 0.0f ? -1 : 0));
  const float3 vtDelta = make_float3(VD.x != 0.0f ? fabsf(1.0f / VD.x) : 1e30f,
                                      VD.y != 0.0f ? fabsf(1.0f / VD.y) : 1e30f,
                                      VD.z != 0.0f ? fabsf(1.0f / VD.z) : 1e30f);
  const float3 vbound = make_float3(VD.x > 0.0f ? float(vcell.x + 1) : float(vcell.x),
                                     VD.y > 0.0f ? float(vcell.y + 1) : float(vcell.y),
                                     VD.z > 0.0f ? float(vcell.z + 1) : float(vcell.z));
  float3 vtMax = make_float3(VD.x != 0.0f ? (vbound.x - V.x) / VD.x : 1e30f,
                               VD.y != 0.0f ? (vbound.y - V.y) / VD.y : 1e30f,
                               VD.z != 0.0f ? (vbound.z - V.z) / VD.z : 1e30f);

  float vt_current = t_enter;

  for (int vs = 0; vs < SDF_MAX_VOXEL_STEPS; vs++) {
    if (vcell.x < 0 || vcell.y < 0 || vcell.z < 0 || vcell.x > SDF_BRICK_SIZE - 1 ||
        vcell.y > SDF_BRICK_SIZE - 1 || vcell.z > SDF_BRICK_SIZE - 1)
    {
      break;
    }

    float vt_cell_exit = min(min(vtMax.x, vtMax.y), vtMax.z);
    vt_cell_exit = min(vt_cell_exit, t_brick_exit);

    float s[8];
    sdf_fetch_corners_shape(kg, atlas_off, vcell, slot_org, atlas_dim, voxel_size, s);

    float smin = min(min(min(s[0], s[1]), min(s[2], s[3])),
                     min(min(s[4], s[5]), min(s[6], s[7])));

    if (smin <= 0.0f) {
      float smax = max(max(max(s[0], s[1]), max(s[2], s[3])),
                       max(max(s[4], s[5]), max(s[6], s[7])));
      if (smax < 0.0f) {
        return true;
      }
      float T_max = vt_cell_exit - vt_current;
      if (T_max > 1e-8f) {
        float k[8];
        sdf_trilinear_coeffs(s, k);
        float3 o_local = V + vt_current * VD - make_float3(float(vcell.x),
                                                              float(vcell.y),
                                                              float(vcell.z));
        o_local = clamp(o_local, zero_float3(), one_float3());
        float3 d_scaled = VD * T_max;
        float c[4];
        sdf_cubic_coeffs(k, o_local, d_scaled, c);
        if (c[0] < -1e-5f || sdf_has_cubic_root(c, 1.0f)) {
          return true;
        }
      }
    }

    if (vtMax.x < vtMax.y) {
      if (vtMax.x < vtMax.z) {
        vt_current = vtMax.x; vcell.x += vstep.x; vtMax.x += vtDelta.x;
      }
      else {
        vt_current = vtMax.z; vcell.z += vstep.z; vtMax.z += vtDelta.z;
      }
    }
    else {
      if (vtMax.y < vtMax.z) {
        vt_current = vtMax.y; vcell.y += vstep.y; vtMax.y += vtDelta.y;
      }
      else {
        vt_current = vtMax.z; vcell.z += vstep.z; vtMax.z += vtDelta.z;
      }
    }
    if (vt_current >= t_brick_exit) {
      break;
    }
  }
  return false;
}

/* -------------------------------------------------------------------- */
/* Two-level DDA ray march (CPU/BVH2 path). */

ccl_device bool sdf_intersect(KernelGlobals kg,
                               ccl_private const Ray *ray,
                               ccl_private Intersection *isect,
                               const int sdf_index)
{
  const KernelSDF ksdf = kernel_data_fetch(sdf_objects, sdf_index);
  const int grid_res = ksdf.grid_res;
  const float voxel_size = ksdf.voxel_size;
  const float3 origin = make_float3(ksdf.origin.x, ksdf.origin.y, ksdf.origin.z);
  const int bpa = ksdf.bricks_per_axis;
  const int atlas_off = ksdf.atlas_offset;
  const int indir_off = ksdf.indirection_offset;
  const int atlas_dim = bpa * SDF_BRICK_STORAGE;

  /* Clip ray to atlas AABB. */
  const int3 total_res = make_int3(grid_res * SDF_BRICK_SIZE,
                                   grid_res * SDF_BRICK_SIZE,
                                   grid_res * SDF_BRICK_SIZE);
  const float3 grid_min = origin;
  const float3 grid_max = origin + make_float3(float(total_res.x),
                                                float(total_res.y),
                                                float(total_res.z)) *
                                       voxel_size;

  const float3 inv_dir = safe_divide(one_float3(), ray->D);
  const float3 t0 = (grid_min - ray->P) * inv_dir;
  const float3 t1 = (grid_max - ray->P) * inv_dir;
  const float3 t_lo = min(t0, t1);
  const float3 t_hi = max(t0, t1);
  float t_enter = max(max(t_lo.x, t_lo.y), t_lo.z);
  const float t_exit_grid = min(min(t_hi.x, t_hi.y), t_hi.z);

  if (t_enter > t_exit_grid || t_exit_grid < 0.0f) {
    return false;
  }
  /* Start slightly ahead to avoid self-intersection on shadow/bounce rays.
   * Without this, secondary rays that originate on the SDF surface immediately
   * re-intersect the same surface, causing dark noise ("jeans texture").
   * The offset is 1 voxel — enough to clear the surface voxel. */
  const float t_self_offset = voxel_size;
  t_enter = max(t_enter, t_self_offset);

  /* Limit to current closest hit. */
  const float t_exit = min(t_exit_grid, isect->t);
  if (t_enter >= t_exit) {
    return false;
  }

  /* Brick-level DDA setup. */
  const float inv_voxel = 1.0f / voxel_size;
  const float brick_world = float(SDF_BRICK_SIZE) * voxel_size;
  const float inv_brick = 1.0f / brick_world;

  const float3 B = (ray->P - origin) * inv_brick;
  const float3 BD = ray->D * inv_brick;

  float3 B_enter = B + t_enter * BD;
  int3 brick_cell = make_int3((int)floorf(B_enter.x),
                              (int)floorf(B_enter.y),
                              (int)floorf(B_enter.z));
  brick_cell.x = clamp(brick_cell.x, 0, grid_res - 1);
  brick_cell.y = clamp(brick_cell.y, 0, grid_res - 1);
  brick_cell.z = clamp(brick_cell.z, 0, grid_res - 1);

  const int3 brick_step = make_int3(BD.x > 0.0f ? 1 : (BD.x < 0.0f ? -1 : 0),
                                    BD.y > 0.0f ? 1 : (BD.y < 0.0f ? -1 : 0),
                                    BD.z > 0.0f ? 1 : (BD.z < 0.0f ? -1 : 0));

  const float3 brick_tDelta = make_float3(BD.x != 0.0f ? fabsf(1.0f / BD.x) : 1e30f,
                                          BD.y != 0.0f ? fabsf(1.0f / BD.y) : 1e30f,
                                          BD.z != 0.0f ? fabsf(1.0f / BD.z) : 1e30f);

  float3 brick_boundary = make_float3(BD.x > 0.0f ? float(brick_cell.x + 1) : float(brick_cell.x),
                                      BD.y > 0.0f ? float(brick_cell.y + 1) : float(brick_cell.y),
                                      BD.z > 0.0f ? float(brick_cell.z + 1) : float(brick_cell.z));

  float3 brick_tMax = make_float3(BD.x != 0.0f ? (brick_boundary.x - B.x) / BD.x : 1e30f,
                                  BD.y != 0.0f ? (brick_boundary.y - B.y) / BD.y : 1e30f,
                                  BD.z != 0.0f ? (brick_boundary.z - B.z) / BD.z : 1e30f);

  /* Two-level DDA traversal. */
  float hit_t = -1.0f;
  int hit_brick_slot = -1;
  int3 hit_brick = make_int3(0, 0, 0);
  float t_current = t_enter;

  for (int bstep = 0; bstep < SDF_MAX_BRICK_STEPS; bstep++) {
    if (brick_cell.x < 0 || brick_cell.y < 0 || brick_cell.z < 0 ||
        brick_cell.x >= grid_res || brick_cell.y >= grid_res || brick_cell.z >= grid_res)
    {
      break;
    }

    float t_brick_exit = min(min(brick_tMax.x, brick_tMax.y), brick_tMax.z);
    t_brick_exit = min(t_brick_exit, t_exit);

    /* Read indirection. */
    int slot = sdf_fetch_indirection(
        kg, indir_off, brick_cell.x, brick_cell.y, brick_cell.z, grid_res);

    if (slot == -2) {
      /* Fully inside: immediate hit at entry.
       * Keep slot=-2 so shader_setup uses fallback normal (-ray->D). */
      hit_t = t_current;
      hit_brick = brick_cell;
      hit_brick_slot = -2;
      break;
    }

    if (slot >= 0) {
      /* Active brick: voxel-level DDA. */
      const int3 slot_org = sdf_slot_origin(slot, bpa);
      float3 brick_origin = origin + make_float3(float(brick_cell.x * SDF_BRICK_SIZE),
                                                  float(brick_cell.y * SDF_BRICK_SIZE),
                                                  float(brick_cell.z * SDF_BRICK_SIZE)) *
                                         voxel_size;
      float3 V = (ray->P - brick_origin) * inv_voxel;
      float3 VD = ray->D * inv_voxel;

      float3 V_enter = V + t_current * VD;
      int3 vcell = make_int3(
          (int)floorf(V_enter.x), (int)floorf(V_enter.y), (int)floorf(V_enter.z));
      vcell.x = clamp(vcell.x, 0, SDF_BRICK_SIZE - 1);
      vcell.y = clamp(vcell.y, 0, SDF_BRICK_SIZE - 1);
      vcell.z = clamp(vcell.z, 0, SDF_BRICK_SIZE - 1);

      int3 vstep = make_int3(VD.x > 0.0f ? 1 : (VD.x < 0.0f ? -1 : 0),
                             VD.y > 0.0f ? 1 : (VD.y < 0.0f ? -1 : 0),
                             VD.z > 0.0f ? 1 : (VD.z < 0.0f ? -1 : 0));

      float3 vtDelta = make_float3(VD.x != 0.0f ? fabsf(1.0f / VD.x) : 1e30f,
                                   VD.y != 0.0f ? fabsf(1.0f / VD.y) : 1e30f,
                                   VD.z != 0.0f ? fabsf(1.0f / VD.z) : 1e30f);

      float3 vbound = make_float3(VD.x > 0.0f ? float(vcell.x + 1) : float(vcell.x),
                                  VD.y > 0.0f ? float(vcell.y + 1) : float(vcell.y),
                                  VD.z > 0.0f ? float(vcell.z + 1) : float(vcell.z));

      float3 vtMax = make_float3(VD.x != 0.0f ? (vbound.x - V.x) / VD.x : 1e30f,
                                 VD.y != 0.0f ? (vbound.y - V.y) / VD.y : 1e30f,
                                 VD.z != 0.0f ? (vbound.z - V.z) / VD.z : 1e30f);

      float vt_current = t_current;
      bool voxel_hit = false;

      for (int vs = 0; vs < SDF_MAX_VOXEL_STEPS; vs++) {
        if (vcell.x < 0 || vcell.y < 0 || vcell.z < 0 || vcell.x > SDF_BRICK_SIZE - 1 ||
            vcell.y > SDF_BRICK_SIZE - 1 || vcell.z > SDF_BRICK_SIZE - 1)
        {
          break;
        }

        float vt_cell_exit = min(min(vtMax.x, vtMax.y), vtMax.z);
        vt_cell_exit = min(vt_cell_exit, t_brick_exit);

        /* Fetch 8 corners. */
        float s[8];
        sdf_fetch_corners(kg, atlas_off, vcell, slot_org, atlas_dim, s);

        float smin = min(min(min(s[0], s[1]), min(s[2], s[3])),
                         min(min(s[4], s[5]), min(s[6], s[7])));
        float smax = max(max(max(s[0], s[1]), max(s[2], s[3])),
                         max(max(s[4], s[5]), max(s[6], s[7])));

        if (smin <= 0.0f) {
          if (smax < 0.0f) {
            /* All negative: immediate hit. */
            hit_t = vt_current;
            hit_brick = brick_cell;
            hit_brick_slot = slot;
            voxel_hit = true;
            break;
          }

          /* Mixed signs: cubic solve. */
          float T_max = vt_cell_exit - vt_current;
          if (T_max > 1e-8f) {
            float k[8];
            sdf_trilinear_coeffs(s, k);

            float3 o_local = V + vt_current * VD - make_float3(float(vcell.x),
                                                                float(vcell.y),
                                                                float(vcell.z));
            o_local = clamp(o_local, zero_float3(), one_float3());
            float3 d_scaled = VD * T_max;

            float c[4];
            sdf_cubic_coeffs(k, o_local, d_scaled, c);

            if (c[0] < -1e-5f) {
              /* Genuinely inside the surface at ray origin.
               * Use a threshold to avoid false positives from secondary rays
               * that start on the surface with c[0] ≈ 0. */
              hit_t = vt_current;
              hit_brick = brick_cell;
              hit_brick_slot = slot;
              voxel_hit = true;
              break;
            }

            float u_hit = sdf_solve_cubic(c, 1.0f);
            if (u_hit >= 0.0f) {
              hit_t = vt_current + u_hit * T_max;
              hit_brick = brick_cell;
              hit_brick_slot = slot;
              voxel_hit = true;
              break;
            }
          }
        }

        /* Advance voxel DDA. */
        if (vtMax.x < vtMax.y) {
          if (vtMax.x < vtMax.z) {
            vt_current = vtMax.x;
            vcell.x += vstep.x;
            vtMax.x += vtDelta.x;
          }
          else {
            vt_current = vtMax.z;
            vcell.z += vstep.z;
            vtMax.z += vtDelta.z;
          }
        }
        else {
          if (vtMax.y < vtMax.z) {
            vt_current = vtMax.y;
            vcell.y += vstep.y;
            vtMax.y += vtDelta.y;
          }
          else {
            vt_current = vtMax.z;
            vcell.z += vstep.z;
            vtMax.z += vtDelta.z;
          }
        }

        if (vt_current >= t_brick_exit) {
          break;
        }
      }

      if (voxel_hit) {
        break;
      }
    }

    /* Advance brick-level DDA. */
    if (brick_tMax.x < brick_tMax.y) {
      if (brick_tMax.x < brick_tMax.z) {
        t_current = brick_tMax.x;
        brick_cell.x += brick_step.x;
        brick_tMax.x += brick_tDelta.x;
      }
      else {
        t_current = brick_tMax.z;
        brick_cell.z += brick_step.z;
        brick_tMax.z += brick_tDelta.z;
      }
    }
    else {
      if (brick_tMax.y < brick_tMax.z) {
        t_current = brick_tMax.y;
        brick_cell.y += brick_step.y;
        brick_tMax.y += brick_tDelta.y;
      }
      else {
        t_current = brick_tMax.z;
        brick_cell.z += brick_step.z;
        brick_tMax.z += brick_tDelta.z;
      }
    }

    if (t_current > t_exit) {
      break;
    }
  }

  if (hit_t < 0.0f || hit_t >= isect->t) {
    return false;
  }

  /* Record intersection.
   * Encode brick cell index and slot in u/v so shader_setup can
   * recover the exact brick without recomputing from the float hit position. */
  isect->t = hit_t;
  isect->prim = sdf_index;
  isect->object = ksdf.object_id;
  isect->type = PRIMITIVE_SDF;
  const int brick_linear = hit_brick.x + hit_brick.y * grid_res +
                           hit_brick.z * grid_res * grid_res;
  isect->u = __int_as_float(brick_linear);
  isect->v = __int_as_float(hit_brick_slot);

  return true;
}

/* March all SDF objects, return closest hit. */
ccl_device bool sdf_intersect_all(KernelGlobals kg,
                                   ccl_private const Ray *ray,
                                   ccl_private Intersection *isect,
                                   const uint /*visibility*/)
{
  bool hit = false;
  const int num_sdfs = kernel_data.num_sdfs;

  for (int i = 0; i < num_sdfs; i++) {
    if (sdf_intersect(kg, ray, isect, i)) {
      hit = true;
    }
  }

  return hit;
}

/* -------------------------------------------------------------------- */
/* Shadow ray intersection (CPU/BVH2 path).
 * Uses sdf_has_cubic_root() — skips Newton-Raphson for ~6-14% speedup
 * on shadow rays (per JCGT 2022 paper benchmarks). */

ccl_device bool sdf_intersect_shadow(KernelGlobals kg,
                                      ccl_private const Ray *ray,
                                      const float t_max,
                                      const int sdf_index)
{
  const KernelSDF ksdf = kernel_data_fetch(sdf_objects, sdf_index);
  const int grid_res = ksdf.grid_res;
  const float voxel_size = ksdf.voxel_size;
  const float3 origin = make_float3(ksdf.origin.x, ksdf.origin.y, ksdf.origin.z);
  const int bpa = ksdf.bricks_per_axis;
  const int atlas_off = ksdf.atlas_offset;
  const int indir_off = ksdf.indirection_offset;
  const int atlas_dim = bpa * SDF_BRICK_STORAGE;

  /* Clip ray to atlas AABB. */
  const int3 total_res = make_int3(grid_res * SDF_BRICK_SIZE,
                                   grid_res * SDF_BRICK_SIZE,
                                   grid_res * SDF_BRICK_SIZE);
  const float3 grid_min = origin;
  const float3 grid_max = origin + make_float3(float(total_res.x),
                                                float(total_res.y),
                                                float(total_res.z)) *
                                       voxel_size;

  const float3 inv_dir = safe_divide(one_float3(), ray->D);
  const float3 t0 = (grid_min - ray->P) * inv_dir;
  const float3 t1 = (grid_max - ray->P) * inv_dir;
  const float3 t_lo = min(t0, t1);
  const float3 t_hi = max(t0, t1);
  float t_enter = max(max(t_lo.x, t_lo.y), t_lo.z);
  const float t_exit_grid = min(min(t_hi.x, t_hi.y), t_hi.z);

  if (t_enter > t_exit_grid || t_exit_grid < 0.0f) {
    return false;
  }
  t_enter = max(t_enter, voxel_size);

  const float t_exit = min(t_exit_grid, t_max);
  if (t_enter >= t_exit) {
    return false;
  }

  /* Brick-level DDA setup. */
  const float inv_voxel = 1.0f / voxel_size;
  const float brick_world = float(SDF_BRICK_SIZE) * voxel_size;
  const float inv_brick = 1.0f / brick_world;

  const float3 B = (ray->P - origin) * inv_brick;
  const float3 BD = ray->D * inv_brick;

  float3 B_enter = B + t_enter * BD;
  int3 brick_cell = make_int3((int)floorf(B_enter.x),
                              (int)floorf(B_enter.y),
                              (int)floorf(B_enter.z));
  brick_cell.x = clamp(brick_cell.x, 0, grid_res - 1);
  brick_cell.y = clamp(brick_cell.y, 0, grid_res - 1);
  brick_cell.z = clamp(brick_cell.z, 0, grid_res - 1);

  const int3 brick_step = make_int3(BD.x > 0.0f ? 1 : (BD.x < 0.0f ? -1 : 0),
                                    BD.y > 0.0f ? 1 : (BD.y < 0.0f ? -1 : 0),
                                    BD.z > 0.0f ? 1 : (BD.z < 0.0f ? -1 : 0));

  const float3 brick_tDelta = make_float3(BD.x != 0.0f ? fabsf(1.0f / BD.x) : 1e30f,
                                          BD.y != 0.0f ? fabsf(1.0f / BD.y) : 1e30f,
                                          BD.z != 0.0f ? fabsf(1.0f / BD.z) : 1e30f);

  float3 brick_boundary = make_float3(BD.x > 0.0f ? float(brick_cell.x + 1) : float(brick_cell.x),
                                      BD.y > 0.0f ? float(brick_cell.y + 1) : float(brick_cell.y),
                                      BD.z > 0.0f ? float(brick_cell.z + 1) : float(brick_cell.z));

  float3 brick_tMax = make_float3(BD.x != 0.0f ? (brick_boundary.x - B.x) / BD.x : 1e30f,
                                  BD.y != 0.0f ? (brick_boundary.y - B.y) / BD.y : 1e30f,
                                  BD.z != 0.0f ? (brick_boundary.z - B.z) / BD.z : 1e30f);

  float t_current = t_enter;

  for (int bstep = 0; bstep < SDF_MAX_BRICK_STEPS; bstep++) {
    if (brick_cell.x < 0 || brick_cell.y < 0 || brick_cell.z < 0 ||
        brick_cell.x >= grid_res || brick_cell.y >= grid_res || brick_cell.z >= grid_res)
    {
      break;
    }

    float t_brick_exit = min(min(brick_tMax.x, brick_tMax.y), brick_tMax.z);
    t_brick_exit = min(t_brick_exit, t_exit);

    int slot = sdf_fetch_indirection(
        kg, indir_off, brick_cell.x, brick_cell.y, brick_cell.z, grid_res);

    if (slot == -2) {
      return true; /* Fully inside: shadow blocked. */
    }

    if (slot >= 0) {
      /* Active brick: voxel-level DDA with shadow fast path. */
      const int3 slot_org = sdf_slot_origin(slot, bpa);
      float3 brick_origin = origin + make_float3(float(brick_cell.x * SDF_BRICK_SIZE),
                                                  float(brick_cell.y * SDF_BRICK_SIZE),
                                                  float(brick_cell.z * SDF_BRICK_SIZE)) *
                                         voxel_size;
      float3 V = (ray->P - brick_origin) * inv_voxel;
      float3 VD = ray->D * inv_voxel;

      float3 V_enter = V + t_current * VD;
      int3 vcell = make_int3(
          (int)floorf(V_enter.x), (int)floorf(V_enter.y), (int)floorf(V_enter.z));
      vcell.x = clamp(vcell.x, 0, SDF_BRICK_SIZE - 1);
      vcell.y = clamp(vcell.y, 0, SDF_BRICK_SIZE - 1);
      vcell.z = clamp(vcell.z, 0, SDF_BRICK_SIZE - 1);

      int3 vstep = make_int3(VD.x > 0.0f ? 1 : (VD.x < 0.0f ? -1 : 0),
                             VD.y > 0.0f ? 1 : (VD.y < 0.0f ? -1 : 0),
                             VD.z > 0.0f ? 1 : (VD.z < 0.0f ? -1 : 0));

      float3 vtDelta = make_float3(VD.x != 0.0f ? fabsf(1.0f / VD.x) : 1e30f,
                                   VD.y != 0.0f ? fabsf(1.0f / VD.y) : 1e30f,
                                   VD.z != 0.0f ? fabsf(1.0f / VD.z) : 1e30f);

      float3 vbound = make_float3(VD.x > 0.0f ? float(vcell.x + 1) : float(vcell.x),
                                  VD.y > 0.0f ? float(vcell.y + 1) : float(vcell.y),
                                  VD.z > 0.0f ? float(vcell.z + 1) : float(vcell.z));

      float3 vtMax = make_float3(VD.x != 0.0f ? (vbound.x - V.x) / VD.x : 1e30f,
                                 VD.y != 0.0f ? (vbound.y - V.y) / VD.y : 1e30f,
                                 VD.z != 0.0f ? (vbound.z - V.z) / VD.z : 1e30f);

      float vt_current = t_current;

      for (int vs = 0; vs < SDF_MAX_VOXEL_STEPS; vs++) {
        if (vcell.x < 0 || vcell.y < 0 || vcell.z < 0 || vcell.x > SDF_BRICK_SIZE - 1 ||
            vcell.y > SDF_BRICK_SIZE - 1 || vcell.z > SDF_BRICK_SIZE - 1)
        {
          break;
        }

        float vt_cell_exit = min(min(vtMax.x, vtMax.y), vtMax.z);
        vt_cell_exit = min(vt_cell_exit, t_brick_exit);

        float s[8];
        sdf_fetch_corners(kg, atlas_off, vcell, slot_org, atlas_dim, s);

        float smin = min(min(min(s[0], s[1]), min(s[2], s[3])),
                         min(min(s[4], s[5]), min(s[6], s[7])));
        float smax = max(max(max(s[0], s[1]), max(s[2], s[3])),
                         max(max(s[4], s[5]), max(s[6], s[7])));

        if (smin <= 0.0f) {
          if (smax < 0.0f) {
            return true; /* All negative: shadow blocked. */
          }

          float T_max = vt_cell_exit - vt_current;
          if (T_max > 1e-8f) {
            float k[8];
            sdf_trilinear_coeffs(s, k);

            float3 o_local = V + vt_current * VD - make_float3(float(vcell.x),
                                                                float(vcell.y),
                                                                float(vcell.z));
            o_local = clamp(o_local, zero_float3(), one_float3());
            float3 d_scaled = VD * T_max;

            float c[4];
            sdf_cubic_coeffs(k, o_local, d_scaled, c);

            if (c[0] < -1e-5f) {
              return true;
            }

            /* Shadow fast path: existence check only, no NR refinement. */
            if (sdf_has_cubic_root(c, 1.0f)) {
              return true;
            }
          }
        }

        /* Advance voxel DDA. */
        if (vtMax.x < vtMax.y) {
          if (vtMax.x < vtMax.z) {
            vt_current = vtMax.x;
            vcell.x += vstep.x;
            vtMax.x += vtDelta.x;
          }
          else {
            vt_current = vtMax.z;
            vcell.z += vstep.z;
            vtMax.z += vtDelta.z;
          }
        }
        else {
          if (vtMax.y < vtMax.z) {
            vt_current = vtMax.y;
            vcell.y += vstep.y;
            vtMax.y += vtDelta.y;
          }
          else {
            vt_current = vtMax.z;
            vcell.z += vstep.z;
            vtMax.z += vtDelta.z;
          }
        }

        if (vt_current >= t_brick_exit) {
          break;
        }
      }
    }

    /* Advance brick-level DDA. */
    if (brick_tMax.x < brick_tMax.y) {
      if (brick_tMax.x < brick_tMax.z) {
        t_current = brick_tMax.x;
        brick_cell.x += brick_step.x;
        brick_tMax.x += brick_tDelta.x;
      }
      else {
        t_current = brick_tMax.z;
        brick_cell.z += brick_step.z;
        brick_tMax.z += brick_tDelta.z;
      }
    }
    else {
      if (brick_tMax.y < brick_tMax.z) {
        t_current = brick_tMax.y;
        brick_cell.y += brick_step.y;
        brick_tMax.y += brick_tDelta.y;
      }
      else {
        t_current = brick_tMax.z;
        brick_cell.z += brick_step.z;
        brick_tMax.z += brick_tDelta.z;
      }
    }

    if (t_current > t_exit) {
      break;
    }
  }

  return false;
}

/* Shadow test across all SDF objects. */
ccl_device bool sdf_intersect_all_shadow(KernelGlobals kg,
                                          ccl_private const Ray *ray,
                                          const float t_max,
                                          const uint /*visibility*/)
{
  const int num_sdfs = kernel_data.num_sdfs;
  for (int i = 0; i < num_sdfs; i++) {
    if (sdf_intersect_shadow(kg, ray, t_max, i)) {
      return true;
    }
  }
  return false;
}

/* -------------------------------------------------------------------- */
/* Per-instance CPU/CUDA intersection (instanced mode).
 *
 * Transforms the world-space ray into the instance's normalized local space
 * and does two-level DDA through the shape's brick grid (non-cubic).
 * The transform is uniform-scale + rotation + translation, so t is preserved
 * (t_world == t_local). Uses sdf_fetch_shape_indirection and
 * sdf_fetch_corners_shape for per-shape atlas access. */

ccl_device bool sdf_intersect_instanced(KernelGlobals kg,
                                         ccl_private const Ray *ray,
                                         ccl_private Intersection *isect,
                                         const int inst_id)
{
  const KernelSDFInstance kinst = kernel_data_fetch(sdf_shape_instances, inst_id);
  const KernelSDFShape kshape = kernel_data_fetch(sdf_shape_objects, kinst.shape_id);

  if (kshape.active_bricks <= 0) {
    return false;
  }

  /* Transform ray to instance-local space. */
  const float3 local_P = transform_point(&kinst.world_to_local, ray->P);
  const float3 local_D = transform_direction(&kinst.world_to_local, ray->D);

  const int3 grid_res = make_int3(kshape.grid_res_x, kshape.grid_res_y, kshape.grid_res_z);
  const float voxel_size = kshape.voxel_size;
  const float3 origin = make_float3(kshape.origin.x, kshape.origin.y, kshape.origin.z);
  const int bpa = kshape.bricks_per_axis;
  const int atlas_off = kshape.atlas_offset;
  const int indir_off = kshape.indirection_offset;
  const int atlas_dim = bpa * SDF_BRICK_STORAGE;

  /* Clip ray to local grid AABB. */
  const float3 grid_min = origin;
  const float3 grid_max = origin + make_float3(float(grid_res.x * SDF_BRICK_SIZE),
                                                float(grid_res.y * SDF_BRICK_SIZE),
                                                float(grid_res.z * SDF_BRICK_SIZE)) *
                                       voxel_size;

  const float3 inv_dir = safe_divide(one_float3(), local_D);
  const float3 t0 = (grid_min - local_P) * inv_dir;
  const float3 t1 = (grid_max - local_P) * inv_dir;
  const float3 t_lo = min(t0, t1);
  const float3 t_hi = max(t0, t1);
  float t_enter = max(max(t_lo.x, t_lo.y), t_lo.z);
  const float t_exit_grid = min(min(t_hi.x, t_hi.y), t_hi.z);

  if (t_enter > t_exit_grid || t_exit_grid < 0.0f) {
    return false;
  }
  t_enter = max(t_enter, voxel_size);

  const float t_exit = min(t_exit_grid, isect->t);
  if (t_enter >= t_exit) {
    return false;
  }

  /* Brick-level DDA setup (non-cubic grid). */
  const float brick_world = float(SDF_BRICK_SIZE) * voxel_size;
  const float inv_brick = 1.0f / brick_world;
  const float inv_voxel = 1.0f / voxel_size;

  const float3 B = (local_P - origin) * inv_brick;
  const float3 BD = local_D * inv_brick;

  float3 B_enter = B + t_enter * BD;
  int3 brick_cell = make_int3((int)floorf(B_enter.x),
                              (int)floorf(B_enter.y),
                              (int)floorf(B_enter.z));
  brick_cell.x = clamp(brick_cell.x, 0, grid_res.x - 1);
  brick_cell.y = clamp(brick_cell.y, 0, grid_res.y - 1);
  brick_cell.z = clamp(brick_cell.z, 0, grid_res.z - 1);

  const int3 brick_step = make_int3(BD.x > 0.0f ? 1 : (BD.x < 0.0f ? -1 : 0),
                                    BD.y > 0.0f ? 1 : (BD.y < 0.0f ? -1 : 0),
                                    BD.z > 0.0f ? 1 : (BD.z < 0.0f ? -1 : 0));

  const float3 brick_tDelta = make_float3(BD.x != 0.0f ? fabsf(1.0f / BD.x) : 1e30f,
                                          BD.y != 0.0f ? fabsf(1.0f / BD.y) : 1e30f,
                                          BD.z != 0.0f ? fabsf(1.0f / BD.z) : 1e30f);

  float3 brick_boundary = make_float3(BD.x > 0.0f ? float(brick_cell.x + 1) : float(brick_cell.x),
                                      BD.y > 0.0f ? float(brick_cell.y + 1) : float(brick_cell.y),
                                      BD.z > 0.0f ? float(brick_cell.z + 1) : float(brick_cell.z));

  float3 brick_tMax = make_float3(BD.x != 0.0f ? (brick_boundary.x - B.x) / BD.x : 1e30f,
                                  BD.y != 0.0f ? (brick_boundary.y - B.y) / BD.y : 1e30f,
                                  BD.z != 0.0f ? (brick_boundary.z - B.z) / BD.z : 1e30f);

  float hit_t = -1.0f;
  int hit_brick_slot = -1;
  int3 hit_brick = make_int3(0, 0, 0);
  float t_current = t_enter;

  for (int bstep = 0; bstep < SDF_MAX_BRICK_STEPS; bstep++) {
    if (brick_cell.x < 0 || brick_cell.y < 0 || brick_cell.z < 0 ||
        brick_cell.x >= grid_res.x || brick_cell.y >= grid_res.y || brick_cell.z >= grid_res.z)
    {
      break;
    }

    float t_brick_exit = min(min(brick_tMax.x, brick_tMax.y), brick_tMax.z);
    t_brick_exit = min(t_brick_exit, t_exit);

    int slot = sdf_fetch_shape_indirection(
        kg, indir_off, brick_cell.x, brick_cell.y, brick_cell.z, grid_res.x, grid_res.y);

    if (slot == -2) {
      hit_t = t_current;
      hit_brick = brick_cell;
      hit_brick_slot = -2;
      break;
    }

    if (slot >= 0) {
      /* Active brick: voxel-level DDA. */
      const int3 slot_org = sdf_slot_origin(slot, bpa);
      float3 brick_origin = origin + make_float3(float(brick_cell.x * SDF_BRICK_SIZE),
                                                  float(brick_cell.y * SDF_BRICK_SIZE),
                                                  float(brick_cell.z * SDF_BRICK_SIZE)) *
                                         voxel_size;
      float3 V = (local_P - brick_origin) * inv_voxel;
      float3 VD = local_D * inv_voxel;

      float3 V_enter = V + t_current * VD;
      int3 vcell = make_int3(
          (int)floorf(V_enter.x), (int)floorf(V_enter.y), (int)floorf(V_enter.z));
      vcell.x = clamp(vcell.x, 0, SDF_BRICK_SIZE - 1);
      vcell.y = clamp(vcell.y, 0, SDF_BRICK_SIZE - 1);
      vcell.z = clamp(vcell.z, 0, SDF_BRICK_SIZE - 1);

      int3 vstep = make_int3(VD.x > 0.0f ? 1 : (VD.x < 0.0f ? -1 : 0),
                             VD.y > 0.0f ? 1 : (VD.y < 0.0f ? -1 : 0),
                             VD.z > 0.0f ? 1 : (VD.z < 0.0f ? -1 : 0));

      float3 vtDelta = make_float3(VD.x != 0.0f ? fabsf(1.0f / VD.x) : 1e30f,
                                   VD.y != 0.0f ? fabsf(1.0f / VD.y) : 1e30f,
                                   VD.z != 0.0f ? fabsf(1.0f / VD.z) : 1e30f);

      float3 vbound = make_float3(VD.x > 0.0f ? float(vcell.x + 1) : float(vcell.x),
                                  VD.y > 0.0f ? float(vcell.y + 1) : float(vcell.y),
                                  VD.z > 0.0f ? float(vcell.z + 1) : float(vcell.z));

      float3 vtMax = make_float3(VD.x != 0.0f ? (vbound.x - V.x) / VD.x : 1e30f,
                                 VD.y != 0.0f ? (vbound.y - V.y) / VD.y : 1e30f,
                                 VD.z != 0.0f ? (vbound.z - V.z) / VD.z : 1e30f);

      float vt_current = t_current;
      bool voxel_hit = false;

      for (int vs = 0; vs < SDF_MAX_VOXEL_STEPS; vs++) {
        if (vcell.x < 0 || vcell.y < 0 || vcell.z < 0 || vcell.x > SDF_BRICK_SIZE - 1 ||
            vcell.y > SDF_BRICK_SIZE - 1 || vcell.z > SDF_BRICK_SIZE - 1)
        {
          break;
        }

        float vt_cell_exit = min(min(vtMax.x, vtMax.y), vtMax.z);
        vt_cell_exit = min(vt_cell_exit, t_brick_exit);

        float s[8];
        sdf_fetch_corners_shape(kg, atlas_off, vcell, slot_org, atlas_dim, voxel_size, s);

        float smin = min(min(min(s[0], s[1]), min(s[2], s[3])),
                         min(min(s[4], s[5]), min(s[6], s[7])));
        float smax = max(max(max(s[0], s[1]), max(s[2], s[3])),
                         max(max(s[4], s[5]), max(s[6], s[7])));

        if (smin <= 0.0f) {
          if (smax < 0.0f) {
            hit_t = vt_current;
            hit_brick = brick_cell;
            hit_brick_slot = slot;
            voxel_hit = true;
            break;
          }

          float T_max = vt_cell_exit - vt_current;
          if (T_max > 1e-8f) {
            float k[8];
            sdf_trilinear_coeffs(s, k);

            float3 o_local = V + vt_current * VD - make_float3(float(vcell.x),
                                                                float(vcell.y),
                                                                float(vcell.z));
            o_local = clamp(o_local, zero_float3(), one_float3());
            float3 d_scaled = VD * T_max;

            float c[4];
            sdf_cubic_coeffs(k, o_local, d_scaled, c);

            if (c[0] < -1e-5f) {
              hit_t = vt_current;
              hit_brick = brick_cell;
              hit_brick_slot = slot;
              voxel_hit = true;
              break;
            }

            float u_hit = sdf_solve_cubic(c, 1.0f);
            if (u_hit >= 0.0f) {
              hit_t = vt_current + u_hit * T_max;
              hit_brick = brick_cell;
              hit_brick_slot = slot;
              voxel_hit = true;
              break;
            }
          }
        }

        /* Advance voxel DDA. */
        if (vtMax.x < vtMax.y) {
          if (vtMax.x < vtMax.z) {
            vt_current = vtMax.x;
            vcell.x += vstep.x;
            vtMax.x += vtDelta.x;
          }
          else {
            vt_current = vtMax.z;
            vcell.z += vstep.z;
            vtMax.z += vtDelta.z;
          }
        }
        else {
          if (vtMax.y < vtMax.z) {
            vt_current = vtMax.y;
            vcell.y += vstep.y;
            vtMax.y += vtDelta.y;
          }
          else {
            vt_current = vtMax.z;
            vcell.z += vstep.z;
            vtMax.z += vtDelta.z;
          }
        }

        if (vt_current >= t_brick_exit) {
          break;
        }
      }

      if (voxel_hit) {
        break;
      }
    }

    /* Advance brick-level DDA. */
    if (brick_tMax.x < brick_tMax.y) {
      if (brick_tMax.x < brick_tMax.z) {
        t_current = brick_tMax.x;
        brick_cell.x += brick_step.x;
        brick_tMax.x += brick_tDelta.x;
      }
      else {
        t_current = brick_tMax.z;
        brick_cell.z += brick_step.z;
        brick_tMax.z += brick_tDelta.z;
      }
    }
    else {
      if (brick_tMax.y < brick_tMax.z) {
        t_current = brick_tMax.y;
        brick_cell.y += brick_step.y;
        brick_tMax.y += brick_tDelta.y;
      }
      else {
        t_current = brick_tMax.z;
        brick_cell.z += brick_step.z;
        brick_tMax.z += brick_tDelta.z;
      }
    }

    if (t_current > t_exit) {
      break;
    }
  }

  if (hit_t < 0.0f || hit_t >= isect->t) {
    return false;
  }

  /* Record intersection.
   * prim = instance ID (matches OptiX convention for sdf_shader_setup).
   * u/v encode brick_linear and brick_slot. */
  isect->t = hit_t;
  isect->prim = inst_id;
  isect->object = kinst.object_id;
  isect->type = PRIMITIVE_SDF;
  const int brick_linear = hit_brick.x + hit_brick.y * grid_res.x +
                           hit_brick.z * grid_res.x * grid_res.y;
  isect->u = __int_as_float(brick_linear);
  isect->v = __int_as_float(hit_brick_slot);

  return true;
}

/* March all instanced SDF instances, return closest hit (CPU/CUDA path). */
ccl_device bool sdf_intersect_all_instanced(KernelGlobals kg,
                                             ccl_private const Ray *ray,
                                             ccl_private Intersection *isect,
                                             const uint /*visibility*/)
{
  bool hit = false;
  const int num_instances = kernel_data.num_sdf_instances;

  for (int ii = 0; ii < num_instances; ii++) {
    if (sdf_intersect_instanced(kg, ray, isect, ii)) {
      hit = true;
    }
  }

  return hit;
}

/* Shadow variant: instanced mode. */
ccl_device bool sdf_intersect_instanced_shadow(KernelGlobals kg,
                                                ccl_private const Ray *ray,
                                                const float t_max,
                                                const int inst_id)
{
  const KernelSDFInstance kinst = kernel_data_fetch(sdf_shape_instances, inst_id);
  const KernelSDFShape kshape = kernel_data_fetch(sdf_shape_objects, kinst.shape_id);

  if (kshape.active_bricks <= 0) {
    return false;
  }

  const float3 local_P = transform_point(&kinst.world_to_local, ray->P);
  const float3 local_D = transform_direction(&kinst.world_to_local, ray->D);

  const int3 grid_res = make_int3(kshape.grid_res_x, kshape.grid_res_y, kshape.grid_res_z);
  const float voxel_size = kshape.voxel_size;
  const float3 origin = make_float3(kshape.origin.x, kshape.origin.y, kshape.origin.z);
  const int bpa = kshape.bricks_per_axis;
  const int atlas_off = kshape.atlas_offset;
  const int indir_off = kshape.indirection_offset;
  const int atlas_dim = bpa * SDF_BRICK_STORAGE;

  const float3 grid_min = origin;
  const float3 grid_max = origin + make_float3(float(grid_res.x * SDF_BRICK_SIZE),
                                                float(grid_res.y * SDF_BRICK_SIZE),
                                                float(grid_res.z * SDF_BRICK_SIZE)) *
                                       voxel_size;

  const float3 inv_dir = safe_divide(one_float3(), local_D);
  const float3 t0 = (grid_min - local_P) * inv_dir;
  const float3 t1 = (grid_max - local_P) * inv_dir;
  const float3 t_lo = min(t0, t1);
  const float3 t_hi = max(t0, t1);
  float t_enter = max(max(t_lo.x, t_lo.y), t_lo.z);
  const float t_exit_grid = min(min(t_hi.x, t_hi.y), t_hi.z);

  if (t_enter > t_exit_grid || t_exit_grid < 0.0f) {
    return false;
  }
  t_enter = max(t_enter, voxel_size);

  const float t_exit = min(t_exit_grid, t_max);
  if (t_enter >= t_exit) {
    return false;
  }

  const float brick_world = float(SDF_BRICK_SIZE) * voxel_size;
  const float inv_brick = 1.0f / brick_world;
  const float inv_voxel = 1.0f / voxel_size;

  const float3 B = (local_P - origin) * inv_brick;
  const float3 BD = local_D * inv_brick;

  float3 B_enter = B + t_enter * BD;
  int3 brick_cell = make_int3((int)floorf(B_enter.x),
                              (int)floorf(B_enter.y),
                              (int)floorf(B_enter.z));
  brick_cell.x = clamp(brick_cell.x, 0, grid_res.x - 1);
  brick_cell.y = clamp(brick_cell.y, 0, grid_res.y - 1);
  brick_cell.z = clamp(brick_cell.z, 0, grid_res.z - 1);

  const int3 brick_step = make_int3(BD.x > 0.0f ? 1 : (BD.x < 0.0f ? -1 : 0),
                                    BD.y > 0.0f ? 1 : (BD.y < 0.0f ? -1 : 0),
                                    BD.z > 0.0f ? 1 : (BD.z < 0.0f ? -1 : 0));
  const float3 brick_tDelta = make_float3(BD.x != 0.0f ? fabsf(1.0f / BD.x) : 1e30f,
                                          BD.y != 0.0f ? fabsf(1.0f / BD.y) : 1e30f,
                                          BD.z != 0.0f ? fabsf(1.0f / BD.z) : 1e30f);
  float3 brick_boundary = make_float3(BD.x > 0.0f ? float(brick_cell.x + 1) : float(brick_cell.x),
                                      BD.y > 0.0f ? float(brick_cell.y + 1) : float(brick_cell.y),
                                      BD.z > 0.0f ? float(brick_cell.z + 1) : float(brick_cell.z));
  float3 brick_tMax = make_float3(BD.x != 0.0f ? (brick_boundary.x - B.x) / BD.x : 1e30f,
                                  BD.y != 0.0f ? (brick_boundary.y - B.y) / BD.y : 1e30f,
                                  BD.z != 0.0f ? (brick_boundary.z - B.z) / BD.z : 1e30f);

  float t_current = t_enter;

  for (int bstep = 0; bstep < SDF_MAX_BRICK_STEPS; bstep++) {
    if (brick_cell.x < 0 || brick_cell.y < 0 || brick_cell.z < 0 ||
        brick_cell.x >= grid_res.x || brick_cell.y >= grid_res.y || brick_cell.z >= grid_res.z)
    {
      break;
    }

    float t_brick_exit = min(min(brick_tMax.x, brick_tMax.y), brick_tMax.z);
    t_brick_exit = min(t_brick_exit, t_exit);

    int slot = sdf_fetch_shape_indirection(
        kg, indir_off, brick_cell.x, brick_cell.y, brick_cell.z, grid_res.x, grid_res.y);

    if (slot == -2) {
      return true;
    }

    if (slot >= 0) {
      const int3 slot_org = sdf_slot_origin(slot, bpa);
      float3 brick_origin = origin + make_float3(float(brick_cell.x * SDF_BRICK_SIZE),
                                                  float(brick_cell.y * SDF_BRICK_SIZE),
                                                  float(brick_cell.z * SDF_BRICK_SIZE)) *
                                         voxel_size;
      float3 V = (local_P - brick_origin) * inv_voxel;
      float3 VD = local_D * inv_voxel;

      float3 V_enter = V + t_current * VD;
      int3 vcell = make_int3(
          (int)floorf(V_enter.x), (int)floorf(V_enter.y), (int)floorf(V_enter.z));
      vcell.x = clamp(vcell.x, 0, SDF_BRICK_SIZE - 1);
      vcell.y = clamp(vcell.y, 0, SDF_BRICK_SIZE - 1);
      vcell.z = clamp(vcell.z, 0, SDF_BRICK_SIZE - 1);

      int3 vstep = make_int3(VD.x > 0.0f ? 1 : (VD.x < 0.0f ? -1 : 0),
                             VD.y > 0.0f ? 1 : (VD.y < 0.0f ? -1 : 0),
                             VD.z > 0.0f ? 1 : (VD.z < 0.0f ? -1 : 0));
      float3 vtDelta = make_float3(VD.x != 0.0f ? fabsf(1.0f / VD.x) : 1e30f,
                                   VD.y != 0.0f ? fabsf(1.0f / VD.y) : 1e30f,
                                   VD.z != 0.0f ? fabsf(1.0f / VD.z) : 1e30f);
      float3 vbound = make_float3(VD.x > 0.0f ? float(vcell.x + 1) : float(vcell.x),
                                  VD.y > 0.0f ? float(vcell.y + 1) : float(vcell.y),
                                  VD.z > 0.0f ? float(vcell.z + 1) : float(vcell.z));
      float3 vtMax = make_float3(VD.x != 0.0f ? (vbound.x - V.x) / VD.x : 1e30f,
                                 VD.y != 0.0f ? (vbound.y - V.y) / VD.y : 1e30f,
                                 VD.z != 0.0f ? (vbound.z - V.z) / VD.z : 1e30f);

      float vt_current = t_current;

      for (int vs = 0; vs < SDF_MAX_VOXEL_STEPS; vs++) {
        if (vcell.x < 0 || vcell.y < 0 || vcell.z < 0 || vcell.x > SDF_BRICK_SIZE - 1 ||
            vcell.y > SDF_BRICK_SIZE - 1 || vcell.z > SDF_BRICK_SIZE - 1)
        {
          break;
        }

        float vt_cell_exit = min(min(vtMax.x, vtMax.y), vtMax.z);
        vt_cell_exit = min(vt_cell_exit, t_brick_exit);

        float s[8];
        sdf_fetch_corners_shape(kg, atlas_off, vcell, slot_org, atlas_dim, voxel_size, s);

        float smin = min(min(min(s[0], s[1]), min(s[2], s[3])),
                         min(min(s[4], s[5]), min(s[6], s[7])));

        if (smin <= 0.0f) {
          float smax = max(max(max(s[0], s[1]), max(s[2], s[3])),
                           max(max(s[4], s[5]), max(s[6], s[7])));
          if (smax < 0.0f) {
            return true;
          }

          float T_max = vt_cell_exit - vt_current;
          if (T_max > 1e-8f) {
            float k[8];
            sdf_trilinear_coeffs(s, k);

            float3 o_local = V + vt_current * VD - make_float3(float(vcell.x),
                                                                float(vcell.y),
                                                                float(vcell.z));
            o_local = clamp(o_local, zero_float3(), one_float3());
            float3 d_scaled = VD * T_max;

            float c[4];
            sdf_cubic_coeffs(k, o_local, d_scaled, c);

            if (c[0] < -1e-5f) {
              return true;
            }
            if (sdf_has_cubic_root(c, 1.0f)) {
              return true;
            }
          }
        }

        if (vtMax.x < vtMax.y) {
          if (vtMax.x < vtMax.z) {
            vt_current = vtMax.x;
            vcell.x += vstep.x;
            vtMax.x += vtDelta.x;
          }
          else {
            vt_current = vtMax.z;
            vcell.z += vstep.z;
            vtMax.z += vtDelta.z;
          }
        }
        else {
          if (vtMax.y < vtMax.z) {
            vt_current = vtMax.y;
            vcell.y += vstep.y;
            vtMax.y += vtDelta.y;
          }
          else {
            vt_current = vtMax.z;
            vcell.z += vstep.z;
            vtMax.z += vtDelta.z;
          }
        }

        if (vt_current >= t_brick_exit) {
          break;
        }
      }
    }

    if (brick_tMax.x < brick_tMax.y) {
      if (brick_tMax.x < brick_tMax.z) {
        t_current = brick_tMax.x;
        brick_cell.x += brick_step.x;
        brick_tMax.x += brick_tDelta.x;
      }
      else {
        t_current = brick_tMax.z;
        brick_cell.z += brick_step.z;
        brick_tMax.z += brick_tDelta.z;
      }
    }
    else {
      if (brick_tMax.y < brick_tMax.z) {
        t_current = brick_tMax.y;
        brick_cell.y += brick_step.y;
        brick_tMax.y += brick_tDelta.y;
      }
      else {
        t_current = brick_tMax.z;
        brick_cell.z += brick_step.z;
        brick_tMax.z += brick_tDelta.z;
      }
    }

    if (t_current > t_exit) {
      break;
    }
  }

  return false;
}

ccl_device bool sdf_intersect_all_instanced_shadow(KernelGlobals kg,
                                                    ccl_private const Ray *ray,
                                                    const float t_max,
                                                    const uint /*visibility*/)
{
  const int num_instances = kernel_data.num_sdf_instances;
  for (int ii = 0; ii < num_instances; ii++) {
    if (sdf_intersect_instanced_shadow(kg, ray, t_max, ii)) {
      return true;
    }
  }
  return false;
}

/* Set up shader data for an SDF hit. */
ccl_device void sdf_shader_setup(KernelGlobals kg,
                                  ccl_private ShaderData *sd,
                                  ccl_private const Ray *ray,
                                  ccl_private const Intersection *isect)
{
  /* Hit position. */
  sd->P = ray->P + ray->D * isect->t;
  sd->u = 0.0f;
  sd->v = 0.0f;

  /* Default: no material blend. */
  sd->sdf_blend_shader = -1;
  sd->sdf_blend_factor = 0.0f;

  if (kernel_data.num_sdf_shapes > 0) {
    /* ---- Per-shape instanced mode ---- */
    const int inst_id = isect->prim; /* Stored by intersection program. */
    const KernelSDFInstance kinst = kernel_data_fetch(sdf_shape_instances, inst_id);
    const KernelSDFShape kshape = kernel_data_fetch(sdf_shape_objects, kinst.shape_id);

    sd->shader = kinst.shader_id;

    const float3 origin = make_float3(kshape.origin.x, kshape.origin.y, kshape.origin.z);
    const float voxel_size = kshape.voxel_size;
    const int bpa = kshape.bricks_per_axis;
    const int atlas_off = kshape.atlas_offset;
    const int atlas_dim = bpa * SDF_BRICK_STORAGE;
    const int3 grid_res = make_int3(kshape.grid_res_x, kshape.grid_res_y, kshape.grid_res_z);

    const int brick_linear = __float_as_int(isect->u);
    const int brick_slot = __float_as_int(isect->v);
    const int3 brick_cell = make_int3(brick_linear % grid_res.x,
                                       (brick_linear / grid_res.x) % grid_res.y,
                                       brick_linear / (grid_res.x * grid_res.y));

    /* Compute hit position in local space for normal computation.
     * sd->P is always world-space, so transform to instance-local space. */
    float3 local_hit = transform_point(&kinst.world_to_local, sd->P);
    float3 brick_origin_l = origin + make_float3(float(brick_cell.x * SDF_BRICK_SIZE),
                                                  float(brick_cell.y * SDF_BRICK_SIZE),
                                                  float(brick_cell.z * SDF_BRICK_SIZE)) *
                                         voxel_size;
    float3 grid_pos_in_brick = (local_hit - brick_origin_l) / voxel_size;
    /* Gentle clamp — keep within the range reachable by the 2-voxel border.
     * The dual voxel stencil accesses dc-1..dc+1 where dc = floor(pos+0.5),
     * so the minimum atlas offset is -1 and maximum is BRICK_SIZE.
     * With border=2 the atlas stores [-2, BRICK_SIZE+1], giving ample margin.
     * Using [-0.49, BRICK_SIZE+0.49] ensures dc stays in [0, BRICK_SIZE]
     * while keeping boundary continuity (no 0.01-epsilon gap). */
    grid_pos_in_brick = clamp(grid_pos_in_brick,
                              make_float3(-0.49f, -0.49f, -0.49f),
                              make_float3(float(SDF_BRICK_SIZE) + 0.49f,
                                          float(SDF_BRICK_SIZE) + 0.49f,
                                          float(SDF_BRICK_SIZE) + 0.49f));

    if (brick_slot >= 0) {
      const int3 slot_org = sdf_slot_origin(brick_slot, bpa);
      /* Dual voxel normal in local space using per-shape atlas. */
      const float3 shifted = grid_pos_in_brick + make_float3(0.5f, 0.5f, 0.5f);
      const int3 dc = make_int3((int)floorf(shifted.x),
                                 (int)floorf(shifted.y),
                                 (int)floorf(shifted.z));

      /* Fetch 3x3x3 neighborhood from shape atlas. */
      float vals[27];
      const int3 base = sdf_grid_to_compact(
          make_int3(dc.x - 1, dc.y - 1, dc.z - 1), slot_org);
      for (int k = 0; k < 3; k++) {
        for (int j = 0; j < 3; j++) {
          for (int i = 0; i < 3; i++) {
            const int idx = atlas_off + (base.z + k) * atlas_dim * atlas_dim +
                            (base.y + j) * atlas_dim + (base.x + i);
            vals[k * 9 + j * 3 + i] = sdf_decode_dist16(
                kernel_data_fetch(sdf_shape_atlas, idx), voxel_size);
          }
        }
      }

      float3 grad_local = sdf_dual_voxel_normal(vals, grid_pos_in_brick, dc);
      /* Transform local-space normal to world space. */
      float3 grad_world = transform_direction(&kinst.local_to_world, grad_local);
      float l = len(grad_world);
      sd->Ng = (l > 1e-8f) ? grad_world / l : make_float3(0.0f, 0.0f, 1.0f);
    }
    else {
      sd->Ng = -ray->D;
    }
    sd->N = sd->Ng;
  }
  else {
    /* ---- World-space mode (original) ---- */
    const int sdf_index = isect->prim;
    const KernelSDF ksdf = kernel_data_fetch(sdf_objects, sdf_index);
    const float3 origin = make_float3(ksdf.origin.x, ksdf.origin.y, ksdf.origin.z);
    const float voxel_size = ksdf.voxel_size;
    const int bpa = ksdf.bricks_per_axis;
    const int atlas_off = ksdf.atlas_offset;
    const int matid_off = ksdf.matid_offset;
    const int atlas_dim = bpa * SDF_BRICK_STORAGE;
    const int grid_res = ksdf.grid_res;

    sd->shader = kernel_data_fetch(sdf_shader_map, ksdf.shader_offset);

    const int brick_linear = __float_as_int(isect->u);
    const int brick_slot = __float_as_int(isect->v);
    const int3 brick_cell = make_int3(brick_linear % grid_res,
                                       (brick_linear / grid_res) % grid_res,
                                       brick_linear / (grid_res * grid_res));

    float3 brick_origin_w = origin + make_float3(float(brick_cell.x * SDF_BRICK_SIZE),
                                                  float(brick_cell.y * SDF_BRICK_SIZE),
                                                  float(brick_cell.z * SDF_BRICK_SIZE)) *
                                         voxel_size;
    float3 grid_pos_in_brick = (sd->P - brick_origin_w) / voxel_size;
    /* Same gentle clamp as instanced path — see comment above. */
    grid_pos_in_brick = clamp(grid_pos_in_brick,
                              make_float3(-0.49f, -0.49f, -0.49f),
                              make_float3(float(SDF_BRICK_SIZE) + 0.49f,
                                          float(SDF_BRICK_SIZE) + 0.49f,
                                          float(SDF_BRICK_SIZE) + 0.49f));

    const int3 slot_org = (brick_slot >= 0) ? sdf_slot_origin(brick_slot, bpa) :
                                               make_int3(0, 0, 0);

    if (brick_slot >= 0) {
      sd->Ng = sdf_compute_normal(kg, atlas_off, grid_pos_in_brick, slot_org, atlas_dim);
    }
    else {
      sd->Ng = -ray->D;
    }
    sd->N = sd->Ng;

    /* Read material ID. */
    if (brick_slot >= 0) {
      int3 atlas_coord = sdf_grid_to_compact(
          make_int3((int)floorf(grid_pos_in_brick.x),
                    (int)floorf(grid_pos_in_brick.y),
                    (int)floorf(grid_pos_in_brick.z)),
          slot_org);
      atlas_coord.x = clamp(atlas_coord.x, 0, atlas_dim - 1);
      atlas_coord.y = clamp(atlas_coord.y, 0, atlas_dim - 1);
      atlas_coord.z = clamp(atlas_coord.z, 0, atlas_dim - 1);

      int obj_id = sdf_fetch_matid(
          kg, matid_off, atlas_coord.x, atlas_coord.y, atlas_coord.z, atlas_dim);
      if (obj_id >= 0 && obj_id < ksdf.num_objects) {
        sd->shader = kernel_data_fetch(sdf_shader_map, ksdf.shader_offset + obj_id);
      }

      /* Read blend data for material mixing (only when blend arrays exist). */
      if (ksdf.blend_offset >= 0) {
        const int flat_idx = ksdf.blend_offset + atlas_coord.z * atlas_dim * atlas_dim +
                             atlas_coord.y * atlas_dim + atlas_coord.x;
        const int blend_obj = kernel_data_fetch(sdf_blend_id, flat_idx);
        const float blend_fac = kernel_data_fetch(sdf_blend_factor, flat_idx);
        if (blend_obj >= 0 && blend_obj < ksdf.num_objects && blend_fac > 0.001f) {
          sd->sdf_blend_shader = kernel_data_fetch(sdf_shader_map,
                                                    ksdf.shader_offset + blend_obj);
          sd->sdf_blend_factor = blend_fac;
        }
      }
    }
  }

  /* Tangent frame from normal (shared between both modes). */
  float3 up = (fabsf(sd->N.z) < 0.999f) ? make_float3(0.0f, 0.0f, 1.0f) :
                                            make_float3(1.0f, 0.0f, 0.0f);
  sd->dPdu = normalize(cross(up, sd->N));
  sd->dPdv = cross(sd->N, sd->dPdu);
}

CCL_NAMESPACE_END
