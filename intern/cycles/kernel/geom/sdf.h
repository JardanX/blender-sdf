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

/* Convert grid-space brick + local voxel to compact atlas coordinate. */
ccl_device_inline int3 sdf_grid_to_compact(const int3 local_voxel,
                                            const int brick_slot,
                                            const int bpa)
{
  const int sx = brick_slot % bpa;
  const int sy = (brick_slot / bpa) % bpa;
  const int sz = brick_slot / (bpa * bpa);
  return make_int3(sx * SDF_BRICK_STORAGE + local_voxel.x + SDF_BRICK_BORDER,
                   sy * SDF_BRICK_STORAGE + local_voxel.y + SDF_BRICK_BORDER,
                   sz * SDF_BRICK_STORAGE + local_voxel.z + SDF_BRICK_BORDER);
}

/* Fetch 8 corner SDF values for a voxel cell. */
ccl_device void sdf_fetch_corners(KernelGlobals kg,
                                   const int atlas_offset,
                                   const int3 local_cell,
                                   const int brick_slot,
                                   const int bpa,
                                   const int atlas_dim,
                                   float s[8])
{
  const int3 base = sdf_grid_to_compact(local_cell, brick_slot, bpa);
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

/* -------------------------------------------------------------------- */
/* Normal computation using dual voxel method. */

ccl_device float3 sdf_compute_normal(KernelGlobals kg,
                                      const int atlas_offset,
                                      const float3 grid_pos_in_brick,
                                      const int brick_slot,
                                      const int bpa,
                                      const int atlas_dim)
{
  /* Dual voxel base: shift by -0.5 then floor.
   * Allow range [-1, BRICK_SIZE-1] so the 3x3x3 neighborhood uses the
   * 2-voxel border overlap (BRICK_STORAGE=12 provides voxels -2..9). */
  int3 base = make_int3((int)floorf(grid_pos_in_brick.x - 0.5f),
                        (int)floorf(grid_pos_in_brick.y - 0.5f),
                        (int)floorf(grid_pos_in_brick.z - 0.5f));
  base.x = clamp(base.x, -1, SDF_BRICK_SIZE - 1);
  base.y = clamp(base.y, -1, SDF_BRICK_SIZE - 1);
  base.z = clamp(base.z, -1, SDF_BRICK_SIZE - 1);

  float3 uvw = make_float3(grid_pos_in_brick.x - 0.5f - float(base.x),
                           grid_pos_in_brick.y - 0.5f - float(base.y),
                           grid_pos_in_brick.z - 0.5f - float(base.z));
  uvw = clamp(uvw, zero_float3(), one_float3());

  /* Compact atlas base. */
  const int sx = brick_slot % bpa;
  const int sy = (brick_slot / bpa) % bpa;
  const int sz = brick_slot / (bpa * bpa);
  const int3 atlas_base = make_int3(sx * SDF_BRICK_STORAGE + base.x + SDF_BRICK_BORDER,
                                    sy * SDF_BRICK_STORAGE + base.y + SDF_BRICK_BORDER,
                                    sz * SDF_BRICK_STORAGE + base.z + SDF_BRICK_BORDER);

  /* Fetch 3x3x3 = 27 neighborhood. */
  float v[27];
  for (int dz = 0; dz < 3; dz++) {
    for (int dy = 0; dy < 3; dy++) {
      for (int dx = 0; dx < 3; dx++) {
        v[dz * 9 + dy * 3 + dx] = sdf_fetch_distance(
            kg, atlas_offset, atlas_base.x + dx, atlas_base.y + dy, atlas_base.z + dz, atlas_dim);
      }
    }
  }

  /* For each of 8 overlapping primal voxels: analytic gradient, normalize. */
  float3 normals[8];
  for (int dz = 0; dz < 2; dz++) {
    for (int dy = 0; dy < 2; dy++) {
      for (int dx = 0; dx < 2; dx++) {
        const int o = dz * 9 + dy * 3 + dx;
        float corners[8];
        corners[0] = v[o];
        corners[1] = v[o + 1];
        corners[2] = v[o + 3];
        corners[3] = v[o + 4];
        corners[4] = v[o + 9];
        corners[5] = v[o + 10];
        corners[6] = v[o + 12];
        corners[7] = v[o + 13];

        float3 local = make_float3(grid_pos_in_brick.x - float(base.x + dx),
                                   grid_pos_in_brick.y - float(base.y + dy),
                                   grid_pos_in_brick.z - float(base.z + dz));
        float3 grad = sdf_trilinear_gradient(corners, local);
        float l = len(grad);
        normals[dz * 4 + dy * 2 + dx] = (l > 1e-8f) ? grad / l :
                                                        make_float3(0.0f, 0.0f, 1.0f);
      }
    }
  }

  /* Trilinear blend of 8 normals. */
  float3 n00 = normals[0] * (1.0f - uvw.x) + normals[1] * uvw.x;
  float3 n10 = normals[2] * (1.0f - uvw.x) + normals[3] * uvw.x;
  float3 n01 = normals[4] * (1.0f - uvw.x) + normals[5] * uvw.x;
  float3 n11 = normals[6] * (1.0f - uvw.x) + normals[7] * uvw.x;
  float3 n0 = n00 * (1.0f - uvw.y) + n10 * uvw.y;
  float3 n1 = n01 * (1.0f - uvw.y) + n11 * uvw.y;
  return normalize(n0 * (1.0f - uvw.z) + n1 * uvw.z);
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
    sdf_fetch_corners(kg, atlas_off, vcell, brick_slot, bpa, atlas_dim, s);

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
        sdf_fetch_corners(kg, atlas_off, vcell, slot, bpa, atlas_dim, s);

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
                                   const uint visibility)
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

/* Set up shader data for an SDF hit. */
ccl_device void sdf_shader_setup(KernelGlobals kg,
                                  ccl_private ShaderData *sd,
                                  ccl_private const Ray *ray,
                                  ccl_private const Intersection *isect)
{
  const int sdf_index = isect->prim;
  const KernelSDF ksdf = kernel_data_fetch(sdf_objects, sdf_index);
  const float3 origin = make_float3(ksdf.origin.x, ksdf.origin.y, ksdf.origin.z);
  const float voxel_size = ksdf.voxel_size;
  const int bpa = ksdf.bricks_per_axis;
  const int atlas_off = ksdf.atlas_offset;
  const int matid_off = ksdf.matid_offset;
  const int atlas_dim = bpa * SDF_BRICK_STORAGE;
  const int grid_res = ksdf.grid_res;

  /* Default shader (overridden below if matid lookup succeeds). */
  sd->shader = kernel_data_fetch(sdf_shader_map, ksdf.shader_offset);

  /* Hit position. */
  sd->P = ray->P + ray->D * isect->t;

  /* Recover brick cell and slot encoded during ray march (stored in u/v).
   * This avoids recomputing from float position which can give wrong
   * brick at cell boundaries due to floating-point precision. */
  const int brick_linear = __float_as_int(isect->u);
  const int brick_slot = __float_as_int(isect->v);
  const int3 brick_cell = make_int3(brick_linear % grid_res,
                                     (brick_linear / grid_res) % grid_res,
                                     brick_linear / (grid_res * grid_res));

  /* Reset u/v to sensible values so downstream shader code doesn't
   * read the bit-cast brick info as garbage float UVs. */
  sd->u = 0.0f;
  sd->v = 0.0f;

  /* Grid position within the brick (0..BRICK_SIZE). */
  float3 brick_origin_w = origin + make_float3(float(brick_cell.x * SDF_BRICK_SIZE),
                                                float(brick_cell.y * SDF_BRICK_SIZE),
                                                float(brick_cell.z * SDF_BRICK_SIZE)) *
                                       voxel_size;
  float3 grid_pos_in_brick = (sd->P - brick_origin_w) / voxel_size;
  grid_pos_in_brick = clamp(grid_pos_in_brick,
                            make_float3(0.01f, 0.01f, 0.01f),
                            make_float3(float(SDF_BRICK_SIZE) - 0.01f,
                                        float(SDF_BRICK_SIZE) - 0.01f,
                                        float(SDF_BRICK_SIZE) - 0.01f));

  /* Normal from dual voxel gradient. */
  if (brick_slot >= 0) {
    sd->Ng = sdf_compute_normal(kg, atlas_off, grid_pos_in_brick, brick_slot, bpa, atlas_dim);
  }
  else {
    /* Fully-inside brick (slot=-2) or fallback: use ray direction as normal. */
    sd->Ng = -ray->D;
  }
  sd->N = sd->Ng;

  /* Tangent frame from normal. */
  float3 up = (fabsf(sd->N.z) < 0.999f) ? make_float3(0.0f, 0.0f, 1.0f) :
                                            make_float3(1.0f, 0.0f, 0.0f);
  sd->dPdu = normalize(cross(up, sd->N));
  sd->dPdv = cross(sd->N, sd->dPdu);

  /* Read material ID from matid array to determine shader. */
  if (brick_slot >= 0) {
    int3 atlas_coord = sdf_grid_to_compact(
        make_int3((int)floorf(grid_pos_in_brick.x),
                  (int)floorf(grid_pos_in_brick.y),
                  (int)floorf(grid_pos_in_brick.z)),
        brick_slot,
        bpa);
    atlas_coord.x = clamp(atlas_coord.x, 0, atlas_dim - 1);
    atlas_coord.y = clamp(atlas_coord.y, 0, atlas_dim - 1);
    atlas_coord.z = clamp(atlas_coord.z, 0, atlas_dim - 1);

    int obj_id = sdf_fetch_matid(
        kg, matid_off, atlas_coord.x, atlas_coord.y, atlas_coord.z, atlas_dim);

    /* Look up shader for this object. */
    if (obj_id >= 0 && obj_id < ksdf.num_objects) {
      sd->shader = kernel_data_fetch(sdf_shader_map, ksdf.shader_offset + obj_id);
    }
  }
}

CCL_NAMESPACE_END
