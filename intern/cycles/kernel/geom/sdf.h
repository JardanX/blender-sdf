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
#define SDF_BRICK_STORAGE 10
#define SDF_MAX_BRICK_STEPS 128
#define SDF_MAX_VOXEL_STEPS 24

/* -------------------------------------------------------------------- */
/* Texture access helpers. */

/* Fetch integer value from indirection 3D texture at brick coordinate. */
ccl_device_inline int sdf_fetch_indirection(KernelGlobals kg,
                                             const int slot,
                                             const int bx,
                                             const int by,
                                             const int bz,
                                             const int grid_res)
{
  /* Indirection is a 3D texture of grid_res^3. We linearize and use
   * kernel_data_fetch into the texture data. */
  float3 uvw = make_float3((float(bx) + 0.5f) / float(grid_res),
                           (float(by) + 0.5f) / float(grid_res),
                           (float(bz) + 0.5f) / float(grid_res));
  float4 val = kernel_tex_image_interp_3d(kg, slot, uvw, INTERPOLATION_CLOSEST);
  return (int)val.x;
}

/* Fetch float value from atlas 3D texture at given texel. */
ccl_device_inline float4 sdf_fetch_atlas(KernelGlobals kg,
                                          const int slot,
                                          const int x,
                                          const int y,
                                          const int z,
                                          const int atlas_dim)
{
  float3 uvw = make_float3((float(x) + 0.5f) / float(atlas_dim),
                           (float(y) + 0.5f) / float(atlas_dim),
                           (float(z) + 0.5f) / float(atlas_dim));
  return kernel_tex_image_interp_3d(kg, slot, uvw, INTERPOLATION_CLOSEST);
}

/* Fetch the SDF distance at a compact atlas coordinate. */
ccl_device_inline float sdf_fetch_distance(KernelGlobals kg,
                                            const int slot,
                                            const int x,
                                            const int y,
                                            const int z,
                                            const int atlas_dim)
{
  return sdf_fetch_atlas(kg, slot, x, y, z, atlas_dim).x;
}

/* Convert grid-space brick + local voxel to compact atlas coordinate. */
ccl_device_inline int3 sdf_grid_to_compact(const int3 local_voxel,
                                            const int brick_slot,
                                            const int bpa)
{
  const int sx = brick_slot % bpa;
  const int sy = (brick_slot / bpa) % bpa;
  const int sz = brick_slot / (bpa * bpa);
  return make_int3(sx * SDF_BRICK_STORAGE + local_voxel.x + 1,
                   sy * SDF_BRICK_STORAGE + local_voxel.y + 1,
                   sz * SDF_BRICK_STORAGE + local_voxel.z + 1);
}

/* Fetch 8 corner SDF values for a voxel cell. */
ccl_device void sdf_fetch_corners(KernelGlobals kg,
                                   const int atlas_slot,
                                   const int3 local_cell,
                                   const int brick_slot,
                                   const int bpa,
                                   const int atlas_dim,
                                   float s[8])
{
  const int3 base = sdf_grid_to_compact(local_cell, brick_slot, bpa);
  s[0] = sdf_fetch_distance(kg, atlas_slot, base.x, base.y, base.z, atlas_dim);
  s[1] = sdf_fetch_distance(kg, atlas_slot, base.x + 1, base.y, base.z, atlas_dim);
  s[2] = sdf_fetch_distance(kg, atlas_slot, base.x, base.y + 1, base.z, atlas_dim);
  s[3] = sdf_fetch_distance(kg, atlas_slot, base.x + 1, base.y + 1, base.z, atlas_dim);
  s[4] = sdf_fetch_distance(kg, atlas_slot, base.x, base.y, base.z + 1, atlas_dim);
  s[5] = sdf_fetch_distance(kg, atlas_slot, base.x + 1, base.y, base.z + 1, atlas_dim);
  s[6] = sdf_fetch_distance(kg, atlas_slot, base.x, base.y + 1, base.z + 1, atlas_dim);
  s[7] = sdf_fetch_distance(
      kg, atlas_slot, base.x + 1, base.y + 1, base.z + 1, atlas_dim);
}

/* -------------------------------------------------------------------- */
/* Normal computation using dual voxel method. */

ccl_device float3 sdf_compute_normal(KernelGlobals kg,
                                      const int atlas_slot,
                                      const float3 grid_pos_in_brick,
                                      const int brick_slot,
                                      const int bpa,
                                      const int atlas_dim)
{
  /* Dual voxel base: shift by -0.5 then floor. */
  int3 base = make_int3((int)floorf(grid_pos_in_brick.x - 0.5f),
                        (int)floorf(grid_pos_in_brick.y - 0.5f),
                        (int)floorf(grid_pos_in_brick.z - 0.5f));
  base.x = clamp(base.x, 0, SDF_BRICK_SIZE - 2);
  base.y = clamp(base.y, 0, SDF_BRICK_SIZE - 2);
  base.z = clamp(base.z, 0, SDF_BRICK_SIZE - 2);

  float3 uvw = make_float3(grid_pos_in_brick.x - 0.5f - float(base.x),
                           grid_pos_in_brick.y - 0.5f - float(base.y),
                           grid_pos_in_brick.z - 0.5f - float(base.z));
  uvw = clamp(uvw, zero_float3(), one_float3());

  /* Compact atlas base. */
  const int sx = brick_slot % bpa;
  const int sy = (brick_slot / bpa) % bpa;
  const int sz = brick_slot / (bpa * bpa);
  const int3 atlas_base = make_int3(sx * SDF_BRICK_STORAGE + base.x + 1,
                                    sy * SDF_BRICK_STORAGE + base.y + 1,
                                    sz * SDF_BRICK_STORAGE + base.z + 1);

  /* Fetch 3x3x3 = 27 neighborhood. */
  float v[27];
  for (int dz = 0; dz < 3; dz++) {
    for (int dy = 0; dy < 3; dy++) {
      for (int dx = 0; dx < 3; dx++) {
        v[dz * 9 + dy * 3 + dx] = sdf_fetch_distance(
            kg, atlas_slot, atlas_base.x + dx, atlas_base.y + dy, atlas_base.z + dz, atlas_dim);
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
/* Two-level DDA ray march. */

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
  const int atlas_slot = ksdf.atlas_slot;
  const int indir_slot = ksdf.indirection_slot;
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
  t_enter = max(t_enter, 0.0f);

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
        kg, indir_slot, brick_cell.x, brick_cell.y, brick_cell.z, grid_res);

    if (slot == -2) {
      /* Fully inside: immediate hit at entry. */
      hit_t = t_current;
      hit_brick = brick_cell;
      hit_brick_slot = 0;
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
        sdf_fetch_corners(kg, atlas_slot, vcell, slot, bpa, atlas_dim, s);

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

            if (c[0] <= 0.0f) {
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

  /* Record intersection. */
  isect->t = hit_t;
  isect->prim = sdf_index;
  isect->object = ksdf.object_id;
  isect->type = PRIMITIVE_SDF;
  isect->u = 0.0f;
  isect->v = 0.0f;

  return true;
}

/* March all SDF objects, return closest hit. */
ccl_device bool sdf_intersect_all(KernelGlobals kg,
                                   ccl_private const Ray *ray,
                                   ccl_private Intersection *isect,
                                   const uint visibility)
{
  bool hit = false;
  const int num_sdfs = kernel_data.sdf.num_sdfs;

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
  const int atlas_slot = ksdf.atlas_slot;
  const int atlas_dim = bpa * SDF_BRICK_STORAGE;
  const int grid_res = ksdf.grid_res;

  /* Hit position. */
  sd->P = ray->P + ray->D * isect->t;

  /* Find which brick and local position the hit is in. */
  float3 grid_pos = (sd->P - origin) / voxel_size;
  int3 brick_cell = make_int3((int)floorf(grid_pos.x / float(SDF_BRICK_SIZE)),
                              (int)floorf(grid_pos.y / float(SDF_BRICK_SIZE)),
                              (int)floorf(grid_pos.z / float(SDF_BRICK_SIZE)));
  brick_cell.x = clamp(brick_cell.x, 0, grid_res - 1);
  brick_cell.y = clamp(brick_cell.y, 0, grid_res - 1);
  brick_cell.z = clamp(brick_cell.z, 0, grid_res - 1);

  int brick_slot = sdf_fetch_indirection(
      kg, ksdf.indirection_slot, brick_cell.x, brick_cell.y, brick_cell.z, grid_res);

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
    sd->Ng = sdf_compute_normal(kg, atlas_slot, grid_pos_in_brick, brick_slot, bpa, atlas_dim);
  }
  else {
    sd->Ng = make_float3(0.0f, 0.0f, 1.0f);
  }
  sd->N = sd->Ng;

  /* Tangent frame from normal. */
  float3 up = (fabsf(sd->N.z) < 0.999f) ? make_float3(0.0f, 0.0f, 1.0f) :
                                            make_float3(1.0f, 0.0f, 0.0f);
  sd->dPdu = normalize(cross(up, sd->N));
  sd->dPdv = cross(sd->N, sd->dPdu);

  /* Read material ID from matid texture to determine shader. */
  if (brick_slot >= 0) {
    /* Sample matid texture at hit position. */
    int3 atlas_coord = sdf_grid_to_compact(
        make_int3((int)floorf(grid_pos_in_brick.x),
                  (int)floorf(grid_pos_in_brick.y),
                  (int)floorf(grid_pos_in_brick.z)),
        brick_slot,
        bpa);
    atlas_coord.x = clamp(atlas_coord.x, 0, atlas_dim - 1);
    atlas_coord.y = clamp(atlas_coord.y, 0, atlas_dim - 1);
    atlas_coord.z = clamp(atlas_coord.z, 0, atlas_dim - 1);

    float4 matid_val = sdf_fetch_atlas(
        kg, ksdf.matid_slot, atlas_coord.x, atlas_coord.y, atlas_coord.z, atlas_dim);
    int obj_id = (int)matid_val.x;

    /* Look up shader for this object. */
    if (obj_id >= 0 && ksdf.shader_offset + obj_id < ksdf.shader_offset + ksdf.num_objects) {
      sd->shader = kernel_data_fetch(sdf_shader_map, ksdf.shader_offset + obj_id);
    }
  }
}

CCL_NAMESPACE_END
