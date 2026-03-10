/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Per-shape SDF bake compute shader.
 * Bakes a single SDF primitive into its local-space atlas region.
 * Each workgroup handles one active brick, 12x12 threads cover XY, loop Z.
 *
 * Unlike the world-space bake (sdf_bake_comp.glsl), this evaluates exactly
 * one SDF primitive per voxel — no BVH traversal, no multi-object blending.
 * Color is NOT stored (comes from per-instance data at march time).
 */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_shape_bake)

#include "sdf_lib.glsl"

#define BRICK_SIZE 8
#define BRICK_STORAGE 12

/* SDF primitive types (must match eSDFType in DNA_sdf_types.h). */
#define SDF_TYPE_BOX 0
#define SDF_TYPE_SPHERE 1
#define SDF_TYPE_CYLINDER 2
#define SDF_TYPE_CONE 3
#define SDF_TYPE_CAPSULE 4
#define SDF_TYPE_TORUS 5
#define SDF_TYPE_NGON 6

float sdSphere(float3 p, float r)
{
  return length(p) - r;
}

float sdCapsule(float3 p, float3 size)
{
  float h = size.y;
  float r = size.x;
  p.z -= clamp(p.z, -h, h);
  return length(p) - r;
}

float sdTorus(float3 p, float2 t)
{
  float2 q = float2(length(p.xy) - t.x, p.z);
  return length(q) - t.y;
}

float sdCylinder(float3 p, float3 size)
{
  float2 e = max(size.xy, float2(0.001f));
  float2 pn = p.xy / e;
  float rn = length(pn);
  float2 g = pn / (e * max(rn, 1e-6f));
  float radial = (rn - 1.0f) / max(length(g), 1e-6f);
  float vertical = abs(p.z) - size.z;
  float2 d = float2(radial, vertical);
  return length(max(d, float2(0.0f))) + min(max(d.x, d.y), 0.0f);
}

float sdCone(float3 p, float r, float h)
{
  float2 q = float2(length(p.xy), p.z);
  float2 k1 = float2(0.0f, h);
  float2 k2 = float2(-r, 2.0f * h);
  float2 ca = float2(q.x - min(q.x, (q.y < 0.0f) ? r : 0.0f), abs(q.y) - h);
  float2 cb = q - k1 + k2 * clamp(dot(k1 - q, k2) / dot(k2, k2), 0.0f, 1.0f);
  float s = (cb.x < 0.0f && ca.y < 0.0f) ? -1.0f : 1.0f;
  return s * sqrt(min(dot(ca, ca), dot(cb, cb)));
}

void main()
{
  /* Active-brick-only dispatch: one workgroup per active brick. */
  int brick_idx = int(gl_WorkGroupID.x) + int(gl_WorkGroupID.y) * dispatch_width;
  if (brick_idx >= active_brick_count) {
    return;
  }

  int4 brick_data = active_bricks[brick_idx + brick_offset].coord;
  int3 brick = brick_data.xyz;
  int slot = brick_data.w;

  /* Compute slot origin in compact atlas. */
  int bpa = bricks_per_axis;
  int3 slot_block = int3(slot % bpa, (slot / bpa) % bpa, slot / (bpa * bpa));
  int3 slot_origin = slot_block * BRICK_STORAGE;

  /* Local thread covers XY, loop over Z. */
  int2 local_xy = int2(gl_LocalInvocationID.xy);

  /* Pre-compute size with bevel compensation. */
  float bevel = shape_bevel;
  float3 sz = max(shape_size - bevel, float3(0.001f));

  for (int lz = 0; lz < BRICK_STORAGE; lz++) {
    int3 local_voxel = int3(local_xy, lz);

    /* Local-space position: brick * 8 + (local - 2), times voxel_size.
     * The -2 accounts for the 2-voxel overlap border.
     * Values at voxel CORNERS to match the DDA trilinear interpolation. */
    float3 local_pos = local_origin +
                       float3(brick * BRICK_SIZE + local_voxel - int3(2)) *
                           local_voxel_size;

    /* Evaluate SDF primitive based on shape type. */
    float dist;
    if (shape_type == SDF_TYPE_SPHERE) {
      dist = sdSphere(local_pos, sz.x);
    }
    else if (shape_type == SDF_TYPE_CYLINDER) {
      dist = sdCylinder(local_pos, sz);
    }
    else if (shape_type == SDF_TYPE_CONE) {
      dist = sdCone(local_pos, max(shape_size.x - bevel, 0.001f),
                                max(shape_size.y - bevel, 0.001f));
    }
    else if (shape_type == SDF_TYPE_CAPSULE) {
      dist = sdCapsule(local_pos, sz);
    }
    else if (shape_type == SDF_TYPE_TORUS) {
      float major = max(shape_size.x - bevel, 0.001f);
      float minor = max(shape_size.y - bevel, 0.001f);
      if (shape_torus_sc.y > -0.999f && shape_torus_sc.x > 0.001f) {
        /* Capped torus: shape_torus_sc = (sin, cos) of half-angle. */
        dist = sdCappedTorus(local_pos, shape_torus_sc, major, minor);
      }
      else {
        dist = sdTorus(local_pos, float2(major, minor));
      }
    }
    else if (shape_type == SDF_TYPE_NGON) {
      float R = sz.x;
      float halfH = sz.z;
      float d2d;
      if (shape_star > 0.001f) {
        d2d = sdStarPolygon2D(local_pos.xy, R, shape_sides, shape_star);
      }
      else {
        d2d = sdRegularPolygon2D(local_pos.xy, R, shape_sides);
      }
      float dz = abs(local_pos.z) - halfH;
      float2 dd = float2(d2d, dz);
      dist = length(max(dd, float2(0.0f))) + min(max(dd.x, dd.y), 0.0f);
    }
    else {
      /* SDF_TYPE_BOX (default). */
      dist = sdBox(local_pos, sz);
    }

    dist -= bevel;

    /* Store distance only. Color channel is zero — color comes from
     * per-instance data at march time, not from the shape atlas. */
    int3 atlas_coord = slot_origin + local_voxel;
    imageStore(compact_atlas, atlas_coord, float4(dist, 0.0f, 0.0f, 0.0f));
  }
}
