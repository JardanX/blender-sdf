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

  /* Pre-compute SDF box half-extents (size - bevel, clamped). */
  float3 box_size = shape_size - shape_bevel;
  box_size = max(box_size, float3(0.001f));

  for (int lz = 0; lz < BRICK_STORAGE; lz++) {
    int3 local_voxel = int3(local_xy, lz);

    /* Local-space position: brick * 8 + (local - 2) + 0.5, times voxel_size.
     * The -2 accounts for the 2-voxel overlap border. */
    float3 local_pos = local_origin +
                       (float3(brick * BRICK_SIZE + local_voxel - int3(2)) + 0.5f) *
                           local_voxel_size;

    /* Evaluate single SDF (box with bevel). */
    float dist = sdBox(local_pos, box_size) - shape_bevel;

    /* Store distance only. Color channel is zero — color comes from
     * per-instance data at march time, not from the shape atlas. */
    int3 atlas_coord = slot_origin + local_voxel;
    imageStore(compact_atlas, atlas_coord, float4(dist, 0.0f, 0.0f, 0.0f));
  }
}
