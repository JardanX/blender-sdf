/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Marks chunk-local bricks that overlap dense grid AABBs as active. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_augment_grids)

#define BRICK_SIZE 8
#define CHUNK_BRICK_RES 16

bool aabb_overlap_local(float3 min_a, float3 max_a, float3 min_b, float3 max_b)
{
  return !any(greaterThan(min_a, max_b)) && !any(lessThan(max_a, min_b));
}

void main()
{
  int chunk_idx = int(gl_WorkGroupID.x) + int(gl_WorkGroupID.y) * dispatch_width;
  if (chunk_idx >= chunk_count) {
    return;
  }

  ChunkPageGPU chunk = chunk_pages[chunk_idx];
  float chunk_world = float(CHUNK_BRICK_RES * BRICK_SIZE) * voxel_size;
  float3 chunk_min = float3(chunk.coord.xyz) * chunk_world;
  float3 chunk_max = chunk_min + float3(chunk_world);
  if (!aabb_overlap_local(chunk_min, chunk_max, grid_world_min, grid_world_max)) {
    return;
  }

  int2 local_xy = int2(gl_LocalInvocationID.xy);
  int chunk_offset = chunk_idx * (CHUNK_BRICK_RES * CHUNK_BRICK_RES * CHUNK_BRICK_RES);
  float brick_world = float(BRICK_SIZE) * voxel_size;

  for (int lz = 0; lz < CHUNK_BRICK_RES; lz++) {
    int3 local_brick = int3(local_xy, lz);
    int flat_index = local_brick.x + local_brick.y * CHUNK_BRICK_RES +
                     local_brick.z * CHUNK_BRICK_RES * CHUNK_BRICK_RES;
    int3 brick = chunk.coord.xyz * CHUNK_BRICK_RES + local_brick;

    float3 brick_min = float3(brick * BRICK_SIZE) * voxel_size;
    float3 brick_max = brick_min + float3(brick_world);
    if (!aabb_overlap_local(brick_min, brick_max, grid_world_min, grid_world_max)) {
      continue;
    }

    int current = chunk_bricks[chunk_offset + flat_index];
    if (current >= 0) {
      continue;
    }

    uint slot = atomicAdd(brick_counter.count, 1u);
    if (slot < uint(max_active_bricks)) {
      chunk_bricks[chunk_offset + flat_index] = int(slot);
      active_bricks[slot].coord = int4(brick, int(slot));
    }
    else {
      atomicMax(brick_counter.next_slot, 1u);
    }
  }
}
