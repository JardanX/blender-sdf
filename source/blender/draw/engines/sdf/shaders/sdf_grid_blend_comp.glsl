/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Grid SDF blend compute shader (sparse brick version).
 * Blends one dense-float SDF grid into the existing compact atlas using smooth union.
 * Dispatched once per grid object, with a memory barrier between dispatches.
 * Each workgroup handles one brick, same layout as the bake shader.
 */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_grid_blend)

#define BRICK_SIZE 8
#define BRICK_STORAGE 10

void main()
{
  /* Each workgroup = one brick. gl_WorkGroupID = brick coord. */
  int3 brick = int3(gl_WorkGroupID);

  if (any(greaterThanEqual(brick, grid_resolution.xyz))) {
    return;
  }

  /* Read indirection: skip void bricks. */
  int slot = texelFetch(indirection_tx, brick, 0).r;
  if (slot < 0) {
    return;
  }

  /* Compute slot origin in compact atlas. */
  int bpa = bricks_per_axis;
  int3 slot_block = int3(slot % bpa, (slot / bpa) % bpa, slot / (bpa * bpa));
  int3 slot_origin = slot_block * BRICK_STORAGE;

  /* Local thread covers XY, loop over Z. */
  int2 local_xy = int2(gl_LocalInvocationID.xy);
  if (any(greaterThanEqual(local_xy, int2(BRICK_STORAGE)))) {
    return;
  }

  for (int lz = 0; lz < BRICK_STORAGE; lz++) {
    int3 local_voxel = int3(local_xy, lz);

    /* World-space position: same calculation as the bake shader.
     * brick_coord * 8 + (local - 1) + 0.5, times voxel_size.
     * The -1 accounts for the overlap border. */
    float3 world_pos = atlas_origin +
                       (float3(brick * BRICK_SIZE + local_voxel - int3(1)) + 0.5f) * voxel_size;

    /* Transform world position to grid texture UVW [0,1]^3. */
    float3 grid_uvw = (grid_world_to_texture * float4(world_pos, 1.0f)).xyz;

    /* Skip voxels outside the grid volume. */
    if (any(lessThan(grid_uvw, float3(0.0f))) || any(greaterThan(grid_uvw, float3(1.0f)))) {
      continue;
    }

    /* Sample the grid SDF at this position.
     * OpenVDB stores SDF values in index space (voxel units).
     * Multiply by grid_voxel_size to convert to world-space distance. */
    float grid_dist = texture(sdf_grid, grid_uvw).r * grid_voxel_size;

    /* Read current accumulated atlas value. */
    int3 atlas_coord = slot_origin + local_voxel;
    float4 current = imageLoad(compact_atlas, atlas_coord);
    float acc_dist = current.r;
    float3 acc_color = current.gba;

    /* Blend grid distance into atlas using smooth union. */
    float k = grid_blend;
    if (k > 0.0f && acc_dist < 1e9f) {
      float h = clamp(0.5f + 0.5f * (acc_dist - grid_dist) / k, 0.0f, 1.0f);
      acc_color = mix(acc_color, grid_color.rgb, h);
      acc_dist = mix(acc_dist, grid_dist, h) - k * h * (1.0f - h);
    }
    else {
      if (grid_dist < acc_dist) {
        acc_color = grid_color.rgb;
        acc_dist = grid_dist;
      }
    }

    imageStore(compact_atlas, atlas_coord, float4(acc_dist, acc_color));
  }
}
