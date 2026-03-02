/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * SDF bake compute shader (sparse brick version).
 * Each workgroup handles one brick. Local threads cover the 12x12 XY slice,
 * looping over Z to fill 12x12x12 voxels (8 inner + 2 overlap each side).
 */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_bake)

#include "sdf_lib.glsl"

#define BRICK_SIZE 8
#define BRICK_STORAGE 12

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

    /* World-space position: brick_coord * 8 + (local - 2) + 0.5, times voxel_size.
     * The -2 accounts for the 2-voxel overlap border. */
    float3 world_pos = atlas_origin +
                       (float3(brick * BRICK_SIZE + local_voxel - int3(2)) + 0.5f) * voxel_size;

    float acc_dist = 1e10f;
    float3 acc_color = float3(0.0f);

    for (int i = 0; i < object_count; i++) {
      SDFObjectGPU obj = objects[i];

      float3 local_pos = (obj.inverse_matrix * float4(world_pos - obj.position.xyz, 1.0f)).xyz;

      float3 size = obj.sdf_size.xyz - obj.bevel;
      size = max(size, float3(0.001f));
      float dist = sdBox(local_pos, size) - obj.bevel;

      float k = obj.blend;
      if (k > 0.0f && acc_dist < 1e9f) {
        float h = clamp(0.5f + 0.5f * (acc_dist - dist) / k, 0.0f, 1.0f);
        acc_color = mix(acc_color, obj.color.rgb, h);
        acc_dist = mix(acc_dist, dist, h) - k * h * (1.0f - h);
      }
      else {
        if (dist < acc_dist) {
          acc_color = obj.color.rgb;
          acc_dist = dist;
        }
      }
    }

    int3 atlas_coord = slot_origin + local_voxel;
    imageStore(compact_atlas, atlas_coord, float4(acc_dist, acc_color));
  }
}
