/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Grid SDF blend: blends dense-float SDF grid into compact atlas via CSG.
 * One workgroup per brick, dispatched once per grid object. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_grid_blend)

#include "sdf_lib.glsl"

#define BRICK_SIZE 8
#define BRICK_STORAGE 12

void main()
{
  int brick_idx = int(gl_WorkGroupID.x) + int(gl_WorkGroupID.y) * dispatch_width;
  if (brick_idx >= active_brick_count) {
    return;
  }

  int4 brick_data = active_bricks[brick_idx].coord;
  int3 brick = brick_data.xyz;
  int slot = brick_data.w;

  int bpa = bricks_per_axis;
  int3 slot_block = int3(slot % bpa, (slot / bpa) % bpa, slot / (bpa * bpa));
  int3 slot_origin = slot_block * BRICK_STORAGE;

  int2 local_xy = int2(gl_LocalInvocationID.xy);

  for (int lz = 0; lz < BRICK_STORAGE; lz++) {
    int3 local_voxel = int3(local_xy, lz);

    float3 world_pos = atlas_origin +
                       float3(brick * BRICK_SIZE + local_voxel - int3(2)) * voxel_size;

    float3 grid_uvw = (grid_world_to_texture * float4(world_pos, 1.0f)).xyz;

    if (any(lessThan(grid_uvw, float3(0.0f))) || any(greaterThan(grid_uvw, float3(1.0f)))) {
      continue;
    }

    float grid_dist = textureLod(sdf_grid, grid_uvw, 0.0).r;

    int3 atlas_coord = slot_origin + local_voxel;
    float4 current = imageLoad(compact_atlas, atlas_coord);
    float acc_dist = current.r;
    float3 acc_color = current.gba;

    float k = grid_blend;
    if (acc_dist >= 1e9f) {
      if (grid_csg_operation == SDF_CSG_OP_SUBTRACT ||
          grid_csg_operation == SDF_CSG_OP_INTERSECT)
      {
        continue;
      }
      acc_color = grid_color.rgb;
      acc_dist = grid_dist;
    }
    else {
      float new_dist = combineCSG(
          acc_dist, grid_dist, grid_csg_operation, grid_blend_type, k, grid_shell_distance);

      float h = csgColorWeight(
          acc_dist, grid_dist, grid_csg_operation, grid_blend_type, k, grid_shell_distance);
      acc_color = mix(acc_color, grid_color.rgb, h);
      acc_dist = new_dist;
    }

    imageStore(compact_atlas, atlas_coord, float4(acc_dist, acc_color));
  }
}
