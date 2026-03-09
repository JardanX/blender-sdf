/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Grid SDF blend compute shader (sparse brick version).
 * Blends one dense-float SDF grid into the existing compact atlas using the
 * specified CSG operation and blend type.
 * Dispatched once per grid object, with a memory barrier between dispatches.
 * Each workgroup handles one brick, same layout as the bake shader.
 */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_grid_blend)

#include "sdf_lib.glsl"

#define BRICK_SIZE 8
#define BRICK_STORAGE 12

void main()
{
  /* Active-brick-only dispatch: one workgroup per active brick.
   * 2D dispatch to avoid GL's 65535 workgroup limit per axis. */
  int brick_idx = int(gl_WorkGroupID.x) + int(gl_WorkGroupID.y) * dispatch_width;
  if (brick_idx >= active_brick_count) {
    return;
  }

  int4 brick_data = active_bricks[brick_idx].coord;
  int3 brick = brick_data.xyz;
  int slot = brick_data.w;

  /* Compute slot origin in compact atlas. */
  int bpa = bricks_per_axis;
  int3 slot_block = int3(slot % bpa, (slot / bpa) % bpa, slot / (bpa * bpa));
  int3 slot_origin = slot_block * BRICK_STORAGE;

  /* Local thread covers XY, loop over Z. */
  int2 local_xy = int2(gl_LocalInvocationID.xy);

  for (int lz = 0; lz < BRICK_STORAGE; lz++) {
    int3 local_voxel = int3(local_xy, lz);

    /* World-space position: same calculation as the bake shader.
     * brick_coord * 8 + (local - 2) + 0.5, times voxel_size.
     * The -2 accounts for the 2-voxel overlap border. */
    float3 world_pos = atlas_origin +
                       (float3(brick * BRICK_SIZE + local_voxel - int3(2)) + 0.5f) * voxel_size;

    /* Transform world position to grid texture UVW [0,1]^3. */
    float3 grid_uvw = (grid_world_to_texture * float4(world_pos, 1.0f)).xyz;

    /* Skip voxels outside the grid volume. */
    if (any(lessThan(grid_uvw, float3(0.0f))) || any(greaterThan(grid_uvw, float3(1.0f)))) {
      continue;
    }

    /* Sample the grid SDF at this position.
     * OpenVDB level-set values are already in world-space units.
     * Inactive voxels (outside the narrow band) are filled with ±background,
     * which provides a conservative distance estimate. Including them prevents
     * discontinuities between grid data and the atlas clear value (1e10) that
     * would cause the ray marcher to overshoot and miss the surface. */
    float grid_dist = textureLod(sdf_grid, grid_uvw, 0.0).r;

    /* Read current accumulated atlas value. */
    int3 atlas_coord = slot_origin + local_voxel;
    float4 current = imageLoad(compact_atlas, atlas_coord);
    float acc_dist = current.r;
    float3 acc_color = current.gba;

    /* Blend grid distance into atlas using the specified CSG operation and blend type. */
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

      if (grid_csg_operation == SDF_CSG_OP_SUBTRACT) {
        acc_dist = new_dist;
      }
      else if (k > 0.0f && grid_blend_type == SDF_BLEND_TYPE_SMOOTH) {
        float h;
        if (grid_csg_operation == SDF_CSG_OP_UNION) {
          h = clamp(0.5f + 0.5f * (acc_dist - grid_dist) / k, 0.0f, 1.0f);
        }
        else {
          h = clamp(0.5f - 0.5f * (acc_dist - grid_dist) / k, 0.0f, 1.0f);
        }
        acc_color = mix(acc_color, grid_color.rgb, h);
        acc_dist = new_dist;
      }
      else {
        if (grid_csg_operation == SDF_CSG_OP_UNION && grid_dist < acc_dist) {
          acc_color = grid_color.rgb;
        }
        else if (grid_csg_operation == SDF_CSG_OP_INTERSECT && grid_dist > acc_dist) {
          acc_color = grid_color.rgb;
        }
        acc_dist = new_dist;
      }
    }

    imageStore(compact_atlas, atlas_coord, float4(acc_dist, acc_color));
  }
}
