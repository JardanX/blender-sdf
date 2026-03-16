/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Group atlas blend: merges a group's sparse brick atlas into the main atlas via CSG.
 * One workgroup per MAIN atlas active brick, 12x12 threads cover XY, loop Z.
 * Both grids share the same voxel_size and are chunk-aligned, so brick
 * coordinates map via integer offset — no floating-point conversion needed. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_group_blend)

#include "sdf_lib.glsl"

#define BRICK_SIZE 8
#define BRICK_STORAGE 12

void main()
{
  int brick_idx = int(gl_WorkGroupID.x) + int(gl_WorkGroupID.y) * dispatch_width;
  if (brick_idx >= main_active_brick_count) {
    return;
  }

  int4 brick_data = active_bricks[brick_idx].coord;
  int3 brick = brick_data.xyz;
  int slot = brick_data.w;

  int bpa = main_bricks_per_axis;
  int3 slot_block = int3(slot % bpa, (slot / bpa) % bpa, slot / (bpa * bpa));
  int3 slot_origin = slot_block * BRICK_STORAGE;

  /* Map main brick to group brick via integer offset (chunk-aligned grids). */
  int3 group_brick = brick - group_brick_offset;

  if (any(lessThan(group_brick, int3(0))) ||
      any(greaterThanEqual(group_brick, group_grid_res)))
  {
    return;
  }

  int group_slot = texelFetch(group_indirection_tx, group_brick, 0).r;
  if (group_slot == -1) {
    return;
  }

  int gbpa = group_bricks_per_axis;
  int3 g_slot_origin = int3(0);
  if (group_slot >= 0) {
    int3 gsb = int3(group_slot % gbpa,
                    (group_slot / gbpa) % gbpa,
                    group_slot / (gbpa * gbpa));
    g_slot_origin = gsb * BRICK_STORAGE;
  }

  int2 local_xy = int2(gl_LocalInvocationID.xy);

  for (int lz = 0; lz < BRICK_STORAGE; lz++) {
    int3 local_voxel = int3(local_xy, lz);

    float group_dist;
    float3 group_color;

    if (group_slot == -2) {
      group_dist = -voxel_size * 4.0f;
      group_color = group_tint.rgb;
    }
    else {
      int3 g_atlas_coord = g_slot_origin + local_voxel;
      float4 g_val = texelFetch(group_compact_atlas_tx, g_atlas_coord, 0);
      group_dist = g_val.r;
      group_color = g_val.gba;
    }

    group_color *= group_tint.rgb;

    int3 atlas_coord = slot_origin + local_voxel;
    float4 current = imageLoad(compact_atlas, atlas_coord);
    float acc_dist = current.r;
    float3 acc_color = current.gba;

    float new_dist = combineCSG(
        acc_dist, group_dist, group_csg_op, group_blend_type, group_blend_k, group_shell_dist);
    float h = csgColorWeight(
        acc_dist, group_dist, group_csg_op, group_blend_type, group_blend_k, group_shell_dist);
    acc_color = mix(acc_color, group_color, h);
    acc_dist = new_dist;

    imageStore(compact_atlas, atlas_coord, float4(acc_dist, acc_color));
  }
}
