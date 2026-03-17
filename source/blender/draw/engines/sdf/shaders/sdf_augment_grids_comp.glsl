/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* GPU-side grid augmentation: marks bricks overlapping grid AABBs as active.
 * Replaces the CPU-side augment_indirection_for_grids() to avoid GPU->CPU readback. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_augment_grids)

void main()
{
  int3 brick = grid_brick_min + int3(gl_GlobalInvocationID);
  if (any(greaterThanEqual(brick, grid_brick_max))) {
    return;
  }
  if (any(lessThan(brick, int3(0))) || any(greaterThanEqual(brick, grid_resolution.xyz))) {
    return;
  }

  int current = imageLoad(indirection_tex, brick).r;
  if (current >= 0) {
    return;
  }

  uint slot = atomicAdd(brick_counter.count, 1u);
  if (int(slot) >= max_active_bricks) {
    return;
  }
  imageStore(indirection_tex, brick, int4(int(slot), 0, 0, 0));
  active_bricks[slot].coord = int4(brick, int(slot));
}
