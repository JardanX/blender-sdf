/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * SDF prepare dispatch compute shader.
 * Single-thread shader that converts brick_counter.count into indirect
 * dispatch parameters, avoiding the GPU->CPU readback stall between
 * classify and bake.
 *
 * Also writes dispatch_x / dispatch_y into brick_counter so the bake
 * shader can derive dispatch_width without a push constant.
 */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_prepare_dispatch)

void main()
{
  uint count = brick_counter.count;
  uint x = min(count, 65535u);
  uint y = (x > 0u) ? ((count + x - 1u) / x) : 1u;

  dispatch_indirect.groups_x = x;
  dispatch_indirect.groups_y = max(y, 1u);
  dispatch_indirect.groups_z = 1u;

  /* Store in brick_counter so bake shader can read dispatch_width. */
  brick_counter.dispatch_x = x;
  brick_counter.dispatch_y = max(y, 1u);
}
