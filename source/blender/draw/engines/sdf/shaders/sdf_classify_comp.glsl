/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * SDF classify compute shader.
 * One thread per brick. Evaluates SDF at brick center to determine if the
 * brick contains surface. Active bricks get a compact atlas slot via atomic
 * counter; void bricks are marked -1 (outside) or -2 (inside).
 */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_classify)

#include "sdf_lib.glsl"

#define BRICK_SIZE 8

void main()
{
  int3 brick = int3(gl_GlobalInvocationID);

  if (any(greaterThanEqual(brick, grid_resolution.xyz))) {
    return;
  }

  /* World-space center of this brick. */
  float3 brick_center = atlas_origin +
                         (float3(brick) * float(BRICK_SIZE) + float(BRICK_SIZE) * 0.5f) *
                             voxel_size;

  /* Per-brick AABB for object culling.
   * Expand by brick_half_diag to match the surface test threshold. */
  float3 brick_min = brick_center - float3(brick_half_diag);
  float3 brick_max = brick_center + float3(brick_half_diag);

  /* Evaluate SDF at brick center using all objects. */
  float acc_dist = 1e10f;

  for (int i = 0; i < object_count; i++) {
    SDFObjectGPU obj = objects[i];

    /* Per-brick AABB culling: skip objects that can't affect this brick. */
    if (any(greaterThan(brick_min, obj.bbox_max.xyz)) ||
        any(lessThan(brick_max, obj.bbox_min.xyz))) {
      continue;
    }

    float3 local_pos = (obj.inverse_matrix * float4(brick_center - obj.position.xyz, 1.0f)).xyz;

    float3 size = obj.sdf_size.xyz - obj.bevel;
    size = max(size, float3(0.001f));
    float dist = sdBox(local_pos, size) - obj.bevel;

    float k = obj.blend;
    if (k > 0.0f && acc_dist < 1e9f) {
      float h = clamp(0.5f + 0.5f * (acc_dist - dist) / k, 0.0f, 1.0f);
      acc_dist = mix(acc_dist, dist, h) - k * h * (1.0f - h);
    }
    else {
      acc_dist = min(acc_dist, dist);
    }
  }

  /* Conservative surface test: if the absolute distance at the brick center
   * is less than the brick's half-diagonal, the surface might pass through. */
  if (abs(acc_dist) < brick_half_diag) {
    /* Active brick: allocate a compact atlas slot. */
    uint slot = atomicAdd(brick_counter.count, 1u);
    imageStore(indirection_tex, brick, int4(int(slot), 0, 0, 0));
  }
  else if (acc_dist < 0.0f) {
    /* Fully inside: mark as -2. */
    imageStore(indirection_tex, brick, int4(-2, 0, 0, 0));
  }
  else {
    /* Fully outside: mark as -1. */
    imageStore(indirection_tex, brick, int4(-1, 0, 0, 0));
  }
}
