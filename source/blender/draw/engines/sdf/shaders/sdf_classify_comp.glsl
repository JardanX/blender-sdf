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
#define MAX_CANDIDATES 64

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
   * Expand by brick_half_diag to match the surface test threshold,
   * plus max_blend to capture objects contributing to smooth union. */
  float expand = brick_half_diag + max_blend;
  float3 brick_min = brick_center - float3(expand);
  float3 brick_max = brick_center + float3(expand);

  /* Collect candidate objects from BVH, then evaluate in index order.
   * Deterministic evaluation order ensures consistent blending when
   * objects have different blend values. */
  int candidates[MAX_CANDIDATES];
  int num_candidates = 0;

  if (bvh_node_count > 0) {
    int stack[BVH_MAX_STACK];
    int sp = 0;
    stack[sp++] = 0;

    while (sp > 0) {
      int node_idx = stack[--sp];
      BVHNodeGPU node = bvh_nodes[node_idx];

      if (!aabb_overlap(brick_min, brick_max, node.min_and_left.xyz, node.max_and_right.xyz)) {
        continue;
      }

      int left = bvh_decode_int(node.min_and_left.w);
      int right = bvh_decode_int(node.max_and_right.w);

      if (left == -1) {
        if (num_candidates < MAX_CANDIDATES) {
          candidates[num_candidates++] = right;
        }
      }
      else {
        if (sp < BVH_MAX_STACK - 1) {
          stack[sp++] = left;
          stack[sp++] = right;
        }
      }
    }

    /* Sort candidates by object index (insertion sort, small N). */
    for (int i = 1; i < num_candidates; i++) {
      int key = candidates[i];
      int j = i - 1;
      while (j >= 0 && candidates[j] > key) {
        candidates[j + 1] = candidates[j];
        j--;
      }
      candidates[j + 1] = key;
    }
  }
  else {
    /* Fallback: linear scan collects all AABB-overlapping objects. */
    for (int i = 0; i < object_count; i++) {
      SDFObjectGPU obj = objects[i];

      if (any(greaterThan(brick_min, obj.bbox_max.xyz)) ||
          any(lessThan(brick_max, obj.bbox_min.xyz))) {
        continue;
      }

      if (num_candidates < MAX_CANDIDATES) {
        candidates[num_candidates++] = i;
      }
    }
    /* Already in index order, no sort needed. */
  }

  /* Evaluate candidates in deterministic index order. */
  float acc_dist = 1e10f;

  for (int c = 0; c < num_candidates; c++) {
    SDFObjectGPU obj = objects[candidates[c]];

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
    /* Record brick coordinate for active-brick-only dispatch in bake/grid_blend. */
    active_bricks[slot].coord = int4(brick, int(slot));
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
