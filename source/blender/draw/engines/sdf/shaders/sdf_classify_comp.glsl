/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * SDF classify compute shader.
 * One thread per brick, 4x4x4 workgroup = one super-brick.
 * Thread 0 does a single BVH traversal for the whole super-brick and stores
 * the candidate list in shared memory. A coarse SDF evaluation at the
 * super-brick center lets entire workgroups skip early when the surface is
 * far away. Remaining threads evaluate per-brick SDF from the shared list.
 */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_classify)

#include "sdf_lib.glsl"

#define BRICK_SIZE 8
#define MAX_CANDIDATES 64

/* Shared memory: candidate object indices + count + coarse SDF result. */
shared int s_candidates[MAX_CANDIDATES];
shared int s_num_candidates;
shared float s_coarse_dist;

void main()
{
  int3 brick = int3(gl_GlobalInvocationID);
  uint local_id = gl_LocalInvocationIndex;

  /* ---- Phase 1: thread 0 does BVH traversal for the super-brick ---- */
  if (local_id == 0u) {
    s_num_candidates = 0;
    s_coarse_dist = 1e10f;

    /* Super-brick world AABB (covers all 4x4x4 bricks in this workgroup). */
    int3 sb_origin = int3(gl_WorkGroupID) * 4;
    float3 sb_min_world = atlas_origin +
                          float3(sb_origin) * float(BRICK_SIZE) * voxel_size;
    float3 sb_max_world = atlas_origin +
                          float3(sb_origin + 4) * float(BRICK_SIZE) * voxel_size;
    float3 sb_center = (sb_min_world + sb_max_world) * 0.5f;

    /* Expand by coarse_threshold + max_blend for conservative object culling. */
    float expand = coarse_threshold + max_blend;
    float3 sb_min = sb_min_world - float3(expand);
    float3 sb_max = sb_max_world + float3(expand);

    if (bvh_node_count > 0) {
      int stack[BVH_MAX_STACK];
      int sp = 0;
      stack[sp++] = 0;

      while (sp > 0) {
        int node_idx = stack[--sp];
        BVHNodeGPU node = bvh_nodes[node_idx];

        if (!aabb_overlap(sb_min, sb_max, node.min_and_left.xyz, node.max_and_right.xyz)) {
          continue;
        }

        int left = bvh_decode_int(node.min_and_left.w);
        int right = bvh_decode_int(node.max_and_right.w);

        if (left == -1) {
          if (s_num_candidates < MAX_CANDIDATES) {
            s_candidates[s_num_candidates++] = right;
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
      for (int i = 1; i < s_num_candidates; i++) {
        int key = s_candidates[i];
        int j = i - 1;
        while (j >= 0 && s_candidates[j] > key) {
          s_candidates[j + 1] = s_candidates[j];
          j--;
        }
        s_candidates[j + 1] = key;
      }
    }
    else {
      /* Fallback: linear scan for objects overlapping super-brick AABB. */
      for (int i = 0; i < object_count; i++) {
        SDFObjectGPU obj = objects[i];

        if (any(greaterThan(sb_min, obj.bbox_max.xyz)) ||
            any(lessThan(sb_max, obj.bbox_min.xyz))) {
          continue;
        }

        if (s_num_candidates < MAX_CANDIDATES) {
          s_candidates[s_num_candidates++] = i;
        }
      }
      /* Already in index order, no sort needed. */
    }

    /* Coarse SDF evaluation at super-brick center. */
    if (s_num_candidates > 0) {
      float acc = 1e10f;

      for (int c = 0; c < s_num_candidates; c++) {
        SDFObjectGPU obj = objects[s_candidates[c]];

        float3 local_pos = (obj.inverse_matrix * float4(sb_center - obj.position.xyz, 1.0f)).xyz;

        float3 size = obj.sdf_size.xyz - obj.bevel;
        size = max(size, float3(0.001f));
        float dist = sdBox(local_pos, size) - obj.bevel;

        float k = obj.blend;
        if (k > 0.0f && acc < 1e9f) {
          float h = clamp(0.5f + 0.5f * (acc - dist) / k, 0.0f, 1.0f);
          acc = mix(acc, dist, h) - k * h * (1.0f - h);
        }
        else {
          acc = min(acc, dist);
        }
      }
      s_coarse_dist = acc;
    }
  }

  barrier();

  /* ---- Phase 2: coarse early-out for the entire workgroup ---- */

  /* No candidates, or surface is farther than coarse_threshold from
   * the super-brick center — no brick in this group can contain surface. */
  if (s_num_candidates == 0 || abs(s_coarse_dist) > coarse_threshold) {
    if (all(lessThan(brick, grid_resolution.xyz))) {
      imageStore(indirection_tex, brick, int4(s_coarse_dist < 0.0f ? -2 : -1, 0, 0, 0));
    }
    return;
  }

  /* Out-of-bounds threads exit after the coarse pass (they still
   * participated in the barrier above). */
  if (any(greaterThanEqual(brick, grid_resolution.xyz))) {
    return;
  }

  /* ---- Phase 3: per-brick SDF from shared candidate list ---- */

  float3 brick_center = atlas_origin +
                         (float3(brick) * float(BRICK_SIZE) + float(BRICK_SIZE) * 0.5f) *
                             voxel_size;

  /* Per-brick AABB for fine culling against candidates. */
  float fine_expand = brick_half_diag + max_blend;
  float3 b_min = brick_center - float3(fine_expand);
  float3 b_max = brick_center + float3(fine_expand);

  float acc_dist = 1e10f;

  for (int c = 0; c < s_num_candidates; c++) {
    SDFObjectGPU obj = objects[s_candidates[c]];

    /* Fine per-brick AABB cull. */
    if (any(greaterThan(b_min, obj.bbox_max.xyz)) ||
        any(lessThan(b_max, obj.bbox_min.xyz))) {
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
