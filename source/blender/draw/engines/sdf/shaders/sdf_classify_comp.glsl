/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * SDF classify compute shader.
 * One thread per brick. Evaluates SDF at brick center to determine if the
 * brick contains surface. Active bricks get a compact atlas slot via atomic
 * counter; void bricks are marked -1 (outside) or -2 (inside).
 *
 * A workgroup-level "super-brick" coarse check evaluates the blended SDF at
 * the center of the 4x4x4 brick workgroup. If the distance exceeds
 * coarse_threshold, all 64 bricks are guaranteed to have no surface
 * (Lipschitz bound) and can skip per-brick evaluation entirely.
 *
 * Optimization: Thread 0 gathers candidate objects into shared memory during
 * the coarse check. All surviving threads reuse this shared candidate list
 * for per-brick evaluation, eliminating 63/64 redundant BVH traversals per
 * workgroup.
 */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_classify)

#include "sdf_lib.glsl"

#define BRICK_SIZE 8
#define MAX_CANDIDATES 64

/* Shared memory: coarse result + candidate list gathered once by thread 0. */
shared float coarse_dist;
shared int shared_candidates[MAX_CANDIDATES];
shared int shared_num_candidates;

void main()
{
  int3 brick = int3(gl_GlobalInvocationID);
  bool in_bounds = !any(greaterThanEqual(brick, grid_resolution.xyz));

  /* --- Super-brick coarse check (workgroup = 4x4x4 bricks = 32^3 voxels) ---
   * Thread 0 evaluates the blended SDF at the super-brick center AND gathers
   * candidate objects into shared memory. The super-brick AABB is a superset
   * of all individual brick AABBs, so the shared candidate list is
   * conservative but correct — extra candidates produce large distances that
   * don't affect the smooth union result. */
  if (gl_LocalInvocationIndex == 0u) {
    float3 super_center = atlas_origin +
                          (float3(gl_WorkGroupID) * 4.0f * float(BRICK_SIZE) +
                           2.0f * float(BRICK_SIZE)) *
                              voxel_size;
    float super_expand = coarse_threshold + max_blend;
    float3 super_min = super_center - float3(super_expand);
    float3 super_max = super_center + float3(super_expand);

    /* Gather candidates into shared memory. */
    shared_num_candidates = 0;

    if (bvh_node_count > 0) {
      int stack[BVH_MAX_STACK];
      int sp = 0;
      stack[sp++] = 0;

      while (sp > 0) {
        int node_idx = stack[--sp];
        BVHNodeGPU node = bvh_nodes[node_idx];

        if (!aabb_overlap(
                super_min, super_max, node.min_and_left.xyz, node.max_and_right.xyz))
        {
          continue;
        }

        int left = bvh_decode_int(node.min_and_left.w);
        int right = bvh_decode_int(node.max_and_right.w);

        if (left == -1) {
          if (shared_num_candidates < MAX_CANDIDATES) {
            shared_candidates[shared_num_candidates++] = right;
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
      for (int i = 1; i < shared_num_candidates; i++) {
        int key = shared_candidates[i];
        int j = i - 1;
        while (j >= 0 && shared_candidates[j] > key) {
          shared_candidates[j + 1] = shared_candidates[j];
          j--;
        }
        shared_candidates[j + 1] = key;
      }
    }
    else {
      /* Fallback: linear scan with OBB culling. */
      float3 query_half = (super_max - super_min) * 0.5f;
      float expand_radius = length(query_half);

      for (int i = 0; i < object_count; i++) {
        SDFObjectGPU obj = objects[i];

        float3 local_qc = (obj.inverse_matrix *
                           float4(super_center - obj.position.xyz, 1.0f))
                              .xyz;
        float3 obb_ext = obj.sdf_size.xyz + obj.bevel + obj.blend;
        if (any(greaterThan(abs(local_qc), obb_ext + expand_radius))) {
          continue;
        }

        if (shared_num_candidates < MAX_CANDIDATES) {
          shared_candidates[shared_num_candidates++] = i;
        }
      }
      /* Already in index order, no sort needed. */
    }

    /* Evaluate coarse SDF using shared candidates. */
    float acc = 1e10f;

    for (int c = 0; c < shared_num_candidates; c++) {
      SDFObjectGPU obj = objects[shared_candidates[c]];

      float3 local_pos = (obj.inverse_matrix *
                          float4(super_center - obj.position.xyz, 1.0f))
                             .xyz;

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

    coarse_dist = acc;
  }
  barrier();

  if (abs(coarse_dist) > coarse_threshold) {
    /* Entire super-brick is far from any surface. Mark in-bounds bricks
     * as inside (-2) or outside (-1) based on coarse sign. */
    if (in_bounds) {
      if (coarse_dist < 0.0f) {
        imageStore(indirection_tex, brick, int4(-2, 0, 0, 0));
      }
      else {
        imageStore(indirection_tex, brick, int4(-1, 0, 0, 0));
      }
    }
    return;
  }

  /* --- Per-brick fine classification --- */
  if (!in_bounds) {
    return;
  }

  /* World-space center of this brick. */
  float3 brick_center = atlas_origin +
                        (float3(brick) * float(BRICK_SIZE) + float(BRICK_SIZE) * 0.5f) *
                            voxel_size;

  /* Evaluate SDF using the shared candidate list (no per-thread BVH traversal). */
  float acc_dist = 1e10f;

  for (int c = 0; c < shared_num_candidates; c++) {
    SDFObjectGPU obj = objects[shared_candidates[c]];

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
