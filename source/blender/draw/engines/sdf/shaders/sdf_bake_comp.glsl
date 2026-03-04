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
#define MAX_CANDIDATES 64

/* Shared candidate list: BVH traversal done once per workgroup by thread 0,
 * then all 144 threads read from shared memory. Eliminates 143 redundant
 * BVH traversals per brick. */
shared int shared_candidates[MAX_CANDIDATES];
shared int shared_num_candidates;

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

  /* Per-brick AABB for object culling (includes 2-voxel overlap border).
   * Expand by max_blend so objects contributing to smooth union are evaluated.
   * Objects outside max_blend of the storage region get h=0 in the smooth
   * union (no contribution), so this is safe for cross-brick consistency. */
  float3 brick_min = atlas_origin + (float3(brick * BRICK_SIZE) - 2.0f) * voxel_size -
                      float3(max_blend);
  float3 brick_max = atlas_origin + (float3(brick * BRICK_SIZE + BRICK_SIZE) + 2.0f) * voxel_size +
                      float3(max_blend);

  /* Elect thread 0 to collect candidates for the entire workgroup.
   * All threads in the brick share the same AABB, so the candidate list
   * is identical — no need for each thread to traverse independently. */
  if (gl_LocalInvocationIndex == 0u) {
    shared_num_candidates = 0;

    if (bvh_node_count > 0) {
      int stack[BVH_MAX_STACK];
      int sp = 0;
      stack[sp++] = 0;

      while (sp > 0) {
        int node_idx = stack[--sp];
        BVHNodeGPU node = bvh_nodes[node_idx];

        if (!aabb_overlap(
                brick_min, brick_max, node.min_and_left.xyz, node.max_and_right.xyz))
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
      /* Fallback: linear scan collects all AABB-overlapping objects. */
      for (int i = 0; i < object_count; i++) {
        SDFObjectGPU obj = objects[i];

        if (any(greaterThan(brick_min, obj.bbox_max.xyz)) ||
            any(lessThan(brick_max, obj.bbox_min.xyz)))
        {
          continue;
        }

        if (shared_num_candidates < MAX_CANDIDATES) {
          shared_candidates[shared_num_candidates++] = i;
        }
      }
      /* Already in index order, no sort needed. */
    }
  }
  barrier();

  /* Local thread covers XY, loop over Z. */
  int2 local_xy = int2(gl_LocalInvocationID.xy);
  int num_candidates = shared_num_candidates;

  for (int lz = 0; lz < BRICK_STORAGE; lz++) {
    int3 local_voxel = int3(local_xy, lz);

    /* World-space position: brick_coord * 8 + (local - 2) + 0.5, times voxel_size.
     * The -2 accounts for the 2-voxel overlap border. */
    float3 world_pos = atlas_origin +
                       (float3(brick * BRICK_SIZE + local_voxel - int3(2)) + 0.5f) * voxel_size;

    float acc_dist = 1e10f;
    float3 acc_color = float3(0.0f);
    int acc_obj_id = -1;

    /* Evaluate candidates in deterministic index order. */
    for (int c = 0; c < num_candidates; c++) {
      int i = shared_candidates[c];
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
        acc_obj_id = (h > 0.5f) ? i : acc_obj_id;
      }
      else {
        if (dist < acc_dist) {
          acc_color = obj.color.rgb;
          acc_dist = dist;
          acc_obj_id = i;
        }
      }
    }

    int3 atlas_coord = slot_origin + local_voxel;
    imageStore(compact_atlas, atlas_coord, float4(acc_dist, acc_color));
    imageStore(object_id_atlas, atlas_coord, int4(acc_obj_id, 0, 0, 0));
  }
}
