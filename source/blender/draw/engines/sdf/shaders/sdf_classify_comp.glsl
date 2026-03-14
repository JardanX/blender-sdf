/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* SDF classify: one thread per brick, surface test → atlas slot allocation.
 * Supports incremental mode (dirty region only, reuses existing slots). */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_classify)

#include "sdf_lib.glsl"

#define BRICK_SIZE 8
#define MAX_CANDIDATES 128

float evalSDFPrimitive(float3 local_pos, SDFObjectGPU obj)
{
  SDFPrimitiveData prim_data;
  prim_data.sdf_type = obj.sdf_type;
  prim_data.size = obj.sdf_size.xyz;
  prim_data.bevel = obj.bevel;
  prim_data.box_corners = obj.box_corners;
  prim_data.box_edges = obj.box_edges;
  prim_data.box_modes = obj.box_modes;
  prim_data.modifier_start = obj.modifier_start;
  prim_data.modifier_count = obj.modifier_count;

  return evalObjectSDF(prim_data, local_pos);
}

void main()
{
  int3 brick;

  if (incremental_mode == 1) {
    brick = dirty_brick_min + int3(gl_GlobalInvocationID);
    if (any(greaterThanEqual(brick, dirty_brick_max))) {
      return;
    }
    if (any(lessThan(brick, int3(0))) || any(greaterThanEqual(brick, grid_resolution.xyz))) {
      return;
    }
  }
  else {
    brick = int3(gl_GlobalInvocationID);
    if (any(greaterThanEqual(brick, grid_resolution.xyz))) {
      return;
    }
  }

  float3 brick_center = atlas_origin +
                         (float3(brick) * float(BRICK_SIZE) + float(BRICK_SIZE) * 0.5f) *
                             voxel_size;

  /* Per-brick AABB for culling (object AABBs already include blend padding). */
  float expand = brick_half_diag;
  float3 brick_min = brick_center - float3(expand);
  float3 brick_max = brick_center + float3(expand);

  /* Collect candidates from BVH, evaluate in index order. */
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
  }

  /* Group-aware sequential evaluation. */
  float acc_dist = 1e10f;

  for (int g = 0; g < group_count; g++) {
    SDFGroupGPU grp = groups[g];
    float grp_dist = 1e10f;

    for (int c = 0; c < num_candidates; c++) {
      SDFObjectGPU obj = objects[candidates[c]];
      if (obj.group_id != g) {
        continue;
      }

      float3 local_pos = (obj.inverse_matrix * float4(brick_center - obj.position.xyz, 1.0f)).xyz;
      float dist = evalSDFPrimitive(local_pos, obj);

      if (obj.group_first == 1) {
        grp_dist = dist;
      }
      else {
        grp_dist = combineCSG(
            grp_dist, dist, obj.csg_operation, obj.blend_type, obj.blend, obj.shell_distance);
      }
    }

    if (grp_dist >= 1e10f) {
      continue;
    }

    if (g == 0) {
      acc_dist = grp_dist;
    }
    else {
      acc_dist = combineCSG(
          acc_dist, grp_dist, grp.csg_operation, grp.blend_type, grp.blend, grp.shell_distance);
    }
  }

  /* Ungrouped objects. */
  for (int c = 0; c < num_candidates; c++) {
    SDFObjectGPU obj = objects[candidates[c]];
    if (obj.group_id != -1) {
      continue;
    }

    float3 local_pos = (obj.inverse_matrix * float4(brick_center - obj.position.xyz, 1.0f)).xyz;
    float dist = evalSDFPrimitive(local_pos, obj);

    acc_dist = combineCSG(
        acc_dist, dist, obj.csg_operation, obj.blend_type, obj.blend, obj.shell_distance);
  }

  float surface_threshold = brick_half_diag;
  if (abs(acc_dist) < surface_threshold) {
    int slot;
    if (incremental_mode == 1) {
      int old_slot = imageLoad(indirection_tex, brick).r;
      if (old_slot >= 0) {
        slot = old_slot;
      }
      else {
        slot = int(atomicAdd(brick_counter.next_slot, 1u));
      }
    }
    else {
      slot = int(atomicAdd(brick_counter.count, 1u));
    }

    imageStore(indirection_tex, brick, int4(slot, 0, 0, 0));

    if (incremental_mode == 1) {
      uint dirty_idx = atomicAdd(brick_counter.count, 1u);
      active_bricks[dirty_idx].coord = int4(brick, slot);
    }
    else {
      active_bricks[slot].coord = int4(brick, slot);
    }
  }
  else if (acc_dist < 0.0f) {
    imageStore(indirection_tex, brick, int4(-2, 0, 0, 0));
  }
  else {
    imageStore(indirection_tex, brick, int4(-1, 0, 0, 0));
  }
}
