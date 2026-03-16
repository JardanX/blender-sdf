/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* SDF classify: one thread per brick, surface test -> atlas slot allocation.
 * Linear scan through sorted objects — no candidate buffer, no hard limits.
 * Supports incremental mode (dirty region only, reuses existing slots). */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_classify)

#include "sdf_lib.glsl"

#define BRICK_SIZE 8

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

  float expand = brick_half_diag;
  float3 brick_min = brick_center - float3(expand);
  float3 brick_max = brick_center + float3(expand);

  /* Incremental dirty check: skip bricks where no overlapping object changed. */
  if (incremental_mode == 1 && has_dirty_flags == 1) {
    bool any_dirty = false;
    for (int i = 0; i < object_count && !any_dirty; i++) {
      SDFObjectGPU obj = objects[i];
      if (any(greaterThan(brick_min, obj.bbox_max.xyz)) ||
          any(lessThan(brick_max, obj.bbox_min.xyz)))
      {
        continue;
      }
      if (dirty_flags[i] != 0) {
        any_dirty = true;
      }
    }
    if (!any_dirty) {
      return;
    }
  }

  /* BVH Traversal to build object presence bitmask.
   * Eliminates the need to evaluate AABBs or SDFs for culled objects,
   * while naturally preserving the required topological sort order. */
  #define MAX_MASK_WORDS 64 /* Supports up to 2048 objects. */
  uint mask[MAX_MASK_WORDS];
  for (int i = 0; i < MAX_MASK_WORDS; i++) {
    mask[i] = 0u;
  }

  bool use_bvh = (bvh_node_count > 0 && object_count <= MAX_MASK_WORDS * 32);

  if (use_bvh) {
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
        if (right < MAX_MASK_WORDS * 32) {
          mask[right >> 5] |= (1u << (right & 31));
        }
      }
      else {
        if (sp < BVH_MAX_STACK - 1) {
          stack[sp++] = left;
          stack[sp++] = right;
        }
      }
    }
  }

  /* Single-pass group-aware evaluation over all objects in sorted order. */
  float acc_dist = 1e10f;
  int prev_group = -2;
  float grp_dist = 1e10f;

  for (int i = 0; i < object_count; i++) {
    /* If BVH is active, skip objects not found in the BVH traversal. */
    if (use_bvh && (mask[i >> 5] & (1u << (i & 31))) == 0u) {
      continue;
    }

    SDFObjectGPU obj = objects[i];

    /* Group transition (before AABB cull so group boundaries are tracked). */
    if (obj.group_id != prev_group) {
      if (prev_group >= 0 && grp_dist < 1e10f) {
        SDFGroupGPU grp = groups[prev_group];
        acc_dist = combineCSG(
            acc_dist, grp_dist, grp.csg_operation, grp.blend_type, grp.blend, grp.shell_distance);
      }
      prev_group = obj.group_id;
      if (prev_group >= 0) {
        grp_dist = 1e10f;
      }
    }

    /* Per-brick AABB cull. */
    if (any(greaterThan(brick_min, obj.bbox_max.xyz)) ||
        any(lessThan(brick_max, obj.bbox_min.xyz)))
    {
      if (obj.group_id >= 0 && obj.group_first == 1) {
        grp_dist = 1e10f;
      }
      continue;
    }

    float3 local_pos = (obj.inverse_matrix * float4(brick_center - obj.position.xyz, 1.0f)).xyz;
    float dist = evalSDFPrimitive(local_pos, obj);

    if (obj.group_id >= 0) {
      if (obj.group_first == 1) {
        grp_dist = dist;
      }
      else {
        grp_dist = combineCSG(
            grp_dist, dist, obj.csg_operation, obj.blend_type, obj.blend, obj.shell_distance);
      }
    }
    else {
      acc_dist = combineCSG(
          acc_dist, dist, obj.csg_operation, obj.blend_type, obj.blend, obj.shell_distance);
    }
  }

  /* Finalize last group. */
  if (prev_group >= 0 && grp_dist < 1e10f) {
    SDFGroupGPU grp = groups[prev_group];
    acc_dist = combineCSG(
        acc_dist, grp_dist, grp.csg_operation, grp.blend_type, grp.blend, grp.shell_distance);
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
