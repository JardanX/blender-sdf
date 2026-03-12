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

/** Evaluate the actual SDF primitive for an object, with modifiers. */
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
   * plus max_blend for smooth union and max_shell_distance so bricks in the
   * shell zone can find base union objects as candidates. */
  float expand = brick_half_diag + max_blend + max_shell_distance;
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

  /* Group-aware sequential evaluation (matches sdf_bake_comp.glsl).
   * Grouped objects are evaluated sequentially within each group, then
   * groups combine via group-level CSG. Ungrouped objects use legacy two-pass. */
  float acc_dist = 1e10f;

  /* Evaluate grouped objects. */
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

      if (grp_dist >= 1e9f) {
        /* First object in group is always the base shape — CSG op ignored. */
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

    if (acc_dist >= 1e9f) {
      /* First group is always the base — group-level CSG op ignored. */
      acc_dist = grp_dist;
    }
    else {
      acc_dist = combineCSG(
          acc_dist, grp_dist, grp.csg_operation, grp.blend_type, grp.blend, grp.shell_distance);
    }
  }

  /* Evaluate ungrouped objects (group_id == -1) sequentially.
   * First ungrouped object is the base shape (CSG op ignored),
   * subsequent objects apply their CSG in order (top-to-bottom). */
  for (int c = 0; c < num_candidates; c++) {
    SDFObjectGPU obj = objects[candidates[c]];
    if (obj.group_id != -1) {
      continue;
    }

    float3 local_pos = (obj.inverse_matrix * float4(brick_center - obj.position.xyz, 1.0f)).xyz;
    float dist = evalSDFPrimitive(local_pos, obj);

    if (acc_dist >= 1e9f) {
      /* First ungrouped object is the base shape — CSG op ignored. */
      acc_dist = dist;
    }
    else {
      acc_dist = combineCSG(
          acc_dist, dist, obj.csg_operation, obj.blend_type, obj.blend, obj.shell_distance);
    }
  }

  /* Surface test: brick_half_diag is sufficient — the shell operation in
   * combineCSG produces correct thin-wall distances via two-stage blend,
   * so no global expansion needed. */
  float surface_threshold = brick_half_diag;
  if (abs(acc_dist) < surface_threshold) {
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
