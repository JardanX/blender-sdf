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
  prim_data.inverse_matrix = obj.inverse_matrix;

  return evalObjectSDF(prim_data, local_pos);
}

#define MAX_MASK_WORDS 64 /* Supports up to 2048 objects. */
shared uint wg_mask[MAX_MASK_WORDS];
shared uint wg_flags;

float evaluateSceneAt(float3 sample_pos,
                      float3 brick_min,
                      float3 brick_max,
                      bool use_bvh)
{
  float acc_dist = 1e10f;
  int prev_group = -2;
  float grp_dist = 1e10f;

  for (int i = 0; i < object_count; i++) {
    if (use_bvh && (wg_mask[i >> 5] & (1u << (i & 31))) == 0u) {
      continue;
    }

    SDFObjectGPU obj = objects[i];

    if (obj.group_id != prev_group) {
      if (prev_group >= 0 && grp_dist < 1e10f) {
        SDFGroupGPU grp = groups[prev_group];
        acc_dist = combineCSG(
            acc_dist, grp_dist, grp.csg_operation, grp.blend_type, grp.blend, grp.shell_distance, grp.shell_mode);
      }
      prev_group = obj.group_id;
      if (prev_group >= 0) {
        grp_dist = 1e10f;
      }
    }

    if (any(greaterThan(brick_min, obj.bbox_max.xyz)) ||
        any(lessThan(brick_max, obj.bbox_min.xyz)))
    {
      if (obj.group_id >= 0 && obj.group_first == 1) {
        grp_dist = 1e10f;
      }
      continue;
    }

    float3 true_min = obj.bbox_min.xyz;
    float3 true_max = obj.bbox_max.xyz;
    float blend_radius = (obj.blend_type != 0) ? obj.blend : 0.0f;
    if (blend_radius > 0.0f) {
      true_min += float3(blend_radius);
      true_max -= float3(blend_radius);
    }
    float3 d_box = max(true_min - sample_pos, sample_pos - true_max);
    float true_dist = length(max(d_box, 0.0f)) + min(max(d_box.x, max(d_box.y, d_box.z)), 0.0f);

    float threshold = blend_radius + brick_half_diag * 2.0f;
    bool pruned = false;

    if (obj.group_id >= 0) {
      if (obj.group_first == 0 && grp_dist < 1e9f) {
        if (obj.csg_operation == 0 || obj.csg_operation == 3 || obj.csg_operation == 4)
          pruned = (true_dist - grp_dist > threshold);
        else if (obj.csg_operation == 1 || obj.csg_operation == 5)
          pruned = (grp_dist + true_dist > threshold);
        else if (obj.csg_operation == 2)
          pruned = (grp_dist - true_dist > threshold);
      }
    }
    else {
      if (acc_dist < 1e9f) {
        if (obj.csg_operation == 0 || obj.csg_operation == 3 || obj.csg_operation == 4)
          pruned = (true_dist - acc_dist > threshold);
        else if (obj.csg_operation == 1 || obj.csg_operation == 5)
          pruned = (acc_dist + true_dist > threshold);
        else if (obj.csg_operation == 2)
          pruned = (acc_dist - true_dist > threshold);
      }
    }

    if (pruned) {
      continue;
    }

    float3 local_pos = (obj.inverse_matrix * float4(sample_pos - obj.position.xyz, 1.0f)).xyz;
    float dist = evalSDFPrimitive(local_pos, obj);

    if (obj.group_id >= 0) {
      if (obj.group_first == 1) {
        grp_dist = dist;
      }
      else {
        if (grp_dist >= 1e9f) {
          if (obj.csg_operation != 1 && obj.csg_operation != 2 && obj.csg_operation != 5) {
            grp_dist = dist;
          }
        }
        else {
          grp_dist = combineCSG(
              grp_dist, dist, obj.csg_operation, obj.blend_type, obj.blend, obj.shell_distance, obj.shell_mode);
        }
      }
    }
    else {
      if (acc_dist >= 1e9f) {
        if (obj.csg_operation != 1 && obj.csg_operation != 2 && obj.csg_operation != 5) {
          acc_dist = dist;
        }
      }
      else {
        acc_dist = combineCSG(
            acc_dist, dist, obj.csg_operation, obj.blend_type, obj.blend, obj.shell_distance, obj.shell_mode);
      }
    }
  }

  if (prev_group >= 0 && grp_dist < 1e10f) {
    SDFGroupGPU grp = groups[prev_group];
    acc_dist = combineCSG(
        acc_dist, grp_dist, grp.csg_operation, grp.blend_type, grp.blend, grp.shell_distance, grp.shell_mode);
  }

  return acc_dist;
}

void main()
{
  int3 brick;
  bool valid_brick = true;

  if (incremental_mode == 1) {
    brick = dirty_brick_min + int3(gl_GlobalInvocationID);
    if (any(greaterThanEqual(brick, dirty_brick_max))) {
      valid_brick = false;
    }
    if (any(lessThan(brick, int3(0))) || any(greaterThanEqual(brick, grid_resolution.xyz))) {
      valid_brick = false;
    }
  }
  else {
    brick = int3(gl_GlobalInvocationID);
    if (any(greaterThanEqual(brick, grid_resolution.xyz))) {
      valid_brick = false;
    }
  }

  int3 wg_brick_base;
  if (incremental_mode == 1) {
    wg_brick_base = dirty_brick_min + int3(gl_WorkGroupID) * 4;
  }
  else {
    wg_brick_base = int3(gl_WorkGroupID) * 4;
  }

  float expand = brick_half_diag;
  float3 wg_min_bound = atlas_origin + float3(wg_brick_base) * float(BRICK_SIZE) * voxel_size;
  float3 wg_max_bound = atlas_origin + float3(wg_brick_base + 4) * float(BRICK_SIZE) * voxel_size;
  wg_min_bound -= float3(expand);
  wg_max_bound += float3(expand);

  uint local_idx = gl_LocalInvocationIndex;
  if (local_idx < MAX_MASK_WORDS) {
    wg_mask[local_idx] = 0u;
  }
  if (local_idx == 0) {
    wg_flags = 0u;
  }
  barrier();

  bool use_bvh = (bvh_node_count > 0 && object_count <= MAX_MASK_WORDS * 32);

  if (local_idx == 0) {
    bool wg_dirty = false;
    if (incremental_mode == 1 && has_dirty_flags == 1) {
      for (int i = 0; i < object_count && !wg_dirty; i++) {
        SDFObjectGPU obj = objects[i];
        if (any(greaterThan(wg_min_bound, obj.bbox_max.xyz)) ||
            any(lessThan(wg_max_bound, obj.bbox_min.xyz))) {
          continue;
        }
        if (dirty_flags[i] != 0) {
          wg_dirty = true;
        }
      }
    } else {
      wg_dirty = true;
    }

    if (wg_dirty) {
      atomicOr(wg_flags, 1u);
    }

    if (wg_dirty && use_bvh) {
      int stack[BVH_MAX_STACK];
      int sp = 0;
      stack[sp++] = 0;

      while (sp > 0) {
        int node_idx = stack[--sp];
        BVHNodeGPU node = bvh_nodes[node_idx];

        if (!aabb_overlap(wg_min_bound, wg_max_bound, node.min_and_left.xyz, node.max_and_right.xyz)) {
          continue;
        }

        int left = bvh_decode_int(node.min_and_left.w);
        int right = bvh_decode_int(node.max_and_right.w);

        if (left == -1) {
          if (right < MAX_MASK_WORDS * 32) {
            wg_mask[right >> 5] |= (1u << (right & 31));
          }
        }
        else {
          if (sp < BVH_MAX_STACK - 1) {
            stack[sp++] = left;
            stack[sp++] = right;
          }
        }
      }

      bool empty = true;
      for (int i = 0; i < MAX_MASK_WORDS; i++) {
        if (wg_mask[i] != 0u) {
          empty = false;
          break;
        }
      }
      if (empty) {
        atomicOr(wg_flags, 2u);
      }
    }
  }
  barrier();

  bool wg_dirty = (wg_flags & 1u) != 0u;
  bool wg_empty = (wg_flags & 2u) != 0u;

  if (!wg_dirty) {
    return;
  }

  if (use_bvh && wg_empty) {
    if (valid_brick) {
      if (incremental_mode == 1) {
        int old_slot = imageLoad(indirection_tex, brick).r;
        if (old_slot >= 0) {
          /* Brick was active, but is now empty. Just mark it empty.
           * We do not reclaim the slot memory yet. */
          imageStore(indirection_tex, brick, int4(-1, 0, 0, 0));
        }
      } else {
        imageStore(indirection_tex, brick, int4(-1, 0, 0, 0));
      }
    }
    return;
  }

  if (!valid_brick) {
    return;
  }

  float3 brick_center = atlas_origin +
                         (float3(brick) * float(BRICK_SIZE) + float(BRICK_SIZE) * 0.5f) *
                             voxel_size;

  float3 brick_min = brick_center - float3(expand);
  float3 brick_max = brick_center + float3(expand);

  float acc_dist = evaluateSceneAt(brick_center, brick_min, brick_max, use_bvh);

  /* Multi-sample sign-change detection for borderline bricks.
   * Chamfer/round blend operations have Lipschitz constant up to sqrt(2),
   * so the center distance can overestimate by that factor. Check opposite
   * corners for sign changes only in the borderline zone. */
  float surface_threshold = brick_half_diag;
  bool is_surface = (abs(acc_dist) < surface_threshold);
  if (!is_surface && abs(acc_dist) < surface_threshold * 3.0f) {
    float hext = float(BRICK_SIZE) * 0.5f * voxel_size;
    for (int c = 0; c < 8 && !is_surface; c++) {
      float3 corner = float3(
          ((c & 1) != 0) ? hext : -hext,
          ((c & 2) != 0) ? hext : -hext,
          ((c & 4) != 0) ? hext : -hext);
      float dc = evaluateSceneAt(brick_center + corner, brick_min, brick_max, use_bvh);
      if (sign(dc) != sign(acc_dist)) {
        is_surface = true;
      }
    }
  }

  if (is_surface) {
    int slot;
    if (incremental_mode == 1) {
      int old_slot = imageLoad(indirection_tex, brick).r;
      if (old_slot >= 0) {
        slot = old_slot;
      }
      else {
        slot = int(atomicAdd(brick_counter.next_slot, 1u));
      }
      imageStore(indirection_tex, brick, int4(slot, 0, 0, 0));
      uint dirty_idx = atomicAdd(brick_counter.count, 1u);
      if (int(dirty_idx) < max_active_bricks) {
        active_bricks[dirty_idx].coord = int4(brick, slot);
      }
    }
    else {
      slot = int(atomicAdd(brick_counter.count, 1u));
      if (slot >= max_active_bricks) {
        return;
      }
      imageStore(indirection_tex, brick, int4(slot, 0, 0, 0));
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
