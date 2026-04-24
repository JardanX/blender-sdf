/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* SDF classify: one workgroup per chunk, dense local brick table per chunk.
 * Chunks are sparse in world space, looked up later through a hash table. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_classify)

#include "sdf_lib.glsl"

#define BRICK_SIZE 8
#define CHUNK_BRICK_RES 16

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
  int chunk_idx = int(gl_WorkGroupID.x) + int(gl_WorkGroupID.y) * dispatch_width;
  if (chunk_idx >= chunk_count) {
    return;
  }

  ChunkPageGPU chunk = chunk_pages[chunk_idx];
  int2 local_xy = int2(gl_LocalInvocationID.xy);
  int chunk_offset = chunk_idx * (CHUNK_BRICK_RES * CHUNK_BRICK_RES * CHUNK_BRICK_RES);

  for (int lz = 0; lz < CHUNK_BRICK_RES; lz++) {
    int3 local_brick = int3(local_xy, lz);
    int flat_index = local_brick.x + local_brick.y * CHUNK_BRICK_RES +
                     local_brick.z * CHUNK_BRICK_RES * CHUNK_BRICK_RES;
    int3 brick = chunk.coord.xyz * CHUNK_BRICK_RES + local_brick;

    float3 brick_center =
        (float3(brick * BRICK_SIZE) + float(BRICK_SIZE) * 0.5f) * voxel_size;

    float expand = brick_half_diag;
    float3 brick_min = brick_center - float3(expand);
    float3 brick_max = brick_center + float3(expand);

    #define MAX_MASK_WORDS 64
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

    float acc_dist = 1e10f;
    int prev_group = -2;
    float grp_dist = 1e10f;

    for (int i = 0; i < object_count; i++) {
      SDFObjectGPU obj = objects[i];
      bool has_mask = !use_bvh || ((mask[i >> 5] & (1u << (i & 31))) != 0u);

      if (!has_mask && obj.csg_operation != SDF_CSG_OP_INTERSECT) {
        continue;
      }

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

      if (obj.csg_operation != SDF_CSG_OP_INTERSECT &&
          (any(greaterThan(brick_min, obj.bbox_max.xyz)) ||
           any(lessThan(brick_max, obj.bbox_min.xyz))))
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

    if (prev_group >= 0 && grp_dist < 1e10f) {
      SDFGroupGPU grp = groups[prev_group];
      acc_dist = combineCSG(
          acc_dist, grp_dist, grp.csg_operation, grp.blend_type, grp.blend, grp.shell_distance);
    }

    int out_slot = -1;
    if (abs(acc_dist) <= brick_half_diag) {
      uint slot = atomicAdd(brick_counter.count, 1u);
      if (slot < uint(max_active_bricks)) {
        out_slot = int(slot);
        active_bricks[slot].coord = int4(brick, out_slot);
      }
      else {
        atomicMax(brick_counter.next_slot, 1u);
      }
    }
    else if (acc_dist < -brick_half_diag) {
      out_slot = -2;
    }

    chunk_bricks[chunk_offset + flat_index] = out_slot;
  }
}
