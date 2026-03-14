/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* SDF bake: one workgroup per brick, 12x12 threads cover XY, loop Z. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_bake)

#include "sdf_lib.glsl"

#define BRICK_SIZE 8
#define BRICK_STORAGE 12
#define MAX_CANDIDATES 128

/* Per-object data cached in shared memory. */
struct SharedObj {
  float4x4 inverse_matrix;
  float4 position;
  float4 sdf_size;
  float4 color;
  float bevel;
  float blend;
  int sdf_type;
  int blend_type;
  int csg_operation;
  float shell_distance;

  int modifier_start;
  int modifier_count;
  int group_id;
  int group_first;
  int group_order;
  float4 box_corners;
  float4 box_edges;
  int4 box_modes;
};

float evalSDFPrimitiveSh(float3 local_pos, SharedObj obj)
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

/* Shared candidate list and object cache (BVH traversal by thread 0). */
shared int shared_candidates[MAX_CANDIDATES];
shared int shared_num_candidates;
shared SharedObj shared_objs[MAX_CANDIDATES];

void main()
{
  int brick_idx = int(gl_WorkGroupID.x) + int(gl_WorkGroupID.y) * dispatch_width;
  if (brick_idx >= int(brick_counter.count)) {
    return;
  }

  int4 brick_data = active_bricks[brick_idx].coord;
  int3 brick = brick_data.xyz;
  int slot = brick_data.w;

  int bpa = bricks_per_axis;
  int3 slot_block = int3(slot % bpa, (slot / bpa) % bpa, slot / (bpa * bpa));
  int3 slot_origin = slot_block * BRICK_STORAGE;

  /* Per-brick AABB for culling (object AABBs already include blend padding). */
  float bhd = float(BRICK_SIZE) * voxel_size * 0.866025f;
  float candidate_expand = bhd;
  float3 brick_min = atlas_origin + (float3(brick * BRICK_SIZE) - 2.0f) * voxel_size -
                      float3(candidate_expand);
  float3 brick_max = atlas_origin + (float3(brick * BRICK_SIZE + BRICK_SIZE) + 2.0f) * voxel_size +
                      float3(candidate_expand);

  /* Thread 0 collects candidates for the workgroup. */
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
    }
  }
  barrier();

  /* Cooperatively load candidate objects into shared memory. */
  int num_candidates = shared_num_candidates;
  uint tid = gl_LocalInvocationIndex;
  for (int c = int(tid); c < num_candidates; c += 144) {
    int i = shared_candidates[c];
    SDFObjectGPU obj = objects[i];
    shared_objs[c].inverse_matrix = obj.inverse_matrix;
    shared_objs[c].position = obj.position;
    shared_objs[c].sdf_size = obj.sdf_size;
    shared_objs[c].color = obj.color;
    shared_objs[c].bevel = obj.bevel;
    shared_objs[c].blend = obj.blend;
    shared_objs[c].sdf_type = obj.sdf_type;
    shared_objs[c].blend_type = obj.blend_type;
    shared_objs[c].csg_operation = obj.csg_operation;
    shared_objs[c].shell_distance = obj.shell_distance;

    shared_objs[c].modifier_start = obj.modifier_start;
    shared_objs[c].modifier_count = obj.modifier_count;
    shared_objs[c].group_id = obj.group_id;
    shared_objs[c].group_first = obj.group_first;
    shared_objs[c].group_order = obj.group_order;
    shared_objs[c].box_corners = obj.box_corners;
    shared_objs[c].box_edges = obj.box_edges;
    shared_objs[c].box_modes = obj.box_modes;
  }
  barrier();

  int2 local_xy = int2(gl_LocalInvocationID.xy);

  for (int lz = 0; lz < BRICK_STORAGE; lz++) {
    int3 local_voxel = int3(local_xy, lz);

    /* World position: -2 offset for overlap border, stored at voxel corners. */
    float3 world_pos = atlas_origin +
                       float3(brick * BRICK_SIZE + local_voxel - int3(2)) * voxel_size;

    float acc_dist = 1e10f;
    float3 acc_color = float3(0.0f);

    /* Group-aware sequential evaluation. */
    for (int g = 0; g < group_count; g++) {
      SDFGroupGPU grp = groups[g];
      float grp_dist = 1e10f;
      float3 grp_color = float3(0.0f);

      for (int c = 0; c < num_candidates; c++) {
        SharedObj sobj = shared_objs[c];
        if (sobj.group_id != g) {
          continue;
        }

        float3 local_pos = (sobj.inverse_matrix * float4(world_pos - sobj.position.xyz, 1.0f)).xyz;
        float dist = evalSDFPrimitiveSh(local_pos, sobj);

        if (sobj.group_first == 1) {
          grp_dist = dist;
          grp_color = sobj.color.rgb;
        }
        else {
          float k = sobj.blend;
          int bt = sobj.blend_type;
          int op = sobj.csg_operation;

          float new_dist = combineCSG(grp_dist, dist, op, bt, k, sobj.shell_distance);

          float h = csgColorWeight(grp_dist, dist, op, bt, k, sobj.shell_distance);
          grp_color = mix(grp_color, sobj.color.rgb, h);

          grp_dist = new_dist;
        }
      }

      if (grp_dist >= 1e10f) {
        continue;
      }

      grp_color *= grp.color.rgb;

      if (g == 0) {
        acc_dist = grp_dist;
        acc_color = grp_color;
      }
      else {
        float new_dist = combineCSG(
            acc_dist, grp_dist, grp.csg_operation, grp.blend_type, grp.blend, grp.shell_distance);

        float h = csgColorWeight(
            acc_dist, grp_dist, grp.csg_operation, grp.blend_type, grp.blend, grp.shell_distance);
        acc_color = mix(acc_color, grp_color, h);

        acc_dist = new_dist;
      }
    }

    /* Ungrouped objects. */
    for (int c = 0; c < num_candidates; c++) {
      SharedObj sobj = shared_objs[c];
      if (sobj.group_id != -1) {
        continue;
      }

      float3 local_pos = (sobj.inverse_matrix * float4(world_pos - sobj.position.xyz, 1.0f)).xyz;
      float dist = evalSDFPrimitiveSh(local_pos, sobj);

      {
        float k = sobj.blend;
        int bt = sobj.blend_type;
        int op = sobj.csg_operation;

        float new_dist = combineCSG(acc_dist, dist, op, bt, k, sobj.shell_distance);

        float h = csgColorWeight(acc_dist, dist, op, bt, k, sobj.shell_distance);
        acc_color = mix(acc_color, sobj.color.rgb, h);

        acc_dist = new_dist;
      }
    }

    int3 atlas_coord = slot_origin + local_voxel;
    imageStore(compact_atlas, atlas_coord, float4(acc_dist, acc_color));
  }
}
