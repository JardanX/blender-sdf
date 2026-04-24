/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* SDF bake: one workgroup per active world-space brick, 12x12 threads cover XY,
 * loop Z. Streaming evaluation: candidates processed in batches through shared
 * memory. No hard limit on candidate count — overflow falls back to linear scan. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_bake)

#include "sdf_lib.glsl"

#define BRICK_SIZE 8
#define BRICK_STORAGE 12

/* Streaming batch size (shared memory budget, not a correctness limit). */
#define BATCH_SIZE 64

/* BVH candidate index buffer. If exceeded, falls back to linear scan. */
#define CANDIDATE_BUF_SIZE 2048

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

/* Finalize a completed group into the scene accumulator. */
void finalizeGroup(int group_id,
                   float grp_dist,
                   float3 grp_color,
                   inout float acc_dist,
                   inout float3 acc_color)
{
  if (group_id < 0 || grp_dist >= 1e10f) {
    return;
  }
  SDFGroupGPU grp = groups[group_id];
  float3 tinted = grp_color * grp.color.rgb;
  float new_dist = combineCSG(
      acc_dist, grp_dist, grp.csg_operation, grp.blend_type, grp.blend, grp.shell_distance);
  float h = csgColorWeight(
      acc_dist, grp_dist, grp.csg_operation, grp.blend_type, grp.blend, grp.shell_distance);
  acc_color = mix(acc_color, tinted, h);
  acc_dist = new_dist;
}

/* BVH candidate indices. */
shared int shared_candidates[CANDIDATE_BUF_SIZE];
shared int shared_num_candidates;
shared int shared_overflow;

/* Lipschitz pruning cache. */
shared float shared_center_dist[CANDIDATE_BUF_SIZE];

/* Streaming batch cache. */
shared SharedObj shared_batch[BATCH_SIZE];

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

  /* Per-brick AABB for BVH culling. */
  float bhd = float(BRICK_SIZE) * voxel_size * 0.866025f;
  float candidate_expand = bhd;
  float3 brick_min = (float3(brick * BRICK_SIZE) - 2.0f) * voxel_size - float3(candidate_expand);
  float3 brick_max =
      (float3(brick * BRICK_SIZE + BRICK_SIZE) + 2.0f) * voxel_size + float3(candidate_expand);

  uint tid = gl_LocalInvocationIndex;

  /* Thread 0: collect candidates via BVH traversal. */
  if (tid == 0u) {
    shared_num_candidates = 0;
    shared_overflow = 0;

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
          if (shared_num_candidates < CANDIDATE_BUF_SIZE) {
            shared_candidates[shared_num_candidates++] = right;
          }
          else {
            shared_overflow = 1;
          }
        }
        else {
          if (sp < BVH_MAX_STACK - 1) {
            stack[sp++] = left;
            stack[sp++] = right;
          }
          else {
            shared_overflow = 1;
          }
        }
      }

      /* Sort candidates by object index for correct CSG evaluation order. */
      if (shared_overflow == 0) {
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
    }
    else {
      for (int i = 0; i < object_count; i++) {
        SDFObjectGPU obj = objects[i];

        if (obj.csg_operation != SDF_CSG_OP_INTERSECT &&
            (any(greaterThan(brick_min, obj.bbox_max.xyz)) ||
             any(lessThan(brick_max, obj.bbox_min.xyz))))
        {
          continue;
        }

        if (shared_num_candidates < CANDIDATE_BUF_SIZE) {
          shared_candidates[shared_num_candidates++] = i;
        }
        else {
          shared_overflow = 1;
        }
      }
    }

    if (shared_overflow == 0) {
      for (int i = 0; i < object_count; i++) {
        SDFObjectGPU obj = objects[i];
        if (obj.csg_operation != SDF_CSG_OP_INTERSECT) {
          continue;
        }

        bool already_present = false;
        for (int c = 0; c < shared_num_candidates; c++) {
          if (shared_candidates[c] == i) {
            already_present = true;
            break;
          }
        }

        if (already_present) {
          continue;
        }

        if (shared_num_candidates < CANDIDATE_BUF_SIZE) {
          shared_candidates[shared_num_candidates++] = i;
        }
        else {
          shared_overflow = 1;
          break;
        }
      }

      if (shared_overflow == 0) {
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
    }
  }
  barrier();

  /* Lipschitz pruning: cooperatively evaluate candidates at brick center,
   * then thread 0 prunes those provably outside blend influence. */
  if (shared_overflow == 0 && shared_num_candidates > 1) {
    float3 brick_center = (float3(brick * BRICK_SIZE) + float(BRICK_SIZE) * 0.5f) * voxel_size;
    float two_R = 2.0f * float(BRICK_STORAGE) * voxel_size * 0.866025f;

    for (int c = int(tid); c < shared_num_candidates; c += 144) {
      int i = shared_candidates[c];
      SDFObjectGPU obj = objects[i];
      float3 lp = (obj.inverse_matrix * float4(brick_center - obj.position.xyz, 1.0f)).xyz;
      SDFPrimitiveData pd;
      pd.sdf_type = obj.sdf_type;
      pd.size = obj.sdf_size.xyz;
      pd.bevel = obj.bevel;
      pd.box_corners = obj.box_corners;
      pd.box_edges = obj.box_edges;
      pd.box_modes = obj.box_modes;
      pd.modifier_start = obj.modifier_start;
      pd.modifier_count = obj.modifier_count;
      shared_center_dist[c] = evalObjectSDF(pd, lp);
    }
    barrier();

    if (tid == 0u) {
      float acc_center = 1e10f;
      for (int g = 0; g < group_count; g++) {
        float grp_center = 1e10f;
        for (int c = 0; c < shared_num_candidates; c++) {
          SDFObjectGPU obj = objects[shared_candidates[c]];
          if (obj.group_id != g) continue;
          float d = shared_center_dist[c];
          if (obj.group_first == 1) {
            grp_center = d;
            continue;
          }
          float threshold = obj.blend + two_R;
          bool pruned = false;
          if (obj.csg_operation == SDF_CSG_OP_UNION) pruned = (d - grp_center > threshold);
          else if (obj.csg_operation == SDF_CSG_OP_SUBTRACT) pruned = (grp_center + d > threshold);
          else if (obj.csg_operation == SDF_CSG_OP_INTERSECT) pruned = (grp_center - d > threshold);
          
          if (pruned) shared_center_dist[c] = -1e20f;
          else grp_center = combineCSG(grp_center, d, obj.csg_operation, obj.blend_type, obj.blend, obj.shell_distance);
        }
        if (grp_center < 1e10f) {
          SDFGroupGPU grp = groups[g];
          if (g == 0) acc_center = grp_center;
          else acc_center = combineCSG(acc_center, grp_center, grp.csg_operation, grp.blend_type, grp.blend, grp.shell_distance);
        }
      }
      for (int c = 0; c < shared_num_candidates; c++) {
        SDFObjectGPU obj = objects[shared_candidates[c]];
        if (obj.group_id != -1) continue;
        float d = shared_center_dist[c];
        float threshold = obj.blend + two_R;
        bool pruned = false;
        if (acc_center < 1e9f) {
          if (obj.csg_operation == SDF_CSG_OP_UNION) pruned = (d - acc_center > threshold);
          else if (obj.csg_operation == SDF_CSG_OP_SUBTRACT) pruned = (acc_center + d > threshold);
          else if (obj.csg_operation == SDF_CSG_OP_INTERSECT) pruned = (acc_center - d > threshold);
        }
        if (pruned) shared_center_dist[c] = -1e20f;
        else acc_center = combineCSG(acc_center, d, obj.csg_operation, obj.blend_type, obj.blend, obj.shell_distance);
      }

      int new_count = 0;
      for (int c = 0; c < shared_num_candidates; c++) {
        if (shared_center_dist[c] > -1e19f) {
          shared_candidates[new_count++] = shared_candidates[c];
        }
      }
      shared_num_candidates = new_count;
    }
    barrier();
  }

  /* Overflow: iterate all objects in sorted order (no BVH filtering). */
  int total_count = (shared_overflow == 1) ? object_count : shared_num_candidates;
  int2 local_xy = int2(gl_LocalInvocationID.xy);

  /* Z-outer: each voxel layer is fully evaluated before the next. */
  for (int lz = 0; lz < BRICK_STORAGE; lz++) {
    int3 local_voxel = int3(local_xy, lz);
    float3 world_pos = float3(brick * BRICK_SIZE + local_voxel - int3(2)) * voxel_size;

    float acc_dist = 1e10f;
    float3 acc_color = float3(0.0f);
    float grp_dist = 1e10f;
    float3 grp_color = float3(0.0f);
    int prev_group = -2;

    /* Stream candidates in batches through shared memory. */
    for (int batch_start = 0; batch_start < total_count; batch_start += BATCH_SIZE) {
      int batch_count = min(BATCH_SIZE, total_count - batch_start);

      /* Cooperative load. */
      for (int c = int(tid); c < batch_count; c += 144) {
        int i = (shared_overflow == 1) ? (batch_start + c) :
                                         shared_candidates[batch_start + c];
        SDFObjectGPU obj = objects[i];
        shared_batch[c].inverse_matrix = obj.inverse_matrix;
        shared_batch[c].position = obj.position;
        shared_batch[c].sdf_size = obj.sdf_size;
        shared_batch[c].color = obj.color;
        shared_batch[c].bevel = obj.bevel;
        shared_batch[c].blend = obj.blend;
        shared_batch[c].sdf_type = obj.sdf_type;
        shared_batch[c].blend_type = obj.blend_type;
        shared_batch[c].csg_operation = obj.csg_operation;
        shared_batch[c].shell_distance = obj.shell_distance;

        shared_batch[c].modifier_start = obj.modifier_start;
        shared_batch[c].modifier_count = obj.modifier_count;
        shared_batch[c].group_id = obj.group_id;
        shared_batch[c].group_first = obj.group_first;
        shared_batch[c].group_order = obj.group_order;
        shared_batch[c].box_corners = obj.box_corners;
        shared_batch[c].box_edges = obj.box_edges;
        shared_batch[c].box_modes = obj.box_modes;
      }
      barrier();

        /* Evaluate batch with single-pass group tracking. */
      for (int c = 0; c < batch_count; c++) {
        SharedObj sobj = shared_batch[c];

        /* Group transition: finalize previous group when entering a new one. */
        if (sobj.group_id != prev_group) {
          finalizeGroup(prev_group, grp_dist, grp_color, acc_dist, acc_color);
          prev_group = sobj.group_id;
          if (prev_group >= 0) {
            grp_dist = 1e10f;
            grp_color = float3(0.0f);
          }
        }

        float3 local_pos = (sobj.inverse_matrix * float4(world_pos - sobj.position.xyz, 1.0f)).xyz;
        float dist = evalSDFPrimitiveSh(local_pos, sobj);

        if (sobj.group_id >= 0) {
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
        else {
          float k = sobj.blend;
          int bt = sobj.blend_type;
          int op = sobj.csg_operation;

          float new_dist = combineCSG(acc_dist, dist, op, bt, k, sobj.shell_distance);
          float h = csgColorWeight(acc_dist, dist, op, bt, k, sobj.shell_distance);
          acc_color = mix(acc_color, sobj.color.rgb, h);
          acc_dist = new_dist;
        }
      }
      barrier();
    }

    /* Finalize last group. */
    finalizeGroup(prev_group, grp_dist, grp_color, acc_dist, acc_color);

    int3 atlas_coord = slot_origin + local_voxel;
    imageStore(compact_atlas, atlas_coord, float4(acc_dist, acc_color));
  }
}
