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
#define MAX_COLLECT 1024

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

/* Pre-pruning candidate list (larger buffer to avoid silent drops). */
shared int shared_candidates[MAX_COLLECT];
shared int shared_num_candidates;
shared int shared_any_dirty;

/* Post-pruning object cache (fits in shared memory at 128 entries). */
shared SharedObj shared_objs[MAX_CANDIDATES];
shared float3 shared_bb_min[MAX_CANDIDATES];
shared float3 shared_bb_max[MAX_CANDIDATES];

/* Lipschitz pruning: center distances for the larger pre-pruning buffer. */
shared float shared_center_dist[MAX_COLLECT];

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
          if (shared_num_candidates < MAX_COLLECT) {
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

        if (shared_num_candidates < MAX_COLLECT) {
          shared_candidates[shared_num_candidates++] = i;
        }
      }
    }

    /* Per-brick dirty check. */
    shared_any_dirty = 0;
    if (has_dirty_flags == 1) {
      for (int c = 0; c < shared_num_candidates; c++) {
        if (dirty_flags[shared_candidates[c]] != 0) {
          shared_any_dirty = 1;
          break;
        }
      }
    }
    else {
      shared_any_dirty = 1;
    }

  }
  barrier();

  if (shared_any_dirty == 0) {
    return;
  }

  /* Lipschitz pruning: cooperatively evaluate all candidates at brick center,
   * then thread 0 prunes those provably outside blend influence.
   * |f1(p) - f2(p)| >= k + 2R => operator reduces to one operand.
   * (Barbier et al., Eurographics 2025). */
  int num_candidates = shared_num_candidates;
  uint tid = gl_LocalInvocationIndex;

  if (num_candidates > 1) {
    float3 brick_center = atlas_origin +
                          (float3(brick * BRICK_SIZE) + float(BRICK_SIZE) * 0.5f) * voxel_size;
    float two_R = 2.0f * float(BRICK_STORAGE) * voxel_size * 0.866025f;

    /* All threads cooperatively evaluate candidates at brick center. */
    for (int c = int(tid); c < num_candidates; c += 144) {
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

    /* Thread 0: sequential Lipschitz pruning from shared memory. */
    if (tid == 0u) {
      float acc_center = 1e10f;

      for (int g = 0; g < group_count; g++) {
        float grp_center = 1e10f;
        for (int c = 0; c < num_candidates; c++) {
          SDFObjectGPU obj = objects[shared_candidates[c]];
          if (obj.group_id != g) {
            continue;
          }
          float d = shared_center_dist[c];
          if (obj.group_first == 1) {
            grp_center = d;
            continue;
          }
          float threshold = obj.blend + two_R;
          bool pruned = false;
          if (obj.csg_operation == SDF_CSG_OP_UNION) {
            pruned = (d - grp_center > threshold);
          }
          else if (obj.csg_operation == SDF_CSG_OP_SUBTRACT) {
            pruned = (grp_center + d > threshold);
          }
          else if (obj.csg_operation == SDF_CSG_OP_INTERSECT) {
            pruned = (grp_center - d > threshold);
          }
          if (pruned) {
            shared_center_dist[c] = -1e20f;
          }
          else {
            grp_center = combineCSG(
                grp_center, d, obj.csg_operation, obj.blend_type, obj.blend, obj.shell_distance);
          }
        }
        if (grp_center >= 1e10f) {
          continue;
        }
        SDFGroupGPU grp = groups[g];
        if (g == 0) {
          acc_center = grp_center;
        }
        else {
          acc_center = combineCSG(
              acc_center, grp_center, grp.csg_operation, grp.blend_type, grp.blend,
              grp.shell_distance);
        }
      }

      for (int c = 0; c < num_candidates; c++) {
        SDFObjectGPU obj = objects[shared_candidates[c]];
        if (obj.group_id != -1) {
          continue;
        }
        float d = shared_center_dist[c];
        float threshold = obj.blend + two_R;
        bool pruned = false;
        if (acc_center < 1e9f) {
          if (obj.csg_operation == SDF_CSG_OP_UNION) {
            pruned = (d - acc_center > threshold);
          }
          else if (obj.csg_operation == SDF_CSG_OP_SUBTRACT) {
            pruned = (acc_center + d > threshold);
          }
          else if (obj.csg_operation == SDF_CSG_OP_INTERSECT) {
            pruned = (acc_center - d > threshold);
          }
        }
        if (pruned) {
          shared_center_dist[c] = -1e20f;
        }
        else {
          acc_center = combineCSG(
              acc_center, d, obj.csg_operation, obj.blend_type, obj.blend, obj.shell_distance);
        }
      }

      /* Compact: remove pruned candidates, keep center distances in sync. */
      int new_count = 0;
      for (int c = 0; c < num_candidates; c++) {
        if (shared_center_dist[c] > -1e19f) {
          shared_candidates[new_count] = shared_candidates[c];
          shared_center_dist[new_count] = shared_center_dist[c];
          new_count++;
        }
      }

      /* Priority truncation: if still > MAX_CANDIDATES, find the distance
       * threshold that keeps exactly MAX_CANDIDATES entries, then drop the
       * rest while preserving original object-index order (critical for CSG). */
      if (new_count > MAX_CANDIDATES) {
        /* Find the MAX_CANDIDATES-th smallest |distance| as the cut threshold.
         * Use a single pass: track the k-th smallest by repeated scan. */
        int to_drop = new_count - MAX_CANDIDATES;
        for (int d = 0; d < to_drop; d++) {
          /* Find the candidate with the largest |center_dist| and mark it. */
          float worst_d = -1.0f;
          int worst_c = -1;
          for (int c = 0; c < new_count; c++) {
            if (shared_center_dist[c] > -1e19f) {
              float ad = abs(shared_center_dist[c]);
              if (ad > worst_d) {
                worst_d = ad;
                worst_c = c;
              }
            }
          }
          if (worst_c >= 0) {
            shared_center_dist[worst_c] = -1e20f;
          }
        }
        /* Re-compact, preserving original order. */
        int final_count = 0;
        for (int c = 0; c < new_count; c++) {
          if (shared_center_dist[c] > -1e19f) {
            shared_candidates[final_count] = shared_candidates[c];
            final_count++;
          }
        }
        new_count = final_count;
      }

      shared_num_candidates = new_count;
    }
    barrier();
    num_candidates = shared_num_candidates;
  }

  /* Cooperatively load candidate objects into shared memory. */
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

    shared_bb_min[c] = obj.bbox_min.xyz;
    shared_bb_max[c] = obj.bbox_max.xyz;
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

    /* Group-aware sequential evaluation with per-voxel AABB culling.
     * Object AABBs (already expanded by blend) are read from SSBO — coalesced across workgroup.
     * Skips the expensive matrix multiply + SDF eval for objects far from this voxel. */
    for (int g = 0; g < group_count; g++) {
      SDFGroupGPU grp = groups[g];
      float grp_dist = 1e10f;
      float3 grp_color = float3(0.0f);

      for (int c = 0; c < num_candidates; c++) {
        SharedObj sobj = shared_objs[c];
        if (sobj.group_id != g) {
          continue;
        }

        /* Per-voxel AABB cull (bbox cached in shared memory). */
        if (any(greaterThan(world_pos, shared_bb_max[c])) ||
            any(lessThan(world_pos, shared_bb_min[c])))
        {
          if (sobj.group_first == 1) {
            grp_dist = 1e10f;
            grp_color = sobj.color.rgb;
          }
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

    /* Ungrouped objects with per-voxel AABB culling. */
    for (int c = 0; c < num_candidates; c++) {
      SharedObj sobj = shared_objs[c];
      if (sobj.group_id != -1) {
        continue;
      }

      if (any(greaterThan(world_pos, shared_bb_max[c])) ||
          any(lessThan(world_pos, shared_bb_min[c])))
      {
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
