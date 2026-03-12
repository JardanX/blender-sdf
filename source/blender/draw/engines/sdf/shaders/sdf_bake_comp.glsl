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

/* Compact per-object data cached in shared memory.
 * Only the fields needed for SDF evaluation — avoids re-reading the
 * full SDFObjectGPU struct from SSBO for every voxel. */
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
  int obj_index;
  int modifier_start;
  int modifier_count;
  int group_id;
  int group_first;
  int group_order;
  float4 box_corners;
  float4 box_edges;
  int4 box_modes;
};

/** Evaluate the actual SDF primitive (reads from shared memory cache). */
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

/* Shared candidate list and object cache: BVH traversal done once per
 * workgroup by thread 0, object data loaded cooperatively by all threads.
 * Eliminates redundant SSBO reads across 144 threads x 12 Z iterations. */
shared int shared_candidates[MAX_CANDIDATES];
shared int shared_num_candidates;
shared SharedObj shared_objs[MAX_CANDIDATES];

void main()
{
  /* Active-brick-only dispatch: one workgroup per active brick.
   * 2D dispatch to avoid GL's 65535 workgroup limit per axis.
   * Read active count from SSBO (avoids CPU readback stall between classify and bake). */
  int brick_idx = int(gl_WorkGroupID.x) + int(gl_WorkGroupID.y) * dispatch_width;
  if (brick_idx >= int(brick_counter.count)) {
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
   * Expand by max_blend for smooth union and max_shell_distance so bricks in
   * the shell zone can find base union objects as candidates. */
  float candidate_expand = max_blend + max_shell_distance;
  float3 brick_min = atlas_origin + (float3(brick * BRICK_SIZE) - 2.0f) * voxel_size -
                      float3(candidate_expand);
  float3 brick_max = atlas_origin + (float3(brick * BRICK_SIZE + BRICK_SIZE) + 2.0f) * voxel_size +
                      float3(candidate_expand);

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

  /* Cooperatively load candidate object data into shared memory.
   * Each thread loads one object; if num_candidates > 144 (workgroup size),
   * threads loop. This replaces 144 * 12 = 1728 redundant SSBO reads
   * per brick with at most MAX_CANDIDATES reads total. */
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
    shared_objs[c].obj_index = i;
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

  /* Local thread covers XY, loop over Z. */
  int2 local_xy = int2(gl_LocalInvocationID.xy);

  for (int lz = 0; lz < BRICK_STORAGE; lz++) {
    int3 local_voxel = int3(local_xy, lz);

    /* World-space position: brick_coord * 8 + (local - 2), times voxel_size.
     * The -2 accounts for the 2-voxel overlap border.
     * Values are stored at voxel CORNERS to match the DDA trilinear interpolation. */
    float3 world_pos = atlas_origin +
                       float3(brick * BRICK_SIZE + local_voxel - int3(2)) * voxel_size;

    float acc_dist = 1e10f;
    float3 acc_color = float3(0.0f);
    int acc_obj_id = -1;
    float closest_raw_dist = 1e10f;

    /* Group-aware sequential evaluation.
     * Objects are sorted by group, then by order within group.
     * Each group evaluates its members sequentially (first object = base,
     * subsequent objects apply their per-object CSG). Groups then combine
     * with each other via group-level CSG operations.
     *
     * Ungrouped objects (group_id == -1) fall back to the legacy two-pass
     * approach for backward compatibility during migration. */

    /* Evaluate grouped objects: iterate groups, accumulate within each,
     * then combine group results with the scene. */
    for (int g = 0; g < group_count; g++) {
      SDFGroupGPU grp = groups[g];
      float grp_dist = 1e10f;
      float3 grp_color = float3(0.0f);

      /* Find candidates belonging to this group and evaluate sequentially. */
      for (int c = 0; c < num_candidates; c++) {
        SharedObj sobj = shared_objs[c];
        if (sobj.group_id != g) {
          continue;
        }

        float3 local_pos = (sobj.inverse_matrix * float4(world_pos - sobj.position.xyz, 1.0f)).xyz;
        float dist = evalSDFPrimitiveSh(local_pos, sobj);

        if (dist < closest_raw_dist) {
          closest_raw_dist = dist;
          acc_obj_id = sobj.obj_index;
        }

        if (grp_dist >= 1e9f) {
          /* First object in group is always the base shape — CSG op ignored. */
          grp_dist = dist;
          grp_color = sobj.color.rgb;
        }
        else {
          /* Apply this object's CSG to running group accumulator. */
          float k = sobj.blend;
          int bt = sobj.blend_type;
          int op = sobj.csg_operation;

          float new_dist = combineCSG(grp_dist, dist, op, bt, k, sobj.shell_distance);

          /* Color blending based on CSG operation type. */
          if (op == SDF_CSG_OP_UNION || op == SDF_CSG_OP_SHELL) {
            if (k > 0.0f && bt > 0) {
              float h = clamp(0.5f + 0.5f * (grp_dist - dist) / k, 0.0f, 1.0f);
              grp_color = mix(grp_color, sobj.color.rgb, h);
            }
            else {
              if (dist < grp_dist) {
                grp_color = sobj.color.rgb;
              }
            }
          }
          else if (op == SDF_CSG_OP_SUBTRACT) {
            if (k > 0.0f && bt > 0) {
              float h = clamp(0.5f - 0.5f * (grp_dist + dist) / k, 0.0f, 1.0f);
              grp_color = mix(grp_color, sobj.color.rgb, h);
            }
            else {
              if (grp_dist + dist < 0.0f) {
                grp_color = sobj.color.rgb;
              }
            }
          }
          else if (op == SDF_CSG_OP_INTERSECT) {
            if (k > 0.0f && bt > 0) {
              float h = clamp(0.5f + 0.5f * (dist - grp_dist) / k, 0.0f, 1.0f);
              grp_color = mix(grp_color, sobj.color.rgb, h);
            }
            else {
              if (dist > grp_dist) {
                grp_color = sobj.color.rgb;
              }
            }
          }
          else if (op == SDF_CSG_OP_PUSH) {
            float sub_base = combineCSG(grp_dist, dist, SDF_CSG_OP_SUBTRACT, bt, k, 0.0f);
            if (dist <= sub_base) {
              grp_color = sobj.color.rgb;
            }
          }
          else if (op == SDF_CSG_OP_AVOID) {
            float carved;
            if (k > 0.0f && bt > 0) {
              if (bt == SDF_BLEND_TYPE_SMOOTH) {
                carved = opSmoothSubtraction(grp_dist, dist, k);
              }
              else if (bt == SDF_BLEND_TYPE_CHAMFER) {
                carved = opChamferSubtraction(grp_dist, dist, k);
              }
              else if (bt == SDF_BLEND_TYPE_ROUND) {
                carved = opRoundSubtraction(grp_dist, dist, k);
              }
              else {
                carved = max(dist, -grp_dist);
              }
            }
            else {
              carved = max(dist, -grp_dist);
            }
            if (carved < grp_dist) {
              grp_color = sobj.color.rgb;
            }
          }

          grp_dist = new_dist;
        }
      }

      if (grp_dist >= 1e10f) {
        continue; /* No members hit this brick. */
      }

      /* Combine group result with scene using group-level CSG. */
      if (acc_dist >= 1e9f) {
        acc_dist = grp_dist;
        acc_color = grp_color;
      }
      else {
        float new_dist = combineCSG(
            acc_dist, grp_dist, grp.csg_operation, grp.blend_type, grp.blend, grp.shell_distance);

        /* Inter-group color blending. */
        float gk = grp.blend;
        int gbt = grp.blend_type;
        int gop = grp.csg_operation;

        if (gop == SDF_CSG_OP_UNION || gop == SDF_CSG_OP_SHELL) {
          if (gk > 0.0f && gbt > 0) {
            float h = clamp(0.5f + 0.5f * (acc_dist - grp_dist) / gk, 0.0f, 1.0f);
            acc_color = mix(acc_color, grp_color, h);
          }
          else {
            if (grp_dist < acc_dist) {
              acc_color = grp_color;
            }
          }
        }
        else if (gop == SDF_CSG_OP_SUBTRACT) {
          if (gk > 0.0f && gbt > 0) {
            float h = clamp(0.5f - 0.5f * (acc_dist + grp_dist) / gk, 0.0f, 1.0f);
            acc_color = mix(acc_color, grp_color, h);
          }
          else {
            if (acc_dist + grp_dist < 0.0f) {
              acc_color = grp_color;
            }
          }
        }
        else if (gop == SDF_CSG_OP_INTERSECT) {
          if (gk > 0.0f && gbt > 0) {
            float h = clamp(0.5f + 0.5f * (grp_dist - acc_dist) / gk, 0.0f, 1.0f);
            acc_color = mix(acc_color, grp_color, h);
          }
          else {
            if (grp_dist > acc_dist) {
              acc_color = grp_color;
            }
          }
        }
        else {
          /* Push/Avoid at group level: use group result color if closer. */
          if (grp_dist < acc_dist) {
            acc_color = grp_color;
          }
        }

        acc_dist = new_dist;
      }
    }

    /* Evaluate ungrouped objects (group_id == -1) sequentially.
     * First ungrouped object is the base shape (CSG op ignored),
     * subsequent objects apply their CSG in order (top-to-bottom). */
    for (int c = 0; c < num_candidates; c++) {
      SharedObj sobj = shared_objs[c];
      if (sobj.group_id != -1) {
        continue;
      }

      float3 local_pos = (sobj.inverse_matrix * float4(world_pos - sobj.position.xyz, 1.0f)).xyz;
      float dist = evalSDFPrimitiveSh(local_pos, sobj);

      if (dist < closest_raw_dist) {
        closest_raw_dist = dist;
        acc_obj_id = sobj.obj_index;
      }

      if (acc_dist >= 1e9f) {
        /* First ungrouped object is the base shape — CSG op ignored. */
        acc_color = sobj.color.rgb;
        acc_dist = dist;
      }
      else {
        float k = sobj.blend;
        int bt = sobj.blend_type;
        int op = sobj.csg_operation;

        float new_dist = combineCSG(acc_dist, dist, op, bt, k, sobj.shell_distance);

        /* Color blending based on CSG operation type. */
        if (op == SDF_CSG_OP_UNION || op == SDF_CSG_OP_SHELL) {
          if (k > 0.0f && bt > 0) {
            float h = clamp(0.5f + 0.5f * (acc_dist - dist) / k, 0.0f, 1.0f);
            acc_color = mix(acc_color, sobj.color.rgb, h);
          }
          else {
            if (dist < acc_dist) {
              acc_color = sobj.color.rgb;
            }
          }
        }
        else if (op == SDF_CSG_OP_SUBTRACT) {
          if (k > 0.0f && bt > 0) {
            float h = clamp(0.5f - 0.5f * (acc_dist + dist) / k, 0.0f, 1.0f);
            acc_color = mix(acc_color, sobj.color.rgb, h);
          }
          else {
            if (acc_dist + dist < 0.0f) {
              acc_color = sobj.color.rgb;
            }
          }
        }
        else if (op == SDF_CSG_OP_INTERSECT) {
          if (k > 0.0f && bt > 0) {
            float h = clamp(0.5f + 0.5f * (dist - acc_dist) / k, 0.0f, 1.0f);
            acc_color = mix(acc_color, sobj.color.rgb, h);
          }
          else {
            if (dist > acc_dist) {
              acc_color = sobj.color.rgb;
            }
          }
        }
        else if (op == SDF_CSG_OP_PUSH) {
          float sub_base = combineCSG(acc_dist, dist, SDF_CSG_OP_SUBTRACT, bt, k, 0.0f);
          if (dist <= sub_base) {
            acc_color = sobj.color.rgb;
          }
        }
        else if (op == SDF_CSG_OP_AVOID) {
          float carved;
          if (k > 0.0f && bt > 0) {
            if (bt == SDF_BLEND_TYPE_SMOOTH) {
              carved = opSmoothSubtraction(acc_dist, dist, k);
            }
            else if (bt == SDF_BLEND_TYPE_CHAMFER) {
              carved = opChamferSubtraction(acc_dist, dist, k);
            }
            else if (bt == SDF_BLEND_TYPE_ROUND) {
              carved = opRoundSubtraction(acc_dist, dist, k);
            }
            else {
              carved = max(dist, -acc_dist);
            }
          }
          else {
            carved = max(dist, -acc_dist);
          }
          if (carved < acc_dist) {
            acc_color = sobj.color.rgb;
          }
        }

        acc_dist = new_dist;
      }
    }

    int3 atlas_coord = slot_origin + local_voxel;
    imageStore(compact_atlas, atlas_coord, float4(acc_dist, acc_color));
    imageStore(object_id_atlas, atlas_coord, int4(acc_obj_id, 0, 0, 0));
  }
}
