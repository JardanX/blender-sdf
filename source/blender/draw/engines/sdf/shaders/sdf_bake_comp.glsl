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

/* SDF primitive types (must match eSDFType in DNA_sdf_types.h). */
#define SDF_TYPE_BOX 0
#define SDF_TYPE_SPHERE 1
#define SDF_TYPE_CYLINDER 2
#define SDF_TYPE_CONE 3
#define SDF_TYPE_CAPSULE 4
#define SDF_TYPE_TORUS 5

float sdSphere(float3 p, float r)
{
  return length(p) - r;
}

float sdCapsule(float3 p, float3 size)
{
  float h = size.y;
  float r = size.x;
  p.z -= clamp(p.z, -h, h);
  return length(p) - r;
}

float sdTorus(float3 p, float2 t)
{
  float2 q = float2(length(p.xy) - t.x, p.z);
  return length(q) - t.y;
}

float sdCylinder(float3 p, float3 size)
{
  /* Elliptical cylinder: size.xy = XY radii, size.z = half-height.
   * Gradient-corrected radial distance for proper SDF. */
  float2 e = max(size.xy, float2(0.001f));
  float2 pn = p.xy / e;
  float rn = length(pn);
  float2 g = pn / (e * max(rn, 1e-6f));
  float radial = (rn - 1.0f) / max(length(g), 1e-6f);
  float vertical = abs(p.z) - size.z;
  float2 d = float2(radial, vertical);
  return length(max(d, float2(0.0f))) + min(max(d.x, d.y), 0.0f);
}

float sdCone(float3 p, float r, float h)
{
  /* Capped cone: base radius r at z=-h, apex at z=+h.
   * Based on Inigo Quilez's sdCappedCone. */
  float2 q = float2(length(p.xy), p.z);
  float2 k1 = float2(0.0f, h);
  float2 k2 = float2(-r, 2.0f * h);
  float2 ca = float2(q.x - min(q.x, (q.y < 0.0f) ? r : 0.0f), abs(q.y) - h);
  float2 cb = q - k1 + k2 * clamp(dot(k1 - q, k2) / dot(k2, k2), 0.0f, 1.0f);
  float s = (cb.x < 0.0f && ca.y < 0.0f) ? -1.0f : 1.0f;
  return s * sqrt(min(dot(ca, ca), dot(cb, cb)));
}

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
  float4 box_corners;
  float4 box_edges;
  int4 box_modes;
};

/** Evaluate the actual SDF primitive (reads from shared memory cache). */
float evalSDFPrimitiveSh(float3 local_pos, SharedObj obj)
{
  /* Apply domain modifiers (warp sampling space). */
  if (obj.modifier_count > 0) {
    local_pos = applyDomainModifiers(local_pos, obj.modifier_start, obj.modifier_count);
  }

  float3 size = obj.sdf_size.xyz;
  float bevel = obj.bevel;
  float dist;

  if (obj.sdf_type == SDF_TYPE_SPHERE) {
    dist = sdSphere(local_pos, size.x - bevel);
  }
  else if (obj.sdf_type == SDF_TYPE_CYLINDER) {
    float3 cyl_size = size - float3(bevel);
    cyl_size = max(cyl_size, float3(0.001f));
    dist = sdCylinder(local_pos, cyl_size);
  }
  else if (obj.sdf_type == SDF_TYPE_CONE) {
    float cone_r = max(size.x - bevel, 0.001f);
    float cone_h = max(size.y - bevel, 0.001f);
    dist = sdCone(local_pos, cone_r, cone_h);
  }
  else if (obj.sdf_type == SDF_TYPE_CAPSULE) {
    float3 cap_size = size - float3(bevel);
    cap_size = max(cap_size, float3(0.001f));
    dist = sdCapsule(local_pos, cap_size);
  }
  else if (obj.sdf_type == SDF_TYPE_TORUS) {
    float major = size.x - bevel;
    float minor = size.y - bevel;
    major = max(major, 0.001f);
    minor = max(minor, 0.001f);
    dist = sdTorus(local_pos, float2(major, minor));
  }
  else {
    /* SDF_TYPE_BOX (default). */
    float4 corners = obj.box_corners;
    float edgeTop = obj.box_edges.x;
    float edgeBot = obj.box_edges.y;
    float tapTop = obj.box_edges.z;
    float tapBot = obj.box_edges.w;
    bool hasAdvanced = (corners.x + corners.y + corners.z + corners.w +
                        edgeTop + edgeBot + tapTop + tapBot) > 0.001f;
    if (hasAdvanced) {
      float3 box_size = size - float3(bevel);
      box_size = max(box_size, float3(0.001f));
      dist = sdAdvancedBox(local_pos,
                           box_size,
                           corners,
                           edgeTop,
                           edgeBot,
                           tapTop,
                           tapBot,
                           obj.box_modes.x,
                           obj.box_modes.y,
                           box_size.z);
    }
    else {
      float3 box_size = size - float3(bevel);
      box_size = max(box_size, float3(0.001f));
      dist = sdBox(local_pos, box_size);
    }
  }

  dist -= bevel;

  /* Apply distance modifiers. */
  if (obj.modifier_count > 0) {
    dist = applyDistanceModifiers(dist, obj.modifier_start, obj.modifier_count);
  }

  return dist;
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
    shared_objs[c].box_corners = obj.box_corners;
    shared_objs[c].box_edges = obj.box_edges;
    shared_objs[c].box_modes = obj.box_modes;
  }
  barrier();

  /* Local thread covers XY, loop over Z. */
  int2 local_xy = int2(gl_LocalInvocationID.xy);

  for (int lz = 0; lz < BRICK_STORAGE; lz++) {
    int3 local_voxel = int3(local_xy, lz);

    /* World-space position: brick_coord * 8 + (local - 2) + 0.5, times voxel_size.
     * The -2 accounts for the 2-voxel overlap border. */
    float3 world_pos = atlas_origin +
                       (float3(brick * BRICK_SIZE + local_voxel - int3(2)) + 0.5f) * voxel_size;

    float acc_dist = 1e10f;
    float3 acc_color = float3(0.0f);
    int acc_obj_id = -1;
    float closest_raw_dist = 1e10f;

    /* Two-pass evaluation: build base from union/shell objects first, then
     * apply modifiers (subtract/intersect). Shell (extrusion) is union-like —
     * it adds geometry then clips with a limit plane, so it runs in pass 1
     * and naturally gets the accumulated base without needing expanded AABBs. */

    /* Pass 1: accumulate union and shell objects to build the base. */
    for (int c = 0; c < num_candidates; c++) {
      SharedObj sobj = shared_objs[c];
      if (sobj.csg_operation != SDF_CSG_OP_UNION && sobj.csg_operation != SDF_CSG_OP_SHELL) {
        continue;
      }

      float3 local_pos = (sobj.inverse_matrix * float4(world_pos - sobj.position.xyz, 1.0f)).xyz;
      float dist = evalSDFPrimitiveSh(local_pos, sobj);

      if (dist < closest_raw_dist) {
        closest_raw_dist = dist;
        acc_obj_id = sobj.obj_index;
      }

      float k = sobj.blend;
      int bt = sobj.blend_type;
      int op = sobj.csg_operation;

      if (acc_dist >= 1e9f) {
        if (op == SDF_CSG_OP_SHELL) {
          continue;
        }
        acc_color = sobj.color.rgb;
        acc_dist = dist;
      }
      else {
        float new_dist = combineCSG(acc_dist, dist, op, bt, k, sobj.shell_distance);

        /* Union color: blend factor = how much the new object contributes.
         * Works for all blend types — smooth formula is a good approximation
         * of the blend zone for chamfer/round too. */
        if (k > 0.0f && bt > 0) {
          float h = clamp(0.5f + 0.5f * (acc_dist - dist) / k, 0.0f, 1.0f);
          acc_color = mix(acc_color, sobj.color.rgb, h);
        }
        else {
          if (dist < acc_dist) {
            acc_color = sobj.color.rgb;
          }
        }
        acc_dist = new_dist;
      }
    }

    /* Pass 2: apply modifiers (subtract/intersect/push/avoid) to the base. */
    for (int c = 0; c < num_candidates; c++) {
      SharedObj sobj = shared_objs[c];
      if (sobj.csg_operation == SDF_CSG_OP_UNION || sobj.csg_operation == SDF_CSG_OP_SHELL) {
        continue;
      }

      float3 local_pos = (sobj.inverse_matrix * float4(world_pos - sobj.position.xyz, 1.0f)).xyz;
      float dist = evalSDFPrimitiveSh(local_pos, sobj);

      if (dist < closest_raw_dist) {
        closest_raw_dist = dist;
        acc_obj_id = sobj.obj_index;
      }

      float k = sobj.blend;
      int bt = sobj.blend_type;
      int op = sobj.csg_operation;

      if (acc_dist >= 1e9f) {
        if (op == SDF_CSG_OP_PUSH || op == SDF_CSG_OP_AVOID) {
          acc_color = sobj.color.rgb;
          acc_dist = dist;
        }
        continue;
      }

      float new_dist = combineCSG(acc_dist, dist, op, bt, k, 0.0f);

      /* Color blending for all 6 operations × 4 blend types.
       * Smooth-style blend factor (h) approximates all blend types well. */
      if (op == SDF_CSG_OP_SUBTRACT) {
        /* Subtraction: subtractor's color where it has carved (inside the cutter),
         * base color where the original surface remains. Sharp seam at the
         * subtractor's d=0 boundary — no gradual falloff into base. Works for
         * all blend types (linear, smooth, chamfer, round). */
        if (dist <= 0.0f) {
          acc_color = sobj.color.rgb;
        }
        acc_dist = new_dist;
      }
      else if (op == SDF_CSG_OP_INTERSECT) {
        /* Intersect: intersector's color shows where it limits the base. */
        if (k > 0.0f && bt > 0) {
          float h = clamp(0.5f + 0.5f * (dist - acc_dist) / k, 0.0f, 1.0f);
          acc_color = mix(acc_color, sobj.color.rgb, h);
        }
        else {
          if (dist > acc_dist) {
            acc_color = sobj.color.rgb;
          }
        }
        acc_dist = new_dist;
      }
      else if (op == SDF_CSG_OP_PUSH) {
        /* Push = subtract base by d2, then union d2 back.
         * Color by surface ownership: push object where it's closer than the
         * dented base, base color otherwise. Sharp boundary — no fading. */
        float sub_base = combineCSG(acc_dist, dist, SDF_CSG_OP_SUBTRACT, bt, k, 0.0f);
        if (dist <= sub_base) {
          acc_color = sobj.color.rgb;
        }
        acc_dist = new_dist;
      }
      else if (op == SDF_CSG_OP_AVOID) {
        /* Avoid = subtract base from avoid, then union with base.
         * Color by surface ownership: avoid color where it's visible (outside
         * base), base color otherwise. Sharp boundary — no fading. */
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
        acc_dist = new_dist;
      }
    }

    int3 atlas_coord = slot_origin + local_voxel;
    imageStore(compact_atlas, atlas_coord, float4(acc_dist, acc_color));
    imageStore(object_id_atlas, atlas_coord, int4(acc_obj_id, 0, 0, 0));
  }
}
