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

/* SDF primitive types (must match eSDFType in DNA_sdf_types.h). */
#define SDF_TYPE_BOX 0
#define SDF_TYPE_SPHERE 1
#define SDF_TYPE_CYLINDER 2
#define SDF_TYPE_CONE 3
#define SDF_TYPE_CAPSULE 4
#define SDF_TYPE_TORUS 5
#define SDF_TYPE_NGON 6

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

/** Evaluate the actual SDF primitive for an object, with modifiers. */
float evalSDFPrimitive(float3 local_pos, SDFObjectGPU obj)
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
    if (obj.box_modes.w != 0) {
      /* Capped torus: box_corners.xy = (sin, cos) of half-angle. */
      dist = sdCappedTorus(local_pos, obj.box_corners.xy, major, minor);
    }
    else {
      dist = sdTorus(local_pos, float2(major, minor));
    }
  }
  else if (obj.sdf_type == SDF_TYPE_NGON) {
    float R = max(size.x - bevel, 0.001f);
    float halfH = max(size.z - bevel, 0.001f);
    int sides = obj.box_modes.z;
    float corner = obj.box_corners.x;
    float star = obj.box_corners.y;
    float edgeTop = obj.box_edges.x;
    float edgeBot = obj.box_edges.y;
    float tapTop = obj.box_edges.z;
    float tapBot = obj.box_edges.w;
    int edgeMode = obj.box_modes.y;
    bool hasAdvanced = (corner + edgeTop + edgeBot + tapTop + tapBot + star) > 0.001f;
    if (hasAdvanced) {
      dist = sdAdvancedNgon(
          local_pos, R, halfH, sides, corner, edgeTop, edgeBot, tapTop, tapBot, edgeMode, halfH, star);
    }
    else {
      float d2d = sdRegularPolygon2D(local_pos.xy, R, sides);
      float dz = abs(local_pos.z) - halfH;
      float2 dd = float2(d2d, dz);
      dist = length(max(dd, float2(0.0f))) + min(max(dd.x, dd.y), 0.0f);
    }
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

  /* Two-pass evaluation: build the base from union/shell objects first, then
   * apply modifiers (subtract/intersect). Shell (extrusion) is union-like —
   * it adds geometry then clips with a limit plane, so it runs in pass 1
   * and naturally gets the accumulated base without needing expanded AABBs. */
  float acc_dist = 1e10f;

  /* Pass 1: accumulate union and shell objects to build the base. */
  for (int c = 0; c < num_candidates; c++) {
    SDFObjectGPU obj = objects[candidates[c]];
    if (obj.csg_operation != SDF_CSG_OP_UNION && obj.csg_operation != SDF_CSG_OP_SHELL) {
      continue;
    }
    float3 local_pos = (obj.inverse_matrix * float4(brick_center - obj.position.xyz, 1.0f)).xyz;
    float dist = evalSDFPrimitive(local_pos, obj);

    if (acc_dist >= 1e9f) {
      /* Shell/extrusion needs a base to extrude from — skip if no base yet. */
      if (obj.csg_operation == SDF_CSG_OP_SHELL) {
        continue;
      }
      acc_dist = dist;
    }
    else {
      acc_dist = combineCSG(
          acc_dist, dist, obj.csg_operation, obj.blend_type, obj.blend, obj.shell_distance);
    }
  }

  /* Pass 2: apply modifiers (subtract/intersect/push/avoid) to the base. */
  for (int c = 0; c < num_candidates; c++) {
    SDFObjectGPU obj = objects[candidates[c]];
    if (obj.csg_operation == SDF_CSG_OP_UNION || obj.csg_operation == SDF_CSG_OP_SHELL) {
      continue;
    }

    float3 local_pos = (obj.inverse_matrix * float4(brick_center - obj.position.xyz, 1.0f)).xyz;
    float dist = evalSDFPrimitive(local_pos, obj);

    if (acc_dist >= 1e9f) {
      /* Push/avoid create visible geometry even without a base. */
      if (obj.csg_operation == SDF_CSG_OP_PUSH || obj.csg_operation == SDF_CSG_OP_AVOID) {
        acc_dist = dist;
      }
      continue;
    }

    acc_dist = combineCSG(
        acc_dist, dist, obj.csg_operation, obj.blend_type, obj.blend, 0.0f);
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
