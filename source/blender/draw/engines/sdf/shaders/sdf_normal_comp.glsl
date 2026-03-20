/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Normal pass: BVH point-query + analytical per-object gradient + CSG gradient combine.
 * Separate kernel — zero trace pass register impact. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_normal_comp)

#include "sdf_lib.glsl"

#define NORMAL_BVH_STACK 20
#define NORMAL_BIT_WORDS 16
#define NORMAL_MAX_BITS (NORMAL_BIT_WORDS * 32)

void main()
{
  int2 pixel = ivec2(gl_GlobalInvocationID.xy);
  if (pixel.x >= screen_size.x || pixel.y >= screen_size.y) {
    return;
  }

  float4 gbuf = imageLoad(gbuf_pos_img, pixel);
  if (gbuf.w == 0.0f) {
    return;
  }

  float3 hit_pos = gbuf.xyz;

  /* BVH point-query: find objects whose AABB contains the hit point */
  uint bvh_bits[NORMAL_BIT_WORDS];
  for (int w = 0; w < NORMAL_BIT_WORDS; w++) {
    bvh_bits[w] = 0u;
  }

  if (use_bvh != 0 && bvh_root >= 0) {
    float thresh = sdf_ray_epsilon * 8.0f;
    float3 pt_min = hit_pos - float3(thresh);
    float3 pt_max = hit_pos + float3(thresh);

    int stack[NORMAL_BVH_STACK];
    int top = 0;
    stack[0] = bvh_root;

    while (top >= 0) {
      int nidx = stack[top--];
      if (nidx < 0) continue;

      SdfAabbNodeGPU node = aabb_nodes[nidx];

      if (any(greaterThan(node.bounds_min.xyz, pt_max)) ||
          any(greaterThan(pt_min, node.bounds_max.xyz)))
      {
        continue;
      }

      if (node.child_a < 0) {
        int si = node.shape_index;
        if (si >= 0 && si < NORMAL_MAX_BITS) {
          bvh_bits[uint(si) >> 5u] |= 1u << (uint(si) & 31u);
        }
      }
      else {
        if (top + 2 < NORMAL_BVH_STACK) {
          stack[++top] = node.child_a;
          stack[++top] = node.child_b;
        }
      }
    }
  }
  else {
    for (int w = 0; w < NORMAL_BIT_WORDS; w++) {
      bvh_bits[w] = 0xFFFFFFFFu;
    }
  }

  /* Analytical gradient with CSG combine, BVH-filtered */
  float4 scene_dg = float4(1e10f, 0.0f, 0.0f, 1.0f);

  for (int g = 0; g < group_count; g++) {
    SDFGroupGPU grp = groups[g];
    float4 grp_dg = float4(1e10f, 0.0f, 0.0f, 1.0f);
    bool grp_has_hit = false;

    for (int m = grp.first_object; m < grp.first_object + grp.object_count; m++) {
      if (m < NORMAL_MAX_BITS) {
        if ((bvh_bits[uint(m) >> 5u] & (1u << (uint(m) & 31u))) == 0u) continue;
      }
      float mgb = intBitsToFloat(objects[m]._pad3);
      float normal_thresh = max(sdf_ray_epsilon * 4.0f, mgb);
      if (point_aabb_dist(hit_pos, objects[m].bbox_min.xyz, objects[m].bbox_max.xyz) >
          normal_thresh)
      {
        continue;
      }

      SDFObjectGPU obj = objects[m];
      float3 lp = (obj.inverse_matrix * float4(hit_pos - obj.position.xyz, 1.0f)).xyz;

      SDFPrimitiveData prim;
      prim.sdf_type = obj.sdf_type;
      prim.size = obj.sdf_size.xyz;
      prim.bevel = obj.bevel;
      prim.box_corners = obj.box_corners;
      prim.box_edges = obj.box_edges;
      prim.box_modes = obj.box_modes;
      prim.modifier_start = obj.modifier_start;
      prim.modifier_count = obj.modifier_count;
      prim.polygon_point_start = obj.polygon_point_start;
      prim.polygon_point_count = obj.polygon_point_count;
      prim.inverse_matrix = obj.inverse_matrix;

      float4 dg = evalObjectSDFGrad(prim, lp);

      float3 c0 = obj.inverse_matrix[0].xyz;
      float3 c1 = obj.inverse_matrix[1].xyz;
      float3 c2 = obj.inverse_matrix[2].xyz;
      float3 lg = dg.yzw;
      dg = float4(dg.x, float3(dot(c0, lg), dot(c1, lg), dot(c2, lg)));

      if (!grp_has_hit) {
        grp_dg = dg;
        grp_has_hit = true;
      }
      else {
        grp_dg = combineCSGGrad(
            grp_dg, dg, obj.csg_operation, obj.blend_type, obj.blend,
            obj.shell_distance, obj.shell_mode, obj.chamfer_k2, obj.chamfer_k3);
      }
    }

    if (!grp_has_hit) continue;

    if (scene_dg.x >= 1e9f) {
      scene_dg = grp_dg;
    }
    else {
      scene_dg = combineCSGGrad(
          scene_dg, grp_dg, grp.csg_operation, grp.blend_type, grp.blend,
          grp.shell_distance, grp.shell_mode, grp.chamfer_k2, grp.chamfer_k3);
    }
  }

  for (int i = 0; i < object_count; i++) {
    if (objects[i].group_id >= 0) continue;
    if (i < NORMAL_MAX_BITS) {
      if ((bvh_bits[uint(i) >> 5u] & (1u << (uint(i) & 31u))) == 0u) continue;
    }
    if (point_aabb_dist(hit_pos, objects[i].bbox_min.xyz, objects[i].bbox_max.xyz) >
        sdf_ray_epsilon)
    {
      continue;
    }

    SDFObjectGPU obj = objects[i];
    float3 lp = (obj.inverse_matrix * float4(hit_pos - obj.position.xyz, 1.0f)).xyz;

    SDFPrimitiveData prim;
    prim.sdf_type = obj.sdf_type;
    prim.size = obj.sdf_size.xyz;
    prim.bevel = obj.bevel;
    prim.box_corners = obj.box_corners;
    prim.box_edges = obj.box_edges;
    prim.box_modes = obj.box_modes;
    prim.modifier_start = obj.modifier_start;
    prim.modifier_count = obj.modifier_count;
    prim.inverse_matrix = obj.inverse_matrix;

    float4 dg = evalObjectSDFGrad(prim, lp);

    float3 c0 = obj.inverse_matrix[0].xyz;
    float3 c1 = obj.inverse_matrix[1].xyz;
    float3 c2 = obj.inverse_matrix[2].xyz;
    float3 lg = dg.yzw;
    dg = float4(dg.x, float3(dot(c0, lg), dot(c1, lg), dot(c2, lg)));

    if (scene_dg.x >= 1e9f) {
      scene_dg = dg;
    }
    else {
      scene_dg = combineCSGGrad(
          scene_dg, dg, obj.csg_operation, obj.blend_type, obj.blend,
          obj.shell_distance, obj.shell_mode);
    }
  }

  float3 normal = normalize(scene_dg.yzw);
  if (any(isnan(normal)) || dot(normal, normal) < 0.5f) {
    normal = float3(0.0f, 0.0f, 1.0f);
  }

  imageStore(gbuf_normal_img, pixel, float4(normal, 0.0f));
}
