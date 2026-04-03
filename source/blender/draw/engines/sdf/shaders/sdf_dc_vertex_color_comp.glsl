/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Dual Contouring pass 3: sample SDF color at each DC vertex position.
 * For every generated vertex, evaluates the full CSG tree to determine
 * the surface color via nearest-object blending. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_dc_vertex_color_comp)

#include "sdf_lib.glsl"

void main()
{
  int vi = int(gl_GlobalInvocationID.x);
  if (vi >= vert_count) { return; }

  float3 world_pos = dc_positions[vi * 2].xyz;

  /* Build per-object active-bits via BVH or brute-force AABB test. */
  uint obj_bits[32];
  for (int w = 0; w < 32; w++) { obj_bits[w] = 0u; }

  if (use_bvh != 0 && bvh_root >= 0 && object_count <= 1024) {
    int stack[64];
    int sp = 0;
    stack[sp++] = bvh_root;
    float margin = 0.5f;

    while (sp > 0) {
      int ni = stack[--sp];
      SdfAabbNodeGPU node = aabb_nodes[ni];

      if (world_pos.x < node.bounds_min.x - margin ||
          world_pos.x > node.bounds_max.x + margin ||
          world_pos.y < node.bounds_min.y - margin ||
          world_pos.y > node.bounds_max.y + margin ||
          world_pos.z < node.bounds_min.z - margin ||
          world_pos.z > node.bounds_max.z + margin)
      {
        continue;
      }

      if (node.shape_index >= 0) {
        int si = node.shape_index;
        obj_bits[si >> 5] |= (1u << (si & 31));
      }
      else {
        if (node.child_a >= 0 && sp < 63) { stack[sp++] = node.child_a; }
        if (node.child_b >= 0 && sp < 63) { stack[sp++] = node.child_b; }
      }
    }
  }
  else {
    for (int i = 0; i < min(object_count, 1024); i++) {
      float3 bmin = objects[i].bbox_min.xyz;
      float3 bmax = objects[i].bbox_max.xyz;
      float margin = max(objects[i].blend, 0.1f);
      if (world_pos.x >= bmin.x - margin && world_pos.x <= bmax.x + margin &&
          world_pos.y >= bmin.y - margin && world_pos.y <= bmax.y + margin &&
          world_pos.z >= bmin.z - margin && world_pos.z <= bmax.z + margin)
      {
        obj_bits[i >> 5] |= (1u << (i & 31));
      }
    }
  }

  /* Evaluate the full CSG tree with color tracking. */
  float scene_dist = 1e10f;
  float3 scene_color = float3(0.5f);
  int cur_group = -2;
  float grp_dist = 1e10f;
  float3 grp_color = float3(0.5f);
  bool grp_has_hit = false;

  for (int i = 0; i < object_count; i++) {
    if ((obj_bits[i >> 5] & (1u << (i & 31))) == 0u) {
      int gid_obj = objects[i].group_id;
      if (gid_obj != cur_group && grp_has_hit) {
        flushGroup(cur_group, grp_dist, grp_color, scene_dist, scene_color);
        grp_has_hit = false;
        grp_dist = 1e10f;
        grp_color = float3(0.5f);
      }
      cur_group = gid_obj;
      continue;
    }

    SDFObjectGPU obj = objects[i];
    int gid_obj = obj.group_id;

    if (gid_obj != cur_group && grp_has_hit) {
      flushGroup(cur_group, grp_dist, grp_color, scene_dist, scene_color);
      grp_has_hit = false;
      grp_dist = 1e10f;
      grp_color = float3(0.5f);
    }

    float3 lp = (obj.inverse_matrix * float4(world_pos - obj.position.xyz, 1.0f)).xyz;
    float d = evalPrimitive(lp, obj);
    cur_group = gid_obj;

    if (gid_obj < 0) {
      /* Ungrouped object: combine directly into scene. */
      if (scene_dist >= 1e9f) {
        if (obj.csg_operation == 0) {
          scene_dist = d;
          scene_color = obj.color.rgb;
        }
      }
      else {
        float prev = scene_dist;
        scene_dist = combineCSG(scene_dist, d, obj.csg_operation, obj.blend_type, obj.blend,
                                obj.shell_distance, obj.shell_mode, obj.shell_op, obj.shell_blend_top,
                                obj.shell_blend_bottom, obj.chamfer_k2, obj.chamfer_k3);
        float t = csgColorFactor(prev, d, obj.csg_operation, obj.blend_type, obj.blend,
                                 obj.shell_distance, obj.shell_op);
        scene_color = mix(scene_color, obj.color.rgb, t);
      }
    }
    else {
      /* Grouped object: combine within group, flush later. */
      if (!grp_has_hit) {
        if (obj.csg_operation != SDF_CSG_OP_SUBTRACT && obj.csg_operation != SDF_CSG_OP_SHELL &&
            obj.csg_operation != SDF_CSG_OP_INTERSECT) {
          grp_dist = d;
          grp_color = obj.color.rgb;
          grp_has_hit = true;
        }
      }
      else {
        float prev = grp_dist;
        grp_dist = combineCSG(grp_dist, d, obj.csg_operation, obj.blend_type, obj.blend,
                              obj.shell_distance, obj.shell_mode, obj.shell_op, obj.shell_blend_top,
                              obj.shell_blend_bottom, obj.chamfer_k2, obj.chamfer_k3);
        float t = csgColorFactor(prev, d, obj.csg_operation, obj.blend_type, obj.blend,
                                 obj.shell_distance, obj.shell_op);
        grp_color = mix(grp_color, obj.color.rgb, t);
      }
    }
  }

  if (grp_has_hit) {
    flushGroup(cur_group, grp_dist, grp_color, scene_dist, scene_color);
  }

  dc_colors[vi] = float4(scene_color, 1.0f);
}
