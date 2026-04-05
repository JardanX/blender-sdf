/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Grid evaluation: samples SDF at grid vertices.
 * Uses BVH to cull distant objects, then evaluates nearby objects in scene order. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_grid_eval_comp)

#include "sdf_lib.glsl"

void main()
{
  int3 gid = int3(gl_GlobalInvocationID.xyz);
  if (gid.x >= grid_verts || gid.y >= grid_verts || gid.z >= grid_verts) {
    return;
  }

  float3 world_pos = grid_origin + float3(gid) * cell_size;

  /* Phase 1: BVH cull — build bitmask of nearby objects (max 1024) */
  uint obj_bits[32]; /* 1024 bits */
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
    /* No BVH: mark all objects, with per-object AABB cull */
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

  /* Phase 2: evaluate in scene order (preserves CSG) */
  float scene_dist = 1e10f;
  float3 dummy_color = float3(0.5f);
  int cur_group = -2;
  float grp_dist = 1e10f;
  float3 grp_color = float3(0.5f);
  bool grp_has_hit = false;
  float3 grp_pos = world_pos;
  float grp_scale = 1.0f;

  for (int i = 0; i < object_count; i++) {
    /* Skip if not flagged by BVH cull */
    if ((obj_bits[i >> 5] & (1u << (i & 31))) == 0u) {
      int gid_obj = objects[i].group_id;
      if (gid_obj != cur_group && grp_has_hit) {
        if (cur_group >= 0 && groups[cur_group].modifier_count > 0) {
          grp_dist = applyGroupDistanceModifiers(grp_dist, grp_pos, groups[cur_group].modifier_start, groups[cur_group].modifier_count);
          grp_dist *= grp_scale;
        }
        flushGroup(cur_group, grp_dist, grp_color, scene_dist, dummy_color);
        grp_has_hit = false;
        grp_dist = 1e10f;
        grp_color = (gid_obj >= 0) ? groups[gid_obj].color.rgb : float3(0.5f);
      }
      if (gid_obj != cur_group && gid_obj >= 0 && groups[gid_obj].modifier_count > 0) {
        float4 dm = applyDomainModifiers(world_pos, groups[gid_obj].modifier_start, groups[gid_obj].modifier_count, float4x4(1.0));
        grp_pos = dm.xyz;
        grp_scale = dm.w;
      }
      else if (gid_obj != cur_group) {
        grp_pos = world_pos;
        grp_scale = 1.0f;
      }
      cur_group = gid_obj;
      continue;
    }

    SDFObjectGPU obj = objects[i];
    int gid_obj = obj.group_id;

    if (gid_obj != cur_group && grp_has_hit) {
      if (cur_group >= 0 && groups[cur_group].modifier_count > 0) {
        grp_dist = applyGroupDistanceModifiers(grp_dist, grp_pos, groups[cur_group].modifier_start, groups[cur_group].modifier_count);
        grp_dist *= grp_scale;
      }
      flushGroup(cur_group, grp_dist, grp_color, scene_dist, dummy_color);
      grp_has_hit = false;
      grp_dist = 1e10f;
      grp_color = (gid_obj >= 0) ? groups[gid_obj].color.rgb : float3(0.5f);
    }

    if (gid_obj != cur_group && gid_obj >= 0 && groups[gid_obj].modifier_count > 0) {
      float4 dm = applyDomainModifiers(world_pos, groups[gid_obj].modifier_start, groups[gid_obj].modifier_count, float4x4(1.0));
      grp_pos = dm.xyz;
      grp_scale = dm.w;
    }
    else if (gid_obj != cur_group) {
      grp_pos = world_pos;
      grp_scale = 1.0f;
    }

    float3 eval_pos = (gid_obj >= 0) ? grp_pos : world_pos;
    float3 lp = (obj.inverse_matrix * float4(eval_pos - obj.position.xyz, 1.0f)).xyz;
    float d = evalPrimitive(lp, obj);
    cur_group = gid_obj;

    if (gid_obj < 0) {
      if (scene_dist >= 1e9f) {
        if (obj.csg_operation == 0) { scene_dist = d; }
      }
      else {
        scene_dist = combineCSG(scene_dist, d, obj.csg_operation, obj.blend_type, obj.blend,
                                obj.shell_distance, obj.shell_mode, obj.shell_op, obj.shell_blend_top,
                                obj.shell_blend_bottom, obj.chamfer_k2, obj.chamfer_k3, obj.chamfer_k4, obj.chamfer_k5, obj.flip_blend, obj.flip_blend_end);
      }
    }
    else {
      if (!grp_has_hit) {
        if (obj.csg_operation != SDF_CSG_OP_SUBTRACT && obj.csg_operation != SDF_CSG_OP_SHELL &&
            obj.csg_operation != SDF_CSG_OP_INTERSECT) {
          grp_dist = d;
          grp_has_hit = true;
        }
      }
      else {
        grp_dist = combineCSG(grp_dist, d, obj.csg_operation, obj.blend_type, obj.blend,
                              obj.shell_distance, obj.shell_mode, obj.shell_op, obj.shell_blend_top,
                              obj.shell_blend_bottom, obj.chamfer_k2, obj.chamfer_k3, obj.chamfer_k4, obj.chamfer_k5, obj.flip_blend, obj.flip_blend_end);
      }
    }
  }

  if (grp_has_hit) {
    if (cur_group >= 0 && groups[cur_group].modifier_count > 0) {
      grp_dist = applyGroupDistanceModifiers(grp_dist, grp_pos, groups[cur_group].modifier_start, groups[cur_group].modifier_count);
      grp_dist *= grp_scale;
    }
    flushGroup(cur_group, grp_dist, grp_color, scene_dist, dummy_color);
  }

  int idx = gid.z * grid_verts * grid_verts + gid.y * grid_verts + gid.x;
  grid_values[idx] = scene_dist;
}
