/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Grid evaluation: dispatch 33^3 threads, each evaluates the full scene SDF
 * (with CSG operations) at a grid vertex for proxy mesh generation. */

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

  /* Full scene evaluation with CSG, matching the trace shader's logic.
   * Objects are sorted by (group_id, group_order) in the SSBO. */
  float scene_dist = 1e10f;
  float3 dummy_color = float3(0.5f);

  int cur_group = -2;
  float grp_dist = 1e10f;
  float3 grp_color = float3(0.5f);
  bool grp_has_hit = false;

  for (int i = 0; i < object_count; i++) {
    SDFObjectGPU obj = objects[i];
    int gid_obj = obj.group_id;

    /* Group boundary: flush previous group into scene */
    if (gid_obj != cur_group && grp_has_hit) {
      flushGroup(cur_group, grp_dist, grp_color, scene_dist, dummy_color);
      grp_has_hit = false;
      grp_dist = 1e10f;
    }

    float3 lp = (obj.inverse_matrix * float4(world_pos - obj.position.xyz, 1.0f)).xyz;
    float d = evalPrimitive(lp, obj);
    cur_group = gid_obj;

    if (gid_obj < 0) {
      /* Ungrouped */
      if (scene_dist >= 1e9f) {
        if (obj.csg_operation == 0) {
          scene_dist = d;
        }
      }
      else {
        scene_dist = combineCSG(
            scene_dist, d, obj.csg_operation, obj.blend_type, obj.blend,
            obj.shell_distance, obj.shell_mode, 0.0f, 0.0f, obj.chamfer_k2, obj.chamfer_k3);
      }
    }
    else {
      /* Grouped */
      if (!grp_has_hit) {
        if (obj.csg_operation == 0) {
          grp_dist = d;
          grp_has_hit = true;
        }
      }
      else {
        grp_dist = combineCSG(
            grp_dist, d, obj.csg_operation, obj.blend_type, obj.blend,
            obj.shell_distance, obj.shell_mode, 0.0f, 0.0f, obj.chamfer_k2, obj.chamfer_k3);
      }
    }
  }

  /* Flush final group */
  if (grp_has_hit) {
    flushGroup(cur_group, grp_dist, grp_color, scene_dist, dummy_color);
  }

  int idx = gid.z * grid_verts * grid_verts + gid.y * grid_verts + gid.x;
  grid_values[idx] = scene_dist;
}
