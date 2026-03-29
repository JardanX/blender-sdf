/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Color resolve: evaluates scene at trace hit positions to determine color.
 * Runs after trace (which only computes distance), before normal/shade. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_color_resolve_comp)

#include "sdf_lib.glsl"

shared int s_candidates[kMaxTileObjects];
shared int s_numCandidates;

void main()
{
  int2 pixel = int2(gl_GlobalInvocationID.xy);

  int tilesX = (screen_size.x + kTileSize - 1) / kTileSize;
  int tileIdx = int(gl_WorkGroupID.x) + int(gl_WorkGroupID.y) * tilesX;
  if (gl_LocalInvocationIndex == 0u) {
    s_numCandidates = min(tile_prim_counts[tileIdx], kMaxTileObjects);
  }
  barrier();
  int base = tileIdx * kMaxTileObjects;
  int nc = s_numCandidates;
  for (int i = int(gl_LocalInvocationIndex); i < nc; i += 64) {
    s_candidates[i] = tile_prim_lists[base + i];
  }
  barrier();

  if (pixel.x >= screen_size.x || pixel.y >= screen_size.y) {
    return;
  }

  float4 gbuf = imageLoad(gbuf_pos_img, pixel);
  if (gbuf.w == 0.0f) {
    imageStore(gbuf_color_img, pixel, float4(0.0));
    return;
  }

  float3 world_pos = gbuf.xyz;
  float obj_id = imageLoad(gbuf_color_img, pixel).a;

  float3 out_color = float3(0.5f);
  float scene_dist = 1e10f;

  int cur_group = -2;
  float grp_dist = 1e10f;
  float3 grp_color = float3(0.5f);
  bool grp_has_hit = false;

  for (int u = 0; u < nc; u++) {
    int i = s_candidates[u];
    SDFObjectAABB aabb = object_aabbs[i];
    int gid = aabb.group_id;

    if (gid != cur_group && grp_has_hit) {
      flushGroup(cur_group, grp_dist, grp_color, scene_dist, out_color);
      grp_has_hit = false;
      grp_dist = 1e10f;
    }

    float da = point_aabb_dist(world_pos, aabb.bbox_min.xyz, aabb.bbox_max.xyz);
    float skip_threshold = max(0.001f, aabb.max_group_blend);
    if (da > skip_threshold) {
      cur_group = gid;
      continue;
    }

    SDFObjectGPU obj = objects[i];
    float3 lp = (obj.inverse_matrix * float4(world_pos - obj.position.xyz, 1.0f)).xyz;
    float d = evalPrimitive(lp, obj);
    cur_group = gid;

    if (gid < 0) {
      if (scene_dist >= 1e9f) {
        if (obj.csg_operation == 0) {
          scene_dist = d;
          out_color = obj.color.rgb;
        }
      }
      else {
        float prev = scene_dist;
        scene_dist = combineCSG(
            scene_dist, d, obj.csg_operation, obj.blend_type, obj.blend,
            obj.shell_distance, obj.shell_mode, obj.shell_op, obj.shell_blend_top, obj.shell_blend_bottom, obj.chamfer_k2, obj.chamfer_k3);
        float t = csgColorFactor(prev, d, obj.csg_operation, obj.blend_type, obj.blend);
        out_color = mix(out_color, obj.color.rgb, t);
      }
    }
    else {
      if (!grp_has_hit) {
        if (obj.csg_operation != SDF_CSG_OP_SUBTRACT && obj.csg_operation != SDF_CSG_OP_SHELL) {
          grp_dist = d;
          grp_color = obj.color.rgb;
          grp_has_hit = true;
        }
      }
      else {
        float prev = grp_dist;
        grp_dist = combineCSG(
            grp_dist, d, obj.csg_operation, obj.blend_type, obj.blend,
            obj.shell_distance, obj.shell_mode, obj.shell_op, obj.shell_blend_top, obj.shell_blend_bottom, obj.chamfer_k2, obj.chamfer_k3);
        float t = csgColorFactor(prev, d, obj.csg_operation, obj.blend_type, obj.blend);
        grp_color = mix(grp_color, obj.color.rgb, t);
      }
    }
  }

  if (grp_has_hit) {
    flushGroup(cur_group, grp_dist, grp_color, scene_dist, out_color);
  }

  imageStore(gbuf_color_img, pixel, float4(out_color, obj_id));
}
