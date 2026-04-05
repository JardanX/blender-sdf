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
  float3 grp_pos = world_pos;
  float grp_scale = 1.0f;

  for (int u = 0; u < nc; u++) {
    int i = s_candidates[u];
    SDFObjectAABB aabb = object_aabbs[i];
    int gid = aabb.group_id;

    if (gid != cur_group && grp_has_hit) {
      if (cur_group >= 0 && groups[cur_group].modifier_count > 0) {
        grp_dist = applyGroupDistanceModifiers(grp_dist, grp_pos, groups[cur_group].modifier_start, groups[cur_group].modifier_count);
        grp_dist *= grp_scale;
      }
      flushGroup(cur_group, grp_dist, grp_color, scene_dist, out_color);
      grp_has_hit = false;
      grp_dist = 1e10f;
      grp_color = (gid >= 0) ? groups[gid].color.rgb : float3(0.5f);
    }

    if (gid != cur_group && gid >= 0 && groups[gid].modifier_count > 0) {
      float4 dm = applyDomainModifiers(world_pos, groups[gid].modifier_start, groups[gid].modifier_count, float4x4(1.0));
      grp_pos = dm.xyz;
      grp_scale = dm.w;
    }
    else if (gid != cur_group) {
      grp_pos = world_pos;
      grp_scale = 1.0f;
    }

    SDFObjectGPU obj = objects[i];
    float3 eval_pos = (gid >= 0) ? grp_pos : world_pos;
    float da = point_aabb_dist(eval_pos, obj.orig_bbox_min.xyz, obj.orig_bbox_max.xyz);
    float skip_threshold = max(0.001f, aabb.max_group_blend);

    int obj_op = obj.csg_operation;
    bool must_eval = (obj_op == SDF_CSG_OP_INTERSECT || obj_op == SDF_CSG_OP_SUBTRACT) &&
                     ((gid >= 0 && grp_has_hit && gid == cur_group) ||
                      (gid < 0 && scene_dist < 1e9f));

    float d;
    if (da > skip_threshold) {
      if (!must_eval) {
        cur_group = gid;
        continue;
      }
      d = da;
      cur_group = gid;
    }
    else {
      float3 lp = (obj.inverse_matrix * float4(eval_pos - obj.position.xyz, 1.0f)).xyz;
      d = evalPrimitive(lp, obj);
      cur_group = gid;
    }

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
            obj.shell_distance, obj.shell_mode, obj.shell_op, obj.shell_blend_top, obj.shell_blend_bottom, obj.chamfer_k2, obj.chamfer_k3, obj.chamfer_k4, obj.chamfer_k5, obj.flip_blend, obj.flip_blend_end);
        float t = csgColorFactor(prev, d, obj.csg_operation, obj.blend_type, obj.blend,
                             obj.shell_distance, obj.shell_op);
        out_color = mix(out_color, obj.color.rgb, t);
      }
    }
    else {
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
        grp_dist = combineCSG(
            grp_dist, d, obj.csg_operation, obj.blend_type, obj.blend,
            obj.shell_distance, obj.shell_mode, obj.shell_op, obj.shell_blend_top, obj.shell_blend_bottom, obj.chamfer_k2, obj.chamfer_k3, obj.chamfer_k4, obj.chamfer_k5, obj.flip_blend, obj.flip_blend_end);
        float t = csgColorFactor(prev, d, obj.csg_operation, obj.blend_type, obj.blend,
                             obj.shell_distance, obj.shell_op);
        grp_color = mix(grp_color, obj.color.rgb, t);
      }
    }
  }

  if (grp_has_hit) {
    if (cur_group >= 0 && groups[cur_group].modifier_count > 0) {
      grp_dist = applyGroupDistanceModifiers(grp_dist, grp_pos, groups[cur_group].modifier_start, groups[cur_group].modifier_count);
      grp_dist *= grp_scale;
    }
    flushGroup(cur_group, grp_dist, grp_color, scene_dist, out_color);
  }

  imageStore(gbuf_color_img, pixel, float4(out_color, obj_id));
}
