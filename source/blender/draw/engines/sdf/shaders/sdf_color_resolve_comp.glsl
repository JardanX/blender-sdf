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

bool sdfBlendNormalContributions(float weight_a,
                                 float weight_b,
                                 float3 normal_a,
                                 bool valid_a,
                                 float3 normal_b,
                                 bool valid_b,
                                 out float3 normal)
{
  bool uses_a = abs(weight_a) > 1e-4f;
  bool uses_b = abs(weight_b) > 1e-4f;
  if ((uses_a && !valid_a) || (uses_b && !valid_b)) {
    normal = float3(0.0f);
    return false;
  }
  if (!uses_a && !uses_b) {
    normal = float3(0.0f);
    return false;
  }

  normal = (uses_a ? normal_a * weight_a : float3(0.0f)) +
           (uses_b ? normal_b * weight_b : float3(0.0f));
  float normal_len_squared = dot(normal, normal);
  if (normal_len_squared <= 1e-12f || any(isnan(normal))) {
    normal = abs(weight_a) >= abs(weight_b) ? normal_a * weight_a : normal_b * weight_b;
    normal_len_squared = dot(normal, normal);
    if (normal_len_squared <= 1e-12f || any(isnan(normal))) {
      normal = float3(0.0f);
      return false;
    }
  }
  return true;
}

bool sdfMeshModifiersPreserveNormal(SDFObjectGPU obj)
{
  for (int i = obj.modifier_start; i < obj.modifier_start + obj.modifier_count; i++) {
    int modifier_type = sdf_modifiers[i].header.x;
    if (modifier_type != SDF_MOD_ROUND && modifier_type != SDF_MOD_BEVEL) {
      return false;
    }
  }
  return true;
}

float sdfObjectDistanceAtPosition(SDFObjectGPU obj, float3 position)
{
  float3 local_pos = (obj.inverse_matrix * float4(position - obj.position.xyz, 1.0f)).xyz;
  return evalPrimitive(local_pos, obj);
}

bool sdfAnalyticWorldNormals(SDFObjectGPU obj,
                             float3 position,
                             out float3 shading_normal,
                             out float3 geometric_normal)
{
  float e = max(sdf_ray_epsilon * 0.5f, 1e-4f);
  float3 ex = float3(e, 0.0f, 0.0f);
  float3 ey = float3(0.0f, e, 0.0f);
  float3 ez = float3(0.0f, 0.0f, e);
  geometric_normal = float3(
                         sdfObjectDistanceAtPosition(obj, position + ex) -
                             sdfObjectDistanceAtPosition(obj, position - ex),
                         sdfObjectDistanceAtPosition(obj, position + ey) -
                             sdfObjectDistanceAtPosition(obj, position - ey),
                         sdfObjectDistanceAtPosition(obj, position + ez) -
                             sdfObjectDistanceAtPosition(obj, position - ez)) /
                     (2.0f * e);
  float normal_len_squared = dot(geometric_normal, geometric_normal);
  if (normal_len_squared <= 1e-12f || any(isnan(geometric_normal))) {
    shading_normal = float3(0.0f);
    geometric_normal = float3(0.0f);
    return false;
  }
  shading_normal = geometric_normal;
  return true;
}

float2 sdfCSGNormalWeights(float distance_a,
                           float distance_b,
                           int operation,
                           int blend_type,
                           float blend,
                           float clearance,
                           float shell_distance,
                           int shell_mode,
                           int shell_op,
                           float shell_blend_top,
                           float shell_blend_bottom,
                           float chamfer_k2,
                           float chamfer_k3,
                           float chamfer_k4,
                           float chamfer_k5,
                           int flip_blend,
                           int flip_blend_end)
{
  if (operation == SDF_CSG_OP_PAINT) {
    return float2(1.0f, 0.0f);
  }

  if (blend_type == SDF_BLEND_TYPE_LINEAR || blend <= 0.0001f) {
    if (operation == SDF_CSG_OP_UNION) {
      return distance_a <= distance_b ? float2(1.0f, 0.0f) : float2(0.0f, 1.0f);
    }
    if (operation == SDF_CSG_OP_SUBTRACT) {
      return distance_a >= -distance_b ? float2(1.0f, 0.0f) : float2(0.0f, -1.0f);
    }
    if (operation == SDF_CSG_OP_INTERSECT) {
      return distance_a >= distance_b ? float2(1.0f, 0.0f) : float2(0.0f, 1.0f);
    }
  }

  if (blend_type == SDF_BLEND_TYPE_SMOOTH && blend > 0.0001f) {
    if (operation == SDF_CSG_OP_UNION) {
      float h = clamp(0.5f + 0.5f * (distance_b - distance_a) / blend, 0.0f, 1.0f);
      return float2(h, 1.0f - h);
    }
    if (operation == SDF_CSG_OP_SUBTRACT) {
      float h = clamp(0.5f - 0.5f * (distance_a + distance_b) / blend, 0.0f, 1.0f);
      return float2(1.0f - h, -h);
    }
    if (operation == SDF_CSG_OP_INTERSECT) {
      float h = clamp(0.5f - 0.5f * (distance_b - distance_a) / blend, 0.0f, 1.0f);
      return float2(h, 1.0f - h);
    }
  }

  float e = max(sdf_ray_epsilon * 0.25f, 1e-5f);
  float weight_a = (combineCSG(distance_a + e,
                               distance_b,
                               operation,
                               blend_type,
                               blend,
                               clearance,
                               shell_distance,
                               shell_mode,
                               shell_op,
                               shell_blend_top,
                               shell_blend_bottom,
                               chamfer_k2,
                               chamfer_k3,
                               chamfer_k4,
                               chamfer_k5,
                               flip_blend,
                               flip_blend_end) -
                    combineCSG(distance_a - e,
                               distance_b,
                               operation,
                               blend_type,
                               blend,
                               clearance,
                               shell_distance,
                               shell_mode,
                               shell_op,
                               shell_blend_top,
                               shell_blend_bottom,
                               chamfer_k2,
                               chamfer_k3,
                               chamfer_k4,
                               chamfer_k5,
                               flip_blend,
                               flip_blend_end)) /
                   (2.0f * e);
  float weight_b = (combineCSG(distance_a,
                               distance_b + e,
                               operation,
                               blend_type,
                               blend,
                               clearance,
                               shell_distance,
                               shell_mode,
                               shell_op,
                               shell_blend_top,
                               shell_blend_bottom,
                               chamfer_k2,
                               chamfer_k3,
                               chamfer_k4,
                               chamfer_k5,
                               flip_blend,
                               flip_blend_end) -
                    combineCSG(distance_a,
                               distance_b - e,
                               operation,
                               blend_type,
                               blend,
                               clearance,
                               shell_distance,
                               shell_mode,
                               shell_op,
                               shell_blend_top,
                               shell_blend_bottom,
                               chamfer_k2,
                               chamfer_k3,
                               chamfer_k4,
                               chamfer_k5,
                               flip_blend,
                               flip_blend_end)) /
                   (2.0f * e);
  return float2(weight_a, weight_b);
}

float2 sdfObjectNormalWeights(float distance_a, float distance_b, SDFObjectGPU obj)
{
  return sdfCSGNormalWeights(distance_a,
                             distance_b,
                             obj.csg_operation,
                             obj.blend_type,
                             obj.blend,
                             obj.clearance,
                             obj.shell_distance,
                             obj.shell_mode,
                             obj.shell_op,
                             obj.shell_blend_top,
                             obj.shell_blend_bottom,
                             obj.chamfer_k2,
                             obj.chamfer_k3,
                             obj.chamfer_k4,
                             obj.chamfer_k5,
                             obj.flip_blend,
                             obj.flip_blend_end);
}

float2 sdfGroupNormalWeights(float distance_a, float distance_b, SDFGroupGPU group)
{
  return sdfCSGNormalWeights(distance_a,
                             distance_b,
                             group.csg_operation,
                             group.blend_type,
                             group.blend,
                             group.clearance,
                             group.shell_distance,
                             group.shell_mode,
                             group.shell_op,
                             group.shell_blend_top,
                             group.shell_blend_bottom,
                             group.chamfer_k2,
                             group.chamfer_k3,
                             group.chamfer_k4,
                             group.chamfer_k5,
                             group.flip_blend,
                             group.flip_blend_end);
}

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
    imageStore(gbuf_normal_img, pixel, float4(0.0));
    return;
  }

  float3 world_pos = gbuf.xyz;
  float obj_id = imageLoad(gbuf_color_img, pixel).a;

  float3 out_color = float3(0.5f);
  float scene_dist = 1e10f;
  float3 scene_normal = float3(0.0f);
  float3 scene_gradient = float3(0.0f);
  bool scene_normal_valid = false;
  bool scene_gradient_valid = false;

  int cur_group = -2;
  float grp_dist = 1e10f;
  float3 grp_color = float3(0.5f);
  bool grp_has_hit = false;
  float3 grp_normal = float3(0.0f);
  float3 grp_gradient = float3(0.0f);
  bool grp_normal_valid = false;
  bool grp_gradient_valid = false;
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
        grp_normal_valid = false;
        grp_gradient_valid = false;
      }
      if (scene_dist >= 1e9f) {
        scene_normal = grp_normal;
        scene_gradient = grp_gradient;
        scene_normal_valid = grp_normal_valid;
        scene_gradient_valid = grp_gradient_valid;
      }
      else {
        SDFGroupGPU group = groups[cur_group];
        float2 weights = sdfGroupNormalWeights(scene_dist, grp_dist, group);
        float3 combined_normal;
        float3 combined_gradient;
        bool combined_normal_valid = sdfBlendNormalContributions(weights.x,
                                                                 weights.y,
                                                                 scene_normal,
                                                                 scene_normal_valid,
                                                                 grp_normal,
                                                                 grp_normal_valid,
                                                                 combined_normal);
        bool combined_gradient_valid = sdfBlendNormalContributions(weights.x,
                                                                   weights.y,
                                                                   scene_gradient,
                                                                   scene_gradient_valid,
                                                                   grp_gradient,
                                                                   grp_gradient_valid,
                                                                   combined_gradient);
        scene_normal = combined_normal;
        scene_gradient = combined_gradient;
        scene_normal_valid = combined_normal_valid;
        scene_gradient_valid = combined_gradient_valid;
      }
      flushGroup(cur_group, grp_dist, grp_color, scene_dist, out_color);
      grp_has_hit = false;
      grp_dist = 1e10f;
      grp_color = (gid >= 0) ? groups[gid].color.rgb : float3(0.5f);
      grp_normal = float3(0.0f);
      grp_gradient = float3(0.0f);
      grp_normal_valid = false;
      grp_gradient_valid = false;
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

    int obj_op = obj.csg_operation;
    bool must_eval = (obj_op == SDF_CSG_OP_INTERSECT || obj_op == SDF_CSG_OP_SUBTRACT ||
                      obj_op == SDF_CSG_OP_PUSH) &&
                     ((gid >= 0 && grp_has_hit && gid == cur_group) ||
                      (gid < 0 && scene_dist < 1e9f));

    float d;
    if (da > aabb.max_group_blend && !must_eval) {
      cur_group = gid;
      continue;
    }
    float3 lp = (obj.inverse_matrix * float4(eval_pos - obj.position.xyz, 1.0f)).xyz;
    g_sdf_mesh_last_triangle = -1;
    d = evalPrimitive(lp, obj);
    float3 obj_normal = float3(0.0f);
    float3 obj_gradient = float3(0.0f);
    bool obj_normal_valid;
    if (obj.sdf_type == SDF_GPU_TYPE_MESH && sdfMeshModifiersPreserveNormal(obj)) {
      obj_normal_valid = sdfMeshLastWorldNormals(obj, obj_normal, obj_gradient);
    }
    else if (obj.sdf_type != SDF_GPU_TYPE_MESH) {
      obj_normal_valid = sdfAnalyticWorldNormals(obj, eval_pos, obj_normal, obj_gradient);
    }
    else {
      obj_normal_valid = false;
    }
    cur_group = gid;

    if (gid < 0) {
      if (scene_dist >= 1e9f) {
        if (obj.csg_operation == 0) {
          scene_dist = d;
          out_color = obj.color.rgb;
          scene_normal = obj_normal;
          scene_gradient = obj_gradient;
          scene_normal_valid = obj_normal_valid;
          scene_gradient_valid = obj_normal_valid;
        }
      }
      else {
        float prev = scene_dist;
        float2 weights = sdfObjectNormalWeights(prev, d, obj);
        float3 combined_normal;
        float3 combined_gradient;
        bool combined_normal_valid = sdfBlendNormalContributions(weights.x,
                                                                 weights.y,
                                                                 scene_normal,
                                                                 scene_normal_valid,
                                                                 obj_normal,
                                                                 obj_normal_valid,
                                                                 combined_normal);
        bool combined_gradient_valid = sdfBlendNormalContributions(weights.x,
                                                                   weights.y,
                                                                   scene_gradient,
                                                                   scene_gradient_valid,
                                                                   obj_gradient,
                                                                   obj_normal_valid,
                                                                   combined_gradient);
        scene_dist = combineCSG(
            scene_dist, d, obj.csg_operation, obj.blend_type, obj.blend, obj.clearance,
            obj.shell_distance, obj.shell_mode, obj.shell_op, obj.shell_blend_top, obj.shell_blend_bottom, obj.chamfer_k2, obj.chamfer_k3, obj.chamfer_k4, obj.chamfer_k5, obj.flip_blend, obj.flip_blend_end);
        float t = csgColorFactor(prev, d, scene_dist, obj.csg_operation, obj.color_blend, obj.clearance,
                                 obj.shell_distance, obj.shell_op);
        out_color = blendSDFColor(out_color, obj.color.rgb, t, obj.color_blend_type);
        scene_normal = combined_normal;
        scene_gradient = combined_gradient;
        scene_normal_valid = combined_normal_valid;
        scene_gradient_valid = combined_gradient_valid;
      }
    }
    else {
      float3 grp_tint = groups[gid].color.rgb;
      if (!grp_has_hit) {
        if (obj.csg_operation != SDF_CSG_OP_SUBTRACT && obj.csg_operation != SDF_CSG_OP_SHELL &&
            obj.csg_operation != SDF_CSG_OP_INTERSECT && obj.csg_operation != SDF_CSG_OP_PAINT) {
          grp_dist = d;
          grp_color = obj.color.rgb * grp_tint;
          grp_normal = obj_normal;
          grp_gradient = obj_gradient;
          grp_normal_valid = obj_normal_valid;
          grp_gradient_valid = obj_normal_valid;
          grp_has_hit = true;
        }
      }
      else {
        float prev = grp_dist;
        float2 weights = sdfObjectNormalWeights(prev, d, obj);
        float3 combined_normal;
        float3 combined_gradient;
        bool combined_normal_valid = sdfBlendNormalContributions(weights.x,
                                                                 weights.y,
                                                                 grp_normal,
                                                                 grp_normal_valid,
                                                                 obj_normal,
                                                                 obj_normal_valid,
                                                                 combined_normal);
        bool combined_gradient_valid = sdfBlendNormalContributions(weights.x,
                                                                   weights.y,
                                                                   grp_gradient,
                                                                   grp_gradient_valid,
                                                                   obj_gradient,
                                                                   obj_normal_valid,
                                                                   combined_gradient);
        grp_dist = combineCSG(
            grp_dist, d, obj.csg_operation, obj.blend_type, obj.blend, obj.clearance,
            obj.shell_distance, obj.shell_mode, obj.shell_op, obj.shell_blend_top, obj.shell_blend_bottom, obj.chamfer_k2, obj.chamfer_k3, obj.chamfer_k4, obj.chamfer_k5, obj.flip_blend, obj.flip_blend_end);
        float t = csgColorFactor(prev, d, grp_dist, obj.csg_operation, obj.color_blend, obj.clearance,
                                 obj.shell_distance, obj.shell_op);
        grp_color = blendSDFColor(grp_color, obj.color.rgb * grp_tint, t, obj.color_blend_type);
        grp_normal = combined_normal;
        grp_gradient = combined_gradient;
        grp_normal_valid = combined_normal_valid;
        grp_gradient_valid = combined_gradient_valid;
      }
    }
  }

  if (grp_has_hit) {
    if (cur_group >= 0 && groups[cur_group].modifier_count > 0) {
      grp_dist = applyGroupDistanceModifiers(grp_dist, grp_pos, groups[cur_group].modifier_start, groups[cur_group].modifier_count);
      grp_dist *= grp_scale;
      grp_normal_valid = false;
      grp_gradient_valid = false;
    }
    if (scene_dist >= 1e9f) {
      scene_normal = grp_normal;
      scene_gradient = grp_gradient;
      scene_normal_valid = grp_normal_valid;
      scene_gradient_valid = grp_gradient_valid;
    }
    else {
      SDFGroupGPU group = groups[cur_group];
      float2 weights = sdfGroupNormalWeights(scene_dist, grp_dist, group);
      float3 combined_normal;
      float3 combined_gradient;
      bool combined_normal_valid = sdfBlendNormalContributions(weights.x,
                                                               weights.y,
                                                               scene_normal,
                                                               scene_normal_valid,
                                                               grp_normal,
                                                               grp_normal_valid,
                                                               combined_normal);
      bool combined_gradient_valid = sdfBlendNormalContributions(weights.x,
                                                                 weights.y,
                                                                 scene_gradient,
                                                                 scene_gradient_valid,
                                                                 grp_gradient,
                                                                 grp_gradient_valid,
                                                                 combined_gradient);
      scene_normal = combined_normal;
      scene_gradient = combined_gradient;
      scene_normal_valid = combined_normal_valid;
      scene_gradient_valid = combined_gradient_valid;
    }
    flushGroup(cur_group, grp_dist, grp_color, scene_dist, out_color);
  }

  if (scene_gradient_valid) {
    float gradient_len_squared = dot(scene_gradient, scene_gradient);
    scene_gradient_valid = gradient_len_squared > 1e-12f && !any(isnan(scene_gradient));
    if (scene_gradient_valid) {
      if (!scene_normal_valid) {
        scene_normal = scene_gradient;
        scene_normal_valid = true;
      }
      else {
        float alignment = dot(scene_normal, scene_gradient) / gradient_len_squared;
        if (alignment < 0.0f) {
          scene_normal -= 2.0f * alignment * scene_gradient;
        }
      }
    }
  }
  if (!scene_gradient_valid) {
    scene_normal_valid = false;
  }
  if (scene_normal_valid) {
    float normal_len_squared = dot(scene_normal, scene_normal);
    scene_normal_valid = normal_len_squared > 1e-12f && !any(isnan(scene_normal));
    if (scene_normal_valid) {
      scene_normal *= inversesqrt(normal_len_squared);
    }
  }

  imageStore(gbuf_color_img, pixel, float4(out_color, obj_id));
  imageStore(gbuf_normal_img,
             pixel,
             scene_normal_valid ? float4(scene_normal, 1.0f) : float4(0.0f));
}
