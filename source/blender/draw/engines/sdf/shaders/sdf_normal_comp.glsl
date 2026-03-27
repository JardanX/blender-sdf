/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Normal pass: fused tetrahedron CD — single object loop, 4 evals per object. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_normal_comp)

#include "sdf_lib.glsl"

shared int s_candidates[kMaxTileObjects];
shared int s_numCandidates;

void main()
{
  int2 pixel = int2(gl_GlobalInvocationID.xy);

  /* Cooperative tile list load. */
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
    imageStore(gbuf_normal_img, pixel, float4(0.0f));
    return;
  }

  float3 p = gbuf.xyz;
  float eps = sdf_ray_epsilon * 5.0f;
  float margin = eps * 1.5f;

  const float2 k = float2(1.0f, -1.0f);
  float3 s0 = p + k.xyy * eps;
  float3 s1 = p + k.yyx * eps;
  float3 s2 = p + k.yxy * eps;
  float3 s3 = p + k.xxx * eps;

  float sd0 = 1e10f, sd1 = 1e10f, sd2 = 1e10f, sd3 = 1e10f;
  int cur_group = -2;
  float gd0 = 1e10f, gd1 = 1e10f, gd2 = 1e10f, gd3 = 1e10f;
  bool grp_has_hit = false;

  for (int u = 0; u < nc; u++) {
    int i = s_candidates[u];
    int gid = objects[i].group_id;

    if (gid != cur_group && grp_has_hit) {
      if (sd0 >= 1e9f) { sd0 = gd0; sd1 = gd1; sd2 = gd2; sd3 = gd3; }
      else {
        SDFGroupGPU grp = groups[cur_group];
        sd0 = combineCSG(sd0, gd0, grp.csg_operation, grp.blend_type, grp.blend, grp.shell_distance, grp.shell_mode, grp.shell_op, grp.shell_blend_top, grp.shell_blend_bottom, grp.chamfer_k2, grp.chamfer_k3);
        sd1 = combineCSG(sd1, gd1, grp.csg_operation, grp.blend_type, grp.blend, grp.shell_distance, grp.shell_mode, grp.shell_op, grp.shell_blend_top, grp.shell_blend_bottom, grp.chamfer_k2, grp.chamfer_k3);
        sd2 = combineCSG(sd2, gd2, grp.csg_operation, grp.blend_type, grp.blend, grp.shell_distance, grp.shell_mode, grp.shell_op, grp.shell_blend_top, grp.shell_blend_bottom, grp.chamfer_k2, grp.chamfer_k3);
        sd3 = combineCSG(sd3, gd3, grp.csg_operation, grp.blend_type, grp.blend, grp.shell_distance, grp.shell_mode, grp.shell_op, grp.shell_blend_top, grp.shell_blend_bottom, grp.chamfer_k2, grp.chamfer_k3);
      }
      grp_has_hit = false;
      gd0 = 1e10f; gd1 = 1e10f; gd2 = 1e10f; gd3 = 1e10f;
    }

    float da = point_aabb_dist(p, objects[i].bbox_min.xyz, objects[i].bbox_max.xyz);
    float skip = max(margin, intBitsToFloat(objects[i]._pad3) + margin);
    if (da > skip) {
      cur_group = gid;
      continue;
    }

    SDFObjectGPU obj = objects[i];
    float3 lp0 = (obj.inverse_matrix * float4(s0 - obj.position.xyz, 1.0f)).xyz;
    float3 lp1 = (obj.inverse_matrix * float4(s1 - obj.position.xyz, 1.0f)).xyz;
    float3 lp2 = (obj.inverse_matrix * float4(s2 - obj.position.xyz, 1.0f)).xyz;
    float3 lp3 = (obj.inverse_matrix * float4(s3 - obj.position.xyz, 1.0f)).xyz;
    float d0 = evalPrimitive(lp0, obj);
    float d1 = evalPrimitive(lp1, obj);
    float d2 = evalPrimitive(lp2, obj);
    float d3 = evalPrimitive(lp3, obj);
    cur_group = gid;

    if (gid < 0) {
      if (sd0 >= 1e9f) { sd0 = d0; sd1 = d1; sd2 = d2; sd3 = d3; }
      else {
        sd0 = combineCSG(sd0, d0, obj.csg_operation, obj.blend_type, obj.blend, obj.shell_distance, obj.shell_mode, obj.shell_op, obj.shell_blend_top, obj.shell_blend_bottom, obj.chamfer_k2, obj.chamfer_k3);
        sd1 = combineCSG(sd1, d1, obj.csg_operation, obj.blend_type, obj.blend, obj.shell_distance, obj.shell_mode, obj.shell_op, obj.shell_blend_top, obj.shell_blend_bottom, obj.chamfer_k2, obj.chamfer_k3);
        sd2 = combineCSG(sd2, d2, obj.csg_operation, obj.blend_type, obj.blend, obj.shell_distance, obj.shell_mode, obj.shell_op, obj.shell_blend_top, obj.shell_blend_bottom, obj.chamfer_k2, obj.chamfer_k3);
        sd3 = combineCSG(sd3, d3, obj.csg_operation, obj.blend_type, obj.blend, obj.shell_distance, obj.shell_mode, obj.shell_op, obj.shell_blend_top, obj.shell_blend_bottom, obj.chamfer_k2, obj.chamfer_k3);
      }
    }
    else {
      if (!grp_has_hit) {
        gd0 = d0; gd1 = d1; gd2 = d2; gd3 = d3;
        grp_has_hit = true;
      }
      else {
        gd0 = combineCSG(gd0, d0, obj.csg_operation, obj.blend_type, obj.blend, obj.shell_distance, obj.shell_mode, obj.shell_op, obj.shell_blend_top, obj.shell_blend_bottom, obj.chamfer_k2, obj.chamfer_k3);
        gd1 = combineCSG(gd1, d1, obj.csg_operation, obj.blend_type, obj.blend, obj.shell_distance, obj.shell_mode, obj.shell_op, obj.shell_blend_top, obj.shell_blend_bottom, obj.chamfer_k2, obj.chamfer_k3);
        gd2 = combineCSG(gd2, d2, obj.csg_operation, obj.blend_type, obj.blend, obj.shell_distance, obj.shell_mode, obj.shell_op, obj.shell_blend_top, obj.shell_blend_bottom, obj.chamfer_k2, obj.chamfer_k3);
        gd3 = combineCSG(gd3, d3, obj.csg_operation, obj.blend_type, obj.blend, obj.shell_distance, obj.shell_mode, obj.shell_op, obj.shell_blend_top, obj.shell_blend_bottom, obj.chamfer_k2, obj.chamfer_k3);
      }
    }
  }

  if (grp_has_hit) {
    if (sd0 >= 1e9f) { sd0 = gd0; sd1 = gd1; sd2 = gd2; sd3 = gd3; }
    else {
      SDFGroupGPU grp = groups[cur_group];
      sd0 = combineCSG(sd0, gd0, grp.csg_operation, grp.blend_type, grp.blend, grp.shell_distance, grp.shell_mode, grp.shell_op, grp.shell_blend_top, grp.shell_blend_bottom, grp.chamfer_k2, grp.chamfer_k3);
      sd1 = combineCSG(sd1, gd1, grp.csg_operation, grp.blend_type, grp.blend, grp.shell_distance, grp.shell_mode, grp.shell_op, grp.shell_blend_top, grp.shell_blend_bottom, grp.chamfer_k2, grp.chamfer_k3);
      sd2 = combineCSG(sd2, gd2, grp.csg_operation, grp.blend_type, grp.blend, grp.shell_distance, grp.shell_mode, grp.shell_op, grp.shell_blend_top, grp.shell_blend_bottom, grp.chamfer_k2, grp.chamfer_k3);
      sd3 = combineCSG(sd3, gd3, grp.csg_operation, grp.blend_type, grp.blend, grp.shell_distance, grp.shell_mode, grp.shell_op, grp.shell_blend_top, grp.shell_blend_bottom, grp.chamfer_k2, grp.chamfer_k3);
    }
  }

  float3 n = k.xyy * sd0 + k.yyx * sd1 + k.yxy * sd2 + k.xxx * sd3;
  float nl = length(n);
  n = nl > 1e-8f ? n / nl : float3(0.0f, 0.0f, 1.0f);
  if (any(isnan(n))) { n = float3(0.0f, 0.0f, 1.0f); }

  imageStore(gbuf_normal_img, pixel, float4(n, 1.0f));
}
