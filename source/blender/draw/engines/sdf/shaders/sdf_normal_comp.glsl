/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Normal pass: fused tetrahedron CD with distance screening.
 * Single object loop: AABB once, struct load once, center eval for screening,
 * 4 tetrahedron evals only for near-surface objects. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_normal_comp)

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
    imageStore(gbuf_normal_img, pixel, float4(0.0f));
    return;
  }

  float3 p = gbuf.xyz;
  float eps = sdf_ray_epsilon * 5.0f;
  float margin = eps * 1.5f;
  float grad_thresh = eps * 50.0f;
  const float2 k = float2(1.0f, -1.0f);

  float sd0 = 1e10f, sd1 = 1e10f, sd2 = 1e10f, sd3 = 1e10f;
  int cur_group = -2;
  float gd0 = 1e10f, gd1 = 1e10f, gd2 = 1e10f, gd3 = 1e10f;
  bool grp_has_hit = false;

  for (int u = 0; u < nc; u++) {
    int i = s_candidates[u];
    SDFObjectAABB n_aabb = object_aabbs[i];
    int gid = n_aabb.group_id;

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

    float da = point_aabb_dist(p, n_aabb.bbox_min.xyz, n_aabb.bbox_max.xyz);
    if (da > max(margin, n_aabb.max_group_blend + margin)) {
      cur_group = gid;
      continue;
    }

    SDFObjectGPU obj = objects[i];
    float3 off = obj.position.xyz;
    float4x4 inv = obj.inverse_matrix;
    float dc = evalPrimitive((inv * float4(p - off, 1.0f)).xyz, obj);

    float d0, d1, d2, d3;
    if (abs(dc) < max(grad_thresh, n_aabb.max_group_blend)) {
      d0 = evalPrimitive((inv * float4(p + k.xyy * eps - off, 1.0f)).xyz, obj);
      d1 = evalPrimitive((inv * float4(p + k.yyx * eps - off, 1.0f)).xyz, obj);
      d2 = evalPrimitive((inv * float4(p + k.yxy * eps - off, 1.0f)).xyz, obj);
      d3 = evalPrimitive((inv * float4(p + k.xxx * eps - off, 1.0f)).xyz, obj);
    }
    else {
      d0 = dc; d1 = dc; d2 = dc; d3 = dc;
    }
    cur_group = gid;

    int cop = obj.csg_operation, cbt = obj.blend_type, csm = obj.shell_mode, cso = obj.shell_op;
    float cbl = obj.blend, csd = obj.shell_distance, cst = obj.shell_blend_top, csb = obj.shell_blend_bottom, ck2 = obj.chamfer_k2, ck3 = obj.chamfer_k3;

    if (gid < 0) {
      if (sd0 >= 1e9f) { sd0 = d0; sd1 = d1; sd2 = d2; sd3 = d3; }
      else {
        sd0 = combineCSG(sd0, d0, cop, cbt, cbl, csd, csm, cso, cst, csb, ck2, ck3);
        sd1 = combineCSG(sd1, d1, cop, cbt, cbl, csd, csm, cso, cst, csb, ck2, ck3);
        sd2 = combineCSG(sd2, d2, cop, cbt, cbl, csd, csm, cso, cst, csb, ck2, ck3);
        sd3 = combineCSG(sd3, d3, cop, cbt, cbl, csd, csm, cso, cst, csb, ck2, ck3);
      }
    }
    else {
      if (!grp_has_hit) {
        gd0 = d0; gd1 = d1; gd2 = d2; gd3 = d3;
        grp_has_hit = true;
      }
      else {
        gd0 = combineCSG(gd0, d0, cop, cbt, cbl, csd, csm, cso, cst, csb, ck2, ck3);
        gd1 = combineCSG(gd1, d1, cop, cbt, cbl, csd, csm, cso, cst, csb, ck2, ck3);
        gd2 = combineCSG(gd2, d2, cop, cbt, cbl, csd, csm, cso, cst, csb, ck2, ck3);
        gd3 = combineCSG(gd3, d3, cop, cbt, cbl, csd, csm, cso, cst, csb, ck2, ck3);
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
