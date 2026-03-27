/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Normal pass: 3-tap coplanar CD + SDF unit-length constraint.
 * Offsets (eps,0,0), (0,eps,0), (-eps,-eps,0) sum to zero (bias cancels).
 * gx, gy exact from samples. gz from |grad|=1 constraint, sign from view ray. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_normal_comp)

#include "draw_view_lib.glsl"
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

  /* 3 coplanar offsets with zero sum. */
  float3 s0 = p + float3(eps, 0.0f, 0.0f);
  float3 s1 = p + float3(0.0f, eps, 0.0f);
  float3 s2 = p + float3(-eps, -eps, 0.0f);

  /* 3 scene distance accumulators. */
  float sd0 = 1e10f, sd1 = 1e10f, sd2 = 1e10f;
  int cur_group = -2;
  float g0 = 1e10f, g1 = 1e10f, g2 = 1e10f;
  bool grp_has_hit = false;

  for (int u = 0; u < nc; u++) {
    int i = s_candidates[u];
    SDFObjectAABB n_aabb = object_aabbs[i];
    int gid = n_aabb.group_id;

    if (gid != cur_group && grp_has_hit) {
      if (sd0 >= 1e9f) { sd0 = g0; sd1 = g1; sd2 = g2; }
      else {
        SDFGroupGPU grp = groups[cur_group];
        sd0 = combineCSG(sd0, g0, grp.csg_operation, grp.blend_type, grp.blend, grp.shell_distance, grp.shell_mode, grp.shell_op, grp.shell_blend_top, grp.shell_blend_bottom, grp.chamfer_k2, grp.chamfer_k3);
        sd1 = combineCSG(sd1, g1, grp.csg_operation, grp.blend_type, grp.blend, grp.shell_distance, grp.shell_mode, grp.shell_op, grp.shell_blend_top, grp.shell_blend_bottom, grp.chamfer_k2, grp.chamfer_k3);
        sd2 = combineCSG(sd2, g2, grp.csg_operation, grp.blend_type, grp.blend, grp.shell_distance, grp.shell_mode, grp.shell_op, grp.shell_blend_top, grp.shell_blend_bottom, grp.chamfer_k2, grp.chamfer_k3);
      }
      grp_has_hit = false;
      g0 = 1e10f; g1 = 1e10f; g2 = 1e10f;
    }

    float da = point_aabb_dist(p, n_aabb.bbox_min.xyz, n_aabb.bbox_max.xyz);
    float skip = max(margin, n_aabb.max_group_blend + margin);
    if (da > skip) {
      cur_group = gid;
      continue;
    }

    float d0, d1, d2;
    float grad_thresh = eps * 50.0f;

    if (da > grad_thresh) {
      /* Medium distance: AABB proxy, skip evalPrimitive entirely.
       * Constant across all samples → zero gradient contribution. */
      d0 = da; d1 = da; d2 = da;
    }
    else {
      /* Close to AABB: need actual SDF evaluation. */
      SDFObjectGPU obj = objects[i];
      float3 obj_off = obj.position.xyz;
      float3 lp_c = (obj.inverse_matrix * float4(p - obj_off, 1.0f)).xyz;
      float dc = evalPrimitive(lp_c, obj);

      if (abs(dc) < max(grad_thresh, n_aabb.max_group_blend)) {
        d0 = evalPrimitive((obj.inverse_matrix * float4(s0 - obj_off, 1.0f)).xyz, obj);
        d1 = evalPrimitive((obj.inverse_matrix * float4(s1 - obj_off, 1.0f)).xyz, obj);
        d2 = evalPrimitive((obj.inverse_matrix * float4(s2 - obj_off, 1.0f)).xyz, obj);
      }
      else {
        d0 = dc; d1 = dc; d2 = dc;
      }
    }
    cur_group = gid;

    if (gid < 0) {
      if (sd0 >= 1e9f) { sd0 = d0; sd1 = d1; sd2 = d2; }
      else {
        sd0 = combineCSG(sd0, d0, obj.csg_operation, obj.blend_type, obj.blend, obj.shell_distance, obj.shell_mode, obj.shell_op, obj.shell_blend_top, obj.shell_blend_bottom, obj.chamfer_k2, obj.chamfer_k3);
        sd1 = combineCSG(sd1, d1, obj.csg_operation, obj.blend_type, obj.blend, obj.shell_distance, obj.shell_mode, obj.shell_op, obj.shell_blend_top, obj.shell_blend_bottom, obj.chamfer_k2, obj.chamfer_k3);
        sd2 = combineCSG(sd2, d2, obj.csg_operation, obj.blend_type, obj.blend, obj.shell_distance, obj.shell_mode, obj.shell_op, obj.shell_blend_top, obj.shell_blend_bottom, obj.chamfer_k2, obj.chamfer_k3);
      }
    }
    else {
      if (!grp_has_hit) {
        g0 = d0; g1 = d1; g2 = d2;
        grp_has_hit = true;
      }
      else {
        g0 = combineCSG(g0, d0, obj.csg_operation, obj.blend_type, obj.blend, obj.shell_distance, obj.shell_mode, obj.shell_op, obj.shell_blend_top, obj.shell_blend_bottom, obj.chamfer_k2, obj.chamfer_k3);
        g1 = combineCSG(g1, d1, obj.csg_operation, obj.blend_type, obj.blend, obj.shell_distance, obj.shell_mode, obj.shell_op, obj.shell_blend_top, obj.shell_blend_bottom, obj.chamfer_k2, obj.chamfer_k3);
        g2 = combineCSG(g2, d2, obj.csg_operation, obj.blend_type, obj.blend, obj.shell_distance, obj.shell_mode, obj.shell_op, obj.shell_blend_top, obj.shell_blend_bottom, obj.chamfer_k2, obj.chamfer_k3);
      }
    }
  }

  if (grp_has_hit) {
    if (sd0 >= 1e9f) { sd0 = g0; sd1 = g1; sd2 = g2; }
    else {
      SDFGroupGPU grp = groups[cur_group];
      sd0 = combineCSG(sd0, g0, grp.csg_operation, grp.blend_type, grp.blend, grp.shell_distance, grp.shell_mode, grp.shell_op, grp.shell_blend_top, grp.shell_blend_bottom, grp.chamfer_k2, grp.chamfer_k3);
      sd1 = combineCSG(sd1, g1, grp.csg_operation, grp.blend_type, grp.blend, grp.shell_distance, grp.shell_mode, grp.shell_op, grp.shell_blend_top, grp.shell_blend_bottom, grp.chamfer_k2, grp.chamfer_k3);
      sd2 = combineCSG(sd2, g2, grp.csg_operation, grp.blend_type, grp.blend, grp.shell_distance, grp.shell_mode, grp.shell_op, grp.shell_blend_top, grp.shell_blend_bottom, grp.chamfer_k2, grp.chamfer_k3);
    }
  }

  /* Recover gx, gy from zero-sum samples (bias-free). */
  float inv3eps = 1.0f / (3.0f * eps);
  float gx = (2.0f * sd0 - sd1 - sd2) * inv3eps;
  float gy = (2.0f * sd1 - sd0 - sd2) * inv3eps;

  /* gz from SDF unit-length constraint. */
  float gz_sq = max(0.0f, 1.0f - gx * gx - gy * gy);
  float gz = sqrt(gz_sq);

  /* Sign from view ray: normal faces toward camera. */
  ViewMatrices vm = drw_view();
  float3 cam_pos = vm.viewinv[3].xyz;
  float3 view_dir = normalize(p - cam_pos);
  /* For ortho: viewinv[2] is the view direction, viewinv[3] may be far away. */
  bool is_ortho = (vm.winmat[3][3] == 1.0f);
  if (is_ortho) {
    view_dir = -normalize(vm.viewinv[2].xyz);
  }
  float sign_test = gx * view_dir.x + gy * view_dir.y + gz * view_dir.z;
  if (sign_test > 0.0f) { gz = -gz; }

  float3 n = float3(gx, gy, gz);
  float nl = length(n);
  n = nl > 1e-8f ? n / nl : float3(0.0f, 0.0f, 1.0f);
  if (any(isnan(n))) { n = float3(0.0f, 0.0f, 1.0f); }

  imageStore(gbuf_normal_img, pixel, float4(n, 1.0f));
}
