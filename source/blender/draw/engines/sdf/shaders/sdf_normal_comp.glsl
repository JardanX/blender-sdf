/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Normal pass: analytical SDF gradients with tile-based iteration.
 * Each primitive returns float4(dist, grad.xyz) in a single eval.
 * Correct for all CSG blend operations. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_normal_comp)

#include "sdf_lib.glsl"
#include "sdf_grad_lib.glsl"

shared int s_candidates[kMaxTileObjects];
shared int s_numCandidates;

/* Evaluate primitive gradient in local space + apply distance modifiers. */
float4 evalLocalGrad(float3 lp, SDFObjectGPU obj, float scale)
{
  float4 dg = evalPrimitiveGrad(lp, obj, sdf_ray_epsilon);
  if (obj.modifier_count > 0) {
    dg = applyDistanceModifiersGrad(dg, obj.modifier_start, obj.modifier_count);
  }
  dg.x *= scale;
  return dg;
}

/* Reflect gradient through mirrors applied in a forking cell. */
float3 reflectForkMirrorGrad(float3 grad, int ci, int mflags,
                             float3 N_x, float3 N_y, float3 N_z)
{
  int flip_idx = ci;
  if ((mflags & SDF_MOD_MIRROR_X) != 0) {
    if ((flip_idx & 1) != 0) {
      grad -= 2.0f * dot(grad, N_x) * N_x;
    }
    flip_idx >>= 1;
  }
  if ((mflags & SDF_MOD_MIRROR_Y) != 0) {
    if ((flip_idx & 1) != 0) {
      grad -= 2.0f * dot(grad, N_y) * N_y;
    }
    flip_idx >>= 1;
  }
  if ((mflags & SDF_MOD_MIRROR_Z) != 0) {
    if ((flip_idx & 1) != 0) {
      grad -= 2.0f * dot(grad, N_z) * N_z;
    }
  }
  return grad;
}

/* Evaluate one object's analytical gradient in world space.
 * Mirrors evalObjectSDF: handles forking (mirror/array with CSG blending). */
float4 evalObjectGrad(float3 world_pos, int i)
{
  SDFObjectGPU obj = objects[i];
  float3 off = obj.position.xyz;
  float4x4 inv = obj.inverse_matrix;

  /* Fast path: no modifiers (most common case).
   * Skip fork detection, domain mods, invertDomainModifiersGrad. */
  if (obj.modifier_count == 0) {
    float3 lp = (inv * float4(world_pos - off, 1.0f)).xyz;
    float4 dg = evalPrimitiveGrad(lp, obj, sdf_ray_epsilon);
    float3x3 inv3 = float3x3(inv[0].xyz, inv[1].xyz, inv[2].xyz);
    float3 gw = transpose(inv3) * dg.yzw;
    float gl = max(length(gw), 1e-8f);
    return float4(dg.x, gw / gl);
  }

  /* Transform to local space */
  float3 p = (inv * float4(world_pos - off, 1.0f)).xyz;
  float3 orig_p = p;
  float4 dg;
  float domain_scale = 1.0f;
  if (obj.modifier_count > 0) {
    float4 dm = applyDomainModifiers(p, obj.modifier_start, obj.modifier_count, inv);
    p = dm.xyz;
    domain_scale = dm.w;
  }
  dg = evalLocalGrad(p, obj, domain_scale);
  if (obj.modifier_count > 0) {
    dg.yzw = invertDomainModifiersGrad(dg.yzw, orig_p,
                                       obj.modifier_start, obj.modifier_count, inv);
  }

  /* Transform gradient from local to world space */
  float3x3 inv3 = float3x3(inv[0].xyz, inv[1].xyz, inv[2].xyz);
  float3 gw = transpose(inv3) * dg.yzw;
  float gl = max(length(gw), 1e-8f);

  return float4(dg.x, gw / gl);
}

void main()
{
  int2 pixel = int2(gl_GlobalInvocationID.xy);

  /* Load tile candidates into shared memory */
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
  if (gbuf.w < 0.5f) {
    imageStore(gbuf_normal_img, pixel, float4(0.0f));
    return;
  }

  float3 p = gbuf.xyz;
  float margin = sdf_ray_epsilon * 1.5f;

  float4 scene_dg = float4(1e10f, 0.0f, 0.0f, 1.0f);

  int cur_group = -2;
  float4 grp_dg = float4(1e10f, 0.0f, 0.0f, 1.0f);
  bool grp_has_hit = false;

  for (int u = 0; u < nc; u++) {
    int i = s_candidates[u];
    SDFObjectAABB n_aabb = object_aabbs[i];
    int gid = n_aabb.group_id;

    if (gid != cur_group && grp_has_hit) {
      if (scene_dg.x >= 1e9f) {
        scene_dg = grp_dg;
      }
      else {
        SDFGroupGPU grp = groups[cur_group];
        scene_dg = combineCSGGrad(scene_dg, grp_dg,
                                  grp.csg_operation, grp.blend_type, grp.blend,
                                  grp.shell_distance, grp.shell_mode, grp.shell_op,
                                  grp.shell_blend_top, grp.shell_blend_bottom,
                                  grp.chamfer_k2, grp.chamfer_k3);
      }
      grp_has_hit = false;
      grp_dg = float4(1e10f, 0.0f, 0.0f, 1.0f);
    }

    SDFObjectGPU obj = objects[i];
    float da = point_aabb_dist(p, obj.orig_bbox_min.xyz, obj.orig_bbox_max.xyz);
    if (da > max(margin, n_aabb.max_group_blend + margin)) {
      cur_group = gid;
      continue;
    }

    float4 dg = evalObjectGrad(p, i);
    cur_group = gid;

    int cop = objects[i].csg_operation;
    int cbt = objects[i].blend_type;
    float cbl = objects[i].blend;
    float csd = objects[i].shell_distance;
    int csm = objects[i].shell_mode;
    int cso = objects[i].shell_op;
    float cst = objects[i].shell_blend_top;
    float csb = objects[i].shell_blend_bottom;
    float ck2 = objects[i].chamfer_k2;
    float ck3 = objects[i].chamfer_k3;

    if (gid < 0) {
      if (scene_dg.x >= 1e9f) {
        if (cop == SDF_CSG_OP_UNION) { scene_dg = dg; }
      }
      else if (cop == SDF_CSG_OP_UNION && cbl <= 0.0001f) {
        /* Sharp union fast path: skip combineCSGGrad entirely. */
        scene_dg = (dg.x < scene_dg.x) ? dg : scene_dg;
      }
      else {
        scene_dg = combineCSGGrad(scene_dg, dg, cop, cbt, cbl, csd, csm, cso, cst, csb, ck2, ck3);
      }
    }
    else {
      if (!grp_has_hit) {
        if (cop != SDF_CSG_OP_SUBTRACT && cop != SDF_CSG_OP_SHELL &&
            cop != SDF_CSG_OP_INTERSECT) {
          grp_dg = dg;
          grp_has_hit = true;
        }
      }
      else if (cop == SDF_CSG_OP_UNION && cbl <= 0.0001f) {
        grp_dg = (dg.x < grp_dg.x) ? dg : grp_dg;
      }
      else {
        grp_dg = combineCSGGrad(grp_dg, dg, cop, cbt, cbl, csd, csm, cso, cst, csb, ck2, ck3);
      }
    }
  }

  if (grp_has_hit) {
    if (scene_dg.x >= 1e9f) {
      scene_dg = grp_dg;
    }
    else {
      SDFGroupGPU grp = groups[cur_group];
      scene_dg = combineCSGGrad(scene_dg, grp_dg,
                                grp.csg_operation, grp.blend_type, grp.blend,
                                grp.shell_distance, grp.shell_mode, grp.shell_op,
                                grp.shell_blend_top, grp.shell_blend_bottom,
                                grp.chamfer_k2, grp.chamfer_k3);
    }
  }

  float3 n = scene_dg.yzw;
  float nl = length(n);
  n = nl > 1e-8f ? n / nl : float3(0.0f, 0.0f, 1.0f);

  if (any(isnan(n))) {
    n = float3(0.0f, 0.0f, 1.0f);
  }
  imageStore(gbuf_normal_img, pixel, float4(n, 1.0f));
}
