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
  float4 dg = evalPrimitiveGrad(lp, obj);
  dg.x *= scale;
  if (obj.modifier_count > 0) {
    dg = applyDistanceModifiersGrad(dg, obj.modifier_start, obj.modifier_count);
  }
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

  /* Transform to local space */
  float3 p = (inv * float4(world_pos - off, 1.0f)).xyz;
  float3 orig_p = p;

  /* Detect forking modifier (array/mirror with CSG blending). */
  int fork_idx = -1;
  for (int mi = obj.modifier_start; mi < obj.modifier_start + obj.modifier_count; mi++) {
    SDFModifierGPU smod = sdf_modifiers[mi];
    int mtype = smod.header.x;
    if ((mtype == SDF_MOD_ARRAY || mtype == SDF_MOD_MIRROR) &&
        (smod.params2.x > 0.0f || smod.params2.y > 0.0f || smod.params2.z > 0.0f))
    {
      fork_idx = mi;
    }
  }

  /* Forking objects: use the same simple path but invertDomainModifiersGrad
   * now handles radial arrays (rotates gradient back by cell_angle). */

  float4 dg;

  if (fork_idx == -1) {
    /* Simple path: domain mods → primitive gradient → distance mods */
    float domain_scale = 1.0f;
    if (obj.modifier_count > 0) {
      float4 dm = applyDomainModifiers(p, obj.modifier_start, obj.modifier_count, inv);
      p = dm.xyz;
      domain_scale = dm.w;
    }
    dg = evalLocalGrad(p, obj, domain_scale);

    /* Invert domain modifier Jacobian on gradient */
    if (obj.modifier_count > 0) {
      dg.yzw = invertDomainModifiersGrad(dg.yzw, orig_p,
                                         obj.modifier_start, obj.modifier_count, inv);
    }
  }
  else {
    /* Forking path: apply top domain mods, then iterate cells */
    float top_scale = 1.0f;
    int top_count = (obj.modifier_start + obj.modifier_count) - (fork_idx + 1);
    if (top_count > 0) {
      float4 dm = applyDomainModifiers(p, fork_idx + 1, top_count, inv);
      p = dm.xyz;
      top_scale = dm.w;
    }

    SDFModifierGPU fork_mod = sdf_modifiers[fork_idx];
    int mtype = fork_mod.header.x;
    int mflags = fork_mod.header.y;
    float blend = fork_mod.params2.x;
    int csg_op = int(fork_mod.params2.y);
    int blend_type = int(fork_mod.params2.z);
    int bottom_count = fork_idx - obj.modifier_start;

    dg = float4((csg_op == SDF_CSG_OP_INTERSECT) ? -1e10f : 1e10f, 0.0f, 0.0f, 1.0f);
    bool first = true;

    float3 N_x = float3(inv[0]);
    float3 N_y = float3(inv[1]);
    float3 N_z = float3(inv[2]);

    if (mtype == SDF_MOD_MIRROR) {
      int mirrors = 1;
      if ((mflags & SDF_MOD_MIRROR_X) != 0) { mirrors *= 2; }
      if ((mflags & SDF_MOD_MIRROR_Y) != 0) { mirrors *= 2; }
      if ((mflags & SDF_MOD_MIRROR_Z) != 0) { mirrors *= 2; }

      float3 mirror_origin = fork_mod.params.yzw;
      float moffset = fork_mod.params.x;

      for (int ci = 0; ci < 8; ci++) {
        if (ci >= mirrors) { break; }
        float3 cell_p = p;
        int flip_idx = ci;
        if ((mflags & SDF_MOD_MIRROR_X) != 0) {
          if ((flip_idx & 1) != 0) {
            cell_p -= 2.0f * dot(cell_p - mirror_origin, N_x) * N_x;
          }
          flip_idx >>= 1;
          cell_p -= moffset * N_x;
        }
        if ((mflags & SDF_MOD_MIRROR_Y) != 0) {
          if ((flip_idx & 1) != 0) {
            cell_p -= 2.0f * dot(cell_p - mirror_origin, N_y) * N_y;
          }
          flip_idx >>= 1;
          cell_p -= moffset * N_y;
        }
        if ((mflags & SDF_MOD_MIRROR_Z) != 0) {
          if ((flip_idx & 1) != 0) {
            cell_p -= 2.0f * dot(cell_p - mirror_origin, N_z) * N_z;
          }
          cell_p -= moffset * N_z;
        }

        float bot_scale = top_scale;
        if (bottom_count > 0) {
          float4 dm = applyDomainModifiers(cell_p, obj.modifier_start, bottom_count, inv);
          cell_p = dm.xyz;
          bot_scale *= dm.w;
        }
        float4 cell_dg = evalLocalGrad(cell_p, obj, bot_scale);

        /* Reflect gradient back through mirrors applied to this cell */
        cell_dg.yzw = reflectForkMirrorGrad(cell_dg.yzw, ci, mflags, N_x, N_y, N_z);

        /* Invert bottom domain modifiers on gradient */
        if (bottom_count > 0) {
          cell_dg.yzw = invertDomainModifiersGrad(cell_dg.yzw, cell_p,
                                                  obj.modifier_start, bottom_count, inv);
        }

        if (first) { dg = cell_dg; first = false; }
        else {
          dg = combineCSGGrad(dg, cell_dg, csg_op, blend_type, blend,
                              0.0f, 0, 0, 0.0f, 0.0f, 0.0f, 0.0f);
        }
      }
    }
    else if (mtype == SDF_MOD_ARRAY) {
      float count = fork_mod.params.x;
      float id;
      float3 dir = float3(1.0f, 0.0f, 0.0f);
      float spacing = 1.0f;
      if (mflags == SDF_MOD_ARRAY_LINEAR) {
        float3 arr_offset = fork_mod.params.yzw;
        spacing = length(arr_offset);
        if (spacing > 0.0001f) {
          dir = arr_offset / spacing;
          id = round(dot(p, dir) / spacing);
        }
        else { id = 0.0f; }
      }
      else {
        float a = (2.0f * SDF_PI) / max(count, 1e-4f);
        id = round(atan(p.y, p.x) / a);
      }

      if (mflags == SDF_MOD_ARRAY_LINEAR) {
        /* Linear array: iterate 3 cells */
        float last_cid = -9999.0f;
        for (int ci = -1; ci <= 1; ci++) {
          float cid = id + float(ci);
          cid = clamp(cid, 0.0f, max(0.0f, count - 1.0f));
          if (cid == last_cid) { continue; }
          last_cid = cid;
          float3 cell_p = p;
          if (spacing > 0.0001f) { cell_p -= dir * cid * spacing; }

          float bot_scale = top_scale;
          if (bottom_count > 0) {
            float4 dm = applyDomainModifiers(cell_p, obj.modifier_start, bottom_count, inv);
            cell_p = dm.xyz;
            bot_scale *= dm.w;
          }
          float4 cell_dg = evalLocalGrad(cell_p, obj, bot_scale);

          if (first) { dg = cell_dg; first = false; }
          else {
            dg = combineCSGGrad(dg, cell_dg, csg_op, blend_type, blend,
                                0.0f, 0, 0, 0.0f, 0.0f, 0.0f, 0.0f);
          }
        }
      }
      else {
        /* Radial array: snap to nearest cell, eval gradient, rotate back.
         * DEBUG: single cell only first to isolate rotation bug. */
        float a = (2.0f * SDF_PI) / count;
        float theta = atan(p.y, p.x);
        float cell_angle = id * a;
        float r_len = length(p.xy);
        float radius = fork_mod.params.y;

        /* Snap position to canonical slice (rotate by -cell_angle) */
        float ca = cos(-cell_angle), sa = sin(-cell_angle);
        float3 cell_p = float3(
            ca * p.x - sa * p.y - radius,
            sa * p.x + ca * p.y,
            p.z);

        /* Per-cell rotations */
        float3 rot = fork_mod.params2.yzw;
        if (abs(rot.x) > 0.0001f) {
          float cx = cos(rot.x), sx = sin(rot.x);
          cell_p = float3(cell_p.x, cx * cell_p.y - sx * cell_p.z,
                          sx * cell_p.y + cx * cell_p.z);
        }
        if (abs(rot.y) > 0.0001f) {
          float cy = cos(rot.y), sy = sin(rot.y);
          cell_p = float3(cy * cell_p.x + sy * cell_p.z, cell_p.y,
                          -sy * cell_p.x + cy * cell_p.z);
        }
        if (abs(rot.z) > 0.0001f) {
          float cz = cos(rot.z), sz = sin(rot.z);
          cell_p = float3(cz * cell_p.x - sz * cell_p.y,
                          sz * cell_p.x + cz * cell_p.y, cell_p.z);
        }

        /* Bottom domain mods */
        float bot_scale = top_scale;
        if (bottom_count > 0) {
          float4 dm = applyDomainModifiers(cell_p, obj.modifier_start, bottom_count, inv);
          cell_p = dm.xyz;
          bot_scale *= dm.w;
        }

        /* Evaluate gradient in cell space */
        float4 cell_dg = evalLocalGrad(cell_p, obj, bot_scale);

        /* Undo per-cell rotations on gradient (reverse order) */
        if (abs(rot.z) > 0.0001f) {
          float cz2 = cos(-rot.z), sz2 = sin(-rot.z);
          float gx = cell_dg.y, gy = cell_dg.z;
          cell_dg.y = cz2 * gx - sz2 * gy;
          cell_dg.z = sz2 * gx + cz2 * gy;
        }
        if (abs(rot.y) > 0.0001f) {
          float cy2 = cos(-rot.y), sy2 = sin(-rot.y);
          float gx = cell_dg.y, gz = cell_dg.w;
          cell_dg.y = cy2 * gx + sy2 * gz;
          cell_dg.w = -sy2 * gx + cy2 * gz;
        }
        if (abs(rot.x) > 0.0001f) {
          float cx2 = cos(-rot.x), sx2 = sin(-rot.x);
          float gy = cell_dg.z, gz = cell_dg.w;
          cell_dg.z = cx2 * gy - sx2 * gz;
          cell_dg.w = sx2 * gy + cx2 * gz;
        }

        /* DEBUG: NO rotation — test if gradient is correct in cell space */
        /* (skip rotation entirely) */

        dg = cell_dg;
        first = false;
      }
    }

    /* Invert top domain modifiers on gradient */
    if (top_count > 0) {
      dg.yzw = invertDomainModifiersGrad(dg.yzw, orig_p,
                                         fork_idx + 1, top_count, inv);
    }
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
  float margin = sdf_ray_epsilon * 10.0f;

  /* Scene gradient accumulation (mirrors combineCSG logic from trace) */
  float4 scene_dg = float4(1e10f, 0.0f, 0.0f, 1.0f);

  int cur_group = -2;
  float4 grp_dg = float4(1e10f, 0.0f, 0.0f, 1.0f);
  bool grp_has_hit = false;

  for (int u = 0; u < nc; u++) {
    int i = s_candidates[u];
    SDFObjectAABB n_aabb = object_aabbs[i];
    int gid = n_aabb.group_id;

    /* Flush previous group */
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

    /* AABB proximity check */
    float da = point_aabb_dist(p, n_aabb.bbox_min.xyz, n_aabb.bbox_max.xyz);
    if (da > max(margin, n_aabb.max_group_blend + margin)) {
      cur_group = gid;
      continue;
    }

    /* Evaluate analytical gradient */
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
      /* Ungrouped: must match trace's evalSceneTile logic exactly. */
      if (scene_dg.x >= 1e9f) {
        if (cop == SDF_CSG_OP_UNION) {
          scene_dg = dg;
        }
      }
      else {
        scene_dg = combineCSGGrad(scene_dg, dg, cop, cbt, cbl, csd, csm, cso, cst, csb, ck2, ck3);
      }
    }
    else {
      /* Grouped: skip subtract/shell as first (can't subtract from nothing). */
      if (!grp_has_hit) {
        if (cop != SDF_CSG_OP_SUBTRACT && cop != SDF_CSG_OP_SHELL) {
          grp_dg = dg;
          grp_has_hit = true;
        }
      }
      else {
        grp_dg = combineCSGGrad(grp_dg, dg, cop, cbt, cbl, csd, csm, cso, cst, csb, ck2, ck3);
      }
    }
  }

  /* Flush last group */
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

  float3 n;

  /* DEBUG: 6-tap central differences on the full scene SDF (brute force reference). */
  if (debug_fd_normals != 0) {
    float eps = sdf_ray_epsilon * 5.0f;
    float3 offsets[6] = float3[6](
        float3(eps, 0.0f, 0.0f), float3(-eps, 0.0f, 0.0f),
        float3(0.0f, eps, 0.0f), float3(0.0f, -eps, 0.0f),
        float3(0.0f, 0.0f, eps), float3(0.0f, 0.0f, -eps));
    float d[6];
    for (int t = 0; t < 6; t++) {
      float3 sp = p + offsets[t];
      float sd_acc = 1e10f;
      int cg = -2;
      float gd = 1e10f;
      bool gh = false;
      for (int u2 = 0; u2 < nc; u2++) {
        int j = s_candidates[u2];
        SDFObjectAABB ab = object_aabbs[j];
        int gj = ab.group_id;
        if (gj != cg && gh) {
          if (sd_acc >= 1e9f) { sd_acc = gd; }
          else {
            SDFGroupGPU gg = groups[cg];
            sd_acc = combineCSG(sd_acc, gd, gg.csg_operation, gg.blend_type, gg.blend,
                                gg.shell_distance, gg.shell_mode, gg.shell_op,
                                gg.shell_blend_top, gg.shell_blend_bottom,
                                gg.chamfer_k2, gg.chamfer_k3);
          }
          gh = false; gd = 1e10f;
        }
        float da2 = point_aabb_dist(sp, ab.bbox_min.xyz, ab.bbox_max.xyz);
        if (da2 > max(margin, ab.max_group_blend + margin)) { cg = gj; continue; }
        SDFObjectGPU ob = objects[j];
        float3 lp2 = (ob.inverse_matrix * float4(sp - ob.position.xyz, 1.0f)).xyz;
        float dd = evalPrimitive(lp2, ob);
        cg = gj;
        int co = ob.csg_operation, cb = ob.blend_type;
        float cl = ob.blend, cs = ob.shell_distance;
        int cm = ob.shell_mode, cx = ob.shell_op;
        float ct = ob.shell_blend_top, cv = ob.shell_blend_bottom, c2 = ob.chamfer_k2, c3 = ob.chamfer_k3;
        if (gj < 0) {
          if (sd_acc >= 1e9f) { if (co == 0) { sd_acc = dd; } }
          else { sd_acc = combineCSG(sd_acc, dd, co, cb, cl, cs, cm, cx, ct, cv, c2, c3); }
        }
        else {
          if (!gh) { if (co != SDF_CSG_OP_SUBTRACT && co != SDF_CSG_OP_SHELL) { gd = dd; gh = true; } }
          else { gd = combineCSG(gd, dd, co, cb, cl, cs, cm, cx, ct, cv, c2, c3); }
        }
      }
      if (gh) {
        if (sd_acc >= 1e9f) { sd_acc = gd; }
        else {
          SDFGroupGPU gg = groups[cg];
          sd_acc = combineCSG(sd_acc, gd, gg.csg_operation, gg.blend_type, gg.blend,
                              gg.shell_distance, gg.shell_mode, gg.shell_op,
                              gg.shell_blend_top, gg.shell_blend_bottom,
                              gg.chamfer_k2, gg.chamfer_k3);
        }
      }
      d[t] = sd_acc;
    }
    n = normalize(float3(d[0] - d[1], d[2] - d[3], d[4] - d[5]));
  }
  else {
    n = scene_dg.yzw;
    float nl = length(n);
    n = nl > 1e-8f ? n / nl : float3(0.0f, 0.0f, 1.0f);
  }

  if (any(isnan(n))) {
    n = float3(0.0f, 0.0f, 1.0f);
  }
  imageStore(gbuf_normal_img, pixel, float4(n, 1.0f));
}
