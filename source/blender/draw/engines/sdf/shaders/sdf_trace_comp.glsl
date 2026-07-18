/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Trace pass: sphere tracing to find hit positions, outputs G-buffer. */

#include "infos/sdf_shader_infos.hh"

#ifdef USE_TILE_CULLING
COMPUTE_SHADER_CREATE_INFO(sdf_trace_tile_comp)
#else
COMPUTE_SHADER_CREATE_INFO(sdf_trace_comp)
#endif

#define kFltMax 1e30f
#define kTileSize 8
#define kMaxTileObjects 256

/* Profiling stat indices (matches SdfTraceStats layout) */
#define STAT_RAYS           0
#define STAT_HITS           1
#define STAT_STEPS          2
#define STAT_EMPTY_STEPS    3
#define STAT_EVALS          4
#define STAT_AABB_SKIPS     5
#define STAT_SOR_FAILURES   6
#define STAT_MAX_STEPS      7

#ifdef USE_TILE_CULLING
shared uint s_tileObjCount;
shared int s_tileObjList[kMaxTileObjects];
shared float4 s_coneHitPos;
#endif

#include "draw_view_lib.glsl"
#include "sdf_lib.glsl"

#define kAabbTreeStackSize 16
#define kMaxBitfieldBits 128
#define kBitfieldWords (kMaxBitfieldBits / 32)

/* Per-ray bitfield: set during ray setup, marks objects whose AABB the ray intersects. */
uint g_bvh_bits[kBitfieldWords];
int g_numNearShapes;
int g_numEvaluated;

bool is_shape_near(int idx)
{
  if (idx >= kMaxBitfieldBits) {
    return true;
  }
  return (g_bvh_bits[idx >> 5] & (1u << (uint(idx) & 31u))) != 0u;
}

float3 find_ortho(float3 v)
{
  if (abs(v.x) >= 0.57735f) {
    return float3(v.y, -v.x, 0.0f);
  }
  return float3(0.0f, v.z, -v.y);
}

/*
 * Combined per-step evaluation + empty-space skip.
 * Iterates ONLY ray-visible objects (from per-ray bitfield).
 * Per-step AABB containment test skips objects whose AABB doesn't contain pos.
 * Returns SDF distance in scene_dist, nearest AABB distance in out_aabb_skip.
 */
float evalSceneBVH(float3 world_pos, out float3 out_color, out float out_aabb_skip, out float out_obj_id)
{
  float scene_dist = 1e10f;
  out_color = float3(0.5f);
  out_aabb_skip = 1e30f;
  out_obj_id = -1.0f;
  g_numEvaluated = 0;

  /* Unified evaluation: iterate all objects in sorted order.
   * Grouped objects are evaluated as a single field, then combined with scene. */
  int i = 0;
  while (i < object_count) {
    int gid = objects[i].group_id;

    if (gid >= 0) {
      /* Evaluate entire group as one field */
      SDFGroupGPU grp = groups[gid];
      float grp_dist = 1e10f;
      float3 grp_tint = grp.color.rgb;
      float3 grp_color = float3(0.5f);
      bool grp_has_hit = false;
      float grp_winner_id = -1.0f;

      float3 grp_pos = world_pos;
      float grp_scale = 1.0f;
      if (grp.modifier_count > 0) {
        float4 dm = applyDomainModifiers(world_pos, grp.modifier_start, grp.modifier_count, float4x4(1.0));
        grp_pos = dm.xyz;
        grp_scale = dm.w;
      }

      int grp_end = i;
      while (grp_end < object_count && objects[grp_end].group_id == gid) { grp_end++; }

      for (int m = i; m < grp_end; m++) {
        if (m == skip_object) { continue; }

        SDFObjectGPU obj = objects[m];
        float d;

        /* Always evaluate group children — no BVH or AABB skip.
         * All children define the combined group field; skipping any member
         * breaks subtract/intersect which need max() over all members. */
        {
          float3 lp = (obj.inverse_matrix * float4(grp_pos - obj.position.xyz, 1.0f)).xyz;
          d = evalPrimitive(lp, obj);
          g_numEvaluated++;
          if (prof_enabled != 0) { atomicAdd(prof_eval_counts[m], 1u); }
        }

        if (!grp_has_hit) {
          if (obj.csg_operation != SDF_CSG_OP_SUBTRACT && obj.csg_operation != SDF_CSG_OP_SHELL &&
              obj.csg_operation != SDF_CSG_OP_INTERSECT) {
            grp_dist = d;
            grp_color = obj.color.rgb * grp_tint;
            grp_winner_id = float(obj.original_index);
            grp_has_hit = true;
          }
        }
        else {
          float prev = grp_dist;
          grp_dist = combineCSG(
              grp_dist, d, obj.csg_operation, obj.blend_type, obj.blend,
              obj.shell_distance, obj.shell_mode, obj.shell_op, obj.shell_blend_top, obj.shell_blend_bottom, obj.chamfer_k2, obj.chamfer_k3, obj.chamfer_k4, obj.chamfer_k5, obj.flip_blend, obj.flip_blend_end);
          if (obj.csg_operation == SDF_CSG_OP_SUBTRACT) {
            if (-d > prev) { grp_winner_id = float(obj.original_index); }
          }
          else if (obj.csg_operation == SDF_CSG_OP_INTERSECT) {
            if (d > prev) { grp_winner_id = float(obj.original_index); }
          }
          else if (obj.csg_operation == SDF_CSG_OP_PUSH) {
            if (d - grp_dist < sdf_ray_epsilon) { grp_winner_id = float(obj.original_index); }
          }
          else if (obj.csg_operation == SDF_CSG_OP_AVOID) {
            if (prev - grp_dist > sdf_ray_epsilon) { grp_winner_id = float(obj.original_index); }
          }
          else if (obj.csg_operation == SDF_CSG_OP_SHELL) {
            if (obj.shell_mode == SDF_SHELL_MODE_PUSH) {
              if (d - grp_dist < sdf_ray_epsilon) { grp_winner_id = float(obj.original_index); }
            }
            else if (obj.shell_mode == SDF_SHELL_MODE_AVOID) {
              if (prev - grp_dist > sdf_ray_epsilon) { grp_winner_id = float(obj.original_index); }
            }
            else if (obj.shell_op == SDF_SHELL_OP_SUBTRACTION) {
              if (-d > prev) { grp_winner_id = float(obj.original_index); }
            }
            else {
              if (d < prev) { grp_winner_id = float(obj.original_index); }
            }
          }
          else {
            if (d < prev) { grp_winner_id = float(obj.original_index); }
          }
          if (obj.csg_operation == SDF_CSG_OP_UNION) {
            float t = colorBlendFactor(prev, d, obj.blend_type, obj.blend);
            grp_color = mix(grp_color, obj.color.rgb * grp_tint, t);
          }
        }
      }

      if (grp_has_hit && grp.modifier_count > 0) {
        grp_dist = applyGroupDistanceModifiers(grp_dist, grp_pos, grp.modifier_start, grp.modifier_count);
        grp_dist *= grp_scale;
      }

      if (!grp_has_hit) {
        float grp_aabb = 1e30f;
        for (int m = i; m < grp_end; m++) {
          grp_aabb = min(grp_aabb, point_aabb_dist(world_pos, objects[m].orig_bbox_min.xyz, objects[m].orig_bbox_max.xyz));
        }
        out_aabb_skip = min(out_aabb_skip, grp_aabb);
      }
      else if (scene_dist >= 1e9f) {
        scene_dist = grp_dist;
        out_color = grp_color;
        out_obj_id = grp_winner_id;
      }
      else {
        float prev = scene_dist;
        scene_dist = combineCSG(
            scene_dist, grp_dist, grp.csg_operation, grp.blend_type, grp.blend,
            grp.shell_distance, grp.shell_mode, grp.shell_op,
            grp.shell_blend_top, grp.shell_blend_bottom,
            grp.chamfer_k2, grp.chamfer_k3, grp.chamfer_k4, grp.chamfer_k5, grp.flip_blend, grp.flip_blend_end);
        if (grp.csg_operation == SDF_CSG_OP_SUBTRACT) {
          if (-grp_dist > prev) { out_obj_id = grp_winner_id; }
        }
        else if (grp.csg_operation == SDF_CSG_OP_INTERSECT) {
          if (grp_dist > prev) { out_obj_id = grp_winner_id; }
        }
        else if (grp.csg_operation == SDF_CSG_OP_PUSH) {
          if (grp_dist - scene_dist < sdf_ray_epsilon) { out_obj_id = grp_winner_id; }
        }
        else if (grp.csg_operation == SDF_CSG_OP_AVOID) {
          if (prev - scene_dist > sdf_ray_epsilon) { out_obj_id = grp_winner_id; }
        }
        else if (grp.csg_operation == SDF_CSG_OP_SHELL) {
          if (grp.shell_mode == SDF_SHELL_MODE_PUSH) {
            if (grp_dist - scene_dist < sdf_ray_epsilon) { out_obj_id = grp_winner_id; }
          }
          else if (grp.shell_mode == SDF_SHELL_MODE_AVOID) {
            if (prev - scene_dist > sdf_ray_epsilon) { out_obj_id = grp_winner_id; }
          }
          else if (grp.shell_op == SDF_SHELL_OP_SUBTRACTION) {
            if (-grp_dist > prev) { out_obj_id = grp_winner_id; }
          }
          else {
            if (grp_dist < prev) { out_obj_id = grp_winner_id; }
          }
        }
        else if (grp.csg_operation == SDF_CSG_OP_UNION) {
          if (grp_dist < prev) { out_obj_id = grp_winner_id; }
        }
        if (grp.csg_operation == SDF_CSG_OP_UNION) {
          float t = colorBlendFactor(prev, grp_dist, grp.blend_type, grp.blend);
          out_color = mix(out_color, grp_color, t);
        }
      }

      i = grp_end;
      continue;
    }

    /* Ungrouped object */
    if (i == skip_object) { i++; continue; }
    if (!is_shape_near(i)) { i++; continue; }

    float da = point_aabb_dist(world_pos, objects[i].orig_bbox_min.xyz, objects[i].orig_bbox_max.xyz);
    float ungrouped_skip_thresh = object_aabbs[i].max_group_blend;
    int ungrouped_op = objects[i].csg_operation;
    bool ungrouped_must_eval = scene_dist < 1e9f &&
                               (ungrouped_op == SDF_CSG_OP_INTERSECT ||
                                ungrouped_op == SDF_CSG_OP_SUBTRACT);

    SDFObjectGPU obj = objects[i];
    float d;

    if (da > ungrouped_skip_thresh && !ungrouped_must_eval) {
      out_aabb_skip = min(out_aabb_skip, da);
      i++;
      continue;
    }

    /* Always evaluate the actual primitive — never use AABB approximation
     * for ungrouped objects, as subtract/intersect groups need accurate scene field. */
    {
      float3 lp = (obj.inverse_matrix * float4(world_pos - obj.position.xyz, 1.0f)).xyz;
      d = evalPrimitive(lp, obj);
      g_numEvaluated++;
      if (prof_enabled != 0) { atomicAdd(prof_eval_counts[i], 1u); }
    }

    if (scene_dist >= 1e9f) {
      if (obj.csg_operation != SDF_CSG_OP_SUBTRACT && obj.csg_operation != SDF_CSG_OP_SHELL &&
          obj.csg_operation != SDF_CSG_OP_INTERSECT) {
        scene_dist = d;
        out_color = obj.color.rgb;
        out_obj_id = float(obj.original_index);
      }
    }
    else {
      float prev = scene_dist;
      scene_dist = combineCSG(
          scene_dist, d, obj.csg_operation, obj.blend_type, obj.blend,
          obj.shell_distance, obj.shell_mode, obj.shell_op, obj.shell_blend_top, obj.shell_blend_bottom, obj.chamfer_k2, obj.chamfer_k3, obj.chamfer_k4, obj.chamfer_k5, obj.flip_blend, obj.flip_blend_end);
      if (obj.csg_operation == SDF_CSG_OP_SUBTRACT) {
        if (-d > prev) { out_obj_id = float(obj.original_index); }
      }
      else if (obj.csg_operation == SDF_CSG_OP_INTERSECT) {
        if (d > prev) { out_obj_id = float(obj.original_index); }
      }
      else if (obj.csg_operation == SDF_CSG_OP_PUSH) {
        if (d - scene_dist < sdf_ray_epsilon) { out_obj_id = float(obj.original_index); }
      }
      else if (obj.csg_operation == SDF_CSG_OP_AVOID) {
        if (prev - scene_dist > sdf_ray_epsilon) { out_obj_id = float(obj.original_index); }
      }
      else if (obj.csg_operation == SDF_CSG_OP_SHELL) {
        if (obj.shell_mode == SDF_SHELL_MODE_PUSH) {
          if (d - scene_dist < sdf_ray_epsilon) { out_obj_id = float(obj.original_index); }
        }
        else if (obj.shell_mode == SDF_SHELL_MODE_AVOID) {
          if (prev - scene_dist > sdf_ray_epsilon) { out_obj_id = float(obj.original_index); }
        }
        else if (obj.shell_op == SDF_SHELL_OP_SUBTRACTION) {
          if (-d > prev) { out_obj_id = float(obj.original_index); }
        }
        else {
          if (d < prev) { out_obj_id = float(obj.original_index); }
        }
      }
      else if (obj.csg_operation == SDF_CSG_OP_UNION) {
        if (d < prev) { out_obj_id = float(obj.original_index); }
      }
      if (obj.csg_operation == SDF_CSG_OP_UNION) {
        float t = colorBlendFactor(prev, d, obj.blend_type, obj.blend);
        out_color = mix(out_color, obj.color.rgb, t);
      }
    }
    i++;
  }

  return scene_dist;
}

float evalSceneDistBVH(float3 world_pos)
{
  float3 dummy_color;
  float dummy_skip;
  float dummy_id;
  return evalSceneBVH(world_pos, dummy_color, dummy_skip, dummy_id);
}

#ifdef USE_TILE_CULLING

/* Distance-only tile evaluation (color resolved in separate pass). */
float evalSceneTile(float3 world_pos, out float out_aabb_skip, out float out_obj_id, out float out_step_factor)
{
  float scene_dist = 1e10f;
  out_aabb_skip = 1e30f;
  out_obj_id = -1.0f;
  out_step_factor = 0.95f;

  int cur_group = -2;
  float grp_dist = 1e10f;
  bool grp_has_hit = false;
  float grp_winner_id = -1.0f;

  uint n = min(s_tileObjCount, uint(kMaxTileObjects));
  for (uint u = 0u; u < n; u++) {
    int i = s_tileObjList[u];
    if (i == skip_object) { continue; }
    if (!is_shape_near(i)) { continue; }

    SDFObjectAABB aabb = object_aabbs[i];
    int gid = aabb.group_id;

    if (gid != cur_group && grp_has_hit) {
      float prev_scene = scene_dist;
      int grp_csg = (cur_group >= 0) ? groups[cur_group].csg_operation : 0;
      flushGroupDist(cur_group, grp_dist, scene_dist);
      /* Per-operation winner check (same as evalSceneBVH) */
      if (grp_winner_id >= 0.0f) {
        if (prev_scene >= 1e9f) {
          out_obj_id = grp_winner_id;
        }
        else if (grp_csg == SDF_CSG_OP_SUBTRACT) {
          if (-grp_dist > prev_scene) { out_obj_id = grp_winner_id; }
        }
        else if (grp_csg == SDF_CSG_OP_INTERSECT) {
          if (grp_dist > prev_scene) { out_obj_id = grp_winner_id; }
        }
        else {
          if (grp_dist < prev_scene) { out_obj_id = grp_winner_id; }
        }
      }
      grp_has_hit = false;
      grp_dist = 1e10f;
      grp_winner_id = -1.0f;
    }

    SDFObjectGPU obj = objects[i];
    float da = point_aabb_dist(world_pos, obj.orig_bbox_min.xyz, obj.orig_bbox_max.xyz);
    int tile_skip_op = obj.csg_operation;
    bool tile_must_eval = (tile_skip_op == SDF_CSG_OP_INTERSECT ||
                           tile_skip_op == SDF_CSG_OP_SUBTRACT) &&
                          ((gid >= 0 && grp_has_hit && gid == cur_group) ||
                           (gid < 0 && scene_dist < 1e9f));

    float d;

    if (da > aabb.max_group_blend && !tile_must_eval) {
      if (prof_enabled != 0) { atomicAdd(prof_trace_stats[STAT_AABB_SKIPS], 1u); }
      out_aabb_skip = min(out_aabb_skip, da);
      cur_group = gid;
      continue;
    }

    {
      float3 lp = (obj.inverse_matrix * float4(world_pos - obj.position.xyz, 1.0f)).xyz;
      d = evalPrimitive(lp, obj);
      out_step_factor = min(out_step_factor, obj.orig_bbox_min.w);
      if (prof_enabled != 0) {
        atomicAdd(prof_eval_counts[i], 1u);
        atomicAdd(prof_trace_stats[STAT_EVALS], 1u);
      }
      cur_group = gid;
    }

    if (gid < 0) {
      if (scene_dist >= 1e9f) {
        if (obj.csg_operation == 0) {
          scene_dist = d;
          out_obj_id = float(obj.original_index);
        }
      }
      else {
        float prev = scene_dist;
        scene_dist = combineCSG(
            scene_dist, d, obj.csg_operation, obj.blend_type, obj.blend,
            obj.shell_distance, obj.shell_mode, obj.shell_op, obj.shell_blend_top, obj.shell_blend_bottom, obj.chamfer_k2, obj.chamfer_k3, obj.chamfer_k4, obj.chamfer_k5, obj.flip_blend, obj.flip_blend_end);
        if (obj.csg_operation == SDF_CSG_OP_SUBTRACT) {
          if (-d > prev) { out_obj_id = float(obj.original_index); }
        }
        else if (obj.csg_operation == SDF_CSG_OP_INTERSECT) {
          if (d > prev) { out_obj_id = float(obj.original_index); }
        }
        else if (obj.csg_operation == SDF_CSG_OP_PUSH) {
          if (d - scene_dist < sdf_ray_epsilon) { out_obj_id = float(obj.original_index); }
        }
        else if (obj.csg_operation == SDF_CSG_OP_AVOID) {
          if (prev - scene_dist > sdf_ray_epsilon) { out_obj_id = float(obj.original_index); }
        }
        else if (obj.csg_operation == SDF_CSG_OP_SHELL) {
          if (obj.shell_mode == SDF_SHELL_MODE_PUSH) {
            if (d - scene_dist < sdf_ray_epsilon) { out_obj_id = float(obj.original_index); }
          }
          else if (obj.shell_mode == SDF_SHELL_MODE_AVOID) {
            if (prev - scene_dist > sdf_ray_epsilon) { out_obj_id = float(obj.original_index); }
          }
          else if (obj.shell_op == SDF_SHELL_OP_SUBTRACTION) {
            if (-d > prev) { out_obj_id = float(obj.original_index); }
          }
          else {
            if (d < prev) { out_obj_id = float(obj.original_index); }
          }
        }
        else {
          if (d < prev) { out_obj_id = float(obj.original_index); }
        }
      }
    }
    else {
      if (!grp_has_hit) {
        if (obj.csg_operation != SDF_CSG_OP_SUBTRACT && obj.csg_operation != SDF_CSG_OP_SHELL &&
            obj.csg_operation != SDF_CSG_OP_INTERSECT) {
          grp_dist = d;
          grp_winner_id = float(obj.original_index);
          grp_has_hit = true;
        }
      }
      else {
        float prev = grp_dist;
        grp_dist = combineCSG(
            grp_dist, d, obj.csg_operation, obj.blend_type, obj.blend,
            obj.shell_distance, obj.shell_mode, obj.shell_op, obj.shell_blend_top, obj.shell_blend_bottom, obj.chamfer_k2, obj.chamfer_k3, obj.chamfer_k4, obj.chamfer_k5, obj.flip_blend, obj.flip_blend_end);
        if (obj.csg_operation == SDF_CSG_OP_SUBTRACT) {
          if (-d > prev) { grp_winner_id = float(obj.original_index); }
        }
        else if (obj.csg_operation == SDF_CSG_OP_INTERSECT) {
          if (d > prev) { grp_winner_id = float(obj.original_index); }
        }
        else if (obj.csg_operation == SDF_CSG_OP_PUSH) {
          if (d - grp_dist < sdf_ray_epsilon) { grp_winner_id = float(obj.original_index); }
        }
        else if (obj.csg_operation == SDF_CSG_OP_AVOID) {
          if (prev - grp_dist > sdf_ray_epsilon) { grp_winner_id = float(obj.original_index); }
        }
        else if (obj.csg_operation == SDF_CSG_OP_SHELL) {
          if (obj.shell_mode == SDF_SHELL_MODE_PUSH) {
            if (d - grp_dist < sdf_ray_epsilon) { grp_winner_id = float(obj.original_index); }
          }
          else if (obj.shell_mode == SDF_SHELL_MODE_AVOID) {
            if (prev - grp_dist > sdf_ray_epsilon) { grp_winner_id = float(obj.original_index); }
          }
          else if (obj.shell_op == SDF_SHELL_OP_SUBTRACTION) {
            if (-d > prev) { grp_winner_id = float(obj.original_index); }
          }
          else {
            if (d < prev) { grp_winner_id = float(obj.original_index); }
          }
        }
        else {
          if (d < prev) { grp_winner_id = float(obj.original_index); }
        }
      }
    }
  }

  if (grp_has_hit) {
    float prev_s = scene_dist;
    int grp_csg = (cur_group >= 0) ? groups[cur_group].csg_operation : 0;
    flushGroupDist(cur_group, grp_dist, scene_dist);
    if (grp_winner_id >= 0.0f) {
      if (prev_s >= 1e9f) {
        out_obj_id = grp_winner_id;
      }
      else if (grp_csg == SDF_CSG_OP_SUBTRACT) {
        if (-grp_dist > prev_s) { out_obj_id = grp_winner_id; }
      }
      else if (grp_csg == SDF_CSG_OP_INTERSECT) {
        if (grp_dist > prev_s) { out_obj_id = grp_winner_id; }
      }
      else {
        if (grp_dist < prev_s) { out_obj_id = grp_winner_id; }
      }
    }
  }

  return scene_dist;
}

float evalSceneDistTile(float3 world_pos)
{
  float dummy_skip, dummy_id, dummy_sf;
  return evalSceneTile(world_pos, dummy_skip, dummy_id, dummy_sf);
}

int tile_active_obj_count()
{
  return int(min(s_tileObjCount, uint(kMaxTileObjects)));
}

#else
#endif

float3 heat_color(float t)
{
  t = clamp(t, 0.0f, 1.0f);
  float3 cool = float3(0.0, 0.0, 1.0);
  float3 medium = float3(0.0, 1.0, 0.0);
  float3 warm = float3(1.0, 1.0, 0.0);
  float3 hot = float3(1.0, 0.0, 0.0);
  if (t < 0.33f) {
    return mix(cool, medium, t / 0.33f);
  }
  if (t < 0.66f) {
    return mix(medium, warm, (t - 0.33f) / 0.33f);
  }
  return mix(warm, hot, (t - 0.66f) / 0.34f);
}

void main()
{
  int2 pixel = int2(gl_GlobalInvocationID.xy);
  int local_idx = int(gl_LocalInvocationID.y * kTileSize + gl_LocalInvocationID.x);

#ifdef USE_TILE_CULLING
  /* Load tile prim list from SSBOs (built by tile cull pass) */
  int tilesX = (screen_size.x + 7) / 8;
  int tileIdx = int(gl_WorkGroupID.x) + int(gl_WorkGroupID.y) * tilesX;

  if (local_idx == 0) {
    s_tileObjCount = uint(tile_prim_counts[tileIdx]);
    s_coneHitPos = (use_cone_trace != 0) ? tile_hit_pos[tileIdx]
                                          : float4(0.0f, 0.0f, 0.0f, -1.0f);
  }
  barrier();

  /* Empty tile (zeroed by cone march for CSG-empty tiles): clear and exit */
  if (s_tileObjCount == 0u) {
    if (pixel.x < screen_size.x && pixel.y < screen_size.y) {
      if (debug_bvh_views != 0) {
        imageStore(out_color_img, pixel, float4(heat_color(0.0f), 1.0f));
        imageStore(out_depth_img, pixel, float4(0.0f));
      }
      imageStore(gbuf_pos_img, pixel, float4(0.0));
      imageStore(gbuf_color_img, pixel, float4(0.0));
    }
    return;
  }

  uint n = min(s_tileObjCount, uint(kMaxTileObjects));
  int base = tileIdx * kMaxTileObjects;
  for (uint i = uint(local_idx); i < n; i += 64u) {
    s_tileObjList[i] = tile_prim_lists[base + int(i)];
  }
  barrier();


  ViewMatrices vm = drw_view();
#endif

  if (pixel.x >= screen_size.x || pixel.y >= screen_size.y) {
    return;
  }

  float2 uv = (float2(pixel) + 0.5f) / float2(screen_size);

#ifndef USE_TILE_CULLING
  ViewMatrices vm = drw_view();
#endif
  float4x4 view_inv = vm.viewinv;
  float4x4 win_inv = vm.wininv;

  float4 ndc_near = float4(uv * 2.0f - 1.0f, -1.0f, 1.0f);
  float4 ndc_far = float4(uv * 2.0f - 1.0f, 1.0f, 1.0f);
  float4 world_near = view_inv * (win_inv * ndc_near);
  float4 world_far = view_inv * (win_inv * ndc_far);
  world_near.xyz /= world_near.w;
  world_far.xyz /= world_far.w;

  float3 ray_origin = world_near.xyz;
  float3 ray_dir = normalize(world_far.xyz - world_near.xyz);
  float3 safe_dir = float3(
      abs(ray_dir.x) < 1e-8f ? 1e-8f : ray_dir.x,
      abs(ray_dir.y) < 1e-8f ? 1e-8f : ray_dir.y,
      abs(ray_dir.z) < 1e-8f ? 1e-8f : ray_dir.z);
  float3 inv_dir = 1.0f / safe_dir;

  float max_dist = 1000.0f;

  /* Scene AABB ray clipping */
  float3 s_min = scene_aabb_min;
  float3 s_max = scene_aabb_max;
  bool scene_aabb_valid = all(lessThan(s_min, s_max));
  float t_enter, t_exit;

  if (scene_aabb_valid) {
    float3 t0 = (s_min - ray_origin) * inv_dir;
    float3 t1 = (s_max - ray_origin) * inv_dir;
    float3 t_lo = min(t0, t1);
    float3 t_hi = max(t0, t1);
    t_enter = max(max(t_lo.x, t_lo.y), t_lo.z);
    t_exit = min(min(t_hi.x, t_hi.y), t_hi.z);

    if (t_enter > t_exit || t_exit < 0.0f) {
      if (debug_bvh_views != 0) {
        imageStore(out_color_img, pixel, float4(heat_color(0.0f), 1.0f));
        imageStore(out_depth_img, pixel, float4(0.0f));
      }
      imageStore(gbuf_pos_img, pixel, float4(0.0));
      imageStore(gbuf_color_img, pixel, float4(0.0));
      return;
    }
    t_enter = max(t_enter, 0.0f);
    t_exit = min(t_exit, max_dist);
  }
  else {
    t_enter = 0.0f;
    t_exit = max_dist;
  }

#ifdef USE_TILE_CULLING
  float t_enter_before_cone = t_enter;
  float cone_skip_target = -1.0f;
  if (use_cone_trace != 0 && s_coneHitPos.w >= 0.0f) {
    float projected = dot(s_coneHitPos.xyz - ray_origin, ray_dir);
    if (projected > t_enter) {
      cone_skip_target = projected;
    }
  }
#endif

  /* Per-ray BVH: mark objects whose AABB the ray intersects (done ONCE per ray) */
  g_numNearShapes = 0;
  g_numEvaluated = 0;
  for (int w = 0; w < kBitfieldWords; w++) {
    g_bvh_bits[w] = 0u;
  }

  if (use_bvh != 0 && bvh_root >= 0) {
    float3 ray_to = ray_origin + ray_dir * t_exit;
    float3 rayDirOrtho = normalize(find_ortho(ray_dir));
    float3 rayDirOrthoAbs = abs(rayDirOrtho);
    float3 rayBoundsMin = min(ray_origin, ray_to);
    float3 rayBoundsMax = max(ray_origin, ray_to);

    int stackTop = 0;
    int stack[kAabbTreeStackSize];
    stack[stackTop] = bvh_root;

    while (stackTop >= 0) {
      int index = stack[stackTop--];
      if (index < 0) { continue; }

      SdfAabbNodeGPU node = aabb_nodes[index];

      if (any(greaterThan(node.bounds_min.xyz, rayBoundsMax)) ||
          any(greaterThan(rayBoundsMin, node.bounds_max.xyz)))
      {
        continue;
      }

      float3 aabbCenter = 0.5f * (node.bounds_min.xyz + node.bounds_max.xyz);
      float3 aabbHalfExtents = 0.5f * (node.bounds_max.xyz - node.bounds_min.xyz);
      float separation = abs(dot(rayDirOrtho, ray_origin - aabbCenter)) -
                          dot(rayDirOrthoAbs, aabbHalfExtents);
      if (separation > 0.0f) { continue; }

      if (node.child_a < 0) {
        float3 lt1 = (node.bounds_min.xyz - ray_origin) * inv_dir;
        float3 lt2 = (node.bounds_max.xyz - ray_origin) * inv_dir;
        float ltMin = max(max(min(lt1.x, lt2.x), min(lt1.y, lt2.y)), min(lt1.z, lt2.z));
        float ltMax = min(min(max(lt1.x, lt2.x), max(lt1.y, lt2.y)), max(lt1.z, lt2.z));
        if (ltMax < 0.0f || ltMin > ltMax) { continue; }

        int idx = node.shape_index;
        if (idx >= 0 && idx < kMaxBitfieldBits) {
          uint word = uint(idx) >> 5u;
          uint bit = 1u << (uint(idx) & 31u);
          if ((g_bvh_bits[word] & bit) == 0u) {
            g_bvh_bits[word] |= bit;
            g_numNearShapes++;
          }
        }
      }
      else {
        if (stackTop + 2 < kAabbTreeStackSize) {
          stack[++stackTop] = node.child_a;
          stack[++stackTop] = node.child_b;
        }
      }
    }
  }
  else {
    g_numNearShapes = object_count;
    for (int w = 0; w < kBitfieldWords; w++) {
      g_bvh_bits[w] = 0xFFFFFFFFu;
    }
  }

#ifndef USE_TILE_CULLING
#endif

  float step_factor = sdf_step_factor;

  /* Over-relaxation sphere tracing */
  float t = t_enter;
  bool hit = false;
  float3 hit_pos;
  float hit_obj_id = -1.0f;
  int steps_taken = 0;
  float omega = sdf_over_relaxation;
  float prev_radius = 0.0f;
  float step_length = 0.0f;
  float t_prev = t_enter;
  float d_prev = 1e10f;

#ifdef USE_TILE_CULLING
  if (cone_skip_target > t_enter) {
    float3 skip_pos = ray_origin + ray_dir * cone_skip_target;
    float skip_aabb;
    float skip_id;
    float skip_sf;
    float skip_d = evalSceneTile(skip_pos, skip_aabb, skip_id, skip_sf);
    if (skip_d > 0.0f) {
      t = cone_skip_target;
      if (skip_d < 1e9f) {
        d_prev = skip_d;
        t_prev = cone_skip_target - skip_d;
      }
    }
  }
#endif

  uint p_steps = 0u, p_empty = 0u, p_sor_fail = 0u;

  for (int step = 0; step < sdf_max_steps; step++) {
    if (t > t_exit) { break; }
    float3 pos = ray_origin + ray_dir * t;

    float d;
    float aabb_skip;
    float cur_obj_id;
    float local_step_factor;
#ifdef USE_TILE_CULLING
    d = evalSceneTile(pos, aabb_skip, cur_obj_id, local_step_factor);

    if (d >= 1e9f) {
      p_empty++;
      t += max(aabb_skip, sdf_ray_epsilon);
      prev_radius = 0.0f;
      step_length = 0.0f;
      if (t > t_exit) { break; }
      continue;
    }

    if (aabb_skip < d) {
      d = max(aabb_skip, sdf_ray_epsilon * 10.0f);
    }
#else
    local_step_factor = sdf_step_factor;
    float3 color;
    d = evalSceneBVH(pos, color, aabb_skip, cur_obj_id);

    if (d >= 1e9f) {
      p_empty++;
      t += max(aabb_skip, sdf_ray_epsilon);
      prev_radius = 0.0f;
      step_length = 0.0f;
      if (t > t_exit) { break; }
      continue;
    }

    if (aabb_skip < d) {
      d = max(aabb_skip, sdf_ray_epsilon * 10.0f);
    }
#endif

    p_steps++;
    steps_taken++;
    float abs_d = abs(d);

    bool sor_fail = omega > 1.0f && (abs_d + prev_radius) < step_length * 1.01f;
    if (sor_fail) {
      p_sor_fail++;
      step_length -= omega * step_length;
      omega = 1.0f;
    } else {
      step_length = abs_d * local_step_factor * omega;
    }

    float adaptive_epsilon = sdf_ray_epsilon * (1.0f + t * 0.001f);

    /* Sign change: ray crossed the surface between t_prev and t.
     * Use secant interpolation to find crossing point. */
    bool sign_change = !sor_fail && d_prev < 1e9f && d_prev > 0.0f && d < 0.0f;

    if (!sor_fail && (abs_d < adaptive_epsilon || sign_change)) {
      hit = true;
      /* Surface projection with angle correction.
       * cos_theta estimated from distance decrease rate. Distance-adaptive clamp:
       * aggressive close (eliminates step bands), conservative far (no puffiness). */
      float denom = d_prev - d;
      if (denom > 1e-8f) {
        float alpha = d_prev / denom;
        if (alpha > 0.0f && alpha < 1.0f) {
          hit_pos = ray_origin + ray_dir * mix(t_prev, t, alpha);
        }
        else {
          float cos_theta = denom / max(t - t_prev, 1e-8f);
          float near = clamp(5.0f / max(t, 0.1f), 0.0f, 1.0f);
          float min_cos = mix(0.9f, 0.25f, near);
          hit_pos = pos + ray_dir * d / clamp(cos_theta, min_cos, 1.0f);
        }
      }
      else {
        hit_pos = pos + ray_dir * d;
      }
      /* History-independent surface snap: estimate cos_theta from two fresh
       * SDF evals. Eliminates cone march tile-boundary position artifacts. */
      {
        float eps_snap = sdf_ray_epsilon * 0.5f;
#ifdef USE_TILE_CULLING
        float d0 = evalSceneDistTile(hit_pos);
        float d1 = evalSceneDistTile(hit_pos + ray_dir * eps_snap);
#else
        float d0 = evalSceneDistBVH(hit_pos);
        float d1 = evalSceneDistBVH(hit_pos + ray_dir * eps_snap);
#endif
        /* Sign-preserving cosine so exit-surface snap heads forward, not backward. */
        float cos_raw = (d0 - d1) / eps_snap;
        float cos_est = (cos_raw >= 0.0f ? 1.0f : -1.0f) * clamp(abs(cos_raw), 0.1f, 1.0f);
        hit_pos += ray_dir * d0 / cos_est;
      }

      hit_obj_id = cur_obj_id;
      break;
    }

    /* Track previous step for secant refinement */
    t_prev = t;
    d_prev = d;
    prev_radius = abs_d;

    if (sor_fail) {
      t += step_length;
    } else {
      t += max(step_length, sdf_ray_epsilon * 0.5f);
    }
  }

  /* Flush per-ray profiling stats */
  if (prof_enabled != 0) {
    atomicAdd(prof_trace_stats[STAT_RAYS], 1u);
    atomicAdd(prof_trace_stats[STAT_STEPS], p_steps);
    atomicAdd(prof_trace_stats[STAT_EMPTY_STEPS], p_empty);
    atomicAdd(prof_trace_stats[STAT_SOR_FAILURES], p_sor_fail);
    if (hit) { atomicAdd(prof_trace_stats[STAT_HITS], 1u); }
    atomicMax(prof_trace_stats[STAT_MAX_STEPS], p_steps);
  }

  /* Debug views write directly to out_color/out_depth */
#ifdef USE_TILE_CULLING
  if (debug_bvh_views == 1) {
    float heat = float(tile_active_obj_count()) / max(float(object_count), 1.0f);
    imageStore(out_color_img, pixel, float4(heat_color(heat), 1.0f));
    imageStore(out_depth_img, pixel, float4(0.0f));
    imageStore(gbuf_pos_img, pixel, float4(0.0));
    imageStore(gbuf_color_img, pixel, float4(0.0));
    return;
  }
  else if (debug_bvh_views == 3) {
    float heat = float(steps_taken) / 128.0f;
    imageStore(out_color_img, pixel, float4(heat_color(heat), 1.0f));
    imageStore(out_depth_img, pixel, float4(0.0f));
    imageStore(gbuf_pos_img, pixel, float4(0.0));
    imageStore(gbuf_color_img, pixel, float4(0.0));
    return;
  }
  else if (debug_bvh_views == 5) {
    float skip = (cone_skip_target > 0.0f) ? cone_skip_target - t_enter_before_cone : 0.0f;
    float total_range = t_exit - t_enter_before_cone;
    float savings = (total_range > 0.001f) ? clamp(skip / total_range, 0.0f, 1.0f) : 0.0f;
    float3 cone_color = mix(float3(1.0f, 0.0f, 0.0f), float3(0.0f, 1.0f, 0.0f), savings);
    if (s_coneHitPos.w < 0.0f) { cone_color = float3(0.0f); }
    imageStore(out_color_img, pixel, float4(cone_color, 1.0f));
    imageStore(out_depth_img, pixel, float4(0.0f));
    imageStore(gbuf_pos_img, pixel, float4(0.0));
    imageStore(gbuf_color_img, pixel, float4(0.0));
    return;
  }
#else
  if (debug_bvh_views == 1) {
    float heat = float(g_numNearShapes) / max(float(object_count), 1.0f);
    imageStore(out_color_img, pixel, float4(heat_color(heat), 1.0f));
    imageStore(out_depth_img, pixel, float4(0.0f));
    imageStore(gbuf_pos_img, pixel, float4(0.0));
    imageStore(gbuf_color_img, pixel, float4(0.0));
    return;
  }
  else if (debug_bvh_views == 3) {
    float heat = float(steps_taken) / 128.0f;
    imageStore(out_color_img, pixel, float4(heat_color(heat), 1.0f));
    imageStore(out_depth_img, pixel, float4(0.0f));
    imageStore(gbuf_pos_img, pixel, float4(0.0));
    imageStore(gbuf_color_img, pixel, float4(0.0));
    return;
  }
  else if (debug_bvh_views == 5) {
    imageStore(out_color_img, pixel, float4(0.0f, 0.0f, 0.0f, 1.0f));
    imageStore(out_depth_img, pixel, float4(0.0f));
    imageStore(gbuf_pos_img, pixel, float4(0.0));
    imageStore(gbuf_color_img, pixel, float4(0.0));
    return;
  }
#endif

  if (!hit) {
    imageStore(gbuf_pos_img, pixel, float4(0.0));
    imageStore(gbuf_color_img, pixel, float4(0.0));
    return;
  }

  /* Output G-buffer */
  imageStore(gbuf_pos_img, pixel, float4(hit_pos, 1.0));
  imageStore(gbuf_color_img, pixel, float4(0.0f, 0.0f, 0.0f, hit_obj_id));
}
