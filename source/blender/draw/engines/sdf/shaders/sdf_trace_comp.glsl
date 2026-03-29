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
#define kMaxTileObjects 128

#ifdef USE_TILE_CULLING
shared uint s_tileObjCount;
shared int s_tileObjList[kMaxTileObjects];
shared float4 s_coneHitPos;
#else
shared float tile_heat[kTileSize * kTileSize];
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

  /* Groups: per-object AABB check, substitute AABB distance when outside */
  for (int g = 0; g < group_count; g++) {
    SDFGroupGPU grp = groups[g];
    float grp_dist = 1e10f;
    float3 grp_color = grp.color.rgb;
    bool grp_has_hit = false;
    float grp_winner_id = -1.0f;

    for (int m = grp.first_object; m < grp.first_object + grp.object_count; m++) {
      if (!is_shape_near(m)) { continue; }

      float aabb_skip_thresh = max(sdf_ray_epsilon, objects[m].blend);
      if (point_aabb_dist(world_pos, objects[m].bbox_min.xyz, objects[m].bbox_max.xyz) > aabb_skip_thresh) {
        continue;
      }

      SDFObjectGPU obj = objects[m];
      float3 lp = (obj.inverse_matrix * float4(world_pos - obj.position.xyz, 1.0f)).xyz;
      float d = evalPrimitive(lp, obj);
      g_numEvaluated++;

      if (!grp_has_hit) {
        grp_dist = d;
        grp_color = obj.color.rgb;
        grp_winner_id = float(obj.original_index);
        grp_has_hit = true;
      }
      else {
        float prev = grp_dist;
        grp_dist = combineCSG(
            grp_dist, d, obj.csg_operation, obj.blend_type, obj.blend,
            obj.shell_distance, obj.shell_mode, obj.shell_op, obj.shell_blend_top, obj.shell_blend_bottom, obj.chamfer_k2, obj.chamfer_k3);
        if (obj.csg_operation == 0 && d < prev) {
          grp_winner_id = float(obj.original_index);
        }
        else if (obj.csg_operation != 0 && -d > prev) {
          grp_winner_id = float(obj.original_index);
        }
        if (obj.csg_operation == 0) {
          float t = colorBlendFactor(prev, d, obj.blend_type, obj.blend);
          grp_color = mix(grp_color, obj.color.rgb, t);
        }
      }
    }

    if (!grp_has_hit) {
      float grp_aabb = 1e30f;
      for (int m = grp.first_object; m < grp.first_object + grp.object_count; m++) {
        grp_aabb = min(grp_aabb, point_aabb_dist(world_pos, objects[m].bbox_min.xyz, objects[m].bbox_max.xyz));
      }
      out_aabb_skip = min(out_aabb_skip, grp_aabb);
      continue;
    }

    if (scene_dist >= 1e9f) {
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
          grp.chamfer_k2, grp.chamfer_k3);
      if (grp.csg_operation == 0 && grp_dist < prev) {
        out_obj_id = grp_winner_id;
      }
      if (grp.csg_operation == 0) {
        float t = colorBlendFactor(prev, grp_dist, grp.blend_type, grp.blend);
        out_color = mix(out_color, grp_color, t);
      }
    }
  }

  /* Ungrouped objects: per-step AABB test per object */
  for (int i = 0; i < object_count; i++) {
    if (!is_shape_near(i)) { continue; }
    if (objects[i].group_id >= 0) { continue; }

    float da = point_aabb_dist(world_pos, objects[i].bbox_min.xyz, objects[i].bbox_max.xyz);
    float ungrouped_skip_thresh = max(0.0f, objects[i].blend);
    if (da > ungrouped_skip_thresh) {
      out_aabb_skip = min(out_aabb_skip, da);
      continue;
    }

    SDFObjectGPU obj = objects[i];
    float3 lp = (obj.inverse_matrix * float4(world_pos - obj.position.xyz, 1.0f)).xyz;
    float d = evalPrimitive(lp, obj);
    g_numEvaluated++;

    if (scene_dist >= 1e9f) {
      scene_dist = d;
      out_color = obj.color.rgb;
      out_obj_id = float(obj.original_index);
    }
    else {
      float prev = scene_dist;
      scene_dist = combineCSG(
          scene_dist, d, obj.csg_operation, obj.blend_type, obj.blend,
          obj.shell_distance, obj.shell_mode, obj.shell_op, obj.shell_blend_top, obj.shell_blend_bottom, obj.chamfer_k2, obj.chamfer_k3);
      if (obj.csg_operation == 0 && d < prev) {
        out_obj_id = float(obj.original_index);
      }
      if (obj.csg_operation == 0) {
        float t = colorBlendFactor(prev, d, obj.blend_type, obj.blend);
        out_color = mix(out_color, obj.color.rgb, t);
      }
    }
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
float evalSceneTile(float3 world_pos, out float out_aabb_skip, out float out_obj_id)
{
  float scene_dist = 1e10f;
  out_aabb_skip = 1e30f;
  out_obj_id = -1.0f;

  int cur_group = -2;
  float grp_dist = 1e10f;
  bool grp_has_hit = false;
  float grp_winner_id = -1.0f;

  uint n = min(s_tileObjCount, uint(kMaxTileObjects));
  for (uint u = 0u; u < n; u++) {
    int i = s_tileObjList[u];

    /* Hot buffer: 48 bytes instead of 256 for AABB skip path */
    SDFObjectAABB aabb = object_aabbs[i];
    int gid = aabb.group_id;

    if (gid != cur_group && grp_has_hit) {
      flushGroupDist(cur_group, grp_dist, scene_dist);
      if (scene_dist < 1e9f) { out_obj_id = grp_winner_id; }
      grp_has_hit = false;
      grp_dist = 1e10f;
      grp_winner_id = -1.0f;
    }

    float da = point_aabb_dist(world_pos, aabb.bbox_min.xyz, aabb.bbox_max.xyz);
    float skip_threshold = max(sdf_ray_epsilon, aabb.max_group_blend);
    if (da > skip_threshold) {
      out_aabb_skip = min(out_aabb_skip, da);
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
          out_obj_id = float(obj.original_index);
        }
      }
      else {
        float prev = scene_dist;
        scene_dist = combineCSG(
            scene_dist, d, obj.csg_operation, obj.blend_type, obj.blend,
            obj.shell_distance, obj.shell_mode, obj.shell_op, obj.shell_blend_top, obj.shell_blend_bottom, obj.chamfer_k2, obj.chamfer_k3);
        if (obj.csg_operation == 0 && d < prev) {
          out_obj_id = float(obj.original_index);
        }
        else if (obj.csg_operation != 0 && -d > prev) {
          out_obj_id = float(obj.original_index);
        }
      }
    }
    else {
      if (!grp_has_hit) {
        if (obj.csg_operation != SDF_CSG_OP_SUBTRACT && obj.csg_operation != SDF_CSG_OP_SHELL) {
          grp_dist = d;
          grp_winner_id = float(obj.original_index);
          grp_has_hit = true;
        }
      }
      else {
        float prev = grp_dist;
        grp_dist = combineCSG(
            grp_dist, d, obj.csg_operation, obj.blend_type, obj.blend,
            obj.shell_distance, obj.shell_mode, obj.shell_op, obj.shell_blend_top, obj.shell_blend_bottom, obj.chamfer_k2, obj.chamfer_k3);
        if (obj.csg_operation == 0 && d < prev) {
          grp_winner_id = float(obj.original_index);
        }
        else if (obj.csg_operation != 0 && -d > prev) {
          grp_winner_id = float(obj.original_index);
        }
      }
    }
  }

  if (grp_has_hit) {
    float prev_s = scene_dist;
    flushGroupDist(cur_group, grp_dist, scene_dist);
    if (prev_s >= 1e9f || grp_dist < prev_s) {
      out_obj_id = grp_winner_id;
    }
  }

  return scene_dist;
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

#ifndef USE_TILE_CULLING
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
  /* Cone skip: jump to cone position and evaluate SDF there.
   * Handles both surface approach AND empty-space skipping (booleans). */
  if (cone_skip_target > 0.0f) {
    float3 skip_pos = ray_origin + ray_dir * cone_skip_target;
    float skip_aabb;
    float skip_id;
    float skip_d = evalSceneTile(skip_pos, skip_aabb, skip_id);
    if (skip_d > sdf_ray_epsilon * 2.0f) {
      t = cone_skip_target;
      if (skip_d < 1e9f) {
        d_prev = skip_d;
        t_prev = cone_skip_target - skip_d;
      }
    }
    else if (cone_skip_target > t_enter + sdf_ray_epsilon * 8.0f) {
      /* Near/on surface at skip point — back off slightly but still skip bulk of empty space */
      t = cone_skip_target - sdf_ray_epsilon * 8.0f;
    }
  }

#endif

  for (int step = 0; step < sdf_max_steps; step++) {
    if (t > t_exit) { break; }
    float3 pos = ray_origin + ray_dir * t;

    float d;
    float aabb_skip;
    float cur_obj_id;
#ifdef USE_TILE_CULLING
    d = evalSceneTile(pos, aabb_skip, cur_obj_id);

    if (d >= 1e9f) {
      t += max(aabb_skip, sdf_ray_epsilon);
      prev_radius = 0.0f;
      step_length = 0.0f;
      if (t > t_exit) { break; }
      continue;
    }

    /* Safe step: clamp by skipped AABBs with fixed floor to prevent crawling.
     * Floor at 10x epsilon — safe for CSG cuts (features < 0.01 are sub-pixel). */
    if (aabb_skip < d) {
      d = max(aabb_skip, sdf_ray_epsilon * 10.0f);
    }
#else
    float3 color;
    d = evalSceneBVH(pos, color, aabb_skip, cur_obj_id);

    /* If no AABB contains this point, skip by nearest AABB distance */
    if (d >= 1e9f) {
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

    steps_taken++;
    float abs_d = abs(d);

    /* Over-relaxation (Keinert et al. 2014).
     * Margin ensures SOR triggers consistently at the boundary (linear SDF),
     * preventing per-pixel divergence that causes banding in axis-aligned views. */
    bool sor_fail = omega > 1.0f && (abs_d + prev_radius) < step_length * 1.01f;
    if (sor_fail) {
      step_length -= omega * step_length;
      omega = 1.0f;
    } else {
      step_length = abs_d * step_factor * omega;
    }

    float adaptive_epsilon = sdf_ray_epsilon * (1.0f + t * 0.001f);

    if (!sor_fail && abs_d < adaptive_epsilon) {
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
  else if (debug_bvh_views == 2) {
    s_tileObjList[local_idx] = floatBitsToInt(
        float(tile_active_obj_count()) / max(float(object_count), 1.0f));
    barrier();
    for (uint stride = 32u; stride > 0u; stride >>= 1u) {
      if (uint(local_idx) < stride) {
        s_tileObjList[local_idx] = floatBitsToInt(
            max(intBitsToFloat(s_tileObjList[local_idx]),
                intBitsToFloat(s_tileObjList[local_idx + int(stride)])));
      }
      barrier();
    }
    float max_heat = intBitsToFloat(s_tileObjList[0]);
    imageStore(out_color_img, pixel, float4(heat_color(max_heat), 1.0f));
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
  else if (debug_bvh_views == 4) {
    s_tileObjList[local_idx] = floatBitsToInt(float(steps_taken) / 128.0f);
    barrier();
    for (uint stride = 32u; stride > 0u; stride >>= 1u) {
      if (uint(local_idx) < stride) {
        s_tileObjList[local_idx] = floatBitsToInt(
            max(intBitsToFloat(s_tileObjList[local_idx]),
                intBitsToFloat(s_tileObjList[local_idx + int(stride)])));
      }
      barrier();
    }
    float max_heat = intBitsToFloat(s_tileObjList[0]);
    imageStore(out_color_img, pixel, float4(heat_color(max_heat), 1.0f));
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
  else if (debug_bvh_views == 2) {
    tile_heat[local_idx] = float(g_numNearShapes) / max(float(object_count), 1.0f);
    barrier();
    for (uint stride = 32u; stride > 0u; stride >>= 1u) {
      if (uint(local_idx) < stride) {
        tile_heat[local_idx] = max(tile_heat[local_idx], tile_heat[local_idx + stride]);
      }
      barrier();
    }
    float max_heat = tile_heat[0];
    imageStore(out_color_img, pixel, float4(heat_color(max_heat), 1.0f));
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
  else if (debug_bvh_views == 4) {
    tile_heat[local_idx] = float(steps_taken) / 128.0f;
    barrier();
    for (uint stride = 32u; stride > 0u; stride >>= 1u) {
      if (uint(local_idx) < stride) {
        tile_heat[local_idx] = max(tile_heat[local_idx], tile_heat[local_idx + stride]);
      }
      barrier();
    }
    float max_heat = tile_heat[0];
    imageStore(out_color_img, pixel, float4(heat_color(max_heat), 1.0f));
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
