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

#include "draw_view_lib.glsl"
#include "sdf_lib.glsl"

#define kFltMax 1e30f
#define kTileSize 8

shared float tile_heat[kTileSize * kTileSize];

float evalPrimitive(float3 local_pos, SDFObjectGPU obj)
{
  SDFPrimitiveData prim_data;
  prim_data.sdf_type = obj.sdf_type;
  prim_data.size = obj.sdf_size.xyz;
  prim_data.bevel = obj.bevel;
  prim_data.box_corners = obj.box_corners;
  prim_data.box_edges = obj.box_edges;
  prim_data.box_modes = obj.box_modes;
  prim_data.modifier_start = obj.modifier_start;
  prim_data.modifier_count = obj.modifier_count;
  prim_data.inverse_matrix = obj.inverse_matrix;

  return evalObjectSDF(prim_data, local_pos);
}

float evalObjectDist(float3 world_pos, int idx)
{
  SDFObjectGPU obj = objects[idx];
  float3 lp = (obj.inverse_matrix * float4(world_pos - obj.position.xyz, 1.0f)).xyz;
  return evalPrimitive(lp, obj);
}

#ifdef USE_TILE_CULLING

#define kMaxTileObjects 256
shared uint s_tileObjCount;
shared int s_tileObjList[kMaxTileObjects];
shared float4 s_coneHitPos;

#else

#define kAabbTreeStackSize 32
#define kMaxBitfieldBits 256
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
float evalSceneBVH(float3 world_pos, out float3 out_color, out float out_aabb_skip)
{
  float scene_dist = 1e10f;
  out_color = float3(0.5f);
  out_aabb_skip = 1e30f;
  g_numEvaluated = 0;

  /* Groups: per-object AABB check, substitute AABB distance when outside */
  for (int g = 0; g < group_count; g++) {
    SDFGroupGPU grp = groups[g];
    float grp_dist = 1e10f;
    float3 grp_color = grp.color.rgb;
    bool grp_has_hit = false;

    for (int m = grp.first_object; m < grp.first_object + grp.object_count; m++) {
      if (!is_shape_near(m)) continue;

      /* Per-step AABB skip */
      if (point_aabb_dist(world_pos, objects[m].bbox_min.xyz, objects[m].bbox_max.xyz) > sdf_ray_epsilon) {
        continue;
      }

      SDFObjectGPU obj = objects[m];
      float3 lp = (obj.inverse_matrix * float4(world_pos - obj.position.xyz, 1.0f)).xyz;
      float d = evalPrimitive(lp, obj);
      g_numEvaluated++;

      if (!grp_has_hit) {
        grp_dist = d;
        grp_color = obj.color.rgb;
        grp_has_hit = true;
      }
      else {
        float prev = grp_dist;
        grp_dist = combineCSG(
            grp_dist, d, obj.csg_operation, obj.blend_type, obj.blend,
            obj.shell_distance, obj.shell_mode);
        if (obj.csg_operation == 0 && d < prev) {
          float t = clamp(1.0f - (d - grp_dist) / max(obj.blend, 0.001f), 0.0f, 1.0f);
          grp_color = mix(grp_color, obj.color.rgb, t);
        }
      }
    }

    if (!grp_has_hit) {
      /* No object AABB contained this point — track nearest for empty-space skip */
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
    }
    else {
      float prev = scene_dist;
      scene_dist = combineCSG(
          scene_dist, grp_dist, grp.csg_operation, grp.blend_type, grp.blend,
          grp.shell_distance, grp.shell_mode);
      if (grp.csg_operation == 0 && grp_dist < prev) {
        float t = clamp(1.0f - (grp_dist - scene_dist) / max(grp.blend, 0.001f), 0.0f, 1.0f);
        out_color = mix(out_color, grp_color, t);
      }
    }
  }

  /* Ungrouped objects: per-step AABB test per object */
  for (int i = 0; i < object_count; i++) {
    if (!is_shape_near(i)) continue;
    SDFObjectGPU obj = objects[i];
    if (obj.group_id >= 0) continue;

    float da = point_aabb_dist(world_pos, obj.bbox_min.xyz, obj.bbox_max.xyz);
    if (da > 0.0f) {
      out_aabb_skip = min(out_aabb_skip, da);
      continue;
    }

    float3 lp = (obj.inverse_matrix * float4(world_pos - obj.position.xyz, 1.0f)).xyz;
    float d = evalPrimitive(lp, obj);
    g_numEvaluated++;

    if (scene_dist >= 1e9f) {
      scene_dist = d;
      out_color = obj.color.rgb;
    }
    else {
      float prev = scene_dist;
      scene_dist = combineCSG(
          scene_dist, d, obj.csg_operation, obj.blend_type, obj.blend,
          obj.shell_distance, obj.shell_mode);
      if (obj.csg_operation == 0 && d < prev) {
        float t = clamp(1.0f - (d - scene_dist) / max(obj.blend, 0.001f), 0.0f, 1.0f);
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
  return evalSceneBVH(world_pos, dummy_color, dummy_skip);
}

#endif

#ifdef USE_TILE_CULLING

/* Flush accumulated group result into scene distance. */
void flushGroup(int gid, float grp_dist, float3 grp_color,
                inout float scene_dist, inout float3 out_color)
{
  if (scene_dist >= 1e9f) {
    scene_dist = grp_dist;
    out_color = grp_color;
  }
  else {
    SDFGroupGPU grp = groups[gid];
    float prev = scene_dist;
    scene_dist = combineCSG(
        scene_dist, grp_dist, grp.csg_operation, grp.blend_type, grp.blend,
        grp.shell_distance, grp.shell_mode);
    if (grp.csg_operation == 0 && grp_dist < prev) {
      float t = clamp(1.0f - (grp_dist - scene_dist) / max(grp.blend, 0.001f), 0.0f, 1.0f);
      out_color = mix(out_color, grp_color, t);
    }
  }
}

/* Single-pass evaluation over flat sorted tile-visible list. */
float evalSceneTile(float3 world_pos, out float3 out_color, out float out_aabb_skip)
{
  float scene_dist = 1e10f;
  out_color = float3(0.5f);
  out_aabb_skip = 1e30f;

  int cur_group = -2;
  float grp_dist = 1e10f;
  float3 grp_color = float3(0.5f);
  bool grp_has_hit = false;

  uint n = min(s_tileObjCount, uint(kMaxTileObjects));
  for (uint u = 0u; u <= n; u++) {
    int i = (u < n) ? s_tileObjList[u] : -1;
    int gid = (i >= 0) ? objects[i].group_id : -2;

    /* Group boundary: flush previous group into scene */
    if (gid != cur_group && grp_has_hit) {
      flushGroup(cur_group, grp_dist, grp_color, scene_dist, out_color);
      grp_has_hit = false;
      grp_dist = 1e10f;
    }

    if (u >= n) break;

    /* Per-step AABB skip (uses max group blend from _pad1) */
    SDFObjectGPU obj = objects[i];
    float da = point_aabb_dist(world_pos, obj.bbox_min.xyz, obj.bbox_max.xyz);
    float max_group_blend = intBitsToFloat(obj._pad1);
    float skip_threshold = max(sdf_ray_epsilon, max_group_blend);
    if (da > skip_threshold) {
      out_aabb_skip = min(out_aabb_skip, da);
      continue;
    }
    float3 lp = (obj.inverse_matrix * float4(world_pos - obj.position.xyz, 1.0f)).xyz;
    float d = evalPrimitive(lp, obj);
    cur_group = gid;

    if (gid < 0) {
      /* Ungrouped: combine directly into scene */
      if (scene_dist >= 1e9f) {
        /* Only union (op=0) can create geometry from nothing.
         * Subtraction/intersection with no base produces no surface. */
        if (obj.csg_operation == 0) {
          scene_dist = d;
          out_color = obj.color.rgb;
        }
      }
      else {
        float prev = scene_dist;
        scene_dist = combineCSG(
            scene_dist, d, obj.csg_operation, obj.blend_type, obj.blend,
            obj.shell_distance, obj.shell_mode);
        if (obj.csg_operation == 0 && d < prev) {
          float t = clamp(1.0f - (d - scene_dist) / max(obj.blend, 0.001f), 0.0f, 1.0f);
          out_color = mix(out_color, obj.color.rgb, t);
        }
      }
    }
    else {
      /* Grouped: accumulate into group CSG chain */
      if (!grp_has_hit) {
        if (obj.csg_operation == 0) {
          grp_dist = d;
          grp_color = obj.color.rgb;
          grp_has_hit = true;
        }
      }
      else {
        float prev = grp_dist;
        grp_dist = combineCSG(
            grp_dist, d, obj.csg_operation, obj.blend_type, obj.blend,
            obj.shell_distance, obj.shell_mode);
        if (obj.csg_operation == 0 && d < prev) {
          float t = clamp(1.0f - (d - grp_dist) / max(obj.blend, 0.001f), 0.0f, 1.0f);
          grp_color = mix(grp_color, obj.color.rgb, t);
        }
      }
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
  if (t < 0.33f)
    return mix(cool, medium, t / 0.33f);
  if (t < 0.66f)
    return mix(medium, warm, (t - 0.33f) / 0.33f);
  return mix(warm, hot, (t - 0.66f) / 0.34f);
}

void main()
{
  int2 pixel = ivec2(gl_GlobalInvocationID.xy);
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
  float3 inv_dir = 1.0f / ray_dir;

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
      if (index < 0) continue;

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
      if (separation > 0.0f) continue;

      if (node.child_a < 0) {
        float3 lt1 = (node.bounds_min.xyz - ray_origin) * inv_dir;
        float3 lt2 = (node.bounds_max.xyz - ray_origin) * inv_dir;
        float ltMin = max(max(min(lt1.x, lt2.x), min(lt1.y, lt2.y)), min(lt1.z, lt2.z));
        float ltMax = min(min(max(lt1.x, lt2.x), max(lt1.y, lt2.y)), max(lt1.z, lt2.z));
        if (ltMax < 0.0f || ltMin > ltMax) continue;

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
  float3 hit_color;
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
    float3 skip_color;
    float skip_aabb;
    float skip_d = evalSceneTile(skip_pos, skip_color, skip_aabb);
    if (skip_d > sdf_ray_epsilon * 2.0f) {
      t = cone_skip_target;
      if (skip_d < 1e9f) {
        d_prev = skip_d;
        t_prev = cone_skip_target - skip_d;
      }
    }
  }
#endif

  for (int step = 0; step < sdf_max_steps; step++) {
    if (t > t_exit) break;
    float3 pos = ray_origin + ray_dir * t;

    /* Combined evaluation + empty-space skip */
    float3 color;
    float d;
    float aabb_skip;
#ifdef USE_TILE_CULLING
    d = evalSceneTile(pos, color, aabb_skip);

    if (d >= 1e9f) {
      t += max(aabb_skip, sdf_ray_epsilon);
      prev_radius = 0.0f;
      step_length = 0.0f;
      if (t > t_exit) break;
      continue;
    }

    /* Safe step: clamp by skipped AABBs with fixed floor to prevent crawling.
     * Floor at 10x epsilon — safe for CSG cuts (features < 0.01 are sub-pixel). */
    if (aabb_skip < d) {
      d = max(aabb_skip, sdf_ray_epsilon * 10.0f);
    }
#else
    d = evalSceneBVH(pos, color, aabb_skip);

    /* If no AABB contains this point, skip by nearest AABB distance */
    if (d >= 1e9f) {
      t += max(aabb_skip, sdf_ray_epsilon);
      prev_radius = 0.0f;
      step_length = 0.0f;
      if (t > t_exit) break;
      continue;
    }

    if (aabb_skip < d) {
      d = max(aabb_skip, sdf_ray_epsilon * 10.0f);
    }
#endif

    steps_taken++;
    float abs_d = abs(d);

    /* Proper over-relaxation from Keinert et al. 2014 */
    bool sor_fail = omega > 1.0f && (abs_d + prev_radius) < step_length;
    if (sor_fail) {
      step_length -= omega * step_length;
      omega = 1.0f;
    } else {
      step_length = abs_d * step_factor * omega;
    }

    float adaptive_epsilon = sdf_ray_epsilon * (1.0f + t * 0.001f);

    if (!sor_fail && d < adaptive_epsilon) {
      hit = true;
      /* Surface projection with angle correction.
       * Estimate cos(theta) between ray and surface normal from the rate of
       * distance decrease: cos_theta ≈ (d_prev - d) / (t - t_prev).
       * Project along ray by d/cos_theta to reach the true d=0 surface. */
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
      hit_color = color;
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
    tile_heat[local_idx] = float(tile_active_obj_count()) / max(float(object_count), 1.0f);
    barrier();
    float max_heat = 0.0f;
    for (int i = 0; i < kTileSize * kTileSize; i++) {
      max_heat = max(max_heat, tile_heat[i]);
    }
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
    float heat = float(steps_taken) / 128.0f;
    tile_heat[local_idx] = heat;
    barrier();
    float max_heat = 0.0f;
    for (int i = 0; i < kTileSize * kTileSize; i++) {
      max_heat = max(max_heat, tile_heat[i]);
    }
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
    if (s_coneHitPos.w < 0.0f) cone_color = float3(0.0f);
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
    float heat = float(g_numNearShapes) / max(float(object_count), 1.0f);
    tile_heat[local_idx] = heat;
    barrier();
    float max_heat = 0.0f;
    for (int i = 0; i < kTileSize * kTileSize; i++) {
      max_heat = max(max_heat, tile_heat[i]);
    }
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
    float heat = float(steps_taken) / 128.0f;
    tile_heat[local_idx] = heat;
    barrier();
    float max_heat = 0.0f;
    for (int i = 0; i < kTileSize * kTileSize; i++) {
      max_heat = max(max_heat, tile_heat[i]);
    }
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
    imageStore(gbuf_normal_img, pixel, float4(0.0));
    return;
  }

  /* Output G-buffer */
  imageStore(gbuf_pos_img, pixel, float4(hit_pos, 1.0));
  imageStore(gbuf_color_img, pixel, float4(hit_color, 0.0));
  imageStore(gbuf_normal_img, pixel, float4(0.0));
}
