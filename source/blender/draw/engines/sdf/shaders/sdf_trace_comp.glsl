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

#ifdef USE_TILE_CULLING

#define kMaxTileObjects 256
#define kMaxGroupBits 128
#define kGroupBitWords (kMaxGroupBits / 32)

shared uint s_tileObjCount;
shared int s_tileObjList[kMaxTileObjects];
shared uint s_groupActive[kGroupBitWords];

#else

#define kAabbTreeStackSize 32
#define kMaxBitfieldBits 256
#define kBitfieldWords (kMaxBitfieldBits / 32)

uint g_bvh_bits[kBitfieldWords];
int g_numNearShapes;

bool is_shape_near(int idx)
{
  if (idx >= kMaxBitfieldBits) {
    return true;
  }
  return (g_bvh_bits[idx >> 5] & (1u << (uint(idx) & 31u))) != 0u;
}

#endif

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

bool aabb_overlaps_tile(float3 bmin, float3 bmax, float4 tile_ndc, float4x4 viewproj)
{
  float2 proj_min = float2(1e30f);
  float2 proj_max = float2(-1e30f);
  int behind_count = 0;

  for (int i = 0; i < 8; i++) {
    float3 corner = float3(
        (i & 1) != 0 ? bmax.x : bmin.x,
        (i & 2) != 0 ? bmax.y : bmin.y,
        (i & 4) != 0 ? bmax.z : bmin.z);
    float4 clip = viewproj * float4(corner, 1.0f);
    if (clip.w <= 0.0f) {
      behind_count++;
      continue;
    }
    float2 ndc = clip.xy / clip.w;
    proj_min = min(proj_min, ndc);
    proj_max = max(proj_max, ndc);
  }

  if (behind_count == 8) return false;
  if (behind_count > 0) return true;

  return !(proj_max.x < tile_ndc.x || proj_min.x > tile_ndc.z ||
           proj_max.y < tile_ndc.y || proj_min.y > tile_ndc.w);
}

float evalSceneTile(float3 world_pos, out float3 out_color)
{
  float scene_dist = 1e10f;
  out_color = float3(0.5f);

  for (int g = 0; g < group_count; g++) {
    bool group_active;
    if (g < kMaxGroupBits) {
      group_active = (s_groupActive[uint(g) >> 5u] & (1u << (uint(g) & 31u))) != 0u;
    }
    else {
      group_active = true;
    }
    if (!group_active) continue;

    SDFGroupGPU grp = groups[g];
    float grp_dist = 1e10f;
    float3 grp_color = grp.color.rgb;
    bool grp_has_hit = false;

    for (int m = grp.first_object; m < grp.first_object + grp.object_count; m++) {
      SDFObjectGPU obj = objects[m];
      float3 lp = (obj.inverse_matrix * float4(world_pos - obj.position.xyz, 1.0f)).xyz;
      float d = evalPrimitive(lp, obj);

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

    if (!grp_has_hit) continue;

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

  uint tile_count = min(s_tileObjCount, uint(kMaxTileObjects));
  for (uint u = 0u; u < tile_count; u++) {
    int i = s_tileObjList[u];
    SDFObjectGPU obj = objects[i];
    float3 lp = (obj.inverse_matrix * float4(world_pos - obj.position.xyz, 1.0f)).xyz;
    float d = evalPrimitive(lp, obj);

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

float evalSceneDistTile(float3 world_pos)
{
  float3 dummy_color;
  return evalSceneTile(world_pos, dummy_color);
}

int tile_active_obj_count()
{
  int cnt = int(min(s_tileObjCount, uint(kMaxTileObjects)));
  for (int g = 0; g < group_count; g++) {
    bool is_active;
    if (g < kMaxGroupBits) {
      is_active = (s_groupActive[uint(g) >> 5u] & (1u << (uint(g) & 31u))) != 0u;
    }
    else {
      is_active = true;
    }
    if (is_active) cnt += groups[g].object_count;
  }
  return cnt;
}

#else

float3 find_ortho(float3 v)
{
  if (abs(v.x) >= 0.57735f) {
    return float3(v.y, -v.x, 0.0f);
  }
  return float3(0.0f, v.z, -v.y);
}

float evalSceneBVH(float3 world_pos, out float3 out_color)
{
  float scene_dist = 1e10f;
  out_color = float3(0.5f);

  for (int g = 0; g < group_count; g++) {
    SDFGroupGPU grp = groups[g];
    float grp_dist = 1e10f;
    float3 grp_color = grp.color.rgb;
    bool grp_has_hit = false;

    for (int m = grp.first_object; m < grp.first_object + grp.object_count; m++) {
      if (!is_shape_near(m)) continue;

      SDFObjectGPU obj = objects[m];
      float3 lp = (obj.inverse_matrix * float4(world_pos - obj.position.xyz, 1.0f)).xyz;
      float d = evalPrimitive(lp, obj);

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

    if (!grp_has_hit) continue;

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

  for (int i = 0; i < object_count; i++) {
    if (!is_shape_near(i)) continue;

    SDFObjectGPU obj = objects[i];
    if (obj.group_id >= 0) continue;

    float3 lp = (obj.inverse_matrix * float4(world_pos - obj.position.xyz, 1.0f)).xyz;
    float d = evalPrimitive(lp, obj);

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

float evalSceneFull(float3 world_pos, out float3 out_color)
{
  float scene_dist = 1e10f;
  out_color = float3(0.5f);

  for (int g = 0; g < group_count; g++) {
    SDFGroupGPU grp = groups[g];
    float grp_dist = 1e10f;
    float3 grp_color = grp.color.rgb;
    bool grp_has_hit = false;

    for (int m = grp.first_object; m < grp.first_object + grp.object_count; m++) {
      SDFObjectGPU obj = objects[m];
      float3 lp = (obj.inverse_matrix * float4(world_pos - obj.position.xyz, 1.0f)).xyz;
      float d = evalPrimitive(lp, obj);

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

    if (!grp_has_hit) continue;

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

  for (int i = 0; i < object_count; i++) {
    SDFObjectGPU obj = objects[i];
    if (obj.group_id >= 0) continue;

    float3 lp = (obj.inverse_matrix * float4(world_pos - obj.position.xyz, 1.0f)).xyz;
    float d = evalPrimitive(lp, obj);

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

float evalSceneDistFull(float3 world_pos)
{
  float3 dummy_color;
  return evalSceneFull(world_pos, dummy_color);
}

float evalSceneDistBVH(float3 world_pos)
{
  float3 dummy_color;
  return evalSceneBVH(world_pos, dummy_color);
}

#endif

float3 heat_color(float t)
{
  float3 cool = float3(0.0, 1.0, 0.0);
  float3 medium = float3(1.0, 1.0, 0.0);
  float3 hot = float3(1.0, 0.0, 0.0);
  return t < 0.5f ? mix(cool, medium, t / 0.5f) : mix(medium, hot, (t - 0.5f) / 0.5f);
}

void main()
{
  int2 pixel = ivec2(gl_GlobalInvocationID.xy);
  int local_idx = int(gl_LocalInvocationID.y * kTileSize + gl_LocalInvocationID.x);

#ifdef USE_TILE_CULLING
  if (local_idx == 0) s_tileObjCount = 0u;
  if (local_idx < kGroupBitWords) s_groupActive[local_idx] = 0u;
  barrier();

  ViewMatrices vm = drw_view();
  {
    float4x4 viewproj = vm.winmat * vm.viewmat;
    float2 tile_lo = float2(gl_WorkGroupID.xy) * float(kTileSize);
    float2 tile_hi = min(tile_lo + float(kTileSize), float2(screen_size));
    float4 tile_ndc = float4(
        tile_lo / float2(screen_size) * 2.0f - 1.0f,
        tile_hi / float2(screen_size) * 2.0f - 1.0f);

    for (int i = local_idx; i < object_count; i += kTileSize * kTileSize) {
      SDFObjectGPU obj = objects[i];
      if (!aabb_overlaps_tile(obj.bbox_min.xyz, obj.bbox_max.xyz, tile_ndc, viewproj)) continue;
      if (obj.group_id >= 0) {
        if (obj.group_id < kMaxGroupBits) {
          atomicOr(s_groupActive[uint(obj.group_id) >> 5u], 1u << (uint(obj.group_id) & 31u));
        }
      }
      else {
        uint slot = atomicAdd(s_tileObjCount, 1u);
        if (slot < uint(kMaxTileObjects)) s_tileObjList[slot] = i;
      }
    }
  }
  barrier();
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

#ifndef USE_TILE_CULLING
  float3 ray_to = ray_origin + ray_dir * t_exit;

  g_numNearShapes = 0;
  for (int w = 0; w < kBitfieldWords; w++) {
    g_bvh_bits[w] = 0u;
  }

  if (use_bvh != 0 && bvh_root >= 0) {
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

  /* Sphere tracing */
  float t = t_enter;
  bool hit = false;
  float3 hit_pos;
  float3 hit_color;
  int steps_taken = 0;

  for (int step = 0; step < sdf_max_steps; step++) {
    steps_taken++;
    float3 pos = ray_origin + ray_dir * t;
    float3 color;
    float d;
#ifdef USE_TILE_CULLING
    d = evalSceneTile(pos, color);
#else
    if (use_bvh != 0) {
      d = evalSceneBVH(pos, color);
    }
    else {
      d = evalSceneFull(pos, color);
    }
#endif

    if (d < sdf_ray_epsilon) {
      hit = true;
      hit_pos = pos;
      hit_color = color;
      break;
    }

    t += max(d, sdf_ray_epsilon);
    if (t > t_exit) {
      break;
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
    float heat = float(steps_taken) / float(sdf_max_steps);
    imageStore(out_color_img, pixel, float4(heat_color(heat), 1.0f));
    imageStore(out_depth_img, pixel, float4(0.0f));
    imageStore(gbuf_pos_img, pixel, float4(0.0));
    imageStore(gbuf_color_img, pixel, float4(0.0));
    return;
  }
  else if (debug_bvh_views == 4) {
    float heat = float(steps_taken) / float(sdf_max_steps);
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
#else
  if (debug_bvh_views == 1) {
    float heat = float(use_bvh != 0 ? g_numNearShapes : object_count) / max(float(object_count), 1.0f);
    imageStore(out_color_img, pixel, float4(heat_color(heat), 1.0f));
    imageStore(out_depth_img, pixel, float4(0.0f));
    imageStore(gbuf_pos_img, pixel, float4(0.0));
    imageStore(gbuf_color_img, pixel, float4(0.0));
    return;
  }
  else if (debug_bvh_views == 2) {
    float heat = float(use_bvh != 0 ? g_numNearShapes : object_count) / max(float(object_count), 1.0f);
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
    float heat = float(steps_taken) / float(sdf_max_steps);
    imageStore(out_color_img, pixel, float4(heat_color(heat), 1.0f));
    imageStore(out_depth_img, pixel, float4(0.0f));
    imageStore(gbuf_pos_img, pixel, float4(0.0));
    imageStore(gbuf_color_img, pixel, float4(0.0));
    return;
  }
  else if (debug_bvh_views == 4) {
    float heat = float(steps_taken) / float(sdf_max_steps);
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
#endif

  if (!hit) {
    imageStore(gbuf_pos_img, pixel, float4(0.0));
    imageStore(gbuf_color_img, pixel, float4(0.0));
    return;
  }

  /* Output G-buffer */
  imageStore(gbuf_pos_img, pixel, float4(hit_pos, 1.0));
  imageStore(gbuf_color_img, pixel, float4(hit_color, 0.0));
}
