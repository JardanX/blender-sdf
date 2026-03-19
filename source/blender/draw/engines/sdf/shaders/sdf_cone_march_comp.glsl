/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Cone march pre-pass: one thread per tile, coarse sphere trace using
 * tile-culled primitive lists from tile_prim_counts/tile_prim_lists SSBOs.
 * Finds approximate surface distance along tile center ray and writes
 * hit position to tile_hit_pos SSBO for the per-pixel trace to skip. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_cone_march_comp)

#include "draw_view_lib.glsl"
#include "sdf_lib.glsl"

#define kMaxTileObjects 256

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

/* Evaluate scene SDF using tile-culled prim list (same logic as evalSceneTile). */
float evalSceneCone(float3 world_pos, int tile_count, int base_offset, out float out_aabb_skip)
{
  float scene_dist = 1e10f;
  float3 dummy_color = float3(0.5f);
  out_aabb_skip = 1e30f;

  int cur_group = -2;
  float grp_dist = 1e10f;
  float3 grp_color = float3(0.5f);
  bool grp_has_hit = false;

  for (int u = 0; u <= tile_count; u++) {
    int i = (u < tile_count) ? tile_prim_lists[base_offset + u] : -1;
    int gid = (i >= 0) ? objects[i].group_id : -2;

    if (gid != cur_group && grp_has_hit) {
      flushGroup(cur_group, grp_dist, grp_color, scene_dist, dummy_color);
      grp_has_hit = false;
      grp_dist = 1e10f;
    }

    if (u >= tile_count) break;

    SDFObjectGPU obj = objects[i];

    /* Per-step AABB skip */
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
      if (scene_dist >= 1e9f) {
        if (obj.csg_operation == 0) {
          scene_dist = d;
        }
      }
      else {
        scene_dist = combineCSG(
            scene_dist, d, obj.csg_operation, obj.blend_type, obj.blend,
            obj.shell_distance, obj.shell_mode);
      }
    }
    else {
      if (!grp_has_hit) {
        if (obj.csg_operation == 0) {
          grp_dist = d;
          grp_has_hit = true;
        }
      }
      else {
        grp_dist = combineCSG(
            grp_dist, d, obj.csg_operation, obj.blend_type, obj.blend,
            obj.shell_distance, obj.shell_mode);
      }
    }
  }

  return scene_dist;
}

void main()
{
  /* Super-tile: each thread covers a 2×2 block of 8×8 tiles (16×16 pixels) */
  int stX = int(gl_GlobalInvocationID.x);
  int stY = int(gl_GlobalInvocationID.y);
  int stTilesX = (screen_size.x + 15) / 16;
  int stTilesY = (screen_size.y + 15) / 16;
  if (stX >= stTilesX || stY >= stTilesY) return;

  int stIdx = stX + stY * stTilesX;

  /* Read prim list from one of the 4 sub-tiles (center-biased) */
  int tilesX = (screen_size.x + 7) / 8;
  int subTileX = min(stX * 2 + 1, tilesX - 1);
  int subTileY = min(stY * 2 + 1, ((screen_size.y + 7) / 8) - 1);
  int subTileIdx = subTileX + subTileY * tilesX;
  int tile_count = min(tile_prim_counts[subTileIdx], kMaxTileObjects);

  if (tile_count == 0) {
    tile_hit_pos[stIdx] = float4(0.0f, 0.0f, 0.0f, -1.0f);
    return;
  }

  /* Build ray through super-tile center (16×16 pixel area) */
  float2 tile_center = (float2(stX, stY) + 0.5f) * 16.0f;
  float2 uv = tile_center / float2(screen_size);

  ViewMatrices vm = drw_view();
  float4x4 view_inv = vm.viewinv;
  float4x4 win_inv = vm.wininv;

  float4 ndc_near = float4(uv * 2.0f - 1.0f, -1.0f, 1.0f);
  float4 ndc_far = float4(uv * 2.0f - 1.0f, 1.0f, 1.0f);
  float4 world_near = view_inv * (win_inv * ndc_near);
  float4 world_far = view_inv * (win_inv * ndc_far);
  world_near.xyz /= world_near.w;
  world_far.xyz /= world_far.w;

  float3 ro = world_near.xyz;
  float3 rd = normalize(world_far.xyz - world_near.xyz);
  float3 inv_dir = 1.0f / rd;

  /* Scene AABB ray clipping */
  float t_enter = 0.0f;
  float t_exit = 1000.0f;
  bool scene_aabb_valid = all(lessThan(scene_aabb_min, scene_aabb_max));

  if (scene_aabb_valid) {
    float3 t0 = (scene_aabb_min - ro) * inv_dir;
    float3 t1 = (scene_aabb_max - ro) * inv_dir;
    float3 t_lo = min(t0, t1);
    float3 t_hi = max(t0, t1);
    t_enter = max(max(t_lo.x, t_lo.y), t_lo.z);
    t_exit = min(min(t_hi.x, t_hi.y), t_hi.z);

    if (t_enter > t_exit || t_exit < 0.0f) {
      tile_hit_pos[stIdx] = float4(0.0f, 0.0f, 0.0f, -1.0f);
      return;
    }
    t_enter = max(t_enter, 0.0f);
    t_exit = min(t_exit, 1000.0f);
  }

  float tan_half_tile = float(16) / float(screen_size.y);
  float cone_epsilon = sdf_ray_epsilon * 8.0f;
  int base_offset = subTileIdx * kMaxTileObjects;

  float t = t_enter;
  float t_safe = t_enter;
  for (int step = 0; step < 16; step++) {
    float3 pos = ro + rd * t;
    float aabb_skip;
    float d = evalSceneCone(pos, tile_count, base_offset, aabb_skip);

    if (d >= 1e9f) {
      t += max(aabb_skip, cone_epsilon);
      if (t > t_exit) break;
      continue;
    }

    if (aabb_skip < d) {
      d = max(aabb_skip, cone_epsilon);
    }

    float cone_r = t * tan_half_tile * 1.25f;
    if (d < cone_r) {
      tile_hit_pos[stIdx] = float4(ro + rd * t_safe, t_safe);
      return;
    }

    t_safe = t;
    t += d;
    if (t > t_exit) break;
  }

  /* Didn't find surface in 16 steps — still write t_safe for partial skip */
  if (t_safe > t_enter) {
    tile_hit_pos[stIdx] = float4(ro + rd * t_safe, t_safe);
  }
  else {
    tile_hit_pos[stIdx] = float4(0.0f, 0.0f, 0.0f, -1.0f);
  }
}
