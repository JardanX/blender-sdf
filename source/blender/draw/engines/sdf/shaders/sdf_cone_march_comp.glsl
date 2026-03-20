/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Cone march pass: one thread per tile, coarse sphere trace using
 * tile-culled prim lists. Zeros tile_prim_counts for empty tiles,
 * writes hit position to tile_hit_pos for non-empty tiles. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_cone_march_comp)

#include "draw_view_lib.glsl"
#include "sdf_lib.glsl"

#define kMaxTileObjects 512

float evalSceneCone(float3 world_pos, int tile_count, int base_offset, out float out_aabb_skip)
{
  float scene_dist = 1e10f;
  float3 dummy_color = float3(0.5f);
  out_aabb_skip = 1e30f;

  int cur_group = -2;
  float grp_dist = 1e10f;
  float3 grp_color = float3(0.5f);
  bool grp_has_hit = false;

  for (int u = 0; u < tile_count; u++) {
    int i = tile_prim_lists[base_offset + u];

    /* Partial reads: group_id + AABB only */
    int gid = objects[i].group_id;

    /* Group boundary flush */
    if (gid != cur_group && grp_has_hit) {
      flushGroup(cur_group, grp_dist, grp_color, scene_dist, dummy_color);
      grp_has_hit = false;
      grp_dist = 1e10f;
    }

    /* AABB skip (partial read) */
    float da = point_aabb_dist(world_pos, objects[i].bbox_min.xyz, objects[i].bbox_max.xyz);
    float max_group_blend = intBitsToFloat(objects[i]._pad3);
    float skip_threshold = max(sdf_ray_epsilon, max_group_blend);
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
        if (obj.csg_operation == 0) { scene_dist = d; }
      }
      else {
        scene_dist = combineCSG(
            scene_dist, d, obj.csg_operation, obj.blend_type, obj.blend,
            obj.shell_distance, obj.shell_mode, obj.chamfer_k2, obj.chamfer_k3);
      }
    }
    else {
      if (!grp_has_hit) {
        if (obj.csg_operation == 0) { grp_dist = d; grp_has_hit = true; }
      }
      else {
        grp_dist = combineCSG(
            grp_dist, d, obj.csg_operation, obj.blend_type, obj.blend,
            obj.shell_distance, obj.shell_mode, obj.chamfer_k2, obj.chamfer_k3);
      }
    }
  }

  /* Flush final group */
  if (grp_has_hit) {
    flushGroup(cur_group, grp_dist, grp_color, scene_dist, dummy_color);
  }

  return scene_dist;
}

void main()
{
  int tileX = int(gl_GlobalInvocationID.x);
  int tileY = int(gl_GlobalInvocationID.y);
  int tilesX = (screen_size.x + 7) / 8;
  int tilesY = (screen_size.y + 7) / 8;
  if (tileX >= tilesX || tileY >= tilesY) return;

  int tileIdx = tileX + tileY * tilesX;
  int tile_count = min(tile_prim_counts[tileIdx], kMaxTileObjects);

  if (tile_count == 0) {
    tile_hit_pos[tileIdx] = float4(0.0f, 0.0f, 0.0f, -1.0f);
    tile_far_hint[tileIdx] = 0.0f;
    return;
  }

  float2 tile_center = (float2(tileX, tileY) + 0.5f) * 8.0f;
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
  float3 safe_rd = float3(
      abs(rd.x) < 1e-8f ? 1e-8f : rd.x,
      abs(rd.y) < 1e-8f ? 1e-8f : rd.y,
      abs(rd.z) < 1e-8f ? 1e-8f : rd.z);
  float3 inv_dir = 1.0f / safe_rd;

  bool is_persp = drw_view_is_perspective();

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
      tile_prim_counts[tileIdx] = 0;
      tile_hit_pos[tileIdx] = float4(0.0f, 0.0f, 0.0f, -1.0f);
      return;
    }
    t_enter = max(t_enter, 0.0f);
    t_exit = min(t_exit, 1000.0f);
  }

  float tan_half_tile = float(8) / float(screen_size.y);
  /* Ortho: tile world-space half-size is constant, not distance-dependent.
   * Use a larger radius than perspective to trigger earlier — this pushes the
   * skip position further from the surface, giving the trace enough steps to
   * converge consistently across tile boundaries. */
  float ortho_cone_r = 0.0f;
  float ortho_tile_world = 0.0f;
  if (!is_persp) {
    float ortho_height = 2.0f / vm.winmat[1][1];
    ortho_tile_world = (ortho_height / float(screen_size.y)) * 8.0f;
    ortho_cone_r = ortho_tile_world * sdf_cone_aperture;
  }
  float cone_epsilon = sdf_ray_epsilon * 32.0f;
  int base_offset = tileIdx * kMaxTileObjects;

  float t = t_enter;
  float t_safe = t_enter;
  float d_safe = 0.0f;
  float cone_r_safe = 0.0f;
  for (int step = 0; step < sdf_cone_steps; step++) {
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

    float cone_r = is_persp ? t * tan_half_tile * sdf_cone_aperture : ortho_cone_r;
    if (d < cone_r) {
      float margin = is_persp ? cone_r_safe * 3.0f : ortho_tile_world * 3.0f;
      float t_skip = max(t_safe + max(d_safe - margin, 0.0f), t_enter);
      tile_hit_pos[tileIdx] = float4(ro + rd * t_skip, t_skip);
      tile_far_hint[tileIdx] = t_exit;
      return;
    }

    t_safe = t;
    d_safe = d;
    cone_r_safe = cone_r;
    t += d;
    if (t > t_exit) break;
  }

  /* Traversed full range without finding surface — tile is empty */
  if (t >= t_exit && t_safe > t_enter) {
    tile_prim_counts[tileIdx] = 0;
    tile_hit_pos[tileIdx] = float4(0.0f, 0.0f, 0.0f, -1.0f);
    tile_far_hint[tileIdx] = 0.0f;
    return;
  }

  /* Ran out of steps — write best safe skip */
  if (t_safe > t_enter) {
    float margin = is_persp ? cone_r_safe * 3.0f : ortho_tile_world * 3.0f;
    float t_skip = max(t_safe + max(d_safe - margin, 0.0f), t_enter);
    tile_hit_pos[tileIdx] = float4(ro + rd * t_skip, t_skip);
    tile_far_hint[tileIdx] = t_exit;
  }
  else {
    tile_hit_pos[tileIdx] = float4(0.0f, 0.0f, 0.0f, -1.0f);
    tile_far_hint[tileIdx] = 0.0f;
  }
}
