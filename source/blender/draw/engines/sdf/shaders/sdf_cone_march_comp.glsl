/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Cone march pre-pass: one thread per tile, finds the nearest object AABB
 * hit along the tile center ray. Stores 3D hit position in tile_hit_pos
 * SSBO for the per-pixel trace pass to project onto each pixel's ray.
 *
 * Uses AABB ray intersection only — no SDF evaluation — so this pass is
 * near-free (~0.1ms). The per-pixel pass then starts sphere tracing from
 * the projected AABB hit distance instead of the scene AABB entry. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_cone_march_comp)

#include "draw_view_lib.glsl"

void main()
{
  int tileX = int(gl_GlobalInvocationID.x);
  int tileY = int(gl_GlobalInvocationID.y);
  int tilesX = (screen_size.x + 7) / 8;
  int tilesY = (screen_size.y + 7) / 8;
  if (tileX >= tilesX || tileY >= tilesY) return;

  int tileIdx = tileX + tileY * tilesX;

  if (object_count == 0) {
    tile_hit_pos[tileIdx] = float4(0.0f, 0.0f, 0.0f, -1.0f);
    return;
  }

  /* Build ray through tile center */
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
  float3 inv_dir = 1.0f / rd;

  /* Find nearest AABB hit across all objects */
  float nearest_t = 1e30f;

  for (int i = 0; i < object_count; i++) {
    float3 bmin = objects[i].bbox_min.xyz;
    float3 bmax = objects[i].bbox_max.xyz;

    float3 t0 = (bmin - ro) * inv_dir;
    float3 t1 = (bmax - ro) * inv_dir;
    float3 t_lo = min(t0, t1);
    float3 t_hi = max(t0, t1);
    float t_near = max(max(t_lo.x, t_lo.y), t_lo.z);
    float t_far = min(min(t_hi.x, t_hi.y), t_hi.z);

    if (t_near <= t_far && t_far >= 0.0f) {
      nearest_t = min(nearest_t, max(t_near, 0.0f));
    }
  }

  if (nearest_t >= 1e29f) {
    tile_hit_pos[tileIdx] = float4(0.0f, 0.0f, 0.0f, -1.0f);
    return;
  }

  float3 hit_pos = ro + nearest_t * rd;
  tile_hit_pos[tileIdx] = float4(hit_pos, nearest_t);
}
