/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Tile cull pass: builds per-tile primitive lists into SSBOs. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_tile_cull_comp)

#include "draw_view_lib.glsl"

#define kTileSize 8
#define kMaxTileObjects 256

shared uint s_tileObjCount;
shared int s_tileObjList[kMaxTileObjects];

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

void main()
{
  int local_idx = int(gl_LocalInvocationID.y * kTileSize + gl_LocalInvocationID.x);

  if (local_idx == 0) s_tileObjCount = 0u;
  barrier();

  ViewMatrices vm = drw_view();
  float4x4 viewproj = vm.winmat * vm.viewmat;
  float2 tile_lo = float2(gl_WorkGroupID.xy) * float(kTileSize);
  float2 tile_hi = min(tile_lo + float(kTileSize), float2(screen_size));
  float4 tile_ndc = float4(
      tile_lo / float2(screen_size) * 2.0f - 1.0f,
      tile_hi / float2(screen_size) * 2.0f - 1.0f);

  for (int i = local_idx; i < object_count; i += kTileSize * kTileSize) {
    SDFObjectGPU obj = objects[i];
    if (obj._pad0 == 0) continue;
    if (!aabb_overlaps_tile(obj.bbox_min.xyz, obj.bbox_max.xyz, tile_ndc, viewproj)) continue;

    uint slot = atomicAdd(s_tileObjCount, 1u);
    if (slot < uint(kMaxTileObjects)) s_tileObjList[slot] = i;
  }
  barrier();

  /* Thread 0: sort + expand groups + write to SSBOs */
  if (local_idx == 0) {
    int n = int(min(s_tileObjCount, uint(kMaxTileObjects)));

    /* Insertion sort by index preserves CSG group ordering */
    for (int i = 1; i < n; i++) {
      int key = s_tileObjList[i];
      int j = i - 1;
      while (j >= 0 && s_tileObjList[j] > key) {
        s_tileObjList[j + 1] = s_tileObjList[j];
        j--;
      }
      s_tileObjList[j + 1] = key;
    }

    s_tileObjCount = uint(n);
  }
  barrier();

  /* All threads cooperatively write shared → SSBOs */
  int tilesX = (screen_size.x + 7) / 8;
  int tileIdx = int(gl_WorkGroupID.x) + int(gl_WorkGroupID.y) * tilesX;

  if (local_idx == 0) {
    tile_prim_counts[tileIdx] = int(s_tileObjCount);
  }

  uint n = min(s_tileObjCount, uint(kMaxTileObjects));
  int base = tileIdx * kMaxTileObjects;
  for (uint i = uint(local_idx); i < n; i += 64u) {
    tile_prim_lists[base + int(i)] = s_tileObjList[i];
  }
}
