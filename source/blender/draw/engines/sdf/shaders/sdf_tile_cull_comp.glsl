/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Tile cull pass: builds per-tile primitive lists from pre-projected AABBs.
 * Two-pass: intersection/push objects get guaranteed slots first,
 * then regular objects fill remaining slots. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_tile_cull_comp)

#define SDF_CSG_OP_INTERSECT 2
#define SDF_CSG_OP_PUSH 4

shared uint s_tileObjCount;
shared int s_tileObjList[kMaxTileObjects];

void main()
{
  int local_idx = int(gl_LocalInvocationID.y * kTileSize + gl_LocalInvocationID.x);

  if (local_idx == 0) {
    s_tileObjCount = 0u;
  }
  barrier();

  int tile_min_x = int(gl_WorkGroupID.x) * kTileSize;
  int tile_min_y = int(gl_WorkGroupID.y) * kTileSize;
  int tile_max_x = min(tile_min_x + kTileSize, screen_size.x);
  int tile_max_y = min(tile_min_y + kTileSize, screen_size.y);

  /* Pass 1: intersection/push objects get guaranteed slots */
  for (int i = local_idx; i < object_count; i += kTileSize * kTileSize) {
    int op = objects[i].csg_operation;
    if (op != SDF_CSG_OP_INTERSECT && op != SDF_CSG_OP_PUSH) {
      continue;
    }
    int4 aabb = screen_aabbs[i];
    if (aabb.z < aabb.x) { continue; }
    if (aabb.z < tile_min_x || aabb.x > tile_max_x ||
        aabb.w < tile_min_y || aabb.y > tile_max_y) {
      continue;
    }
    uint slot = atomicAdd(s_tileObjCount, 1u);
    if (slot < uint(kMaxTileObjects)) {
      s_tileObjList[slot] = i;
    }
  }
  barrier();

  /* Pass 2: regular objects fill remaining slots */
  for (int i = local_idx; i < object_count; i += kTileSize * kTileSize) {
    int op = objects[i].csg_operation;
    if (op == SDF_CSG_OP_INTERSECT || op == SDF_CSG_OP_PUSH) {
      continue;
    }
    int4 aabb = screen_aabbs[i];
    if (aabb.z < aabb.x) { continue; }
    if (aabb.z < tile_min_x || aabb.x > tile_max_x ||
        aabb.w < tile_min_y || aabb.y > tile_max_y) {
      continue;
    }
    uint slot = atomicAdd(s_tileObjCount, 1u);
    if (slot < uint(kMaxTileObjects)) {
      s_tileObjList[slot] = i;
    }
  }
  barrier();

  /* Bitonic sort — CSG evaluation requires ascending index order */
  {
    uint n = min(s_tileObjCount, uint(kMaxTileObjects));

    for (uint i = n + uint(local_idx); i < uint(kMaxTileObjects); i += 64u) {
      s_tileObjList[i] = 0x7FFFFFFF;
    }
    barrier();

    for (uint k = 2u; k <= uint(kMaxTileObjects); k <<= 1u) {
      for (uint j = k >> 1u; j > 0u; j >>= 1u) {
        for (uint t = uint(local_idx); t < uint(kMaxTileObjects) / 2u; t += 64u) {
          uint block = t / j;
          uint offset = t & (j - 1u);
          uint i = block * 2u * j + offset;
          uint partner = i + j;
          bool ascending = ((i & k) == 0u);
          int a = s_tileObjList[i];
          int b = s_tileObjList[partner];
          int lo = min(a, b);
          int hi = max(a, b);
          s_tileObjList[i] = ascending ? lo : hi;
          s_tileObjList[partner] = ascending ? hi : lo;
        }
        barrier();
      }
    }

    if (local_idx == 0) {
      s_tileObjCount = n;
    }
  }
  barrier();

  /* Write to SSBOs */
  int tilesX = (screen_size.x + kTileSize - 1) / kTileSize;
  int tileIdx = int(gl_WorkGroupID.x) + int(gl_WorkGroupID.y) * tilesX;
  uint n = min(s_tileObjCount, uint(kMaxTileObjects));

  if (local_idx == 0) {
    tile_prim_counts[tileIdx] = int(n);
  }

  int base = tileIdx * kMaxTileObjects;
  for (uint i = uint(local_idx); i < n; i += 64u) {
    tile_prim_lists[base + int(i)] = s_tileObjList[i];
  }
}
