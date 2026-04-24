/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Build conservative per-brick distance hierarchy levels from baked cell minima. */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_build_hierarchy)

#define BRICK_SIZE 8
#define BRICK_STORAGE 12

shared float shared_cell_smin[BRICK_SIZE * BRICK_SIZE * BRICK_SIZE];
shared float shared_level2[8];

int slotToIndex(int x, int y, int z)
{
  return x + y * BRICK_SIZE + z * BRICK_SIZE * BRICK_SIZE;
}

int level2Index(int x, int y, int z)
{
  return x + y * 2 + z * 4;
}

float fetchCellSmin(int3 slot_block, int3 cell)
{
  int3 base = slot_block * BRICK_STORAGE + cell + int3(2);
  float smin = 1e20f;
  for (int dz = 0; dz < 2; dz++) {
    for (int dy = 0; dy < 2; dy++) {
      for (int dx = 0; dx < 2; dx++) {
        smin = min(smin, texelFetch(compact_atlas, base + int3(dx, dy, dz), 0).r);
      }
    }
  }
  return smin;
}

void main()
{
  int brick_idx = int(gl_WorkGroupID.x) + int(gl_WorkGroupID.y) * dispatch_width;
  if (brick_idx >= active_brick_count) {
    return;
  }

  int slot = active_bricks[brick_idx].coord.w;
  int bpa = bricks_per_axis;
  int3 slot_block = int3(slot % bpa, (slot / bpa) % bpa, slot / (bpa * bpa));

  int lx = int(gl_LocalInvocationID.x);
  int ly = int(gl_LocalInvocationID.y);

  for (int z = 0; z < BRICK_SIZE; z++) {
    int3 cell = int3(lx, ly, z);
    shared_cell_smin[slotToIndex(cell.x, cell.y, cell.z)] = fetchCellSmin(slot_block, cell);
  }
  barrier();

  if (lx < 4 && ly < 4) {
    int3 slot_block4 = slot_block * 4;
    for (int z = 0; z < 4; z++) {
      int3 macro = int3(lx, ly, z);
      float smin = 1e20f;
      for (int dz = 0; dz < 2; dz++) {
        for (int dy = 0; dy < 2; dy++) {
          for (int dx = 0; dx < 2; dx++) {
            int3 child = macro * 2 + int3(dx, dy, dz);
            smin = min(smin, shared_cell_smin[slotToIndex(child.x, child.y, child.z)]);
          }
        }
      }
      imageStore(hierarchy4_atlas, slot_block4 + macro, float4(smin));
    }
  }

  if (lx < 2 && ly < 2) {
    int3 slot_block2 = slot_block * 2;
    for (int z = 0; z < 2; z++) {
      int3 macro = int3(lx, ly, z);
      float smin = 1e20f;
      for (int dz = 0; dz < 4; dz++) {
        for (int dy = 0; dy < 4; dy++) {
          for (int dx = 0; dx < 4; dx++) {
            int3 child = macro * 4 + int3(dx, dy, dz);
            smin = min(smin, shared_cell_smin[slotToIndex(child.x, child.y, child.z)]);
          }
        }
      }
      shared_level2[level2Index(macro.x, macro.y, macro.z)] = smin;
      imageStore(hierarchy2_atlas, slot_block2 + macro, float4(smin));
    }
  }
  barrier();

  if (gl_LocalInvocationIndex == 0u) {
    float smin = 1e20f;
    for (int i = 0; i < 8; i++) {
      smin = min(smin, shared_level2[i]);
    }
    imageStore(hierarchy1_atlas, slot_block, float4(smin));
  }
}
