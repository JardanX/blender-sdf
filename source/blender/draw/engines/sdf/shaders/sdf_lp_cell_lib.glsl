/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Lipschitz pruning: cell indexing and heatmap helpers. Kept separate from
 * sdf_lp_common.glsl so the debug colorize pass (sdf_lp_debug_comp) can use
 * them without pulling in the whole primitive evaluation library. */

#pragma once

/* ------------------------------------------------------------------ */
/** \name Morton cell indexing (64 consecutive codes per 4x4x4 block,
 * so parent cell = cell_idx / 64)
 * \{ */

uint lp_part1by2(uint x)
{
  x &= 0x000003ffu;
  x = (x ^ (x << 16)) & 0xff0000ffu;
  x = (x ^ (x << 8)) & 0x0300f00fu;
  x = (x ^ (x << 4)) & 0x030c30c3u;
  x = (x ^ (x << 2)) & 0x09249249u;
  return x;
}

uint lp_cell_idx(int3 cell)
{
  return (lp_part1by2(uint(cell.z)) << 2) + (lp_part1by2(uint(cell.y)) << 1) +
         lp_part1by2(uint(cell.x));
}

int3 lp_cell_from_pos(float3 p, float3 aabb_min, float3 cell_size, int grid_size)
{
  int3 cell = int3((p - aabb_min) / cell_size);
  return clamp(cell, int3(0), int3(grid_size - 1));
}

/** \} */

/* ------------------------------------------------------------------ */
/** \name Heatmap (inferno polynomial approximation)
 * \{ */

float3 lp_inferno(float t)
{
  const float3 c0 = float3(0.0002189403691192265f, 0.001651004631001012f, -0.01948089843709184f);
  const float3 c1 = float3(0.1065134194856116f, 0.5639564367884091f, 3.932712388889277f);
  const float3 c2 = float3(11.60249308247187f, -3.972853965665698f, -15.9423941062914f);
  const float3 c3 = float3(-41.70399613139459f, 17.43639888205313f, 44.35414519872813f);
  const float3 c4 = float3(77.162935699427f, -33.40235894210092f, -81.80730925738993f);
  const float3 c5 = float3(-71.31942824499214f, 32.62606426397723f, 73.20951985803202f);
  const float3 c6 = float3(25.13112622477341f, -12.24266895238567f, -23.07032500287172f);
  return c0 + t * (c1 + t * (c2 + t * (c3 + t * (c4 + t * (c5 + t * c6)))));
}

/** \} */
