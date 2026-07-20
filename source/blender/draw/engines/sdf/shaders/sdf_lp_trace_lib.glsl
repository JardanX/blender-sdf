/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Helpers shared between the LP march and resolve passes: cell lookup and the
 * pruned-tree distance evaluation. Expects the including shader to declare
 * (via its CREATE_INFO): lp_cell_meta[], lp_active_in[], lp_nodes[],
 * lp_binary_ops[], lp_prims[] and the culling_enabled / total_num_nodes push
 * constants. */

#pragma once

int lp_cell_num_active(int cell_idx)
{
  return lp_cell_meta[cell_idx].x;
}

float lp_sdf(float3 p, int cell_idx, out bool near_field)
{
  if (culling_enabled == 0) {
    near_field = true;
    return lp_list_eval(p, total_num_nodes, 0);
  }
  /* Single meta load: num_active (x), list offset (y), far value (z). */
  int4 meta = lp_cell_meta[cell_idx];
  int num_active = meta.x;
  if (num_active == SDF_LP_FALLBACK_LIST) {
    /* Cell overflowed the dynamic pools during pruning: full tree eval. */
    near_field = true;
    return lp_list_eval(p, total_num_nodes, 0);
  }
  if (num_active == 0) {
    near_field = false;
    return intBitsToFloat(meta.z);
  }
  near_field = true;
  return lp_list_eval(p, num_active, meta.y);
}

/* Dominant-object id at p (seed for gbuf_color.a, consumed by object
 * picking). Same cell dispatch as lp_sdf: far-field and overflowed cells
 * evaluate the full tree (rare; costs one eval). */
float lp_sdf_obj_id(float3 p, int cell_idx)
{
  if (culling_enabled == 0) {
    return lp_list_eval_obj_id(p, total_num_nodes, 0);
  }
  int4 meta = lp_cell_meta[cell_idx];
  int num_active = meta.x;
  if (num_active == SDF_LP_FALLBACK_LIST || num_active == 0) {
    return lp_list_eval_obj_id(p, total_num_nodes, 0);
  }
  return lp_list_eval_obj_id(p, num_active, meta.y);
}
