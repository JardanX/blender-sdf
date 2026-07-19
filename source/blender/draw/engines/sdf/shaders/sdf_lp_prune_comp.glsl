/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Lipschitz pruning pass: one thread per grid cell.
 *
 * Hierarchical construction: level `grid_size` reads the active node lists of
 * the parent level (grid_size/4) and produces, for each cell, a compacted
 * list of the nodes that can influence the SDF inside the cell. A binary op
 * child is pruned when its value at the cell center differs from its sibling
 * by more than 2R + k (R = half the cell diagonal, k = blend radius): the
 * dominated subtree cannot win the min()/max() anywhere inside the cell.
 * Cells whose center distance exceeds 2R are "far field" and only store a
 * constant lower bound instead of a node list.
 *
 * Port of culling.comp.glsl / common_culling.glsl from the reference engine.
 * The subgroup-based scratch allocation of the original is replaced by a
 * workgroup-local one (64 lanes per 4x4x4 workgroup).
 */

#include "infos/sdf_shader_infos.hh"

COMPUTE_SHADER_CREATE_INFO(sdf_lp_prune_comp)

#include "sdf_lp_common.glsl"

/* Tmp bit layout: state(2) | active_global(1) | inactive_ancestors(1) | sign(1) | parent(16). */
#define LP_TMP_STATE_MASK 3u
#define LP_NODESTATE_INACTIVE 0u
#define LP_NODESTATE_SKIPPED 1u
#define LP_NODESTATE_ACTIVE 2u

uint lp_tmp_state(uint t)
{
  return t & LP_TMP_STATE_MASK;
}

void lp_tmp_write_state(inout uint t, uint state)
{
  t = (t & ~LP_TMP_STATE_MASK) | state;
}

bool lp_tmp_active_global(uint t)
{
  return ((t >> 2) & 1u) != 0u;
}

bool lp_tmp_inactive_ancestors(uint t)
{
  return ((t >> 3) & 1u) != 0u;
}

bool lp_tmp_sign(uint t)
{
  return ((t >> 4) & 1u) != 0u;
}

uint lp_tmp_parent(uint t)
{
  return (t >> 5) & 0xffffu;
}

uint lp_tmp_pack(uint state, bool active_global, bool inactive_ancestors, bool sign, uint parent)
{
  return state | (uint(active_global) << 2) | (uint(inactive_ancestors) << 3) |
         (uint(sign) << 4) | ((parent & 0xffffu) << 5);
}

shared int s_tmp_offset;

void main()
{
  int3 cell = int3(gl_GlobalInvocationID.xyz);
  int cell_idx = int(lp_cell_idx(cell));

  float3 cell_size = (aabb_max - aabb_min) / float(grid_size);
  float3 cell_center = aabb_min + cell_size * (float3(cell) + 0.5f);
  float R = length(cell_size) * 0.5f;

  int parent_cell_idx = 0;
  int parent_offset = 0;
  int num_nodes = total_num_nodes;
  if (first_lvl == 0) {
    parent_cell_idx = cell_idx / 64;
    parent_offset = lp_cell_meta_in[parent_cell_idx].y;
    num_nodes = lp_cell_meta_in[parent_cell_idx].x;
  }

  /* Workgroup scratch allocation: one column of num_nodes entries per lane.
   * Must be reached by every thread in the workgroup (barrier below). */
  int lane = int(gl_LocalInvocationIndex);
  if (lane == 0) {
    s_tmp_offset = (num_nodes > 1) ? atomicAdd(lp_counters[16 + counter_slot], 64 * num_nodes) : 0;
  }
  barrier();
  int tmp_offset = s_tmp_offset;

  if (num_nodes == 0) {
    /* Parent was far field: propagate the constant value. */
    lp_cell_meta_out[cell_idx].x = 0;
    lp_cell_meta_out[cell_idx].z = (first_lvl == 0) ? lp_cell_meta_in[parent_cell_idx].z :
                                                      floatBitsToInt(0.0f);
    return;
  }

  if (num_nodes == 1) {
    /* Parent reduced to a single node: copy it without evaluating. */
    int cell_offset = atomicAdd(lp_counters[counter_slot], 1);
    if (cell_offset < active_capacity) {
      lp_cell_meta_out[cell_idx].x = 1;
      lp_cell_meta_out[cell_idx].y = cell_offset;
      lp_active_out[cell_offset] = uint2(lp_active_in[parent_offset].x, SDF_LP_INVALID_INDEX);
    }
    else {
      /* List does not fit the pool: trace falls back to the full tree. */
      lp_cell_meta_out[cell_idx].x = SDF_LP_FALLBACK_LIST;
      lp_cell_meta_out[cell_idx].y = 0;
    }
    lp_cell_meta_out[cell_idx].z = (first_lvl == 0) ? lp_cell_meta_in[parent_cell_idx].z :
                                                      floatBitsToInt(0.0f);
    return;
  }

  /* ---- Forward pass: evaluate the parent's active list at the cell center,
   * marking binary-op children that are dominated within this cell. ---- */
  float d_stack[SDF_LP_STACK_DEPTH];
  int i_stack[SDF_LP_STACK_DEPTH];
  int stack_idx = 0;

  for (int i = 0; i < num_nodes; i++) {
    uint active_node = lp_active_in[parent_offset + i].x;
    SDFLpNode node = lp_nodes[lp_active_node_index(active_node)];

    float d;
    uint node_state = LP_NODESTATE_ACTIVE;
    if (node.type == SDF_LP_NODETYPE_BINARY) {
      float left_val = d_stack[stack_idx - 2];
      int left_i = i_stack[stack_idx - 2];
      float right_val = d_stack[stack_idx - 1];
      int right_i = i_stack[stack_idx - 1];
      stack_idx -= 2;

      uint op = lp_binary_ops[node.idx_in_type];
      float k = lp_op_blend_factor(op);
      float s = lp_op_sign(op);
      d = s * (min(s * left_val, s * right_val) - lp_kernel(abs(left_val - right_val), k));

      /* PAINT's right child carries color, not geometry: never cull it. */
      if (!lp_op_cullable(op) || abs(left_val - right_val) <= 2.0f * R + k) {
        node_state = LP_NODESTATE_ACTIVE;
      }
      else {
        node_state = LP_NODESTATE_SKIPPED;
        int dominated = (s * left_val < s * right_val) ? right_i : left_i;
        int addr = tmp_offset + 64 * dominated + lane;
        if (addr < tmp_capacity) {
          uint t = lp_tmp[addr];
          lp_tmp_write_state(t, LP_NODESTATE_INACTIVE);
          lp_tmp[addr] = t;
        }
      }
    }
    else {
      SDFLpPrimitive prim = lp_prims[node.idx_in_type];
      d = lp_eval_prim(cell_center, prim);
    }

    int my_addr = tmp_offset + 64 * i + lane;
    if (my_addr < tmp_capacity) {
      lp_tmp[my_addr] = lp_tmp_pack(node_state, false, false, true, 0xffffu);
    }

    d *= lp_active_node_sign(active_node);
    d_stack[stack_idx] = d;
    i_stack[stack_idx] = i;
    stack_idx++;
  }

  float d = d_stack[0];

  /* Far field: the whole cell maps to a constant lower bound. */
  if (abs(d) > 2.0f * R) {
    lp_cell_meta_out[cell_idx].x = 0;
    lp_cell_meta_out[cell_idx].z = floatBitsToInt(sign(d) * (abs(d) - R));
    return;
  }

  /* ---- Backward pass: propagate inactivity through ancestors and compute
   * the compacted parent/sign information for surviving nodes. ---- */
  int cell_num_active = 0;
  bool tmp_overflow = false;
  for (int i = num_nodes - 1; i >= 0; i--) {
    int my_addr = tmp_offset + 64 * i + lane;
    if (my_addr >= tmp_capacity) {
      tmp_overflow = true;
      break;
    }
    uint t = lp_tmp[my_addr];

    if (lp_tmp_state(t) == LP_NODESTATE_INACTIVE) {
      lp_tmp[my_addr] = lp_tmp_pack(LP_NODESTATE_INACTIVE, false, true, lp_tmp_sign(t), 0xffffu);
    }
    else {
      uint parent_idx = lp_active_in[parent_offset + i].y;
      uint t_parent = 0u;
      bool has_inactive_ancestors = false;
      if (parent_idx != SDF_LP_INVALID_INDEX) {
        int paddr = tmp_offset + 64 * int(parent_idx) + lane;
        if (paddr < tmp_capacity) {
          t_parent = lp_tmp[paddr];
          has_inactive_ancestors = lp_tmp_inactive_ancestors(t_parent);
        }
      }
      uint local_state = lp_tmp_state(t);
      bool active_global = (local_state == LP_NODESTATE_ACTIVE) && !has_inactive_ancestors;
      if (active_global) {
        cell_num_active++;
      }

      uint old_active = lp_active_in[parent_offset + i].x;
      int node_sign = (lp_active_node_sign(old_active) > 0.0f) ? 1 : -1;
      uint new_parent_idx;
      if (parent_idx != SDF_LP_INVALID_INDEX &&
          lp_tmp_state(t_parent) == LP_NODESTATE_SKIPPED)
      {
        node_sign *= lp_tmp_sign(t_parent) ? 1 : -1;
        new_parent_idx = lp_tmp_parent(t_parent);
      }
      else {
        new_parent_idx = parent_idx;
      }

      lp_tmp[my_addr] = lp_tmp_pack(
          local_state, active_global, has_inactive_ancestors, node_sign > 0, new_parent_idx);
    }
  }

  /* ---- Emit the compacted active list for this cell. ---- */
  int cell_offset = atomicAdd(lp_counters[counter_slot], cell_num_active);
  bool active_overflow = (cell_offset + cell_num_active > active_capacity);

  int out_idx = cell_num_active - 1;
  for (int i = num_nodes - 1; i >= 0; i--) {
    int my_addr = tmp_offset + 64 * i + lane;
    if (my_addr >= tmp_capacity) {
      tmp_overflow = true;
      break;
    }
    uint t = lp_tmp[my_addr];
    if (lp_tmp_active_global(t) && !active_overflow && !tmp_overflow) {
      if (out_idx >= 0) {
        uint old_active = lp_active_in[parent_offset + i].x;
        uint node_idx = lp_active_node_index(old_active);
        uint node_word = node_idx | (lp_tmp_sign(t) ? 0u : SDF_LP_SIGN_BIT);
        lp_scratch[my_addr] = uint(out_idx);

        uint new_parent_old = lp_tmp_parent(t);
        uint new_parent_idx = SDF_LP_INVALID_INDEX;
        if (new_parent_old != 0xffffu) {
          int paddr = tmp_offset + 64 * int(new_parent_old) + lane;
          if (paddr < tmp_capacity) {
            new_parent_idx = lp_scratch[paddr];
          }
        }
        lp_active_out[cell_offset + out_idx] = uint2(node_word, new_parent_idx);
      }
      out_idx--;
    }
  }

  if (tmp_overflow || active_overflow) {
    /* The list did not fit the dynamic pools: the trace pass evaluates the
     * full tree for this cell (exact, just slower). */
    lp_cell_meta_out[cell_idx].x = SDF_LP_FALLBACK_LIST;
    lp_cell_meta_out[cell_idx].y = 0;
  }
  else {
    lp_cell_meta_out[cell_idx].y = cell_offset;
    lp_cell_meta_out[cell_idx].x = cell_num_active;
  }
  lp_cell_meta_out[cell_idx].z = floatBitsToInt(0.0f);
}
