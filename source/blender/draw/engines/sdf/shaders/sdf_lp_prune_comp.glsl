/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Lipschitz pruning pass: one thread per grid cell.
 *
 * Hierarchical construction: level `grid_size` reads the active node lists of
 * the parent level (grid_size/4) and produces, for each cell, a compacted
 * list of the nodes that can influence the SDF inside the cell. The forward
 * pass propagates a distance INTERVAL [lo, hi] per node over the cell:
 * leaves use exact primitive bounds / value(center) +- lip*R
 * (lp_prim_interval), ops use the exact corner evaluation
 * [eval(lo), eval(hi)] (lp_binary_op_eval is componentwise monotone in
 * stack space — verified numerically — a hard requirement, see its doc).
 * A binary op child is culled when its whole interval clears the sibling's
 * by more than the blend radius k: the dominated subtree cannot win the
 * min()/max() anywhere inside the cell and the op reduces to the exact
 * winner there. Cells whose root interval clears zero by more than the cell
 * diagonal (lo > 2R, resp. hi < -2R) are "far field" and only store a
 * constant lower bound on the distance to the surface,
 * lo/scene_l (resp. hi/scene_l), where scene_l is the full tree's Lipschitz
 * constant — the field-to-distance conversion the ROUND fillet needs (its
 * field can exceed the true distance by up to sqrt(n) along fillet crests
 * of parallel surfaces, so un-divided |d| steps overshoot thin features;
 * see sdf_lp_march_comp.glsl). The 2R margin keeps the stored step bounded
 * below by 2R/scene_l so the march can never asymptotically stall in the
 * far shell around a surface. The interval form keeps dominance and
 * far-cell culling effective for ROUND / many-object scenes, where the old
 * center value +- (Ll+Lr)*R form degraded with the sqrt-composed per-node
 * Lipschitz constants.
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

  if (num_nodes == SDF_LP_FALLBACK_LIST) {
    /* Parent cell overflowed the dynamic pools: propagate the full-tree
     * fallback so this cell (and, transitively, all finer descendants) is
     * traced against the complete node list instead of a corrupt one. */
    lp_cell_meta_out[cell_idx].x = SDF_LP_FALLBACK_LIST;
    lp_cell_meta_out[cell_idx].y = 0;
    lp_cell_meta_out[cell_idx].z = floatBitsToInt(0.0f);
    return;
  }

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
      lp_active_out[cell_offset] = lp_active_in[parent_offset];
      lp_active_parents_out[cell_offset] = SDF_LP_INVALID_INDEX;
    }
    else {
      /* List does not fit the pool: trace falls back to the full tree. */
      lp_cell_meta_out[cell_idx].x = SDF_LP_FALLBACK_LIST;
      lp_cell_meta_out[cell_idx].y = 0;
      atomicAdd(lp_counters[SDF_LP_STAT_ACTIVE_OVERFLOW], 1);
    }
    lp_cell_meta_out[cell_idx].z = (first_lvl == 0) ? lp_cell_meta_in[parent_cell_idx].z :
                                                      floatBitsToInt(0.0f);
    return;
  }

  /* ---- Forward pass: propagate a distance INTERVAL [lo, hi] of each node
   * over this cell, marking binary-op children that are dominated on the
   * whole cell. lp_binary_op_eval is componentwise monotone in stack space
   * (verified numerically), so a node's exact interval is simply
   * [eval(lo_corner), eval(hi_corner)]; leaves get exact/primitive-specific
   * intervals (lp_prim_interval). This replaces the old
   * value(center) +- (Ll+Lr)*R form, whose sqrt-composed Lipschitz constants
   * (~sqrt(n) for n nested ROUND ops) inflated every dominance band until
   * culling stopped firing for ROUND / many-object scenes. ---- */
  float lo_stack[SDF_LP_STACK_DEPTH];
  float hi_stack[SDF_LP_STACK_DEPTH];
  int i_stack[SDF_LP_STACK_DEPTH];
  int stack_idx = 0;

  for (int i = 0; i < num_nodes; i++) {
    uint active_node = lp_active_in[parent_offset + i];
    SDFLpNode node = lp_nodes[lp_active_node_index(active_node)];

    float lo, hi;
    uint node_state = LP_NODESTATE_ACTIVE;
    if (node.type == SDF_LP_NODETYPE_BINARY) {
      float left_lo = lo_stack[stack_idx - 2];
      float left_hi = hi_stack[stack_idx - 2];
      int left_i = i_stack[stack_idx - 2];
      float right_lo = lo_stack[stack_idx - 1];
      float right_hi = hi_stack[stack_idx - 1];
      int right_i = i_stack[stack_idx - 1];
      stack_idx -= 2;

      uint4 opw = lp_binary_ops[node.idx_in_type];
      uint op = opw.x;
      float k = lp_op_dom_k(opw);
      float s = lp_op_sign(op);
      /* Monotone in stack space: exact op interval from the two corners. */
      lo = lp_binary_op_eval(opw, left_lo, right_lo);
      hi = lp_binary_op_eval(opw, left_hi, right_hi);

      /* PAINT's right child carries color, not geometry: never cull it.
       * Interval dominance: outside |a-b| > k every op reduces to the exact
       * winner (lp_op_iround's max(q.x, ad) clamp; lp_op_dom_k for the
       * k2/k3 variants), so when a child's whole interval clears the
       * sibling's by more than k, the dominated subtree cannot win the
       * min()/max() anywhere inside the cell and is culled. The one
       * exception is lp_op_intersection_round (inward SHELL start edge):
       * only sign-exact outside the band, but its value always
       * over-estimates the winner, so the substituted field stays
       * conservative and its zero set always lies inside the band (see
       * lp_op_cullable). */
      if (lp_op_cullable(op)) {
        /* (s>0) min-form: right dominated when left_hi < right_lo - k.
         * (s<0) max-form: right dominated when left_lo > right_hi + k. */
        bool cull_right = (s > 0.0f) ? (left_hi < right_lo - k) :
                                       (left_lo > right_hi + k);
        bool cull_left = (s > 0.0f) ? (right_hi < left_lo - k) :
                                      (right_lo > left_hi + k);
        if (cull_right || cull_left) {
          node_state = LP_NODESTATE_SKIPPED;
          int dominated = cull_right ? right_i : left_i;
          int addr = tmp_offset + 64 * dominated + lane;
          if (addr < tmp_capacity) {
            uint t = lp_tmp[addr];
            lp_tmp_write_state(t, LP_NODESTATE_INACTIVE);
            lp_tmp[addr] = t;
          }
        }
      }
    }
    else if (node.type == SDF_LP_NODETYPE_OFFSET) {
      /* Unary offset (desugared SHELL/PUSH/AVOID): shift both bounds. Never
       * culled (it shifts both sides of the parent op equally). */
      lo = lo_stack[stack_idx - 1] + lp_offset_node_value(node);
      hi = hi_stack[stack_idx - 1] + lp_offset_node_value(node);
      stack_idx -= 1;
    }
    else {
      SDFLpPrimitive prim = lp_prims[node.idx_in_type];
      float2 iv = lp_prim_interval(cell_center, cell_size * 0.5f, R, prim, node.lipschitz);
      lo = iv.x;
      hi = iv.y;
    }

    int my_addr = tmp_offset + 64 * i + lane;
    if (my_addr < tmp_capacity) {
      lp_tmp[my_addr] = lp_tmp_pack(node_state, false, false, true, 0xffffu);
    }

    /* A negated subtree flips the field: negate and swap the bounds. */
    if (lp_active_node_sign(active_node) < 0.0f) {
      float tmp = lo;
      lo = -hi;
      hi = -tmp;
    }
    lo_stack[stack_idx] = lo;
    hi_stack[stack_idx] = hi;
    i_stack[stack_idx] = i;
    stack_idx++;
  }

  float lo = lo_stack[0];
  float hi = hi_stack[0];
  /* Lipschitz constant of the FULL tree: converts a field magnitude into a
   * distance bound. Needed because the ROUND fillet field is NOT a
   * conservative distance under-estimate: along a fillet crest between two
   * (near-)parallel surfaces the field grows at up to sqrt(2) per op (up to
   * sqrt(n) for n nested ROUND ops, measured numerically), so |f| can exceed
   * the true distance to the zero set by that factor. The march pass must
   * therefore step |f|/scene_l (and far cells store a pre-divided constant),
   * or a step can clear a thin feature between two fillet crossings — the
   * classic engine catches those with its SOR radius-sum backtrack
   * (sdf_trace_comp.glsl:805), which this pipeline does not have. */
  float scene_l = lp_nodes[total_num_nodes - 1].lipschitz;

  /* Far field: the whole cell maps to a constant lower bound on the DISTANCE
   * to the surface. The interval already proves the cell has no zero
   * crossing (lo > 0 or hi < 0) and |f| >= lo (resp. |hi|) inside it;
   * dividing by scene_l turns that field bound into a distance bound (see
   * above). The 2R margin is NOT optional: the march consumes the constant
   * as a step length, and far cells form a closed shell around every
   * surface — with a bare lo > 0 test the shell's inner cells have
   * lo ~ 0, the step lo/scene_l collapses, and the ray crawls
   * asymptotically without ever reaching a near (list) cell, burning
   * max_steps and missing (blocky, cell-aligned holes). lo > 2R guarantees
   * z >= 2R/scene_l, so a ray crosses any far cell in at most scene_l
   * steps. Still ~root_l times tighter than the old
   * abs(d_center) > 2*root_l*R form it replaces, since the interval lo/hi
   * never uses the compounded per-node Lipschitz constants. */
  if (lo > 2.0f * R || hi < -2.0f * R) {
    lp_cell_meta_out[cell_idx].x = 0;
    lp_cell_meta_out[cell_idx].z = floatBitsToInt(((lo > 0.0f) ? lo : hi) / scene_l);
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
      uint parent_idx = lp_active_parents_in[parent_offset + i];
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

      uint old_active = lp_active_in[parent_offset + i];
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
        uint old_active = lp_active_in[parent_offset + i];
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
        lp_active_out[cell_offset + out_idx] = node_word;
        lp_active_parents_out[cell_offset + out_idx] = new_parent_idx;
      }
      out_idx--;
    }
  }

  if (tmp_overflow || active_overflow) {
    /* The list did not fit the dynamic pools: the trace pass evaluates the
     * full tree for this cell (exact, just slower). */
    lp_cell_meta_out[cell_idx].x = SDF_LP_FALLBACK_LIST;
    lp_cell_meta_out[cell_idx].y = 0;
    if (active_overflow) {
      atomicAdd(lp_counters[SDF_LP_STAT_ACTIVE_OVERFLOW], 1);
    }
    if (tmp_overflow) {
      atomicAdd(lp_counters[SDF_LP_STAT_TMP_OVERFLOW], 1);
    }
  }
  else {
    lp_cell_meta_out[cell_idx].y = cell_offset;
    lp_cell_meta_out[cell_idx].x = cell_num_active;
  }
  lp_cell_meta_out[cell_idx].z = floatBitsToInt(0.0f);
}
