#!/usr/bin/env python3
"""End-to-end validation of the pool-overflow fallback fix.

Models the LP pipeline at the final grid level INCLUDING the active-list
pool: per-cell emitted lists are appended to a flat pool (capacity like the
engine); cells that do not fit become SDF_LP_FALLBACK_LIST.

OLD (bug): fallback cells evaluated lp_list_eval(p, total_num_nodes, 0)
against lp_active_in == the POOL buffer -> a garbage tree of concatenated
cell lists (objects disappear).
NEW (fix): fallback cells evaluate the init list (lp_active_init).

Checks the rendered field of both against the true full-tree field at sample
points inside affected cells, for 50 ROUND and 800 SMOOTH objects.
"""
import numpy as np
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scaling import make_scene, eval_nodes_vals
from scene_sim import build_tree
from harness import UNION, SUBTRACT, INTERSECT, SMOOTH, ROUND, lp_binary_op_eval, lp_op_dom_k

rng = np.random.default_rng(4)

def run_case(n, bt, gs=32, cap_override=None):
    prims, blends = make_scene(n)
    nodes, root = build_tree(prims, blends, bt)
    nn = len(nodes)
    bmin = np.full(3, 1e30); bmax = np.full(3, -1e30)
    for kind, c, s in prims:
        bmin = np.minimum(bmin, c - s); bmax = np.maximum(bmax, c + s)
    pad = (bmax - bmin) * 0.001 + 1e-4
    aabb_min, aabb_max = bmin - pad, bmax + pad
    cs = (aabb_max - aabb_min) / gs
    R = np.linalg.norm(cs) * 0.5
    lin = np.arange(gs)
    C = np.stack(np.meshgrid(lin, lin, lin, indexing="ij"), axis=-1)
    centers = (aabb_min + cs * (C + 0.5)).reshape(-1, 3)
    ncells = gs ** 3
    vals = eval_nodes_vals(nodes, centers)
    # dominance per node (single level from full tree)
    skip = np.zeros((ncells, nn), bool)
    dl = np.zeros((ncells, nn), bool)
    for idx, nd in enumerate(nodes):
        if nd["type"] != "binary":
            continue
        k = lp_op_dom_k(nd["bt"], nd["k"], nd["k2"], nd["k3"])
        band = (nodes[nd["left"]]["lip"] + nodes[nd["right"]]["lip"]) * R + k
        skip[:, idx] = np.abs(vals[nd["left"]] - vals[nd["right"]]) > band
        s = 1.0 if nd["op"] == UNION else -1.0
        dl[:, idx] = skip[:, idx] & (s * vals[nd["left"]] > s * vals[nd["right"]])
    root_v, root_l = vals[root], nodes[root]["lip"]
    far = np.abs(root_v) > 2.0 * root_l * R
    scene_l = root_l
    far_val = np.sign(root_v) * (np.abs(root_v) - root_l * R) / scene_l

    # per-cell surviving node list (shader semantics: skipped op and its
    # dominated subtree are removed; the surviving subtree takes its place).
    dommask = np.zeros((nn, nn), dtype=bool)  # [op_idx] -> removed nodes
    for idx, nd in enumerate(nodes):
        if nd["type"] != "binary":
            continue
        for side in (0, 1):
            pass
    def subtree_mask(root_idx):
        m = np.zeros(nn, bool)
        st = [root_idx]
        while st:
            x = st.pop()
            m[x] = True
            xn = nodes[x]
            if xn["type"] == "binary":
                st += [xn["left"], xn["right"]]
        return m
    lmask = np.zeros((nn, nn), dtype=bool)
    rmask = np.zeros((nn, nn), dtype=bool)
    for idx, nd in enumerate(nodes):
        if nd["type"] != "binary":
            continue
        lmask[idx] = subtree_mask(nd["left"])
        rmask[idx] = subtree_mask(nd["right"])
    # removed[ci, x] = any op skipped with x in dominated subtree, or x itself skipped
    dom_l = skip & dl          # cells where left subtree is dominated
    dom_r = skip & ~dl
    removed = (dom_l @ lmask) | (dom_r @ rmask) | skip
    def cell_list(ci):
        return np.flatnonzero(~removed[ci])

    # emit into the pool with the engine capacity
    active_cap = min(max(8 * gs ** 3, 1 << 22), 1 << 26)
    if cap_override is not None:
        active_cap = cap_override
    pool = []
    cell_mode = np.zeros(ncells, dtype=int)  # 0 far, 1 list, 2 fallback
    cell_off = np.zeros(ncells, dtype=int)
    for ci in range(ncells):
        if far[ci]:
            cell_mode[ci] = 0
            continue
        lst = cell_list(ci)
        if len(pool) + len(lst) > active_cap:
            cell_mode[ci] = 2
        else:
            cell_mode[ci] = 1
            cell_off[ci] = len(pool)
            pool.extend(lst.tolist())
    pool = np.array(pool, dtype=int)
    n_fallback = int((cell_mode == 2).sum())
    print(f"n={n} bt={bt} nodes={nn}: pool entries={len(pool)} cap={active_cap} "
          f"fallback cells={n_fallback} ({100*n_fallback/ncells:.1f}%) "
          f"far={100*far.mean():.1f}%")

    def eval_with_list(lst, P):
        # stack machine over a node-index list (sign +1 scene)
        st = []
        for idx in lst:
            nd = nodes[idx]
            if nd["type"] == "prim":
                kind, c, s = nd["prim"]
                st.append(prim_eval_(kind, c, s, P))
            else:
                b = st.pop(); a = st.pop()
                st.append(lp_binary_op_eval(nd["op"], nd["bt"], nd["k"], nd["k2"],
                                            nd["k3"], bool(nd["flags"]), a, b))
        return st[0]

    from scene_sim import prim_eval as prim_eval_

    # sample cells: some fallback (or list if none), compare rendered paths
    cand = np.flatnonzero(cell_mode == 2) if n_fallback else np.flatnonzero(cell_mode == 1)
    test = cand[:8] if len(cand) >= 8 else cand
    offs = np.array([[0,0,0],[0.3,0.2,-0.4],[-0.4,0.1,0.35],[0.2,-0.35,0.25]])
    worst_old = worst_new = 0.0
    for ci in test:
        c = centers[ci]
        P = c[None,:] + offs * cs[None,:]
        full = eval_nodes_vals(nodes, P)[root]
        if cell_mode[ci] == 2:
            old = eval_with_list(pool[:nn], P)      # OLD: garbage pool read
            new = eval_with_list(np.arange(nn), P)  # NEW: init list
        else:
            old = new = eval_with_list(cell_list(ci), P)
        worst_old = max(worst_old, float(np.max(np.abs(old - full))))
        worst_new = max(worst_new, float(np.max(np.abs(new - full))))
    print(f"   rendered-field error vs full tree: OLD(pool read)={worst_old:.4f} "
          f"NEW(init read)={worst_new:.2e}")
    return n_fallback, worst_old, worst_new

print("== 50 ROUND (real cap) =="); run_case(50, ROUND)
print("== 800 SMOOTH (real cap) =="); run_case(800, SMOOTH)
print("== 50 ROUND (cap forced to overflow) =="); run_case(50, ROUND, cap_override=100_000)
print("== 800 SMOOTH (cap forced to overflow) =="); run_case(800, SMOOTH, cap_override=100_000)
