#!/usr/bin/env python3
"""Scale object count with SMOOTH vs ROUND and model the LP GPU resources:
per-level active-pool counter (capacity), tmp pool per workgroup, fallback
fractions — to find the saturating resource (sdf_lp_engine.cc
lp_ensure_grid_buffers, sdf_lp_prune_comp.glsl)."""
import numpy as np
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scene_sim import prim_eval, build_tree
from harness import UNION, SUBTRACT, INTERSECT, LINEAR, SMOOTH, CHAMFER, ROUND, \
    lp_binary_op_eval, lp_op_dom_k

rng = np.random.default_rng(2)

def eval_nodes_vals(nodes, P):
    vals = []
    for nd in nodes:
        if nd["type"] == "prim":
            kind, c, s = nd["prim"]
            vals.append(prim_eval(kind, c, s, P))
        else:
            vals.append(lp_binary_op_eval(nd["op"], nd["bt"], nd["k"], nd["k2"],
                                          nd["k3"], bool(nd["flags"]),
                                          vals[nd["left"]], vals[nd["right"]]))
    return vals

def make_scene(n, spread=3.0, seed=2):
    rng = np.random.default_rng(seed)
    prims = []
    for i in range(n):
        c = rng.uniform(-spread, spread, 3)
        if i % 2 == 0:
            prims.append(("sphere", c, rng.uniform(0.3, 0.7)))
        else:
            prims.append(("box", c, rng.uniform(0.2, 0.5, 3)))
    blends = rng.uniform(0.15, 0.4, n)
    return prims, blends

def resource_report(n, bt, levels=(2, 4, 6), grid_level=6):
    prims, blends = make_scene(n)
    nodes, root = build_tree(prims, blends, bt)
    bmin = np.full(3, 1e30); bmax = np.full(3, -1e30)
    for kind, c, s in prims:
        bmin = np.minimum(bmin, c - s); bmax = np.maximum(bmax, c + s)
    pad = (bmax - bmin) * 0.001 + 1e-4
    aabb_min, aabb_max = bmin - pad, bmax + pad
    nn = len(nodes)
    active_cap = max(8 * (1 << (3 * grid_level)), 1 << 22)
    active_cap = min(active_cap, 1 << 26)
    cells_total = sum((1 << l) ** 3 for l in range(2, grid_level + 1, 2))
    tmp_cap = min(cells_total * nn, 1 << 26)
    tmp_cap = max(tmp_cap, 1 << 16)
    print(f"--- n={n} bt={bt} nodes={nn} root_l={nodes[root]['lip']:.2f} "
          f"active_cap={active_cap} tmp_cap={tmp_cap}")
    parent_lists = None  # per-cell skip sets
    total_entries = 0
    for lvl in range(2, grid_level + 1, 2):
        gs = 1 << lvl
        cs = (aabb_max - aabb_min) / gs
        R = np.linalg.norm(cs) * 0.5
        lin = np.arange(gs)
        C = np.stack(np.meshgrid(lin, lin, lin, indexing="ij"), axis=-1)
        centers = (aabb_min + cs * (C + 0.5)).reshape(-1, 3)
        ncells = gs ** 3
        if parent_lists is None:
            pskip = np.zeros((ncells, nn), bool)
            pdl = np.zeros((ncells, nn), bool)
            pmode = np.ones(ncells)  # 1 list, 0 far
        else:
            pgs = 1 << (lvl - 2)
            par = (C // 4).reshape(-1, 3)
            pci = (par[:, 0] * pgs + par[:, 1]) * pgs + par[:, 2]
            prev = parent_lists
            pskip = prev["skip"][pci]; pdl = prev["dl"][pci]; pmode = prev["mode"][pci]
        # eval all nodes at centers with inherited substitution
        vals = [None] * nn
        for idx, nd in enumerate(nodes):
            if nd["type"] == "prim":
                kind, c, s = nd["prim"]
                vals[idx] = prim_eval(kind, c, s, centers)
            else:
                lv, rv = vals[nd["left"]], vals[nd["right"]]
                opv = lp_binary_op_eval(nd["op"], nd["bt"], nd["k"], nd["k2"],
                                        nd["k3"], bool(nd["flags"]), lv, rv)
                vals[idx] = np.where(pskip[:, idx],
                                     np.where(pdl[:, idx], rv, lv), opv)
        skip = pskip.copy(); dl = pdl.copy()
        for idx, nd in enumerate(nodes):
            if nd["type"] != "binary":
                continue
            k = lp_op_dom_k(nd["bt"], nd["k"], nd["k2"], nd["k3"])
            band = (nodes[nd["left"]]["lip"] + nodes[nd["right"]]["lip"]) * R + k
            ns = (~skip[:, idx]) & (np.abs(vals[nd["left"]] - vals[nd["right"]]) > band)
            s = 1.0 if nd["op"] == UNION else -1.0
            dl[:, idx] = np.where(ns, (s * vals[nd["left"]] > s * vals[nd["right"]]), dl[:, idx])
            skip[:, idx] |= ns
        root_v, root_l = vals[root], nodes[root]["lip"]
        mode = pmode.copy()
        newly_far = (mode == 1) & (np.abs(root_v) > 2.0 * root_l * R)
        mode[newly_far] = 0
        # accounting: per-cell emitted entries = active nodes (non-far cells)
        nbin = np.array([1.0 if nd["type"] == "binary" else 0.0 for nd in nodes])
        # count active: nodes not skipped & not under skipped parents.
        # approximation like the shader: node emitted if ACTIVE and no inactive
        # ancestors; compute exactly via parent chain.
        parent = np.full(nn, -1)
        for idx, nd in enumerate(nodes):
            if nd["type"] == "binary":
                parent[nd["left"]] = idx; parent[nd["right"]] = idx
        # a node is dropped if itself dominated-marked: i.e. any ancestor op
        # skipped with this node on the dominated side. Simulate per cell for a
        # sample of cells.
        sample = rng.choice(ncells, min(2000, ncells), replace=False)
        active_counts = np.zeros(len(sample))
        for si, ci in enumerate(sample):
            dead = np.zeros(nn, bool)
            for idx in range(nn - 1, -1, -1):
                nd = nodes[idx]
                if nd["type"] != "binary":
                    continue
                if skip[ci, idx]:
                    # dominated subtree dies
                    dom = nd["left"] if dl[ci, idx] else nd["right"]
                    # mark subtree of dom
                    stackm = [dom]
                    while stackm:
                        x = stackm.pop()
                        dead[x] = True
                        xn = nodes[x]
                        if xn["type"] == "binary":
                            stackm += [xn["left"], xn["right"]]
            active_counts[si] = np.count_nonzero(~dead)
        near_frac = float((mode == 1).mean())
        est_entries = float(active_counts.mean() * (mode == 1).sum())
        total_entries += est_entries
        print(f"  lvl {lvl} gs={gs:3d}: near/list cells={100*near_frac:6.2f}% "
              f"mean active/cell={active_counts.mean():7.1f} est entries={int(est_entries)}")
        parent_lists = dict(mode=mode, skip=skip, dl=dl)
    fits = "FITS" if total_entries <= active_cap else f"OVERFLOW x{total_entries/active_cap:.1f}"
    print(f"  TOTAL est entries={int(total_entries)} vs cap={active_cap}: {fits}")

for bt, name in ((SMOOTH, "SMOOTH"), (ROUND, "ROUND")):
    for n in ((800,) if bt == SMOOTH else (20, 50, 100)):
        resource_report(n, bt)
