#!/usr/bin/env python3
"""Interval-based prune pass (candidate redesign) vs the current center-based
pass. Measures far-cell fraction, culled ops/cell, emitted entries for
(a) 50 ROUND, (b) 800 SMOOTH, (c) 20 ROUND, and validates safety:
- substitution exactness (pruned == full at sample points)
- far cells contain no zero crossing of the full field
Leaf intervals are EXACT (sphere/box; sim prims are translation-only).
Op intervals are EXACT via stack-space monotonicity (interval_check.py)."""
import numpy as np
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scaling import make_scene, eval_nodes_vals
from scene_sim import build_tree
from harness import (UNION, SUBTRACT, INTERSECT, SMOOTH, CHAMFER, ROUND,
                     lp_binary_op_eval, lp_op_dom_k)

def prim_interval(kind, c, s, cell_lo, cell_hi):
    """Exact [lo,hi] of the prim SDF over the cell AABB (translation-only)."""
    closest = np.clip(c, cell_lo, cell_hi)
    farthest = np.where(np.abs(c - cell_lo) > np.abs(c - cell_hi), cell_lo, cell_hi)
    if kind == "sphere":
        return (np.linalg.norm(c - closest, axis=-1) - s,
                np.linalg.norm(c - farthest, axis=-1) - s)
    def box_sdf(p):
        q = np.abs(p - c) - s
        return np.linalg.norm(np.maximum(q, 0.0), axis=-1) +             np.minimum(np.max(q, axis=-1), 0.0)
    return box_sdf(closest), box_sdf(farthest)

def interval_prune(nodes, root, aabb_min, aabb_max, gs, max_cells=40000):
    cs = (aabb_max - aabb_min) / gs
    lin = np.arange(gs)
    C = np.stack(np.meshgrid(lin, lin, lin, indexing="ij"), axis=-1).reshape(-1, 3)
    cell_lo = aabb_min + C * cs
    cell_hi = cell_lo + cs
    centers = cell_lo + 0.5 * cs
    if len(centers) > max_cells:
        sel = np.random.default_rng(1).choice(len(centers), max_cells, replace=False)
        cell_lo, cell_hi, centers = cell_lo[sel], cell_hi[sel], centers[sel]
    lo = [None] * len(nodes)
    hi = [None] * len(nodes)
    for idx, nd in enumerate(nodes):
        if nd["type"] == "prim":
            kind, c, s = nd["prim"]
            l, h = prim_interval(kind, c, s, cell_lo, cell_hi)
            lo[idx], hi[idx] = l, h
        else:
            a_l, a_h = lo[nd["left"]], hi[nd["left"]]
            b_l, b_h = lo[nd["right"]], hi[nd["right"]]
            lo[idx] = lp_binary_op_eval(nd["op"], nd["bt"], nd["k"], nd["k2"],
                                        nd["k3"], bool(nd["flags"]), a_l, b_l)
            hi[idx] = lp_binary_op_eval(nd["op"], nd["bt"], nd["k"], nd["k2"],
                                        nd["k3"], bool(nd["flags"]), a_h, b_h)
    # dominance
    nloc = centers.shape[0]
    skip = np.zeros((nloc, len(nodes)), bool)
    dl = np.zeros((nloc, len(nodes)), bool)
    for idx, nd in enumerate(nodes):
        if nd["type"] != "binary":
            continue
        dom_k = lp_op_dom_k(nd["bt"], nd["k"], nd["k2"], nd["k3"])
        s = 1.0 if nd["op"] == UNION else -1.0
        a_l, a_h = lo[nd["left"]], hi[nd["left"]]
        b_l, b_h = lo[nd["right"]], hi[nd["right"]]
        if s > 0:
            cr = b_l - a_h > dom_k   # left wins -> right dominated
            cl = a_l - b_h > dom_k
        else:
            cr = a_l - b_h > dom_k
            cl = b_l - a_h > dom_k
        skip[:, idx] = cr | cl
        dl[:, idx] = cl
    # far: interval clears zero by more than the cell diagonal (2R margin,
    # sdf_lp_prune_comp.glsl far-field block: without it the stored step
    # z = lo/scene_l collapses to ~0 at the far shell and the march stalls).
    R = np.linalg.norm(cs) * 0.5
    rlo, rhi = lo[root], hi[root]
    scene_l = nodes[root]["lip"]
    far = (rlo > 2.0 * R) | (rhi < -2.0 * R)
    far_val = np.where(rlo > 0, rlo / scene_l, np.where(rhi < 0, rhi / scene_l, 0.0))
    return dict(lo=lo, hi=hi, skip=skip, dl=dl, far=far, far_val=far_val,
                centers=centers, cs=cs, gs=gs, aabb_min=aabb_min, aabb_max=aabb_max)

def center_prune_entries(nodes, root, aabb_min, aabb_max, gs, max_cells=40000):
    """Current center-based pass (for comparison): entries estimate."""
    P = (np.stack(np.meshgrid(*[np.arange(gs)]*3, indexing="ij"), -1).reshape(-1,3) * (aabb_max-aabb_min)/gs + aabb_min + 0.5*(aabb_max-aabb_min)/gs)
    if len(P) > max_cells:
        sel = np.random.default_rng(1).choice(len(P), max_cells, replace=False)
        P = P[sel]
    vals = eval_nodes_vals(nodes, P)
    cs = (aabb_max - aabb_min) / gs
    R = np.linalg.norm(cs) * 0.5
    skip = np.zeros((vals[0].shape[0], len(nodes)), bool)
    for idx, nd in enumerate(nodes):
        if nd["type"] != "binary":
            continue
        dom_k = lp_op_dom_k(nd["bt"], nd["k"], nd["k2"], nd["k3"])
        band = (nodes[nd["left"]]["lip"] + nodes[nd["right"]]["lip"]) * R + dom_k
        skip[:, idx] = np.abs(vals[nd["left"]] - vals[nd["right"]]) > band
    root_v, root_l = vals[root], nodes[root]["lip"]
    far = np.abs(root_v) > 2.0 * root_l * R
    return skip, far

def removed_mask(nodes, skip, dl):
    nn = len(nodes)
    def subm(ri):
        m = np.zeros(nn, bool); st = [ri]
        while st:
            x = st.pop(); m[x] = True
            xn = nodes[x]
            if xn["type"] == "binary": st += [xn["left"], xn["right"]]
        return m
    removed = np.zeros_like(skip)
    for idx, nd in enumerate(nodes):
        if nd["type"] != "binary": continue
        lm = subm(nd["left"]); rm = subm(nd["right"])
        removed |= (skip & dl)[:, idx:idx+1] & lm
        removed |= (skip & ~dl)[:, idx:idx+1] & rm
    removed |= skip
    return removed

def run_case(n, bt, gs=64):
    prims, blends = make_scene(n)
    nodes, root = build_tree(prims, blends, bt)
    bmin = np.full(3, 1e30); bmax = np.full(3, -1e30)
    for kind, c, s in prims:
        bmin = np.minimum(bmin, c - s); bmax = np.maximum(bmax, c + s)
    pad = (bmax - bmin) * 0.001 + 1e-4
    aabb_min, aabb_max = bmin - pad, bmax + pad
    pr = interval_prune(nodes, root, aabb_min, aabb_max, gs)
    rem = removed_mask(nodes, pr["skip"], pr["dl"])
    active = (~rem).sum(axis=1)
    ncells = gs ** 3
    scale = ncells / len(active)
    entries = np.where(pr["far"], 0, active).sum() * scale
    # comparison: center-based
    cskip, cfar = center_prune_entries(nodes, root, aabb_min, aabb_max, gs)
    cdl = np.zeros_like(cskip)
    crem = removed_mask(nodes, cskip, cdl)
    cactive = (~crem).sum(axis=1)
    centries = np.where(cfar, 0, cactive).sum() * scale
    cap = min(max(8 * ncells, 1 << 22), 1 << 26)
    print(f"n={n} bt={bt}: INTERVAL far={100*pr['far'].mean():5.1f}% "
          f"active/cell={active[pr['far']==0].mean() if (~pr['far']).any() else 0:6.1f} "
          f"entries={int(entries)} ({'FITS' if entries<=cap else 'OVER x%.1f'%(entries/cap)}) | "
          f"CENTER far={100*cfar.mean():5.1f}% entries={int(centries)} "
          f"({'FITS' if centries<=cap else 'OVER x%.1f'%(centries/cap)})")
    # safety: far cells have no zero crossing; substitution exact
    rng = np.random.default_rng(0)
    offs = rng.uniform(-0.45, 0.45, (24, 3))
    zviol = 0; suberr = 0.0
    fidx = np.flatnonzero(pr["far"])
    for ci in fidx[::max(1, len(fidx)//400)]:
        P = pr["centers"][ci][None,:] + offs * pr["cs"][None,:]
        f = eval_nodes_vals(nodes, P)[root]
        scene_l = nodes[root]["lip"]
        if (f.min() < 0 < f.max()) or (np.sign(f[0]) != np.sign(pr["far_val"][ci])) or \
                np.any(np.abs(f) < scene_l * abs(pr["far_val"][ci]) - 1e-6):
            zviol += 1
    # substitution: sample non-far cells, eval pruned vs full
    nidx = np.flatnonzero(~pr["far"])
    lmask_cache = {}
    for ci in nidx[::max(1, len(nidx)//300)]:
        P = pr["centers"][ci][None,:] + offs * pr["cs"][None,:]
        full = eval_nodes_vals(nodes, P)[root]
        st = []
        for idx in range(len(nodes)):
            if rem[ci, idx]:
                continue
            nd = nodes[idx]
            if nd["type"] == "prim":
                kind, c, s = nd["prim"]
                from scene_sim import prim_eval as pe
                st.append(pe(kind, c, s, P))
            else:
                b = st.pop(); a = st.pop()
                st.append(lp_binary_op_eval(nd["op"], nd["bt"], nd["k"], nd["k2"],
                                            nd["k3"], bool(nd["flags"]), a, b))
        suberr = max(suberr, float(np.max(np.abs(st[0] - full))))
    print(f"   safety: far zero-cross violations={zviol} substitution max err={suberr:.2e}")

run_case(50, ROUND)
run_case(800, SMOOTH)
run_case(20, ROUND)
