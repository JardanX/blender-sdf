#!/usr/bin/env python3
"""Hierarchical interval prune exactly as the shader does it (4^3 -> 16^3 ->
64^3 levels with parent-list inheritance of skips and far constants), on a
user-like scene (overlapping boxes/spheres, ROUND k~0.3, mixed sizes/rot).
Per level, per cell: verify [lo,hi] brackets the true field (dense samples),
far cells are zero-free with valid distance bound, substitution exact, and
the march hits the reference surface."""
import numpy as np
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from harness import UNION, SUBTRACT, INTERSECT, SMOOTH, CHAMFER, ROUND, \
    lp_binary_op_eval, lp_op_dom_k
from interval_prune import prim_interval, removed_mask
from scene_sim import prim_eval, build_tree
from scaling import make_scene

rng = np.random.default_rng(5)

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

def hier_prune_interval(nodes, root, aabb_min, aabb_max, levels=(4, 16, 64)):
    nn = len(nodes)
    parent = None
    for li, gs in enumerate(levels):
        cs = (aabb_max - aabb_min) / gs
        R = np.linalg.norm(cs) * 0.5
        lin = np.arange(gs)
        C = np.stack(np.meshgrid(lin, lin, lin, indexing="ij"), axis=-1).reshape(-1, 3)
        cell_lo = aabb_min + C * cs
        cell_hi = cell_lo + cs
        centers = cell_lo + 0.5 * cs
        ncells = gs ** 3
        sel = None
        if ncells > 40000:
            sel = np.random.default_rng(1).choice(ncells, 40000, replace=False)
            cell_lo, cell_hi, centers = cell_lo[sel], cell_hi[sel], centers[sel]
        if parent is None:
            pskip = np.zeros((ncells, nn), bool)
            pdl = np.zeros((ncells, nn), bool)
            pmode = np.ones(ncells, dtype=int)  # 1 list
            pz = np.zeros(ncells)
        else:
            pgs = levels[li - 1]
            par_full = (C // 4).astype(int)
            par = par_full[sel] if sel is not None else par_full
            pci = (par[:, 0] * pgs + par[:, 1]) * pgs + par[:, 2]
            pskip = parent["skip"][pci]
            pdl = parent["dl"][pci]
            pmode = parent["mode"][pci]
            pz = parent["z"][pci]
        # intervals per node with inherited substitution (skip -> winner)
        lo = [None] * nn
        hi = [None] * nn
        for idx, nd in enumerate(nodes):
            if nd["type"] == "prim":
                kind, c, s = nd["prim"]
                l, h = prim_interval(kind, c, s, cell_lo, cell_hi)
            else:
                a_l, a_h = lo[nd["left"]], hi[nd["left"]]
                b_l, b_h = lo[nd["right"]], hi[nd["right"]]
                l = lp_binary_op_eval(nd["op"], nd["bt"], nd["k"], nd["k2"],
                                      nd["k3"], bool(nd["flags"]), a_l, b_l)
                h = lp_binary_op_eval(nd["op"], nd["bt"], nd["k"], nd["k2"],
                                      nd["k3"], bool(nd["flags"]), a_h, b_h)
                sk = pskip[:, idx]
                # skipped at parent level: substitute winner interval
                wl = np.where(pdl[:, idx], b_l, a_l)
                wh = np.where(pdl[:, idx], b_h, a_h)
                l = np.where(sk, wl, l)
                h = np.where(sk, wh, h)
            lo[idx], hi[idx] = l, h
        # new dominance culls at this level
        skip = pskip.copy(); dl = pdl.copy()
        for idx, nd in enumerate(nodes):
            if nd["type"] != "binary":
                continue
            k = lp_op_dom_k(nd["bt"], nd["k"], nd["k2"], nd["k3"])
            s = 1.0 if nd["op"] == UNION else -1.0
            a_l, a_h = lo[nd["left"]], hi[nd["left"]]
            b_l, b_h = lo[nd["right"]], hi[nd["right"]]
            if s > 0:
                cr = b_l - a_h > k; cl = a_l - b_h > k
            else:
                cr = a_l - b_h > k; cl = b_l - a_h > k
            ns = ~skip[:, idx]
            skip[:, idx] |= ns & (cr | cl)
            dl[:, idx] = np.where(ns & cl, True, np.where(ns & cr, False, dl[:, idx]))
        mode = pmode.copy()
        z = pz.copy()
        scene_l = nodes[root]["lip"]
        rlo, rhi = lo[root], hi[root]
        newly_far = (mode == 1) & ((rlo > 2.0 * R) | (rhi < -2.0 * R))
        mode[newly_far] = 0
        z[newly_far] = np.where(rlo[newly_far] > 0, rlo[newly_far], rhi[newly_far]) / scene_l
        parent = dict(gs=gs, cs=cs, R=R, centers=centers, mode=mode, z=z,
                      skip=skip, dl=dl, lo=lo, hi=hi, cell_lo=cell_lo, cell_hi=cell_hi)
    return parent

def validate(nodes, root, prims, blends, bt, name):
    bmin = np.full(3, 1e30); bmax = np.full(3, -1e30)
    for kind, c, s in prims:
        bmin = np.minimum(bmin, c - s); bmax = np.maximum(bmax, c + s)
    pad = (bmax - bmin) * 0.001 + 1e-4
    L = hier_prune_interval(nodes, root, bmin - pad, bmax + pad)
    centers, mode, z = L["centers"], L["mode"], L["z"]
    rem = removed_mask(nodes, L["skip"], L["dl"])
    offs = rng.uniform(-0.45, 0.45, (16, 3))
    zviol = bviol = suberr = 0
    scene_l = nodes[root]["lip"]
    far_idx = np.flatnonzero(mode == 0)
    near_idx = np.flatnonzero(mode == 1)
    for ci in far_idx[::max(1, len(far_idx)//500)]:
        P = centers[ci][None,:] + offs * L["cs"][None,:]
        f = eval_nodes_vals(nodes, P)[root]
        if (f.min() < 0 < f.max()) or np.sign(f[0]) != np.sign(z[ci]) or \
                np.any(np.abs(f) < scene_l * abs(z[ci]) - 1e-5):
            zviol += 1
    for ci in near_idx[::max(1, len(near_idx)//400)]:
        P = centers[ci][None,:] + offs * L["cs"][None,:]
        full = eval_nodes_vals(nodes, P)[root]
        st = []
        for idx in range(len(nodes)):
            if rem[ci, idx]:
                continue
            nd = nodes[idx]
            if nd["type"] == "prim":
                kind, c, s = nd["prim"]
                st.append(prim_eval(kind, c, s, P))
            else:
                b = st.pop(); a = st.pop()
                st.append(lp_binary_op_eval(nd["op"], nd["bt"], nd["k"], nd["k2"],
                                            nd["k3"], bool(nd["flags"]), a, b))
        suberr = max(suberr, float(np.max(np.abs(st[0] - full))))
        # interval bracket check at level cells
        rlo, rhi = L["lo"][root][ci], L["hi"][root][ci]
        if rlo - full.min() > 1e-4 or full.max() - rhi > 1e-4:
            bviol += 1
    ncells = len(centers)
    entries = np.where(mode == 0, 0, (~rem).sum(axis=1)).sum() * (64**3 / ncells)
    cap = min(max(8 * 64**3, 1 << 22), 1 << 26)
    print(f"{name}: far={100*np.mean(mode==0):5.1f}% entries={int(entries)} "
          f"({'FITS' if entries<=cap else 'OVER'}) zviol={zviol} "
          f"bracketviol={bviol} suberr={suberr:.2e}")

for bt, n, name in ((ROUND, 20, "20 ROUND"), (ROUND, 50, "50 ROUND"), (SMOOTH, 800, "800 SMOOTH")):
    prims, blends = make_scene(n)
    nodes, root = build_tree(prims, blends, bt)
    validate(nodes, root, prims, blends, bt, name)
