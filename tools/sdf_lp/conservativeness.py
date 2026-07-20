#!/usr/bin/env python3
"""Definitive conservativeness audit of the interval prune (GLSL replica):
rotated/non-uniformly-scaled spheres+boxes, UNION/SUBTRACT/INTERSECT x
LINEAR/SMOOTH/CHAMFER/ROUND trees, per-cell dense field sampling must lie
inside [lo, hi]; pruned substitution exact; far cells zero-free."""
import numpy as np
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from harness import (UNION, SUBTRACT, INTERSECT, LINEAR, SMOOTH, CHAMFER, ROUND,
                     lp_binary_op_eval, lp_op_dom_k)
from interval_rotation import rot_matrix, box_sdf

rng = np.random.default_rng(12)

class Prim:
    def __init__(self, kind, pos, Minv3, scale3, scalew, size3, sizew):
        self.kind, self.pos, self.Minv3 = kind, pos, Minv3
        self.scale3, self.scalew, self.size3, self.sizew = scale3, scalew, size3, sizew
    def field(self, P):
        lp = (P - self.pos) @ self.Minv3.T / self.scale3
        if self.kind == "sphere":
            return self.scalew * (np.linalg.norm(lp, axis=-1) - self.size3[0] - self.sizew)
        return self.scalew * (box_sdf(lp, self.size3) - self.sizew)
    def interval(self, cell_c, cell_h):
        d4 = cell_c - self.pos
        cl = (self.Minv3 @ d4) / self.scale3
        hl = (np.abs(self.Minv3) @ cell_h) / self.scale3
        lo3, hi3 = cl - hl, cl + hl
        closest = np.clip(np.zeros(3), lo3, hi3)
        farthest = np.maximum(np.abs(lo3), np.abs(hi3))
        if self.kind == "sphere":
            return (self.scalew * (np.linalg.norm(closest) - self.size3[0] - self.sizew),
                    self.scalew * (np.linalg.norm(farthest) - self.size3[0] - self.sizew))
        return (self.scalew * (box_sdf(closest, self.size3) - self.sizew),
                self.scalew * (box_sdf(farthest, self.size3) - self.sizew))

def rand_prim():
    kind = "sphere" if rng.random() < 0.5 else "box"
    M = rot_matrix(rng.normal(size=3), rng.uniform(0, np.pi))
    sc = rng.uniform(0.3, 2.0, 3)
    Minv3 = np.diag(1.0 / sc) @ M.T
    scalew = float(np.min(sc))
    pos = rng.uniform(-2.5, 2.5, 3)
    size3 = (np.full(3, rng.uniform(0.3, 0.8)) if kind == "sphere"
             else rng.uniform(0.2, 0.7, 3))
    return Prim(kind, pos, Minv3, sc, scalew, size3, 0.0)

def build(prims, ops):
    """ops: list of (csg, bt, k) applied left fold."""
    nodes = []
    for i, p in enumerate(prims):
        nodes.append(dict(type="prim", prim=p, left=-1, right=-1))
        if i > 0:
            csg, bt, k = ops[i - 1]
            nodes.append(dict(type="binary", op=csg, bt=bt, k=k, k2=0.0, k3=0.0,
                              flags=0, left=len(nodes) - 2, right=len(nodes) - 1))
    return nodes, len(nodes) - 1

def eval_tree(nodes, root, P):
    vals = []
    for nd in nodes:
        if nd["type"] == "prim":
            vals.append(nd["prim"].field(P))
        else:
            b = vals[nd["right"]]
            a = vals[nd["left"]]
            if nd["op"] == SUBTRACT:
                b = -b
            vals.append(lp_binary_op_eval(nd["op"], nd["bt"], nd["k"], 0.0, 0.0,
                                          False, a, b))
    return vals[root]

def interval_tree(nodes, root, cell_c, cell_h):
    """GLSL interval forward pass replica (incl. SUB sign swap at push)."""
    lo_v = [None] * len(nodes)
    hi_v = [None] * len(nodes)
    for idx, nd in enumerate(nodes):
        if nd["type"] == "prim":
            lo_v[idx], hi_v[idx] = nd["prim"].interval(cell_c, cell_h)
        else:
            a_l, a_h = lo_v[nd["left"]], hi_v[nd["left"]]
            b_l, b_h = lo_v[nd["right"]], hi_v[nd["right"]]
            if nd["op"] == SUBTRACT:
                b_l, b_h = -b_h, -b_l
            lo_v[idx] = lp_binary_op_eval(nd["op"], nd["bt"], nd["k"], 0.0, 0.0,
                                          False, a_l, b_l)
            hi_v[idx] = lp_binary_op_eval(nd["op"], nd["bt"], nd["k"], 0.0, 0.0,
                                          False, a_h, b_h)
    return lo_v[root], hi_v[root]

worst = 0.0
nbad = 0
for trial in range(300):
    n = rng.integers(2, 8)
    prims = [rand_prim() for _ in range(n)]
    ops = [(rng.choice([UNION, SUBTRACT, INTERSECT]), rng.choice([SMOOTH, CHAMFER, ROUND]),
            rng.uniform(0.1, 0.6)) for _ in range(n - 1)]
    nodes, root = build(prims, ops)
    cell_c = rng.uniform(-3, 3, 3)
    cell_h = rng.uniform(0.05, 0.6, 3)
    lo, hi = interval_tree(nodes, root, cell_c, cell_h)
    P = cell_c + rng.uniform(-1, 1, (512, 3)) * cell_h
    f = eval_tree(nodes, root, P)
    viol = max(lo - f.min(), f.max() - hi)
    if viol > 1e-4:
        nbad += 1
        worst = max(worst, viol)
        if nbad <= 6:
            print(f"VIOL {viol:.4f} ops={ops} lo={lo:.3f} hi={hi:.3f} fmin={f.min():.3f} fmax={f.max():.3f}")
print(f"trees with interval violation: {nbad}/300, worst={worst:.3e}")
