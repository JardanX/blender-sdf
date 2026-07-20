#!/usr/bin/env python3
"""Stress lp_prim_interval's exact sphere/box path under arbitrary rotation
and non-uniform scale: for random cell AABBs and random prim transforms,
the [lo,hi] interval must bracket the true field at dense sample points."""
import numpy as np

rng = np.random.default_rng(6)

def rot_matrix(ax, ang):
    ax = ax / np.linalg.norm(ax)
    x, y, z = ax
    c, s = np.cos(ang), np.sin(ang)
    C = 1 - c
    return np.array([
        [x*x*C+c,   x*y*C-z*s, x*z*C+y*s],
        [y*x*C+z*s, y*y*C+c,   y*z*C-x*s],
        [z*x*C-y*s, z*y*C+x*s, z*z*C+c  ]])

def box_sdf(p, b):
    q = np.abs(p) - b
    return np.linalg.norm(np.maximum(q, 0.0), axis=-1) + np.minimum(np.max(q, axis=-1), 0.0)

def interval_prim(kind, Minv3, pos, scale3, scalew, size3, sizew,
                  cell_c, cell_h):
    """Replica of lp_prim_interval (GLSL) exact path."""
    d4 = cell_c - pos
    cl = (Minv3 @ d4) / scale3
    hl = (np.abs(Minv3) @ cell_h) / scale3
    lo3, hi3 = cl - hl, cl + hl
    closest = np.clip(np.zeros(3), lo3, hi3)
    farthest = np.maximum(np.abs(lo3), np.abs(hi3))
    if kind == "sphere":
        return scalew * (np.linalg.norm(closest) - size3[0] - sizew), \
               scalew * (np.linalg.norm(farthest) - size3[0] - sizew)
    return scalew * (box_sdf(closest, size3) - sizew), \
           scalew * (box_sdf(farthest, size3) - sizew)

def field_prim(kind, Minv3, pos, scale3, scalew, size3, sizew, P):
    lp = (P - pos) @ Minv3.T / scale3
    if kind == "sphere":
        return scalew * (np.linalg.norm(lp, axis=-1) - size3[0] - sizew)
    return scalew * (box_sdf(lp, size3) - sizew)

worst = 0.0
for trial in range(4000):
    kind = "sphere" if trial % 2 == 0 else "box"
    M = rot_matrix(rng.normal(size=3), rng.uniform(0, np.pi))
    sc = rng.uniform(0.3, 2.5, 3)          # non-uniform, positive
    if trial % 7 == 0:
        sc[rng.integers(0, 3)] *= -1.0     # mirrored scale sometimes
    scalew = np.min(np.abs(sc))
    Minv3 = np.diag(1.0 / sc) @ M.T        # inverse of M @ diag(sc)
    pos = rng.uniform(-3, 3, 3)
    if kind == "sphere":
        size3 = np.full(3, rng.uniform(0.3, 1.0))
    else:
        size3 = rng.uniform(0.2, 1.0, 3)
    sizew = 0.0
    cell_c = rng.uniform(-4, 4, 3)
    cell_h = rng.uniform(0.05, 0.8, 3)
    lo, hi = interval_prim(kind, Minv3, pos, sc, scalew, size3, sizew, cell_c, cell_h)
    P = cell_c + rng.uniform(-1, 1, (256, 3)) * cell_h
    f = field_prim(kind, Minv3, pos, sc, scalew, size3, sizew, P)
    viol = max(lo - f.min(), f.max() - hi)
    if viol > 1e-5:
        worst = max(worst, viol)
        if viol > 0.05:
            print(f"VIOL {viol:.4f} kind={kind} sc={sc} lo={lo:.3f} hi={hi:.3f} "
                  f"fmin={f.min():.3f} fmax={f.max():.3f}")
print(f"max violation = {worst:.3e}")
