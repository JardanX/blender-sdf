#!/usr/bin/env python3
"""Adversarial test: parallel thin slabs round-unioned (fillet crests with
parallel child gradients -> composed field Lipschitz ~ sqrt(n) -> |f| can
exceed the true distance; sphere tracing with step |f| skips thin features).
"""
import numpy as np
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scene_sim import prim_eval, build_tree, eval_nodes, eval_classic
from harness import UNION, ROUND, SMOOTH

rng = np.random.default_rng(3)

# Thin slab: box centered c, half-thickness h in x, wide in y/z.
def slab(c, h, w=3.0):
    return ("box", np.array(c, float), np.array([h, w, w]))

def zero_set_dist(P, fvals=None, f=None, res=400, bbox=(-4, 4)):
    """Approx dist(p, Z) via dense 1D sampling along x (scene is x-varying)."""
    xs = np.linspace(bbox[0], bbox[1], 4001)
    if f is None:
        fx = fvals
    else:
        fx = f(np.stack([xs, np.zeros_like(xs), np.zeros_like(xs)], -1))
    return xs, fx

def overshoot_ratio(f):
    """max |f(p)| / dist(p,Z) on the x-axis (scene varies along x mostly)."""
    xs = np.linspace(-4.0, 4.0, 8001)
    P = np.stack([xs, np.zeros_like(xs), np.zeros_like(xs)], -1)
    fx = f(P)
    # zero crossings
    sgn = np.sign(fx)
    cross = np.flatnonzero(sgn[:-1] * sgn[1:] < 0)
    if len(cross) == 0:
        return None
    zx = []
    for c in cross:
        # bisection
        a, b = xs[c], xs[c + 1]
        fa, fb = fx[c], fx[c + 1]
        for _ in range(60):
            m = 0.5 * (a + b)
            fm = f(np.array([[m, 0.0, 0.0]]))[0]
            if fa * fm <= 0:
                b, fb = m, fm
            else:
                a, fa = m, fm
        zx.append(0.5 * (a + b))
    zx = np.array(zx)
    dist = np.min(np.abs(xs[:, None] - zx[None, :]), axis=1)
    ratio = np.abs(fx) / np.maximum(dist, 1e-12)
    i = np.argmax(ratio)
    return ratio[i], xs[i], dist[i], len(zx)

def march_1d(f, x0, step_scale):
    """March along +x from x0 with step = step_scale*|f|; returns hit x or None,
    and whether a thin feature was skipped (crossed an even number of zero
    crossings in one step without detecting)."""
    t = x0
    d_prev = 1e30
    t_prev = t
    for _ in range(1000):
        d = f(np.array([[t, 0.0, 0.0]]))[0]
        if abs(d) < 1e-4:
            return t, False
        if d_prev < 1e29 and d_prev > 0 and d < 0:
            return t, False
        t_prev, d_prev = t, d
        t += step_scale * abs(d)
        if t > 8.0:
            return None, False
    return None, False

def first_crossing(f, x0):
    xs = np.linspace(x0, 8.0, 20001)
    fx = f(np.stack([xs, np.zeros_like(xs), np.zeros_like(xs)], -1))
    s = np.sign(fx)
    c = np.flatnonzero(s[:-1] * s[1:] < 0)
    return xs[c[0]] if len(c) else None

def run():
    # Wall of N parallel slabs thickness 2h, gap g between surfaces, round k.
    for (N, h, g, k) in ((6, 0.15, 0.25, 0.5), (6, 0.1, 0.15, 0.4),
                         (10, 0.08, 0.12, 0.45), (4, 0.2, 0.3, 0.6)):
        prims = []
        x = 0.0
        for i in range(N):
            prims.append(slab((x, 0, 0), h))
            x += 2 * h + g
        blends = [0.0] + [k] * (N - 1)
        nodes, root = build_tree(prims, blends, ROUND)
        f_lp = lambda P: eval_nodes(nodes, root, P)[root]
        f_cl = lambda P: eval_classic(prims, blends, ROUND, P)
        r_lp = overshoot_ratio(f_lp)
        r_cl = overshoot_ratio(f_cl)
        print(f"N={N} h={h} g={g} k={k}: overshoot |f|/dist max: "
              f"LP={r_lp[0]:.3f} at x={r_lp[1]:.3f} (dist={r_lp[2]:.4f}, {r_lp[3]} crossings), "
              f"classic={r_cl[0]:.3f}" if r_lp and r_cl else "no crossings")
        # march through the wall with L=1 (current) vs L=root (fixed)
        L = nodes[root]["lip"]
        misses1 = missesL = 0
        for x0 in np.linspace(-3.9, -0.5, 60):
            ref = first_crossing(f_lp, x0)
            h1, _ = march_1d(f_lp, x0, 1.0)
            hL, _ = march_1d(f_lp, x0, 1.0 / L)
            if ref is not None:
                if h1 is None or h1 > ref + 0.05:
                    misses1 += 1
                if hL is None or hL > ref + 0.05:
                    missesL += 1
        print(f"   root L={L:.2f}  march misses (step=|f|): {misses1}/60   "
              f"(step=|f|/L): {missesL}/60")

run()
