#!/usr/bin/env python3
"""Scene-level simulator for the LP engine: tree build, prune pass, marcher.

Reproduces a ~20-object all-ROUND-union scene and checks, at SCENE level:
  1. full LP tree field vs classic engine field (sign / zero-set equality),
  2. per-cell prune output vs the full LP field (substitution exactness,
     far-field conservativeness — no culled cell may contain a zero crossing),
  3. the march loop vs a reference march on the classic field (depth match,
     overshoot detection),
  4. prune efficiency stats ROUND vs SMOOTH vs LINEAR (why perf differs),
  5. true position-Lipschitz constant of the composed field vs the engine's
     per-node sqrt(Ll^2+Lr^2) constants.

Ports (verbatim as feasible):
  - tree build + per-node Lipschitz constants: sdf_lp_engine.cc:868-923
  - prune forward pass (dominance + far field): sdf_lp_prune_comp.glsl:141-240
  - march loop: sdf_lp_march_comp.glsl:77-117
  - kernels: harness.py (classic sdf_lib.glsl + LP sdf_lp_common.glsl)
"""

import numpy as np
import sys, os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from harness import (UNION, SUBTRACT, INTERSECT, LINEAR, SMOOTH, CHAMFER, ROUND,
                     lp_binary_op_eval, lp_op_dom_k, combineCSG)

rng = np.random.default_rng(11)

# ---------------------------------------------------------------------------
# Scene: ~20 spheres/boxes, pairwise ROUND union.
# ---------------------------------------------------------------------------

def make_scene(n=20, spread=2.2, seed=11):
    rng = np.random.default_rng(seed)
    prims = []
    for i in range(n):
        c = rng.uniform(-spread, spread, 3)
        if i % 2 == 0:
            prims.append(("sphere", c, rng.uniform(0.35, 0.8)))
        else:
            prims.append(("box", c, rng.uniform(0.25, 0.6, 3)))
    blends = rng.uniform(0.1, 0.5, n)  # blend factor of object i (i>=1 used)
    return prims, blends


def prim_eval(kind, c, s, P):
    """P: (...,3) points -> (...,) signed distance. 1-Lipschitz leaves."""
    if kind == "sphere":
        return np.linalg.norm(P - c, axis=-1) - s
    q = np.abs(P - c) - s
    return (np.linalg.norm(np.maximum(q, 0.0), axis=-1) +
            np.minimum(np.max(q, axis=-1), 0.0))


# ---------------------------------------------------------------------------
# Tree build (sdf_lp_engine.cc lp_build_tree: left fold, post-order).
# Nodes: dict(type, prim, op, bt, k, k2, k3, flags, left, right, lip)
# All ops here are UNION (sign +1 everywhere, no SUB right-child negation).
# ---------------------------------------------------------------------------

def build_tree(prims, blends, blend_type=ROUND):
    nodes = []

    def add_leaf(i):
        nodes.append(dict(type="prim", prim=prims[i], left=-1, right=-1, lip=1.0))
        return len(nodes) - 1

    def add_op(op, bt, k, l, r, k2=0.0, k3=0.0, flags=0):
        ll, lr = nodes[l]["lip"], nodes[r]["lip"]
        lip = np.sqrt(ll * ll + lr * lr) if (bt == ROUND and k > 0.0) else max(ll, lr)
        nodes.append(dict(type="binary", op=op, bt=bt, k=k, k2=k2, k3=k3,
                          flags=flags, left=l, right=r, lip=lip))
        return len(nodes) - 1

    acc = add_leaf(0)
    for i in range(1, len(prims)):
        operand = add_leaf(i)
        acc = add_op(UNION, blend_type, blends[i], acc, operand)
    return nodes, acc


def eval_nodes(nodes, root, P, upto=None):
    """Evaluate all node values at points P (...,3) -> list of (...,) arrays.
    Stack-machine order == post-order append order (lp_list_eval)."""
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


def eval_classic(prims, blends, blend_type, P):
    """Classic engine field: combineCSG fold with the classic kernels."""
    acc = prim_eval(*prims[0], P)
    for i in range(1, len(prims)):
        d2 = prim_eval(*prims[i], P)
        acc = combineCSG(acc, d2, UNION, blend_type, blends[i])
    return acc


# ---------------------------------------------------------------------------
# Prune pass (single level from the full tree; hierarchical culling gives the
# same result per the exactness argument — a coarse substitution valid on the
# coarse cell is valid on every child cell).
# ---------------------------------------------------------------------------

def prune_grid(nodes, root, aabb_min, aabb_max, gs):
    """Returns per-cell: (mode, payload) where mode in
    {0: far (payload=constant), 1: list (payload=skip set / survivor map)}.
    Vectorized over cells."""
    cs = (aabb_max - aabb_min) / gs
    R = np.linalg.norm(cs) * 0.5
    lin = np.arange(gs)
    C = np.stack(np.meshgrid(lin, lin, lin, indexing="ij"), axis=-1)
    centers = aabb_min + cs * (C + 0.5)                      # (gs,gs,gs,3)
    P = centers.reshape(-1, 3)

    vals = eval_nodes(nodes, root, P)                        # per node (ncells,)
    root_v = vals[root]
    root_l = nodes[root]["lip"]

    # Dominance decisions per binary node (sdf_lp_prune_comp.glsl:175-199).
    n = len(nodes)
    skipped = np.zeros((n, len(P)), dtype=bool)
    dominated_left = np.zeros((n, len(P)), dtype=bool)
    for idx, nd in enumerate(nodes):
        if nd["type"] != "binary":
            continue
        k = lp_op_dom_k(nd["bt"], nd["k"], nd["k2"], nd["k3"])
        lv, rv = vals[nd["left"]], vals[nd["right"]]
        band = (nodes[nd["left"]]["lip"] + nodes[nd["right"]]["lip"]) * R + k
        skip = np.abs(lv - rv) > band
        s = 1.0 if nd["op"] == UNION else -1.0
        skipped[idx] = skip
        dominated_left[idx] = skip & (s * lv > s * rv)  # left dominated

    # Far field (sdf_lp_prune_comp.glsl:236-239).
    far = np.abs(root_v) > 2.0 * root_l * R
    scene_l = nodes[root]["lip"]
    far_val = np.sign(root_v) * (np.abs(root_v) - root_l * R) / scene_l

    return dict(centers=P, vals=vals, skipped=skipped, dominated_left=dominated_left,
                far=far, far_val=far_val, R=R, cs=cs, gs=gs,
                aabb_min=aabb_min, aabb_max=aabb_max)


def pruned_eval_at(nodes, root, prune, cell_ids, P):
    """Evaluate the cell's pruned tree at points P (one point per cell id).
    Recursive with per-cell skip substitution."""
    memo = {}

    def ev(idx):
        if idx in memo:
            return memo[idx]
        nd = nodes[idx]
        if nd["type"] == "prim":
            kind, c, s = nd["prim"]
            out = prim_eval(kind, c, s, P)
        else:
            skip = prune["skipped"][idx][cell_ids]
            dl = prune["dominated_left"][idx][cell_ids]
            lv = ev(nd["left"])
            rv = ev(nd["right"])
            # Where skipped: substitute the winner (surviving child).
            win = np.where(dl, rv, lv)
            opv = lp_binary_op_eval(nd["op"], nd["bt"], nd["k"], nd["k2"],
                                    nd["k3"], bool(nd["flags"]), lv, rv)
            out = np.where(skip, win, opv)
        memo[idx] = out
        return out

    return ev(root)


# ---------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------

def check_field_vs_classic(nodes, root, prims, blends, bt, aabb_min, aabb_max, res=96):
    lin = np.linspace(aabb_min[0], aabb_max[0], res)
    P = np.stack(np.meshgrid(lin, lin, lin, indexing="ij"), axis=-1).reshape(-1, 3)
    lp = eval_nodes(nodes, root, P)[root]
    cl = eval_classic(prims, blends, bt, P)
    sm = (np.sign(lp) != np.sign(cl)) & (np.abs(lp) > 1e-9) & (np.abs(cl) > 1e-9)
    dv = np.abs(lp - cl)
    print(f"[field] LP vs classic on {res}^3: sign-mismatch={sm.sum()} "
          f"max|dv|={dv.max():.4f} max|dv|@|classic|<0.05="
          f"{dv[np.abs(cl) < 0.05].max() if (np.abs(cl) < 0.05).any() else 0:.5f}")
    return P, lp, cl


def check_prune(nodes, root, prune, samples_per_cell=9):
    """Per-cell: pruned eval == full eval (exactness of substitution), and far
    cells contain no zero crossing of the full field."""
    gs = prune["gs"]
    ncells = gs ** 3
    cs, R = prune["cs"], prune["R"]
    centers = prune["centers"]
    # Sample points: center + 8 corners shrunk 90%.
    offs = np.array([[0, 0, 0]] + [[sx, sy, sz] for sx in (-0.45, 0.45)
                                   for sy in (-0.45, 0.45) for sz in (-0.45, 0.45)])
    max_sub_err = 0.0
    far_viol = 0
    far_checked = 0
    lipschitz_viol = 0
    lip_max_ratio = 0.0
    # Check a random subset plus all near-surface cells.
    near = np.abs(prune["vals"][root]) < 2.5 * nodes[root]["lip"] * R
    subset = np.unique(np.concatenate([
        np.flatnonzero(near),
        rng.choice(ncells, min(3000, ncells), replace=False)]))
    for ci in subset:
        c = centers[ci]
        P = c[None, :] + offs * cs[None, :]
        ids = np.full(len(P), ci)
        full = eval_nodes(nodes, root, P)[root]
        pru = pruned_eval_at(nodes, root, prune, ids, P)
        err = np.max(np.abs(full - pru))
        max_sub_err = max(max_sub_err, err)
        if prune["far"][ci]:
            far_checked += 1
            # bound must hold: sign(const) == sign(full) and |full| >= |const|
            fv = prune["far_val"][ci]
            if np.any(np.sign(full) != np.sign(fv)) or \
                np.any(np.abs(full) < nodes[root]["lip"] * np.abs(fv) - 1e-6):
                far_viol += 1
        if err > 1e-4:
            lipschitz_viol += 1
        # Local Lipschitz probe on the root field.
        d = np.linalg.norm(P - c, axis=-1)
        nz = d > 1e-9
        ratio = np.max(np.abs(full[nz] - prune["vals"][root][ci]) / d[nz]) if nz.any() else 0.0
        lip_max_ratio = max(lip_max_ratio, ratio)
    print(f"[prune] cells checked={len(subset)} substitution max err={max_sub_err:.2e} "
          f"(cells with err>1e-4: {lipschitz_viol})")
    print(f"[prune] far cells checked={far_checked} far-bound violations={far_viol}")
    print(f"[prune] root field local secant ratio max={lip_max_ratio:.4f} "
          f"(engine root Lipschitz={nodes[root]['lip']:.4f})")
    return max_sub_err, far_viol, lip_max_ratio


def check_march(nodes, root, prune, prims, blends, bt, nrays=2000, max_steps=256):
    """March rays against the pruned cell field (port of sdf_lp_march_comp)
    and against the classic field; compare depth. Detect overshoot: any march
    step whose segment contains a zero crossing of the FULL LP field while
    both endpoint values are positive."""
    gs, cs = prune["gs"], prune["cs"]
    aabb_min, aabb_max = prune["aabb_min"], prune["aabb_max"]
    # Rays: random origins on a sphere around the AABB, aimed at random
    # points inside.
    cen = 0.5 * (aabb_min + aabb_max)
    rad = np.linalg.norm(aabb_max - aabb_min)
    O = cen + rad * rng.normal(size=(nrays, 3))
    T = cen + 0.5 * (aabb_max - aabb_min) * rng.uniform(-1, 1, (nrays, 3))
    D = T - O
    D /= np.linalg.norm(D, axis=-1, keepdims=True)

    def cell_of(P):
        c = np.clip(((P - aabb_min) / cs).astype(int), 0, gs - 1)
        return (c[:, 0] * gs + c[:, 1]) * gs + c[:, 2]

    ray_eps = 1e-3

    def march(eval_cell):
        t = np.zeros(nrays)
        hit = np.zeros(nrays, dtype=bool)
        overshoot = np.zeros(nrays, dtype=bool)
        t_prev = np.zeros(nrays)
        d_prev = np.full(nrays, 1e30)
        alive = np.ones(nrays, dtype=bool)
        for step in range(max_steps):
            if not alive.any():
                break
            idx = np.flatnonzero(alive)
            P = O[idx] + t[idx, None] * D[idx]
            inside = np.all((P >= aabb_min) & (P < aabb_max), axis=-1)
            # evaluate
            d = np.full(len(idx), 1e30)
            pin = P[inside]
            ci = cell_of(pin)
            far = prune["far"][ci]
            d_in = np.empty(len(pin))
            if far.any():
                d_in[far] = prune["far_val"][ci[far]]
            if (~far).any():
                d_in[~far] = pruned_eval_at(nodes, root, prune, ci[~far], pin[~far])
            d[inside] = d_in
            # overshoot probe: full LP field sampled along the segment the
            # step just crossed; negative mid-sample with positive endpoints
            # means the step jumped a surface the hit test cannot see.
            span = t[idx] - t_prev[idx]
            for frac in (0.25, 0.5, 0.75):
                Pm = O[idx] + (t_prev[idx] + frac * span)[:, None] * D[idx]
                inb = np.all((Pm >= aabb_min) & (Pm < aabb_max), axis=-1) & \
                      (span > 4 * ray_eps)
                if inb.any():
                    fm = eval_nodes(nodes, root, Pm[inb])[root]
                    neg = np.zeros(len(idx), bool)
                    neg[inb] = fm < 0.0
                    overshoot[idx[neg & (d_prev[idx] > 0) & (d > 0)]] = True
            sign_change = (d_prev[idx] < 1e29) & (d_prev[idx] > 0) & (d < 0)
            new_hit = inside & ((np.abs(d) < ray_eps) | sign_change)
            # secant refine
            hit[idx[new_hit]] = True
            alive[idx[new_hit]] = False
            cont = ~new_hit
            t_prev[idx[cont]] = t[idx[cont]]
            d_prev[idx[cont]] = d[cont]
            # Step (sdf_lp_march_comp): near cells |d|/scene_l; far cells use
            # the pre-divided constant as-is (d already holds it).
            far_mask = np.zeros(len(idx), dtype=bool)
            far_mask[inside] = far
            step = np.abs(d) / np.where(far_mask, 1.0, nodes[root]["lip"])
            t[idx[cont]] += step[cont]
            out = inside & (np.abs(d) > 1e29)
            alive[idx[out]] = False
            # exited AABB
            alive[idx[~inside]] = False
        return hit, t, overshoot

    def march_ref():
        # classic field, same stepping
        def eval_cell(P):
            return eval_classic(prims, blends, bt, P)
        t = np.zeros(nrays)
        hit = np.zeros(nrays, dtype=bool)
        alive = np.ones(nrays, dtype=bool)
        d_prev = np.full(nrays, 1e30)
        t_prev = np.zeros(nrays)
        for step in range(max_steps):
            if not alive.any():
                break
            idx = np.flatnonzero(alive)
            P = O[idx] + t[idx, None] * D[idx]
            inside = np.all((P >= aabb_min) & (P < aabb_max), axis=-1)
            d = np.full(len(idx), 1e30)
            d[inside] = eval_classic(prims, blends, bt, P[inside])
            sign_change = (d_prev[idx] < 1e29) & (d_prev[idx] > 0) & (d < 0)
            new_hit = inside & ((np.abs(d) < ray_eps) | sign_change)
            hit[idx[new_hit]] = True
            alive[idx[new_hit]] = False
            cont = ~new_hit
            t_prev[idx[cont]] = t[idx[cont]]
            d_prev[idx[cont]] = d[cont]
            t[idx[cont]] += np.abs(d[cont])
            alive[idx[~inside]] = False
        return hit, t

    hit_lp, t_lp, over = march(None)
    hit_cl, t_cl = march_ref()
    both = hit_lp & hit_cl
    dt = np.abs(t_lp - t_cl)[both]
    print(f"[march] rays={nrays} hits lp={hit_lp.sum()} classic={hit_cl.sum()} "
          f"depth|dt|: max={dt.max() if dt.size else 0:.5f} "
          f"mean={dt.mean() if dt.size else 0:.2e} "
          f"overshoot-suspect rays={over.sum()}")
    return hit_lp, t_lp, hit_cl, t_cl, over


def prune_stats(nodes, root, prune, bt_name):
    gs = prune["gs"]
    ncells = gs ** 3
    nskip = np.zeros(ncells)
    for idx, nd in enumerate(nodes):
        if nd["type"] == "binary":
            nskip += prune["skipped"][idx]
    far = prune["far"].mean()
    print(f"[stats] {bt_name:7s} root Lipschitz={nodes[root]['lip']:7.4f} "
          f"far-cell fraction={far:6.2%} mean skipped ops per cell={nskip.mean():5.2f} "
          f"/ {sum(1 for nd in nodes if nd['type']=='binary')}")
    return nskip.mean(), far


def check_iround_lipschitz_euclidean():
    """Adversarial secant probes of lp_op_iround in the Euclidean (a,b)
    metric, focused on the mirror crease (q.x=0), the negative quadrant and
    the |a-b|~r boundary."""
    from harness import lp_op_iround
    worst = 0.0
    for k in (0.2, 0.5, 1.0):
        for region in ((0.0, 0.3, -1.5, 0.0),      # near crease u~0, v<r
                       (-1.5, -0.05, -1.5, -0.05),  # negative quadrant
                       (-0.5, 1.5, -0.5, 1.5)):     # around band boundary
            a = rng.uniform(region[0], region[1], 20000)
            b = rng.uniform(region[2], region[3], 20000)
            ang = rng.uniform(0, 2 * np.pi, 20000)
            e = rng.uniform(1e-6, 1e-2, 20000)
            da, db = e * np.cos(ang), e * np.sin(ang)
            f0 = lp_op_iround(a, b, k)
            f1 = lp_op_iround(a + da, b + db, k)
            ratio = np.abs(f1 - f0) / e
            worst = max(worst, np.nanmax(ratio))
    print(f"[iround] Euclidean (a,b) secant max ratio = {worst:.4f} (must be <= 1)")
    return worst


def run():
    prims, blends = make_scene()
    aabb_min = np.array([-3.4, -3.4, -3.4])
    aabb_max = np.array([3.4, 3.4, 3.4])
    gs = 32

    check_iround_lipschitz_euclidean()
    print()
    for bt, name in ((ROUND, "ROUND"), (SMOOTH, "SMOOTH"), (LINEAR, "LINEAR")):
        print("=" * 78)
        print(f"blend type: {name}")
        print("=" * 78)
        nodes, root = build_tree(prims, blends, bt)
        check_field_vs_classic(nodes, root, prims, blends, bt, aabb_min, aabb_max)
        prune = prune_grid(nodes, root, aabb_min, aabb_max, gs)
        prune_stats(nodes, root, prune, name)
        check_prune(nodes, root, prune)
        check_march(nodes, root, prune, prims, blends, bt)
        print()


if __name__ == "__main__":
    run()


# ---------------------------------------------------------------------------
# Hierarchical prune (levels 4,16,64 -> here gs0=4, two refinements) with the
# exact inheritance semantics of sdf_lp_prune_comp.glsl: each cell starts from
# its parent cell's emitted list (substitutions + rewired survivors), evaluates
# at its own center, culls, and emits its own list / far constant.
# ---------------------------------------------------------------------------

def hier_prune(nodes, root, aabb_min, aabb_max, levels=(2, 4)):
    """levels: grid levels as power-of-two exponents minus... use gs list like
    (4, 16): level grids. Returns per-level cell data for validation."""
    n = len(nodes)
    out_levels = []
    # Level state: for each cell: dict(mode='list'|'far', skip=bool array per
    # node (this cell's additional skips inherit via re-eval), val=const,
    # survivor mapping implicit via skip evaluation).
    parent = None  # array (ncells,) of dict
    for gs in levels:
        cs = (aabb_max - aabb_min) / gs
        R = np.linalg.norm(cs) * 0.5
        lin = np.arange(gs)
        C = np.stack(np.meshgrid(lin, lin, lin, indexing="ij"), axis=-1)
        centers = (aabb_min + cs * (C + 0.5)).reshape(-1, 3)
        ncells = gs ** 3
        # parent cell index: 4x4x4 children per parent.
        if parent is None:
            pmode = np.full(ncells, 1)  # 1 = list (full tree)
            pskip = np.zeros((ncells, n), dtype=bool)
            pdl = np.zeros((ncells, n), dtype=bool)
            pval = np.zeros(ncells)
        else:
            pgs = levels[out_levels.__len__() - 1]
            par_lin = (C // 4).reshape(-1, 3)
            pci = (par_lin[:, 0] * pgs + par_lin[:, 1]) * pgs + par_lin[:, 2]
            prev = out_levels[-1]
            pmode = prev["mode"][pci]
            pskip = prev["skip"][pci]
            pdl = prev["dl"][pci]
            pval = prev["val"][pci]

        # Evaluate all node values at centers WITH inherited substitutions.
        vals = [None] * n
        for idx, nd in enumerate(nodes):
            if nd["type"] == "prim":
                kind, c, s = nd["prim"]
                vals[idx] = prim_eval(kind, c, s, centers)
            else:
                lv, rv = vals[nd["left"]], vals[nd["right"]]
                opv = lp_binary_op_eval(nd["op"], nd["bt"], nd["k"], nd["k2"],
                                        nd["k3"], bool(nd["flags"]), lv, rv)
                sk = pskip[:, idx]
                win = np.where(pdl[:, idx], rv, lv)
                vals[idx] = np.where(sk, win, opv)

        mode = np.copy(pmode)  # 1 list, 0 far
        val = np.copy(pval)
        skip = np.copy(pskip)
        dl = np.copy(pdl)
        for idx, nd in enumerate(nodes):
            if nd["type"] != "binary":
                continue
            already = skip[:, idx]
            k = lp_op_dom_k(nd["bt"], nd["k"], nd["k2"], nd["k3"])
            lv, rv = vals[nd["left"]], vals[nd["right"]]
            band = (nodes[nd["left"]]["lip"] + nodes[nd["right"]]["lip"]) * R + k
            new_skip = (~already) & (np.abs(lv - rv) > band)
            s = 1.0 if nd["op"] == UNION else -1.0
            dl[:, idx] = np.where(new_skip, (s * lv > s * rv), dl[:, idx])
            skip[:, idx] |= new_skip
        # Far-field test on cells still in list mode.
        root_v, root_l = vals[root], nodes[root]["lip"]
        newly_far = (mode == 1) & (np.abs(root_v) > 2.0 * root_l * R)
        mode[newly_far] = 0
        scene_l = nodes[root]["lip"]
        val[newly_far] = np.sign(root_v[newly_far]) * \
            (np.abs(root_v[newly_far]) - root_l * R) / scene_l
        out_levels.append(dict(gs=gs, cs=cs, R=R, centers=centers, mode=mode,
                               val=val, skip=skip, dl=dl, root_v=root_v))
    return out_levels


def check_hier_prune(nodes, root, levels_data, aabb_min, aabb_max):
    last = levels_data[-1]
    gs, cs, R = last["gs"], last["cs"], last["R"]
    centers, mode, val = last["centers"], last["mode"], last["val"]
    skip, dl = last["skip"], last["dl"]
    offs = np.array([[0, 0, 0]] + [[sx, sy, sz] for sx in (-0.45, 0.45)
                                   for sy in (-0.45, 0.45) for sz in (-0.45, 0.45)])
    root_v = last["root_v"]
    near = (mode == 1) & (np.abs(root_v) < 2.5 * nodes[root]["lip"] * R)
    far_cells = np.flatnonzero(mode == 0)
    subset = np.unique(np.concatenate([
        np.flatnonzero(near),
        rng.choice(len(centers), min(3000, len(centers)), replace=False)]))
    max_err = 0.0
    err_cells = 0
    for ci in subset:
        c = centers[ci]
        P = c[None, :] + offs * cs[None, :]
        full = eval_nodes(nodes, root, P)[root]
        # pruned eval with this cell's final skip set
        prune = dict(skipped=skip[ci], dominated_left=dl[ci])
        pru = np.empty(len(P))
        # inline substitution eval (skip arrays per single cell)
        vals = [None] * len(nodes)
        for idx, nd in enumerate(nodes):
            if nd["type"] == "prim":
                kind, cc, s = nd["prim"]
                vals[idx] = prim_eval(kind, cc, s, P)
            else:
                lv, rv = vals[nd["left"]], vals[nd["right"]]
                opv = lp_binary_op_eval(nd["op"], nd["bt"], nd["k"], nd["k2"],
                                        nd["k3"], bool(nd["flags"]), lv, rv)
                win = rv if prune["dominated_left"][idx] else lv
                vals[idx] = win if prune["skipped"][idx] else opv
        pru = vals[root]
        err = np.max(np.abs(full - pru))
        if err > max_err:
            max_err = err
        if err > 1e-4:
            err_cells += 1
    # far cells: verify no zero crossing and bound validity, sampled.
    far_bad = 0
    far_checked = 0
    for ci in far_cells[::max(1, len(far_cells) // 2000)]:
        c = centers[ci]
        P = c[None, :] + offs * cs[None, :]
        full = eval_nodes(nodes, root, P)[root]
        fv = val[ci]
        far_checked += 1
        if np.any(np.sign(full) != np.sign(fv)) or \
                np.any(np.abs(full) < nodes[root]["lip"] * np.abs(fv) - 1e-6):
            far_bad += 1
    print(f"[hier] levels={[l['gs'] for l in levels_data]} "
          f"final cells={len(centers)} far={100*np.mean(mode==0):.1f}% "
          f"substitution max err={max_err:.2e} bad cells={err_cells} "
          f"far checked={far_checked} far violations={far_bad}")
    return max_err, far_bad


def check_aabb_bulge(nodes, root, prims, blends):
    """Scene AABB (object bounds + 0.1% pad, sdf_lp_engine.cc:139-152) vs the
    blended surface: how much of the zero set lies OUTSIDE the box?"""
    bmin = np.full(3, 1e30)
    bmax = np.full(3, -1e30)
    for kind, c, s in prims:
        r = s if kind == "sphere" else s
        bmin = np.minimum(bmin, c - r)
        bmax = np.maximum(bmax, c + r)
    pad = (bmax - bmin) * 0.001 + 1e-4
    aabb_min, aabb_max = bmin - pad, bmax + pad
    # dense sample of the field near its zero set
    lin = np.linspace(aabb_min[0] - 1.0, aabb_max[0] + 1.0, 160)
    P = np.stack(np.meshgrid(lin, lin, lin, indexing="ij"), axis=-1).reshape(-1, 3)
    f = eval_nodes(nodes, root, P)[root]
    onsurf = np.abs(f) < 0.02
    outside = np.any((P < aabb_min) | (P > aabb_max), axis=-1)
    # how far outside?
    dout = np.maximum.reduce([aabb_min - P, P - aabb_max, np.zeros_like(P)]).max(-1)
    bad = onsurf & outside
    print(f"[aabb] object-bounds box + 0.1% pad: surface samples outside box: "
          f"{bad.sum()} / {onsurf.sum()} (max overshoot distance "
          f"{dout[bad].max() if bad.any() else 0.0:.4f}; k range "
          f"{blends.min():.2f}..{blends.max():.2f})")
    return bad.sum(), dout[bad].max() if bad.any() else 0.0


def run2():
    prims, blends = make_scene()
    nodes, root = build_tree(prims, blends, ROUND)
    check_aabb_bulge(nodes, root, prims, blends)
    # Tight scene AABB from object bounds (as the engine computes) for the
    # hierarchical test.
    bmin = np.full(3, 1e30); bmax = np.full(3, -1e30)
    for kind, c, s in prims:
        bmin = np.minimum(bmin, c - s); bmax = np.maximum(bmax, c + s)
    pad = (bmax - bmin) * 0.001 + 1e-4
    levels = hier_prune(nodes, root, bmin - pad, bmax + pad, levels=(4, 16, 64))
    check_hier_prune(nodes, root, levels, bmin - pad, bmax + pad)


def pool_stats(nodes, root, prune, capacity=None):
    """Emulate the active-list pool: total emitted entries across all cells vs
    the engine's capacity (sdf_lp_engine.cc:982: max(2*num_cells, 2^21))."""
    gs = prune["gs"]
    ncells = gs ** 3
    nbin = sum(1 for nd in nodes if nd["type"] == "binary")
    active_per_cell = float(len(nodes)) - prune["skipped"].sum(axis=0)
    # far cells store nothing
    active_per_cell = np.where(prune["far"], 0.0, active_per_cell)
    total = active_per_cell.sum()
    cap = max(2 * ncells, 1 << 21) if capacity is None else capacity
    print(f"[pool] total active entries emitted={int(total)} capacity={cap} "
          f"-> {'OVERFLOW x%.1f' % (total / cap) if total > cap else 'fits'} "
          f"(fallback cells would trace the full {len(nodes)}-node tree)")
    return total, cap
