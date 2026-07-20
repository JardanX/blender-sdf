#!/usr/bin/env python3
"""Standalone numerical harness for the SDF Lipschitz-pruning (LP) engine.

Ports the classic GLSL blend kernels (sdf_lib.glsl) and the LP stack-machine
kernels (sdf_lp_common.glsl) verbatim and compares them on dense (a,b) grids:

  1. zero-set / sign equality classic vs LP,
  2. value differences (where, how much),
  3. winner-exactness of each LP op outside its dominance band (what the
     prune pass relies on: op == s*min(s*a,s*b) for |a-b| >= lp_op_dom_k),
  4. numeric (a,b)-Lipschitz constants of each LP op vs what
     sdf_lp_engine.cc assumes (max(Ll,Lr) vs sqrt(Ll^2+Lr^2)),
  5. (a,b)-gradient normal weights (the w2 selection in lp_list_eval_color_nrm)
     vs finite-difference gradients of the actual LP op, and vs the classic
     sdfCSGNormalWeights FD-of-combineCSG weights.

Conventions
-----------
Classic (scene convention): combineCSG(d1, d2) with d1 = accumulated/left,
d2 = new/right operand, exactly as in sdf_lib.glsl.

LP (stack convention): lp_binary_op_eval(a, b) where b is the value on the
stack, i.e. the right subtree value ALREADY negated for SUBTRACT (the engine
sets SDF_LP_SIGN_BIT on immediate right children of SUB ops). So for
SUBTRACT: LP(a=d1, b=-d2) must match classic(d1, d2).
"""

import numpy as np

# Blend type ids (eSDFBlendType / SDF_LP_BLEND_*)
LINEAR, SMOOTH, CHAMFER, ROUND = 0, 1, 2, 3
# CSG ids (eSDFCSGOperation subset / SDF_LP_CSG_*)
UNION, SUBTRACT, INTERSECT = 0, 1, 2


def glsl_sign(x):
    return np.sign(x)  # np.sign(0)=0, same as GLSL


def length2(x, y):
    return np.hypot(x, y)


# ---------------------------------------------------------------------------
# Classic kernels (verbatim ports of sdf_lib.glsl)
# ---------------------------------------------------------------------------

def opSmoothUnion(d1, d2, k):
    if k <= 0.0001:
        return np.minimum(d1, d2)
    out = np.minimum(d1, d2)
    m = np.abs(d2 - d1) < k
    h = np.clip(0.5 + 0.5 * (d2 - d1) / k, 0.0, 1.0)
    out = np.where(m, (d2 * (1 - h) + d1 * h) - k * h * (1 - h), out)
    return out


def opSmoothSubtraction(d1, d2, k):
    if k <= 0.0001:
        return np.maximum(-d1, d2)
    out = np.maximum(-d1, d2)
    m = np.abs(d2 + d1) < k
    h = np.clip(0.5 - 0.5 * (d2 + d1) / k, 0.0, 1.0)
    out = np.where(m, (d2 * (1 - h) + (-d1) * h) + k * h * (1 - h), out)
    return out


def opSmoothIntersection(d1, d2, k):
    if k <= 0.0001:
        return np.maximum(d1, d2)
    out = np.maximum(d1, d2)
    m = np.abs(d2 - d1) < k
    h = np.clip(0.5 - 0.5 * (d2 - d1) / k, 0.0, 1.0)
    out = np.where(m, (d2 * (1 - h) + d1 * h) + k * h * (1 - h), out)
    return out


def opChamferUnion(a, b, r):
    if r <= 0.0:
        return np.minimum(a, b)
    out = np.minimum(a, b)
    m = np.abs(a - b) < r
    out = np.where(m, np.minimum(np.minimum(a, b), (a - r + b) * 0.5), out)
    return out


def opChamferIntersection(a, b, r):
    if r <= 0.0:
        return np.maximum(a, b)
    out = np.maximum(a, b)
    m = np.abs(a - b) < r
    out = np.where(m, np.maximum(np.maximum(a, b), (a + r + b) * 0.5), out)
    return out


def opChamferSubtraction(d1, d2, r):
    return opChamferIntersection(d2, -d1, r)


def opSmoothChamferUnion(d1, d2, k, k2, k3):
    chamfer_plane = (d1 + d2 - k) * 0.5
    t1 = opSmoothUnion(d1, chamfer_plane, k2)
    t2 = opSmoothUnion(d2, chamfer_plane, k3)
    return np.minimum(t1, t2)


def opSmoothChamferSubtraction(d1, d2, k, k2, k3):
    A = -d1
    B = d2
    chamfer_plane = (A + B + k) * 0.5
    t1 = opSmoothIntersection(A, chamfer_plane, k2)
    t2 = opSmoothIntersection(B, chamfer_plane, k3)
    return np.maximum(t1, t2)


def opSmoothChamferIntersection(d1, d2, k, k2, k3):
    chamfer_plane = (d1 + d2 + k) * 0.5
    t1 = opSmoothIntersection(d1, chamfer_plane, k2)
    t2 = opSmoothIntersection(d2, chamfer_plane, k3)
    return np.maximum(t1, t2)


def mirror2D(px, py, nx, ny):
    proj = np.minimum(px * nx + py * ny, 0.0)
    return px - 2.0 * nx * proj, py - 2.0 * ny * proj


def opUnionIRound(a, b, r):
    """Classic spherical fillet, NO clamp (sdf_lib.glsl:849)."""
    n = 1.0 / np.sqrt(2.0)
    qx, qy = mirror2D(a, b, -n, n)
    qy = qy - r
    qy = np.minimum(0.0, qy)
    ad = glsl_sign(qx) * length2(qx, qy)
    sx, sy = np.maximum(a, 0.0), np.maximum(b, 0.0)
    corn = length2(sx, sy) - r
    return np.minimum(ad, corn)


def opRoundUnion(d1, d2, r):
    if r <= 0.0:
        return np.minimum(d1, d2)
    out = np.minimum(d1, d2)
    m = np.abs(d1 - d2) < r
    out = np.where(m, opUnionIRound(d1, d2, r), out)
    return out


def opRoundSubtraction(d1, d2, r):
    if r <= 0.0:
        return np.maximum(-d1, d2)
    out = np.maximum(-d1, d2)
    m = np.abs(d1 + d2) < r
    out = np.where(m, -opUnionIRound(d1, -d2, r), out)
    return out


def opRoundIntersection(d1, d2, r):
    if r <= 0.0:
        return np.maximum(d1, d2)
    out = np.maximum(d1, d2)
    m = np.abs(d1 - d2) < r
    out = np.where(m, -opUnionIRound(-d1, -d2, r), out)
    return out


def opSmoothRoundUnion(a, b, r, k2, k3):
    corner = length2(np.maximum(a, 0.0), np.maximum(b, 0.0)) - r
    t1 = opSmoothUnion(a, corner, k2)
    t2 = opSmoothUnion(b, corner, k3)
    return np.minimum(t1, t2)


def opSmoothRoundSubtraction(d1, d2, r, k2, k3):
    a = d2
    b = d1
    corner = r - length2(np.minimum(a, 0.0), np.maximum(b, 0.0))
    t1 = opSmoothIntersection(a, corner, k2)
    t2 = opSmoothIntersection(-b, corner, k3)
    return np.maximum(t1, t2)


def opSmoothRoundIntersection(d1, d2, r, k2, k3):
    corner = r - length2(np.minimum(d1, 0.0), np.maximum(-d2, 0.0))
    t1 = opSmoothIntersection(d1, corner, k2)
    t2 = opSmoothIntersection(d2, corner, k3)
    return np.maximum(t1, t2)


def opRoundUnionInverted(a, b, r):
    diff = -opUnionIRound(a, -b, r)
    return np.minimum(diff, b)


def opIntersectionRound(a, b, r):
    ux = np.maximum(r + a, 0.0)
    uy = np.maximum(r + b, 0.0)
    return np.minimum(-r, np.maximum(a, b)) + length2(ux, uy)


def opSmoothRoundUnionInverted(a, b, r, k2, k3):
    diff = opSmoothRoundSubtraction(b, a, r, k2, k3)
    return np.minimum(diff, b)


def opSmoothRoundIntersectionInverted(d1, d2, r, k2, k3):
    a = d1
    b = -d2
    corner = r - length2(np.minimum(a, 0.0), np.maximum(b, 0.0))
    t1 = opSmoothIntersection(a, corner, k2)
    t2 = opSmoothIntersection(-b, corner, k3)
    return np.maximum(t1, t2)


def combineCSG(d1, d2, op, bt, k, k2=0.0, k3=0.0):
    """Scene-convention classic dispatch (sdf_lib.glsl:1475, UNION/SUBTRACT/
    INTERSECT branches only)."""
    has_smooth = (k2 > 0.0 or k3 > 0.0)
    if op == UNION:
        if k > 0.0 and bt > 0:
            if bt == SMOOTH:
                return opSmoothUnion(d1, d2, k)
            if bt == CHAMFER:
                if has_smooth:
                    return opSmoothChamferUnion(d1, d2, k, k2, k3)
                return opChamferUnion(d1, d2, k)
            if bt == ROUND:
                if has_smooth:
                    return opSmoothRoundUnion(d1, d2, k, k2, k3)
                return opRoundUnion(d1, d2, k)
        return np.minimum(d1, d2)
    if op == SUBTRACT:
        if k > 0.0 and bt > 0:
            if bt == SMOOTH:
                return opSmoothSubtraction(d2, d1, k)
            if bt == CHAMFER:
                if has_smooth:
                    return opSmoothChamferSubtraction(d2, d1, k, k2, k3)
                return opChamferSubtraction(d2, d1, k)
            if bt == ROUND:
                if has_smooth:
                    return opSmoothRoundSubtraction(d2, d1, k, k2, k3)
                return opRoundSubtraction(d2, d1, k)
        return np.maximum(d1, -d2)
    if op == INTERSECT:
        if k > 0.0 and bt > 0:
            if bt == SMOOTH:
                return opSmoothIntersection(d1, d2, k)
            if bt == CHAMFER:
                if has_smooth:
                    return opSmoothChamferIntersection(d1, d2, k, k2, k3)
                return opChamferIntersection(d1, d2, k)
            if bt == ROUND:
                if has_smooth:
                    return opSmoothRoundIntersection(d1, d2, k, k2, k3)
                return opRoundIntersection(d1, d2, k)
        return np.maximum(d1, d2)
    raise ValueError(op)


# ---------------------------------------------------------------------------
# LP kernels (verbatim ports of sdf_lp_common.glsl)
# ---------------------------------------------------------------------------

def lp_kernel(x, k):
    if k == 0.0:
        return np.zeros_like(x)
    m = np.maximum(0.0, k - x)
    return m * m * 0.25 / k


def lp_kernel_chamfer(x, k):
    return np.maximum(0.0, k - x) * 0.5


def lp_op_iround(a, b, r):
    """Spherical fillet with the max(q.x, ad) clamp (sdf_lp_common.glsl:115)."""
    qx, qy = mirror2D(a, b, -0.70710678, 0.70710678)
    qy = qy - r
    qy = np.minimum(0.0, qy)
    ad = glsl_sign(qx) * length2(qx, qy)
    sx, sy = np.maximum(a, 0.0), np.maximum(b, 0.0)
    corn = length2(sx, sy) - r
    return np.minimum(np.maximum(qx, ad), corn)


# Smooth variants: identical to the classic ports (lp_* aliases).
lp_op_smooth_union = opSmoothUnion
lp_op_smooth_subtraction = opSmoothSubtraction
lp_op_smooth_intersection = opSmoothIntersection
lp_op_smooth_chamfer_union = opSmoothChamferUnion
lp_op_smooth_chamfer_subtraction = opSmoothChamferSubtraction
lp_op_smooth_chamfer_intersection = opSmoothChamferIntersection
lp_op_smooth_round_union = opSmoothRoundUnion
lp_op_smooth_round_subtraction = opSmoothRoundSubtraction
lp_op_smooth_round_intersection = opSmoothRoundIntersection
lp_op_intersection_round = opIntersectionRound
lp_op_smooth_round_intersection_inverted = opSmoothRoundIntersectionInverted


def lp_binary_op_eval(csg, bt, k, k2, k3, inverted, a, b):
    """Stack-convention LP dispatch (sdf_lp_common.glsl:264)."""
    s = 1.0 if csg == UNION else -1.0
    has_smooth = (k2 > 0.0 or k3 > 0.0)
    if (bt == CHAMFER or bt == ROUND) and k > 0.0:
        if has_smooth:
            if bt == CHAMFER:
                if csg == UNION:
                    return lp_op_smooth_chamfer_union(a, b, k, k2, k3)
                if csg == INTERSECT:
                    return lp_op_smooth_chamfer_intersection(a, b, k, k2, k3)
                return lp_op_smooth_chamfer_subtraction(-b, a, k, k2, k3)
            if inverted:
                return lp_op_smooth_round_intersection_inverted(a, b, k, k2, k3)
            if csg == UNION:
                return lp_op_smooth_round_union(a, b, k, k2, k3)
            if csg == INTERSECT:
                return lp_op_smooth_round_intersection(a, b, k, k2, k3)
            return lp_op_smooth_round_subtraction(-b, a, k, k2, k3)
        if bt == ROUND and inverted and csg == SUBTRACT:
            return lp_op_intersection_round(a, b, k)
    if bt == ROUND:
        if k <= 0.0:
            return s * np.minimum(s * a, s * b)
        return s * lp_op_iround(s * a, s * b, k)
    ker = lp_kernel_chamfer(np.abs(a - b), k) if bt == CHAMFER else lp_kernel(np.abs(a - b), k)
    return s * (np.minimum(s * a, s * b) - ker)


def lp_op_dom_k(bt, k, k2, k3):
    """Effective dominance margin (sdf_lp_common.glsl:334)."""
    if bt in (CHAMFER, ROUND) and (k2 > 0.0 or k3 > 0.0):
        return k + 2.0 * max(k2, k3)
    return k


def lp_winner(csg, a, b):
    """The value the prune pass substitutes when a child is culled."""
    s = 1.0 if csg == UNION else -1.0
    return s * np.minimum(s * a, s * b)


# ---------------------------------------------------------------------------
# LP normal-weight selection (port of lp_list_eval_color_nrm w2 code,
# sdf_lp_common.glsl:2259-2311)
# ---------------------------------------------------------------------------

def step(edge, x):
    """GLSL step."""
    return np.where(x >= edge, 1.0, 0.0)


def lp_normal_weights_fixed(csg, bt, k, k2, k3, inverted, left_val, right_val):
    """FIXED version: analytic LINEAR/SMOOTH, central FD of the op for
    CHAMFER/ROUND (all variants), mirroring the classic sdfCSGNormalWeights."""
    s = 1.0 if csg == UNION else -1.0
    x = s * left_val
    y = s * right_val
    if k <= 0.0:
        wl = step(x, y)
        return np.stack([wl, 1.0 - wl], axis=-1)
    if bt == SMOOTH:
        wa = np.clip(0.5 + 0.5 * (y - x) / k, 0.0, 1.0)
        return np.stack([wa, 1.0 - wa], axis=-1)
    e = 1e-5
    dxa = lp_binary_op_eval(csg, bt, k, k2, k3, inverted, left_val + e, right_val) - \
          lp_binary_op_eval(csg, bt, k, k2, k3, inverted, left_val - e, right_val)
    dyb = lp_binary_op_eval(csg, bt, k, k2, k3, inverted, left_val, right_val + e) - \
          lp_binary_op_eval(csg, bt, k, k2, k3, inverted, left_val, right_val - e)
    return np.stack([dxa / (2 * e), dyb / (2 * e)], axis=-1)


def lp_normal_weights(csg, bt, k, k2, k3, inverted, left_val, right_val):
    s = 1.0 if csg == UNION else -1.0
    x = s * left_val
    y = s * right_val

    wl = step(x, y)
    w_hard = np.stack([wl, 1.0 - wl], axis=-1)

    wa = np.clip(0.5 + 0.5 * (y - x) / max(k, 1e-12), 0.0, 1.0)
    w_smooth = np.stack([wa, 1.0 - wa], axis=-1)

    w_chamfer = w_hard * (1.0 - step(np.abs(x - y), k))[..., None] + \
        np.broadcast_to(np.array([0.5, 0.5]), w_hard.shape) * step(np.abs(x - y), k)[..., None]

    # ROUND: gradient of lp_op_iround.
    u = np.minimum(x, y)
    vv = np.maximum(x, y)
    qy = np.minimum(vv - k, 0.0)
    ad_len = length2(u, qy)
    gmx = u / np.maximum(ad_len, 1e-12)
    gmy = qy / np.maximum(ad_len, 1e-12)
    sel = step(u, 0.0)
    gmx = gmx * (1 - sel) + 1.0 * sel
    gmy = gmy * (1 - sel) + 0.0 * sel
    swp = step(y, x)
    gmx, gmy = gmx * (1 - swp) + gmy * swp, gmy * (1 - swp) + gmx * swp
    csx, csy = np.maximum(x, 0.0), np.maximum(y, 0.0)
    corn_len = length2(csx, csy)
    gcx = csx / np.maximum(corn_len, 1e-12)
    gcy = csy / np.maximum(corn_len, 1e-12)
    m_val = np.maximum(u, glsl_sign(u) * ad_len)
    selm = step(m_val, corn_len - k)
    w_round = np.stack([gcx * (1 - selm) + gmx * selm,
                        gcy * (1 - selm) + gmy * selm], axis=-1)

    # INVERTED ROUND (lp_op_intersection_round) on the RAW operands.
    u2x = k + left_val
    u2y = k + right_val
    u2px, u2py = np.maximum(u2x, 0.0), np.maximum(u2y, 0.0)
    l2 = length2(u2px, u2py)
    wl_i = step(right_val, left_val)
    seli = step(0.0, np.maximum(u2x, u2y))
    w_inverted = np.stack([
        wl_i * (1 - seli) + (u2px / np.maximum(l2, 1e-12)) * seli,
        (1.0 - wl_i) * (1 - seli) + (u2py / np.maximum(l2, 1e-12)) * seli,
    ], axis=-1)

    soft = (k2 > 0.0 or k3 > 0.0)
    sel_hard = 1.0 if k <= 0.0 else 0.0
    sel_chamfer = 1.0 if (bt == CHAMFER and not soft) else 0.0
    sel_round = 1.0 if (bt == ROUND and not soft and not inverted) else 0.0
    sel_inverted = 1.0 if (bt == ROUND and not soft and inverted) else 0.0
    sel_smooth = (1.0 - sel_hard) * (1.0 - sel_chamfer - sel_round - sel_inverted)
    w2 = (w_hard * sel_hard + w_chamfer * sel_chamfer + w_round * sel_round +
          w_inverted * sel_inverted + w_smooth * sel_smooth)
    return w2


# ---------------------------------------------------------------------------
# Classic normal weights (sdf_color_resolve_comp.glsl:94): LINEAR/SMOOTH
# analytic, CHAMFER/ROUND central FD of combineCSG. Scene convention; the
# returned weights multiply (n_a, n_b) with n_b the UN-negated right normal
# and b = d2. For SUBTRACT the classic b-weight is negated there; we convert
# to the LP convention (weights on the negated b = -d2) by negating weight_b.
# ---------------------------------------------------------------------------

def classic_normal_weights(d1, d2, op, bt, k, k2=0.0, k3=0.0, e=1e-5):
    if bt == LINEAR or k <= 0.0001:
        if op == UNION:
            w = np.where(d1 <= d2, 1.0, 0.0)
            return np.stack([w, 1.0 - w], axis=-1)
        if op == SUBTRACT:
            w = np.where(d1 >= -d2, 1.0, 0.0)
            return np.stack([w, -(1.0 - w)], axis=-1)
        w = np.where(d1 >= d2, 1.0, 0.0)
        return np.stack([w, 1.0 - w], axis=-1)
    if bt == SMOOTH:
        if op == UNION:
            h = np.clip(0.5 + 0.5 * (d2 - d1) / k, 0.0, 1.0)
            return np.stack([h, 1.0 - h], axis=-1)
        if op == SUBTRACT:
            h = np.clip(0.5 - 0.5 * (d1 + d2) / k, 0.0, 1.0)
            return np.stack([1.0 - h, -h], axis=-1)
        h = np.clip(0.5 - 0.5 * (d2 - d1) / k, 0.0, 1.0)
        return np.stack([h, 1.0 - h], axis=-1)
    wa = (combineCSG(d1 + e, d2, op, bt, k, k2, k3) -
          combineCSG(d1 - e, d2, op, bt, k, k2, k3)) / (2 * e)
    wb = (combineCSG(d1, d2 + e, op, bt, k, k2, k3) -
          combineCSG(d1, d2 - e, op, bt, k, k2, k3)) / (2 * e)
    return np.stack([wa, wb], axis=-1)


# ---------------------------------------------------------------------------
# Test machinery
# ---------------------------------------------------------------------------

def grid(n=801, lo=-3.0, hi=3.0):
    a = np.linspace(lo, hi, n)
    A, B = np.meshgrid(a, a, indexing="ij")
    return A, B


def fd_grad(fn, a, b, e=1e-5):
    """Central FD (a,b)-gradient of fn(a,b)."""
    ga = (fn(a + e, b) - fn(a - e, b)) / (2 * e)
    gb = (fn(a, b + e) - fn(a, b - e)) / (2 * e)
    return ga, gb


def lp_stack_operands(csg, d1, d2):
    """Scene operands -> LP stack operands."""
    return d1, (-d2 if csg == SUBTRACT else d2)


def name_of(csg, bt, soft, inverted):
    c = {UNION: "UNION", SUBTRACT: "SUBTRACT", INTERSECT: "INTERSECT"}[csg]
    b = {SMOOTH: "SMOOTH", CHAMFER: "CHAMFER", ROUND: "ROUND"}[bt]
    t = ""
    if soft:
        t += "+k2k3"
    if inverted:
        t += "+INVERTED"
    return f"{c}/{b}{t}"


def run():
    A, B = grid()
    rng = np.random.default_rng(7)
    ks = [0.2, 0.5, 1.0]

    print("=" * 88)
    print("TEST 1: classic vs LP — sign agreement and value difference")
    print("=" * 88)
    for csg in (UNION, SUBTRACT, INTERSECT):
        for bt in (SMOOTH, CHAMFER, ROUND):
            for k in ks:
                for soft in (False, True):
                    if soft and bt == SMOOTH:
                        continue
                    k2, k3 = (0.3 * k, 0.15 * k) if soft else (0.0, 0.0)
                    a, b = lp_stack_operands(csg, A, B)
                    classic = combineCSG(A, B, csg, bt, k, k2, k3)
                    lp = lp_binary_op_eval(csg, bt, k, k2, k3, False, a, b)
                    sign_mismatch = (np.sign(classic) != np.sign(lp))
                    # treat exact zeros as matching either sign
                    zeroish = (np.abs(classic) < 1e-12) | (np.abs(lp) < 1e-12)
                    sign_mismatch &= ~zeroish
                    dv = np.abs(classic - lp)
                    print(f"{name_of(csg,bt,soft,False):34s} k={k:4.2f} "
                          f"sign-mismatch={sign_mismatch.sum():6d} "
                          f"max|dv|={dv.max():9.4f} "
                          f"max|dv| near zero-set={dv[np.abs(classic)<0.05].max() if (np.abs(classic)<0.05).any() else 0.0:9.5f}")
                    if sign_mismatch.any():
                        idx = np.argwhere(sign_mismatch)[:5]
                        for i, j in idx:
                            print(f"    MISMATCH at a={A[i,j]:+.4f} b={B[i,j]:+.4f}: "
                                  f"classic={classic[i,j]:+.6f} lp={lp[i,j]:+.6f}")

    # Inward-shell start edge: classic opIntersectionRound(d1,-d2,k) vs
    # LP SUBTRACT+ROUND+INVERTED(a=d1, b=-d2).
    print("-" * 88)
    print("Inward shell start edge (SUBTRACT/ROUND+INVERTED vs opIntersectionRound)")
    for k in ks:
        classic = opIntersectionRound(A, -B, k)
        lp = lp_binary_op_eval(SUBTRACT, ROUND, k, 0.0, 0.0, True, A, -B)
        sm = (np.sign(classic) != np.sign(lp)) & ~((np.abs(classic) < 1e-12) | (np.abs(lp) < 1e-12))
        print(f"  k={k:4.2f} sign-mismatch={sm.sum():6d} max|dv|={np.abs(classic-lp).max():9.5f}")

    print()
    print("=" * 88)
    print("TEST 2: LP winner-exactness outside the dominance band |a-b| >= lp_op_dom_k")
    print("(prune substitutes lp_winner when a child is culled; any nonzero error")
    print(" outside the band is a wrong-cull candidate)")
    print("=" * 88)
    for csg in (UNION, SUBTRACT, INTERSECT):
        for bt in (SMOOTH, CHAMFER, ROUND):
            for k in ks:
                for soft in (False, True):
                    if soft and bt == SMOOTH:
                        continue
                    k2, k3 = (0.3 * k, 0.15 * k) if soft else (0.0, 0.0)
                    dom = lp_op_dom_k(bt, k, k2, k3)
                    a, b = lp_stack_operands(csg, A, B)
                    lp = lp_binary_op_eval(csg, bt, k, k2, k3, False, a, b)
                    w = lp_winner(csg, a, b)
                    outside = np.abs(a - b) >= dom - 1e-9
                    err = np.abs(lp - w)[outside]
                    mx = err.max() if err.size else 0.0
                    flag = "  <<< FAIL" if mx > 1e-4 else ""
                    print(f"{name_of(csg,bt,soft,False):34s} k={k:4.2f} dom={dom:5.3f} "
                          f"max|lp-winner| outside band={mx:10.6f}{flag}")
    print("-" * 88)
    print("Inverted round (lp_op_intersection_round), band |a-b| >= k:")
    for k in ks:
        a, b = A, -B
        lp = lp_binary_op_eval(SUBTRACT, ROUND, k, 0.0, 0.0, True, a, b)
        # max-type winner (s = -1 form on signed stack values)
        w = lp_winner(SUBTRACT, a, b)
        outside = np.abs(a - b) >= k - 1e-9
        err = np.abs(lp - w)[outside]
        sgn = ((np.sign(lp) != np.sign(w)) & outside &
               ~((np.abs(lp) < 1e-9) | (np.abs(w) < 1e-9)))
        print(f"  k={k:4.2f} max|lp-winner| outside band={err.max():10.6f} "
              f"sign-mismatch outside band={sgn.sum()}")
        # how large can the value error grow? (prune relevance)
        far = outside & (np.abs(a) < 10)
        print(f"        value error distribution outside band: "
              f"p50={np.percentile(err,50):.4f} p99={np.percentile(err,99):.4f}")

    print()
    print("=" * 88)
    print("TEST 3: numeric (a,b)-Lipschitz constant of each LP op")
    print("(max singular/secant ratio over the grid, plus plane-field position")
    print(" probes vs the engine's per-node constant assumption)")
    print("=" * 88)
    eps = 1e-4
    for csg in (UNION, SUBTRACT, INTERSECT):
        for bt in (SMOOTH, CHAMFER, ROUND):
            for k in (0.5, 1.0):
                for soft in (False, True):
                    if soft and bt == SMOOTH:
                        continue
                    k2, k3 = (0.3 * k, 0.15 * k) if soft else (0.0, 0.0)
                    a, b = lp_stack_operands(csg, A, B)
                    fn = lambda x, y: lp_binary_op_eval(csg, bt, k, k2, k3, False, x, y)
                    ga, gb = fd_grad(fn, a, b, eps)
                    gnorm = np.hypot(ga, gb)
                    # sup-metric lipschitz ~ max(|ga|,|gb|) upper estimate
                    gsup = np.maximum(np.abs(ga), np.abs(gb))
                    assumed = np.sqrt(2.0) if bt == ROUND else 1.0
                    ok2 = gnorm.max() <= assumed + 5e-3
                    print(f"{name_of(csg,bt,soft,False):34s} k={k:4.2f} "
                          f"max|grad|_2={gnorm.max():7.4f} max|grad|_inf={gsup.max():7.4f} "
                          f"assumed(Euclidean)={assumed:5.3f} {'OK' if ok2 else '<<< EXCEEDS'}")
    print("-" * 88)
    print("Inverted round (lp_op_intersection_round):")
    for k in (0.5, 1.0):
        fn = lambda x, y: lp_binary_op_eval(SUBTRACT, ROUND, k, 0.0, 0.0, True, x, y)
        ga, gb = fd_grad(fn, A, -B, eps)
        print(f"  k={k:4.2f} max|grad|_2={np.hypot(ga,gb).max():7.4f} "
              f"max|grad|_inf={np.maximum(np.abs(ga),np.abs(gb)).max():7.4f}")

    print()
    print("=" * 88)
    print("TEST 4: LP normal weights w2 vs FD (a,b)-gradient of the LP op,")
    print("and vs the classic sdfCSGNormalWeights (FD of combineCSG)")
    print("=" * 88)
    for csg in (UNION, SUBTRACT, INTERSECT):
        for bt in (SMOOTH, CHAMFER, ROUND):
            for k in (0.5, 1.0):
                for soft in (False, True):
                    if soft and bt == SMOOTH:
                        continue
                    k2, k3 = (0.3 * k, 0.15 * k) if soft else (0.0, 0.0)
                    a, b = lp_stack_operands(csg, A, B)
                    fn = lambda x, y: lp_binary_op_eval(csg, bt, k, k2, k3, False, x, y)
                    ga, gb = fd_grad(fn, a, b, eps)
                    w2 = lp_normal_weights_fixed(csg, bt, k, k2, k3, False, a, b)
                    # classic FD weights converted to LP convention (b negated
                    # for SUBTRACT): classic wb multiplies the un-negated d2
                    # normal, so w.r.t. b=-d2 the weight is -wb... but the op
                    # value also maps f_classic(d1,d2)=f_lp(a,b): d/d b = -d/d d2.
                    # So the FD-of-LP-op gradient is the ground truth here.
                    err = np.maximum(np.abs(w2[..., 0] - ga), np.abs(w2[..., 1] - gb))
                    # Only report away from kernel creases (gradient jumps):
                    # mask points near |a-b|==k etc. via FD second difference
                    f0 = fn(a, b)
                    curv = np.abs(fn(a+eps,b) + fn(a-eps,b) - 2*f0) + \
                           np.abs(fn(a,b+eps) + fn(a,b-eps) - 2*f0)
                    smooth_zone = curv < 1e-3
                    mx = err[smooth_zone].max() if smooth_zone.any() else 0.0
                    p99 = np.percentile(err[smooth_zone], 99) if smooth_zone.any() else 0.0
                    flag = "  <<< CHECK" if p99 > 2e-2 else ""
                    print(f"{name_of(csg,bt,soft,False):34s} k={k:4.2f} "
                          f"w2-vs-FDgrad: p99={p99:9.5f} max={mx:9.5f}{flag}")
                    if p99 > 2e-2:
                        bad = np.argwhere((err > 0.1) & smooth_zone)
                        for i, j in bad[:4]:
                            print(f"    at a={a[i,j]:+.4f} b={b[i,j]:+.4f}: "
                                  f"w2=({w2[i,j,0]:+.4f},{w2[i,j,1]:+.4f}) "
                                  f"fd=({ga[i,j]:+.4f},{gb[i,j]:+.4f})")
    print("-" * 88)
    print("INVERTED ROUND ops (shell edges):")
    for (csg, soft, note) in ((SUBTRACT, False, "inward start, plain"),
                              (INTERSECT, False, "outward flip-end, plain (eval = generic iround!)"),
                              (INTERSECT, True, "outward flip-end, soft")):
        for k in (0.5, 1.0):
            k2, k3 = (0.3 * k, 0.15 * k) if soft else (0.0, 0.0)
            a, b = lp_stack_operands(csg, A, B)
            fn = lambda x, y: lp_binary_op_eval(csg, ROUND, k, k2, k3, True, x, y)
            ga, gb = fd_grad(fn, a, b, eps)
            w2 = lp_normal_weights_fixed(csg, ROUND, k, k2, k3, True, a, b)
            err = np.maximum(np.abs(w2[..., 0] - ga), np.abs(w2[..., 1] - gb))
            f0 = fn(a, b)
            curv = np.abs(fn(a+eps,b) + fn(a-eps,b) - 2*f0) + \
                   np.abs(fn(a,b+eps) + fn(a,b-eps) - 2*f0)
            smooth_zone = curv < 1e-3
            p99 = np.percentile(err[smooth_zone], 99) if smooth_zone.any() else 0.0
            mx = err[smooth_zone].max() if smooth_zone.any() else 0.0
            flag = "  <<< CHECK" if p99 > 2e-2 else ""
            print(f"  csg={csg} soft={soft} ({note}) k={k:4.2f} "
                  f"p99={p99:9.5f} max={mx:9.5f}{flag}")
            if p99 > 2e-2:
                bad = np.argwhere((err > 0.1) & smooth_zone)
                for i, j in bad[:4]:
                    print(f"    at a={a[i,j]:+.4f} b={b[i,j]:+.4f}: "
                          f"w2=({w2[i,j,0]:+.4f},{w2[i,j,1]:+.4f}) "
                          f"fd=({ga[i,j]:+.4f},{gb[i,j]:+.4f})")

    print()
    print("=" * 88)
    print("TEST 5: cross-check LP FD gradient vs classic FD gradient on the")
    print("classic zero set (where both fields vanish); they must agree there.")
    print("=" * 88)
    for csg in (UNION, SUBTRACT, INTERSECT):
        for bt in (CHAMFER, ROUND):
            for k in (0.5, 1.0):
                a, b = lp_stack_operands(csg, A, B)
                fc = lambda x, y: combineCSG(x, y, csg, bt, k)
                fl = lambda x, y: lp_binary_op_eval(csg, bt, k, 0.0, 0.0, False,
                                                    x if csg != SUBTRACT else x,
                                                    y)
                # classic FD in scene convention
                gca, gcb = fd_grad(fc, A, B, eps)
                # LP FD w.r.t. scene d2 as well: d/d d2 of lp(a, -d2) for SUB
                s = -1.0 if csg == SUBTRACT else 1.0
                gla, glb = fd_grad(lambda x, y: lp_binary_op_eval(
                    csg, bt, k, 0.0, 0.0, False, x, s * y), A, B, eps)
                zon = np.abs(fc(A, B)) < 0.02
                # exclude crease points
                f0 = fc(A, B)
                curv = np.abs(fc(A+eps,B) + fc(A-eps,B) - 2*f0) + \
                       np.abs(fc(A,B+eps) + fc(A,B-eps) - 2*f0)
                zon &= curv < 1e-3
                if zon.any():
                    da = np.abs(gca - gla)[zon]
                    db = np.abs(gcb - glb)[zon]
                    print(f"{name_of(csg,bt,False,False):34s} k={k:4.2f} zero-set "
                          f"|dgrad_a| p99={np.percentile(da,99):9.5f} "
                          f"|dgrad_b| p99={np.percentile(db,99):9.5f}")


# ---------------------------------------------------------------------------
# TEST 6: SHELL desugar. Port of the classic combineCSG SHELL branch
# (sdf_lib.glsl:1580-1735) and of the C++ lp_build_tree combine() SHELL
# desugar (sdf_lp_engine.cc:560-679), compared as functions of (d1, d2).
# ---------------------------------------------------------------------------

def classic_shell(d1, d2, bt, sd, shell_op, k_top, k_bot, k2, k3, k4, k5,
                  fb, fbe):
    """Classic combineCSG SHELL branch (verbatim port)."""
    sd = -sd if shell_op == 1 else sd  # SDF_SHELL_OP_SUBTRACTION == 1
    h = abs(sd)
    has_smooth = (k2 > 0.0 or k3 > 0.0)
    has_smooth_end = (k4 > 0.0 or k5 > 0.0)

    def blend_sub(keep, cutter, k, kk2, kk3):
        """op*Subtraction(cutter, keep) scene-form == max(keep, -cutter)."""
        if k <= 0.0 or bt == LINEAR:
            return np.maximum(keep, -cutter)
        if bt == SMOOTH:
            return opSmoothSubtraction(cutter, keep, k)
        if bt == CHAMFER:
            hs = (kk2 > 0.0 or kk3 > 0.0)
            if hs:
                return opSmoothChamferSubtraction(cutter, keep, k, kk2, kk3)
            return opChamferSubtraction(cutter, keep, k)
        # ROUND
        hs = (kk2 > 0.0 or kk3 > 0.0)
        if hs:
            return opSmoothRoundSubtraction(cutter, keep, k, kk2, kk3)
        return opRoundSubtraction(cutter, keep, k)

    if sd < 0.0:
        # Inward: start = subtraction (bottom), end = union with d1+h.
        if k_top > 0.0 and bt > 0:
            if fb:
                d_sub = blend_sub(d1, d2, k_top, k2, k3)
            elif bt == SMOOTH:
                d_sub = opSmoothSubtraction(d2, d1, k_top)
            elif bt == CHAMFER:
                if has_smooth:
                    d_sub = opSmoothChamferSubtraction(d2, d1, k_top, k2, k3)
                else:
                    d_sub = opChamferSubtraction(d2, d1, k_top)
            elif bt == ROUND:
                d_sub = opIntersectionRound(d1, -d2, k_top)
            else:
                d_sub = np.maximum(d1, -d2)
        else:
            d_sub = np.maximum(d1, -d2)
        lim = d1 + h
        if k_bot > 0.0 and bt > 0:
            if fbe:
                if bt == SMOOTH:
                    esub = opSmoothSubtraction(lim, d_sub, k_bot)
                    d_shell = np.minimum(esub, lim)
                elif bt == CHAMFER:
                    if has_smooth_end:
                        esub = opSmoothChamferSubtraction(lim, d_sub, k_bot, k4, k5)
                    else:
                        esub = opChamferSubtraction(lim, d_sub, k_bot)
                    d_shell = np.minimum(esub, lim)
                else:
                    if has_smooth_end:
                        d_shell = opSmoothRoundUnionInverted(d_sub, lim, k_bot, k4, k5)
                    else:
                        d_shell = opRoundUnionInverted(d_sub, lim, k_bot)
            elif bt == SMOOTH:
                d_shell = opSmoothUnion(d_sub, lim, k_bot)
            elif bt == CHAMFER:
                if has_smooth_end:
                    d_shell = opSmoothChamferUnion(d_sub, lim, k_bot, k4, k5)
                else:
                    d_shell = opChamferUnion(d_sub, lim, k_bot)
            elif bt == ROUND:
                if has_smooth_end:
                    d_shell = opSmoothRoundUnion(d_sub, lim, k_bot, k4, k5)
                else:
                    d_shell = opRoundUnion(d_sub, lim, k_bot)
            else:
                d_shell = np.minimum(d_sub, lim)
        else:
            d_shell = np.minimum(d_sub, lim)
        return d_shell
    # Outward: start = union (top), end = intersection with d1-h.
    if k_top > 0.0 and bt > 0:
        if fb:
            sub = blend_sub(d1, d2, k_top, k2, k3)
            d_union = np.minimum(sub, d2)
        elif bt == SMOOTH:
            d_union = opSmoothUnion(d1, d2, k_top)
        elif bt == CHAMFER:
            if has_smooth:
                d_union = opSmoothChamferUnion(d1, d2, k_top, k2, k3)
            else:
                d_union = opChamferUnion(d1, d2, k_top)
        elif bt == ROUND:
            if has_smooth:
                d_union = opSmoothRoundUnion(d1, d2, k_top, k2, k3)
            else:
                d_union = opRoundUnion(d1, d2, k_top)
        else:
            d_union = np.minimum(d1, d2)
    else:
        d_union = np.minimum(d1, d2)
    lim = d1 - h
    if k_bot > 0.0 and bt > 0:
        if fbe:
            if bt == SMOOTH:
                esub = opSmoothSubtraction(lim, d_union, k_bot)
                d_shell = np.maximum(esub, lim)
            elif bt == CHAMFER:
                if has_smooth_end:
                    esub = opSmoothChamferSubtraction(lim, d_union, k_bot, k4, k5)
                else:
                    esub = opChamferSubtraction(lim, d_union, k_bot)
                d_shell = np.maximum(esub, lim)
            else:
                if has_smooth_end:
                    d_shell = opSmoothRoundIntersectionInverted(d_union, lim, k_bot, k4, k5)
                else:
                    d_shell = opRoundIntersection(d_union, lim, k_bot)
        elif bt == SMOOTH:
            d_shell = opSmoothIntersection(d_union, lim, k_bot)
        elif bt == CHAMFER:
            if has_smooth_end:
                d_shell = opSmoothChamferIntersection(d_union, lim, k_bot, k4, k5)
            else:
                d_shell = opChamferIntersection(d_union, lim, k_bot)
        elif bt == ROUND:
            d_shell = opSmoothIntersection(d_union, lim, k_bot)
        else:
            d_shell = np.maximum(d_union, lim)
    else:
        d_shell = np.maximum(d_union, lim)
    return d_shell


def lp_shell(d1, d2, bt, sd, shell_op, k_top, k_bot, k2, k3, k4, k5, fb, fbe):
    """C++ lp_build_tree SHELL desugar (sdf_lp_engine.cc:560-679): the same
    tree shape, evaluated with lp_binary_op_eval on stack-convention values."""
    sd = -sd if shell_op == 1 else sd
    h = abs(sd)
    INV = True
    NOINV = False

    def evalop(csg, k, btype, a, b, kk2=0.0, kk3=0.0, inv=False):
        bb = -b if csg == SUBTRACT else b
        return lp_binary_op_eval(csg, btype, k, kk2, kk3, inv, a, bb)

    def smooth_ok(btype, k):
        return k > 0.0 and btype in (CHAMFER, ROUND)

    t2, t3 = (k2, k3) if smooth_ok(bt, k_top) else (0.0, 0.0)
    e2, e3 = (k4, k5) if smooth_ok(bt, k_bot) else (0.0, 0.0)
    bt_top = bt if k_top > 0.0 else LINEAR
    bt_bot = bt if k_bot > 0.0 else LINEAR

    if sd < 0.0:
        if bt_top == ROUND and not fb:
            t = evalop(SUBTRACT, k_top, bt_top, d1, d2, 0.0, 0.0, INV)
        else:
            t = evalop(SUBTRACT, k_top, bt_top, d1, d2, t2, t3, NOINV)
        lim = d1 + h
        if not fbe or bt_bot == LINEAR:
            return evalop(UNION, k_bot, bt_bot, t, lim, e2, e3)
        round_plain = (bt_bot == ROUND and e2 == 0.0 and e3 == 0.0)
        if round_plain:
            e = evalop(SUBTRACT, k_bot, bt_bot, lim, t, 0.0, 0.0)
        else:
            e = evalop(SUBTRACT, k_bot, bt_bot, t, lim, e2, e3)
        lim2 = d1 + h
        return evalop(UNION, 0.0, LINEAR, e, lim2)
    # Outward.
    if fb and bt_top != LINEAR:
        sub = evalop(SUBTRACT, k_top, bt_top, d1, d2, t2, t3)
        t = evalop(UNION, 0.0, LINEAR, sub, d2)
    else:
        t = evalop(UNION, k_top, bt_top, d1, d2, t2, t3)
    lim = d1 - h
    if not fbe or bt_bot == LINEAR:
        bt_b, be2, be3 = bt_bot, e2, e3
        if bt_bot == ROUND:
            bt_b, be2, be3 = SMOOTH, 0.0, 0.0
        return evalop(INTERSECT, k_bot, bt_b, t, lim, be2, be3)
    if bt_bot == ROUND:
        return evalop(INTERSECT, k_bot, bt_bot, t, lim, e2, e3, INV)
    e = evalop(SUBTRACT, k_bot, bt_bot, t, lim, e2, e3)
    lim2 = d1 - h
    return evalop(INTERSECT, 0.0, LINEAR, e, lim2)


def test_shells():
    A, B = grid()
    print()
    print("=" * 88)
    print("TEST 6: SHELL desugar (C++ lp_build_tree) vs classic combineCSG SHELL")
    print("=" * 88)
    nfail = 0
    for bt in (SMOOTH, CHAMFER, ROUND):
        for sd, shell_op in ((0.4, 0), (-0.4, 1), (0.4, 1), (-0.4, 0)):
            for fb in (0, 1):
                for fbe in (0, 1):
                    for ktop, kbot in ((0.5, 0.0), (0.0, 0.5), (0.5, 0.3)):
                        for soft in (False, True):
                            k2, k3 = (0.15, 0.1) if soft else (0.0, 0.0)
                            k4, k5 = (0.12, 0.08) if soft else (0.0, 0.0)
                            cl = classic_shell(A, B, bt, sd, shell_op, ktop, kbot,
                                               k2, k3, k4, k5, fb, fbe)
                            lp = lp_shell(A, B, bt, sd, shell_op, ktop, kbot,
                                          k2, k3, k4, k5, fb, fbe)
                            sm = (np.sign(cl) != np.sign(lp)) & \
                                 ~((np.abs(cl) < 1e-9) | (np.abs(lp) < 1e-9))
                            dv = np.abs(cl - lp)
                            # near-zero-set difference
                            zon = np.abs(cl) < 0.05
                            dz = dv[zon].max() if zon.any() else 0.0
                            status = ""
                            if sm.any() or dz > 1e-3:
                                status = "  <<< FAIL"
                                nfail += 1
                            if status or dv.max() > 1e-3:
                                print(f"bt={bt} sd={sd:+.1f} sop={shell_op} fb={fb} fbe={fbe} "
                                      f"kt={ktop:.1f} kb={kbot:.1f} soft={int(soft)}: "
                                      f"sign-mismatch={sm.sum():6d} max|dv|={dv.max():8.4f} "
                                      f"max|dv|@zero={dz:9.5f}{status}")
                            if sm.any():
                                idx = np.argwhere(sm)[:4]
                                for i, j in idx:
                                    print(f"    at d1={A[i,j]:+.4f} d2={B[i,j]:+.4f}: "
                                          f"classic={cl[i,j]:+.6f} lp={lp[i,j]:+.6f}")
    if nfail == 0:
        print("  (no sign mismatches and no near-zero-set value diffs > 1e-3 "
              "in any shell configuration)")


if __name__ == "__main__":
    run()
    test_shells()
