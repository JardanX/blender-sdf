#!/usr/bin/env python3
"""Verify lp_binary_op_eval is componentwise nondecreasing in STACK space
(a, b) for every emittable op form -> op interval = [eval(lo), eval(hi)]
is exact. Random boxes, dense interior probes."""
import numpy as np
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from harness import (UNION, SUBTRACT, INTERSECT, SMOOTH, CHAMFER, ROUND,
                     lp_binary_op_eval)

rng = np.random.default_rng(9)
worst = 0.0
for csg in (UNION, SUBTRACT, INTERSECT):
    for bt in (SMOOTH, CHAMFER, ROUND):
        for k in (0.0, 0.2, 0.5, 1.0):
            for soft in (False, True):
                if soft and (bt == SMOOTH or k == 0.0):
                    continue
                k2, k3 = (0.3 * k, 0.15 * k) if soft else (0.0, 0.0)
                for inv in (False, True):
                    if inv and not (bt == ROUND and not soft):
                        continue
                    for trial in range(300):
                        lo = rng.uniform(-2.5, 2.5, 2)
                        hi = lo + rng.uniform(0.01, 2.0, 2)
                        a_lo, b_lo = lo
                        a_hi, b_hi = hi
                        f_lo = lp_binary_op_eval(csg, bt, k, k2, k3, inv, a_lo, b_lo)
                        f_hi = lp_binary_op_eval(csg, bt, k, k2, k3, inv, a_hi, b_hi)
                        P = rng.uniform(lo, hi, (64, 2))
                        f = lp_binary_op_eval(csg, bt, k, k2, k3, inv, P[:,0], P[:,1])
                        viol = max((f_lo - f).max(), (f - f_hi).max())
                        worst = max(worst, viol)
                        if viol > 1e-5:
                            print(f"VIOLATION csg={csg} bt={bt} k={k} soft={soft} inv={inv} "
                                  f"box=({lo})-({hi}) viol={viol}")
print(f"max monotonicity violation = {worst:.2e} (must be 0)")
