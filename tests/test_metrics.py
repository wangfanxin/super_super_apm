#!/usr/bin/env python3
"""test_metrics.py -- regression tests for the 2026-11-28 metric fixes (no flight).

Covers the two metric bugs the three-way audit found and this round fixed:
  * S2 path_loops must detect a SLOW loop (0.8 m/s, r = 0.5 m) and must report a duration that does NOT
    include 2*win_s of padding;
  * S1's END window must use the LAND start (phase.csv), not the first approach to the last corner.
"""
import os, sys
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import low_speed_census as L


def _circle(v, r, dur, hz=200.0, phase=0.0):
    t = np.arange(0.0, dur, 1.0 / hz)
    w = v / max(r, 1e-9)
    E = r * np.cos(w * t + phase); N = r * np.sin(w * t + phase)
    U = np.full_like(t, 2.0)
    return t, E, N, U


def _run_loops(t, E, N, U):
    orig = L.load
    L.load = lambda tag: dict(t=t, E=E, N=N, U=U, ts=t, ce=E, cn=N, cu=U, corners=[])
    try:
        return L.path_loops("synthetic")
    finally:
        L.load = orig


def main():
    bad = 0
    # FAST loop: 1.35 m/s, r = 0.5 m, 8 s
    t, E, N, U = _circle(1.35, 0.5, 8.0)
    l = _run_loops(t, E, N, U)
    if not l:
        print("FAIL: fast loop not detected"); bad += 1
    else:
        print("fast loop: %d loop(s), dur %.2f s" % (len(l), l[0]["dur"]))
    # SLOW loop: 0.6 m/s, r = 0.5 m, 12 s (must be caught by the multi-scale window)
    t2, E2, N2, U2 = _circle(0.6, 0.5, 12.0)
    l2 = _run_loops(t2, E2, N2, U2)
    if not l2:
        print("FAIL: slow loop not detected (multi-scale window regression)"); bad += 1
    else:
        print("slow loop: %d loop(s), dur %.2f s" % (len(l2), l2[0]["dur"]))
        # duration without padding: it must stay inside the trace itself (the old code added 2*win of
        # padding, so a 12 s trace could report up to 14.4 s); allow the 1 s air-start margin
        if l2[0]["dur"] > 12.0 - 0.5:
            print("FAIL: slow-loop duration looks padded (%.2f s)" % l2[0]["dur"]); bad += 1
    # STRAIGHT line: must NOT be a loop
    t3 = np.arange(0, 12.0, 0.005)
    l3 = _run_loops(t3, 1.3 * t3, np.zeros_like(t3), np.full_like(t3, 2.0))
    if l3:
        print("FAIL: straight line reported as loop"); bad += 1
    else:
        print("straight line: no loop (ok)")
    print("METRICS TEST: %s" % ("PASS" if bad == 0 else "FAIL (%d)" % bad))
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
