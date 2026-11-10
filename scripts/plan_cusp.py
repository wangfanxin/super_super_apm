#!/usr/bin/env python3
"""plan_cusp.py -- does the PLANNED trajectory itself contain a hairpin?

`dip_zoom.py` shows the published reference decelerating to exactly 0 mid-leg and then backing up
8 cm before going on.  The reference is the plan, sampled -- so the question is whether the plan's
own path doubles back, or whether the doubling is created by splicing the new trajectory onto the
prefix of the old one (`temp_exp_traj = temp_exp_traj + out_traj` in `generateExpTraj`).

`traj.csv.points.csv` holds every committed plan's sampled points (`t_us` = the plan's wall clock,
`i` = sample index, `e/n/u` = W).  For each plan this reports the sharpest turn (angle between
consecutive steps) and where it is, then prints the points around the sharpest one.

    python3 scripts/plan_cusp.py /home/fanxin/lio_avoid/logs/missionA-kx1-02-01 --near 0.95 -8.35
"""
import argparse
import csv
import os

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--near", nargs=2, type=float, default=None,
                    help="only plans passing within this many metres of (x y) ... give x y")
    ap.add_argument("--radius", type=float, default=0.35)
    ap.add_argument("--show", type=int, default=2, help="how many cusps to print in full")
    a = ap.parse_args()

    p = os.path.join(a.run, "traj.csv.points.csv")
    plans, cur, key = {}, [], None
    with open(p) as f:
        rdr = csv.DictReader(f)
        for r in rdr:
            k = int(r["t_us"])
            if k != key:
                if cur:
                    plans[key] = cur
                key, cur = k, []
            cur.append((float(r["e"]), float(r["n"]), float(r["u"])))
    if cur:
        plans[key] = cur
    print("== %s: %d committed plans, %d points total"
          % (os.path.basename(a.run), len(plans), sum(len(v) for v in plans.values())))

    rows = []
    for k, pts in plans.items():
        if len(pts) < 8:
            continue
        P = np.array(pts)
        # DECIMATE to a fixed arc-length spacing first: the dump samples densely (millimetre steps),
        # and an angle between two millimetre steps is noise, not a hairpin.
        seg = np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1)
        s = np.r_[0.0, np.cumsum(seg)]
        if s[-1] < 0.5:
            continue
        want = np.arange(0.0, s[-1], 0.10)
        idx = np.searchsorted(s, want)
        idx = idx[(idx > 0) & (idx < len(P))]
        if len(idx) < 4:
            continue
        D = np.diff(P[idx][:, :2], axis=0)
        L = np.linalg.norm(D, axis=1)
        ang = np.zeros(len(D))
        for i in range(1, len(D)):
            if L[i] < 1e-3 or L[i - 1] < 1e-3:
                continue
            c = np.dot(D[i - 1], D[i]) / (L[i - 1] * L[i])
            ang[i] = np.degrees(np.arccos(np.clip(c, -1.0, 1.0)))
        if a.near is not None:
            dist = np.min(np.hypot(P[idx, 0] - a.near[0], P[idx, 1] - a.near[1]))
            if dist > a.radius:
                continue
        i = int(np.argmax(ang))
        rows.append((float(ang[i]), k, i, P[idx], ang, L))
    print("   plans passing near (%s): %d" % (a.near, len(rows)))
    print("   %14s %6s %8s %9s %9s %8s" % ("plan_start_us", "npts", "max_turn", "at_point", "step_m", "len_m"))
    rows.sort(key=lambda r: -r[0])
    for ang_max, k, i, P, ang, L in rows[:12]:
        print("   %14d %6d %8.1f %9d %9.3f %8.2f"
              % (k, len(P), ang_max, i, L[i], np.sum(L)))
    for ang_max, k, i, P, ang, L in rows[:a.show]:
        print("\n   ---- plan %d, turn %.1f deg at point %d (of %d)" % (k, ang_max, i, len(P)))
        lo, hi = max(0, i - 8), min(len(P), i + 9)
        for j in range(lo, hi):
            mark = "  <== turn %.0f deg" % ang[j] if j == i else ""
            print("      i=%3d  (%7.3f, %7.3f, %6.3f)  step %.3f%s"
                  % (j, P[j, 0], P[j, 1], P[j, 2], L[j] if j < len(L) else 0.0, mark))


if __name__ == "__main__":
    main()
