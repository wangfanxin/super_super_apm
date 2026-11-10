#!/usr/bin/env python3
"""wire_fold.py -- at a reference fold, is the step a JUMP or a smooth reversal?

`CUSP #` lines say the published reference turned >101 deg in one tick at |v| ~ 0.2 m/s.  Two very
different defects look like that on a 50 Hz log:

  * a JUMP: one abnormally large step (a splice/re-base discontinuity > a normal 10 ms step)
  * a SMOOTH reversal: the step shrinks to ~0, reverses, and grows again (the plan itself doubles back)

`wire.csv` records `p_l_*` (the plan box's own L-frame position, before the frame conversion) at the
publish rate, so it answers this without any clock fitting: `p_l` is exactly what the thin layer sent.

    python3 scripts/wire_fold.py kx4-01-01
    python3 scripts/wire_fold.py kx4-01-01 --near -0.15 -5.77      # one fold from the CUSP lines
"""
import argparse
import csv
import os

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tag")
    ap.add_argument("--near", nargs=2, type=float, default=None)
    ap.add_argument("--radius", type=float, default=0.25)
    ap.add_argument("--top", type=int, default=10)
    ap.add_argument("--root", default="/home/fanxin/lio_avoid/logs")
    a = ap.parse_args()

    p = os.path.join(a.root, "missionA-" + a.tag, "wire.csv")
    t, P = [], []
    for r in csv.DictReader(open(p)):
        t.append(float(r["t_wall"]))
        P.append((float(r["p_l_x"]), float(r["p_l_y"])))
    t = np.array(t); P = np.array(P)
    t = t - t[0]
    d = np.diff(P, axis=0)
    n = np.linalg.norm(d, axis=1)
    dt = np.median(np.diff(t))
    med = float(np.median(n))
    print("== %s: %d wire rows, %.1f s, median step %.4f m (%.2f m/s), max %.4f m"
          % (a.tag, len(t), t[-1], med, med / dt, n.max()))

    # a fold = consecutive steps with a >120 deg turn
    fold = []
    for i in range(1, len(d)):
        if n[i - 1] < 1e-6 or n[i] < 1e-6:
            continue
        c = float(np.dot(d[i - 1], d[i])) / (n[i - 1] * n[i])
        if c < -0.5:
            fold.append((i, float(np.degrees(np.arccos(np.clip(c, -1, 1))))))
    print("   folds (>120 deg between consecutive steps): %d" % len(fold))
    print("   step BEFORE / AT / AFTER a fold, in units of the median step:")
    mult = []
    for i, ang in fold:
        mult.append((n[i] / med, n[max(0, i - 1)] / med, n[min(len(n) - 1, i + 1)] / med))
    if mult:
        M = np.array(mult)
        print("      before p50 %.1f max %.1f | at p50 %.1f max %.1f | after p50 %.1f max %.1f"
              % (np.percentile(M[:, 1], 50), M[:, 1].max(), np.percentile(M[:, 0], 50), M[:, 0].max(),
                 np.percentile(M[:, 2], 50), M[:, 2].max()))
    big = np.argsort(-n)[:a.top]
    print("   %d largest steps (a splice shows up here):" % a.top)
    for i in big:
        print("      t=%7.2f  step %.4f m (=%.1fx median)  at (%.2f, %.2f)" % (t[i], n[i], n[i] / med, P[i][0], P[i][1]))

    if a.near is not None:
        dist = np.hypot(P[:, 0] - a.near[0], P[:, 1] - a.near[1])
        k = int(np.argmin(dist))
        print("\n   ---- nearest to (%s): t=%.2f, dist %.3f m" % (a.near, t[k], dist[k]))
        for j in range(max(0, k - 6), min(len(t), k + 7)):
            mark = " <== closest" if j == k else ""
            print("      t=%7.3f  (%.4f, %.4f)  step %.4f m = %.1f m/s%s"
                  % (t[j], P[j][0], P[j][1], n[j - 1] if j > 0 else 0.0,
                     (n[j - 1] / dt) if j > 0 else 0.0, mark))


if __name__ == "__main__":
    main()
