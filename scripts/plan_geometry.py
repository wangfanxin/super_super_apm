#!/usr/bin/env python3
"""plan_geometry.py -- the plan's REAL turning radius, after removing the 50 Hz sample jitter.

`path_curvature.py` differentiates the setpoint stream raw, so `kappa` there is dominated by
per-tick jitter: a 0.3 m "radius" at 1 m/s means the heading spins 190 deg/s, which no real path
does.  Here the position stream is low-passed over N samples before differentiating, and N is
swept so the reader can see where the number stops depending on the smoothing.

    R(N)         = 1 / kappa, kappa = |x'y'' - y'x''| / (x'^2 + y'^2)^1.5
    v_ceiling(N) = sqrt(a_tilt * R_p10(N))          a_tilt = g*tan(45) = 9.81 m/s^2
    a_demand(v)  = v^2 * kappa                      → the fraction over the tilt limit

Both the COMMANDED path and the FLOWN path are reported: if the flown radius is much larger than
the commanded one, the aircraft is cutting the weave, which is a tracking problem, not a plan one.

    python3 scripts/plan_geometry.py hx9-01-01 v3h132-01-01 [--arrays 5]
"""
import argparse
import csv
import os

import numpy as np

MISSION = "/home/fanxin/lio_avoid/logs"
COMPACT = "/mnt/d/distill_self/super_super_apm/logs"
A_TILT = 9.81


def find(tag, root):
    for cand in (os.path.join(MISSION, "missionA-" + tag, "setpoints.csv"),
                 os.path.join(root, tag, "raw", "setpoints.csv"),
                 os.path.join(root, tag, "setpoints.csv")):
        if os.path.isfile(cand):
            return cand
    return None


def load(path):
    t, c, f = [], [], []
    for r in csv.DictReader(open(path)):
        if "t_wall" not in r:
            continue
        t.append(float(r["t_wall"]))
        c.append((float(r["cmd_x"]), float(r["cmd_y"])))
        f.append((float(r["fcu_x"]), float(r["fcu_y"])))
    return np.array(t), np.array(c), np.array(f)


def smooth(x, n):
    if n <= 1 or len(x) < n:
        return x
    k = np.ones(n) / n
    if x.ndim == 1:
        return np.convolve(x, k, mode="same")
    return np.stack([np.convolve(x[:, i], k, mode="same") for i in range(x.shape[1])], 1)


def kappa_of(t, p, n):
    q = smooth(p, n)
    dt = np.gradient(t)
    d1 = np.gradient(q, axis=0) / dt[:, None]
    d2 = np.gradient(d1, axis=0) / dt[:, None]
    sp = np.linalg.norm(d1, axis=1)
    num = np.abs(d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0])
    den = np.maximum(sp, 1e-6) ** 3
    k = num / den
    k[sp < 0.15] = np.nan            # a hover has no curvature
    return k, sp


ap = argparse.ArgumentParser()
ap.add_argument("tags", nargs="+")
ap.add_argument("--root", default=COMPACT)
ap.add_argument("--arrays", type=int, default=5,
                help="how many smoothing lengths to sweep (0.02 s each at 50 Hz)")
ap.add_argument("--speed", type=float, default=None,
                help="if given, report the fraction of samples needing more than the tilt limit")
a = ap.parse_args()

NS = [1, 3, 5, 11, 21, 51][:max(1, a.arrays)] if a.arrays <= 6 else [1, 3, 5, 11, 21, 51]
for tag in a.tags:
    p = find(tag, a.root)
    if not p:
        print("%-14s NOT FOUND" % tag)
        continue
    t, c, f = load(p)
    t = t - t[0]
    print("\n=== %s   %d ticks, %.1f s ===" % (tag, len(t), t[-1]))
    head = "  N  %6s %6s %6s %6s %8s %8s" % ("dt", "R p10", "R p25", "R p50", "v_ceil", "k p50")
    print(" COMMANDED path")
    print(head)
    for n in NS:
        k, sp = kappa_of(t, c, n)
        ok = np.isfinite(k) & (sp > 0.2)
        if ok.sum() < 30:
            continue
        R = 1.0 / k[ok]
        vc = np.sqrt(A_TILT * np.percentile(R, 10))
        print("  %2d %6.2f %6.2f %6.2f %6.2f %8.2f %8.3f"
              % (n, n * 0.02, np.percentile(R, 10), np.percentile(R, 25),
                 np.percentile(R, 50), vc, np.median(k[ok])))
    print(" FLOWN path")
    print(head)
    for n in NS:
        k, sp = kappa_of(t, f, n)
        ok = np.isfinite(k) & (sp > 0.2)
        if ok.sum() < 30:
            continue
        R = 1.0 / k[ok]
        vc = np.sqrt(A_TILT * np.percentile(R, 10))
        print("  %2d %6.2f %6.2f %6.2f %6.2f %8.2f %8.3f"
              % (n, n * 0.02, np.percentile(R, 10), np.percentile(R, 25),
                 np.percentile(R, 50), vc, np.median(k[ok])))
    if a.speed:
        k, sp = kappa_of(t, c, 11)
        ok = np.isfinite(k) & (sp > 0.2)
        dem = a.speed ** 2 * k[ok]
        print(" at v=%.2f m/s: lateral demand p50 %.1f p90 %.1f max %.1f m/s^2; "
              "over %.1f m/s^2: %.1f %%"
              % (a.speed, np.median(dem), np.percentile(dem, 90), dem.max(), A_TILT,
                 100.0 * np.mean(dem > A_TILT)))
