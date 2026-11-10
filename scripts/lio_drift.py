#!/usr/bin/env python3
"""lio_drift.py -- does the estimator drift away from the SITL truth, and where does it start?

`setpoints.csv` carries three streams on the shared simulated clock:
    lio_x/lio_y  the estimator's pose, in L   (L = (y_w, -x_w)  ==>  W = (-y_l, x_l))
    fcu_x/fcu_y  the FCU's own EKF estimate, in W
    cmd_x/cmd_y  the published reference, in W
and `tap.csv` carries the SITL model's truth (pe, pn, -pd  ==>  W).

The estimator drifts → the EKF follows it (it is the only nav source) → the aircraft flies to a
place the map calls home.  So the interesting quantity is `lio(W) - truth(W)` over time, and whether
it moves in JUMPS (a scan-matching latch onto the wrong lattice cell: this field is a 1 m lattice of
0.2 m posts, the classic aliasing geometry) or smoothly (a scale/IMU problem).

    python3 scripts/lio_drift.py hz2-01-01 [--step 5]
"""
import argparse
import csv
import os

import numpy as np

MISSION = "/home/fanxin/lio_avoid/logs"


def sp(tag):
    p = os.path.join(MISSION, "missionA-" + tag, "setpoints.csv")
    t, cmd, fcu, lio = [], [], [], []
    for r in csv.DictReader(open(p)):
        t.append(float(r["t_sim_us"]) * 1e-6)
        cmd.append((float(r["cmd_x"]), float(r["cmd_y"])))
        fcu.append((float(r["fcu_x"]), float(r["fcu_y"])))
        lio.append((-float(r["lio_y"]), float(r["lio_x"])))     # L -> W
    return np.array(t), np.array(cmd), np.array(fcu), np.array(lio)


def truth(tag):
    p = os.path.join(MISSION, "missionA-" + tag, "tap.csv")
    t, pxy = [], []
    for r in csv.DictReader(open(p)):
        t.append(float(r["t_us"]) * 1e-6)
        pxy.append((float(r["pe"]), float(r["pn"])))
    return np.array(t), np.array(pxy)


ap = argparse.ArgumentParser()
ap.add_argument("tag")
ap.add_argument("--step", type=float, default=5.0)
a = ap.parse_args()

t, cmd, fcu, lio = sp(a.tag)
tt, tr = truth(a.tag)
# the tap's first samples can be in a different frame (before the EKF origin is set): drop them by
# requiring the truth to be within 100 m of the pad at the start of the window
m0 = (tt >= t[0]) & (tt <= t[0] + 30.0)
if m0.any():
    keep = tt >= tt[m0][np.argmax(np.linalg.norm(tr[m0], axis=1) < 100.0)]
    tt, tr = tt[keep], tr[keep]

print("%s: setpoints %.1f..%.1f s, tap %.1f..%.1f s" % (a.tag, t[0], t[-1], tt[0], tt[-1]))
print("\n    t    truth(x,y)          lio(W)             fcu(W)          |lio-truth|  |fcu-truth|")
x = t[0]
while x <= t[-1]:
    i = int(np.searchsorted(t, x).clip(0, len(t) - 1))
    j = int(np.searchsorted(tt, x).clip(0, len(tt) - 1))
    if abs(tt[j] - x) < 0.5:
        d_l = float(np.linalg.norm(lio[i] - tr[j]))
        d_f = float(np.linalg.norm(fcu[i] - tr[j]))
        print("%6.1f  (%7.2f,%7.2f)  (%7.2f,%7.2f)  (%7.2f,%7.2f)  %8.2f  %8.2f"
              % (x - t[0], tr[j, 0], tr[j, 1], lio[i, 0], lio[i, 1], fcu[i, 0], fcu[i, 1], d_l, d_f))
    x += a.step

# jump census on the estimator: a >0.25 m step between consecutive 50 Hz ticks (1.5 m/s = 0.03 m)
dl = np.linalg.norm(np.diff(lio, axis=0), axis=1)
big = np.flatnonzero(dl > 0.25)
print("\n estimator steps > 0.25 m between ticks (5 m/s at 50 Hz): %d" % len(big))
for k in big[:12]:
    print("   t=%+8.2f  |step| = %.2f m  lio (%7.2f,%7.2f) -> (%7.2f,%7.2f)"
          % (t[k] - t[0], dl[k], lio[k][0], lio[k][1], lio[k + 1][0], lio[k + 1][1]))
