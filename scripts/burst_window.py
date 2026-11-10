#!/usr/bin/env python3
"""burst_window.py -- print one window around a truth-speed burst, tick by tick.

Setpoints are 50 Hz, so a burst that lasts 0.5 s is 25 rows: the ordering of
"published v -> command position -> truth speed" inside that window says whether the aircraft is
following a moving reference (and overshooting it) or chasing a stalled one.

    python3 scripts/burst_window.py --tag ke1-01-01 --t 78.0 --span 1.2
"""
import argparse
import csv
import os

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--tag", default="ke1-01-01")
ap.add_argument("--root", default="/home/fanxin/lio_avoid/logs")
ap.add_argument("--t", type=float, default=78.0)
ap.add_argument("--span", type=float, default=1.2)
a = ap.parse_args()
R = os.path.join(a.root, "missionA-" + a.tag)

sp = list(csv.DictReader(open(os.path.join(R, "setpoints.csv"))))
t = np.array([float(r["t_wall"]) for r in sp]); t -= t[0]
cmd = np.array([[float(r["cmd_x"]), float(r["cmd_y"])] for r in sp])
vel = np.array([[float(r["vel_x"]), float(r["vel_y"])] for r in sp])
acc = np.array([[float(r["acc_x"]), float(r["acc_y"])] for r in sp])
fcu = np.array([[float(r["fcu_x"]), float(r["fcu_y"])] for r in sp])

tap = list(csv.DictReader(open(os.path.join(R, "tap.csv"))))
tt = np.array([float(r["t_us"]) * 1e-6 for r in tap]); tt -= tt[0]
tp = np.array([[float(r["pe"]), float(r["pn"])] for r in tap])
best = None
for sh in np.arange(20.0, 40.0, 0.02):
    q = np.stack([np.interp(t + sh, tt, tp[:, 0]), np.interp(t + sh, tt, tp[:, 1])], 1)
    d = np.median(np.linalg.norm(q - cmd, axis=1))
    if best is None or d < best[0]:
        best = (d, float(sh))
sh = best[1]
k = max(2, int(0.15 / np.median(np.diff(tt))))
sm = np.stack([np.convolve(tp[:, 0], np.ones(k) / k, "same"),
               np.convolve(tp[:, 1], np.ones(k) / k, "same")], 1)
q = np.stack([np.interp(t + sh, tt, sm[:, 0]), np.interp(t + sh, tt, sm[:, 1])], 1)
vtr = np.stack([np.interp(t + sh, tt, np.gradient(sm[:, 0], tt)),
                np.interp(t + sh, tt, np.gradient(sm[:, 1], tt))], 1)

dt = np.median(np.diff(t))
vstep = np.r_[0.0, np.linalg.norm(np.diff(cmd, axis=0), axis=1) / dt]
print("== %s  t=%.2f..%.2f s (shift %+.2f s)" % (a.tag, a.t, a.t + a.span, sh))
print("    t     pub_v  step_v  truth_v   |e|   ref_ax  ref_ay   fcu-truth")
i0 = int(np.searchsorted(t, a.t))
i1 = int(np.searchsorted(t, a.t + a.span))
for i in range(i0, i1):
    print("  %6.2f   %5.2f   %5.2f   %5.2f   %5.2f  %+6.2f  %+6.2f   %5.2f"
          % (t[i], float(np.linalg.norm(vel[i])), vstep[i], float(np.linalg.norm(vtr[i])),
             float(np.linalg.norm(q[i] - cmd[i])), acc[i, 0], acc[i, 1],
             float(np.linalg.norm(fcu[i] - q[i]))))
