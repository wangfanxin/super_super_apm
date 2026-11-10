#!/usr/bin/env python3
"""seam_direction.py -- does a REPLAN ever hand us a trajectory that points BACKWARD?

The owner's shape ("冲过头 → 回到之前规划的点 → 再继续") needs the reference to move backwards at
some point.  `backtrack.py` shows the published reference's ARC LENGTH never decreases (forward 128.8 m,
backward 0.0 m) -- but arc length is measured along the reference's own direction, so a 180 deg flip
would be scored as "forward".  The thing to test is the DIRECTION at the replan seams: every
`trajectory_id` change in `wire.csv` is a new trajectory being installed, and if the new one starts by
heading back the way we came, the vehicle will turn round.

    python3 scripts/seam_direction.py --tag kn3-01-01
"""
import argparse
import csv
import math
import os

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--tag", default="kn3-01-01")
ap.add_argument("--root", default="/home/fanxin/lio_avoid/logs")
a = ap.parse_args()
R = os.path.join(a.root, "missionA-" + a.tag)

w = list(csv.DictReader(open(os.path.join(R, "wire.csv"))))
t = np.array([float(r["t_wall"]) for r in w]); t -= t[0]
vx = np.array([float(r["v_l_x"]) for r in w])
vy = np.array([float(r["v_l_y"]) for r in w])
tid = np.array([int(r["trajectory_id"]) for r in w])
cmd = np.stack([np.array([float(r["p_l_x"]) for r in w]),
                np.array([float(r["p_l_y"]) for r in w])], 1)

seams = np.flatnonzero(np.diff(tid) != 0) + 1
print("== %s: %d wire samples, %d trajectory installs" % (a.tag, len(w), len(seams)))
print("\n   t      dtheta   |v| before -> after   plan step over the next 0.5 s")
rows = []
for k in seams:
    i0, i1 = max(k - 5, 0), min(k + 5, len(w) - 1)
    u0 = np.array([vx[i0], vy[i0]])
    u1 = np.array([vx[i1], vy[i1]])
    n0, n1 = np.linalg.norm(u0), np.linalg.norm(u1)
    if n0 < 0.1 or n1 < 0.1:
        continue
    u0 /= n0
    u1 /= n1
    dth = math.degrees(math.acos(max(-1.0, min(1.0, float(u0 @ u1)))))
    j = min(k + 25, len(w) - 1)          # 0.5 s later
    step = cmd[j] - cmd[k]
    ns = np.linalg.norm(step)
    # does the plan move backwards relative to the pre-seam direction?
    back = float(step @ u0) / max(ns, 1e-9) if ns > 1e-6 else 0.0
    rows.append((t[k], dth, n0, n1, ns, back))
    print("  %6.2f   %5.1f deg   %.2f -> %.2f m/s      %.2f m, cos(pre-dir) %+.2f"
          % (t[k], dth, n0, n1, ns, back))

if rows:
    d = np.array([r[1] for r in rows])
    b = np.array([r[5] for r in rows])
    print("\n   direction change at seams: p50 %.0f  p90 %.0f  max %.0f deg"
          % (np.percentile(d, 50), np.percentile(d, 90), d.max()))
    print("   seams where the next 0.5 s moves BACKWARD vs the pre-seam direction: %d of %d"
          % (int((b < -0.3).sum()), len(rows)))
    print("   -> a reversal here is what makes the vehicle turn round: the reference really does")
    print("      go back, then continue.  Arc length cannot see it; direction can.")
