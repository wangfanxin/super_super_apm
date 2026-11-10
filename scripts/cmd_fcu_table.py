#!/usr/bin/env python3
"""cmd / FCU-estimate / LIO table from `setpoints.csv` (all three in W, one row per 50 Hz tick).

The question this answers: at the moment a sortie dies, is the aircraft ahead of what it was told,
or is the target itself outside where it should be?  `setpoints.csv` carries all three streams on
the same clock, so no cross-file alignment is needed:

    cmd_*  the thin layer's published reference (W)
    fcu_*  the FCU's own EKF estimate (W, from MAVLink LOCAL_POSITION_NED)
    lio_*  the estimator's own pose (W, same as the FCU feed, logged separately)

Usage:  python3 scripts/cmd_fcu_table.py <sortie-dir-or-logs/tag> [--t0 20] [--t1 33] [--step 0.5]
"""
import argparse
import csv
import os
import sys

import numpy as np


def find(root):
    """accept either a harness run dir or a staged logs/<tag> dir"""
    for cand in (os.path.join(root, "setpoints.csv"), os.path.join(root, "raw", "setpoints.csv")):
        if os.path.exists(cand):
            return cand
    sys.exit("no setpoints.csv under %s" % root)


ap = argparse.ArgumentParser()
ap.add_argument("root")
ap.add_argument("--t0", type=float, default=0.0)
ap.add_argument("--t1", type=float, default=1e9)
ap.add_argument("--step", type=float, default=0.5)
a = ap.parse_args()

p = find(a.root)
rows = list(csv.DictReader(open(p)))
K = ("cmd_x", "cmd_y", "cmd_z")
V = ("vel_x", "vel_y", "vel_z")
F = ("fcu_x", "fcu_y", "fcu_z")
L = ("lio_x", "lio_y", "lio_z")
for k in K + V + F + L:
    assert k in rows[0], (k, list(rows[0].keys()))

t = np.array([float(r["t_wall"]) for r in rows])
t -= t[0]
cmd = np.array([[float(r[k]) for k in K] for r in rows])
vel = np.array([[float(r[k]) for k in V] for r in rows])
fcu = np.array([[float(r[k]) for k in F] for r in rows])
lio = np.array([[float(r[k]) for k in L] for r in rows])

spd = np.linalg.norm(vel[:, :2], axis=1)
d_fcu = np.linalg.norm((fcu - cmd)[:, :2], axis=1)
d_lio = np.linalg.norm((lio - cmd)[:, :2], axis=1)
# component of (fcu - cmd) along the commanded velocity: >0 means AHEAD of the target
u = vel[:, :2] / np.maximum(spd, 1e-6)[:, None]
lead = np.sum((fcu - cmd)[:, :2] * u, axis=1)
lead[spd < 0.1] = np.nan

print("rows=%d  span %.2f s" % (len(t), t[-1]))
print("cmd  x[%6.2f,%6.2f] y[%6.2f,%6.2f] z[%5.2f,%5.2f]"
      % (cmd[:, 0].min(), cmd[:, 0].max(), cmd[:, 1].min(), cmd[:, 1].max(),
         cmd[:, 2].min(), cmd[:, 2].max()))
print("fcu  x[%6.2f,%6.2f] y[%6.2f,%6.2f] z[%5.2f,%5.2f]"
      % (fcu[:, 0].min(), fcu[:, 0].max(), fcu[:, 1].min(), fcu[:, 1].max(),
         fcu[:, 2].min(), fcu[:, 2].max()))
ok = spd > 0.1
print("|fcu-cmd|xy  p50 %.2f p90 %.2f max %.2f      |lio-cmd|xy p50 %.2f p90 %.2f"
      % (np.percentile(d_fcu, 50), np.percentile(d_fcu, 90), d_fcu.max(),
         np.percentile(d_lio, 50), np.percentile(d_lio, 90)))
print("lead (fcu ahead of cmd along v)  p50 %+.2f p90 %+.2f max %+.2f m over %d moving ticks"
      % (np.nanpercentile(lead, 50), np.nanpercentile(lead, 90), np.nanmax(lead), ok.sum()))
print("cmd speed  p50 %.2f p90 %.2f max %.2f m/s"
      % (np.percentile(spd[ok], 50), np.percentile(spd[ok], 90), spd[ok].max()))

print("\n    t   cmd(x,y,z)            fcu(x,y,z)            lio(x,y,z)          d_fcu  lead  |v|")
tt = a.t0
while tt <= min(a.t1, t[-1]):
    i = int(np.searchsorted(t, tt).clip(0, len(t) - 1))
    print("%6.2f (%6.2f,%6.2f,%5.2f) (%6.2f,%6.2f,%5.2f) (%6.2f,%6.2f,%5.2f) %5.2f %+5.2f %4.2f"
          % (tt, cmd[i, 0], cmd[i, 1], cmd[i, 2], fcu[i, 0], fcu[i, 1], fcu[i, 2],
             lio[i, 0], lio[i, 1], lio[i, 2], d_fcu[i],
             lead[i] if not np.isnan(lead[i]) else 0.0, spd[i]))
    tt += a.step
