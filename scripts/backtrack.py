#!/usr/bin/env python3
"""backtrack.py -- find the "overshoot, come back to an earlier planned point, then continue" episodes.

The owner sees them at lap1 near (8, 4) and at c3.  Two things can produce that shape:

  (a) THE REFERENCE ITSELF MOVES BACKWARD.  The thin layer publishes SUPER's sampled point; if SUPER's
      goal is re-issued (the `[resend]` lines in the log) or the plan's start is shifted, the sampled
      point can step back along the path.
  (b) THE VEHICLE OVERSHOOTS and the position loop pulls it back to a point it already passed.

They are easy to separate: compare the SIGNED progress of the reference (along its own direction) with
the signed progress of the truth (along the same direction).  This prints, for the requested window,
the cumulative arc length of both, so a negative slope means "went backwards".

    python3 scripts/backtrack.py --tag kh1-02-01 --t0 40 --t1 110
    python3 scripts/backtrack.py --tag kh1-02-01 --find      (scan the whole lap for episodes)
"""
import argparse
import csv
import os

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--tag", default="kh1-02-01")
ap.add_argument("--root", default="/home/fanxin/lio_avoid/logs")
ap.add_argument("--t0", type=float, default=None)
ap.add_argument("--t1", type=float, default=None)
ap.add_argument("--find", action="store_true", help="report every backward episode instead of a window")
ap.add_argument("--back-m", type=float, default=0.15, help="retrace threshold, m")
ap.add_argument("--step", type=float, default=1.0)
a = ap.parse_args()
R = os.path.join(a.root, "missionA-" + a.tag)

sp = list(csv.DictReader(open(os.path.join(R, "setpoints.csv"))))
t = np.array([float(r["t_wall"]) for r in sp]); t -= t[0]
cmd = np.stack([np.array([float(r["cmd_x"]) for r in sp]),
                np.array([float(r["cmd_y"]) for r in sp])], 1)
vel = np.stack([np.array([float(r["vel_x"]) for r in sp]),
                np.array([float(r["vel_y"]) for r in sp])], 1)
tap = list(csv.DictReader(open(os.path.join(R, "tap.csv"))))
tt = np.array([float(r["t_us"]) * 1e-6 for r in tap]); tt -= tt[0]
tp = np.stack([np.array([float(r["pe"]) for r in tap]),
               np.array([float(r["pn"]) for r in tap])], 1)

# clock: same convention as tracking_diag (one shift, fitted on positions)
best = None
for sh in np.arange(20.0, 40.0, 0.02):
    q = np.stack([np.interp(t + sh, tt, tp[:, 0]), np.interp(t + sh, tt, tp[:, 1])], 1)
    d = np.median(np.linalg.norm(q - cmd, axis=1))
    if best is None or d < best[0]:
        best = (d, float(sh))
sh = best[1]
tr = np.stack([np.interp(t + sh, tt, tp[:, 0]), np.interp(t + sh, tt, tp[:, 1])], 1)

# signed progress: project each step onto the REFERENCE's current direction
u = vel / np.maximum(np.linalg.norm(vel, axis=1), 1e-9)[:, None]
d_cmd = np.r_[0.0, np.sum(np.diff(cmd, axis=0) * u[1:], axis=1)]
d_tr = np.r_[0.0, np.sum(np.diff(tr, axis=0) * u[1:], axis=1)]
s_cmd = np.cumsum(d_cmd)
s_tr = np.cumsum(d_tr)

print("== %s: clock %+.2f s" % (a.tag, sh))
print("   reference forward %.1f m, backward %.1f m | truth forward %.1f m, backward %.1f m"
      % (s_cmd.max(), -min(0.0, d_cmd[d_cmd < 0].sum()), s_tr.max(), -min(0.0, d_tr[d_tr < 0].sum())))

if a.find:
    # episodes: consecutive samples where the TRUTH moves backwards by more than --back-m in total
    bad = d_tr < 0
    i, n = 0, len(t)
    eps = []
    while i < n:
        if bad[i]:
            j = i
            acc = 0.0
            while j + 1 < n and bad[j + 1]:
                j += 1
                acc += d_tr[j]
            if -acc >= a.back_m:
                eps.append((i, j, -acc))
            i = j + 1
        else:
            i += 1
    print("\n   BACKWARD episodes of the truth (>= %.2f m retraced): %d" % (a.back_m, len(eps)))
    for (i, j, m) in eps[:20]:
        print("      t=%6.2f..%-6.2f (%.2f s) retraced %.2f m | plan at (%.1f,%.1f) truth (%.1f,%.1f)"
              % (t[i], t[j], t[j] - t[i], m, cmd[i, 0], cmd[i, 1], tr[i, 0], tr[i, 1]))
else:
    t0 = a.t0 if a.t0 is not None else t[0]
    t1 = a.t1 if a.t1 is not None else t[-1]
    print("\n     t   s_cmd   s_tr    d_cmd  d_tr   plan(x,y)   truth(x,y)")
    x = t0
    while x <= min(t1, t[-1]):
        i = int(np.searchsorted(t, x).clip(0, len(t) - 1))
        print("  %6.1f  %6.1f %6.1f  %+5.2f  %+5.2f  (%5.1f,%5.1f) (%5.1f,%5.1f)"
              % (x, s_cmd[i], s_tr[i], d_cmd[i], d_tr[i], cmd[i, 0], cmd[i, 1], tr[i, 0], tr[i, 1]))
        x += a.step
