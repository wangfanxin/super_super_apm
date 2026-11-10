#!/usr/bin/env python3
"""Where does the speed actually go?  A mean speed is not actionable on its own; what matters is
whether the slow parts are at the CORNERS (geometry: a 15 m square with 90 deg corners) or mid-leg
(obstacles, handover hiccups), because only the second one is fixable in software.

  speed_audit.py RUN [--h 7.5] [--alt 2.0]
"""
import argparse
import os

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("run")
ap.add_argument("--h", type=float, default=7.5, help="mission square half extent")
ap.add_argument("--alt", type=float, default=2.0)
ap.add_argument("--vmax", type=float, default=5.0, help="the plan's own ceiling (max_vel)")
ap.add_argument("--amax", type=float, default=4.5, help="the plan's accel ceiling (max_acc)")
a = ap.parse_args()

tp = os.path.join(a.run, "tap.csv")
hdr = open(tp, encoding="utf-8").readline().strip().split(",")
d = np.loadtxt(tp, delimiter=",", skiprows=1, ndmin=2)
c = {n: i for i, n in enumerate(hdr)}
t = d[:, c["t_us"]] * 1e-6
E, N, U = d[:, c["pe"]], d[:, c["pn"]], -d[:, c["pd"]]
v = np.hypot(d[:, c["ve"]], d[:, c["vn"]])
m = U > 0.5 * a.alt
i0, i1 = int(np.argmax(m)), int(len(m) - 1 - np.argmax(m[::-1]))
t, E, N, v = t[i0:i1 + 1], E[i0:i1 + 1], N[i0:i1 + 1], v[i0:i1 + 1]

print("route: %.1f s, %d samples" % (t[-1] - t[0], len(t)))
print("speed: mean %.2f  median %.2f  p90 %.2f  p95 %.2f  max %.2f m/s"
      % (v.mean(), np.median(v), np.percentile(v, 90), np.percentile(v, 95), v.max()))
print("mean over the route = path length / time = %.1f m / %.1f s = %.2f m/s"
      % (float(np.sum(np.hypot(np.diff(E), np.diff(N)))), t[-1] - t[0], v.mean()))
print()
for lo, hi in ((0, 0.5), (0.5, 1.5), (1.5, 2.5), (2.5, 3.5), (3.5, 4.5), (4.5, 99)):
    dtm = np.diff(t)
    frac = float(np.sum(dtm * ((v[:-1] >= lo) & (v[:-1] < hi)))) / (t[-1] - t[0])
    print("  speed %4.1f-%-4.1f m/s : %5.1f %% of the route" % (lo, hi, 100 * frac))

# distance to the nearest of the 4 corners, per sample
corners = np.array([(-a.h, -a.h), (a.h, -a.h), (a.h, a.h), (-a.h, a.h)], float)
dc = np.min(np.hypot(E[:, None] - corners[None, :, 0], N[:, None] - corners[None, :, 1]), axis=1)
print()
print("split of the route by distance to the nearest corner:")
for lo, hi in ((0, 2), (2, 5), (5, 99)):
    sel = (dc >= lo) & (dc < hi)
    if sel.sum() > 10:
        dtm = np.diff(t)
        s2 = sel[:-1]
        frac = float(np.sum(dtm * s2)) / (t[-1] - t[0])
        print("  %4.0f-%-3.0f m from a corner : %5.1f %% of the route, mean speed there %.2f m/s"
              % (lo, hi, 100 * frac, float(v[sel].mean())))
print()
print("what a 15 x 15 m square ALLOWS (geometry, not tuning):")
L, amax, vmax = 2 * a.h, a.amax, a.vmax
for R in (0.5, 1.0, 2.0, 3.0, 5.0, 7.5):
    varc = min(vmax, np.sqrt(amax * R))
    length = 4 * (L - 2 * R) + 2 * np.pi * R
    tim = 4 * (L - 2 * R) / vmax + 2 * np.pi * R / varc
    print("  corner radius %4.1f m -> arc speed %.2f m/s, lap length %5.1f m, lap %.1f s, "
          "average %.2f m/s" % (R, varc, length, tim, length / tim))
print()
print("(the straights can be at vmax; ONLY the corners force the average down, and a corner radius")
print(" of R needs centripetal accel v^2/R <= max_acc.  Raising max_acc is the only way to hold")
print(" 5 m/s through a 90 deg corner, and the required radius is v^2/a.)")
for vv in (3.0, 4.0, 5.0):
    print("  to hold %.1f m/s through a corner you need radius %.1f m at max_acc %.1f"
          % (vv, vv * vv / amax, amax))
