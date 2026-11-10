#!/usr/bin/env python3
"""midleg_stops.py -- the reversals the corner story does NOT explain: the reference stopping MID-LEG.

`ref_stops.py` already showed that some reference-zero episodes sit 4-5 m away from any corner, and
`backtrack.py` finds truth reversals at (7.6, 4.2), (0.7, -8.4), (0.6, 7.8) -- all between waypoints.
A corner-stop explanation cannot cover those, so this script isolates them:

  * published reference speed from `setpoints.csv` (50 Hz),
  * distance to the nearest hexagon corner at that moment,
  * a MID-LEG STOP = ref speed < v_stop AND distance to the nearest corner > d_min,
  * for each one, what happened next (did the truth overshoot? how far did it retrace?),
  * and a census of the thin layer's own log lines that could produce a stop
    (`Replan failed`, `switch to emer`, `PlanFromRest failed`, `[resend]`, `No odom`).

    python3 scripts/midleg_stops.py --tag kn3-01-01
"""
import argparse
import csv
import os
import re

import numpy as np
import yaml

ap = argparse.ArgumentParser()
ap.add_argument("--tag", default="kn3-01-01")
ap.add_argument("--root", default="/home/fanxin/lio_avoid/logs")
ap.add_argument("--v-stop", type=float, default=0.25)
ap.add_argument("--d-min", type=float, default=1.5)
ap.add_argument("--world", default="worlds/forest_hex12.yaml")
a = ap.parse_args()
R = os.path.join(a.root, "missionA-" + a.tag)

sp = list(csv.DictReader(open(os.path.join(R, "setpoints.csv"))))
t = np.array([float(r["t_wall"]) for r in sp]); t -= t[0]
cmd = np.stack([np.array([float(r["cmd_x"]) for r in sp]),
                np.array([float(r["cmd_y"]) for r in sp])], 1)
vel = np.stack([np.array([float(r["vel_x"]) for r in sp]),
                np.array([float(r["vel_y"]) for r in sp])], 1)
v = np.linalg.norm(vel, axis=1)

w = yaml.safe_load(open(a.world, encoding="utf-8"))
ring = np.array([[float(c[0]), float(c[1])] for c in w["mission"]["corners"]][:6])
dc = np.min(np.linalg.norm(cmd[:, None, :] - ring[None, :, :], axis=2), axis=1)
dpad = np.linalg.norm(cmd, axis=1)
near = np.minimum(dc, dpad)

stop = (v < a.v_stop) & (near > a.d_min)
segs = []
i, n = 0, len(t)
while i < n:
    if stop[i]:
        j = i
        while j + 1 < n and stop[j + 1]:
            j += 1
        segs.append((i, j))
        i = j + 1
    else:
        i += 1

print("== %s: %d ticks, reference < %.2f m/s while > %.1f m from any corner/pad: %d episodes"
      % (a.tag, n, a.v_stop, a.d_min, len(segs)))
if segs:
    dur = np.array([t[b] - t[a0] for a0, b in segs])
    print("   duration p50 %.2f s, p90 %.2f s, longest %.2f s, total %.1f s (%.1f %% of the flight)"
          % (np.percentile(dur, 50), np.percentile(dur, 90), dur.max(), dur.sum(),
             100.0 * dur.sum() / t[-1]))
    print("\n   t_start   dur   plan(x,y)        dist-to-corner   next 1.5 s: plan moves  truth speed")
    for (a0, b) in segs[:12]:
        j = min(b + int(1.5 / np.median(np.diff(t))), n - 1)
        dmove = float(np.linalg.norm(cmd[j] - cmd[b]))
        vt = float(np.median(v[b:j + 1]))
        print("   %6.2f   %.2f   (%5.1f,%5.1f)   %5.2f m          %.2f m      %.2f m/s"
              % (t[a0], t[b] - t[a0], cmd[a0, 0], cmd[a0, 1], near[a0], dmove, vt))

# census of the log lines that can produce a stop
log = open(os.path.join(R, "traj.log"), encoding="utf-8", errors="replace").read()
print("\n   thin-layer log census:")
for pat in ("Replan failed", "switch to emer", "PlanFromRest failed", r"\[resend\]",
            "No odom", "TOUR TRUNCATED", "opt failed", "Opt failed"):
    c = len(re.findall(pat, log))
    print("     %-22s %d" % (pat.replace("\\\\", ""), c))
print("\n   -> a mid-leg stop means SUPER stopped asking us to move (its own plan stalled or it went")
print("      to the emergency trajectory), or the plan box re-issued the goal.  The census above says")
print("      which of those is frequent enough to explain the count.")
