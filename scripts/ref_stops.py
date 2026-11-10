#!/usr/bin/env python3
"""ref_stops.py -- does the PUBLISHED reference come to a stop at the waypoint handovers?

SPEC S1 forbids it ("不允许在航点停住/驻停：参考速度在航点处归零 ... 都算违反本条"), and the flight
record shows why it matters: every time the reference collapses to ~0 the aircraft coasts past it and
then over-accelerates to catch up (measured on ke1-01-01: reference 0.00 m/s at t=65.6, aircraft then
0.29 -> 2.29 -> 2.79 m/s inside 1.2 s, along-track error swinging -0.93 -> +0.42 m).

This counts the near-zero episodes of the published speed and locates them relative to the six corners.

    python3 scripts/ref_stops.py --tag ke1-01-01
"""
import argparse
import csv
import os

import numpy as np
import yaml

ap = argparse.ArgumentParser()
ap.add_argument("--tag", default="ke1-01-01")
ap.add_argument("--root", default="/home/fanxin/lio_avoid/logs")
ap.add_argument("--stop-v", type=float, default=0.20)
ap.add_argument("--world", default="worlds/forest_hex12.yaml")
a = ap.parse_args()

R = os.path.join(a.root, "missionA-" + a.tag)
sp = list(csv.DictReader(open(os.path.join(R, "setpoints.csv"))))
t = np.array([float(r["t_wall"]) for r in sp]); t -= t[0]
cmd = np.array([[float(r["cmd_x"]), float(r["cmd_y"])] for r in sp])
vel = np.array([[float(r["vel_x"]), float(r["vel_y"])] for r in sp])
ref = np.linalg.norm(vel, axis=1)
w = yaml.safe_load(open(a.world, encoding="utf-8"))
ring = np.array([[float(c[0]), float(c[1])] for c in w["mission"]["corners"]][:6])

stop = ref < a.stop_v
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
durs = np.array([t[b] - t[a0] for a0, b in segs]) if segs else np.zeros(1)
print("== %s: published reference below %.2f m/s for %d episodes, total %.1f s (%.1f %% of %.1f s)"
      % (a.tag, a.stop_v, len(segs), durs.sum(), 100.0 * durs.sum() / t[-1], t[-1]))
if segs:
    print("   duration p50 %.2f s, p90 %.2f s, longest %.2f s"
          % (np.percentile(durs, 50), np.percentile(durs, 90), durs.max()))
    near = 0
    for (a0, b) in segs:
        dmin = np.min(np.linalg.norm(cmd[a0:b + 1][:, None, :] - ring[None, :, :], axis=2))
        if dmin < 1.5:
            near += 1
    print("   episodes whose stop point is within 1.5 m of a hexagon corner: %d of %d"
          % (near, len(segs)))
    print("   first ten (t_start, duration, distance to nearest corner):")
    for (a0, b) in segs[:10]:
        dmin = float(np.min(np.linalg.norm(cmd[a0:b + 1][:, None, :] - ring[None, :, :], axis=2)))
        print("      t=%6.2f  %.2f s  %.2f m" % (t[a0], t[b] - t[a0], dmin))
print("\n   -> SPEC S1 says the reference must NOT stop at a waypoint.  If these episodes sit on the")
print("      corners, the corner handover (--corner-speed / --corner-mode) is the thing to fix, not")
print("      the FCU's gains: the aircraft is being told to stop and restart 12 times a lap.")
