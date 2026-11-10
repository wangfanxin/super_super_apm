#!/usr/bin/env python3
"""body_rate.py -- what body angular rate does the vehicle actually use, vs what the plan demands?

`jerk_measure.py` converts the reference's a_dot into a tilt-rate demand (omega = a_dot / g) and gets
p99 = 236 deg/s.  This measures the REAL body rate from the truth quaternion, so the comparison is
demand vs delivered -- no parameter defaults to argue about.

    python3 scripts/body_rate.py --tag kh1-02-01
"""
import argparse
import csv
import math
import os

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--tag", default="kh1-02-01")
ap.add_argument("--root", default="/home/fanxin/lio_avoid/logs")
a = ap.parse_args()
R = os.path.join(a.root, "missionA-" + a.tag)

rows = list(csv.DictReader(open(os.path.join(R, "tap.csv"))))
t = np.array([float(r["t_us"]) * 1e-6 for r in rows]); t -= t[0]
q = np.stack([np.array([float(r[k]) for r in rows]) for k in ("qw", "qx", "qy", "qz")], 1)
q /= np.linalg.norm(q, axis=1, keepdims=True)

# body rate from consecutive quaternions:  dq = q_{k+1} * conj(q_k) = (1, w*dt/2)
dt = np.gradient(t)
w = np.zeros((len(q) - 1, 3))
for i in range(len(q) - 1):
    q0, q1 = q[i], q[i + 1]
    conj = np.array([q0[0], -q0[1], -q0[2], -q0[3]])
    a0, b0, c0, d0 = q1
    a1, b1, c1, d1 = conj
    dq = np.array([a0 * a1 - b0 * b1 - c0 * c1 - d0 * d1,
                   a0 * b1 + b0 * a1 + c0 * d1 - d0 * c1,
                   a0 * c1 - b0 * d1 + c0 * a1 + d0 * b1,
                   a0 * d1 + b0 * c1 - c0 * b1 + d0 * a1])
    if dq[0] < 0:
        dq = -dq
    w[i] = 2.0 * dq[1:4] / max(dt[i], 1e-9)
wn = np.linalg.norm(w, axis=1)

# the demand, from the same flight
wire = list(csv.DictReader(open(os.path.join(R, "wire.csv"))))
ax = np.array([float(r["a_l_x"]) for r in wire])
ay = np.array([float(r["a_l_y"]) for r in wire])
tw = np.array([float(r["t_wall"]) for r in wire]); tw -= tw[0]
jer = np.linalg.norm(np.stack([np.gradient(ax, tw), np.gradient(ay, tw)], 1), axis=1)
demand = jer / 9.81

print("== %s: %d attitude samples at %.0f Hz, %d wire samples at %.0f Hz"
      % (a.tag, len(t), 1 / np.median(np.diff(t)), len(tw), 1 / np.median(np.diff(tw))))
print("   DEMAND  tilt rate = a_dot/g : p50 %5.1f  p90 %5.1f  p99 %5.1f  max %5.1f deg/s"
      % tuple(np.degrees(np.percentile(demand, q)) for q in (50, 90, 99, 100)))
print("   ACTUAL body |omega|          : p50 %5.1f  p90 %5.1f  p99 %5.1f  max %5.1f deg/s"
      % tuple(math.degrees(np.percentile(wn, q)) for q in (50, 90, 99, 100)))
print()
print("   -> the plan demands up to %.0f deg/s of tilt-rate; the airframe actually reaches %.0f deg/s."
      % (math.degrees(demand.max()), math.degrees(wn.max())))
print("      Any a_dot beyond g*%.0f = %.0f m/s^3 cannot be produced at all." %
      (wn.max(), 9.81 * wn.max()))
