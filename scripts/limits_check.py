#!/usr/bin/env python3
"""limits_check.py -- does the PLAN ask for more than the SITL vehicle can actually produce?

The owner's question: "规划器现在规划用的这些飞行参数（v、a、a_dot 等），超过 SITL 里模型的上限了吗？"

Three different ceilings are usually confused:

  1. the PLANNER's own bounds      : `max_vel 1.5`, `max_acc 9`, `max_jerk 40` (config)
  2. the AIRFRAME's theoretical one: `ATC_ANGLE_MAX 45 deg` -> g*tan(45) = 9.81 m/s^2 horizontal,
                                     vertical from the thrust/weight ratio
  3. what the CLOSED LOOP actually delivers -- this is the number that decides whether the plan is
     executable, and it is measurable: differentiate the truth velocity and compare with the reference.

This script reports (3) next to (1) and (2).

    python3 scripts/limits_check.py --tag kh1-02-01
"""
import argparse
import csv
import os

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--tag", default="kh1-02-01")
ap.add_argument("--root", default="/home/fanxin/lio_avoid/logs")
a = ap.parse_args()
R = os.path.join(a.root, "missionA-" + a.tag)

sp = list(csv.DictReader(open(os.path.join(R, "setpoints.csv"))))
t = np.array([float(r["t_wall"]) for r in sp]); t -= t[0]
vx = np.array([float(r["vel_x"]) for r in sp])
vy = np.array([float(r["vel_y"]) for r in sp])
ax = np.array([float(r["acc_x"]) for r in sp])
ay = np.array([float(r["acc_y"]) for r in sp])
cmd = np.stack([np.array([float(r["cmd_x"]) for r in sp]),
                np.array([float(r["cmd_y"]) for r in sp])], 1)

tap = list(csv.DictReader(open(os.path.join(R, "tap.csv"))))
tt = np.array([float(r["t_us"]) * 1e-6 for r in tap]); tt -= tt[0]
tp = np.stack([np.array([float(r["pe"]) for r in tap]),
               np.array([float(r["pn"]) for r in tap])], 1)

best = None
for sh in np.arange(20.0, 40.0, 0.02):
    q = np.stack([np.interp(t + sh, tt, tp[:, 0]), np.interp(t + sh, tt, tp[:, 1])], 1)
    d = np.median(np.linalg.norm(q - cmd, axis=1))
    if best is None or d < best[0]:
        best = (d, float(sh))
sh = best[1]
k = max(2, int(0.10 / np.median(np.diff(tt))))
sm = np.stack([np.convolve(tp[:, 0], np.ones(k) / k, "same"),
               np.convolve(tp[:, 1], np.ones(k) / k, "same")], 1)
q = np.stack([np.interp(t + sh, tt, sm[:, 0]), np.interp(t + sh, tt, sm[:, 1])], 1)
vt = np.stack([np.interp(t + sh, tt, np.gradient(sm[:, 0], tt)),
               np.interp(t + sh, tt, np.gradient(sm[:, 1], tt))], 1)

# accelerations
a_ref = np.stack([ax, ay], 1)
a_ref_mag = np.linalg.norm(a_ref, axis=1)
# finite-difference acceleration of the (smoothed) truth velocity
a_tr = np.gradient(vt, axis=0) / np.median(np.diff(t))
a_tr_mag = np.linalg.norm(a_tr, axis=1)
v_ref_mag = np.linalg.norm(np.stack([vx, vy], 1), axis=1)
v_tr_mag = np.linalg.norm(vt, axis=1)


def show(name, x, unit="m/s^2"):
    print("   %-28s p50 %5.2f  p90 %5.2f  p99 %5.2f  max %6.2f %s"
          % (name, np.median(x), np.percentile(x, 90), np.percentile(x, 99), x.max(), unit))


print("== %s (clock %+.2f s)" % (a.tag, sh))
print("\n 1) THE PLAN (what the FCU is asked for)")
show("|v| horizontal (reference)", v_ref_mag, "m/s")
show("|a| horizontal (reference)", a_ref_mag)
print("\n 2) THE AIRFRAME THEORY (SITL params)")
print("   ATC_ANGLE_MAX 45 deg        -> horizontal accel ceiling g*tan(45) = 9.81 m/s^2")
print("   the planner is configured to max_acc 9.0 / max_jerk 40 / max_vel 1.5")
print("\n 3) WHAT THE CLOSED LOOP ACTUALLY DELIVERS (truth, differentiated)")
show("|v| horizontal (truth)", v_tr_mag, "m/s")
show("|a| horizontal (truth)", a_tr_mag)

print("\n   when the reference asks for a lot, what does the vehicle give?")
for lo, hi in ((0, 1), (1, 3), (3, 5), (5, 7), (7, 12)):
    m = (a_ref_mag >= lo) & (a_ref_mag < hi)
    if m.sum() < 10:
        continue
    print("     |a_ref| %2d-%2d m/s^2 : n=%5d  |a_truth| p50 %5.2f  p90 %5.2f  (ratio p50 %.2f)"
          % (lo, hi, int(m.sum()), np.median(a_tr_mag[m]), np.percentile(a_tr_mag[m], 90),
             np.median(a_tr_mag[m]) / max(np.median(a_ref_mag[m]), 1e-9)))

print("\n   -> if the p90/max of the truth sits well BELOW the reference's demand, the plan is asking")
print("      for more than this loop can produce; if the truth's max EXCEEDS the theory ceiling, the")
print("      differentiation is noisy and the p90 is the number to quote.")
