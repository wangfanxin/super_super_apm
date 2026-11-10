#!/usr/bin/env python3
"""ref_pulse.py -- is the published reference SMOOTH, or does it stop and go?

The owner's complaint (round 88): "跟踪的不好，没有贴紧 path".  `tracking_diag` splits the residual into
along-track and lateral, and on the delivered sortie the ALONG-track term dominates
(p90 0.611 m vs lateral 0.344 m).  An along-track error of that size is not a steering problem --
it is what a reference that STOPS and RESTARTS does to a vehicle with a ~0.4 s position loop: the
aircraft coasts past the stopped point, then has to come back.

This script measures the reference itself, per tick, from `setpoints.csv`:

    plan speed      |d(cmd)/dt|            what the plan wants
    published speed |(vel_x, vel_y)|       what the thin layer tells the FCU to fly
    dip             published speed < dip_frac * plan speed AND plan speed > 0.30

and reports the dips, their duration, and what the aircraft's along-track error does around them.

    python3 scripts/ref_pulse.py --tag jd1-01-02 [--dip-frac 0.35]
"""
import argparse
import csv
import os

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--tag", default="jd1-01-02")
ap.add_argument("--root", default="/home/fanxin/lio_avoid/logs")
ap.add_argument("--dip-frac", type=float, default=0.35)
ap.add_argument("--min-plan", type=float, default=0.30)
a = ap.parse_args()

p = os.path.join(a.root, "missionA-" + a.tag, "setpoints.csv")
rows = list(csv.DictReader(open(p)))
t = np.array([float(r["t_wall"]) for r in rows]); t -= t[0]
cmd = np.array([[float(r["cmd_x"]), float(r["cmd_y"])] for r in rows])
vel = np.array([[float(r["vel_x"]), float(r["vel_y"])] for r in rows])
fcu = np.array([[float(r["fcu_x"]), float(r["fcu_y"])] for r in rows])

dt = np.median(np.diff(t))
plan_v = np.r_[0.0, np.linalg.norm(np.diff(cmd, axis=0), axis=1) / dt]
pub_v = np.linalg.norm(vel, axis=1)
moving = plan_v > a.min_plan
dip = moving & (pub_v < a.dip_frac * plan_v)

print("== %s: %d ticks at %.1f Hz, %.1f s" % (a.tag, len(t), 1.0 / dt, t[-1]))
print("   plan speed      p1 %.2f  p50 %.2f  p99 %.2f  max %.2f m/s"
      % (np.percentile(plan_v[moving], 1), np.median(plan_v[moving]),
         np.percentile(plan_v[moving], 99), plan_v.max()))
print("   published speed p1 %.2f  p50 %.2f  p99 %.2f  max %.2f m/s"
      % (np.percentile(pub_v[moving], 1), np.median(pub_v[moving]),
         np.percentile(pub_v[moving], 99), pub_v.max()))
print("   ticks with the plan moving: %d (%.0f %% of the flight)"
      % (int(moving.sum()), 100.0 * moving.mean()))

# dip episodes
segs = []
i = 0
n = len(t)
while i < n:
    if dip[i]:
        j = i
        while j + 1 < n and dip[j + 1]:
            j += 1
        segs.append((i, j))
        i = j + 1
    else:
        i += 1
durs = np.array([t[b] - t[a0] for a0, b in segs]) if segs else np.zeros(1)
print("\n   DIPS (published < %.2f x plan while the plan moves): %d episodes, p50 %.2f s, p90 %.2f s,"
      " longest %.2f s, total %.1f s"
      % (a.dip_frac, len(segs), np.percentile(durs, 50), np.percentile(durs, 90), durs.max(), durs.sum()))
print("   that is %.1f %% of the moving time spent telling the FCU to fly slower than the plan"
      % (100.0 * dip.sum() / max(1, moving.sum())))

# what the aircraft does around a dip: along-track overshoot right after the reference resumes
if segs:
    kmax = int(round(0.6 / dt))
    overs = []
    for (a0, b) in segs[:400]:
        if b + 8 >= n:
            continue
        u = vel[b] / max(np.linalg.norm(vel[b]), 1e-9)
        e_after = float((fcu[min(b + kmax, n - 1)] - cmd[min(b + kmax, n - 1)]) @ u)
        overs.append(e_after)
    if overs:
        overs = np.array(overs)
        print("   along-track error 0.6 s after a dip ends: p50 %+.2f  p90(|e|) %.2f m"
              "  positive = the aircraft is AHEAD of the plan (it kept flying while the reference stopped)"
              % (np.median(overs), np.percentile(np.abs(overs), 90)))
print("\n   -> if the dips are many and the overshoot is positive, the fix is to make the published"
      " speed follow the PLAN's speed smoothly (arc-length synchronisation), not to retune gains.")
