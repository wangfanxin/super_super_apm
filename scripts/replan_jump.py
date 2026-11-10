#!/usr/bin/env python3
"""replan_jump.py -- is the reference velocity step caused by SUPER REPLACING the trajectory?

SUPER's control path, read from `upstream/SUPER/super_planner`:

  * `fsm/replan_rate` (10 Hz) -> `replan_timer_` -> `Fsm::callReplanOnce()` -> `SuperPlanner::ReplanOnce()`:
    A*/CIRI + MINCO; on `NEW_TRAJ` it calls `publishPolyTraj()` and installs a **new trajectory**;
  * `cmd_timer_` at `cmd_rate` (100 Hz) -> `getOnePositionCommand()` -> `getOneCommandFromTraj()`:
    **sample the current trajectory at the current time**.  So the published (p, v, a) is a 100 Hz
    sampling of a trajectory that is swapped 10 times a second;
  * `hot_start_en` reuses the previous spline's leading part so the new one starts on it -- that is the
    mechanism that is supposed to hide the swap.

Whether it succeeds is measurable: every `PositionCommand` carries `trajectory_id`, and the bridge logs
it (`wire.csv`).  So the question "are the velocity steps at the replan seams?" has a direct test:

    compare |dv| between consecutive samples when trajectory_id CHANGED vs when it did not.

    python3 scripts/replan_jump.py --tag kh1-02-01
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

wire = list(csv.DictReader(open(os.path.join(R, "wire.csv"))))
t = np.array([float(r["t_wall"]) for r in wire]); t -= t[0]
vx = np.array([float(r["v_l_x"]) for r in wire])
vy = np.array([float(r["v_l_y"]) for r in wire])
tid = np.array([int(r["trajectory_id"]) for r in wire])
flag = np.array([int(r["trajectory_flag"]) for r in wire])
dt = np.median(np.diff(t))

dv = np.linalg.norm(np.stack([np.diff(vx), np.diff(vy)], 1), axis=1) / dt
changed = (np.diff(tid) != 0) | (np.diff(flag) != 0)

print("== %s: %d wire samples at %.0f Hz, %.1f s" % (a.tag, len(t), 1 / dt, t[-1]))
print("   trajectory_id values: %s" % sorted(set(tid.tolist()))[:12])
print("   replan seams (id/flag changed): %d (%.1f %% of samples)"
      % (int(changed.sum()), 100.0 * changed.mean()))

for label, m in (("AT a replan seam", changed), ("NOT at a seam", ~changed)):
    if m.sum() < 5:
        continue
    x = dv[m]
    print("   %-16s n=%5d  |dv|/dt p50 %.2f  p90 %.2f  p99 %.2f  max %.2f m/s^2"
          % (label, int(m.sum()), np.median(x), np.percentile(x, 90), np.percentile(x, 99), x.max()))

if changed.sum() >= 5:
    r = np.median(dv[changed]) / max(np.median(dv[~changed]), 1e-9)
    print("\n   median |dv|/dt at seams is %.1fx the value elsewhere" % r)
    print("   -> %s" % ("the replan seams ARE a step source" if r > 1.5 else
                        "the replan seams are NOT the step source (the steps are elsewhere)"))

# periodicity: band power of v_x around 2 / 5 / 10 Hz
f = np.fft.rfftfreq(len(vx), d=dt)
P = np.abs(np.fft.rfft(vx - vx.mean())) ** 2
band = (f > 0.2) & (f < 24.0)
tot = P[band].sum()
print("\n   v_x band power (fraction of 0.2-24 Hz):")
for f0 in (0.5, 1.0, 2.0, 5.0, 10.0):
    m = np.abs(f - f0) <= 0.4
    print("      %5.1f Hz +-0.4 : %5.2f %%" % (f0, 100.0 * P[m].sum() / max(tot, 1e-12)))
