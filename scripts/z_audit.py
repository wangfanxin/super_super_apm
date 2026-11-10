#!/usr/bin/env python3
"""z_audit.py -- is the aircraft descending, or does the estimator only THINK it is?

The last sortie (`hz1-01-01`) died with the SITL reporting `SIM Hit ground` and then `EKF variance`
/ `EKF Failsafe` / `Potential Thrust Loss`, while the published reference sat still at z = 2.07.
That is either a real descent (control/thrust) or a wrong altitude estimate (LIO/EKF), and the two
need opposite fixes, so this script puts the three z streams on one clock:

    [1] truth  = SITL model (tap.csv, the physics)
    [2] fcu    = the FCU's own EKF estimate (setpoints.csv fcu_z, W up)
    [3] lio    = the estimator's z (setpoints.csv lio_z, L, up)

`setpoints.t_sim_us` and `tap.t_us` are the same simulated clock (the bridge stamps each row with the
tick it just read), so they are joined directly with a tolerance instead of a shift search.

    python3 scripts/z_audit.py hz1-01-01 [--step 1.0]
"""
import argparse
import csv
import os

import numpy as np

MISSION = "/home/fanxin/lio_avoid/logs"


def load_sp(tag):
    p = os.path.join(MISSION, "missionA-" + tag, "setpoints.csv")
    t, cmd, fcu, lio = [], [], [], []
    for r in csv.DictReader(open(p)):
        t.append(float(r["t_sim_us"]) * 1e-6)
        cmd.append(float(r["cmd_z"]))
        fcu.append(float(r["fcu_z"]))
        lio.append(float(r["lio_z"]))
    return np.array(t), np.array(cmd), np.array(fcu), np.array(lio)


def load_truth(tag):
    p = os.path.join(MISSION, "missionA-" + tag, "tap.csv")
    t, z = [], []
    for r in csv.DictReader(open(p)):
        t.append(float(r["t_us"]) * 1e-6)
        z.append(-float(r["pd"]))
    t = np.array(t)
    z = np.array(z)
    # the tap is 1200 Hz; decimate for the join
    return t, z


ap = argparse.ArgumentParser()
ap.add_argument("tag")
ap.add_argument("--step", type=float, default=1.0)
ap.add_argument("--t0", type=float, default=None)
ap.add_argument("--t1", type=float, default=None)
a = ap.parse_args()

t, cmd, fcu, lio = load_sp(a.tag)
tt, tz = load_truth(a.tag)
# the tap's clock and the setpoint clock must overlap; if not, shift the tap onto the setpoints
if tt[-1] < t[0] or tt[0] > t[-1]:
    print("clocks do not overlap: tap [%.1f, %.1f] setpoints [%.1f, %.1f]"
          % (tt[0], tt[-1], t[0], t[-1]))
    tt = tt - tt[0] + t[0]
print("%s: setpoints %.1f..%.1f s (%d rows), tap %.1f..%.1f s (%d rows)"
      % (a.tag, t[0], t[-1], len(t), tt[0], tt[-1], len(tt)))

t0 = t[0] if a.t0 is None else a.t0
t1 = t[-1] if a.t1 is None else a.t1
print("\n   t   truth_z   fcu_z   lio_z  cmd_z |  fcu-truth   lio-fcu")
tt0 = t0
while tt0 <= t1:
    i = int(np.searchsorted(t, tt0).clip(0, len(t) - 1))
    j = int(np.searchsorted(tt, tt0).clip(0, len(tt) - 1))
    # nearest tap sample within 0.05 s, else blank
    if abs(tt[j] - tt0) < 0.2:
        tr = "%7.3f" % tz[j]
        d1 = "%+8.3f" % (fcu[i] - tz[j])
    else:
        tr = "      -"
        d1 = "       -"
    print("%6.1f %s %7.3f %7.3f %6.2f | %s %+9.3f"
          % (tt0 - t[0], tr, fcu[i], lio[i], cmd[i], d1, lio[i] - fcu[i]))
    tt0 += a.step
