#!/usr/bin/env python3
"""lio_yaw_census.py -- LIO yaw glitches vs "did the sortie end at the fence?"

Every fence escape tonight printed `the FCU disagrees by 5-9 m` while `lio_drift.py` showed the LIO
within 0.1 m of the truth.  An EKF can only lose the position like that if it stops trusting the
external nav; a YAW jump is the one LIO failure that rotates the position estimate without producing
a position jump (`lio_drift.py` already rules the latter out).

    python3 scripts/lio_yaw_census.py kx7-01-01 kx8-03-01 ...
"""
import csv
import glob
import os
import re
import sys

import numpy as np

ROOT = "/home/fanxin/lio_avoid/logs"


def glitches(tag):
    p = os.path.join(ROOT, "missionA-" + tag, "lio.csv")
    if not os.path.exists(p):
        return None
    q = {k: [] for k in ("qw", "qx", "qy", "qz")}
    for r in csv.DictReader(open(p)):
        for k in q:
            q[k].append(float(r[k]))
    qw, qx, qy, qz = (np.array(q[k]) for k in ("qw", "qx", "qy", "qz"))
    yaw = np.unwrap(np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))
    d = np.degrees(np.abs(np.diff(yaw)))
    return dict(n=len(yaw), p99=float(np.percentile(d, 99)), gt2=int((d > 2).sum()),
                gt5=int((d > 5).sum()), mx=float(d.max()))


def outcome(tag):
    p = os.path.join(ROOT, "missionA-" + tag, "supervisor.log")
    if not os.path.exists(p):
        return "?"
    txt = open(p, errors="replace").read()
    if "ABORT: geofence" in txt:
        return "FENCE"
    if "ABORT:" in txt:
        return "ABORT"
    if "landed:" in txt:
        return "landed"
    return "?"


def main():
    tags = sys.argv[1:]
    if not tags:
        tags = sorted({os.path.basename(d).replace("missionA-", "")
                       for d in glob.glob(os.path.join(ROOT, "missionA-kx*"))})
    print("   %-12s %8s %8s %8s %8s %8s  %s" % ("tag", "samples", "dyaw p99", ">2 deg", ">5 deg", "max deg", "outcome"))
    for t in tags:
        g = glitches(t)
        if g is None:
            continue
        print("   %-12s %8d %8.2f %8d %8d %8.1f  %s"
              % (t, g["n"], g["p99"], g["gt2"], g["gt5"], g["mx"], outcome(t)))


if __name__ == "__main__":
    main()
