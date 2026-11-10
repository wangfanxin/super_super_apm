#!/usr/bin/env python3
"""lio_step_census.py -- how big are the LIO's own per-frame steps, good sortie vs runaway?

The bridge forwards the LIO's pose to the FCU at ~45 Hz.  A data-quality gate there has to know what
"normal" looks like: this prints, per sortie, the position step SPEED and the yaw RATE of the LIO's own
stream (from `lio.csv`, 200 Hz), plus when the stream stopped being physically possible.

Thresholds for the gate are then chosen from this table, not invented:
  * the vehicle's own limits are max_vel 1.5 m/s (planner) and the FCU's WP_SPD 1.5 m/s;
  * a good sortie's p99 must sit far below the gate, and a runaway's first excursion must sit above it.

    python3 scripts/lio_step_census.py                # every kx* sortie
    python3 scripts/lio_step_census.py kx8-03-01
"""
import csv
import glob
import os
import sys

import numpy as np

ROOT = "/home/fanxin/lio_avoid/logs"


def one(tag):
    p = os.path.join(ROOT, "missionA-" + tag, "lio.csv")
    if not os.path.exists(p):
        return None
    t, x, y, z, yaw = [], [], [], [], []
    for r in csv.DictReader(open(p)):
        t.append(int(r["t_us"]) * 1e-6)
        x.append(float(r["x"])); y.append(float(r["y"])); z.append(float(r["z"]))
        qw, qx, qy, qz = (float(r[k]) for k in ("qw", "qx", "qy", "qz"))
        yaw.append(np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))
    t = np.array(t); P = np.stack([np.array(x), np.array(y), np.array(z)], axis=1)
    yaw = np.unwrap(np.array(yaw))
    dt = np.diff(t)
    ok = dt > 1e-6
    spd = np.linalg.norm(np.diff(P, axis=0), axis=1)[ok] / dt[ok]
    wr = np.degrees(np.abs(np.diff(yaw)))[ok] / dt[ok]
    return dict(n=len(t), span=t[-1] - t[0], spd=spd, wr=wr)


def outcome(tag):
    p = os.path.join(ROOT, "missionA-" + tag, "supervisor.log")
    if not os.path.exists(p):
        return "?"
    txt = open(p, errors="replace").read()
    if "ABORT: geofence" in txt:
        return "FENCE"
    if "ABORT:" in txt:
        return "ABORT"
    return "landed" if "landed:" in txt else "?"


def main():
    tags = sys.argv[1:]
    if not tags:
        tags = sorted({os.path.basename(d).replace("missionA-", "")
                       for d in glob.glob(os.path.join(ROOT, "missionA-kx*"))})
    print("   %-12s %7s | %7s %7s %7s | %8s %8s %8s | %s"
          % ("tag", "span_s", "spd p50", "p99", "max", "yawr p50", "p99", "max", "outcome"))
    for tg in tags:
        r = one(tg)
        if r is None:
            continue
        print("   %-12s %7.1f | %7.3f %7.2f %7.1f | %8.2f %8.1f %8.0f | %s"
              % (tg, r["span"], np.percentile(r["spd"], 50), np.percentile(r["spd"], 99), r["spd"].max(),
                 np.percentile(r["wr"], 50), np.percentile(r["wr"], 99), r["wr"].max(), outcome(tg)))


if __name__ == "__main__":
    main()
