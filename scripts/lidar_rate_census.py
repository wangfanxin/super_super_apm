#!/usr/bin/env python3
"""lidar_rate_census.py -- the lidar cloud rate each sortie actually got, next to its outcome.

`lidar_imu.log` prints a periodic line `scans N, points/scan P, cloud X Hz, imu Y, cast Z ms, skipped W`
and a final `raycast … ms/scan (… Hz capable)`.  The LIO's scan-to-map matching runs on those clouds, so
"6 Hz instead of the requested 10" is an estimator input deficit, not a cosmetic number.

    python3 scripts/lidar_rate_census.py            # every kx* sortie
    python3 scripts/lidar_rate_census.py kx8-03-01
"""
import glob
import os
import re
import sys

import numpy as np

ROOT = "/home/fanxin/lio_avoid/logs"


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


def one(tag):
    p = os.path.join(ROOT, "missionA-" + tag, "lidar_imu.log")
    if not os.path.exists(p):
        return None
    cloud, cast, skip, capable = [], [], [], float("nan")
    for ln in open(p, errors="replace"):
        m = re.search(r"cloud ([\d.]+) Hz.*?cast ([\d.]+) ms.*?skipped (\d+)", ln)
        if m:
            cloud.append(float(m.group(1)))
            cast.append(float(m.group(2)))
            skip.append(int(m.group(3)))
        m2 = re.search(r"raycast\s+([\d.]+) ms/scan", ln)
        if m2:
            capable = float(m2.group(1))
    if not cloud:
        return None
    # the periodic line starts at scan 1 with a 0.5 Hz startup value: drop the first sample
    c = cloud[1:] if len(cloud) > 2 else cloud
    k = cast[1:] if len(cast) > 2 else cast
    return dict(n=len(cloud), cloud_med=float(np.median(c)), cloud_min=float(np.min(c)),
                cast_med=float(np.median(k)), cast_max=float(np.max(k)),
                skip_last=skip[-1], capable=capable)


def main():
    tags = sys.argv[1:]
    if not tags:
        tags = sorted({os.path.basename(d).replace("missionA-", "")
                       for d in glob.glob(os.path.join(ROOT, "missionA-kx*"))})
    print("   %-12s %8s %8s %8s %8s %8s %7s  %s"
          % ("tag", "cloud p50", "cloud min", "cast p50", "cast MAX", "capable", "skipped", "outcome"))
    for t in tags:
        r = one(t)
        if r is None:
            continue
        print("   %-12s %8.2f %8.2f %8.1f %8.1f %8.1f %7d  %s"
              % (t, r["cloud_med"], r["cloud_min"], r["cast_med"], r["cast_max"], r["capable"],
                 r["skip_last"], outcome(t)))


if __name__ == "__main__":
    main()
