#!/usr/bin/env python3
"""lidar_skips.py -- WHEN did the lidar skipped its scans, and does that line up with the LIO divergence?

`lidar_imu.log` prints, every ~20 scans, `scans N, … cloud X Hz, … cast Z ms, skipped W`.  W is
cumulative, so the increments locate the gaps.  `src/sim/lidar_imu.py` skips a scan when the TRUTH TAP's
samples do not span that scan's 0.1 s window -- i.e. a tap stall becomes a CLOUD GAP, which the LIO then
has to scan-match across.

    python3 scripts/lidar_skips.py kx8-03-01            # the whole flight
    python3 scripts/lidar_skips.py kx8-03-01 --near 99  # only rows around t=99 s
"""
import argparse
import os
import re

ROOT = "/home/fanxin/lio_avoid/logs"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tag")
    ap.add_argument("--near", nargs=2, type=float, default=None, help="only show wall t in [a,b] seconds")
    a = ap.parse_args()
    p = os.path.join(ROOT, "missionA-" + a.tag, "lidar_imu.log")
    prev_skip = 0
    rows = []
    for ln in open(p, errors="replace"):
        m = re.search(r"scans (\d+), points/scan (\d+), cloud ([\d.]+) Hz.*?cast ([\d.]+) ms, skipped (\d+)", ln)
        if not m:
            continue
        scans, hz, cast, skip = int(m.group(1)), float(m.group(3)), float(m.group(4)), int(m.group(5))
        rows.append((scans, hz, cast, skip, skip - prev_skip))
        prev_skip = skip
    print("== %s: %d reports, %d scans, %d skipped in total"
          % (a.tag, len(rows), rows[-1][0] if rows else 0, rows[-1][3] if rows else 0))
    print("   %6s %7s %8s %8s %8s" % ("scans", "cloud Hz", "cast ms", "skipped", "+this line"))
    for scans, hz, cast, skip, inc in rows:
        t = scans * 0.1                      # scans are 10 Hz nominal -> seconds of scan time
        if a.near is not None and not (a.near[0] <= t <= a.near[1]):
            continue
        flag = "  <== a gap" if inc >= 3 else ""
        print("   %6d %7.2f %8.1f %8d %8d%s" % (scans, hz, cast, skip, inc, flag))


if __name__ == "__main__":
    main()
