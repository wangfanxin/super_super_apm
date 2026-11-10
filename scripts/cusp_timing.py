#!/usr/bin/env python3
"""cusp_timing.py -- put each reference fold on the recorded clock.

The thin layer prints `CUSP #n t_wall=...` with `time.monotonic()`, the SAME clock the bridge writes
into `setpoints.csv`/`wire.csv`, so a fold can be located exactly:

  * which leg (`trajectory_id` in wire.csv is the pacer's leg id, so its changes are the handovers),
  * how long after that handover the fold happened,
  * how far the folded point is from the corner the leg is flying to,
  * and what the reference speed did around it.

    python3 scripts/cusp_timing.py kx5-01-01
"""
import argparse
import csv
import os
import re

import numpy as np

CORNERS = [(10.0, 0.0), (5.0, 8.66), (-5.0, 8.66), (-10.0, 0.0), (-5.0, -8.66), (5.0, -8.66)]
ANSI = re.compile(r"\x1b\[[0-9;]*m")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tag")
    ap.add_argument("--root", default="/home/fanxin/lio_avoid/logs")
    a = ap.parse_args()
    d = os.path.join(a.root, "missionA-" + a.tag)

    # --- the folds, as the thin layer printed them (L frame + wall clock)
    cusps = []
    for ln in open(os.path.join(d, "traj.log"), errors="replace"):
        ln = ANSI.sub("", ln)
        m = re.search(r"CUSP #(\d+) t_wall=([\d.]+).*?(\d+) deg.*?\|v\| ([\d.]+) m/s\) at L\(([-\d.]+), ([-\d.]+)\)"
                      r" leg (\d+)/(\d+)", ln)
        if m:
            cusps.append(dict(n=int(m.group(1)), t=float(m.group(2)), ang=float(m.group(3)),
                              v=float(m.group(4)), lx=float(m.group(5)), ly=float(m.group(6)),
                              leg=int(m.group(7))))
    # --- the wire: leg id changes = handovers, on the same clock
    w = list(csv.DictReader(open(os.path.join(d, "wire.csv"))))
    tw = np.array([float(r["t_wall"]) for r in w])
    tid = np.array([int(r["trajectory_id"]) for r in w])
    pl = np.array([[float(r["p_l_x"]), float(r["p_l_y"])] for r in w])
    hand = [(tw[i], int(tid[i])) for i in range(1, len(tw)) if tid[i] != tid[i - 1]]
    print("== %s: %d folds, %d handovers, wire %.1f..%.1f s"
          % (a.tag, len(cusps), len(hand), tw[0], tw[-1]))
    print("   %4s %9s %6s %6s %7s %9s %8s %9s %8s"
          % ("#", "t_wall", "angle", "|v|", "leg", "since_ho", "ho_id", "d_corner", "v_local"))
    for c in cusps:
        prev = [h for h in hand if h[0] <= c["t"]]
        since = (c["t"] - prev[-1][0]) if prev else float("nan")
        hid = prev[-1][1] if prev else -1
        # L -> W: W = (-L_y, L_x) + t   (calib yaw +90.05 deg, translation <=1 cm)
        wx, wy = -c["ly"], c["lx"]
        dc = min(np.hypot(wx - x, wy - y) for x, y in CORNERS)
        k = int(np.argmin(np.abs(tw - c["t"])))
        lo, hi = max(0, k - 10), min(len(tw), k + 11)
        v_local = np.linalg.norm(np.diff(pl[lo:hi], axis=0), axis=1) / np.median(np.diff(tw[lo:hi]))
        print("   %4d %9.3f %6.0f %6.2f %7d %9.2f %8d %9.2f %8.2f"
              % (c["n"], c["t"], c["ang"], c["v"], c["leg"], since, hid, dc,
                 float(np.median(v_local)) if len(v_local) else float("nan")))
    if cusps:
        s = np.array([(c["t"] - [h for h in hand if h[0] <= c["t"]][-1][0])
                      for c in cusps if [h for h in hand if h[0] <= c["t"]]])
        print("   since-handover: p10 %.2f  p50 %.2f  p90 %.2f s" % tuple(np.percentile(s, [10, 50, 90])))
        dc = []
        for c in cusps:
            wx, wy = -c["ly"], c["lx"]
            dc.append(min(np.hypot(wx - x, wy - y) for x, y in CORNERS))
        print("   distance to the nearest corner: p10 %.2f  p50 %.2f  p90 %.2f m"
              % tuple(np.percentile(dc, [10, 50, 90])))


if __name__ == "__main__":
    main()
