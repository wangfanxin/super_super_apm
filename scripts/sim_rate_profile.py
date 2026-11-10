#!/usr/bin/env python3
"""sim_rate_profile.py -- WHERE is the real-time factor lost?

Every sortie is gated on the truth tap's rtf (sim seconds / wall seconds, 0.98-1.02) and four
consecutive attempts came back 0.9765-0.9792 on a host whose load average was 0.85.  That is either a
uniform 2.3 % shortfall (the simulator cannot keep up, e.g. the LiDAR raycast inside the physics
loop) or a startup transient being averaged into the flight.

`setpoints.csv` carries BOTH clocks (`t_wall` from this process, `t_sim_us` from SITL), so the ratio
can be measured per second, without touching the tap.

    python3 scripts/sim_rate_profile.py kx1-01-02 kx1-02-01
"""
import csv
import os
import sys

import numpy as np

ROOT = "/home/fanxin/lio_avoid/logs"


def one(tag, width=5.0):
    d = os.path.join(ROOT, "missionA-" + tag)
    rows = list(csv.DictReader(open(os.path.join(d, "setpoints.csv"))))
    tw = np.array([float(r["t_wall"]) for r in rows])
    ts = np.array([float(r["t_sim_us"]) * 1e-6 for r in rows])
    # t_sim_us is quantised to 5 ms: fit the slope over a window instead of differencing ticks
    tw = tw - tw[0]
    ts = ts - ts[0]
    print("== %s: %.1f s of setpoints, sim advanced %.1f s -> rtf %.4f over the whole stream"
          % (tag, tw[-1], ts[-1], ts[-1] / tw[-1]))
    print("   %6s %6s %8s %8s" % ("t_wall", "t_sim", "d_sim", "local_rtf"))
    edges = np.arange(0.0, tw[-1], width)
    for t0 in edges:
        m = (tw >= t0) & (tw < t0 + width)
        if m.sum() < 20:
            continue
        dtw = tw[m][-1] - tw[m][0]
        dts = ts[m][-1] - ts[m][0]
        if dtw <= 0:
            continue
        print("   %6.1f %6.1f %8.2f %8.4f%s"
              % (t0, ts[m][0], dts, dts / dtw, "   <-- below 0.98" if dts / dtw < 0.98 else ""))
    # a straight least-squares slope is the number the gate effectively measures
    a = np.polyfit(tw, ts, 1)[0]
    print("   least-squares sim/wall slope %.4f" % a)


if __name__ == "__main__":
    for t in sys.argv[1:]:
        one(t)
        print()
