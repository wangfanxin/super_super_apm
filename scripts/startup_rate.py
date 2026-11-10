#!/usr/bin/env python3
"""startup_rate.py -- is the rtf the gate measures lost BEFORE the plan box even starts?

The tap's rtf comes from `truth_rx.py`, which stamps wall time on the FIRST and LAST sample it
receives.  The tap is started first (it must be: the SITL dials out to it), so its window begins
tens of seconds before the plan box's first setpoint.  If the simulator stalls while SITL, LIO and
the 522-pillar map are all initialising, that stall lands ONLY in the tap's window -- and the gate
then rejects a flight that kept perfect real time.

`phase.csv` (written by the supervisor) carries both clocks at every phase boundary, which is exactly
the resolution needed to see where the deficit is.

    python3 scripts/startup_rate.py kx1-01-02 kx1-02-01 kv1-02-01
"""
import csv
import os
import sys

ROOT = "/home/fanxin/lio_avoid/logs"


def one(tag):
    p = os.path.join(ROOT, "missionA-" + tag, "phase.csv")
    if not os.path.exists(p):
        print("%-12s <no phase.csv>" % tag)
        return
    rows = list(csv.DictReader(open(p)))
    if not rows:
        return
    w0 = float(rows[0]["t_wall"])
    s0 = float(rows[0]["t_sim_us"]) * 1e-6
    print("== %s" % tag)
    print("   %-12s %8s %8s %9s %9s" % ("phase", "wall", "sim", "d_sim", "ratio sim/wall"))
    prev_w, prev_s = w0, s0
    for r in rows:
        w = float(r["t_wall"]) - w0
        s = float(r["t_sim_us"]) * 1e-6 - s0
        dw, ds = w - prev_w, s - prev_s
        ratio = (ds / dw) if dw > 1e-9 else float("nan")
        print("   %-12s %8.2f %8.2f %9.2f   %8.4f %s"
              % (r["phase"][:12], w, s, ds, ratio, "  <-- sim stalled" if (dw > 0.5 and ratio < 0.9) else ""))
        prev_w, prev_s = w, s
    print("   total: wall %.2f s, sim %.2f s, ratio %.4f" % (prev_w, prev_s, prev_s / prev_w if prev_w else 0))


if __name__ == "__main__":
    for t in sys.argv[1:]:
        one(t)
        print()
