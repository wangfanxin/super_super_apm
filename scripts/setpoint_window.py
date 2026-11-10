#!/usr/bin/env python3
"""setpoint_window.py -- dump chosen columns of setpoints.csv over a window on the TRUTH clock.

`setpoints.csv` is the plan box's own view: `cmd_*` is what it published (W frame),
`vel_*`/`acc_*` the feed-forward beside the position, `fcu_*` the FCU's estimate, `lio_*` the
estimator, `yaw_cmd` the yaw it asked for.

THE TWO CLOCKS.  `setpoints.csv` timestamps live on the recorder's clock and `tap.csv` on the
SITL's; they start seconds apart and must not be subtracted.  This script lines them up the way
`vertical_forensics.py` does -- the first sample where the plan asks for more than 1 m of altitude
is paired with the first truth sample above 1 m (the take-off), and that offset is applied -- so
the `t` printed below is the TRUTH clock, the same one the figures and the acceptance rows use.

    python3 scripts/setpoint_window.py v3c5-01-01 --t0 31 --t1 35 --cols yaw_cmd cmd_x cmd_y
"""
import argparse
import csv
import os

import numpy as np

MISSION = "/home/fanxin/lio_avoid/logs"


def load(path, cols):
    out = {c: [] for c in cols}
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            try:
                for c in cols:
                    out[c].append(float(r[c]))
            except (KeyError, ValueError):
                continue
    return {c: np.asarray(v, dtype=float) for c, v in out.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tag")
    ap.add_argument("--t0", type=float, default=0.0)
    ap.add_argument("--t1", type=float, default=1e9)
    ap.add_argument("--step", type=float, default=0.2, help="print every this many seconds")
    ap.add_argument("--cols", nargs="+", default=["yaw_cmd", "cmd_x", "cmd_y", "cmd_z",
                                                  "fcu_x", "fcu_y", "fcu_z"])
    a = ap.parse_args()
    d = os.path.join(MISSION, "missionA-" + a.tag)
    sp = load(os.path.join(d, "setpoints.csv"), ["t_sim_us"] + a.cols)
    tap = load(os.path.join(d, "tap.csv"), ["t_us", "pd"])
    st = sp["t_sim_us"] * 1e-6
    tt = (tap["t_us"] - tap["t_us"][0]) * 1e-6
    tz = -tap["pd"]
    # the take-off anchor needs cmd_z even when the caller did not ask for that column
    cz = sp["cmd_z"] if "cmd_z" in sp else load(os.path.join(d, "setpoints.csv"), ["cmd_z"])["cmd_z"]
    up_sp = np.where(cz > 1.0)[0]
    up_tp = np.where(tz > 1.0)[0]
    if len(up_sp) and len(up_tp):
        off = float(st[up_sp[0]] - tt[up_tp[0]])
    else:
        off = 0.0
    print("== %s   t %.2f .. %.2f s   (truth clock; setpoint clock offset %+.2f s)"
          % (a.tag, a.t0, a.t1, off))
    print("   %7s " % "t" + " ".join("%9s" % c for c in a.cols))
    nxt = a.t0
    for i in range(len(st)):
        t = float(st[i] - off)
        if t < a.t0 or t > a.t1 or t < nxt:
            continue
        nxt = t + a.step
        print("   %7.2f " % t + " ".join("%9.3f" % float(sp[c][i]) for c in a.cols))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
