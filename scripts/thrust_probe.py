#!/usr/bin/env python3
"""thrust_probe.py -- what throttle did the FCU actually need in level hover, vs MOT_THST_HOVER?

WHY.  Round 87's 3 m/s sorties all fail the same way: the aircraft holds 2 m, then sinks or climbs
while the FCU's own target stays at 2.0, `Potential Thrust Loss` appears, the model touches the
ground and then the EKF and the plant part company.  The single number that decides whether that is
a THRUST FEED-FORWARD calibration problem is the throttle the FCU needed to hover:

  * if `CTUN.ThO` in level hover is much larger than `MOT_THST_HOVER`, the controller's feed-forward
    is low and the whole vertical loop runs on feedback; under a sustained tilt (which is exactly
    what 3 m/s through these corners asks for) the feedback has to make up ~40 % more thrust and it
    runs out -> sink, then over-thrust, then PIO.  The fix is a PARAMETER (MOT_THST_HOVER), which
    the owner has already authorised ("如果pid参数不够，你需要调整固件的参数").
  * if `ThO` matches `MOT_THST_HOVER` within a few percent, the vertical loop is not mis-trimmed
    and the problem is elsewhere (tilt limit / position gains / the plant).

Also printed: the tilt (from ATT.Roll/Pitch) in the same window, and the ratio `1/cos(tilt)` --
the factor by which the thrust requirement grows.  A window where `ThO / MOT_THST_HOVER` tracks
`1/cos(tilt)` means the trim is right and only the margin is thin.

    python3 scripts/thrust_probe.py v3c5-01-01 --t0 28 --t1 34
    python3 scripts/thrust_probe.py mx8-01-01 --t0 30 --t1 60      # a sortie that completed
"""
import argparse
import math
import os

import numpy as np
from pymavlink import mavutil

MISSION = "/home/fanxin/lio_avoid/logs"


def load_bin(d):
    path = os.path.join(d, "flight.BIN")
    if not os.path.isfile(path):
        path = os.path.join(d, "logs", "00000001.BIN")
    m = mavutil.mavlink_connection(path)
    rows = {"CTUN": [], "ATT": [], "PARM": []}
    while True:
        msg = m.recv_match(type=list(rows), blocking=False)
        if msg is None:
            break
        rows[msg.get_type()].append(msg.to_dict())
    return rows


def series(rows, name, field):
    t = np.array([float(r["TimeUS"]) * 1e-6 for r in rows])
    v = np.array([float(r[field]) if field in r else np.nan for r in rows])
    return t, v


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tag")
    ap.add_argument("--t0", type=float, default=28.0)
    ap.add_argument("--t1", type=float, default=34.0)
    ap.add_argument("--series", action="store_true",
                    help="also print a 1 Hz tilt / throttle / altitude table over the window")
    a = ap.parse_args()
    d = os.path.join(MISSION, "missionA-" + a.tag)
    rows = load_bin(d)
    if not rows["CTUN"]:
        print("no CTUN rows")
        return 1
    t_th, tho = series(rows["CTUN"], "CTUN", "ThO")
    t_al, alt = series(rows["CTUN"], "CTUN", "Alt")
    t_da, dalt = series(rows["CTUN"], "CTUN", "DAlt")
    t_r, roll = series(rows["ATT"], "ATT", "Roll")
    t_p, pitch = series(rows["ATT"], "ATT", "Pitch")
    t_y, yaw = series(rows["ATT"], "ATT", "Yaw")

    # MOT_THST_HOVER as the FCU actually has it
    hov = None
    for r in rows["PARM"]:
        if r.get("Name") == "MOT_THST_HOVER":
            hov = float(r["Value"])
    m = (t_th >= a.t0) & (t_th <= a.t1)
    if not np.any(m):
        print("window %.1f..%.1f has no CTUN rows (t range %.1f..%.1f)"
              % (a.t0, a.t1, t_th.min(), t_th.max()))
        return 1
    tilt = np.interp(t_th[m], t_r, np.abs(roll)) ** 2 + np.interp(t_th[m], t_p, np.abs(pitch)) ** 2
    tilt = np.degrees(np.arctan(np.sqrt(tilt)))
    print("== %s   window %.1f .. %.1f s   (%d CTUN rows)" % (a.tag, a.t0, a.t1, m.sum()))
    print("   MOT_THST_HOVER in the BIN : %s" % ("%.4f" % hov if hov is not None else "<not logged>"))
    print("   ThO     p50 %.3f  p90 %.3f  min %.3f  max %.3f"
          % (np.percentile(tho[m], 50), np.percentile(tho[m], 90), tho[m].min(), tho[m].max()))
    print("   tilt    p50 %.1f deg  p90 %.1f deg  max %.1f deg"
          % (np.percentile(tilt, 50), np.percentile(tilt, 90), tilt.max()))
    print("   alt     p50 %.2f m  min %.2f  max %.2f   (target p50 %.2f m)"
          % (np.percentile(alt[m], 50), alt[m].min(), alt[m].max(),
             -np.percentile(dalt[m], 50)))
    if hov:
        print("   ThO / MOT_THST_HOVER : p50 %.2f  p90 %.2f"
              % (np.percentile(tho[m], 50) / hov, np.percentile(tho[m], 90) / hov))
    print("   1/cos(tilt) at p50 tilt: %.2f   at p90 tilt: %.2f"
          % (1.0 / math.cos(math.radians(np.percentile(tilt, 50))),
             1.0 / math.cos(math.radians(np.percentile(tilt, 90)))))
    print("   reading: a hover where ThO/MOT_THST_HOVER is ~1/cos(tilt) means the trim is right;")
    print("   a hover where ThO is much larger than MOT_THST_HOVER means the vertical loop is")
    print("   running on feedback alone and has no margin left under tilt.")
    if a.series:
        print()
        print("   %6s %8s %8s %8s %8s %8s %8s" % ("t", "tilt_deg", "ThO", "alt", "DAlt_up",
                                                  "yaw_deg", "yaw_rate"))
        nxt = a.t0
        prev_y = None
        for i in range(len(t_th)):
            t = float(t_th[i])
            if t < a.t0 or t > a.t1 or t < nxt:
                continue
            nxt = t + 1.0
            r_ = float(np.interp(t, t_r, np.abs(roll)))
            p_ = float(np.interp(t, t_p, np.abs(pitch)))
            y_ = float(np.interp(t, t_y, yaw))
            yr = 0.0 if prev_y is None else ((y_ - prev_y + 540.0) % 360.0 - 180.0)
            prev_y = y_
            tl = math.degrees(math.atan(math.hypot(r_, p_)))
            print("   %6.2f %8.1f %8.3f %8.2f %8.2f %8.1f %8.1f"
                  % (t, tl, tho[i], alt[i], -dalt[i], y_, yr))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
