#!/usr/bin/env python3
"""attitude_truth.py -- the SITL model's TRUE tilt, from the tap quaternion (not the EKF's log).

Round 87 hit a contradiction: `ATT.Roll/Pitch` in `flight.BIN` reads 85-88 deg on the 3 m/s
sorties AND on a 2 m/s sortie that completed all 19 waypoints (`mx8-01-01`), which cannot both be
a real attitude.  `ATT` is the FCU's estimate and those two fields are not trustworthy in these
logs, so the tilt has to be taken from the plant itself: `tap.csv` carries the model's quaternion
(`qw,qx,qy,qz`, NED body->world), and tilt = angle between body +z and world -z (i.e. how far the
thrust axis is from vertical).

    python3 scripts/attitude_truth.py mx8-01-01 --t0 26 --t1 60
    python3 scripts/attitude_truth.py v3c5-01-01 --t0 26 --t1 42
"""
import argparse
import csv
import math
import os

MISSION = "/home/fanxin/lio_avoid/logs"


def load_tap(path):
    t, q, alt, hz = [], [], [], []
    with open(path, newline="") as fh:
        rd = csv.DictReader(fh)
        for r in rd:
            try:
                t.append(float(r["t_us"]) * 1e-6)
                q.append((float(r["qw"]), float(r["qx"]), float(r["qy"]), float(r["qz"])))
                alt.append(-float(r["pd"]))
                hz.append(math.hypot(float(r["vn"]), float(r["ve"])))
            except (KeyError, ValueError):
                continue
    return t, q, alt, hz


def tilt_deg(q):
    """tilt and heading of the SITL model from its quaternion (NED).

    returns (tilt_deg, yaw_deg):
      tilt = angle between the body +z axis and the world +z axis (both point DOWN when level),
      yaw  = atan2 of the body +x axis in the world horizontal plane, degrees in [-180, 180].

    In NED a LEVEL vehicle has body +z == world +z, so tilt = acos(vz).
    (The first version of this file had that sign the other way round and reported 179 deg for a
     vehicle that was level to 0.7 deg -- caught by running it on mx8-01-01.)
    """
    w, x, y, z = q
    # body +z in world = third column of R;  body +x in world = first column
    vx = 2.0 * (x * z + w * y)
    vy = 2.0 * (y * z - w * x)
    vz = 1.0 - 2.0 * (x * x + y * y)
    fx = 1.0 - 2.0 * (y * y + z * z)
    fy = 2.0 * (x * y + w * z)
    c = max(-1.0, min(1.0, vz))
    return math.degrees(math.acos(c)), math.degrees(math.atan2(fy, fx))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tag")
    ap.add_argument("--t0", type=float, default=20.0)
    ap.add_argument("--t1", type=float, default=60.0)
    ap.add_argument("--step", type=float, default=1.0)
    a = ap.parse_args()
    d = os.path.join(MISSION, "missionA-" + a.tag)
    t, q, alt, hz = load_tap(os.path.join(d, "tap.csv"))
    t = [x - t[0] for x in t]
    tilts = []
    yaws = []
    print("== %s   truth attitude (tap quaternion), t %.1f .. %.1f s" % (a.tag, a.t0, a.t1))
    print("   %7s %8s %9s %8s %8s" % ("t", "alt", "tilt_deg", "yaw_deg", "|v_xy|"))
    nxt = a.t0
    for i in range(len(t)):
        if t[i] < a.t0 or t[i] > a.t1 or t[i] < nxt:
            continue
        nxt = t[i] + a.step
        tl, yw = tilt_deg(q[i])
        tilts.append(tl)
        yaws.append(yw)
        print("   %7.2f %8.2f %9.1f %8.1f %8.2f" % (t[i], alt[i], tl, yw, hz[i]))
    if tilts:
        tilts.sort()
        print("   tilt over the window: p50 %.1f  p90 %.1f  p99 %.1f  max %.1f deg"
              % (tilts[len(tilts) // 2], tilts[int(0.9 * (len(tilts) - 1))],
                 tilts[int(0.99 * (len(tilts) - 1))], tilts[-1]))
    if yaws:
        # the heading is what the EKF's yaw must follow: if the MODEL's yaw wanders while the FCU's
        # yaw stays put (or the other way round), the world-frame position command is being applied
        # along the wrong body axis and the vehicle spirals away without any plan error at all
        span = max(yaws) - min(yaws)
        print("   model yaw over the window: first %.1f  last %.1f  span %.1f deg"
              % (yaws[0], yaws[-1], span))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
