#!/usr/bin/env python3
"""lio_attitude_watch.py -- did the ESTIMATOR'S OWN POSE/VELOCITY glitch when the FCU diverged?

missionA-kx1-01-01 and missionA-kx1-02-02 both ended with the FCU's EKF raising `EKF variance` and its
position estimate jumping by metres while the SITL truth was still near the mission altitude.  The
FCU is fed ONE external-nav message (MAVLink ODOMETRY: position, velocity AND attitude) built from
this LIO.  So before blaming the EKF, look at what it was fed: a yaw step, a velocity spike or a
position jump in `lio.csv` is the difference between "the EKF is broken" and "we fed it a lie".

    python3 scripts/lio_attitude_watch.py kx1-01-01 --t0 80 --t1 105
"""
import argparse
import csv
import math
import os

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--t0", type=float, default=-1e9)
    ap.add_argument("--t1", type=float, default=1e9)
    ap.add_argument("--step", type=float, default=0.5)
    a = ap.parse_args()
    rows = list(csv.DictReader(open(os.path.join(a.run, "lio.csv"))))
    t = np.array([int(r["t_us"]) for r in rows]) * 1e-6
    p = np.array([[float(r["x"]), float(r["y"]), float(r["z"])] for r in rows])
    v = np.array([[float(r["vx"]), float(r["vy"]), float(r["vz"])] for r in rows])
    w = np.array([[float(r["wx"]), float(r["wy"]), float(r["wz"])] for r in rows])
    qw = np.array([float(r["qw"]) for r in rows])
    qx = np.array([float(r["qx"]) for r in rows])
    qy = np.array([float(r["qy"]) for r in rows])
    qz = np.array([float(r["qz"]) for r in rows])
    yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    yaw = np.unwrap(yaw)
    dt = np.median(np.diff(t))
    dp = np.linalg.norm(np.diff(p, axis=0), axis=1) / dt
    dyaw = np.abs(np.diff(yaw)) / dt
    print("== %s: %d rows, %.1f..%.1f s, median dt %.4f s" % (os.path.basename(a.run), len(t), t[0], t[-1], dt))
    print("   implied |v| from position: p50 %.2f p99 %.2f max %.2f m/s" % (np.percentile(dp, 50), np.percentile(dp, 99), dp.max()))
    print("   |yaw rate| from attitude: p50 %.3f p99 %.2f max %.2f rad/s" % (np.percentile(dyaw, 50), np.percentile(dyaw, 99), dyaw.max()))
    k = int(np.argmax(dp))
    print("   worst position step at t=%.2f s: %.2f m in one tick (%.1f m/s)" % (t[k], np.linalg.norm(p[k + 1] - p[k]), dp[k]))
    k = int(np.argmax(dyaw))
    print("   worst yaw step   at t=%.2f s: %.4f rad (%.1f deg) in one tick" % (t[k], abs(yaw[k + 1] - yaw[k]), math.degrees(abs(yaw[k + 1] - yaw[k]))))
    print("   %8s %8s %8s %8s %8s %8s %8s" % ("t", "x", "y", "z", "|v|", "vz", "yaw_deg"))
    tt = max(a.t0, t[0])
    while tt <= min(a.t1, t[-1]):
        i = int(np.argmin(np.abs(t - tt)))
        print("   %8.2f %8.2f %8.2f %8.2f %8.2f %8.2f %8.1f"
              % (t[i], p[i, 0], p[i, 1], p[i, 2], np.linalg.norm(v[i]), v[i, 2], math.degrees(yaw[i])))
        tt += a.step


if __name__ == "__main__":
    main()
