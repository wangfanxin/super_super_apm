#!/usr/bin/env python3
"""chain_rates.py -- the measured rate of EVERY link in the chain, from one sortie's own logs.

The owner asked for a chain document with "nominal vs actual" topic rates.  Nominal lives in the launch
arguments; actual has to be counted from the files a sortie leaves behind, because this chain has already
been bitten twice by a link that was configured at one rate and delivered another (the lidar at 2.4 Hz
when the sim was slow, the external-nav feed at 47 of 48).  This script counts, per link:

    link                        file                          how
    --------------------------  ----------------------------  ------------------------------------------
    truth tap (SITL model)      tap.csv                       rows / sim seconds
    lidar scan in               lidar_imu.log                 "scans N in T s wall"
    IMU in                      lidar_imu.log                 "imu messages N (X Hz)"
    LIO odometry out            lio.csv                       rows / seconds
    SUPER reference out         traj.csv                      rows / seconds
    plan box -> /plan/cmd       setpoints.csv                 rows / wall seconds
    bridge -> FCU (msg 84)      wire.csv                      rows / wall seconds
    external nav -> FCU (331)   bridge.log                    "nav_hz=.."
    FCU logging (PSC/ATT)       flight.BIN                    PSCN rows / seconds

    python3 scripts/chain_rates.py pv1-02-02
"""
import csv
import os
import re
import sys

MISSION = "/home/fanxin/lio_avoid/logs"


def count_rows(path):
    if not os.path.isfile(path):
        return None
    with open(path, errors="replace") as f:
        return sum(1 for _ in f) - 1


def span(path, tcol=None, scale=1.0):
    if not os.path.isfile(path):
        return None
    rows = list(csv.DictReader(open(path, errors="replace")))
    if not rows:
        return None
    if tcol and tcol in rows[0]:
        return (float(rows[-1][tcol]) - float(rows[0][tcol])) * scale
    return float(len(rows))


def main():
    tag = sys.argv[1]
    d = os.path.join(MISSION, "missionA-" + tag)
    print("== %s" % d)

    # tap
    n = count_rows(os.path.join(d, "tap.csv"))
    t = span(os.path.join(d, "tap.csv"), "t_us", 1e-6)
    print("   %-28s %8s rows / %7.1f s = %7.1f Hz" % ("truth tap (SITL model)", n, t or 0,
                                                      (n / t) if n and t else 0))

    # lidar / imu from the sim's own report
    p = os.path.join(d, "lidar_imu.log")
    scans = imu = wall = None
    if os.path.isfile(p):
        txt = open(p, errors="replace").read()
        m = re.search(r"scans\s+(\d+) in ([\d.]+) s wall \(([\d.]+) Hz\)", txt)
        if m:
            scans, wall = int(m.group(1)), float(m.group(2))
            print("   %-28s %8s scans / %7.1f s = %7.2f Hz  (points/scan %.0f)"
                  % ("lidar scan (simulated MID-360)", scans, wall, float(m.group(3)),
                     float((re.search(r"points\s+(\d+) total, (\d+) per scan", txt) or [0, 0, 0])[2] or 0)))
        m = re.search(r"imu messages\s+(\d+) \(([\d.]+) Hz\)", txt)
        if m:
            print("   %-28s %8s msgs              = %7.1f Hz" % ("IMU (simulated)", m.group(1), float(m.group(2))))

    # lio odom
    p = os.path.join(d, "lio.csv")
    n = count_rows(p)
    t = span(p, "t_us", 1e-6)
    print("   %-28s %8s rows / %7.1f s = %7.1f Hz" % ("LIO odometry out", n, t or 0,
                                                      (n / t) if n and t else 0))

    # SUPER's own reference
    p = os.path.join(d, "traj.csv")
    n = count_rows(p)
    if n:
        rows = list(csv.DictReader(open(p, errors="replace")))
        tcol = "t_wall_us" if "t_wall_us" in rows[0] else ("t_us" if "t_us" in rows[0] else None)
        dur = (float(rows[-1][tcol]) - float(rows[0][tcol])) * (1e-6 if tcol and tcol.endswith("_us") else 1.0) if tcol else None
        print("   %-28s %8s rows / %7.1f s = %7.1f Hz" % ("SUPER reference out (traj.csv)", n, dur or 0,
                                                          (n / dur) if dur else 0))

    # the two bridge-side streams
    for name, fn, tcol, sc in (("plan box -> /plan/cmd", "setpoints.csv", "t_wall", 1.0),
                               ("bridge -> FCU (msg 84)", "wire.csv", "t_wall", 1.0)):
        p = os.path.join(d, fn)
        n = count_rows(p)
        t = span(p, tcol, sc)
        print("   %-28s %8s rows / %7.1f s = %7.1f Hz" % (name, n, t or 0, (n / t) if n and t else 0))

    # external nav rate from the bridge's own stop line
    p = os.path.join(d, "bridge.log")
    if os.path.isfile(p):
        txt = open(p, errors="replace").read()
        m = re.search(r"nav_hz=([\d.]+)", txt)
        if m:
            print("   %-28s %8s               = %7.1f Hz  (config 48)" % ("external nav -> FCU (331)", "", float(m.group(1))))
        for key in ("sent=", "plan_rx=", "lio_hz="):
            m = re.search(key + r"([\d.]+)", txt)
            if m:
                print("      %s%s" % (key, m.group(1)))

    # FCU logging rate
    p = os.path.join(d, "flight.BIN")
    if os.path.isfile(p):
        try:
            from pymavlink import mavutil
            import numpy as np  # noqa: F401
            m = mavutil.mavlink_connection(p)
            npsc = 0
            t0 = tl = None
            while True:
                msg = m.recv_match()
                if msg is None:
                    break
                if msg.get_type() == "PSCN":
                    ts = msg.to_dict()["TimeUS"]
                    if t0 is None:
                        t0 = ts
                    tl = ts
                    npsc += 1
            if npsc and tl and tl > t0:
                print("   %-28s %8s rows / %7.1f s = %7.1f Hz" % ("FCU pos-control log (PSCN)", npsc,
                                                                  (tl - t0) / 1e6, npsc / ((tl - t0) / 1e6)))
        except Exception as exc:                                   # noqa: BLE001
            print("   (BIN not parsed: %s)" % exc)


if __name__ == "__main__":
    main()
