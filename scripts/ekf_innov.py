#!/usr/bin/env python3
"""ekf_innov.py -- the EKF's innovations and test ratios on one clock, at the moment it loses the plot.

For each 1 s: XKF4's test ratios (SP pos, SH height, SV velocity, SM mag) and XKF3's innovations
(IPN/IPE/IPD position, IVN/IVE/IVD velocity) plus XKF1's position/velocity, so "which axis rejected
the external nav" is read off the log instead of guessed.

    python3 scripts/ekf_innov.py kx8-03-01 --t0 60 --t1 120
"""
import argparse
import os

from pymavlink import mavutil

ROOT = "/home/fanxin/lio_avoid/logs"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tag")
    ap.add_argument("--t0", type=float, default=-1e9)
    ap.add_argument("--t1", type=float, default=1e9)
    ap.add_argument("--step", type=float, default=1.0)
    a = ap.parse_args()
    p = os.path.join(ROOT, "missionA-" + a.tag, "flight.BIN")
    log = mavutil.mavlink_connection(p, robust_parsing=True)

    rows = {}
    while True:
        m = log.recv_match()
        if m is None:
            break
        t = m.get_type()
        if t not in ("XKF3", "XKF4", "XKF1"):
            continue
        us = int(m.TimeUS)
        d = m.to_dict()
        rows.setdefault(us, {})[t] = d

    print("== %s" % a.tag)
    print("   %7s | %5s %5s %5s %5s | %7s %7s %7s | %7s %7s %7s | %6s %6s"
          % ("t", "SP", "SH", "SV", "SM", "IPN", "IPE", "IPD", "IVN", "IVE", "IVD", "PN", "PE"))
    last = -1e9
    for us in sorted(rows):
        t = us * 1e-6
        if t < a.t0 or t > a.t1 or (t - last) < a.step:
            continue
        last = t
        r = rows[us]
        k4 = r.get("XKF4", {})
        k3 = r.get("XKF3", {})
        k1 = r.get("XKF1", {})
        g = lambda d, k: float(d.get(k, float("nan")))
        print("   %7.2f | %5.2f %5.2f %5.2f %5.2f | %7.2f %7.2f %7.2f | %7.2f %7.2f %7.2f | %6.2f %6.2f"
              % (t, g(k4, "SP"), g(k4, "SH"), g(k4, "SV"), g(k4, "SM"),
                 g(k3, "IPN"), g(k3, "IPE"), g(k3, "IPD"),
                 g(k3, "IVN"), g(k3, "IVE"), g(k3, "IVD"),
                 g(k1, "PN"), g(k1, "PE")))


if __name__ == "__main__":
    main()
