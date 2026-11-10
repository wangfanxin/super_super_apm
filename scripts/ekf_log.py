#!/usr/bin/env python3
"""ekf_log.py -- read the FCU's OWN EKF log and say which axis rejected the external nav.

Every fence escape tonight printed `the FCU disagrees by 5-9 m` while `lio_drift.py` showed the LIO
within 0.1 m of the truth.  The EKF only loses the position like that when it stops accepting the
external-nav observations, and the dataflash log records why: `NKF4` carries, per axis, the
innovation variance (IVN/IVE/IVD) and the normalised test ratio (SVN/SVE/SVD) that its gate compares,
plus the `flags` word that says which aiding the lane has active.

    python3 scripts/ekf_log.py kx8-03-01                # last 40 s of the flight, 1 Hz
    python3 scripts/ekf_log.py kx8-03-01 --t0 60 --t1 100
    python3 scripts/ekf_log.py kx8-03-01 --list         # which message types the log has
"""
import argparse
import os
import sys

from pymavlink import mavutil

ROOT = "/home/fanxin/lio_avoid/logs"
FLAG_BITS = [
    (0x1, "ATTD"), (0x2, "HORIZ"), (0x4, "VERT"), (0x8, "TERR"), (0x10, "CONST"),
    (0x20, "GPS"), (0x40, "OPT"), (0x80, "BODY"), (0x100, "WIND"), (0x200, "MAG"),
    (0x400, "EV"), (0x800, "FIX"), (0x1000, "EXTNAV"), (0x2000, "SAGA"), (0x4000, "TGT"),
]


def flags_str(v):
    on = [name for bit, name in FLAG_BITS if int(v) & bit]
    return "|".join(on) if on else "-"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tag")
    ap.add_argument("--t0", type=float, default=-1e9)
    ap.add_argument("--t1", type=float, default=1e9)
    ap.add_argument("--step", type=float, default=1.0)
    ap.add_argument("--list", action="store_true")
    a = ap.parse_args()

    p = os.path.join(ROOT, "missionA-" + a.tag, "flight.BIN")
    if not os.path.exists(p):
        print("no flight.BIN for %s" % a.tag)
        return 2
    log = mavutil.mavlink_connection(p, robust_parsing=True)
    # a dataflash log is not iterable: DFReader yields via recv_match
    def stream():
        while True:
            m = log.recv_match()
            if m is None:
                return
            yield m

    if a.list:
        seen = {}
        for m in stream():
            seen[m.get_type()] = seen.get(m.get_type(), 0) + 1
        for k in sorted(seen):
            print("  %-8s %d" % (k, seen[k]))
        return 0

    rows = []
    for m in stream():
        t = m.get_type()
        if t in ("NKF4", "XKF4"):
            rows.append((t, m))
    if not rows:
        print("no NKF4/XKF4 rows -- run with --list to see what this log has")
        return 1

    print("== %s: %d NKF4 rows" % (a.tag, len(rows)))
    print("   %8s %8s %8s %8s %8s %8s %8s %8s  %s"
          % ("t_sim", "IVN", "IVE", "IVD", "SVN", "SVE", "SVD", "flags", "aiding"))
    last = -1e9
    for _, m in rows:
        t = float(m.TimeUS) * 1e-6 if hasattr(m, "TimeUS") else float(getattr(m, "time_boot_ms", 0)) * 1e-3
        if t < a.t0 or t > a.t1:
            continue
        if t - last < a.step:
            continue
        last = t
        print("   %8.2f %8.2f %8.2f %8.2f %8.2f %8.2f %8.2f %8d  %s"
              % (t, getattr(m, "IVN", float("nan")), getattr(m, "IVE", float("nan")),
                 getattr(m, "IVD", float("nan")), getattr(m, "SVN", float("nan")),
                 getattr(m, "SVE", float("nan")), getattr(m, "SVD", float("nan")),
                 int(getattr(m, "flags", 0)), flags_str(getattr(m, "flags", 0))))
    return 0


if __name__ == "__main__":
    sys.exit(main())
