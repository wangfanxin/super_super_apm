#!/usr/bin/env python3
"""ekf_fields.py -- dump the field names of the EKF rows in a dataflash log (one of each)."""
import os
import sys

from pymavlink import mavutil

ROOT = "/home/fanxin/lio_avoid/logs"
tag = sys.argv[1] if len(sys.argv) > 1 else "kx8-03-01"
want = set(sys.argv[2:]) or {"XKF4", "XKFS", "XKF3", "VISP", "VISV"}
p = os.path.join(ROOT, "missionA-" + tag, "flight.BIN")
log = mavutil.mavlink_connection(p, robust_parsing=True)
seen = set()
while True:
    m = log.recv_match()
    if m is None:
        break
    t = m.get_type()
    if t in want and t not in seen:
        seen.add(t)
        print("== %s" % t)
        d = m.to_dict()
        for k in d:
            if k not in ("mavpackettype", "C"):
                print("   %-10s %s" % (k, d[k]))
        print()
    if seen == want:
        break
