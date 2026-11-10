#!/usr/bin/env python3
"""pva_wire_census.py -- is the PVA really on the wire, or is the field present and zero?

`type_mask` says which fields the FCU should USE; it says nothing about whether they carry
anything.  Round 87 found `type_mask 2048` (posvelacc) on every row of `wire.csv` while
`v_l_*` and `a_l_*` were **identically zero** -- i.e. the mask promised a velocity and an
acceleration that the numbers did not deliver, so the FCU was flying positions only.

This script prints, for one sortie, four censuses on the same footing:

    plan box   /plan/cmd    traj.csv.cmd.csv   columns vx_l..az_l
    wire       after L->W   wire.csv           columns v_l_* , a_l_*
    truth      the SITL model tap.csv          |vn,ve,vd| (1 kHz)

    python3 scripts/pva_wire_census.py v3l2-01-02
"""
import argparse
import csv
import math
import os

MISSION = "/home/fanxin/lio_avoid/logs"


def census(path, cols, scale=1.0):
    n = nz = 0
    tot = 0.0
    mx = 0.0
    p95 = []
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            try:
                v = scale * math.sqrt(sum(float(r[c]) ** 2 for c in cols))
            except (KeyError, ValueError):
                continue
            n += 1
            tot += v
            mx = max(mx, v)
            if v > 0.1:
                nz += 1
            if n % 7 == 0:
                p95.append(v)
    p95.sort()
    return n, nz, tot / max(n, 1), mx, (p95[int(0.95 * (len(p95) - 1))] if p95 else 0.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tag")
    a = ap.parse_args()
    d = os.path.join(MISSION, "missionA-" + a.tag)

    print("== %s" % a.tag)
    rows = [
        ("plan box  |v_l|", os.path.join(d, "traj.csv.cmd.csv"), ["vx_l", "vy_l", "vz_l"]),
        ("plan box  |a_l|", os.path.join(d, "traj.csv.cmd.csv"), ["ax_l", "ay_l", "az_l"]),
        ("wire     |v_l|", os.path.join(d, "wire.csv"), ["v_l_x", "v_l_y", "v_l_z"]),
        ("wire     |a_l|", os.path.join(d, "wire.csv"), ["a_l_x", "a_l_y", "a_l_z"]),
        ("truth    |v|  ", os.path.join(d, "tap.csv"), ["vn", "ve", "vd"]),
    ]
    print("   %-16s %8s %10s %8s %8s %8s" % ("channel", "rows", "frac>0.1", "mean", "p95", "max"))
    for name, path, cols in rows:
        if not os.path.isfile(path):
            print("   %-16s  <missing %s>" % (name, os.path.basename(path)))
            continue
        n, nz, mean, mx, p95 = census(path, cols)
        print("   %-16s %8d %10.3f %8.3f %8.3f %8.3f"
              % (name, n, nz / max(n, 1), mean, p95, mx))

    print()
    print("   reading: 'wire |v_l|' and 'plan box |v_l|' must both be non-zero for S1's PVA goal")
    print("   to be on the wire at all.  'frac>0.1' is the share of samples that carry a real")
    print("   velocity; 0.000 means the channel is present in type_mask and empty in the numbers.")


if __name__ == "__main__":
    main()
