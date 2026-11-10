"""corner_overshoot.py -- how much does the aircraft swing OUTSIDE the straight leg, corner to corner?

This is the metric the owner's question needs ("why is the overshoot obvious now, it was not before"),
because "distance 2 s after the corner" cannot tell "moved on to the next corner" from "swung wide".

For every consecutive pair of mission corners it takes the truth between the closest approach to corner i
and the closest approach to corner i+1, and reports:

    entry speed      the truth's speed at the closest approach to the corner (how fast it took the turn)
    bow              the maximum PERPENDICULAR distance from the chord c_i -> c_{i+1}
    bow / length     the same as a fraction of the leg

An arm that stops at each corner (round 79: advance only when within 0.15 m AND slower than 1.0 m/s) takes
the turn at ~0.3 m/s and bows a few centimetres.  An arm that passes through (round 85+: 0.5 m sphere at
any speed up to 6 m/s) takes it at 2-4 m/s and bows by up to metres.

    python3 scripts/corner_overshoot.py v4a-01 --world worlds/forest_v2_wide.yaml
    python3 scripts/corner_overshoot.py cmp2-01-01 --world worlds/compare_low.yaml
    python3 scripts/corner_overshoot.py pv1-02-02 --world worlds/forest_v2_hex.yaml
"""
import argparse
import csv
import math
import os

import numpy as np
import yaml

ROOT = "/mnt/d/distill_self/super_super_apm"


def load(prep):
    t, p, v = [], [], []
    for r in csv.DictReader(open(os.path.join(prep, "tap.csv"))):
        t.append(float(r["t_us"]) * 1e-6)
        p.append((float(r["pe"]), float(r["pn"]), -float(r["pd"])))
        v.append(math.hypot(float(r["ve"]), float(r["vn"])))
    return np.array(t), np.array(p), np.array(v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tag")
    ap.add_argument("--world", required=True)
    ap.add_argument("--prep", default="")
    a = ap.parse_args()
    prep = a.prep or os.path.join(ROOT, "logs", a.tag)
    w = yaml.safe_load(open(a.world, encoding="utf-8"))
    cs = []
    for c in w["mission"]["corners"]:
        c2 = np.array([float(c[0]), float(c[1])])
        if not any(np.linalg.norm(c2 - u) < 1e-6 for u in cs):
            cs.append(c2)
    corners = np.array(cs)

    t, P, V = load(prep)
    air = P[:, 2] > 1.0
    # closest-approach index to every corner, in time order
    ev = []
    for i, c in enumerate(corners):
        d = np.where(air, np.linalg.norm(P[:, :2] - c, axis=1), np.inf)
        k = int(np.argmin(d))
        ev.append((float(t[k]), i, k, float(d[k])))
    ev.sort()
    print("== %s  (world %s, %d corners)" % (a.tag, os.path.basename(a.world), len(corners)))
    print("   corner visits: %s" % " ".join("c%d@%.0fs(%.2fm)" % (i + 1, tt, dd) for tt, i, k, dd in ev))
    print("   %-9s %8s %8s %9s %9s" % ("leg", "length", "entry v", "bow", "bow/length"))
    bows, ents = [], []
    for m in range(len(ev) - 1):
        t0, i0, k0, d0 = ev[m]
        t1, i1, k1, d1 = ev[m + 1]
        if t1 <= t0 or i1 == i0:
            continue
        # only ADJACENT corners on the ring: a skipped corner (or a diagonal pairing) makes the "chord"
        # something the aircraft never flew, and the bow then measures the detour, not the corner
        if (i1 - i0) % len(corners) != 1:
            print("   c%d -> c%-2d skipped (not adjacent on the ring: %+d)" % (i0 + 1, i1 + 1, i1 - i0))
            continue
        A, B = corners[i0], corners[i1]
        seg = P[k0:k1 + 1]
        if len(seg) < 5:
            continue
        chord = B - A
        L = float(np.linalg.norm(chord))
        if L < 1e-6:
            continue
        u = chord / L
        n = np.array([-u[1], u[0]])
        bow = float(np.max(np.abs((seg[:, :2] - A) @ n)))
        bows.append(bow)
        ents.append(float(V[k0]))
        print("   c%d -> c%-2d %8.2f %8.2f %9.3f %9.3f"
              % (i0 + 1, i1 + 1, L, float(V[k0]), bow, bow / L))
    if bows:
        print("   ---")
        print("   entry speed at the corner : p50 %.2f  max %.2f m/s" % (np.percentile(ents, 50), max(ents)))
        print("   bow outside the chord     : p50 %.2f  max %.2f m" % (np.percentile(bows, 50), max(bows)))


if __name__ == "__main__":
    main()
