#!/usr/bin/env python3
"""plan_clearance.py -- the clearance of the PLAN itself, not of the flight.

W4 failing is usually read as "the aircraft did not follow the plan".  This script asks the prior
question: did the PLAN keep any clearance at all?  It measures the distance from every reference sample
to the nearest trunk surface, with the trunk modelled as the box it is below z_split (half side r), which
is the same body W4 uses.

    python3 scripts/plan_clearance.py pv1-02-02 soft-02-02 d85-01-02
    python3 scripts/plan_clearance.py <tag> --z 2.0
"""
import argparse
import csv
import math
import os

import numpy as np
import yaml

ROOT = "/mnt/d/distill_self/super_super_apm"


def trunks(world):
    w = yaml.safe_load(open(world, encoding="utf-8"))
    g = w.get("pillars") or w.get("trunks") or []
    out = []
    for p in g:
        x, y = float(p["x"]), float(p["y"])
        r = float(p.get("r", p.get("radius", 0.1)))
        out.append((x, y, r))
    if out:
        return np.array(out)
    # some worlds store the grid instead
    for key in ("pillar_xy", "centres"):
        if key in w:
            return np.array([[float(a), float(b), float(w.get("pillar_radius_m", 0.1))]
                             for a, b in w[key]])
    raise SystemExit("no pillars found in %s" % world)


def load_plan(path):
    t, p = [], []
    for r in csv.DictReader(open(path)):
        if "t_wall" in r:
            t.append(float(r["t_wall"]))
        elif "t_sim_us" in r:
            t.append(float(r["t_sim_us"]) / 1e6)
        else:
            t.append(len(t))
        p.append((float(r["cmd_x"]), float(r["cmd_y"]), float(r["cmd_z"])))
    return np.array(p)


def clearance(pts, tr, z_split=2.1, box=True):
    """signed distance from each point to the nearest trunk surface, using each sample's OWN z."""
    d = np.hypot(pts[:, None, 0] - tr[None, :, 0], pts[:, None, 1] - tr[None, :, 1])
    r = tr[None, :, 2]
    dx = np.abs(pts[:, None, 0] - tr[None, :, 0]) - r
    dy = np.abs(pts[:, None, 1] - tr[None, :, 1]) - r
    outside = np.hypot(np.maximum(dx, 0), np.maximum(dy, 0))
    inside = np.minimum(np.maximum(dx, dy), 0)
    dist_box = np.where((dx <= 0) & (dy <= 0), inside, outside)
    dist_cyl = d - r
    use_box = (pts[:, None, 2] < z_split)
    if not box:
        use_box[:] = False
    dist = np.where(use_box, dist_box, dist_cyl)
    return dist.min(axis=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tags", nargs="+")
    ap.add_argument("--world", default=os.path.join(ROOT, "worlds/forest_v2_hex.yaml"))
    ap.add_argument("--z", type=float, default=2.0)
    a = ap.parse_args()
    tr = trunks(a.world)
    print("world %s: %d trunks, checking the plan at z = %.2f m (box below z_split 2.1)"
          % (os.path.basename(a.world), len(tr), a.z))
    print("%-11s %8s %8s %8s %8s %8s" % ("tag", "p01", "p10", "p50", "min", "<0 m"))
    for tag in a.tags:
        p = os.path.join(ROOT, "logs", tag, "cmd.csv")
        if not os.path.isfile(p):
            print("%-11s (no logs/%s/cmd.csv -- run scripts/prep_run.py first)" % (tag, tag))
            continue
        pts = load_plan(p)
        c = clearance(pts, tr)
        print("%-11s %8.3f %8.3f %8.3f %8.3f %7.1f%%" % (
            tag, np.percentile(c, 1), np.percentile(c, 10), np.percentile(c, 50), c.min(),
            100.0 * (c < 0).mean()))


if __name__ == "__main__":
    main()
