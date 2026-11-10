#!/usr/bin/env python3
"""inflation_sweep.py -- does `inflation_step 2` (0.2 m) make the hexagon UNSOLVABLE?

The owner's worry, in his words: "改2我怀疑可能会导致无解区域很多".  The honest way to answer is not to
argue about the planner's internals but to measure the free space the planner has to work in:

  * the map the A* searches is the pillar field dilated by `inflation_step x 0.05` (= 0.05 m per step
    for the 0.05 m map) and the corridor stage additionally keeps `robot_r` (= 0.10 m);
  * so a leg is PLANNABLE at inflation `i` only if a path exists that stays at least `i + robot_r`
    from every pillar surface, and the leg is COMFORTABLE only if the widest such path has margin.
  * `b_max(leg)` below is the max-min clearance: the largest t such that a 4-connected path from
    corner k to corner k+1 exists in {distance-to-pillar-surface >= t}.  It is a property of the map
    alone, independent of SUPER's A*/MINCO.

    统一口径：柱面距离用「到 0.2 m 方柱表面的欧氏距离」，网格 0.05 m（与 rog_map.resolution 一致）。

    python3 scripts/inflation_sweep.py [--world worlds/forest_hex12.yaml]
"""
import argparse

import numpy as np
import yaml
from scipy.ndimage import distance_transform_edt, label

ap = argparse.ArgumentParser()
ap.add_argument("--world", default="worlds/forest_hex12.yaml")
ap.add_argument("--res", type=float, default=0.05)
ap.add_argument("--robot-r", type=float, default=0.10)
ap.add_argument("--steps", default="1,2,3,4", help="inflation_step values to evaluate")
ap.add_argument("--infl-res", type=float, default=0.10,
                help="metres per inflation step (rog_map.inflation_resolution)")
a = ap.parse_args()

w = yaml.safe_load(open(a.world, encoding="utf-8"))
pillars = w["pillars"]
P = [(float(q["x"]), float(q["y"]), float(q["r"])) for q in pillars]   # x, y, half-width
corners = np.array([[float(c[0]), float(c[1])] for c in w["mission"]["corners"]][:6])
X_LO, X_HI = float(w["arena"]["walls"]["x"][0]), float(w["arena"]["walls"]["x"][1])
Y_LO, Y_HI = float(w["arena"]["walls"]["y"][0]), float(w["arena"]["walls"]["y"][1])
res = a.res
nx = int(round((X_HI - X_LO) / res)) + 1
ny = int(round((Y_HI - Y_LO) / res)) + 1
gx = X_LO + res * np.arange(nx)
gy = Y_LO + res * np.arange(ny)

# occupancy: cell centre inside any pillar box
occ = np.zeros((ny, nx), dtype=bool)
for (x, y, h) in P:
    i0 = max(0, int(np.floor((x - h - X_LO) / res)))
    i1 = min(nx - 1, int(np.ceil((x + h - X_LO) / res)))
    j0 = max(0, int(np.floor((y - h - Y_LO) / res)))
    j1 = min(ny - 1, int(np.ceil((y + h - Y_LO) / res)))
    occ[j0:j1 + 1, i0:i1 + 1] = True
dt = distance_transform_edt(~occ, sampling=res)          # metres to the nearest occupied cell centre
print("world %s: %d pillars, %d x %d cells at %.2f m, %.1f %% of the field occupied"
      % (a.world, len(P), nx, ny, res, 100.0 * occ.mean()))


def cell(x, y):
    return (int(round((y - Y_LO) / res)), int(round((x - X_LO) / res)))


def connected(t, p0, p1):
    free = dt >= t
    lab, n = label(free, structure=np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]]))
    c0, c1 = cell(*p0), cell(*p1)
    if not free[c0] or not free[c1]:
        return False, n, lab
    return bool(lab[c0] == lab[c1] and lab[c0] > 0), n, lab


def bmax(p0, p1, hi=1.5):
    """largest clearance threshold at which the two points are still connected"""
    lo, best = 0.05, 0.0
    if not connected(lo, p0, p1)[0]:
        return 0.0
    for _ in range(14):
        mid = 0.5 * (lo + hi)
        if connected(mid, p0, p1)[0]:
            best, lo = mid, mid
        else:
            hi = mid
    return best


pad = (0.0, 0.0)
legs = [(pad, tuple(corners[0]))] + [(tuple(corners[i]), tuple(corners[i + 1])) for i in range(5)]
legs += [(tuple(corners[5]), tuple(corners[0]))]                    # the last 10 m leg of a lap
names = ["pad→c1"] + ["c%d→c%d" % (i + 1, i + 2) for i in range(5)] + ["c6→c1"]

print("\n== widest-path clearance b_max per leg (max over paths of the min distance to a pillar)")
print("   leg       b_max(m)   " + "  ".join("s=%s" % s for s in a.steps.split(",")))
steps = [int(s) for s in a.steps.split(",")]
rows = []
for (p0, p1), nm in zip(legs, names):
    b = bmax(p0, p1)
    marks = []
    for s in steps:
        need = s * a.infl_res + a.robot_r
        marks.append("OK  " if b >= need else "NO  ")
    rows.append((nm, b, marks))
    print("   %-9s %7.3f     %s" % (nm, b, " ".join("%s(%.2f)" % (m, s * a.infl_res + a.robot_r)
                                                   for m, s in zip(marks, steps))))

print("\n== how much free space each inflation leaves (whole field, and the 3x3 m clear square "
      "around each corner stays clear at every setting)")
print("   step  inflation  need(infl+robot_r)  free area %   free components   all legs solvable")
for s in steps:
    infl = s * a.infl_res
    need = infl + a.robot_r
    free = dt >= need
    lab, n = label(free, structure=np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]]))
    ok = all(connected(need, p0, p1)[0] for p0, p1 in legs)
    print("   %-5d %8.2f  %17.2f  %11.1f  %16d   %s"
          % (s, infl, need, 100.0 * free.mean(), n, "YES" if ok else "NO"))

print("\n== the narrowest passage on the widest path of each leg, at step 2 (0.20 m + robot_r 0.10)")
needed = 2 * a.infl_res + a.robot_r
for (p0, p1), nm in zip(legs, names):
    b = bmax(p0, p1)
    print("   %-9s b_max %.3f m  vs needed %.2f m  -> margin %+0.3f m"
          % (nm, b, needed, b - needed))
