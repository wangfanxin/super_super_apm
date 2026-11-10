#!/usr/bin/env python3
"""make_world_v2_hex.py -- the HEX world of docs/SPEC.md, with the geometry the spec now fixes.

Round 81 changes that this file implements (all of them are recorded in docs/SPEC.md):

  * the 0.20 m "safety radius" is GONE.  Nothing here assumes it.
  * **the cell that contains a waypoint's exact point carries no pillar** (S2's only allowed hole).
  * the pillar is a **0.2 m square below z_split (2.1 m) and a 0.2 m-diameter cylinder above it**, and the
    mission flies at 2.0 m, i.e. through the SQUARE part.  The simulation is box-below from round 81 too
    (`sim/raycast.py`), so the audit has to use the box, not a circle of the same nominal size: the box
    corner is 0.1414 m from the axis, the circle is 0.10 m.

The one number this file is built around is therefore the **surface clearance** d: distance from a point to
the nearest pillar surface, 0 inside.  Everything else follows from it, without a made-up "blocked radius":

    contact  d >= 0.00   the airframe touches (this is the W4 row accept.py measures)
    A*       d >= 0.05   what SUPER's A* can see: the map is NOT inflated (inflation_step 0), and its grid
                         is inflation_resolution = 0.1 m, so the search's resolution is ~half a cell
    CIRI     d >= 0.15   the corridor/minco stage uses robot_r = 0.15 m from the config

and, per leg, the single statistic that answers "is there a route at all": the **widest bottleneck** b =
max over routes of (min clearance along the route).  b >= t  <=>  a route exists at threshold t, so one
number per leg answers all three yardsticks.

    python3 scripts/make_world_v2_hex.py --pitch 1.0 --cols 8
"""
import argparse
import heapq
import os
import re
import sys

import numpy as np
import yaml

ap = argparse.ArgumentParser()
ap.add_argument("--pitch", type=float, default=1.0, help="cell size (SPEC S2: 1.0 m)")
ap.add_argument("--cols", type=int, default=12, help="cells per side of one quadrant: cols*pitch = half field")
ap.add_argument("--r", type=float, default=0.10, help="pillar half-side (box) / radius (cylinder), m")
ap.add_argument("--radius", type=float, default=10.0, help="hexagon circumradius = corner-to-pad distance")
ap.add_argument("--laps", type=int, default=3)
ap.add_argument("--z", type=float, default=2.0, help="mission altitude (audited at this z)")
ap.add_argument("--z-split", type=float, default=2.1, help="box below, cylinder above (m)")
ap.add_argument("--z0", type=float, default=0.0)
ap.add_argument("--z1", type=float, default=4.2)
ap.add_argument("--seed", type=int, default=20240521)
ap.add_argument("--overlap-m", type=float, default=0.02, help="min surface gap between two pillars (m)")
ap.add_argument("--pad-clear", type=float, default=0.35, help="no pillar centre closer than this to (0,0)")
ap.add_argument("--keepout-wp", type=int, default=1,
                help="1 = the cell that contains a waypoint's exact point gets no pillar (SPEC S2)")
ap.add_argument("--keepout-neigh", type=int, default=1,
                help="half-width, in cells, of the empty block around each waypoint: 0 = the waypoint's "
                     "own cell only (the frozen rule), 1 = 3x3 (9 cells, the owner's round-87 "
                     "revision), 2 = 5x5 (25 cells).  This is a CHANGE OF RULE when > 0.")
ap.add_argument("--arena-m", type=float, default=0.0,
                help="if >0, set arena.walls to +-this many metres (the template's walls are +-11); "
                     "use it when the field is bigger than the old arena")
ap.add_argument("--keep-prob", type=float, default=1.0,
                help="fraction of the non-waypoint cells that actually get a pillar (default 1.0 = "
                     "SPEC S2's 'one pillar per cell', the frozen world).  <1 thins the forest; the "
                     "cells that are dropped are drawn from the same seed, so the result is "
                     "reproducible.  This is a CHANGE OF RULE when <1 -- the owner has to approve it "
                     "and docs/SPEC.md S2 gets a modification row.")
ap.add_argument("--template", default="/mnt/d/distill_self/super_super_apm/worlds/square15_forest.yaml")
ap.add_argument("--cfg", default="/mnt/d/distill_self/super_super_apm/config/super_planner_hex.yaml")
ap.add_argument("--out", default="/mnt/d/distill_self/super_super_apm/worlds/forest_hex12.yaml",
                help="默认写 SPEC 冻结的新图；旧的 forest_v2_hex.yaml 已归档，不要用默认参数覆盖它")
a = ap.parse_args()

# ---- what the planner's own config says (read, not assumed) -------------------------------------
txt = open(a.cfg, encoding="utf-8").read() if os.path.isfile(a.cfg) else ""


def cfgval(key, default):
    m = re.search(r"^[ \t]*%s:[ \t]*([0-9.eE+-]+)" % key, txt, re.M)
    return float(m.group(1)) if m else default


res = cfgval("resolution", 0.05)
inf_step = cfgval("inflation_step", 2)
inf_res = cfgval("inflation_resolution", 0.1)
robot_r = cfgval("robot_r", 0.2)
# The three yardsticks, each read from the config rather than assumed (round 81):
#   A*    the search runs on the INFLATED map at `inflation_resolution`, so its clearance is
#         inflation_step * inflation_resolution, and the 0.1 m grid adds up to half a cell on top.
#   CIRI  the corridor stage inflates by robot_r and its seed-line test (`isLineFree` with
#         `seed_line_neighbour` = ceil(robot_r/resolution) voxels) needs the same robot_r of raw clearance.
#   contact  zero.
map_inf = inf_step * inf_res
astar_clear = map_inf
astar_clear_hi = map_inf + inf_res / 2.0
ciri_clear = robot_r

# ---- the hexagon: 6 corners, each exactly `radius` from the pad ---------------------------------
ang = np.arange(6) * (np.pi / 3.0)
corner_xy = np.stack([a.radius * np.cos(ang), a.radius * np.sin(ang)], axis=1)
r_from_pad = np.hypot(corner_xy[:, 0], corner_xy[:, 1])
assert np.allclose(r_from_pad, a.radius, atol=1e-9), "a hexagon corner is not 5 m from the pad"
angles = [np.degrees(np.arctan2(c[1], c[0])) % 360.0 for c in corner_xy]
assert np.allclose(np.sort(angles), 60.0 * np.arange(6), atol=1e-6), \
    "the six corners are not 60 degrees apart"

# ---- which cells exist, and which one carries a waypoint ----------------------------------------
half_field = a.cols * a.pitch
cells_all = [(i, j) for i in range(-a.cols, a.cols) for j in range(-a.cols, a.cols)]


def cell_of(x, y):
    return (int(np.floor(x / a.pitch)), int(np.floor(y / a.pitch)))


wp_cells = sorted({cell_of(c[0], c[1]) for c in corner_xy})
unknown = [c for c in wp_cells if c not in cells_all]
assert not unknown, "a waypoint falls outside the field's cells: %s" % unknown
if a.keepout_neigh > 0:
    # the (2N+1)x(2N+1) block around each waypoint cell; N=1 is the owner's "航点附近 9 个格子"
    _n = int(a.keepout_neigh)
    _keep = set()
    for (ci, cj) in wp_cells:
        for di in range(-_n, _n + 1):
            for dj in range(-_n, _n + 1):
                _keep.add((ci + di, cj + dj))
    keep = _keep if a.keepout_wp else set()
    print("   waypoint keep-out        : %dx%d cells per waypoint (%d cells total)"
          % (2 * _n + 1, 2 * _n + 1, len(keep)))
else:
    keep = set(wp_cells) if a.keepout_wp else set()
cells = [c for c in cells_all if c not in keep]
if not (0.0 < a.keep_prob <= 1.0):
    print("--keep-prob must be in (0, 1]; got %g" % a.keep_prob)
    sys.exit(2)
if a.keep_prob < 1.0:
    # thin the field reproducibly: the SAME seed draws which cells keep a pillar
    _rng_k = np.random.default_rng(a.seed + 7919)
    _n = int(round(a.keep_prob * len(cells)))
    _pick = np.sort(_rng_k.choice(len(cells), size=_n, replace=False))
    cells = [cells[i] for i in _pick]

# ---- placement: uniform in cell, no overlap, not on the pad, never in a waypoint cell ------------
centres = np.array([[i * a.pitch + a.pitch / 2.0, j * a.pitch + a.pitch / 2.0] for (i, j) in cells])
lo = centres - a.pitch / 2.0
d_min = 2.0 * a.r + a.overlap_m
rng = np.random.default_rng(a.seed)
P = lo + rng.random(centres.shape) * a.pitch

for _ in range(3000):
    rd = np.hypot(P[:, 0], P[:, 1])
    near = rd < a.pad_clear
    if near.any():
        s = np.where(rd > 1e-9, a.pad_clear / np.maximum(rd, 1e-9), 1.0)
        P[near] *= s[near][:, None]
    # The pair floor is on the BOX gap, not on the centre distance: at z=2.0 the pillar is a square, so two
    # centres 0.22 m apart can be two OVERLAPPING squares (circle thinking would call that legal).
    DX = np.abs(P[:, None, 0] - P[None, :, 0])
    DY = np.abs(P[:, None, 1] - P[None, :, 1])
    GAP = np.hypot(np.maximum(DX - 2.0 * a.r, 0.0), np.maximum(DY - 2.0 * a.r, 0.0))
    D = np.hypot(DX, DY)
    np.fill_diagonal(GAP, np.inf)
    viol = GAP < a.overlap_m
    if not viol.any():
        break
    disp = np.zeros_like(P)
    for i, j in np.argwhere(np.triu(viol)):
        u = (P[i] - P[j]) / max(D[i, j], 1e-9)
        push = (a.overlap_m - GAP[i, j]) / 2.0 + 1e-4
        disp[i] += u * push
        disp[j] -= u * push
    P = P + 0.5 * disp
D = np.hypot(P[:, None, 0] - P[None, :, 0], P[:, None, 1] - P[None, :, 1])
DX = np.abs(P[:, None, 0] - P[None, :, 0])
DY = np.abs(P[:, None, 1] - P[None, :, 1])
GAP = np.hypot(np.maximum(DX - 2.0 * a.r, 0.0), np.maximum(DY - 2.0 * a.r, 0.0))
np.fill_diagonal(GAP, np.inf)
np.fill_diagonal(D, np.inf)
assert GAP.min() >= a.overlap_m - 1e-6, "pillars overlap: box gap %.4f < %.4f" % (GAP.min(), a.overlap_m)
assert np.hypot(P[:, 0], P[:, 1]).min() >= a.pad_clear - 1e-6, "a pillar sits on the pad"
# The pair-floor repair may push a pillar past its own cell edge.  S2 says the centre is inside its cell,
# so such a draw is INVALID and is rejected rather than silently accepted (exit 3; the caller must pick
# another seed).
for k, (i, j) in enumerate(cells):
    in_x = i * a.pitch - 1e-9 <= P[k, 0] <= (i + 1) * a.pitch + 1e-9
    in_y = j * a.pitch - 1e-9 <= P[k, 1] <= (j + 1) * a.pitch + 1e-9
    if not (in_x and in_y):
        print("INVALID draw (seed %d): the pair-floor repair pushed pillar %d out of its %.2f m cell "
              "(x %+0.4f, y %+0.4f; cell [%d, %d]); SPEC S2 forbids that, so this seed is rejected."
              % (a.seed, k, a.pitch, P[k, 0], P[k, 1], i, j))
        sys.exit(3)

# ---- the clearance field: distance to the BOX surface at the mission altitude --------------------
# Box (not circle): at z < z_split the pillar is an axis-aligned square of half-side r, so its corner is
# r*sqrt(2) = 0.1414 m from the axis.  `d` is 0 inside, and the true (negative) inside value is not needed
# because every threshold below is >= 0.
assert a.z < a.z_split, ("the mission altitude %.2f m is in the cylinder part (z_split %.2f); this audit's "
                         "box geometry would be wrong -- pass --z below --z-split" % (a.z, a.z_split))

g = 0.05
HALF = 11.0                                        # the audit grid: the field is +-8, routes may bow out
N = int(round(2 * HALF / g)) + 1
ax = np.arange(N) * g - HALF
X, Y = np.meshgrid(ax, ax, indexing="ij")
d2grid = np.full(X.shape, np.inf)
for (px, py) in P:
    ox = np.abs(X - px) - a.r
    oy = np.abs(Y - py) - a.r
    d2grid = np.minimum(d2grid, np.hypot(np.maximum(ox, 0.0), np.maximum(oy, 0.0)))
DV = d2grid                                        # surface clearance field (m), 0 = touching


def idx(x, y):
    return (int(round((x + HALF) / g)), int(round((y + HALF) / g)))


NEI = [(di, dj) for di in (-1, 0, 1) for dj in (-1, 0, 1) if (di, dj) != (0, 0)]


def astar(s, t, tmin):
    """Shortest grid route whose cells all keep >= tmin surface clearance."""
    free = DV >= tmin
    if not (free[s] and free[t]):
        return None, float("nan")
    h = lambda n: np.hypot(n[0] - t[0], n[1] - t[1]) * g
    openq = [(h(s), 0.0, s)]
    best = {s: 0.0}
    prev = {}
    while openq:
        f, gc, n = heapq.heappop(openq)
        if n == t:
            path = [n]
            while n in prev:
                n = prev[n]
                path.append(n)
            path.reverse()
            return path, gc
        if gc > best.get(n, np.inf) + 1e-12:
            continue
        i, j = n
        for di, dj in NEI:
            ni, nj = i + di, j + dj
            if not (0 <= ni < N and 0 <= nj < N) or not free[ni, nj]:
                continue
            if di and dj and not (free[i + di, j] and free[i, j + dj]):
                continue
            m = (ni, nj)
            c = gc + g * (1.41421356237 if (di and dj) else 1.0)
            if c < best.get(m, np.inf) - 1e-12:
                best[m] = c
                prev[m] = n
                heapq.heappush(openq, (c + h(m), c, m))
    return None, float("nan")


def widest(s, t):
    """max over routes of (min surface clearance along the route) -- one number per leg."""
    openq = [(-DV[s], s)]
    best = {s: DV[s]}
    done = set()
    while openq:
        nb, n = heapq.heappop(openq)
        b = -nb
        if n in done:
            continue
        done.add(n)
        if n == t:
            return b
        i, j = n
        for di, dj in NEI:
            ni, nj = i + di, j + dj
            if not (0 <= ni < N and 0 <= nj < N):
                continue
            if di and dj and not (DV[i + di, j] > 0 and DV[i, j + dj] > 0):
                continue
            m = (ni, nj)
            b2 = min(b, DV[m])
            if b2 > best.get(m, -1e9) + 1e-12:
                best[m] = b2
                heapq.heappush(openq, (-b2, m))
    return float("nan")


# ---- audit 1: is a waypoint cell really empty, and is the field still dense ----------------------
occupied = {cell_of(x, y) for (x, y) in P}
holes = sorted(c for c in cells_all if c not in occupied)
if a.keep_prob >= 1.0:
    assert set(holes) == keep, ("the field has holes that are not waypoint cells: %s (allowed only %s)"
                                % (sorted(set(holes) - keep), sorted(keep)))
else:
    # --keep-prob < 1 deliberately thins the field, which is NOT the frozen rule: the audit must say
    # so instead of pretending it is a waypoint-cell-only hole pattern.
    extra_holes = sorted(set(holes) - keep)
    print("   *** THINNED FIELD (--keep-prob %.2f): %d cells without a pillar, of which %d are NOT "
          "waypoint cells ***" % (a.keep_prob, len(holes), len(extra_holes)))
    print("   *** this is a change of SPEC S2's 'one pillar per cell' rule; the frozen world is "
          "reproduced only with --keep-prob 1.0 ***")

adj = []
cell_of_k = {c: k for k, c in enumerate(cells)}
for k, (i, j) in enumerate(cells):
    for di in (-1, 0, 1):
        for dj in (-1, 0, 1):
            if di == 0 and dj == 0:
                continue
            m = cell_of_k.get((i + di, j + dj))
            if m is not None and m > k:
                adj.append((k, m))
adj = np.array(adj)
# box-to-box surface gap between the two pillar CENTRES along each axis (boxes are axis aligned):
dx = np.abs(P[adj[:, 0], 0] - P[adj[:, 1], 0])
dy = np.abs(P[adj[:, 0], 1] - P[adj[:, 1], 1])
gap_adj = np.hypot(np.maximum(dx - 2 * a.r, 0.0), np.maximum(dy - 2 * a.r, 0.0))

# ---- audit 2: per-leg widest bottleneck + the A* route SUPER would fly --------------------------
tour = [(0.0, 0.0)] + [tuple(c) for c in corner_xy] + [tuple(corner_xy[0])]
names = ["pad->c1"] + ["c%d->c%d" % (i + 1, i + 2) for i in range(5)] + ["c6->c1"]
rows = []
for k in range(len(tour) - 1):
    s, t = idx(*tour[k]), idx(*tour[k + 1])
    straight = float(np.hypot(tour[k + 1][0] - tour[k][0], tour[k + 1][1] - tour[k][1]))
    b = widest(s, t)
    path, L = astar(s, t, astar_clear)
    route_min = float(min(DV[q] for q in path)) if path else float("nan")
    rows.append((names[k], straight, b, L, route_min))

print("== hex forest (SPEC.md S0/S2), %.1f m cells, box %.2f x %.2f m below z %.1f, no lane"
      % (a.pitch, 2 * a.r, 2 * a.r, a.z_split))
print("   pillars                    : %d in %d cells over [%.1f, %.1f]^2"
      % (len(P), len(cells), -half_field, half_field))
print("   waypoint cells kept empty  : %d %s" % (len(keep), wp_cells))
print("   every other cell has one   : %s"
      % ("yes" if set(holes) == keep else
         "NO -- thinned to %.0f %% (%d of %d non-waypoint cells)"
         % (100.0 * a.keep_prob, len(cells), len(cells_all) - len(keep))))
print("   pillar section             : box %.2f m (corner %.4f m from the axis) below z=%.1f, "
      "cylinder d %.2f m above" % (2 * a.r, a.r * np.sqrt(2), a.z_split, 2 * a.r))
print("   flight altitude            : %.2f m -> the SQUARE part" % a.z)
print("   surface gap (adjacent)     : min %.3f  p05 %.3f  median %.3f  p95 %.3f  max %.3f m"
      % (gap_adj.min(), np.percentile(gap_adj, 5), np.median(gap_adj),
         np.percentile(gap_adj, 95), gap_adj.max()))
print("   yardsticks                 : contact 0.00 | A* %.2f–%.2f (地图膨胀 %g×%.1f m，再加半格 %.1f m 栅格) | "
      "CIRI %.2f (robot_r + 种子线邻域)"
      % (astar_clear, astar_clear_hi, inf_step, inf_res, inf_res, ciri_clear))
print("   free-space fraction        : %.1f %% (d >= %.2f)" % (100.0 * (DV >= astar_clear).mean(), astar_clear))
print()
print("   per leg (0.05 m grid; b = widest bottleneck = max-min surface clearance):")
print("     %-8s %9s %s" % ("leg", "straight", "b (widest)   A* len  A* route min"))
for (nm, straight, b, L, rmin) in rows:
    print("     %-8s %7.2f m   %s   %s   %s"
          % (nm, straight,
             "unreachable" if b != b else "%+.3f m" % b,
             "n/a" if L != L else "%5.2f m" % L,
             "n/a" if rmin != rmin else "%+.3f m" % rmin))
n_contact = sum(1 for (_, _, b, _, _) in rows if b == b and b >= 0.0)
n_astar = sum(1 for (_, _, b, _, _) in rows if b == b and b >= astar_clear)
n_astar_hi = sum(1 for (_, _, b, _, _) in rows if b == b and b >= astar_clear_hi)
n_ciri = sum(1 for (_, _, b, _, _) in rows if b == b and b >= ciri_clear)
print()
print("   legs a route exists for    : contact %d/7 | A*(%.2f) %d/7 | A*(%.2f) %d/7 | CIRI(%.2f) %d/7"
      % (n_contact, astar_clear, n_astar, astar_clear_hi, n_astar_hi, ciri_clear, n_ciri))
print("   (b >= t  <=>  a route with at least t of surface clearance exists, so ONE number per leg")
print("    answers all three yardsticks; the two A* columns are the inflation alone and the inflation plus")
print("    the worst-case half-cell quantisation.)")

# ---- write the world ---------------------------------------------------------------------------
tpl = yaml.safe_load(open(a.template, encoding="utf-8"))
out = dict(tpl)
if a.arena_m > 0.0:
    _ar = dict(out.get("arena") or {})
    _ar["walls"] = {"x": [-a.arena_m, a.arena_m], "y": [-a.arena_m, a.arena_m]}
    out["arena"] = _ar
    print("   arena walls              : +-%.1f m (was the template's)" % a.arena_m)
out["pillars"] = [{"x": round(float(x), 4), "y": round(float(y), 4), "r": a.r, "z0": a.z0, "z1": a.z1}
                  for (x, y) in P]
out["pillar_geometry"] = {"type": "box_below_cylinder_above",
                          "box_half_side_m": a.r, "cylinder_radius_m": a.r,
                          "z_split": a.z_split,
                          "note": "SPEC S2: the pillar is a 0.2 m square below z_split and a 0.2 m "
                                  "diameter cylinder above; the mission flies at %.1f m (the box)" % a.z}
out["mission"] = dict(tpl["mission"])
out["mission"]["takeoff_alt"] = a.z
out["mission"]["corners"] = [[float(c[0]), float(c[1]), a.z] for c in corner_xy] * a.laps
mx = float(np.max(np.abs(corner_xy[:, 0])))
my = float(np.max(np.abs(corner_xy[:, 1])))
out["mission"]["square"] = [-mx, -my, mx, my]        # the hexagon's bounding box (used by the harness)
out["mission"]["shape"] = "hexagon"
out["mission"]["radius_m"] = a.radius
out["mission"]["laps"] = a.laps
out["mission"]["arrive_sphere_m"] = 0.5              # SPEC S1
out["mission"]["corner_exit_speed_mps"] = 4.0        # SPEC S1 (PVA goal)
out["v2"] = {"mode": "random-nolane-wp-cell-keepout", "seed": a.seed, "pitch": a.pitch,
             "cols": 2 * a.cols, "pad_clear": a.pad_clear, "overlap_m": a.overlap_m,
             "hex_radius_m": a.radius, "laps": a.laps,
             "waypoint_cells_empty": [list(c) for c in wp_cells],
             "keepout_neigh": int(a.keepout_neigh),
             "keepout_cells_empty": [list(c) for c in sorted(keep)],
             "gap_adj_min": round(float(gap_adj.min()), 3),
             "gap_adj_median": round(float(np.median(gap_adj)), 3),
             "astar_clear_m": round(astar_clear, 3), "astar_clear_hi_m": round(astar_clear_hi, 3),
             "ciri_clear_m": round(ciri_clear, 3), "map_inflation_m": round(map_inf, 3),
             "legs_route": {"contact": n_contact, "astar": n_astar, "astar_hi": n_astar_hi,
                            "ciri": n_ciri},
             "leg_widest_m": {nm: (None if b != b else round(float(b), 3)) for (nm, _, b, _, _) in rows}}

hdr = """# GENERATED by make_world_v2_hex.py --radius {radius} --seed {seed} --pitch {pitch} --r {r}
# Do not hand-edit; re-run the generator.  The rules are docs/SPEC.md and they bind.
#
# PURPOSE: the HEX world of docs/SPEC.md.  A dense random pillar field over [-{half}, +{half}]^2 with NO
# mission lane, NO launch road -- the pillars cover the whole field -- and exactly {nkeep} holes: the cells
# that contain a waypoint's exact point (SPEC S2, round 81).
#
# GEOMETRY (SPEC S2, round 81): each pillar is a {side} m SQUARE below z={zsplit} m and a {side} m diameter
# CYLINDER above it; the mission flies at {z} m, i.e. through the square part.  The simulation implements
# exactly this (sim/raycast.py), so what the lidar sees is what this file draws.
#
# WHAT THE SEARCH SEES: the map is not inflated (inflation_step 0) and its grid is {infres} m, so the A*
# needs ~{astar} m of surface clearance; the corridor stage needs robot_r = {ciri} m.  Per-leg numbers are
# printed by the generator and stored in the v2: block below.
"""
with open(a.out, "w", encoding="utf-8") as f:
    f.write(hdr.format(radius=a.radius, seed=a.seed, pitch=a.pitch, r=a.r, half=half_field,
                       nkeep=len(keep), side="%.1f" % (2 * a.r), zsplit=a.z_split, z=a.z,
                       infres=inf_res, astar=astar_clear, ciri=ciri_clear,
                       ))
    f.write(yaml.safe_dump({k: v for k, v in out.items() if k != "pillars"},
                           default_flow_style=None, sort_keys=False))
    f.write("\npillars:\n")
    for q in out["pillars"]:
        f.write("  - {x: %+.4f, y: %+.4f, r: %.2f, z0: %.1f, z1: %.1f}\n"
                % (q["x"], q["y"], q["r"], q["z0"], q["z1"]))
print("\nwrote %s (%d pillars, %d waypoint cells empty)" % (a.out, len(P), len(keep)))
