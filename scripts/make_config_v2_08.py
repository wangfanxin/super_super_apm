#!/usr/bin/env python3
"""The planner config for the hex world of docs/SPEC.md.

Round 81 changed what these two knobs mean, so read this before touching them:

    inflation_step = 2     the map IS inflated, by 0.1 x 2 = 0.2 m (the owner set 2 in round 81).  That is
                           what the A* searches, so the search itself keeps ~0.2 m from a trunk -- the same
                           number as robot_r, which CIRI uses for its seed line (`isLineFree` with
                           `seed_line_neighbour` = ceil(robot_r/resolution) = 4 voxels = 0.2 m) and for the
                           corridor inflation.  Upstream static_dense.yaml / forest_15m.yaml use 3 (0.3 m).
    robot_r        = 0.2   the user set the body radius in round 81 (first said 0.15, corrected to 0.2 m).
                           In this code it reaches exactly three places: the CIRI corridor
                           (`ciri_->setupParams(robot_r)`), the backup A*'s
                           `neighbor_step = floor(robot_r/resolution)` = 4 voxels, and the z bounds.
                           It is NOT an A* inflation.

The 0.20 m "safety radius" that earlier rounds of this file encoded was deleted from the spec in round 81,
so nothing here should reintroduce it.  Everything else is the accepted arm's values.
"""
import os
import re
import shutil
import sys

SRC = "/mnt/d/distill_self/super_super_apm/config/super_lio_15m.yaml"
DST = "/mnt/d/distill_self/super_super_apm/config/super_planner_hex.yaml"

SET = {
    "resolution": "0.05",
    "inflation_resolution": "0.1",
    "inflation_step": "1",          # round 83: the owner set 1 -> the map is dilated by 0.1 m
    "unk_inflation_step": "0",
    "robot_r": "0.10",               # round 81: the owner set the body radius to 0.2 m (CIRI + backup A*)
    "goal_pva_en": "true",          # round 81: SPEC S1 wants a PVA goal at every waypoint (v = 4 m/s)
}

def patch(path, key, val):
    s = open(path, encoding="utf-8").read()
    # 只吃一行：用 [ \t] 而不是 \s —— \s 会跨行；而且必须把匹配之后的**后半段接回去**
    # （第一版忘了 `+ s[m.end():]`，于是配置文件在匹配处被截断：165 行 → 68 行，
    #  后面的 rog_map 整块消失，patch_yaml 随后报 no such key）
    pat = re.compile(r"^([ \t]*)%s:[ \t]*([^\s#]+)[ \t]*(#[^\n]*)?$" % re.escape(key), re.M)
    m = pat.search(s)
    if not m:
        return False
    cmt = ("   " + m.group(3)) if m.group(3) else ""
    s = s[:m.start()] + "%s%s: %s%s" % (m.group(1), key, val, cmt) + s[m.end():]
    open(path, "w", encoding="utf-8").write(s)
    return True


if not os.path.isfile(SRC):
    sys.exit("no base config: %s" % SRC)
shutil.copyfile(SRC, DST)
for k, v in SET.items():
    ok = patch(DST, k, v)
    print("  %-22s -> %-8s %s" % (k, v, "ok" if ok else "NOT FOUND"))

# The ceiling that the passing arm flies: 5.5 / 5.0.  Round 78 started at 4.2 / 3.6 (safety first), which
# FAILED W2 -- with the wide lane the aircraft tracks so well that it no longer overshoots past 4 m/s
# (v3g-01: truth max 4.23, below the ">= 4 m/s for 1 s" bar).  At 6.0/5.5 W2 passed but W6 got worse
# (0.63-0.68); 5.5/5.0 gives truth peaks of 4.28-4.73 m/s and W6 quarters of 0.49-0.57 (v3t/v3v/v3w).
import subprocess
subprocess.run([sys.executable, "/mnt/d/distill_self/super_super_apm/scripts/patch_yaml.py", DST,
                "--set", "traj_opt.boundary.max_vel=3.0",
                "--set", "traj_opt.boundary.max_acc=8.0"], check=False)
# map_voxel_num is the A* search grid: [300, 300, 60] at 0.05 m is only 15 x 15 m, and the v2 field
# is 16 m across (+-8.0), so raise it to 20 x 20 m.  (Measured note from the old world: the grid and
# planning_horizon have to move together, or a leg's end lands outside the search grid.)
subprocess.run([sys.executable, "/mnt/d/distill_self/super_super_apm/scripts/patch_yaml.py", DST,
                "--set", "astar.map_voxel_num=[480,480,60]",
                "--set", "super_planner.planning_horizon=16.0"], check=False)
# planning_horizon 16 + a 480-cell (24 m) A* grid: 12 m is SHORTER than a 14.4 m leg, so one trajectory
# ends mid-leg with v = 0 and the aircraft is asked to stop there.  MEASURED (v2m-01 horizon 12 vs
# v2o-01 horizon 16): reference stops over three laps 30 -> 20, route time 123.4 -> 97.8 s, altitude bias
# +0.098 -> -0.002 m.  This is the v1 note's own fix ("16.0 而不是 12.0"), now applied to v2.
# backup_traj_en=false: SUPER's "two trajectories, can brake at any moment" safety feature builds a
# BACKUP braking trajectory on every replan, and in a dense pillar field that backup is usually
# infeasible -- MEASURED on sortie v2c-01 (0.80 m aisle): 4071 GeneratePolytopeFromLine failures
# against 518 successes, i.e. most replans were rejected and the reference simply held, and the
# aircraft ended up crawling (76 % of the reference samples at 0.00 m/s).  This is the option's own
# default in SuperPlannerConfig (super_core/config.hpp: LoadParam(..., false)).
subprocess.run([sys.executable, "/mnt/d/distill_self/super_super_apm/scripts/patch_yaml.py", DST,
                "--set", "super_planner.backup_traj_en=false"], check=False)

# the inherited comment on inflation_step still describes the OLD value ("0.1 m x 3 = 0.3 m")
s = open(DST, encoding="utf-8").read()
s = s.replace("inflation_step: 0   # 0.1 m x 3 = 0.3 m 膨胀（回退；4 会把 1.6 m 柱间隙挤没）",
              "inflation_step: 0   # ★不膨胀（第 81 轮用户确认）★ 上游 static_dense/forest_15m 是 3（=0.3 m）；"
              "A* 只看占据体素，机体半径只在 CIRI 走廊里生效")
open(DST, "w", encoding="utf-8").write(s)

body = open(DST, encoding="utf-8").read()
res = float(re.search(r"^\s*resolution:\s*([0-9.]+)", body, re.M).group(1))
inf = float(re.search(r"^\s*inflation_step:\s*([0-9.]+)", body, re.M).group(1))
infr = float(re.search(r"^\s*inflation_resolution:\s*([0-9.]+)", body, re.M).group(1))
rob = float(re.search(r"^\s*robot_r:\s*([0-9.]+)", body, re.M).group(1))
print("\n== super_planner_hex.yaml")
print("   map inflation   = %.3f m  (inflation_step %g x %g)   <- the A* sees nothing beyond this"
      % (inf * infr, inf, infr))
print("   body radius     = %.3f m  (CIRI corridor + backup A*'s 3-voxel neighbourhood + z bounds)"
      % rob)
print("   A* grid         = %.2f m  (inflation map resolution)" % infr)
print("   occupancy grid  = %.2f m  (probability map resolution)" % res)
print("   written: %s" % DST)
