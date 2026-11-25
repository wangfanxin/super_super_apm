#!/usr/bin/env python3
"""patch_super_new_goal_split.py -- patch 13: a NEW GOAL replans from the vehicle, not from the old guide.

The defect
----------
MEASURED (2026-11-26, phase8a/8c/8e, 1 kg airframe at 1.35 m/s): 44-82 `GenerateExpTrajectory failed`
per sortie and 11-26 `Traj finish` events.  The FSM then falls into WAIT_GOAL -> GENERATE_TRAJ ->
`PlanFromRest`, the plan box publishes its hold fallback while the planner restarts from rest, the
commanded speed collapses to 0 at every corner handover and the aircraft overshoots into small loops
(`scripts/flight_quality.py`: 2-7 loops, 3-12 mid-flight holds).  The loop is planner-side: every
handover is a new goal, and `generateExpTraj` handles a new goal by RECEDING along the OLD committed
trajectory for `cfg_.receding_dis` (1.0 m) and then running A* from a point on that old path.  When the
old path and the new goal disagree (a corner turn), that stitched guide is often infeasible and the
optimiser fails; the committed trajectory then ends at the corner and the FSM goes through the
stop-and-restart path above.

The patch
---------
When `gi_.new_goal` is true, skip the receding splice: `split_dis = 0.0`, which the existing branch
already implements as "No need receding, just path search" -- the guide starts at `pos_init_state`
(the old trajectory's state `replan_forward_dt` ahead, i.e. WITH velocity) and A* goes straight to the
new goal.  No behaviour changes for ordinary replans (`new_goal == false`), and patch 9 keeps
publishing the committed trajectory while the new plan is being generated, so a failure here still
costs nothing except one retry next tick.

Usage
-----
    python3 hku_super/patches/patch_super_new_goal_split.py --check
    python3 hku_super/patches/patch_super_new_goal_split.py --apply
    python3 hku_super/patches/patch_super_new_goal_split.py --revert
"""
from __future__ import annotations

import argparse
import os
import sys

DEFAULT_ROOT = os.environ.get("DSH_ROOT", "") + "/hku_super/super_planner"

ANCHOR = """            double split_dis = cfg_.receding_dis;
            if (last_exp_traj_info.wholeTrajKnownFree() && !gi_.new_goal && cfg_.receding_dis > 0.0) {
                split_dis = std::numeric_limits<double>::max();
            }"""

PATCHED = ANCHOR + """
            /// WHTF-NEW-GOAL-SPLIT (patch 13, 2026-11-26).  A NEW GOAL must not be stitched onto a receding
            /// splice of the OLD trajectory: measured 44-82 GenerateExpTrajectory failures and 11-26
            /// `Traj finish` stop-and-restart handovers per 1.35 m/s sortie, each of them a commanded-speed
            /// collapse and an aircraft loop at a corner.  With split_dis = 0 the existing branch below takes
            /// "No need receding, just path search": the guide starts at pos_init_state (the committed
            /// trajectory's state replan_forward_dt ahead, i.e. with velocity) and A* goes straight to the new
            /// goal -- a fresh, feasible plan from the vehicle instead of a splice of a path it is leaving.
            if (gi_.new_goal) {
                split_dis = 0.0;
            }"""

MARK = "WHTF-NEW-GOAL-SPLIT (patch 13"


def main() -> int:
    ap = argparse.ArgumentParser(description="patch 13: new-goal replan skips the receding splice")
    ap.add_argument("--root", default=DEFAULT_ROOT)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--check", action="store_true")
    g.add_argument("--apply", action="store_true")
    g.add_argument("--revert", action="store_true")
    a = ap.parse_args()
    path = os.path.join(a.root, "src", "super_core", "super_planner.cpp")
    if not os.path.isfile(path):
        print("no such file: %s" % path, file=sys.stderr)
        return 2
    src = open(path, encoding="utf-8").read()
    if a.check:
        print("patch_super_new_goal_split: %s" % ("is APPLIED" if MARK in src else "is NOT APPLIED"))
        return 0 if MARK in src else 1
    if a.apply:
        if MARK in src:
            print("already applied")
            return 0
        if ANCHOR not in src:
            print("ANCHOR NOT FOUND -- the upstream source changed; refusing to guess", file=sys.stderr)
            return 3
        src = src.replace(ANCHOR, PATCHED, 1)
        open(path, "w", encoding="utf-8", newline="\n").write(src)
        print("applied -> %s" % path)
        return 0
    if a.revert:
        if MARK not in src:
            print("not applied, nothing to revert")
            return 0
        src = src.replace(PATCHED, ANCHOR, 1)
        if MARK in src:
            print("revert failed to remove the marker", file=sys.stderr)
            return 4
        open(path, "w", encoding="utf-8", newline="\n").write(src)
        print("reverted -> %s" % path)
        return 0
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
