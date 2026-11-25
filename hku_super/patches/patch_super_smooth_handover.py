#!/usr/bin/env python3
"""patch_super_smooth_handover.py -- patch 14: a new goal first tries the CONTINUOUS replan path.

The defect
----------
MEASURED (2026-11-26, 1.35 m/s forest, 1 kg airframe): 44-82 `GenerateExpTrajectory failed` and 11-26
`Traj finish` per sortie.  Each `Traj finish` makes the FSM go WAIT_GOAL -> GENERATE_TRAJ ->
`PlanFromRest`: the plan box publishes its hold fallback while the planner restarts from rest, the
commanded speed collapses to 0 at the corner and the aircraft overshoots into a small loop
(scripts/flight_quality.py: 2-7 loops, 3-12 mid-flight command holds).

Why a new goal fails: `Fsm::callReplanOnce` passes `gi_.new_goal` straight into
`SuperPlanner::ReplanOnce`, whose `generateExpTraj` then treats the goal as a NEW one and recedes only
`cfg_.receding_dis` (1.0 m) along the OLD committed trajectory before running A* from that splice point
to the new goal.  A splice of a corner-turn is often infeasible.  Trying `split_dis = 0` instead
(patch 13, hku_super/patches/patch_super_new_goal_split.py) was measured WORSE (88 failures, 20 Traj
finish, 5 loops) and reverted.

The patch
---------
When a new goal arrives while FOLLOW_TRAJ, the FSM first calls `ReplanOnce(..., new_goal=false)`.
With `new_goal=false` the planner takes its ordinary CONTINUOUS path: if the committed trajectory is
known-free it uses the WHOLE committed trajectory as the guide (not a 1 m splice), appends an A* to
the (new) goal, and re-optimises -- i.e. the goal moves but the reference does not restart.  Only if
that returns a hard failure does the FSM fall back to the historical `new_goal=true` attempt, so a
regression here is bounded: the fallback is exactly today's behaviour.

NO_NEED / SUCCESS / FINISH from the smooth attempt are all accepted (they mean the planner has a
usable plan); FAILED/EMER/NEW_TRAJ fall through to the new-goal attempt.

Usage
-----
    python3 hku_super/patches/patch_super_smooth_handover.py --check
    python3 hku_super/patches/patch_super_smooth_handover.py --apply
    python3 hku_super/patches/patch_super_smooth_handover.py --revert
"""
from __future__ import annotations

import argparse
import os
import sys

DEFAULT_ROOT = os.environ.get("DSH_ROOT", "") + "/hku_super/super_planner"

ANCHOR = """        RET_CODE ret_code = planner_ptr_->ReplanOnce(gi_.goal_p, gi_.goal_yaw, gi_.new_goal,
                                                     gi_.goal_v, gi_.goal_a);"""

PATCHED = """        RET_CODE ret_code;
        if (gi_.new_goal) {
            /// WHTF-SMOOTH-HANDOVER (patch 14, 2026-11-26).  A NEW GOAL is first handed to the ordinary
            /// CONTINUOUS replan (new_goal=false): that path uses the WHOLE committed trajectory as the
            /// guide instead of receding 1 m along it and splicing, which is what fails at corner turns
            /// (44-82 GenerateExpTrajectory failures / sortie, each one ending in a Traj finish ->
            /// PlanFromRest -> commanded-speed collapse and an aircraft loop).  The historical new-goal
            /// attempt is kept as the fallback, so a regression is bounded to today's behaviour.
            RET_CODE smooth = planner_ptr_->ReplanOnce(gi_.goal_p, gi_.goal_yaw, false,
                                                       gi_.goal_v, gi_.goal_a);
            if (smooth == SUCCESS || smooth == FINISH || smooth == NO_NEED) {
                ret_code = smooth;
            } else {
                ret_code = planner_ptr_->ReplanOnce(gi_.goal_p, gi_.goal_yaw, true,
                                                    gi_.goal_v, gi_.goal_a);
            }
        } else {
            ret_code = planner_ptr_->ReplanOnce(gi_.goal_p, gi_.goal_yaw, gi_.new_goal,
                                                gi_.goal_v, gi_.goal_a);
        }"""

MARK = "WHTF-SMOOTH-HANDOVER (patch 14"


def main() -> int:
    ap = argparse.ArgumentParser(description="patch 14: new goal tries the continuous replan first")
    ap.add_argument("--root", default=DEFAULT_ROOT)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--check", action="store_true")
    g.add_argument("--apply", action="store_true")
    g.add_argument("--revert", action="store_true")
    a = ap.parse_args()
    path = os.path.join(a.root, "src", "super_core", "fsm.cpp")
    if not os.path.isfile(path):
        print("no such file: %s" % path, file=sys.stderr)
        return 2
    src = open(path, encoding="utf-8").read()
    if a.check:
        print("patch_super_smooth_handover: %s" % ("is APPLIED" if MARK in src else "is NOT APPLIED"))
        return 0 if MARK in src else 1
    if a.apply:
        if MARK in src:
            print("already applied")
            return 0
        if ANCHOR not in src:
            print("ANCHOR NOT FOUND -- refusing to guess", file=sys.stderr)
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
            print("revert failed", file=sys.stderr)
            return 4
        open(path, "w", encoding="utf-8", newline="\n").write(src)
        print("reverted -> %s" % path)
        return 0
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
