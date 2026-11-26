#!/usr/bin/env python3
"""patch_super_continuous_new_goal.py -- patch 16: hand a FRESH goal to the CONTINUOUS replanner first.

Why
---
Every corner of every 1.35 m/s flight still has a stop-and-pivot: MEASURED (phase23c, flight_forensics)
command speed 0.04-0.35 m/s at each corner, ~3.1-3.7 s inside 1.5 m, 0.21-0.69 m of back-and-forth, and
5 reference cusps per sortie (plan box prints "CUSP ... the reference folded 120 deg in one tick").
The dispatcher is not the whole story: the FSM's GENERATE_TRAJ branch ALWAYS calls
`PlanFromRest(goal)` for a fresh goal -- a new rest-to-rest MINCO problem from the current state -- and
patch 14's continuous handover only exists in the replan timer, which cannot help when the new goal has
already moved the FSM into GENERATE_TRAJ.  Measured in phase25b/c/d/e: changing WHEN the goal is
dispatched (pass-by at the corner plane, 2 cm arrival radius, +-0.8 m pre-goal) never removes the dip.

What this patch does
--------------------
In GENERATE_TRAJ, for a fresh goal (`gi_.new_goal`), FIRST try the continuous replan
`ReplanOnce(goal, yaw, new_goal=false, ...)` -- the same call patch 14 verified -- and only fall back to
`PlanFromRest` when that fails.  On success the committed trajectory simply continues to the new goal
(and the old trajectory still reaches the corner, so W1b is unchanged); no rest-to-rest restart and no
commanded-speed collapse.  The first goal after take-off has no committed trajectory, so the continuous
call fails and the historical PlanFromRest path is taken, unchanged.

Usage
-----
    python3 hku_super/patches/patch_super_continuous_new_goal.py --check
    python3 hku_super/patches/patch_super_continuous_new_goal.py --apply
    python3 hku_super/patches/patch_super_continuous_new_goal.py --revert
"""
from __future__ import annotations

import argparse
import os
import sys

DEFAULT_ROOT = os.environ.get("DSH_ROOT", "") + "/hku_super/super_planner"
ANCHOR = """                int retcode = planner_ptr_->PlanFromRest(gi_.goal_p, gi_.goal_yaw, gi_.new_goal,
                                                         gi_.goal_v, gi_.goal_a);"""
PATCHED = """                /// WHTF-CONT-NEW-GOAL (patch 16, 2026-11-26).  A FRESH goal is first handed to the
                /// CONTINUOUS replanner (new_goal=false): patch 14 showed that path works while
                /// PlanFromRest restarts the trajectory from rest and the commanded speed collapses at
                /// every corner (MEASURED phase23c: 0.04-0.35 m/s, 3.1-3.7 s inside 1.5 m, 0.21-0.69 m
                /// back-and-forth, 5 reference cusps/sortie).  If there is no committed trajectory yet
                /// (the first goal after take-off) the continuous call fails and the historical
                /// PlanFromRest path below is taken unchanged.
                if (gi_.new_goal) {
                    RET_CODE cont = planner_ptr_->ReplanOnce(gi_.goal_p, gi_.goal_yaw, false,
                                                             gi_.goal_v, gi_.goal_a);
                    if (cont == SUCCESS || cont == FINISH || cont == NO_NEED) {
                        gi_.new_goal = false;
                        plan_from_rest_ = false;
                        publishPolyTraj();
                        ChangeState("MainFsmCallback", FOLLOW_TRAJ);
                        break;
                    }
                }
                int retcode = planner_ptr_->PlanFromRest(gi_.goal_p, gi_.goal_yaw, gi_.new_goal,
                                                         gi_.goal_v, gi_.goal_a);"""
MARK = "WHTF-CONT-NEW-GOAL (patch 16"


def _read(p):
    with open(p, encoding="utf-8") as fh:
        return fh.read()


def _write(p, s):
    with open(p, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(s)


def main() -> int:
    ap = argparse.ArgumentParser(description="patch 16: continuous handover for a fresh goal")
    ap.add_argument("--root", default=DEFAULT_ROOT)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--check", action="store_true")
    g.add_argument("--apply", action="store_true")
    g.add_argument("--revert", action="store_true")
    a = ap.parse_args()
    fsm = os.path.join(a.root, "src", "super_core", "fsm.cpp")
    if not os.path.isfile(fsm):
        print("no such file: %s" % fsm, file=sys.stderr)
        return 2
    s = _read(fsm)
    applied = MARK in s
    if a.check:
        print("patch_super_continuous_new_goal: %s" % ("is APPLIED" if applied else "is NOT APPLIED"))
        return 0 if applied else 1
    if a.apply:
        if applied:
            print("already applied")
            return 0
        if ANCHOR not in s:
            print("ANCHOR NOT FOUND -- refusing to guess", file=sys.stderr)
            return 3
        _write(fsm, s.replace(ANCHOR, PATCHED, 1))
        print("applied -> GENERATE_TRAJ tries the continuous handover first")
        return 0
    if a.revert:
        if not applied:
            print("not applied, nothing to revert")
            return 0
        _write(fsm, s.replace(PATCHED, ANCHOR, 1))
        print("reverted")
        return 0
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
