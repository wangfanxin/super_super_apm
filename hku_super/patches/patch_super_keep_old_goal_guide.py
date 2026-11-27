#!/usr/bin/env python3
"""patch_super_keep_old_goal_guide.py -- patch 22 (plan A): keep the previous corner as a guide interior point.

WHY.  Every corner stop has the same shape: the plan box dispatches the NEXT goal 0.3-0.8 m before the
current corner; the FSM hands that new goal to the CONTINUOUS replan (patch 14, smooth_rc=SUCCESS,
old trajectory remaining 0.01-0.07 s), but the guide is receded by cfg_.receding_dis (1.0 m): all old
trajectory points that are farther than 1.0 m ahead are dropped and an A* is run from the receded point.
The corner itself is therefore NOT in the new guide; the aircraft has to make the 60 deg turn from a
point ~1 m before the corner and MINCO brakes into it (measured dips 0.0-0.3 m/s at every corner).

Patch 22 (owner's plan A): on a NEW GOAL, do not recede -- keep the committed trajectory up to its END
(the corner) and let the A* start at the corner.  The corner becomes a guide vertex of the new
trajectory (still inside the same 3x3 m keepout, so W1b is untouched), the trajectory is continuous
through it, and no reference speed floor is needed.

Usage:
    python3 hku_super/patches/patch_super_keep_old_goal_guide.py --root hku_super/super_planner --check|--apply
"""
from __future__ import annotations

import argparse
import os
import sys

DEFAULT_ROOT = os.environ.get("DSH_ROOT", "") + "/hku_super/super_planner"
MARK = "WHTF-KEEP-OLD-GOAL (patch 22"

CFG_FIELD_ANCHOR = """        bool keep_old_goal_guide;"""
CFG_FIELD_PATCH = """        bool keep_old_goal_guide;"""
CFG_ADD_FIELD = """        double pva_tail_len;"""
CFG_ADD_FIELDS = CFG_ADD_FIELD + """

        /// WHTF-KEEP-OLD-GOAL (patch 22, plan A): at a new-goal handover keep the committed trajectory up
        /// to its END (the previous corner) as the guide prefix, so the corner stays a guide point.
        bool keep_old_goal_guide;"""
CFG_ADD_LOAD = """            loader.LoadParam("super_planner/pva_tail_len", pva_tail_len, 0.0);"""
CFG_ADD_LOADS = CFG_ADD_LOAD + """
            loader.LoadParam("super_planner/keep_old_goal_guide", keep_old_goal_guide, false);"""

CPP_ANCHOR = """            double split_dis = cfg_.receding_dis;
            if (last_exp_traj_info.wholeTrajKnownFree() && !gi_.new_goal && cfg_.receding_dis > 0.0) {
                split_dis = std::numeric_limits<double>::max();
            }"""
CPP_PATCH = """            double split_dis = cfg_.receding_dis;
            /// WHTF-KEEP-OLD-GOAL (patch 22, plan A, 2026-11-27).  For a NEW GOAL, drop no part of the
            /// committed trajectory: keep it up to its END (the previous corner) so the corner is a
            /// guide point of the new trajectory and the A* starts AT the corner instead of 1 m before
            /// it.  The corner stays inside the same keepout, so W1b is untouched.
            if ((!gi_.new_goal || cfg_.keep_old_goal_guide)
                && last_exp_traj_info.wholeTrajKnownFree() && cfg_.receding_dis > 0.0) {
                split_dis = std::numeric_limits<double>::max();
            }"""


def _read(p):
    with open(p, encoding="utf-8") as fh:
        return fh.read()


def _write(p, s):
    with open(p, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(s)


def main() -> int:
    ap = argparse.ArgumentParser(description="patch 22: keep old goal (corner) as a guide point")
    ap.add_argument("--root", default=DEFAULT_ROOT)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--check", action="store_true")
    g.add_argument("--apply", action="store_true")
    a = ap.parse_args()
    cfg = os.path.join(a.root, "include", "super_core", "config.hpp")
    cpp = os.path.join(a.root, "src", "super_core", "super_planner.cpp")
    for f in (cfg, cpp):
        if not os.path.isfile(f):
            print("no such file: %s" % f, file=sys.stderr)
            return 2
    cfg_s, cpp_s = _read(cfg), _read(cpp)
    applied = MARK in cfg_s and MARK in cpp_s
    if a.check:
        print("patch_super_keep_old_goal_guide: %s" % ("is APPLIED" if applied else "is NOT APPLIED"))
        return 0 if applied else 1
    if a.apply:
        if applied:
            print("already applied")
            return 0
        if CPP_ANCHOR not in cpp_s:
            print("ANCHOR NOT FOUND in super_planner.cpp -- refusing to guess", file=sys.stderr)
            return 3
        if CFG_ADD_FIELD not in cfg_s or CFG_ADD_LOAD not in cfg_s:
            print("ANCHOR NOT FOUND in config.hpp -- refusing to guess", file=sys.stderr)
            return 3
        cfg_s = cfg_s.replace(CFG_ADD_FIELD, CFG_ADD_FIELDS, 1).replace(CFG_ADD_LOAD, CFG_ADD_LOADS, 1)
        cpp_s = cpp_s.replace(CPP_ANCHOR, CPP_PATCH, 1)
        _write(cfg, cfg_s)
        _write(cpp, cpp_s)
        print("applied -> keep_old_goal_guide (new goals keep the previous corner in the guide)")
        return 0
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
