#!/usr/bin/env python3
"""patch_super_pva_tail.py -- patch 15: a PVA goal gets a SHORT TAIL past the goal along its terminal velocity.

Why
---
SUPER's trajectory endpoint is always `pos_fina_state.col(0) = guide_path.back()`, and the guide path ends
at `gi_.goal_p` (the corner).  So with the corner as the goal the reference must lose its speed at the
corner and the FSM then has to re-plan from there -- MEASURED (phase9f, flight_forensics.py): every corner
shows command speed dropping to 0.07-0.29 m/s, ~3 s dwell within 1.5 m, 0.2-0.76 m of back-and-forth.
`goal_pva_en` only gives the endpoint a non-zero VELOCITY; it does not move the endpoint, so MINCO still
has to finish at the corner.

Owner rejected the two easier escapes: (a) bigger map inflation (misinterprets the forest), (b) a real
via-point interface (new topic/FSM plumbing).  This patch is the planner-only version of "the trajectory
must not end on the corner": when the guide path has reached a PVA goal whose terminal velocity points
somewhere (`|goal_v_xy| > 0.1`), append a SHORT straight tail of `super_planner/pva_tail_len` metres along
`goal_v` and re-point the terminal position at the tail end.  The corner STAYS a guide/corridor interior
point (so W1b is untouched: the A* path still goes to the corner), the endpoint is now beyond it, and the
terminal velocity direction already equals the tail direction -- MINCO has a feasible straight run-out.

The tail is deliberately short (1.2 m in the delivered config): the 3x3 waypoint keep-out gives 1.5 m of
free space around the corner, so a 1.2 m tail lies in free space; corridor generation still checks it.

Usage
-----
    python3 hku_super/patches/patch_super_pva_tail.py --check
    python3 hku_super/patches/patch_super_pva_tail.py --apply
    python3 hku_super/patches/patch_super_pva_tail.py --revert
"""
from __future__ import annotations

import argparse
import os
import sys

DEFAULT_ROOT = os.environ.get("DSH_ROOT", "") + "/hku_super/super_planner"

# --------------------------------------------------------------------------------------------- anchors
CFG_ANCHOR = """        double receding_dis;"""
CFG_PATCHED = CFG_ANCHOR + """
        /// WHTF-PVA-TAIL (patch 15): length (m) of a straight run-out appended past a PVA goal along the
        /// goal's terminal velocity, so the trajectory does not END on the corner.  0 = historical behaviour.
        double pva_tail_len;"""

LOAD_ANCHOR = """            loader.LoadParam("super_planner/receding_dis", receding_dis, 5.0);"""
LOAD_PATCHED = LOAD_ANCHOR + """
            loader.LoadParam("super_planner/pva_tail_len", pva_tail_len, 0.0);"""

CPP_ANCHOR = """        const bool connected_goal = (guide_path.back().head(2) - gi_.goal_p.head(2)).norm() < cfg_.resolution * 2;
        out_exp_traj_info.setGoalConnectedFlag(connected_goal);"""
CPP_PATCHED = CPP_ANCHOR + """

        /// WHTF-PVA-TAIL (patch 15, 2026-11-26).  A PVA goal carries a terminal velocity, but SUPER's
        /// trajectory endpoint is `guide_path.back()`, which is the goal itself -- so MINCO has to finish
        /// AT the corner and the reference collapses there (MEASURED phase9f: command 0.07-0.29 m/s at
        /// every corner, ~3 s dwell, 0.2-0.76 m back-and-forth).  Append a short straight tail along the
        /// terminal velocity and re-point the terminal position at the tail end: the corner remains a
        /// guide/corridor interior point (so the waypoint is still passed), the endpoint moves past it.
        if (connected_goal && cfg_.goal_pva_en && cfg_.pva_tail_len > 0.0 && !guide_path.empty()) {
            Vec3f gv(gi_.goal_v[0], gi_.goal_v[1], 0.0);
            const double gvn = gv.norm();
            if (gvn > 0.10) {
                const Vec3f u = gv / gvn;
                const Vec3f tail_start = guide_path.back();
                const double stamp0 = guide_stamp.empty() ? 0.0 : guide_stamp.back();
                const double v_ref = std::max(cfg_.exp_traj_cfg.max_vel, 1e-3);
                for (int k = 1; k <= 2; ++k) {
                    const double d = cfg_.pva_tail_len * 0.5 * static_cast<double>(k);
                    guide_path.emplace_back(tail_start + u * d);
                    guide_stamp.emplace_back(stamp0 + d / v_ref);
                }
                if (cfg_.print_log) {
                    ros_ptr_->info(" -- [SUPER] PVA-TAIL: appended %.2f m run-out past the goal along "
                                   "(%.2f, %.2f)", cfg_.pva_tail_len, u.x(), u.y());
                }
            }
        }"""

MARK = "WHTF-PVA-TAIL (patch 15"


def _read(path):
    with open(path, encoding="utf-8") as fh:
        return fh.read()


def _write(path, s):
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(s)


def main() -> int:
    ap = argparse.ArgumentParser(description="patch 15: PVA tail past the goal")
    ap.add_argument("--root", default=DEFAULT_ROOT)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--check", action="store_true")
    g.add_argument("--apply", action="store_true")
    g.add_argument("--revert", action="store_true")
    a = ap.parse_args()
    cfg = os.path.join(a.root, "include", "super_core", "config.hpp")
    cpp = os.path.join(a.root, "src", "super_core", "super_planner.cpp")
    for f in (cfg, cpp):
        if not os.path.isfile(f):
            print("no such file: %s" % f, file=sys.stderr)
            return 2
    cfg_s, cpp_s = _read(cfg), _read(cpp)
    applied = (MARK in cfg_s) and (MARK in cpp_s)
    if a.check:
        print("patch_super_pva_tail: %s" % ("is APPLIED" if applied else "is NOT APPLIED"))
        return 0 if applied else 1
    if a.apply:
        if applied:
            print("already applied")
            return 0
        for name, s, anchor, patched in (("config.hpp", cfg_s, CFG_ANCHOR, CFG_PATCHED),
                                         ("super_planner.cpp", cpp_s, CPP_ANCHOR, CPP_PATCHED)):
            if anchor not in s:
                print("ANCHOR NOT FOUND in %s -- refusing to guess" % name, file=sys.stderr)
                return 3
        cfg_s = cfg_s.replace(CFG_ANCHOR, CFG_PATCHED, 1).replace(LOAD_ANCHOR, LOAD_PATCHED, 1)
        cpp_s = cpp_s.replace(CPP_ANCHOR, CPP_PATCHED, 1)
        _write(cfg, cfg_s)
        _write(cpp, cpp_s)
        print("applied -> configuration + endpoint tail")
        return 0
    if a.revert:
        if not applied:
            print("not applied, nothing to revert")
            return 0
        cfg_s = cfg_s.replace(LOAD_PATCHED, LOAD_ANCHOR, 1).replace(CFG_PATCHED, CFG_ANCHOR, 1)
        cpp_s = cpp_s.replace(CPP_PATCHED, CPP_ANCHOR, 1)
        _write(cfg, cfg_s)
        _write(cpp, cpp_s)
        print("reverted")
        return 0
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
