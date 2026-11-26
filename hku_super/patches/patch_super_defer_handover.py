#!/usr/bin/env python3
"""patch_super_defer_handover.py -- patch 19: DO NOT re-plan a new goal while the committed trajectory
still has the corner turn in it.

Measured evidence (phase34a/36a, 2026-11-27): at every corner the COMMANDED speed dips to 0.14-0.34 m/s
(smoothly, no hold: the published position never freezes).  The dip is in SUPER's new trajectory, not in
tracking: the arrival gate fires 0.3 m before the corner, the new goal immediately triggers a fresh
`PlanFromRest` from a state that is still heading for the corner, and MINCO -- which now has to make the
60 deg turn from 0.3 m away inside a narrow corridor -- slows to ~0.2 m/s before turning.  The OLD committed
trajectory already contains the smooth turn; re-planning throws it away.

Patch 19 defers the re-plan while the committed trajectory still has >0.08 s to run.  The old turn is flown
to its end (its terminal velocity is the plan box's corner speed), `Traj finish` then sends the FSM to
GENERATE_TRAJ, and PlanFromRest runs from the corner with the measured velocity (hot start / patch 17).

Usage:
    python3 hku_super/patches/patch_super_defer_handover.py --root hku_super/super_planner --check|--apply|--revert
"""
from __future__ import annotations

import argparse
import os
import sys

DEFAULT_ROOT = os.environ.get("DSH_ROOT", "") + "/hku_super/super_planner"
ANCHOR = """        TimeConsuming replan_once_time("replan_once_time", false);

        RET_CODE ret_code;
        if (gi_.new_goal) {"""
PATCHED = """        TimeConsuming replan_once_time("replan_once_time", false);

        /// WHTF-DEFER-HANDOVER (patch 19, 2026-11-27).  A new goal that arrives while the committed
        /// trajectory still contains the corner turn must NOT be re-planned yet: MEASURED phase34a/36a,
        /// re-planning 0.3 m before every corner makes MINCO slow the new trajectory to 0.14-0.34 m/s
        /// (a smooth commanded dip, not a hold) so the aircraft stops at every corner.  Fly the old turn
        /// to its end first; `Traj finish` then re-enters GENERATE_TRAJ with the new goal and the plan
        /// starts after the turn instead of in the middle of it.
        if (gi_.new_goal) {
            double _hb_swt = 0.0;
            bool _hb_fin = false;
            planner_ptr_->getOneHeartbeatTime(_hb_swt, _hb_fin);
            planner_ptr_->lockCommittedTraj();
            const double _hb_total = planner_ptr_->getCommittedPositionTrajectory().getTotalDuration();
            planner_ptr_->unlockCommittedTraj();
            const double _hb_elapsed = ros_ptr_->getSimTime() - _hb_swt;
            const double _hb_remain = _hb_total - _hb_elapsed;
            if (!_hb_fin && _hb_total > 0.0 && _hb_remain > 0.08 && _hb_remain < 2.0) {
                static int _hb_n = 0;
                _hb_n++;
                if (_hb_n % 50 == 1) {
                    cout << " -- [Fsm] DEFER-HANDOVER: new goal kept waiting, the committed turn still "
                            "has " << _hb_remain << " s to fly." << endl;
                }
                return;
            }
        }

        RET_CODE ret_code;
        if (gi_.new_goal) {"""
MARK = "WHTF-DEFER-HANDOVER (patch 19"


def _read(p):
    with open(p, encoding="utf-8") as fh:
        return fh.read()


def _write(p, s):
    with open(p, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(s)


def main() -> int:
    ap = argparse.ArgumentParser(description="patch 19: defer new-goal re-planning until the old turn is flown")
    ap.add_argument("--root", default=DEFAULT_ROOT)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--check", action="store_true")
    g.add_argument("--apply", action="store_true")
    g.add_argument("--revert", action="store_true")
    a = ap.parse_args()
    f = os.path.join(a.root, "src", "super_core", "fsm.cpp")
    if not os.path.isfile(f):
        print("no such file: %s" % f, file=sys.stderr)
        return 2
    s = _read(f)
    applied = MARK in s
    if a.check:
        print("patch_super_defer_handover: %s" % ("is APPLIED" if applied else "is NOT APPLIED"))
        return 0 if applied else 1
    if a.apply:
        if applied:
            print("already applied")
            return 0
        if ANCHOR not in s:
            print("ANCHOR NOT FOUND -- refusing to guess", file=sys.stderr)
            return 3
        _write(f, s.replace(ANCHOR, PATCHED, 1))
        print("applied -> new goals defer while the committed corner turn is still being flown")
        return 0
    if a.revert:
        if not applied:
            print("not applied, nothing to revert")
            return 0
        _write(f, s.replace(PATCHED, ANCHOR, 1))
        print("reverted")
        return 0
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
