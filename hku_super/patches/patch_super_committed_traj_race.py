#!/usr/bin/env python3
"""patch_super_committed_traj_race.py -- patch 18: guard the one UNLOCKED read of the committed trajectory.

Why (2026-11-26, real SITL crash): `fsm_node` segfaulted twice (phase23b at the last leg, phase28a at leg 5)
inside `SuperPlanner::getCommittedPositionTrajectory()` -> `memcpy`, called from `pubCmdTimerCallback` on the
MultiThreadedExecutor thread.  The committed trajectory is written by the replan thread under
`lockCommittedTraj()/unlockCommittedTraj()` and read under the same lock in `getCommittedTrajectory()`, but
`pubCmdTimerCallback`'s "is there a committed trajectory?" check calls the getter WITHOUT the lock:

    const bool have_committed =
            planner_ptr_->getCommittedPositionTrajectory().getTotalDuration() > 0.0;

That copies a `Trajectory` (heap coefficients) while the replanner replaces it -> use-after-free / torn read
-> SIGSEGV, then the plan box goes silent and the supervisor aborts.  This patch takes the same lock around
that read; it is the only unlocked call site (fsm_ros1 has the same shape but is not built).

Usage:
    python3 hku_super/patches/patch_super_committed_traj_race.py --check|--apply|--revert
"""
from __future__ import annotations

import argparse
import os
import sys

DEFAULT_ROOT = os.environ.get("DSH_ROOT", "") + "/hku_super/super_planner"
ANCHOR = """            const bool have_committed =
                    planner_ptr_->getCommittedPositionTrajectory().getTotalDuration() > 0.0;"""
PATCHED = """            /// WHTF-COMMITTED-TRAJ-RACE (patch 18, 2026-11-26).  The replanner WRITES the committed
            /// trajectory under lockCommittedTraj(); this read was the one call site without the lock and
            /// it crashed SITL twice (phase23b, phase28a) in getCommittedPositionTrajectory()'s memcpy.
            bool have_committed = false;
            planner_ptr_->lockCommittedTraj();
            have_committed = planner_ptr_->getCommittedPositionTrajectory().getTotalDuration() > 0.0;
            planner_ptr_->unlockCommittedTraj();"""
MARK = "WHTF-COMMITTED-TRAJ-RACE (patch 18"


def _read(p):
    with open(p, encoding="utf-8") as fh:
        return fh.read()


def _write(p, s):
    with open(p, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(s)


def main() -> int:
    ap = argparse.ArgumentParser(description="patch 18: lock the committed-trajectory heartbeat read")
    ap.add_argument("--root", default=DEFAULT_ROOT)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--check", action="store_true")
    g.add_argument("--apply", action="store_true")
    g.add_argument("--revert", action="store_true")
    a = ap.parse_args()
    f = os.path.join(a.root, "include", "ros_interface", "ros2", "fsm_ros2.hpp")
    if not os.path.isfile(f):
        print("no such file: %s" % f, file=sys.stderr)
        return 2
    s = _read(f)
    applied = MARK in s
    if a.check:
        print("patch_super_committed_traj_race: %s" % ("is APPLIED" if applied else "is NOT APPLIED"))
        return 0 if applied else 1
    if a.apply:
        if applied:
            print("already applied")
            return 0
        if ANCHOR not in s:
            print("ANCHOR NOT FOUND -- refusing to guess", file=sys.stderr)
            return 3
        _write(f, s.replace(ANCHOR, PATCHED, 1))
        print("applied -> heartbeat read now takes lockCommittedTraj()")
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
