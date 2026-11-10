#!/usr/bin/env python3
"""patch_super_replan_time_limit.py -- patch 10: give the re-planner a WALL-CLOCK budget of its own.

The bug (upstream SUPER, not ours)
----------------------------------
`super_planner.cpp` refuses a plan in two places when the *computation* took too long:

    line ~242  if (replan_dt       > cfg_.replan_forward_dt * 0.9) return FAILED;   // whole ReplanOnce
    line ~794  if (replan_total_t  > cfg_.replan_forward_dt)       return FAILED;   // generateExpTraj

but `replan_forward_dt` is not a time budget.  It is the lookahead in TRAJECTORY time used three
lines earlier (`replan_state_TT = replan_process_start_TT + cfg_.replan_forward_dt`).  One number is
being asked to mean two different things, so lowering the lookahead (this project lowered it
0.2 -> 0.1 for tracking) silently halved the planner's permission to think.

What that costs, measured
-------------------------
missionA-kwC-01-01 (`docs/INTEGRATION.md` §78, `scripts/leg_events.py`):

    PlanFromRest failed      149   (of which `Replan over time` 140)
    ReplanOnce overtime      123   at replan dt 0.108-0.135 s for a 0.09 s budget
    consequence              22 stops below 0.10 m/s in 156 s, a 5.5 s dead hover mid-leg,
                             every leg (re)started from rest

The plan was optimised, collision-checked and valid; it was thrown away for being 20 ms late, the
FSM retried at its own tick rate, the committed trajectory was never updated, and the published
reference froze.  `scripts/legspeed.py` shows the resulting v(s): 1.5 -> 0.0 -> 0.27 -> 0.96 m/s
inside one leg.

The patch
---------
Add `super_planner/replan_time_limit` (default 0.35 s) and compare against THAT in both places.
`replan_forward_dt` keeps its trajectory-time meaning; nothing else changes.  A late plan is still a
usable plan at 10 Hz -- patch 9 keeps publishing the committed trajectory while the optimizer runs.

Usage
-----
    python3 scripts/patch_super_replan_time_limit.py --check      # is it applied?
    python3 scripts/patch_super_replan_time_limit.py --apply     # patch (idempotent)
    python3 scripts/patch_super_replan_time_limit.py --revert    # put upstream back
    python3 scripts/patch_super_replan_time_limit.py --apply --root /mnt/d/distill_self/_stage/super_src
"""
from __future__ import annotations

# --- release-tree path bootstrap (git clone anywhere) ------------------------
import os as _os


def _dsh_root():
    d = _os.path.dirname(_os.path.abspath(__file__))
    while d != _os.path.dirname(d):
        if _os.path.isdir(_os.path.join(d, "harness")) and _os.path.isdir(_os.path.join(d, "third_party")):
            return d
        d = _os.path.dirname(d)
    return _os.environ.get("DSH_ROOT", "")


_DSH = _os.environ.get("DSH_ROOT") or _dsh_root()
# -----------------------------------------------------------------------------

import argparse
import hashlib
import os
import sys

DEFAULT_ROOT = _DSH + "/third_party/SUPER/super_planner"

# ---------------------------------------------------------------- the three edits, verbatim
HPP_OLD = """        double replan_forward_dt;
        double sample_traj_dt;"""
HPP_NEW = """        double replan_forward_dt;
        /// WHTF-REPLAN-TIME-LIMIT (patch 10): the WALL-CLOCK budget for one re-plan, in seconds.
        /// Upstream compares the computation time against `replan_forward_dt`, which is the
        /// TRAJECTORY-TIME lookahead (super_planner.cpp: replan_state_TT = start_TT + forward_dt).
        /// Those are different quantities: with the lookahead at 0.1 s the planner was discarding
        /// plans it had already finished whenever the host was even slightly loaded (measured:
        /// 123 + 140 refusals in missionA-kwC-01-01, which froze the published reference for up to
        /// 5.5 s at a time).  See docs/INTEGRATION.md section 78.
        double replan_time_limit;
        double sample_traj_dt;"""

# the anchor carries the NEXT loader line as context: without it, the "old" text is a prefix of the
# "new" text and a checker cannot tell the two states apart (the old line is still a substring).
LOAD_OLD = """            loader.LoadParam("super_planner/replan_forward_dt", replan_forward_dt, 0.3);
            loader.LoadParam("super_planner/corridor_bound_dis", corridor_bound_dis, 3.0);"""
LOAD_NEW = """            loader.LoadParam("super_planner/replan_forward_dt", replan_forward_dt, 0.3);
            loader.LoadParam("super_planner/replan_time_limit", replan_time_limit, 0.35);
            loader.LoadParam("super_planner/corridor_bound_dis", corridor_bound_dis, 3.0);"""

CPP_OLD_1 = """        double replan_dt = replan_total_t.stop();
        if (replan_dt > cfg_.replan_forward_dt * 0.9) {
            ros_ptr_->warn(" -- [SUPER] in [ReplanOnce]: Replan overtime, check parameters, replan dt = {}.", replan_dt);
            return FAILED;
        }"""
CPP_NEW_1 = """        double replan_dt = replan_total_t.stop();
        /// WHTF-REPLAN-TIME-LIMIT (patch 10).  This used to compare a WALL-CLOCK duration against
        /// `replan_forward_dt`, which is the TRAJECTORY-TIME lookahead -- one number, two meanings.
        /// At 0.1 s that leaves a 0.09 s budget, and MEASURED on missionA-kwC-01-01 this line alone
        /// refused 123 valid plans at replan dt 0.108-0.135 s.  Every refusal throws away a plan
        /// that was already optimised and collision-checked, so the committed trajectory is never
        /// updated, the published reference freezes, and the aircraft stops dead mid-leg.  The
        /// budget is now its own parameter and generous: a late plan is still a usable plan at
        /// 10 Hz, and patch 9 keeps publishing the committed trajectory while the optimizer runs.
        if (replan_dt > cfg_.replan_time_limit) {
            ros_ptr_->warn(" -- [SUPER] in [ReplanOnce]: Replan too slow ({} s > {} s), check host load.",
                           replan_dt, cfg_.replan_time_limit);
            return FAILED;
        }"""

CPP_OLD_2 = """        double replan_total_t = (ros_ptr_->getSimTime() - replan_process_start_WT);
        if (replan_total_t > cfg_.replan_forward_dt) {
            ros_ptr_->warn(" -- [SUPER] Replan over time({})!!!! Return FAILED", replan_total_t);
            return FAILED;
        }"""
CPP_NEW_2 = """        double replan_total_t = (ros_ptr_->getSimTime() - replan_process_start_WT);
        /// WHTF-REPLAN-TIME-LIMIT (patch 10): the same conflation as in ReplanOnce, with the same
        /// cost -- MEASURED 140 refusals in one sortie.  `replan_forward_dt` is a trajectory-time
        /// lookahead; the wall-clock budget is `replan_time_limit`.  See the long note there.
        if (replan_total_t > cfg_.replan_time_limit) {
            ros_ptr_->warn(" -- [SUPER] Replan over time({} s) > limit({} s)!!!! Return FAILED",
                           replan_total_t, cfg_.replan_time_limit);
            return FAILED;
        }"""

EDITS = [
    ("include/super_core/config.hpp", HPP_OLD, HPP_NEW),
    ("include/super_core/config.hpp", LOAD_OLD, LOAD_NEW),
    ("src/super_core/super_planner.cpp", CPP_OLD_1, CPP_NEW_1),
    ("src/super_core/super_planner.cpp", CPP_OLD_2, CPP_NEW_2),
]
MARKER = "WHTF-REPLAN-TIME-LIMIT"


def md5(path):
    return hashlib.md5(open(path, "rb").read()).hexdigest()


def inspect(root):
    """-> (n_patched_edits, list of problems)"""
    done, problems = 0, []
    for rel, old, new in EDITS:
        p = os.path.join(root, rel)
        if not os.path.exists(p):
            problems.append("missing %s" % p)
            continue
        txt = open(p).read()
        has_new, has_old = new in txt, old in txt
        if has_new and not has_old:
            done += 1
        elif has_old and not has_new:
            pass
        else:
            problems.append("%s: anchor state ambiguous (old=%s new=%s)" % (rel, has_old, has_new))
    return done, problems


def apply(root, revert=False):
    for rel, old, new in EDITS:
        p = os.path.join(root, rel)
        txt = open(p).read()
        src, dst = (new, old) if revert else (old, new)
        if dst in txt and src not in txt:
            print("  already %s: %s" % ("reverted" if revert else "patched", rel))
            continue
        if src not in txt:
            print("  REFUSING: anchor not found in %s" % rel, file=sys.stderr)
            return 1
        if txt.count(src) != 1:
            print("  REFUSING: anchor appears %d times in %s" % (txt.count(src), rel), file=sys.stderr)
            return 1
        open(p, "w").write(txt.replace(src, dst))
        print("  %s %s" % ("reverted" if revert else "patched", rel))
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=DEFAULT_ROOT)
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--apply", action="store_true")
    ap.add_argument("--revert", action="store_true")
    a = ap.parse_args()
    root = a.root

    if a.check or not (a.apply or a.revert):
        done, problems = inspect(root)
        print("root %s" % root)
        for rel in ("include/super_core/config.hpp", "src/super_core/super_planner.cpp"):
            p = os.path.join(root, rel)
            if os.path.exists(p):
                print("  %-38s md5 %s" % (rel, md5(p)))
        print("  edits applied: %d/%d" % (done, len(EDITS)))
        for pr in problems:
            print("  PROBLEM: %s" % pr)
        if done == len(EDITS):
            print("RESULT: patch 10 (WHTF-REPLAN-TIME-LIMIT) is APPLIED")
            return 0
        print("RESULT: patch 10 is NOT applied (upstream state)")
        return 0 if done == 0 and not problems else 1

    rc = apply(root, revert=a.revert)
    if rc == 0:
        done, problems = inspect(root)
        print("  now applied: %d/%d%s" % (done, len(EDITS),
                                          ("  problems: %s" % problems) if problems else ""))
        if not a.revert and MARKER not in open(os.path.join(root, EDITS[0][0])).read():
            print("  ERROR: config.hpp has no marker after apply", file=sys.stderr)
            return 1
    return rc


if __name__ == "__main__":
    sys.exit(main())
