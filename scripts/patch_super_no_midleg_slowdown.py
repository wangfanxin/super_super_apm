#!/usr/bin/env python3
"""patch_super_no_midleg_slowdown.py -- patch 13: a replan may not collapse the committed speed mid-leg.

The defect
----------
`docs/INTEGRATION.md` §109.3: `dip_context.py` on the round-20 arms shows **0 of 43 reference dips start
near a trajectory install** (median gap 6.20 s, 12 installs) -- they are SUPER **re-timing the remaining
path mid-leg**.  The thin layer follows the plan, so when the plan drops speed mid-leg the aircraft
overshoots and has to be pulled back: that is the along-track pendulum of §99.3 (p5 -0.19..-0.40 m) and
the dips (12-67 per sortie depending on arm).  The reference-level fix (`REF_GOV_DEC`, §fly_hex.sh:170)
was measured **worse** twice, because damping the reference's falling edge makes the aircraft oscillate
more; so the fix belongs on the planner side: do not publish a plan that collapses the speed.

The patch
---------
In `generateExpTraj`, after the optimiser succeeds and after patch 11's direction guard: the vehicle's
committed plan at the replan time is `last_exp_traj` evaluated at `replan_process_start_TT`.  If that
speed is above 0.80 m/s and the new plan's initial speed is below **55 %** of it, refuse the replan
(`return FAILED`); the FSM keeps following the committed trajectory, exactly as it does for patch 11.

Why this cannot deadlock: the guard only fires while the committed plan has more than **1.5 s** left.
When it runs out, `remain <= 1.5` and whatever plan comes next is accepted -- so the worst case is one
replan cycle of latency at the end of a leg, not a stuck vehicle.  Both guards are skipped when there is
no committed trajectory (PlanFromRest / after a traj_finish), where a speed drop is legitimate.

Usage
-----
    python3 scripts/patch_super_no_midleg_slowdown.py --check
    python3 scripts/patch_super_no_midleg_slowdown.py --apply
    python3 scripts/patch_super_no_midleg_slowdown.py --revert
"""
from __future__ import annotations

import argparse
import hashlib
import os
import sys

DEFAULT_ROOT = "/home/fanxin/super_super_apm/upstream/SUPER/super_planner"
REL = "src/super_core/super_planner.cpp"

ANCHOR = "        double replan_total_t = (ros_ptr_->getSimTime() - replan_process_start_WT);"

BLOCK = """        /// WHTF-NO-MIDLEG-SLOWDOWN (patch 13): a REPLAN may not collapse the committed speed mid-leg.
        /// MEASURED (docs/INTEGRATION.md §109.3): `dip_context.py` on six arms -- 0 of 43 reference dips
        /// start near a trajectory install, median gap 6.20 s, 12 installs -- the dips are SUPER
        /// RE-TIMING the remaining path, and the thin layer follows the plan, so the aircraft overshoots
        /// the slow patch and is pulled back (the along-track pendulum of §99.3).  Damping the
        /// reference's falling edge instead (`REF_GOV_DEC`) was measured worse twice, so the guard
        /// belongs here: the speed the vehicle already has committed is a property of the plan.
        /// Bounded: only while the committed plan has > 1.5 s left, so the vehicle can always finish it.
        if (!last_exp_traj_info.empty() && !out_traj.empty()) {
            const double remain = last_exp_traj.getTotalDuration() - replan_process_start_TT;
            const double v_old = last_exp_traj.getVel(replan_process_start_TT).head(2).norm();
            const double v_new = out_traj.getVel(0.0).head(2).norm();
            if (remain > 1.5 && v_old > 0.80 && v_new < 0.55 * v_old) {
                ros_ptr_->warn(" -- [SUPER] WHTF-NO-MIDLEG-SLOWDOWN: the new plan starts at {} m/s "
                               "against {} m/s already committed -- keeping the committed plan",
                               v_new, v_old);
                return FAILED;
            }
        }
"""

PATCHED = BLOCK + ANCHOR
MARKER = "WHTF-NO-MIDLEG-SLOWDOWN"


def md5(p):
    return hashlib.md5(open(p, "rb").read()).hexdigest()


def state(root):
    p = os.path.join(root, REL)
    txt = open(p).read()
    return txt, (MARKER in txt)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=DEFAULT_ROOT)
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--apply", action="store_true")
    ap.add_argument("--revert", action="store_true")
    a = ap.parse_args()
    p = os.path.join(a.root, REL)
    txt, patched = state(a.root)

    if a.check or not (a.apply or a.revert):
        print("root %s" % a.root)
        print("  %-46s md5 %s" % (REL, md5(p)))
        print("   anchor occurrences: %d (must be 1 to apply)" % txt.count(ANCHOR))
        print("RESULT: patch 13 is %s" % ("APPLIED" if patched else "NOT applied"))
        return 0

    if a.revert:
        if not patched:
            print("  already reverted")
            return 0
        if PATCHED not in txt:
            print("  REFUSING: patched block not found verbatim", file=sys.stderr)
            return 1
        open(p, "w").write(txt.replace(PATCHED, ANCHOR))
        print("  reverted %s" % p)
        return 0

    if patched:
        print("  already patched")
        return 0
    if txt.count(ANCHOR) != 1:
        print("  REFUSING: anchor appears %d times" % txt.count(ANCHOR), file=sys.stderr)
        return 1
    open(p, "w").write(txt.replace(ANCHOR, PATCHED))
    print("  patched %s" % p)
    txt2, ok = state(a.root)
    print("  now: %s" % ("APPLIED" if ok else "NOT applied"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
