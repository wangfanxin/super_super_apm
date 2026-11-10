#!/usr/bin/env python3
"""patch_super_no_backward_head.py -- patch 11: refuse a plan whose first move is BACKWARDS.

The defect
----------
`docs/INTEGRATION.md` §82 measured 7-10 published reference FOLDS per sortie: the commanded
direction reverses by >101 deg while |v| is about 0.2 m/s.  `scripts/wire_fold.py` proved they are
not splice jumps (the largest single step in the whole sortie is 1.6x the median), so the plan
really doubles back.  Two families:

  (A) 0.4-0.8 s after a leg handover, at the corner (0.02-0.06 m from it)
  (B) 5.3-5.6 s into the leg, 4-5 m from any corner, with `PathSearch`/`Minco ... failed` around it
      -- the old plan keeps being published while the new one is unplannable, and when a plan
      finally comes out it starts by going back the way the vehicle came.

Half of them are family (B), and that family has a clean acceptance test: the planner already
receives, in `pos_init_state.col(1)`, the velocity the vehicle has at the replan state.  A new plan
whose first 0.6 s moves against that velocity is not a plan worth publishing.

The patch
---------
In `generateExpTraj`, after the optimiser succeeds and before the plan is stitched onto the prefix:
sample the new trajectory at t=0 and t=0.6 s (clamped to its own duration), project the net
horizontal displacement onto the current velocity, and if it is negative return FAILED.  The FSM then
keeps following the committed trajectory (`ReplanOnce` FAILED does not change state), which is
strictly better than publishing a u-turn.

Guards, so this can never deadlock:
  * skipped when there is no old trajectory (`PlanFromRest` after a `traj_finish`) -- the fold is
    then family (A), which this patch does not attempt to fix;
  * skipped when the old trajectory has less than 1.0 s left (a reversal is then the only way to
    reach the goal);
  * skipped when the vehicle is slower than 0.30 m/s (no meaningful heading to preserve).

Usage
-----
    python3 scripts/patch_super_no_backward_head.py --check
    python3 scripts/patch_super_no_backward_head.py --apply
    python3 scripts/patch_super_no_backward_head.py --revert
    python3 scripts/patch_super_no_backward_head.py --apply --root /mnt/d/distill_self/_stage/super_src
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

ANCHOR = """        if (!temp_ret) {
            ros_ptr_->warn(" -- [SUPER] OptimizationExpTrajInPolytopes for new path failed");
            return FAILED;
        }"""

PATCHED = ANCHOR + """
        /// WHTF-NO-BACKWARD-HEAD (patch 11): refuse a plan whose first move is BACKWARDS.
        /// MEASURED (docs/INTEGRATION.md §82): 7-10 published folds per sortie, half of them mid-leg
        /// 4-5 m from any corner, 5.3-5.6 s after the handover, with PathSearch/Minco failures
        /// around them: the old plan keeps being published while the new one is unplannable, and the
        /// plan that finally succeeds starts by going back the way the vehicle came.  The optimiser
        /// is handed the current velocity in pos_init_state.col(1), so "does the new trajectory
        /// honour it?" is a testable property of the plan, not a preference.
        if (!last_exp_traj_info.empty() && !out_traj.empty()) {
            const double remain = last_exp_traj.getTotalDuration() - replan_process_start_TT;
            const Vec3f v0 = pos_init_state.col(1);
            const double v0n = v0.norm();
            if (remain > 1.0 && v0n > 0.30) {
                const double head = std::min(0.6, out_traj.getTotalDuration());
                const Vec3f d = out_traj.getPos(head) - out_traj.getPos(0.0);
                const double along = d.head(2).dot(v0.head(2));
                if (along < 0.0) {
                    ros_ptr_->warn(" -- [SUPER] WHTF-NO-BACKWARD-HEAD: the new plan starts BACKWARDS "
                                   "({} m along a {} m/s heading) -- keeping the committed plan",
                                   along, v0n);
                    return FAILED;
                }
            }
        }"""

MARKER = "WHTF-NO-BACKWARD-HEAD"


def md5(p):
    return hashlib.md5(open(p, "rb").read()).hexdigest()


def state(root):
    p = os.path.join(root, "src/super_core/super_planner.cpp")
    txt = open(p).read()
    return txt, (MARKER in txt)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=DEFAULT_ROOT)
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--apply", action="store_true")
    ap.add_argument("--revert", action="store_true")
    a = ap.parse_args()
    p = os.path.join(a.root, "src/super_core/super_planner.cpp")
    txt, patched = state(a.root)

    if a.check or not (a.apply or a.revert):
        print("root %s" % a.root)
        print("  src/super_core/super_planner.cpp  md5 %s" % md5(p))
        print("RESULT: patch 11 is %s" % ("APPLIED" if patched else "NOT applied"))
        return 0 if (patched or a.check) else 0

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
