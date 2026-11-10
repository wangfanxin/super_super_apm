#!/usr/bin/env python3
"""patch_super_sfc_margin.py -- patch 12: let the corridor keep a MARGIN, configurable, default off.

The defect
----------
`docs/INTEGRATION.md` §92: the PLAN itself grazes the pillars.  `scripts/plan_clearance.py` on five
sorties (reference samples vs the 522-pillar world, the same body W4 uses):

    tag          p01      p10      p50      min
    kxK-01-01   0.120    0.235    0.549    0.012
    kxK-02-01   0.139    0.317    0.570    0.086
    kxK-03-02   0.111    0.269    0.543   -0.002
    kxJ-01-01   0.090    0.261    0.547   -0.004
    kxJ-02-02   0.144    0.256    0.537    0.099

and W4's contact is only 0.04-0.10 m deeper than those minima, so W4 = plan_min + tracking excursion.
Two config knobs were tried and did NOT move plan_min:

  * `inflation_step 3` (0.30 m map): p01 stayed 0.11-0.14, min -0.002  (§92, arm rejected)
  * `robot_r 0.15`  (CIRI seed sphere): p01 -0.032, min -0.093, 2.4 % of samples inside a pillar --
    it moved the WRONG way (bigger seed sphere => the polytope is fitted further out, MINCO grazes the
    part of it that reaches the pillars).

That is consistent with how the corridor is built: CIRI's polytopes are fitted to the OCCUPANCY points,
so their boundary is the pillar surface, and MINCO (whose only clearance pressure is the soft
`penna_pos` term, `smooth_eps 0.01`) sits on that boundary.  `scripts/inflation_sweep.py` shows the
geometry is not the limit: the widest path per leg keeps 0.35-0.49 m.

The patch
---------
`traj_opt/boundary/sfc_margin` (default **0.0** = off, so every existing config behaves bit-identically),
applied in `exp_traj_optimizer_s4.cpp` right where the SFC planes are normalised for the optimiser.
Each half-space row is `n.p + d <= 0` with `|n| = 1` (`geometry_utils::findInterior` documents the sign),
so a plane is pulled in by exactly `sfc_margin` metres with `d += sfc_margin`.

Only planes with a (nearly) HORIZONTAL normal are pulled in (`|n_z| < 0.30`): the pillars are vertical,
so those are the planes that decide pillar clearance, and leaving the floor/ceiling planes alone keeps
the take-off/landing ends of the corridor and the aircraft's own start point inside the shrunk polytope
-- the failure mode this patch most has to avoid is "the head state is outside every piece".

`backup_traj` is not patched: `super_planner.backup_traj_en = false` in the delivered config.

Usage
-----
    python3 scripts/patch_super_sfc_margin.py --check
    python3 scripts/patch_super_sfc_margin.py --apply
    python3 scripts/patch_super_sfc_margin.py --revert
    python3 scripts/patch_super_sfc_margin.py --check --config config/super_planner_hex_rr15m.yaml
"""
from __future__ import annotations

import argparse
import hashlib
import os
import sys

DEFAULT_ROOT = "/home/fanxin/super_super_apm/upstream/SUPER/super_planner"

HPP = "include/traj_opt/config.hpp"
CPP = "src/traj_opt/exp_traj_optimizer_s4.cpp"

ANCHOR_FIELD = "        double penna_margin{0.05};"
PATCHED_FIELD = ANCHOR_FIELD + """
        /// WHTF-SFC-MARGIN (patch 12): pull every corridor half-space in by this much, in metres.
        /// 0.0 (the default) reproduces the unpatched optimiser exactly.  MEASURED reason: the plan's
        /// own p01 clearance is 0.09-0.14 m and its minimum 0.01 m (§92, plan_clearance.py), while the
        /// legs' widest paths keep 0.35-0.49 m (inflation_sweep.py) -- so the gap is optimiser
        /// behaviour, not geometry, and the soft SFC penalty alone does not close it.
        double sfc_margin{0.0};"""

ANCHOR_LOAD = (
    '            loader.LoadParam("traj_opt/boundary/penna_margin", penna_margin, 0.05);'
)
PATCHED_LOAD = ANCHOR_LOAD + """
            loader.LoadParam("traj_opt/boundary/sfc_margin", sfc_margin, 0.0);"""

ANCHOR_SETUP = """    for (long i = 0; i < sfcs.size(); i++) {
        opt_vars.hPolytopes[i] = sfcs[i].GetPlanes();
        const Eigen::ArrayXd norms = opt_vars.hPolytopes[i].leftCols<3>().rowwise().norm();
        opt_vars.hPolytopes[i].array().colwise() /= norms;
    }"""

PATCHED_SETUP = ANCHOR_SETUP + """

    /// WHTF-SFC-MARGIN (patch 12): shrink the corridor before the optimiser sees it.
    /// Each row is `n.p + d <= 0` with |n| = 1 after the normalisation above
    /// (`geometry_utils::findInterior` documents the sign), so `d += sfc_margin` pulls that face in by
    /// exactly `sfc_margin` metres.  Horizontal-normal planes only: the pillars are vertical and the
    /// floor/ceiling planes must not move, or the aircraft's own start point can end up outside every
    /// piece of a corridor that starts/ends near the ground.
    if (cfg_.sfc_margin > 1e-9) {
        long shrunk{0}, kept{0};
        for (long i = 0; i < sfcs.size(); i++) {
            auto &H = opt_vars.hPolytopes[i];
            for (long m = 0; m < H.rows(); m++) {
                if (H(m, 2) * H(m, 2) < 0.09) {          // |n_z| < 0.30, no <cmath> needed
                    H(m, 3) += cfg_.sfc_margin;
                    shrunk++;
                } else {
                    kept++;
                }
            }
        }
        if (cfg_.print_optimizer_log) {
            cout << " -- [SUPER] WHTF-SFC-MARGIN: pulled in " << shrunk << " lateral faces by "
                 << cfg_.sfc_margin << " m (" << kept << " vertical faces kept)" << endl;
        }
    }"""

MARKER = "WHTF-SFC-MARGIN"


def md5(p):
    return hashlib.md5(open(p, "rb").read()).hexdigest()


def read(root, rel):
    return open(os.path.join(root, rel)).read()


def applied(root):
    try:
        return MARKER in read(root, HPP) and MARKER in read(root, CPP)
    except OSError:
        return False


def replace_once(txt, old, new, what):
    if txt.count(old) != 1:
        raise SystemExit("REFUSING: %s anchor appears %d times" % (what, txt.count(old)))
    return txt.replace(old, new)


def replace_all(txt, old, new, what, expect):
    """The SFC-setup block exists TWICE in exp_traj_optimizer_s4.cpp: once for the default-init entry
    point and once for the guide-path entry point (lines ~898 and ~991).  Both hand the polytopes to the
    same optimiser, so both must be shrunk -- patching only one would make the margin depend on which
    entry point a replan happened to take, which is exactly the kind of hidden asymmetry this project
    keeps getting bitten by."""
    n = txt.count(old)
    if n != expect:
        raise SystemExit("REFUSING: %s anchor appears %d times (expected %d)" % (what, n, expect))
    return txt.replace(old, new)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=DEFAULT_ROOT)
    ap.add_argument("--config", default=None,
                    help="optional planner config, to print the margin this build would read")
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--apply", action="store_true")
    ap.add_argument("--revert", action="store_true")
    a = ap.parse_args()
    ok = applied(a.root)

    if a.check or not (a.apply or a.revert):
        print("root %s" % a.root)
        for rel in (HPP, CPP):
            p = os.path.join(a.root, rel)
            print("  %-46s md5 %s  marker %s"
                  % (rel, md5(p), "PRESENT" if MARKER in read(a.root, rel) else "absent"))
        print("RESULT: patch 12 is %s" % ("APPLIED" if ok else "NOT applied"))
        if a.config:
            try:
                import re
                t = open(a.config, encoding="utf-8").read()
                m = re.search(r"^[ \t]*sfc_margin:[ \t]*([0-9.eE+-]+)", t, re.M)
                print("  config %s: sfc_margin = %s" % (a.config, m.group(1) if m else "ABSENT (default 0.0 = off)"))
            except OSError as exc:
                print("  config unreadable: %s" % exc)
        return 0 if (ok or a.check) else 1

    if a.revert:
        if not ok:
            print("  already reverted")
            return 0
        p_h, p_c = os.path.join(a.root, HPP), os.path.join(a.root, CPP)
        t = read(a.root, HPP)
        for old, new in ((PATCHED_FIELD, ANCHOR_FIELD), (PATCHED_LOAD, ANCHOR_LOAD)):
            if new not in t:
                print("  REFUSING: hpp block not found verbatim", file=sys.stderr)
                return 1
            t = t.replace(old, new)
        open(p_h, "w").write(t)
        t = read(a.root, CPP)
        if t.count(PATCHED_SETUP) != 2:
            print("  REFUSING: cpp block found %d times (expected 2)" % t.count(PATCHED_SETUP), file=sys.stderr)
            return 1
        open(p_c, "w").write(t.replace(PATCHED_SETUP, ANCHOR_SETUP))
        print("  reverted %s + %s (2 SFC sites)" % (HPP, CPP))
        return 0

    if ok:
        print("  already patched")
        return 0
    p_h, p_c = os.path.join(a.root, HPP), os.path.join(a.root, CPP)
    # ATOMIC: build BOTH new texts first, and only write them when every anchor matched.  MEASURED
    # (round 19, first attempt): the hpp was written and the cpp refused, which left the header patched
    # once and the tree unbuildable; a second --apply then produced `redeclaration of sfc_margin`.
    th = read(a.root, HPP)
    th = replace_once(th, ANCHOR_FIELD, PATCHED_FIELD, "config.hpp field")
    th = replace_once(th, ANCHOR_LOAD, PATCHED_LOAD, "config.hpp loader")
    tc = read(a.root, CPP)
    tc = replace_all(tc, ANCHOR_SETUP, PATCHED_SETUP, "exp_traj_optimizer_s4.cpp setup", 2)
    open(p_h, "w").write(th)
    open(p_c, "w").write(tc)
    print("  patched %s + %s (2 SFC sites)" % (HPP, CPP))
    print("  now: %s" % ("APPLIED" if applied(a.root) else "NOT applied"))
    return 0 if applied(a.root) else 1


if __name__ == "__main__":
    sys.exit(main())
