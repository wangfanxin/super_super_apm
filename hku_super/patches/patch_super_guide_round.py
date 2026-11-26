#!/usr/bin/env python3
"""patch_super_guide_round.py -- patch 21: ROUND the guide path at sharp vertices before corridor/MINCO.

Round 13-14 diagnosis: the continuous handover SUCCEEDS (smooth_rc=SUCCESS, remain 0.01-0.07 s), yet the new
trajectory still slows to 0.0-0.2 m/s at every corner.  The guide path is a 60 deg POLYLINE through the
corner; the corridor is built on it and MINCO then has to turn inside thin polytopes, so it first brakes.
Receding-distance experiments (0.2x / 3x) changed nothing.

Patch 21 replaces every guide vertex whose turn is >45 deg with a quadratic-Bezier rounding arc of radius
cfg_.guide_round_radius (default 0.5 m), IF the arc is KNOWN_FREE in the inflated map.  The corner stays
inside the arc (max deviation 0.25*r = 0.125 m << the 0.5 m W1b box), so bookkeeping is untouched; MINCO gets
a smooth turning guide and a chance to keep the speed through the corner.

Usage:
    python3 hku_super/patches/patch_super_guide_round.py --root hku_super/super_planner --check|--apply
"""
from __future__ import annotations

import argparse
import os
import sys

DEFAULT_ROOT = os.environ.get("DSH_ROOT", "") + "/hku_super/super_planner"
MARK = "WHTF-GUIDE-ROUND (patch 21"

CFG_FIELD_ANCHOR = """        bool pva_tail_adaptive;"""
CFG_FIELD_PATCH = CFG_FIELD_ANCHOR + """
        /// WHTF-GUIDE-ROUND (patch 21): round the guide path at sharp vertices before corridor generation.
        /// The corner stays inside the rounding arc (deviation <= 0.25*r), so W1b is untouched.
        bool guide_round_en;
        double guide_round_radius;"""
CFG_LOAD_ANCHOR = """            loader.LoadParam("super_planner/pva_tail_adaptive", pva_tail_adaptive, false);"""
CFG_LOAD_PATCH = CFG_LOAD_ANCHOR + """
            loader.LoadParam("super_planner/guide_round_en", guide_round_en, false);
            loader.LoadParam("super_planner/guide_round_radius", guide_round_radius, 0.5);"""

CPP_ANCHOR = """        out_exp_traj_info.setGoalConnectedFlag(connected_goal);"""
CPP_PATCH = CPP_ANCHOR + r'''

        /// WHTF-GUIDE-ROUND (patch 21, 2026-11-27 round 14).  A sharp polyline guide gives MINCO thin
        /// corridor polytopes at the corner, so it brakes to 0.0-0.2 m/s before turning (MEASURED every
        /// corner of the 1.35 m/s arm).  Replace each >45 deg vertex with a quadratic-Bezier rounding arc
        /// (radius guide_round_radius) when the arc is known-free.  The corner stays inside the arc
        /// (deviation ~0.25*r = 0.125 m for r=0.5 m, far inside the 0.5 m W1b box).
        if (cfg_.guide_round_en && guide_path.size() >= 3) {
            vec_Vec3f rounded;
            rounded.reserve(guide_path.size() * 2);
            rounded.emplace_back(guide_path.front());
            bool rounded_any = false;
            const double r = cfg_.guide_round_radius;
            for (size_t i = 1; i + 1 < guide_path.size(); ++i) {
                const Vec3f a = guide_path[i - 1], b = guide_path[i], c = guide_path[i + 1];
                Vec3f d1 = b - a; d1.z() = 0.0;
                Vec3f d2 = c - b; d2.z() = 0.0;
                const double n1 = d1.norm(), n2 = d2.norm();
                if (n1 < 1e-6 || n2 < 1e-6) { rounded.emplace_back(b); continue; }
                const double cosang = d1.dot(d2) / (n1 * n2);
                if (cosang < std::cos(45.0 * M_PI / 180.0) && n1 > 2.2 * r && n2 > 2.2 * r) {
                    const Vec3f u1 = d1 / n1, u2 = d2 / n2;
                    const Vec3f p0 = b - u1 * r;
                    const Vec3f p2 = b + u2 * r;
                    bool arc_ok = true;
                    for (int k = 1; k <= 4; ++k) {
                        const double t = 0.2 * static_cast<double>(k);
                        const Vec3f q = p0 * ((1 - t) * (1 - t)) + b * (2 * (1 - t) * t) + p2 * (t * t);
                        if (map_ptr_->getInfGridType(q) != rog_map::GridType::KNOWN_FREE) { arc_ok = false; break; }
                    }
                    if (arc_ok) {
                        for (int k = 1; k <= 5; ++k) {
                            const double t = 0.2 * static_cast<double>(k);
                            const Vec3f q = p0 * ((1 - t) * (1 - t)) + b * (2 * (1 - t) * t) + p2 * (t * t);
                            rounded.emplace_back(q);
                        }
                        rounded_any = true;
                        continue;
                    }
                }
                rounded.emplace_back(b);
            }
            rounded.emplace_back(guide_path.back());
            if (rounded_any) {
                guide_path = rounded;
                guide_stamp.clear();
                double tt = 0.0;
                const double vref = std::max(cfg_.exp_traj_cfg.max_vel, 1e-3);
                for (size_t i = 0; i < guide_path.size(); ++i) {
                    if (i > 0) { tt += (guide_path[i] - guide_path[i - 1]).norm() / vref; }
                    guide_stamp.emplace_back(tt);
                }
                if (cfg_.print_log) {
                    cout << " -- [SUPER] GUIDE-ROUND: guide " << guide_path.size() << " points" << endl;
                }
            }
        }'''


def _read(p):
    with open(p, encoding="utf-8") as fh:
        return fh.read()


def _write(p, s):
    with open(p, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(s)


def main() -> int:
    ap = argparse.ArgumentParser(description="patch 21: round the guide path at corner vertices")
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
        print("patch_super_guide_round: %s" % ("is APPLIED" if applied else "is NOT APPLIED"))
        return 0 if applied else 1
    if a.apply:
        if applied:
            print("already applied")
            return 0
        for name, s_, anchor in (("config.hpp", cfg_s, CFG_FIELD_ANCHOR),
                                 ("config.hpp", cfg_s, CFG_LOAD_ANCHOR),
                                 ("super_planner.cpp", cpp_s, CPP_ANCHOR)):
            if anchor not in s_:
                print("ANCHOR NOT FOUND in %s: %r" % (name, anchor[:60]), file=sys.stderr)
                return 3
        cfg_s = cfg_s.replace(CFG_FIELD_ANCHOR, CFG_FIELD_PATCH, 1).replace(CFG_LOAD_ANCHOR, CFG_LOAD_PATCH, 1)
        cpp_s = cpp_s.replace(CPP_ANCHOR, CPP_PATCH, 1)
        _write(cfg, cfg_s)
        _write(cpp, cpp_s)
        print("applied -> guide path rounding (flags guide_round_en / guide_round_radius)")
        return 0
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
