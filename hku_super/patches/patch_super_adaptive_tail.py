#!/usr/bin/env python3
"""patch_super_adaptive_tail.py -- patch 20: make the PVA run-out tail ADAPTIVE (direction + length).

Patch 15 appends a fixed-length tail ALONG goal_v (the bisector/outgoing direction).  MEASURED (phase24d):
corner command speed still dipped to 0.04-0.35 m/s, because in this forest the outgoing direction often has
a pillar 1.0-1.5 m past the corner (c2: clearance -0.02 m at 1.2 m), so the tail either hits the trunk
(corridor failures) or is too short to give MINCO a run-out.

Patch 20 samples candidate directions within +-60 deg of goal_v (15 deg steps), marches each along the
INFLATED map in 5 cm steps and picks the direction with the longest KNOWN_FREE run (with a small penalty for
deviating from goal_v).  The guide still goes THROUGH the corner (A* endpoint unchanged, corner stays a
corridor interior point, W1b untouched); only the run-out after it moves into free space, and the terminal
velocity is re-pointed along that run-out so the PVA is consistent.

Usage:
    python3 hku_super/patches/patch_super_adaptive_tail.py --root hku_super/super_planner --check|--apply|--revert
"""
from __future__ import annotations

import argparse
import os
import sys

DEFAULT_ROOT = os.environ.get("DSH_ROOT", "") + "/hku_super/super_planner"
MARK = "WHTF-ADAPTIVE-TAIL (patch 20"

CFG_FIELD_ANCHOR = """        double pva_tail_len;"""
CFG_FIELD_PATCH = CFG_FIELD_ANCHOR + """
        /// WHTF-ADAPTIVE-TAIL (patch 20): choose the run-out direction/length from the map's free space
        /// instead of always following goal_v.  Only active when pva_tail_len > 0.
        bool pva_tail_adaptive;"""
CFG_LOAD_ANCHOR = """            loader.LoadParam("super_planner/pva_tail_len", pva_tail_len, 0.0);"""
CFG_LOAD_PATCH = CFG_LOAD_ANCHOR + """
            loader.LoadParam("super_planner/pva_tail_adaptive", pva_tail_adaptive, false);"""

NEW_BLOCK = r'''        if (connected_goal && cfg_.goal_pva_en && cfg_.pva_tail_len > 0.0 && !guide_path.empty()) {
            Vec3f gv(gi_.goal_v[0], gi_.goal_v[1], 0.0);
            const double gvn = gv.norm();
            if (gvn > 0.10) {
                const Vec3f tail_start = guide_path.back();
                const double stamp0 = guide_stamp.empty() ? 0.0 : guide_stamp.back();
                const double v_ref = std::max(cfg_.exp_traj_cfg.max_vel, 1e-3);
                Vec3f u = gv / gvn;
                Vec3f u_in(1.0, 0.0, 0.0);
                if (guide_path.size() >= 2) {
                    const Vec3f a = guide_path[guide_path.size() - 2];
                    Vec3f d = tail_start - a;
                    d.z() = 0.0;
                    const double dn = d.norm();
                    if (dn > 1e-6) { u_in = d / dn; }
                }
                double tail_len = cfg_.pva_tail_len;
                if (cfg_.pva_tail_adaptive) {
                    /// WHTF-ADAPTIVE-TAIL (patch 20, 2026-11-27).  The outgoing direction is not always the
                    /// free direction in this forest (c2 has a pillar 1.0 m past the corner).  Sample 9
                    /// directions in +-60 deg, march the inflated map in 5 cm steps, pick the longest
                    /// KNOWN_FREE run with a small penalty for deviating from goal_v.  The corner remains
                    /// the A* endpoint / guide interior point; only the run-out after it moves.
                    double best_score = -1e9, best_free = 0.0;
                    Vec3f best_dir = u;
                    for (int k = -4; k <= 4; ++k) {
                        const double th = static_cast<double>(k) * M_PI / 12.0;   // +-60 deg, 15 deg steps
                        Vec3f cd(std::cos(th) * u.x() - std::sin(th) * u.y(),
                                 std::sin(th) * u.x() + std::cos(th) * u.y(), 0.0);
                        double fre = 0.0;
                        for (double dd = 0.15; dd <= cfg_.pva_tail_len + 1e-9; dd += 0.05) {
                            const Vec3f p = tail_start + cd * dd;
                            if (map_ptr_->getInfGridType(p) != rog_map::GridType::KNOWN_FREE) { break; }
                            fre = dd;
                        }
                        const double score = fre - 0.02 * std::abs(static_cast<double>(k));
                        if (score > best_score) { best_score = score; best_dir = cd; best_free = fre; }
                    }
                    if (best_free >= 0.35) {
                        tail_len = std::min(cfg_.pva_tail_len, best_free - 0.05);
                        u = best_dir;
                        gi_.goal_v[0] = static_cast<float>(u.x() * gvn);
                        gi_.goal_v[1] = static_cast<float>(u.y() * gvn);
                        gi_.goal_v[2] = 0.0f;
                        if (cfg_.print_log) {
                            cout << " -- [SUPER] ADAPTIVE-TAIL: free run " << best_free
                                 << " m, tail " << tail_len << " m, dir (" << u.x() << "," << u.y()
                                 << ")" << endl;
                        }
                    } else {
                        tail_len = 0.0;
                    }
                }
                if (tail_len > 0.05) {
                    Vec3f u_mid = u_in + u;
                    u_mid.z() = 0.0;
                    const double unm = u_mid.norm();
                    if (unm > 1e-6) { u_mid = u_mid / unm; } else { u_mid = u; }
                    const double d1 = 0.35 * tail_len;
                    const double a2 = 0.70 * tail_len;
                    const double d3 = tail_len;
                    guide_path.emplace_back(tail_start + u_mid * d1);
                    guide_stamp.emplace_back(stamp0 + d1 / v_ref);
                    guide_path.emplace_back(tail_start + u * a2);
                    guide_stamp.emplace_back(stamp0 + a2 / v_ref);
                    guide_path.emplace_back(tail_start + u * d3);
                    guide_stamp.emplace_back(stamp0 + d3 / v_ref);
                    if (cfg_.print_log) {
                        ros_ptr_->info(" -- [SUPER] PVA-TAIL: appended " + std::to_string(tail_len)
                                       + " m run-out past the goal");
                    }
                }
            }
        }

'''


def _read(p):
    with open(p, encoding="utf-8") as fh:
        return fh.read()


def _write(p, s):
    with open(p, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(s)


def main() -> int:
    ap = argparse.ArgumentParser(description="patch 20: adaptive PVA run-out tail")
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
    applied = MARK in cfg_s and MARK in cpp_s
    if a.check:
        print("patch_super_adaptive_tail: %s" % ("is APPLIED" if applied else "is NOT APPLIED"))
        return 0 if applied else 1
    if a.apply:
        if applied:
            print("already applied")
            return 0
        start = cpp_s.find("        if (connected_goal && cfg_.goal_pva_en && cfg_.pva_tail_len > 0.0")
        end = cpp_s.find("        sfc.clear();", start)
        if start < 0 or end < 0:
            print("ANCHOR NOT FOUND in super_planner.cpp -- refusing to guess", file=sys.stderr)
            return 3
        cpp_new = cpp_s[:start] + NEW_BLOCK + cpp_s[end:]
        if CFG_FIELD_ANCHOR not in cfg_s or CFG_LOAD_ANCHOR not in cfg_s:
            print("ANCHOR NOT FOUND in config.hpp -- refusing to guess", file=sys.stderr)
            return 3
        cfg_new = cfg_s.replace(CFG_FIELD_ANCHOR, CFG_FIELD_PATCH, 1).replace(CFG_LOAD_ANCHOR, CFG_LOAD_PATCH, 1)
        _write(cfg, cfg_new)
        _write(cpp, cpp_new)
        print("applied -> adaptive run-out tail (flag super_planner/pva_tail_adaptive)")
        return 0
    if a.revert:
        if not applied:
            print("not applied, nothing to revert")
            return 0
        print("revert: restore from git instead (patch 20 is a source rewrite; use git checkout)")
        return 1
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
