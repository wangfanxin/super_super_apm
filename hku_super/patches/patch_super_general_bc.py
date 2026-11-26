#!/usr/bin/env python3
"""patch_super_general_bc.py -- patch 17: GENERAL boundary conditions (neither end has to be at rest).

Why (owner, 2026-11-26): "我们其实不是 rest2rest，因为是连续飞行！把 SUPER 做成通用形式：起点终点都不一定是静止的".
SUPER's `PlanFromRest` always enters the rest branch (`last_exp_traj_info_.setEmpty()`), so a fresh goal
is a MINCO problem whose start is the current position with **v=0, a=0** -- and hot_start only injected a
velocity capped at `0.6*max_vel` (0.81 m/s at the 1.35 m/s arm), still with a=0.  Measured consequence
(phase23c): command-speed collapses at every corner (0.04-0.35 m/s), reference cusps (CUSP 120 deg in one
tick), 0.2-0.7 m back-and-forth.  The END of the trajectory was already general via `goal_pva_en`
(col(1)=goal_v, col(2)=goal_a); the START was not.

What this patch does (behind `super_planner/general_bc_en`, default false):

  * rest branch: pos_init_state.col(1) = MEASURED velocity (capped at max_vel, not 0.6*max_vel),
    pos_init_state.col(2) = MEASURED acceleration (capped at max_acc).  The old "only when moving and the
    goal is far" gating is skipped: the planner is handed the true state, whatever it is.
  * if the optimiser rejects that boundary (e.g. a goal the current momentum cannot reach), the plan is
    retried ONCE with the historical zero start velocities, so a new goal can never trap the FSM in a
    failing loop (the failure mode that patch 16 exposed: 1897 `PlanFromRest failed` in one sortie).

Usage:
    python3 hku_super/patches/patch_super_general_bc.py --root hku_super/super_planner --check|--apply|--revert
"""
from __future__ import annotations

import argparse
import os
import sys

DEFAULT_ROOT = os.environ.get("DSH_ROOT", "") + "/hku_super/super_planner"

CFG_FIELD_ANCHOR = """        bool hot_start_en;"""
CFG_FIELD_PATCH = CFG_FIELD_ANCHOR + """
        /// WHTF-GENERAL-BC (patch 17): hand the optimiser the MEASURED start velocity AND acceleration
        /// (and, through goal_pva_en, the goal's terminal velocity/acceleration) instead of assuming a
        /// rest boundary.  Default false keeps the delivered behaviour.
        bool general_bc_en;"""

CFG_LOAD_ANCHOR = """            loader.LoadParam("super_planner/hot_start_en", hot_start_en, false);"""
CFG_LOAD_PATCH = CFG_LOAD_ANCHOR + """
            loader.LoadParam("super_planner/general_bc_en", general_bc_en, false);"""

DECL_ANCHOR = """        StatePVAJ pos_init_state, pos_fina_state;"""
DECL_PATCH = DECL_ANCHOR + """
        bool used_general_bc = false;   ///< WHTF-GENERAL-BC (patch 17) -- for the zero-start retry below"""

REST_ANCHOR = """            if (cfg_.hot_start_en) {
                Vec3f v0 = robot_state_.v;
                const double vn0 = v0.norm();
                const double v_cap = cfg_.exp_traj_cfg.max_vel * 0.6;
                if (vn0 > v_cap && vn0 > 1e-6) {
                    v0 *= v_cap / vn0;
                }
                const double stop_dis = v_cap * v_cap / (2.0 * cfg_.exp_traj_cfg.max_acc) + 0.3;
                if (vn0 > 0.05 && (gi_.goal_p - local_start_p_).norm() > stop_dis) {
                    pos_init_state.col(1) = v0;
                }
            }"""
REST_PATCH = """            if (cfg_.general_bc_en) {
                /// WHTF-GENERAL-BC (patch 17).  The vehicle is a CONTINUOUS system: at a handover it is
                /// moving (and accelerating) while the planner starts a fresh MINCO problem.  Hand the
                /// optimiser the measured state so the new trajectory starts on the same physical state
                /// -- no brake-to-zero, no reference cusp.  The caps are the planner's own envelope.
                Vec3f v0 = robot_state_.v;
                const double vn0 = v0.norm();
                const double v_cap = cfg_.exp_traj_cfg.max_vel;
                if (vn0 > v_cap && vn0 > 1e-6) { v0 *= v_cap / vn0; }
                if (vn0 > 1e-3) { pos_init_state.col(1) = v0; }
                Vec3f a0 = robot_state_.a;
                const double an0 = a0.norm();
                const double a_cap = cfg_.exp_traj_cfg.max_acc;
                if (an0 > a_cap && an0 > 1e-6) { a0 *= a_cap / an0; }
                if (an0 > 1e-3) { pos_init_state.col(2) = a0; }
                used_general_bc = true;
                ros_ptr_->info(" -- [SUPER] GENERAL-BC: start from measured state |v| {:.2f} m/s, "
                               "|a| {:.2f} m/s2", vn0, an0);
            } else if (cfg_.hot_start_en) {
                Vec3f v0 = robot_state_.v;
                const double vn0 = v0.norm();
                const double v_cap = cfg_.exp_traj_cfg.max_vel * 0.6;
                if (vn0 > v_cap && vn0 > 1e-6) {
                    v0 *= v_cap / vn0;
                }
                const double stop_dis = v_cap * v_cap / (2.0 * cfg_.exp_traj_cfg.max_acc) + 0.3;
                if (vn0 > 0.05 && (gi_.goal_p - local_start_p_).norm() > stop_dis) {
                    pos_init_state.col(1) = v0;
                }
            }"""
REST_PATCH = REST_PATCH.replace('ros_ptr_->info(" -- [SUPER] GENERAL-BC: start from measured state |v| {:.2f} m/s, "\n                               "|a| {:.2f} m/s2", vn0, an0);',
                                'cout << " -- [SUPER] GENERAL-BC: start from measured state" << endl;')

OPT_ANCHOR = """        temp_ret = exp_traj_opt_->optimize(pos_init_state,
                                           pos_fina_state,
                                           guide_path,
                                           guide_stamp,
                                           sfc,
                                           out_traj);"""
OPT_PATCH = """        temp_ret = exp_traj_opt_->optimize(pos_init_state,
                                           pos_fina_state,
                                           guide_path,
                                           guide_stamp,
                                           sfc,
                                           out_traj);
        if (!temp_ret && used_general_bc) {
            /// WHTF-GENERAL-BC RETRY (patch 17): a goal the current momentum cannot reach must not put
            /// the FSM into a failing loop -- retry ONCE from the historical zero-velocity start.
            ros_ptr_->warn(" -- [SUPER] GENERAL-BC: optimiser rejected the measured start state, "
                           "retrying from rest");
            pos_init_state.col(1).setZero();
            pos_init_state.col(2).setZero();
            sfc = original_sfc;
            temp_ret = exp_traj_opt_->optimize(pos_init_state,
                                               pos_fina_state,
                                               guide_path,
                                               guide_stamp,
                                               sfc,
                                               out_traj);
            used_general_bc = false;
        }"""
MARK = "WHTF-GENERAL-BC (patch 17"


def _read(p):
    with open(p, encoding="utf-8") as fh:
        return fh.read()


def _write(p, s):
    with open(p, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(s)


def main() -> int:
    ap = argparse.ArgumentParser(description="patch 17: general (non-rest) boundary conditions")
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
        print("patch_super_general_bc: %s" % ("is APPLIED" if applied else "is NOT APPLIED"))
        return 0 if applied else 1
    if a.apply:
        if applied:
            print("already applied")
            return 0
        pairs = [("config.hpp", cfg_s, CFG_FIELD_ANCHOR, CFG_FIELD_PATCH),
                 ("config.hpp", cfg_s, CFG_LOAD_ANCHOR, CFG_LOAD_PATCH),
                 ("super_planner.cpp", cpp_s, DECL_ANCHOR, DECL_PATCH),
                 ("super_planner.cpp", cpp_s, REST_ANCHOR, REST_PATCH),
                 ("super_planner.cpp", cpp_s, OPT_ANCHOR, OPT_PATCH)]
        for name, s, anchor, _ in pairs:
            if anchor not in s:
                print("ANCHOR NOT FOUND in %s -- refusing to guess: %r" % (name, anchor[:50]),
                      file=sys.stderr)
                return 3
        cfg_s = cfg_s.replace(CFG_FIELD_ANCHOR, CFG_FIELD_PATCH, 1).replace(CFG_LOAD_ANCHOR, CFG_LOAD_PATCH, 1)
        cpp_s = cpp_s.replace(DECL_ANCHOR, DECL_PATCH, 1)
        cpp_s = cpp_s.replace(REST_ANCHOR, REST_PATCH, 1)
        cpp_s = cpp_s.replace(OPT_ANCHOR, OPT_PATCH, 1)
        _write(cfg, cfg_s)
        _write(cpp, cpp_s)
        print("applied -> general boundary conditions (flag super_planner/general_bc_en)")
        return 0
    if a.revert:
        if not applied:
            print("not applied, nothing to revert")
            return 0
        cfg_s = cfg_s.replace(CFG_LOAD_PATCH, CFG_LOAD_ANCHOR, 1).replace(CFG_FIELD_PATCH, CFG_FIELD_ANCHOR, 1)
        cpp_s = cpp_s.replace(OPT_PATCH, OPT_ANCHOR, 1).replace(REST_PATCH, REST_ANCHOR, 1)
        cpp_s = cpp_s.replace(DECL_PATCH, DECL_ANCHOR, 1)
        _write(cfg, cfg_s)
        _write(cpp, cpp_s)
        print("reverted")
        return 0
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
