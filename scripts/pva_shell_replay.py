#!/usr/bin/env python3
"""pva_shell_replay.py -- offline replay of a self-consistent PVA kinematic shell on recorded command streams.

ROUND 16 LESSON: the first shell implementation kept the planned POSITION and only smoothed VELOCITY, so p and v
disagreed and the flight diverged (135a: W1 6/13, Q3 1.47).  The self-consistent version integrates position from
the LIMITED velocity and slowly re-anchors it to the planned position:

    v_des   = planned v
    v_lim   = jerk/acc limited version of v_des
    p_int  += v_out*dt + 0.5*a_out*dt^2
    e       = p_plan - p_int
    v_corr  = clip(kp*e, +-v_corr_max)
    v_out   = v_lim + v_corr               (and p_int += v_corr*dt via the next step)

The position therefore stays C1-smooth, and the re-anchor term keeps it within `kp*v_corr_max` of the plan.
This script replays it on recorded /plan/cmd PVA and reports the numbers that decide whether a flight is worth it:
jerk reduction, position deviation from the plan, speed loss, time below half speed, path loops.

USAGE
  python3 scripts/pva_shell_replay.py --tag phase133a-01-01 --jerk 20 --acc 6 --kp 1.0 --vcorr 0.15
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))


def run_dir(tag):
    p = os.path.join(ROOT, ".runs", "missionA-%s" % tag)
    if not os.path.isdir(p):
        raise SystemExit("run not found: %s" % tag)
    return p


def load_pva(tag):
    d = run_dir(tag)
    rows = list(csv.DictReader(open(os.path.join(d, "traj.csv.cmd.csv"))))
    t = np.array([float(r["t_us"]) * 1e-6 for r in rows])
    o = np.argsort(t, kind="stable")
    t = t[o]
    p = np.column_stack([[float(r[k]) for r in rows] for k in ("x_l", "y_l", "z_l")])[o]
    v = np.column_stack([[float(r[k]) for r in rows] for k in ("vx_l", "vy_l", "vz_l")])[o]
    a = np.column_stack([[float(r[k]) for r in rows] for k in ("ax_l", "ay_l", "az_l")])[o]
    k = np.concatenate([[True], np.diff(t) > 1e-9])
    t, p, v, a = t[k], p[k], v[k], a[k]
    # drop parked samples at the very beginning/end (|v|==0 for >2 s)
    return t, p, v, a


def limits(v, a, dt, jerk, acc, vmax):
    n = len(v)
    v_out = np.zeros_like(v)
    a_out = np.zeros_like(a)
    v_prev = v[0].copy()
    a_prev = np.zeros(3)
    for i in range(n):
        a_des = (v[i] - v_prev) / dt[i]
        an = np.linalg.norm(a_des)
        if an > acc and an > 1e-9:
            a_des *= acc / an
        da = a_des - a_prev
        dan = np.linalg.norm(da)
        if dan > jerk * dt[i] and dan > 1e-9:
            da *= (jerk * dt[i]) / dan
        a_new = a_prev + da
        v_new = v_prev + a_new * dt[i]
        vn = np.linalg.norm(v_new)
        if vmax > 0 and vn > vmax and vn > 1e-9:
            v_new *= vmax / vn
        v_out[i] = v_new
        a_out[i] = a_new
        v_prev, a_prev = v_new, a_new
    return v_out, a_out


def integrate(p_plan, v_lim, a_lim, dt, kp, vcorr):
    n = len(p_plan)
    p = np.zeros_like(p_plan)
    v = np.zeros_like(v_lim)
    p[0] = p_plan[0].copy()
    v[0] = v_lim[0].copy()
    for i in range(1, n):
        p[i] = p[i - 1] + v[i - 1] * dt[i] + 0.5 * a_lim[i - 1] * dt[i] ** 2
        e = p_plan[i] - p[i]
        en = np.linalg.norm(e)
        vc = kp * e
        vcn = np.linalg.norm(vc)
        if vcorr > 0 and vcn > vcorr and vcn > 1e-9:
            vc *= vcorr / vcn
        v[i] = v_lim[i] + vc
    return p, v


def path_loops(tt, pp, win=1.2, ratio=1.8):
    n = len(tt)
    flag = np.zeros(n, bool)
    for i in range(n):
        w = (tt >= tt[i] - win) & (tt <= tt[i] + win)
        if w.sum() < 15:
            continue
        pl = float(np.sum(np.hypot(np.diff(pp[w, 0]), np.diff(pp[w, 1]))))
        ww = pp[w]
        net = float(np.hypot(ww[-1, 0] - ww[0, 0], ww[-1, 1] - ww[0, 1]))
        if net < 0.35 and pl > 0.8:
            flag[w] = True
        elif pl / max(net, 1e-6) > ratio and pl > 1.0:
            flag[w] = True
    out = 0
    i = 0
    while i < n:
        if not flag[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and flag[j + 1]:
            j += 1
        if j - i >= 3:
            out += 1
        i = j + 1
    return out


def analyse(tag, jerk, acc, vmax, kp, vcorr):
    t, p, v, a = load_pva(tag)
    dt = np.gradient(t)
    dt = np.maximum(dt, 1e-3)
    v_h = np.linalg.norm(v[:, :2], axis=1)
    v_lim, a_lim = limits(v, a, dt, jerk, acc, vmax)
    p_int, v_int = integrate(p, v_lim, a_lim, dt, kp, vcorr)
    # metrics
    jerk_before = np.linalg.norm(np.gradient(a, t, axis=0), axis=1)
    jerk_after = np.linalg.norm(np.gradient(a_lim, t, axis=0), axis=1)
    pos_err = np.linalg.norm((p_int - p)[:, :2], axis=1)
    sp_before = np.linalg.norm(v[:, :2], axis=1)
    sp_after = np.linalg.norm(v_int[:, :2], axis=1)
    thr = 0.675
    return dict(tag=tag, params=dict(jerk=jerk, acc=acc, vmax=vmax, kp=kp, vcorr=vcorr),
                jerk_p95_before=float(np.percentile(jerk_before, 95)),
                jerk_p95_after=float(np.percentile(jerk_after, 95)),
                jerk_max_before=float(jerk_before.max()), jerk_max_after=float(jerk_after.max()),
                pos_err_p50=float(np.percentile(pos_err, 50)), pos_err_p90=float(np.percentile(pos_err, 90)),
                pos_err_max=float(pos_err.max()),
                speed_p50_before=float(np.percentile(sp_before, 50)), speed_p50_after=float(np.percentile(sp_after, 50)),
                speed_p90_before=float(np.percentile(sp_before, 90)), speed_p90_after=float(np.percentile(sp_after, 90)),
                frac_below_half_before=float(np.mean(sp_before < thr)), frac_below_half_after=float(np.mean(sp_after < thr)),
                loops_before=path_loops(t, p), loops_after=path_loops(t, p_int))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", action="append", required=True)
    ap.add_argument("--jerk", type=float, default=20.0)
    ap.add_argument("--acc", type=float, default=6.0)
    ap.add_argument("--vmax", type=float, default=1.35)
    ap.add_argument("--kp", type=float, default=1.0)
    ap.add_argument("--vcorr", type=float, default=0.15)
    ap.add_argument("--json", default="")
    a = ap.parse_args()
    outs = []
    for t in a.tag:
        o = analyse(t, a.jerk, a.acc, a.vmax, a.kp, a.vcorr)
        outs.append(o)
        print("== %s jerk=%g acc=%g kp=%g vcorr=%g" % (t, a.jerk, a.acc, a.kp, a.vcorr))
        print("   jerk p95 %.1f -> %.1f | max %.1f -> %.1f m/s^3" %
              (o["jerk_p95_before"], o["jerk_p95_after"], o["jerk_max_before"], o["jerk_max_after"]))
        print("   position err: p50 %.3f p90 %.3f max %.3f m" % (o["pos_err_p50"], o["pos_err_p90"], o["pos_err_max"]))
        print("   speed p50 %.2f->%.2f p90 %.2f->%.2f | below half: %.3f -> %.3f" %
              (o["speed_p50_before"], o["speed_p50_after"], o["speed_p90_before"], o["speed_p90_after"],
               o["frac_below_half_before"], o["frac_below_half_after"]))
        print("   loops %d -> %d" % (o["loops_before"], o["loops_after"]))
    if a.json:
        json.dump(outs, open(a.json, "w"), indent=1)
        print("json:", a.json)


if __name__ == "__main__":
    main()
