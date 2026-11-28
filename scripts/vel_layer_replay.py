#!/usr/bin/env python3
"""vel_layer_replay.py -- offline sanity check of the A-branch velocity layer on recorded flights.

The layer's command is  v_cmd = v_plan + Kp*(p_plan - p_actual) + Kd*(v_plan - v_actual)
(in W, both plan and actual from the same sim clock).  We cannot simulate the FCU here, but we CAN check the
things that would be fatal: command magnitude vs plan, saturation frequency, high-frequency content, and how
much of the command is the feedback term.  This is the "0-flight gate" before the A/B sortie.

USAGE: python3 scripts/vel_layer_replay.py --tag phase158a-01-01 --kp 1.5 --kd 0.2
"""
import argparse, csv, os, sys
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def run_dir(tag):
    p = os.path.join(ROOT, ".runs", "missionA-%s" % tag)
    if not os.path.isdir(p):
        raise SystemExit("run not found: %s" % tag)
    return p


def load(tag):
    d = run_dir(tag)
    sp = list(csv.DictReader(open(os.path.join(d, "setpoints.csv"))))
    ts = np.array([float(r["t_sim_us"]) * 1e-6 for r in sp])
    cols = {k: np.array([float(r[c]) for r in sp]) for k, c in
            (("cx", "cmd_x"), ("cy", "cmd_y"), ("cz", "cmd_z"),
             ("vx", "vel_x"), ("vy", "vel_y"), ("vz", "vel_z"))}
    o = np.argsort(ts, kind="stable")
    ts = ts[o]
    for k in cols:
        cols[k] = cols[k][o]
    k = np.concatenate([[True], np.diff(ts) > 1e-9])
    ts = ts[k]
    for kk in cols:
        cols[kk] = cols[kk][k]
    tap = list(csv.DictReader(open(os.path.join(d, "tap.csv"))))
    tt = np.array([float(r["t_sim_us"]) * 1e-6 for r in tap])
    E = np.array([float(r["pe"]) for r in tap]); N = np.array([float(r["pn"]) for r in tap]); U = -np.array([float(r["pd"]) for r in tap])
    vE = np.array([float(r["ve"]) for r in tap]); vN = np.array([float(r["vn"]) for r in tap]); vU = -np.array([float(r["vd"]) for r in tap])
    return ts, cols, tt, np.column_stack([E, N, U]), np.column_stack([vE, vN, vU])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", action="append", required=True)
    ap.add_argument("--kp", type=float, default=1.5)
    ap.add_argument("--kd", type=float, default=0.2)
    ap.add_argument("--vmax", type=float, default=6.0)
    ap.add_argument("--amax", type=float, default=6.0)
    a = ap.parse_args()
    for tag in a.tag:
        try:
            ts, c, tt, P, V = load(tag)
        except SystemExit:
            print("%-18s no data" % tag); continue
        p_plan = np.column_stack([c["cx"], c["cy"], c["cz"]])
        v_plan = np.column_stack([c["vx"], c["vy"], c["vz"]])
        p_act = np.column_stack([np.interp(ts, tt, P[:, i]) for i in range(3)])
        v_act = np.column_stack([np.interp(ts, tt, V[:, i]) for i in range(3)])
        ep = p_plan - p_act
        ev = v_plan - v_act
        v_cmd = v_plan + a.kp * ep + a.kd * ev
        sp_cmd = np.linalg.norm(v_cmd[:, :2], axis=1)
        sp_plan = np.linalg.norm(v_plan[:, :2], axis=1)
        sat = np.mean(sp_cmd > a.vmax - 1e-6) if a.vmax > 0 else 0.0
        fb = np.linalg.norm(a.kp * ep[:, :2] + a.kd * ev[:, :2], axis=1)
        act = np.isfinite(ts) & (np.abs(np.gradient(sp_plan, ts)) >= 0)  # all valid
        # high-frequency content of commanded speed vs plan
        def hf(x):
            d = np.diff(x) / np.maximum(np.diff(ts), 1e-3)
            return float(np.percentile(np.abs(d), 99))
        print("%-18s kp=%.2f kd=%.2f | |v_cmd| p50 %.2f p90 %.2f max %.2f (plan p50 %.2f p90 %.2f) | sat %.3f | |feedback| p50 %.2f p90 %.2f | |dv/dt| p99 cmd %.1f plan %.1f m/s^2"
              % (tag, a.kp, a.kd, np.percentile(sp_cmd, 50), np.percentile(sp_cmd, 90), sp_cmd.max(),
                 np.percentile(sp_plan, 50), np.percentile(sp_plan, 90), sat,
                 np.percentile(fb, 50), np.percentile(fb, 90), hf(sp_cmd), hf(sp_plan)))


if __name__ == "__main__":
    main()
