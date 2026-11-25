#!/usr/bin/env python3
"""track_quality.py -- how tightly did the aircraft follow the reference, and WHY it looks loose.

    python3 scripts/track_quality.py --tag phase4q-01-01
    python3 scripts/track_quality.py --run .runs/missionA-xxx --acc-limit 4.0 --json out.json

Reports, on the flight window (altitude >= 1 m):
  * zero-lag horizontal error (p50 / p90 / p99) and the BEST temporal lag;
  * the error at the best lag, split into along-track (ahead/behind the moving reference) and cross-track;
  * the reference's own |a_xy| distribution and how long it spends above --acc-limit (the FCU's
    correction-shaping limit, WP_ACC): when the reference demands more acceleration than the correction can
    supply, the position error is expected to grow -- that is "the controller is soft", not "the estimator";
  * a speed-binned table: error/jerk-lag per reference-speed bin, which is what a speed ladder needs.
No acceptance numbers are computed; this is a diagnostic for tuning and for the racing plan
(docs/HIGH-SPEED-PLAN.md).
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
import _paths as _P  # noqa: E402


def _rows(path):
    with open(path, encoding="utf-8", errors="replace") as fh:
        return list(csv.DictReader(fh))


def load_truth(run):
    rows = _rows(os.path.join(run, "tap.csv"))
    tcol = "t_sim_us" if rows and "t_sim_us" in rows[0] else "t_us"
    t = np.array([float(r[tcol]) * 1e-6 for r in rows])
    E = np.array([float(r["pe"]) for r in rows])
    N = np.array([float(r["pn"]) for r in rows])
    U = np.array([-float(r["pd"]) for r in rows])
    vE = np.array([float(r["ve"]) for r in rows])
    vN = np.array([float(r["vn"]) for r in rows])
    o = np.argsort(t)
    return t[o], np.column_stack([E[o], N[o], U[o]]), np.column_stack([vE[o], vN[o]])


def load_cmd(run):
    rows = _rows(os.path.join(run, "setpoints.csv"))
    t = np.array([float(r["t_sim_us"]) * 1e-6 for r in rows])
    p = np.column_stack([np.array([float(r["cmd_x"]) for r in rows]),
                         np.array([float(r["cmd_y"]) for r in rows]),
                         np.array([float(r["cmd_z"]) for r in rows])])
    v = np.column_stack([np.array([float(r["vel_x"]) for r in rows]),
                         np.array([float(r["vel_y"]) for r in rows])])
    a = np.column_stack([np.array([float(r["acc_x"]) for r in rows]),
                         np.array([float(r["acc_y"]) for r in rows])])
    o = np.argsort(t)
    return t[o], p[o], v[o], a[o]


def _interp3(t, ts, src):
    return np.column_stack([np.interp(t, ts, src[:, k]) for k in range(src.shape[1])])


def main() -> int:
    ap = argparse.ArgumentParser(description="command-tracking quality of one sortie")
    # ArduPilot GUIDED in PVA mode feeds the reference acceleration unshaped and only limits the
    # correction; WP_ACC is that limit (guided_limits_fast.parm = 4.0 m/s^2, see docs/HIGH-SPEED-PLAN.md).
    ap.add_argument("--acc-limit", type=float, default=4.0, help="FCU correction accel limit (WP_ACC)")
    ap.add_argument("--lag-max", type=float, default=0.6)
    ap.add_argument("--json", default="")
    ap.add_argument("--run", default="")
    ap.add_argument("--tag", default="")
    a = ap.parse_args()
    if a.run:
        run = os.path.abspath(a.run)
    elif a.tag:
        run = _P.sortie(a.tag if not a.tag.startswith("missionA-") else a.tag[len("missionA-"):])
    else:
        cands = sorted([p for p in os.listdir(_P.LOGS) if p.startswith("missionA-")],
                       key=lambda p: os.path.getmtime(os.path.join(_P.LOGS, p)), reverse=True)
        run = os.path.join(_P.LOGS, cands[0])
    tag = os.path.basename(run).replace("missionA-", "")
    t, P, V = load_truth(run)
    tc, PC, VC, AC = load_cmd(run)
    m = P[:, 2] >= 1.0
    t, P, V = t[m], P[m], V[m]
    win = (t >= t[0] + 2.0) & (t <= t[-1] - 2.0)
    if win.sum() < 100:
        print("too few samples in the flight window")
        return 2

    def err_at(lag):
        cmd = _interp3(t + lag, tc, PC)
        e = np.hypot(P[:, 0] - cmd[:, 0], P[:, 1] - cmd[:, 1])
        return cmd, e

    best = (1e9, 0.0)
    for lag in np.arange(-a.lag_max, a.lag_max + 1e-9, 0.01):
        cmd, e = err_at(lag)
        p90 = float(np.percentile(e[win], 90))
        if p90 < best[0]:
            best = (p90, float(lag))
    e0 = err_at(0.0)[1]
    cmd_b, e_b = err_at(best[1])
    vel_b = _interp3(t + best[1], tc, VC)
    spd_rel = np.hypot(vel_b[:, 0], vel_b[:, 1])
    u = vel_b / np.maximum(spd_rel[:, None], 1e-6)
    nvec = np.column_stack([-u[:, 1], u[:, 0]])
    dv = P[:, :2] - cmd_b[:, :2]
    along = np.sum(dv * u, axis=1)
    cross = np.sum(dv * nvec, axis=1)
    acc = _interp3(t + best[1], tc, AC)
    amag = np.hypot(acc[:, 0], acc[:, 1])
    spd_t = np.hypot(V[:, 0], V[:, 1])

    def pct(x, q):
        return float(np.percentile(x[win], q)) if x[win].size else float("nan")

    print("== %s  (flight window %.0f s, n=%d)" % (tag, t[win][-1] - t[win][0], int(win.sum())))
    print("zero-lag horizontal error : p50 %.3f  p90 %.3f  p99 %.3f m"
          % (pct(e0, 50), pct(e0, 90), pct(e0, 99)))
    print("best lag                  : %+.2f s  -> p90 error %.3f m" % (best[1], best[0]))
    print("  along  (at best lag)    : p50 %+.3f  p90 %+.3f m  (negative = behind the reference)"
          % (pct(along, 50), pct(along, 90)))
    print("  cross  (at best lag)    |p50| %.3f  p90 %.3f m" % (abs(pct(cross, 50)), pct(np.abs(cross), 90)))
    print("reference |a_xy|          : p50 %.2f  p90 %.2f  p99 %.2f  max %.2f m/s^2"
          % (pct(amag, 50), pct(amag, 90), pct(amag, 99), float(amag[win].max())))
    sat = float(np.mean(amag[win] > a.acc_limit)) if win.any() else 0.0
    print("  above the FCU limit %.1f : %.1f %% of the time (correction is rate-limited there)"
          % (a.acc_limit, 100.0 * sat))
    print("reference speed           : p50 %.2f  p90 %.2f  max %.2f m/s   (truth max %.2f)"
          % (pct(spd_rel, 50), pct(spd_rel, 90), float(spd_rel[win].max()),
             float(spd_t[win].max())))
    # speed-binned table: error and the fraction of time the reference accel exceeds the limit
    bins = [(0.0, 0.5), (0.5, 1.0), (1.0, 1.5), (1.5, 2.5), (2.5, 4.0), (4.0, 7.0), (7.0, 20.0)]
    print("\n  speed bin (m/s)      n     err p50   err p90   a>limit")
    rows_out = []
    for lo, hi in bins:
        mb = win & (spd_rel >= lo) & (spd_rel < hi)
        if mb.sum() < 30:
            continue
        rec = {"lo": lo, "hi": hi, "n": int(mb.sum()),
               "err_p50": float(np.percentile(e_b[mb], 50)),
               "err_p90": float(np.percentile(e_b[mb], 90)),
               "acc_over_frac": float(np.mean(amag[mb] > a.acc_limit))}
        rows_out.append(rec)
        print("  %4.1f - %4.1f         %5d   %7.3f   %7.3f   %5.1f %%"
              % (lo, hi, rec["n"], rec["err_p50"], rec["err_p90"], 100.0 * rec["acc_over_frac"]))
    if a.json:
        json.dump({"tag": tag, "best_lag_s": best[1], "zero_lag_p50": pct(e0, 50),
                   "zero_lag_p90": pct(e0, 90), "best_lag_p90": best[0],
                   "speed_bins": rows_out}, open(a.json, "w"), indent=2)
        print("\njson: %s" % a.json)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
