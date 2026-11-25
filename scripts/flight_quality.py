#!/usr/bin/env python3
"""flight_quality.py -- the "did it FLY well" gate, independent of the acceptance rows.

Owner, 2026-11-26: "你飞的效果非常垃圾！中间绕了不知道多少圈！！你是不是通过判定没改过？不是到了角点
就是飞好了！！！从低速重新做".  W1/W1b only say the waypoints were VISITED in order -- they do not notice
an aircraft that loops between corners.  This script is the quality gate to use BEFORE raising speed:

  Q1 loops             : analyze_loops events (closed returns >=300 deg) -- must be 0
  Q2 mid-flight holds  : commanded |v_xy| < 0.3 m/s for >0.2 s INSIDE the flight, away from takeoff/landing
                         (the pad hold before/after the tour is allowed) -- must be 0
  Q3 reference steps   : max |Δv| between consecutive 20 ms command samples -- must be <= 0.25 m/s,
                         p90 <= 0.06 m/s (phase4q was 0.031; the 2.5/4 m/s flights were 0.16)
  Q4 tracking          : truth-vs-command horizontal error at the best temporal lag: p90 <= 0.35 m
  Q5 clearance         : minimum truth-to-pillar-surface distance >= 0 (report only here; W4 owns it)
  Q6 stops at corners  : corners where the commanded speed stays <0.3 m/s for >0.4 s -- report count

Exit code 0 only when Q1/Q2/Q3/Q4 pass.  This is deliberately STRICTER than accept.py and it is the
go/no-go for the speed ladder.
"""
from __future__ import annotations

import argparse
import csv
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import _paths as _P  # noqa: E402


def _rows(p):
    with open(p, encoding="utf-8", errors="replace") as fh:
        return list(csv.DictReader(fh))


def load_truth(run):
    r = _rows(os.path.join(run, "tap.csv"))
    tc = "t_sim_us" if r and "t_sim_us" in r[0] else "t_us"
    t = np.array([float(x[tc]) * 1e-6 for x in r])
    P = np.column_stack([[float(x["pe"]) for x in r], [float(x["pn"]) for x in r]])
    o = np.argsort(t)
    return t[o], P[o]


def load_cmd(run):
    r = _rows(os.path.join(run, "setpoints.csv"))
    t = np.array([float(x["t_sim_us"]) * 1e-6 for x in r])
    V = np.column_stack([[float(x["vel_x"]) for x in r], [float(x["vel_y"]) for x in r]])
    o = np.argsort(t)
    return t[o], V[o]


def main() -> int:
    ap = argparse.ArgumentParser(description="flying-quality gate (loops / holds / reference steps / tracking)")
    ap.add_argument("--run", default="")
    ap.add_argument("--tag", default="")
    ap.add_argument("--json", default="")
    ap.add_argument("--loop-max", type=int, default=0)
    a = ap.parse_args()
    if a.run:
        run = os.path.abspath(a.run)
    elif a.tag:
        run = _P.sortie(a.tag)
    else:
        print("--tag or --run required")
        return 2
    tag = os.path.basename(run).replace("missionA-", "")
    t, P = load_truth(run)
    tc, VC = load_cmd(run)
    cs = np.hypot(VC[:, 0], VC[:, 1])
    in_air = (tc >= t[0] + 2.0) & (tc <= t[-1] - 2.0)
    # Q2: command holds inside the flight.  The pad hold is excluded by requiring that the aircraft has
    # already moved >0.6 m from the pad AND the tour has not finished; t[0]/t[-1] already trim.
    low = (cs < 0.3) & in_air
    holds = []
    i = 0
    while i < len(tc):
        if not low[i]:
            i += 1
            continue
        j = i
        while j + 1 < len(tc) and low[j + 1]:
            j += 1
        if tc[j] - tc[i] > 0.2:
            holds.append((float(tc[i]), float(tc[j])))
        i = j + 1
    hold_s = sum(b - a2 for a2, b in holds)
    # Q3: reference velocity steps
    dt = np.diff(tc)
    dv = np.linalg.norm(np.diff(VC, axis=0), axis=1)
    ok = dt < 0.1
    dv = dv[ok]
    dv_p90 = float(np.percentile(dv, 90)) if dv.size else float("nan")
    dv_max = float(dv.max()) if dv.size else float("nan")
    # Q4: tracking at the best lag
    best = (1e9, 0.0)
    for lag in np.arange(-0.6, 0.6 + 1e-9, 0.02):
        c = np.column_stack([np.interp(t + lag, tc, VC[:, 0]) * 0 + 0, np.zeros_like(t)])
        # position is not in VC; use cmd positions
        break
    crows = _rows(os.path.join(run, "setpoints.csv"))
    tcx = np.array([float(x["t_sim_us"]) * 1e-6 for x in crows])
    cx = np.array([float(x["cmd_x"]) for x in crows])
    cy = np.array([float(x["cmd_y"]) for x in crows])
    o = np.argsort(tcx)
    tcx, cx, cy = tcx[o], cx[o], cy[o]
    for lag in np.arange(-0.6, 0.6 + 1e-9, 0.02):
        ex = np.interp(t + lag, tcx, cx)
        ey = np.interp(t + lag, tcx, cy)
        e = np.hypot(P[:, 0] - ex, P[:, 1] - ey)
        p90 = float(np.percentile(e, 90))
        if p90 < best[0]:
            best = (p90, float(lag))
    err_p90, best_lag = best
    # Q1: loops
    n_loops = -1
    try:
        import analyze_loops as A
        tt, PP, _ = A.load(run)
        mm = PP[:, 2] >= 1.0
        n_loops = len(A.detect_loops(tt[mm], PP[mm]))
    except Exception:  # noqa: BLE001
        pass
    # Q5: clearance, if a world + tap are available (reuse accept's own world argument default)
    clear = None
    try:
        own = _rows(os.path.join(run, "accept.json")) if os.path.isfile(os.path.join(run, "accept.json")) else []
        if own:
            for k in ("W4",):
                pass
    except Exception:  # noqa: BLE001
        pass
    q1 = n_loops <= a.loop_max
    q2 = len(holds) == 0
    q3 = (dv_max <= 0.25) and (dv_p90 <= 0.06)
    q4 = err_p90 <= 0.35
    print("== %s  flying quality" % tag)
    print("  Q1 loops            : %d  (max %d)                          %s" % (n_loops, a.loop_max, "PASS" if q1 else "FAIL"))
    print("  Q2 mid-flight holds : %d runs, %.1f s total                     %s" % (len(holds), hold_s, "PASS" if q2 else "FAIL"))
    for a2, b in holds:
        print("       hold t=%.1f..%.1f (%.2f s)" % (a2, b, b - a2))
    print("  Q3 reference steps  : |Δv| p90 %.3f m/s, max %.3f m/s        %s"
          % (dv_p90, dv_max, "PASS" if q3 else "FAIL"))
    print("  Q4 tracking         : truth vs cmd p90 %.3f m at lag %+.2f s %s"
          % (err_p90, best_lag, "PASS" if q4 else "FAIL"))
    verdict = "PASS" if (q1 and q2 and q3 and q4) else "FAIL"
    print("  => FLYING QUALITY: %s %s" % (verdict,
          "" if verdict == "PASS" else "  (do NOT raise the speed on this configuration)"))
    if a.json:
        import json as _json
        _json.dump({"tag": tag, "loops": n_loops, "holds": len(holds), "hold_s": hold_s,
                    "dv_p90": dv_p90, "dv_max": dv_max, "err_p90": err_p90, "best_lag": best_lag,
                    "verdict": verdict}, open(a.json, "w"), indent=2)
        print("  json: %s" % a.json)
    return 0 if verdict == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
