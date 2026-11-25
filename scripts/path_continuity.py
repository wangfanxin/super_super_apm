#!/usr/bin/env python3
"""path_continuity.py -- WHOLE-PATH quality of the reference and of the tracking (not corner-by-corner).

    python3 scripts/path_continuity.py --tag phase5a-01-01
    python3 scripts/path_continuity.py --tag phase4q-01-01 --json /tmp/pc.json

Owner direction (2026-11-26): "不能机械性检查上一轮不好的地方，因为每次调完以后飞都会有区别！应该沿着
路径都检查了" and "5a 规划的结果也有一点不连贯".  This tool therefore looks at the WHOLE route:

  * reference continuity: |Δv| / |Δa| between consecutive commanded samples (a "不连贯" reference shows up
    as steps here), how many steps exceed the plan's own max_acc, and where (times/locations);
  * commanded speed profile: how often/where the command drops below 0.3 m/s (stops/holds) and for how long,
    anywhere on the route -- not only at corners;
  * tracking along the whole route: zero-lag horizontal error p50/p90/p99, along/cross split at every
    sample (not only the acceptance's route band), plus a 10 s bin table so a discontinuous stretch is
    visible by position in the run;
  * loops: calls analyze_loops on the same recording and prints how many, where.
Nothing here is an acceptance number; the authority stays scripts/accept.py.
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


def _rows(p):
    with open(p, encoding="utf-8", errors="replace") as fh:
        return list(csv.DictReader(fh))


def load_truth(run):
    r = _rows(os.path.join(run, "tap.csv"))
    tc = "t_sim_us" if r and "t_sim_us" in r[0] else "t_us"
    t = np.array([float(x[tc]) * 1e-6 for x in r])
    P = np.column_stack([[float(x["pe"]) for x in r], [float(x["pn"]) for x in r],
                         [-float(x["pd"]) for x in r]])
    V = np.column_stack([[float(x["ve"]) for x in r], [float(x["vn"]) for x in r]])
    o = np.argsort(t)
    return t[o], P[o], V[o]


def load_cmd(run):
    r = _rows(os.path.join(run, "setpoints.csv"))
    t = np.array([float(x["t_sim_us"]) * 1e-6 for x in r])
    P = np.column_stack([[float(x["cmd_x"]) for x in r], [float(x["cmd_y"]) for x in r],
                         [float(x["cmd_z"]) for x in r]])
    V = np.column_stack([[float(x["vel_x"]) for x in r], [float(x["vel_y"]) for x in r],
                         [float(x["vel_z"]) for x in r]])
    A = np.column_stack([[float(x["acc_x"]) for x in r], [float(x["acc_y"]) for x in r],
                         [float(x["acc_z"]) for x in r]])
    o = np.argsort(t)
    return t[o], P[o], V[o], A[o]


def main() -> int:
    ap = argparse.ArgumentParser(description="whole-path reference/tracking continuity")
    ap.add_argument("--run", default="")
    ap.add_argument("--tag", default="")
    ap.add_argument("--json", default="")
    ap.add_argument("--bin-s", type=float, default=10.0)
    ap.add_argument("--stop-v", type=float, default=0.3)
    a = ap.parse_args()
    if a.run:
        run = os.path.abspath(a.run)
    elif a.tag:
        run = _P.sortie(a.tag)
    else:
        print("--tag or --run required")
        return 2
    tag = os.path.basename(run).replace("missionA-", "")
    t, P, V = load_truth(run)
    tc, PC, VC, AC = load_cmd(run)
    m = P[:, 2] >= 1.0
    t, P, V = t[m], P[m], V[m]
    flew = (tc >= t[0]) & (tc <= t[-1])
    tc, PC, VC, AC = tc[flew], PC[flew], VC[flew], AC[flew]
    # ---- reference continuity, per commanded sample
    dt = np.diff(tc)
    dv = np.linalg.norm(np.diff(VC[:, :2], axis=0), axis=1)
    da = np.linalg.norm(np.diff(AC[:, :2], axis=0), axis=1)
    dv = dv[dt < 0.1] / 1.0
    da = da[dt < 0.1]
    ok = dt < 0.1
    tc2 = tc[1:][ok]
    print("== %s  (%.1f s flown, cmd %d rows)" % (tag, t[-1] - t[0], len(tc)))
    print("reference |Δv| between 20 ms samples: p50 %.4f p90 %.4f p99 %.4f max %.3f m/s"
          % (np.percentile(dv, 50), np.percentile(dv, 90), np.percentile(dv, 99), dv.max()))
    print("reference |Δa| between 20 ms samples: p50 %.3f p90 %.3f p99 %.3f max %.3f m/s^2"
          % (np.percentile(da, 50), np.percentile(da, 90), np.percentile(da, 99), da.max()))
    for thr in (0.05, 0.10, 0.15, 0.25):
        n = int(np.sum(dv > thr))
        print("  |Δv|>%.2f m/s: %d (%.2f %% of samples)" % (thr, n, 100.0 * n / len(dv)))
    # where are the big velocity steps (by position)
    big = np.argsort(dv)[-8:][::-1]
    print("  biggest reference velocity steps:")
    for i in big:
        if i >= len(tc2):
            continue
        j = int(np.searchsorted(tc, tc2[i]))
        print("    t=%.1f s  at cmd (%.1f, %.1f)  |Δv|=%.3f m/s" %
              (tc2[i], PC[j, 0], PC[j, 1], dv[i]))
    # ---- commanded speed profile: stops anywhere on the route
    cs = np.hypot(VC[:, 0], VC[:, 1])
    low = cs < a.stop_v
    runs = []
    i = 0
    while i < len(tc):
        if not low[i]:
            i += 1
            continue
        j = i
        while j + 1 < len(tc) and low[j + 1]:
            j += 1
        if tc[j] - tc[i] > 0.15:
            runs.append((tc[i], tc[j], float(PC[i, 0]), float(PC[i, 1])))
        i = j + 1
    tot = sum(b - c for c, b, _, _ in runs)
    print("\ncommanded |v_xy| < %.1f m/s runs >0.15 s: %d, total %.1f s (%.1f %% of the flight)"
          % (a.stop_v, len(runs), tot, 100.0 * tot / max(1e-9, tc[-1] - tc[0])))
    for c, b, x, y in runs:
        print("    t=%6.1f..%6.1f (%.2f s) at cmd (%.1f, %.1f)" % (c, b, b - c, x, y))
    # ---- tracking along the WHOLE route
    cm = np.column_stack([np.interp(t, tc, PC[:, k]) for k in range(3)])
    vv = np.column_stack([np.interp(t, tc, VC[:, k]) for k in range(2)])
    err = P - cm
    eh = np.hypot(err[:, 0], err[:, 1])
    u = vv / np.maximum(np.linalg.norm(vv, axis=1)[:, None], 1e-9)
    nv = np.column_stack([-u[:, 1], u[:, 0]])
    along = np.sum(err[:, :2] * u, axis=1)
    cross = np.sum(err[:, :2] * nv, axis=1)
    print("\ntracking (whole route, zero lag): p50 %.3f p90 %.3f p99 %.3f max %.3f m"
          % (np.percentile(eh, 50), np.percentile(eh, 90), np.percentile(eh, 99), eh.max()))
    print("  along p50 %+.3f p90 %+.3f | cross |p50| %.3f p90 %.3f m"
          % (np.median(along), np.percentile(along, 90), abs(np.median(cross)),
             np.percentile(np.abs(cross), 90)))
    # ---- 10 s bins: location in the run of the worst stretches
    print("\n  10 s bins:  t0   cmd v mean/min   err p90   big |Δv| count")
    nb = int(np.ceil((tc[-1] - tc[0]) / a.bin_s))
    for b in range(nb):
        t0 = tc[0] + b * a.bin_s
        t1 = t0 + a.bin_s
        mb = (tc >= t0) & (tc < t1)
        mt = (t >= t0) & (t < t1)
        if mb.sum() < 2 or mt.sum() < 2:
            continue
        big_n = int(np.sum((tc2 >= t0) & (tc2 < t1) & (dv > 0.10)))
        print("  %6.1f..%6.1f  %5.2f / %5.2f      %6.3f      %3d"
              % (t0, t1, cs[mb].mean(), cs[mb].min(), np.percentile(eh[mt], 90), big_n))
    # ---- loops
    try:
        import analyze_loops as A
        tt, PP, VV = A.load(run)
        mm = PP[:, 2] >= 1.0
        ev = A.detect_loops(tt[mm], PP[mm])
        print("\nloops (analyze_loops): %d" % len(ev))
        for e in ev:
            print("  t=%.1f..%.1f at (%.1f, %.1f)" % (e["t0"], e["t1"],
                                                      float(np.interp(e["t0"], t, P[:, 0])),
                                                      float(np.interp(e["t0"], t, P[:, 1]))))
    except Exception as exc:  # noqa: BLE001
        print("\nloops: analyze_loops unavailable (%s)" % exc)
    if a.json:
        json.dump({"tag": tag,
                   "dv_p90": float(np.percentile(dv, 90)),
                   "dv_max": float(dv.max()),
                   "stop_runs": len(runs), "stop_seconds": tot,
                   "err_p90": float(np.percentile(eh, 90))}, open(a.json, "w"), indent=2)
        print("\njson: %s" % a.json)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
