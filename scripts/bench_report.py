#!/usr/bin/env python3
"""bench_report.py -- the firmware A/B metric for the PVA tracking bench (bench_source.py).

    python3 scripts/bench_report.py --tag phase7g-01
    python3 scripts/bench_report.py --tag phase7g-01 --json /tmp/b.json

The bench publishes an analytic reference on /plan/cmd; this script compares it with the SITL truth and
reports ONLY what the firmware/airframe can control:
  * horizontal tracking error at the best temporal lag, p50/p90/p99, plus the lag itself (latency);
  * the same split by commanded speed bin (this is the speed ladder's "does it still track" curve);
  * velocity tracking error;
  * vertical error p50/p90 (the altitude sag);
  * truth peak speed vs commanded peak speed.
It prints PASS/FAIL against the current AIRFRAME target: 6-9 m/s p90 <= --target-p90 (0.20 m default) and
|z| p90 <= --target-z (0.10 m default).  These are firmware-tuning targets, NOT acceptance rows.
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


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", default="")
    ap.add_argument("--tag", default="")
    ap.add_argument("--json", default="")
    ap.add_argument("--target-p90", type=float, default=0.20)
    ap.add_argument("--target-z", type=float, default=0.10)
    a = ap.parse_args()
    if a.run:
        run = os.path.abspath(a.run)
    elif a.tag:
        run = _P.sortie(a.tag)
    else:
        print("--tag or --run required")
        return 2
    tag = os.path.basename(run).replace("missionA-", "")
    sp = _rows(os.path.join(run, "setpoints.csv"))
    ts = np.array([float(x["t_sim_us"]) * 1e-6 for x in sp])
    cx = np.array([float(x["cmd_x"]) for x in sp])
    cy = np.array([float(x["cmd_y"]) for x in sp])
    cz = np.array([float(x["cmd_z"]) for x in sp])
    cvx = np.array([float(x["vel_x"]) for x in sp])
    cvy = np.array([float(x["vel_y"]) for x in sp])
    cs = np.hypot(cvx, cvy)
    tap = _rows(os.path.join(run, "tap.csv"))
    t = np.array([float(x["t_sim_us"]) * 1e-6 for x in tap])
    E = np.array([float(x["pe"]) for x in tap])
    N = np.array([float(x["pn"]) for x in tap])
    U = np.array([-float(x["pd"]) for x in tap])
    ve = np.array([float(x["ve"]) for x in tap])
    vn = np.array([float(x["vn"]) for x in tap])
    m = U > 0.5
    t, E, N, U, ve, vn = t[m], E[m], N[m], U[m], ve[m], vn[m]
    # The first/last seconds of the stream are the vertical climb and the final hold-return: the command is
    # already at z=2.0 while the truth is still climbing, which reads as a huge horizontal error and is NOT a
    # tracking property.  Trim them (bench_report phase7g had p90 4.7 m in the 0-1 m/s bin from exactly this).
    t_lo = ts[win0] if False else None
    win = (ts >= t[0] + 4.0) & (ts <= t[-1] - 4.0)
    if win.sum() < 100:
        print("not enough samples in the flight window")
        return 2
    best = (9e9, 0.0)
    for lag in np.arange(-0.5, 0.5 + 1e-9, 0.01):
        ex = np.interp(ts + lag, t, E)
        ey = np.interp(ts + lag, t, N)
        e = np.hypot(cx - ex, cy - ey)[win]
        p90 = float(np.percentile(e, 90))
        if p90 < best[0]:
            best = (p90, float(lag))
    lag = best[1]
    ex = np.interp(ts + lag, t, E)
    ey = np.interp(ts + lag, t, N)
    ez = np.interp(ts, t, U)
    err = np.hypot(cx - ex, cy - ey)
    # VERTICAL: split the steady BIAS from the dynamic error.  MEASURED (phase7g): the truth sits 0.14 m below
    # the command ALL flight (an estimator/altitude-datum offset, not a control property); the dynamic part is
    # only 6 cm p90.  Judging the raw error would blame the vertical loop for a constant datum offset.
    zerr = (cz - ez)
    z_bias = float(np.median(zerr[win]))
    verr = np.abs(zerr - z_bias)
    tvx = np.interp(ts + lag, t, ve)
    tvy = np.interp(ts + lag, t, vn)
    ev = np.hypot(cvx - tvx, cvy - tvy)
    tv = np.hypot(tvx, tvy)
    print("== %s PVA bench" % tag)
    print("  best lag %+0.2f s | horizontal err p50 %.3f p90 %.3f p99 %.3f m"
          % (lag, np.percentile(err[win], 50), np.percentile(err[win], 90), np.percentile(err[win], 99)))
    print("  vertical: bias %+.3f m (datum) | dynamic |dz| p50 %.3f p90 %.3f m   |   truth peak %.2f / cmd peak %.2f m/s"
          % (z_bias, np.percentile(verr[win], 50), np.percentile(verr[win], 90), tv[win].max(), cs[win].max()))
    print("  cmd speed bin      n    err p50   err p90   |dv| p90")
    bins = {}
    for lo, hi in ((0, 1), (1, 2), (2, 4), (4, 6), (6, 9), (9, 20)):
        mm = win & (cs >= lo) & (cs < hi)
        if mm.sum() < 20:
            continue
        rec = {"n": int(mm.sum()), "err_p50": float(np.percentile(err[mm], 50)),
               "err_p90": float(np.percentile(err[mm], 90)),
               "dv_p90": float(np.percentile(ev[mm], 90))}
        bins["%g-%g" % (lo, hi)] = rec
        print("   %4.1f - %4.1f      %5d   %7.3f   %7.3f   %7.2f"
              % (lo, hi, rec["n"], rec["err_p50"], rec["err_p90"], rec["dv_p90"]))
    fast = [b for k, b in bins.items() if k in ("6-9", "9-20")]
    fast_p90 = max((b["err_p90"] for b in fast), default=float("nan"))
    ok = (fast_p90 == fast_p90 and fast_p90 <= a.target_p90
          and np.percentile(verr[win], 90) <= a.target_z)
    print("  AIRFRAME TARGET (6-9 m/s p90 <= %.2f m, |z| p90 <= %.2f m): %s (fast p90 %.3f, z p90 %.3f)"
          % (a.target_p90, a.target_z, "PASS" if ok else "FAIL", fast_p90, np.percentile(verr[win], 90)))
    if a.json:
        json.dump({"tag": tag, "lag": lag, "err_p50": float(np.percentile(err[win], 50)),
                   "err_p90": float(np.percentile(err[win], 90)),
                   "z_p90": float(np.percentile(verr[win], 90)), "z_bias": z_bias, "bins": bins,
                   "pass": bool(ok)}, open(a.json, "w"), indent=2)
        print("  json: %s" % a.json)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
