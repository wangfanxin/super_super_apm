#!/usr/bin/env python3
"""tracking_diag.py -- WHY the aircraft looks like it is not following the plan.

Four different faults all read as "tracking is terrible" in a single number, and they need different
fixes, so this script separates them:

  1. THE PLAN STEPS.  SUPER replans continuously, so the setpoint stream can contain position steps.
     A 0.3 m step at 50 Hz is a 15 m/s ramp; no position controller follows that.  Census here.
  2. THE PLAN STANDS STILL.  When MINCO fails, the thin layer holds the last setpoint.  The aircraft is
     then told to sit at a stale point while the map expects it elsewhere.  Fraction + segment lengths.
  3. THE CONTROL LOOP LAGS.  Position-only setpoints (this wire sends type_mask 2552, so the velocity
     and acceleration fields are MASKED OFF) make the FCU chase position with a P loop; if the velocity
     loop's I term is doing its job there is no steady lag, only a transient.  Measured, not assumed.
  4. THE TURNS ARE TOO SHARP.  `v^2 * kappa` is the lateral acceleration the plan asks for; if it is
     above what `ATC_ANGLE_MAX`/`WP_ACC` allow, the aircraft cuts the corner and the error appears as
     LATERAL error, not lag.  Binned here.

The two clocks (bridge wall clock vs simulation time) are aligned by a whole-run shift search, and the
search reports its own curve: if the minimum is flat, the truth is not a delayed copy of the plan and the
shift means nothing.

    python3 scripts/tracking_diag.py d85-01-02
"""
import argparse
import csv
import os

import numpy as np


def load_plan(path):
    """(t_wall, xyz in W, vxyz in W).  The bridge rotates cmd velocity into W before logging it."""
    t, p, v = [], [], []
    for r in csv.DictReader(open(path)):
        t.append(float(r["t_wall"]))
        p.append((float(r["cmd_x"]), float(r["cmd_y"]), float(r["cmd_z"])))
        v.append((float(r["vel_x"]), float(r["vel_y"]), float(r["vel_z"])))
    return np.array(t), np.array(p), np.array(v)


def load_truth(path):
    """(t, xyz in W).  The tap logs pe/pn/pd = east/north/down."""
    t, p = [], []
    for r in csv.DictReader(open(path)):
        t.append(float(r["t_us"]) / 1e6)
        p.append((float(r["pe"]), float(r["pn"]), -float(r["pd"])))
    return np.array(t), np.array(p)


def freeze_segments(t, frozen):
    segs = []
    i, n = 0, len(t)
    while i < n:
        if frozen[i]:
            j = i
            while j + 1 < n and frozen[j + 1]:
                j += 1
            if j > i:
                segs.append((t[i], t[j]))
            i = j + 1
        else:
            i += 1
    return segs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tag")
    ap.add_argument("--root", default="/mnt/d/distill_self/super_super_apm/logs")
    ap.add_argument("--speed", type=float, default=0.30, help="below this |v| the plan counts as frozen")
    ap.add_argument("--shift-max", type=float, default=60.0)
    args = ap.parse_args()

    d = os.path.join(args.root, args.tag)
    tp, pp, vp = load_plan(os.path.join(d, "cmd.csv"))
    tt, pt = load_truth(os.path.join(d, "tap.csv"))
    tp = tp - tp[0]
    tt = tt - tt[0]
    keep = np.concatenate(([True], np.diff(tt) > 0))
    tt, pt = tt[keep], pt[keep]
    sp = np.hypot(vp[:, 0], vp[:, 1])

    print("== %s   plan %d samples over %.1f s | truth %d samples over %.1f s"
          % (args.tag, len(tp), tp[-1] - tp[0], len(tt), tt[-1] - tt[0]))

    # ---------------------------------------------------------------- 1) plan step census
    dt = np.diff(tp)
    rate = 1.0 / np.median(dt[dt > 1e-9])
    dp = np.linalg.norm(np.diff(pp, axis=0), axis=1)
    print("   plan stream %.1f Hz | step p50 %.4f p90 %.4f p99 %.4f max %.3f m"
          % (rate, np.percentile(dp, 50), np.percentile(dp, 90), np.percentile(dp, 99), dp.max()))
    for thr in (0.10, 0.30, 1.00):
        n = int((dp > thr).sum())
        print("      steps > %.2f m: %5d (%.2f/s, %.1f %% of samples)"
              % (thr, n, n / max(tp[-1] - tp[0], 1e-9), 100.0 * n / max(len(dp), 1)))
    print("      -> the fastest a smooth plan can move is p99/dt = %.1f m/s; a step bigger than that"
          % (np.percentile(dp, 99) / np.median(dt[dt > 1e-9])))
    print("         is a replan discontinuity.  Steps > 0.30 m: %d." % int((dp > 0.30).sum()))

    # ---------------------------------------------------------------- 2) frozen census
    frozen = sp < args.speed
    segs = freeze_segments(tp, frozen)
    lens = np.array([b - a for a, b in segs]) if segs else np.zeros(1)
    print("   plan frozen below %.2f m/s: %.1f %% of samples | %d segments, p50 %.2f s, p90 %.2f s,"
          " longest %.2f s, total %.1f s"
          % (args.speed, 100.0 * frozen.mean(), len(segs), np.percentile(lens, 50),
             np.percentile(lens, 90), lens.max(), float(lens.sum())))

    # ---------------------------------------------------------------- 3) clock alignment
    grid = np.arange(-args.shift_max, args.shift_max + 1e-9, 0.02)
    sub = np.arange(0, len(tp), 5)
    cost = np.full(len(grid), np.nan)
    for k, sh in enumerate(grid):
        tq = tp[sub] + sh
        ok = (tq >= tt[0]) & (tq <= tt[-1])
        if ok.sum() < 50:
            continue
        q = np.column_stack([np.interp(tq[ok], tt, pt[:, c]) for c in range(3)])
        cost[k] = float(np.median(np.linalg.norm(q - pp[sub][ok], axis=1)))
    best = int(np.nanargmin(cost))
    sh = float(grid[best])
    print("   clock shift: truth lags the plan log by %.2f s (median |plan-truth| %.3f m; at +-1 s"
          " it is %.3f / %.3f) -> %s"
          % (sh, cost[best], cost[max(best - 50, 0)], cost[min(best + 50, len(cost) - 1)],
             "a real, well-defined offset" if (cost[min(best + 50, len(cost) - 1)] - cost[best]) > 0.02
             else "FLAT: not a simple delay"))

    tq = tp + sh
    ok = (tq >= tt[0]) & (tq <= tt[-1])
    q = np.column_stack([np.interp(tq[ok], tt, pt[:, c]) for c in range(3)])
    p0, v0 = pp[ok], vp[ok]
    err = q - p0
    sp0 = np.hypot(v0[:, 0], v0[:, 1])
    moving = sp0 > 0.5

    # ---------------------------------------------------------------- 4) error decomposition
    if moving.sum() > 50:
        vh = v0[moving][:, :2] / sp0[moving][:, None]
        e2 = err[moving][:, :2]
        along = (e2 * vh).sum(axis=1)
        side = e2[:, 0] * (-vh[:, 1]) + e2[:, 1] * vh[:, 0]
        lat = np.abs(side)
        print("   while the plan moves (>0.5 m/s, %.0f %% of samples):" % (100.0 * moving.mean()))
        print("      along-track p50 %+.3f p90(|e|) %+.3f m   (negative = the aircraft is BEHIND)"
              % (np.percentile(along, 50), np.percentile(np.abs(along), 90)))
        print("      lateral   p50 %.3f p90 %.3f p99 %.3f m | signed p05 %+.3f p50 %+.3f p95 %+.3f"
              % (np.percentile(lat, 50), np.percentile(lat, 90), np.percentile(lat, 99),
                 np.percentile(side, 5), np.percentile(side, 50), np.percentile(side, 95)))
        for thr in (0.12, 0.245, 0.50):
            print("         |lateral| > %.3f m for %.1f %% of the moving time"
                  % (thr, 100.0 * (lat > thr).mean()))
        print("      vertical  p50 %+.3f p90(|e|) %.3f m"
              % (np.percentile(err[moving][:, 2], 50),
                 np.percentile(np.abs(err[moving][:, 2]), 90)))

        print("      along-track vs plan speed (a steady P-loop lag shows as a line):")
        for lo, hi in ((0.5, 1.0), (1.0, 1.5), (1.5, 2.0), (2.0, 2.5), (2.5, 4.5)):
            m = moving & (sp0 > lo) & (sp0 <= hi)
            if m.sum() < 20:
                continue
            vv = v0[m][:, :2] / sp0[m][:, None]
            al = (err[m][:, :2] * vv).sum(axis=1)
            print("         %4.1f-%.1f m/s n=%5d  along p50 %+.3f m  |lateral| p50 %.3f p90 %.3f m"
                  % (lo, hi, int(m.sum()), np.percentile(al, 50),
                     np.percentile(np.abs(err[m][:, 0] * (-vv[:, 1]) + err[m][:, 1] * vv[:, 0]), 50),
                     np.percentile(np.abs(err[m][:, 0] * (-vv[:, 1]) + err[m][:, 1] * vv[:, 0]), 90)))

        # ------------------------------------------------------------ 5) sharp turns -> lateral error
        acc = np.gradient(v0[:, :2], axis=0) / max(float(np.median(np.diff(tp))), 1e-6)
        cross = v0[:, 0] * acc[:, 1] - v0[:, 1] * acc[:, 0]
        alat = np.abs(cross) / np.maximum(sp0, 1e-3)
        print("      lateral error against the demanded lateral acceleration v^2*kappa"
              " (tilt 45 deg = 9.8 m/s^2):")
        for lo, hi in ((0.0, 0.5), (0.5, 1.5), (1.5, 3.0), (3.0, 5.0), (5.0, 12.0)):
            m = moving & (alat >= lo) & (alat < hi)
            if m.sum() < 20:
                continue
            vv = v0[m][:, :2] / sp0[m][:, None]
            sd = err[m][:, 0] * (-vv[:, 1]) + err[m][:, 1] * vv[:, 0]
            print("         %4.1f-%.1f m/s^2 n=%5d  |lateral| p50 %.3f p90 %.3f m | plan |v| p50 %.2f"
                  % (lo, hi, int(m.sum()), np.percentile(np.abs(sd), 50),
                     np.percentile(np.abs(sd), 90), np.median(sp0[m])))

    # ---------------------------------------------------------------- 6) what the aircraft did while frozen
    print("   freeze anatomy (the plan holds still -- what is the aircraft doing?)")
    for (t0, t1) in sorted(segs, key=lambda s: s[0] - s[1])[:3]:
        m = (tt >= t0) & (tt <= t1)
        if m.sum() < 20:
            continue
        ps = pt[m]
        vels = np.linalg.norm(np.gradient(ps, tt[m], axis=0), axis=1)
        print("      freeze %.1f-%.1f s (%.1f s): truth wandered %.2f m from where it was when the plan"
              " stopped, net displacement %.2f m, speed p50 %.2f max %.2f m/s"
              % (t0, t1, t1 - t0, float(np.max(np.linalg.norm(ps - ps[0], axis=1))),
                 float(np.linalg.norm(ps[-1] - ps[0])), np.median(vels), vels.max()))


if __name__ == "__main__":
    main()
