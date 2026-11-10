#!/usr/bin/env python3
"""ref_timeline.py -- put SUPER's own narration and the published reference on ONE clock.

Why this exists: the plan box writes `setpoints.csv` on a monotonic clock (`t_wall`) while
`fsm_node` stamps its ROS lines with the wall epoch, and the aircraft's own clock is the SITL boot
clock (`t_sim_us`).  The three were being compared by eye, which is how "the reference stops
mid-leg" stayed unexplained: on one clock it looked like a planner sag, on another like a hold.

The bridges used here, in order of reliability:

  * `[ROGMAP] odom cb n=K` gives the odometry message COUNT, and `lio.csv` is that same stream
    recorded row by row -- so row K of lio.csv is the very message counted as K.
  * the same log lines carry the wall epoch, so (epoch, sim) pairs pin the affine map between them.
  * `setpoints.csv` carries both `t_wall` and `t_sim_us`, so once the map is known the published
    reference and the planner's narration share an axis.

    python3 scripts/ref_timeline.py kwC-01-01           # 1 s buckets
    python3 scripts/ref_timeline.py kwC-01-01 --check   # how good is the clock fit?
    python3 scripts/ref_timeline.py kwC-01-01 30 60     # only sim t in [30, 60)
"""
import csv
import os
import re
import sys

import numpy as np

ROOT = "/home/fanxin/lio_avoid/logs"
ANSI = re.compile(r"\x1b\[[0-9;]*m")
EPOCH = re.compile(r"\[(\d{9,}\.\d+)\]")


def read_sim_us(path):
    return np.array([int(r["t_us"]) for r in csv.DictReader(open(path))], dtype=float) * 1e-6


def parse_log(path):
    """-> (anchors [(epoch, count)], events [(epoch, kind, detail)])."""
    anchors, events = [], []
    for raw in open(path, errors="replace"):
        line = ANSI.sub("", raw)
        m = EPOCH.search(line)
        ep = float(m.group(1)) if m else None
        m = re.search(r"odom cb n=(\d+)", line)
        if m and ep is not None:
            anchors.append((ep, int(m.group(1))))
        if ep is None:
            continue
        if "Traj finish" in line:
            events.append((ep, "finish", ""))
        elif "PlanFromRest] GenerateExpTrajectory SUCCESS" in line:
            events.append((ep, "plan+", ""))
        elif "PlanFromRest failed" in line:
            events.append((ep, "plan-", ""))
        elif "ReplanOnce succeed" in line:
            events.append((ep, "replan", ""))
        elif "change state from" in line:
            m2 = re.search(r"from \[(\w+)\] to \[(\w+)\]", line)
            if m2:
                events.append((ep, "state", "%s>%s" % m2.groups()))
    return anchors, events


def fit_clock(anchors, lio_us, verbose=False):
    """epoch -> sim, from (epoch, odom count) anchors and the lio.csv row times."""
    pts = [(ep, lio_us[n - 1]) for ep, n in anchors if 1 <= n <= len(lio_us)]
    if len(pts) < 2:
        return None, None, 0.0
    ep = np.array([p[0] for p in pts])
    sm = np.array([p[1] for p in pts])
    # robust: epoch = sim + offset   (both advance at 1 s per s; fit the offset only)
    off = np.median(ep - sm)
    resid = (ep - sm) - off
    if verbose:
        print("   anchors %d, offset %.3f s, residual p50 %.3f s / max %.3f s"
              % (len(pts), off, np.median(np.abs(resid)), np.max(np.abs(resid))))
    return lambda e, _o=off: e - _o, len(pts), float(np.max(np.abs(resid)))


def main():
    tag = sys.argv[1]
    rest = [a for a in sys.argv[2:]]
    d = os.path.join(ROOT, "missionA-" + tag)
    lio_us = read_sim_us(os.path.join(d, "lio.csv"))
    anchors, events = parse_log(os.path.join(d, "traj.log"))
    to_sim, n_anchor, worst = fit_clock(anchors, lio_us, verbose=True)
    if "--check" in rest:
        return
    lo = float(rest[0]) if len(rest) > 0 else 0.0
    hi = float(rest[1]) if len(rest) > 1 else 1e9

    sp = list(csv.DictReader(open(os.path.join(d, "setpoints.csv"))))
    ts = np.array([float(r["t_sim_us"]) * 1e-6 for r in sp])
    pur = np.array([[float(r["cmd_x"]), float(r["cmd_y"]), float(r["cmd_z"])] for r in sp])
    vel = np.array([[float(r["vel_x"]), float(r["vel_y"]), float(r["vel_z"])] for r in sp])
    spd = np.hypot(vel[:, 0], vel[:, 1])
    # the reference can only *advance* if a fresh plan is being sampled; a frozen cmd position with
    # zero velocity is the aircraft being told "stay", whatever the reason.
    dp = np.r_[0.0, np.linalg.norm(np.diff(pur, axis=0), axis=1)]

    ev = [(to_sim(e), k, det) for e, k, det in events]
    ev.sort()
    kinds = ["finish", "plan+", "plan-", "replan", "state"]

    print("   sim   wall  v_ref p50/max  move m/s |  fin  plan+ plan- repl | events")
    T = np.arange(lo, hi, 1.0)
    for t0 in T:
        m = (ts >= t0) & (ts < t0 + 1.0)
        if not m.any():
            continue
        cnt = {k: sum(1 for tt, kk, _ in ev if t0 <= tt < t0 + 1.0 and kk == k) for k in kinds}
        det = " ".join("%s%s" % (("" if k != "state" else "st:"), d2)
                       for tt, k, d2 in ev if t0 <= tt < t0 + 1.0 and k == "state")
        print("  %5.1f  %5.1f   %4.2f/%4.2f    %5.2f    | %4d %5d %5d %4d | %s"
              % (t0, ts[m][0], np.percentile(spd[m], 50), spd[m].max(),
                 dp[m].sum(), cnt["finish"], cnt["plan+"], cnt["plan-"], cnt["replan"], det))


if __name__ == "__main__":
    main()
