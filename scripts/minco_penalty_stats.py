#!/usr/bin/env python3
"""minco_penalty_stats.py -- the optimizer's own violation numbers, from the log it prints.

With `traj_opt.print_optimizer_log: true` SUPER prints, for every optimisation attempt that fails
(or every attempt, with the failure block), the MAX violation of each bound:

    Pos  = max(|p - polytope|)                       reject if > 0.2
    Vel  = max(|v| - max_vel)                        (its reject line is commented out)
    Acc  = max(|a| - max_acc)                        reject if > max_acc * penna_margin = 0.4
    Omg  = max(|omg| - max_omg)                      reject only if penna_omg > 0 (it is -1: off)
    Thr  = max(|thr - thrustMean| - bound)           reject only if penna_thr > 0 (it is -1: off)

So at 3 m/s the decisive question is: how far over max_acc does the best trajectory actually go?
This script reads the log and prints the distribution, plus what fraction of attempts would pass the
0.4 m/s^2 margin.

    python3 scripts/minco_penalty_stats.py v3d-01-01 [--max-acc 8.0] [--margin 0.05]
"""
import argparse
import os
import re

import numpy as np

MISSION = "/home/fanxin/lio_avoid/logs"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tag")
    ap.add_argument("--max-acc", type=float, default=8.0)
    ap.add_argument("--margin", type=float, default=0.05)
    a = ap.parse_args()
    path = os.path.join(MISSION, "missionA-" + a.tag, "traj.log")
    if not os.path.isfile(path):
        raise SystemExit("no traj.log for %s" % a.tag)
    vals = {k: [] for k in ("Pos", "Vel", "Acc", "Jerk", "Omg", "Thr")}
    pat = re.compile(r"^\s*(Pos|Vel|Acc|Jerk|Omg|Thr):\s*([-0-9.eE+]+)")
    n = 0
    with open(path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            m = pat.match(line)
            if m:
                vals[m.group(1)].append(float(m.group(2)))
                if m.group(1) == "Pos":
                    n += 1
    if n == 0:
        raise SystemExit("no penalty blocks in %s -- was print_optimizer_log on?" % path)
    thr_v = a.max_acc * a.margin
    print("== %s : %d optimisation blocks printed" % (a.tag, n))
    print("   the reject thresholds are Pos > 0.200, Acc > %.3f (= max_acc %.1f x penna_margin %.3f);"
          % (thr_v, a.max_acc, a.margin))
    print("   Omg / Thr are switched OFF in this config (penna_omg = penna_thr = -1), Vel's reject")
    print("   line is commented out in exp_traj_optimizer_s4.cpp:716 -- so Acc and Pos are the only")
    print("   hard gates left.")
    print()
    print("   %-5s %8s %8s %8s %8s %8s %8s" % ("chan", "n", "p50", "p90", "p99", "max", ">thr"))
    for k, v in vals.items():
        if not v:
            continue
        arr = np.asarray(v, dtype=float)
        thr = 0.2 if k == "Pos" else (thr_v if k in ("Acc", "Thr", "Vel") else np.inf)
        frac = float(np.mean(arr > thr)) if np.isfinite(thr) else float("nan")
        print("   %-5s %8d %8.3f %8.3f %8.3f %8.3f %8s"
              % (k, len(arr), np.percentile(arr, 50), np.percentile(arr, 90),
                 np.percentile(arr, 99), arr.max(),
                 "%.1f%%" % (100 * frac) if np.isfinite(frac) else "-"))
    print()
    print("   reading: `Acc p50/p90` is how far the BEST trajectory the optimiser found exceeds")
    print("   max_acc.  If most blocks are over the 0.4 margin, the mission speed is asking for")
    print("   more acceleration than the configured bound allows -- a planner-level infeasibility,")
    print("   not a tuning nuisance.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
