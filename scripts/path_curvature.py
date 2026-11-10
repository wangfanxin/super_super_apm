#!/usr/bin/env python3
"""path_curvature.py -- the path's own turning radius, and the speed that radius allows.

A multirotor leans to turn: the lateral acceleration it can produce is at most g*tan(tilt_max), i.e.
9.8 m/s^2 at the 45 deg this arm allows.  A path of radius R therefore has a speed ceiling v <= sqrt(a*R).
This script measures R along the reference and answers two questions at once:

    is the plan's path geometrically flyable at the mission's corner speed?
    and at what speed would it become flyable?

R is computed from the reference itself (kappa = |v x a| / |v|^3 with a = d v / dt of the 50 Hz setpoint
stream), so it is a property of what SUPER asked for, not of how the aircraft flew.

    python3 scripts/path_curvature.py v4a-01 d84-01-01 d85-01-02 pv1-01-01
"""
import argparse
import csv
import math
import os

import numpy as np

MISSION = "/home/fanxin/lio_avoid/logs"
COMPACT = "/mnt/d/distill_self/super_super_apm/logs"


def find(tag, root):
    for cand in (os.path.join(MISSION, "missionA-" + tag, "setpoints.csv"),
                 os.path.join(root, tag, "cmd.csv")):
        if os.path.isfile(cand):
            return cand
    return None


def load(path):
    t, p, v = [], [], []
    for r in csv.DictReader(open(path)):
        if "t_wall" not in r:
            continue
        t.append(float(r["t_wall"]))
        p.append((float(r["cmd_x"]), float(r["cmd_y"])))
        v.append((float(r["vel_x"]), float(r["vel_y"])))
    t = np.array(t)
    return t - t[0], np.array(p), np.array(v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tags", nargs="+")
    ap.add_argument("--root", default=COMPACT)
    ap.add_argument("--speed", type=float, default=0.5, help="only samples faster than this")
    ap.add_argument("--tilt", type=float, default=45.0, help="the arm's ATC_ANGLE_MAX, for the ceiling")
    ap.add_argument("--want", type=float, default=0.0,
                    help="a speed to test against the tightest tenth of the path, m/s: prints the "
                         "lateral acceleration it demands and how far past the airframe's ceiling "
                         "that is (round 87: the arm asks 3.0, this is the number that decides it)")
    ap.add_argument("--plan-max-acc", type=float, default=0.0,
                    help="the planner's own traj_opt.boundary.max_acc, to compare against as well")
    a = ap.parse_args()
    amax = 9.80665 * math.tan(math.radians(a.tilt))
    print("lateral acceleration ceiling: g*tan(%.0f deg) = %.2f m/s^2" % (a.tilt, amax))
    if a.want > 0:
        print("testing the mission speed %.2f m/s against R p10 of each path"
              % a.want + ("" if a.plan_max_acc <= 0 else
                          " (planner max_acc %.1f m/s^2)" % a.plan_max_acc))
        print("%-11s %7s %10s %12s %12s" % (
            "tag", "R p10", "a(v,R p10)", "x tilt ceiling", "x plan max_acc"))
    print("%-11s %7s %7s %7s %7s %9s %9s %9s" % (
        "tag", "R p10", "R p25", "R p50", "R p90", "v_ceiling", "v p50", "v max"))
    for tag in a.tags:
        path = find(tag, a.root)
        if path is None:
            print("%-11s (no cmd log)" % tag)
            continue
        t, p, v = load(path)
        if len(t) < 100:
            print("%-11s (too few timed rows)" % tag)
            continue
        dt = float(np.median(np.diff(t)))
        acc = np.gradient(v, dt, axis=0)
        sp = np.hypot(v[:, 0], v[:, 1])
        cross = np.abs(v[:, 0] * acc[:, 1] - v[:, 1] * acc[:, 0])
        kap = np.zeros(len(t))
        m = sp > 1e-3
        kap[m] = cross[m] / sp[m] ** 3
        use = sp > a.speed
        R = 1.0 / np.maximum(kap[use], 1e-6)
        # the speed the *tightest tenth* of the path allows
        v_ceiling = math.sqrt(amax * float(np.percentile(R, 10)))
        print("%-11s %7.2f %7.2f %7.2f %7.2f %9.2f %9.2f %9.2f" % (
            tag, np.percentile(R, 10), np.percentile(R, 25), np.percentile(R, 50),
            np.percentile(R, 90), v_ceiling, np.percentile(sp[use], 50), sp.max()))
        if a.want > 0:
            r10 = float(np.percentile(R, 10))
            need = a.want ** 2 / max(r10, 1e-6)
            print("%-11s %7.2f %10.2f %12s %12s" % (
                "", r10, need, "%.2fx" % (need / amax),
                "-" if a.plan_max_acc <= 0 else "%.2fx" % (need / a.plan_max_acc)))
        # ROUND 87: the decisive fraction -- on how much of the REFERENCE does its own geometry ask
        # for more lateral acceleration than the airframe can produce?  A few percent is a tracking
        # error; most of the route means the reference is not flyable at all.
        over = (sp[use] ** 2 / np.maximum(R, 1e-6)) > amax
        print("%-11s %7s %10s %12s %12s" % (
            "", "over", "%.1f%%" % (100.0 * float(np.mean(over))), "", ""))


if __name__ == "__main__":
    main()
