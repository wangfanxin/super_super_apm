#!/usr/bin/env python3
"""leg_events.py -- what was the planner DOING during each leg of the tour?

The three clocks in this harness (the plan box's monotonic `t_wall`, fsm_node's wall epoch, SITL's
boot clock) never got joined for `missionA-kwC-01-01`, so a stall in the published reference could
not be attributed.  This script sidesteps the clocks: `traj.log` is a SINGLE file into which both
the plan box and fsm_node write, so the interleaving preserves order.  The plan box marks each leg
with `LEG k (...)` and `ARRIVED at waypoint k`, so every planner line can be attributed to a leg
without a clock at all.

    python3 scripts/leg_events.py kwC-01-01

Reading of the columns:
  fin   `-- [Fsm] Traj finish.`          the committed trajectory ran out (reference now clamps at its end, v=0)
  plan+ `PlanFromRest ... SUCCESS`       a new rest-to-rest plan was accepted
  plan- `PlanFromRest failed`            a plan attempt was REJECTED -- nothing new to publish
  repl  `ReplanOnce succeed`             a 10 Hz re-plan tick that changed something
  tr    `change state` transitions       FOLLOW_TRAJ -> GENERATE_TRAJ is the FSM leaving the follow loop
"""
import os
import re
import sys

ANSI = re.compile(r"\x1b\[[0-9;]*m")
ROOT = "/home/fanxin/lio_avoid/logs"


def main():
    tag = sys.argv[1]
    path = os.path.join(ROOT, "missionA-" + tag, "traj.log")
    leg = 0
    legs = {}

    def bump(k, v=1):
        d = legs.setdefault(leg, dict(fin=0, plan=0, fail=0, repl=0, tr=0, hold=0, arrived=0))
        d[k] += v

    order = []
    for raw in open(path, errors="replace"):
        line = ANSI.sub("", raw)
        m = re.search(r"^\s*LEG (\d+) ", line)
        if m:
            leg = int(m.group(1))
            order.append(leg)
            bump("arrived", 0)
            continue
        if "ARRIVED at waypoint" in line:
            bump("arrived")
            continue
        if "Traj finish" in line:
            bump("fin")
        elif "PlanFromRest] GenerateExpTrajectory SUCCESS" in line:
            bump("plan")
        elif "PlanFromRest failed" in line:
            bump("fail")
        elif "ReplanOnce succeed" in line:
            bump("repl")
        elif "change state" in line:
            bump("tr")
        elif "HOLD" in line or "hold" in line:
            bump("hold")

    print("== %s: planner activity per leg (legs in the order the plan box announced them)" % tag)
    print("   leg  fin plan+ plan-  repl  state  hold  arrived")
    tot = dict(fin=0, plan=0, fail=0, repl=0, tr=0, hold=0)
    for k in sorted(legs):
        d = legs[k]
        for kk in tot:
            tot[kk] += d[kk]
        print("   %3d  %4d %5d %5d %5d %6d %5d %7d"
              % (k, d["fin"], d["plan"], d["fail"], d["repl"], d["tr"], d["hold"], d["arrived"]))
    print("   ---")
    print("   all  %4d %5d %5d %5d %6d %5d"
          % (tot["fin"], tot["plan"], tot["fail"], tot["repl"], tot["tr"], tot["hold"]))


if __name__ == "__main__":
    main()
