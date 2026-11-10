"""z_plan_audit.py -- did the PLAN ask for the altitude, or did the aircraft do it on its own?

Round 85's `pva1` sorties broke the altitude row (truth to 7.09 m on a 2.00 m mission).  Before blaming any
control channel, this compares the PLAN's own z with the truth's z for a list of sorties: if cmd_z stays at
2.0 while the truth climbs, the aircraft left the reference; if cmd_z climbs with the truth, the PLANNER
asked for it and the aircraft merely obeyed.

    python3 scripts/z_plan_audit.py pv1-02-02 pva1-01-01 pva1-02-02 soft-02-02
"""
import csv
import os
import sys

import numpy as np

MISSION = "/home/fanxin/lio_avoid/logs"


def main():
    print("%-12s %10s %10s %10s %10s   %s" % ("tag", "cmd_z p50", "cmd_z p99", "cmd_z max",
                                              "truth max", "verdict"))
    for tag in sys.argv[1:]:
        d = os.path.join(MISSION, "missionA-" + tag)
        sp = os.path.join(d, "setpoints.csv")
        tp = os.path.join(d, "tap.csv")
        if not (os.path.isfile(sp) and os.path.isfile(tp)):
            print("%-12s (missing)" % tag)
            continue
        cz = np.array([float(r["cmd_z"]) for r in csv.DictReader(open(sp))])
        tz = np.array([-float(r["pd"]) for r in csv.DictReader(open(tp))])
        verdict = "aircraft left the plan" if cz.max() < 2.6 else "THE PLAN ASKED FOR IT"
        print("%-12s %10.2f %10.2f %10.2f %10.2f   %s"
              % (tag, np.percentile(cz, 50), np.percentile(cz, 99), cz.max(), tz.max(), verdict))


if __name__ == "__main__":
    main()
