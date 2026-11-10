#!/usr/bin/env python3
"""cusp_context.py -- what was the planner DOING when the reference folded?

The thin layer now prints one line per reference fold (`CUSP #n: the reference folded ...`).  Those
lines go into the SAME `traj.log` as `fsm_node`'s narration, in real order, so the lines around a
fold say which state the planner was in.  This prints them compactly, with the SUPER spam removed.

    python3 scripts/cusp_context.py kx4-01-01
    python3 scripts/cusp_context.py kx4-01-01 --before 6 --after 6 --all
"""
import argparse
import os
import re

ANSI = re.compile(r"\x1b\[[0-9;]*m")
DROP = (
    "Load param", "Load config file", "PolyMSG", "[ROGMAP] cloud cb", "[ROGMAP] odom cb",
    "[Bench]", "Use sim time", "[ROG] ", "virtual_ceil", "virtual_ground", "l_hit", "l_miss",
    "l_min", "l_max", "l_occ", "l_free", "map index size", "map vox_num", "resolution:",
    "map_sliding", "local_map_size", "[INFO] [",
)
KEEP = (
    "CUSP", "goals issued", "change state", "ReplanOnce succeed", "PlanFromRest", "Traj finish",
    "ARRIVED", "LEG ", "GOAL ", "SUPER ref", "plan box", "OptimizationExpTraj", "PathSearch",
    "corridor", "Minco", "Replan over", "KEEP", "yaw", "YAW", "HOLD", "hold", "stale",
)


def interesting(line):
    if any(d in line for d in DROP):
        return False
    return any(k in line for k in KEEP)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tag")
    ap.add_argument("--before", type=int, default=6)
    ap.add_argument("--after", type=int, default=6)
    ap.add_argument("--all", action="store_true", help="print every kept line, cusp lines marked")
    ap.add_argument("--root", default="/home/fanxin/lio_avoid/logs")
    ap.add_argument("--quiet", action="store_true",
                    help="drop the 10 Hz `ReplanOnce succeed` lines, so the state changes stand out")
    a = ap.parse_args()

    path = os.path.join(a.root, "missionA-" + a.tag, "traj.log")
    lines = [ANSI.sub("", ln.rstrip("\n")) for ln in open(path, errors="replace")]
    if a.quiet:
        lines = ["" if "ReplanOnce succeed" in ln else ln for ln in lines]
    keep = [(i, ln) for i, ln in enumerate(lines) if interesting(ln)]
    cusps = [(i, ln) for i, ln in keep if "CUSP #" in ln]
    print("== %s: %d CUSP lines, %d kept lines of %d" % (a.tag, len(cusps), len(keep), len(lines)))

    if a.all:
        for i, ln in keep:
            print("  %s" % ln[:150])
        return

    for n, (i, ln) in enumerate(cusps):
        print("\n---- %s" % ln[:150])
        prev = [x for x in keep if x[0] < i][-a.before:]
        nxt = [x for x in keep if x[0] > i][:a.after]
        for _, l in prev:
            print("   before | %s" % l[:140])
        for _, l in nxt:
            print("   after  | %s" % l[:140])


if __name__ == "__main__":
    main()
