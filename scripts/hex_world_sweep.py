#!/usr/bin/env python3
"""hex_world_sweep.py -- with the SAME S2 rules, how wide can the corridors be?  (seed sweep)

The hex world is 1 pillar per 1.0 m cell, random inside the cell, 6 waypoint cells empty, no lane.
Those rules do not fix WHERE in each cell the pillar sits, and that freedom is worth a lot: the
per-leg bottleneck b (the widest route's minimum surface clearance, `make_world_v2_hex.py`'s own
number) is 0.245-0.524 m for the frozen seed.  A 3 m/s plan needs a turn radius of v^2/a ~ 1.1 m,
so b is the number that decides whether SUPER can plan at all.

This script re-runs the generator over seeds, parses its audit, and prints the distribution of
    min_b      the worst leg's bottleneck        (larger = roomier)
    astar_min  the worst leg's A* route clearance
    free       free-space fraction at d >= 0.10 m
so the owner can see what the rules can deliver before deciding how to change them.

    python3 scripts/hex_world_sweep.py --seeds 20240521 1 2 3 ... --jobs 4
"""
import argparse
import os
import re
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GEN = os.path.join(ROOT, "scripts", "make_world_v2_hex.py")


def one(seed, extra):
    out = os.path.join(tempfile.gettempdir(), "hex_sweep_%d.yaml" % seed)
    cmd = [sys.executable, GEN, "--seed", str(seed), "--out", out] + extra
    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode != 0:
        return {"seed": seed, "rc": p.returncode, "min_b": None, "astar_min": None, "free": None}
    txt = p.stdout
    # the per-leg table is: `pad->c1   5.00 m   +0.245 m   5.25 m   +0.106 m`
    # (leg, straight, b, A* length, A* route min).  Anchoring on the leg name is what keeps the two
    # "+x.xxx m" columns apart -- a loose regex reads the A* column as b and reports the wrong number.
    bs, ast = [], []
    for line in txt.splitlines():
        m = re.match(r"^\s+(pad->c1|c\d->c\d)\s+[\d.]+ m\s+([+-][\d.]+) m\s+"
                     r"(?:n/a|[\d.]+) m\s+([+-][\d.]+) m", line)
        if m:
            bs.append(float(m.group(2)))
            ast.append(float(m.group(3)))
    free = re.search(r"free-space fraction\s+:\s+([\d.]+) %", txt)
    try:
        os.remove(out)
    except OSError:
        pass
    return {"seed": seed, "rc": 0,
            "min_b": min(bs) if bs else None,
            "astar_min": min(ast) if ast else None,
            "free": float(free.group(1)) if free else None}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", nargs="+", type=int,
                    default=[20240521] + list(range(1, 13)))
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--extra", nargs=argparse.REMAINDER, default=[],
                    help="extra generator args, e.g. --extra --r 0.08")
    a = ap.parse_args()
    with ThreadPoolExecutor(max_workers=a.jobs) as ex:
        rows = list(ex.map(lambda s: one(s, a.extra), a.seeds))
    rows.sort(key=lambda r: (-(r["min_b"] if r["min_b"] is not None else -9)))
    print("%-12s %4s %9s %10s %7s" % ("seed", "rc", "min_b", "astar_min", "free%"))
    for r in rows:
        print("%-12d %4d %9s %10s %7s"
              % (r["seed"], r["rc"],
                 "-" if r["min_b"] is None else "%+.3f" % r["min_b"],
                 "-" if r["astar_min"] is None else "%+.3f" % r["astar_min"],
                 "-" if r["free"] is None else "%.1f" % r["free"]))
    ok = [r for r in rows if r["min_b"] is not None]
    if ok:
        best = ok[0]
        print()
        print("best seed by min_b: %d  (min_b %+.3f m, A* route min %+.3f m, free %.1f %%)"
              % (best["seed"], best["min_b"], best["astar_min"], best["free"]))
        print("worst:              %d  (min_b %+.3f m)"
              % (ok[-1]["seed"], ok[-1]["min_b"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
