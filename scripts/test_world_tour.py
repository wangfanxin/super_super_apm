#!/usr/bin/env python3
"""test_world_tour.py -- the two-lap tour really is two laps.

The 87th round found that `--tour world` ignored `--laps` and always flew every corner the world
file declares (18 for the frozen hex world), so "LAPS=2" produced 19 goals instead of the 13 the
SPEC S0 asks for.  `world_tour()` now truncates against the world's own `mission.laps`; this test
pins the behaviour so it cannot silently regress.

    python3 scripts/test_world_tour.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src"))
sys.path.insert(0, os.path.join(ROOT, "src", "whtf"))

from whtf.ros.planner_source_super import world_tour  # noqa: E402

WORLD = os.path.join(ROOT, "worlds", "forest_hex12.yaml")


def main():
    bad = []
    c2, i2 = world_tour(WORLD, 2, 2.0, verbose=False)
    print("laps=2 -> %d corners, info=%s" % (len(c2), i2))
    if len(c2) != 12:
        bad.append("laps=2 should give 12 corners, got %d" % len(c2))
    if not i2["truncated"]:
        bad.append("laps=2 against a 3-lap world should be reported as truncated")
    if i2["corners_per_lap"] != 6:
        bad.append("corners per lap should be 6, got %s" % i2["corners_per_lap"])

    c3, i3 = world_tour(WORLD, 3, 2.0, verbose=False)
    print("laps=3 -> %d corners, info=%s" % (len(c3), i3))
    if len(c3) != 18:
        bad.append("laps=3 should give all 18 corners, got %d" % len(c3))
    if i3["truncated"]:
        bad.append("laps=3 is the world's own count; it must not be truncated")

    c9, i9 = world_tour(WORLD, 9, 2.0, verbose=False)
    print("laps=9 -> %d corners (a lap cannot be invented)" % len(c9))
    if len(c9) != 18:
        bad.append("asking for more laps than the world declares must not invent corners")

    # ROUND 88: the SPEC hexagon has a 10.000 m side (= circumradius), and the tour is that ring
    import math
    side = [math.hypot(*(c3[(k + 1) % 6] - c3[k])[:2]) for k in range(6)] if len(c3) >= 6 else []
    if side:
        print("hexagon side: min %.6f max %.6f m" % (min(side), max(side)))
        if max(abs(x - 10.0) for x in side) > 1e-6:
            bad.append("the hexagon side is not 10.000 m: %s" % side)

    # the first two laps of the three-lap tour are exactly the first 12 corners, in order
    if len(c3) == 18 and len(c2) == 12:
        if not all(abs(c2[k][0] - c3[k][0]) < 1e-12 and abs(c2[k][1] - c3[k][1]) < 1e-12
                   for k in range(12)):
            bad.append("the truncated tour is not a prefix of the full tour")

    print()
    if bad:
        for b in bad:
            print("FAIL  %s" % b)
        return 1
    print("PASS  world_tour: 2 laps = 12 corners (+ the pad leg = 13 goals), 3 laps = 18, no invention")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
