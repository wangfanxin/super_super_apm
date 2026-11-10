#!/usr/bin/env python3
"""corner_clearance.py -- how much room does each WAYPOINT have?  (the number that sets the corner speed)

Why this and not the leg bottleneck: a hexagon leg is straight, and the aircraft only has to *turn* at
the corners.  The turn needs a radius R, and a multirotor can pull at most g*tan(tilt) laterally, so the
corner speed is v <= sqrt(a * R).  R here is the free radius around the waypoint -- distance from the
waypoint to the nearest pillar SURFACE (the box, at the mission altitude) -- which is exactly what the
"9 cells around a waypoint stay empty" rule buys.

    python3 scripts/corner_clearance.py worlds/forest_v2_hex.yaml worlds/forest_hex12.yaml --tilt 45
"""
import argparse
import math

import numpy as np
import yaml


def surface_dist(dx, dy, r):
    ox, oy = abs(dx) - r, abs(dy) - r
    if ox <= 0.0 and oy <= 0.0:
        return max(ox, oy)
    return math.hypot(max(ox, 0.0), max(oy, 0.0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("worlds", nargs="+")
    ap.add_argument("--tilt", type=float, default=45.0)
    ap.add_argument("--max-acc", type=float, default=8.0, help="the planner's max_acc, for the ceiling")
    a = ap.parse_args()
    g = 9.80665
    a_tilt = g * math.tan(math.radians(a.tilt))
    print("lateral accel: tilt %.0f deg -> %.2f m/s^2 | planner max_acc %.1f m/s^2"
          % (a.tilt, a_tilt, a.max_acc))
    for path in a.worlds:
        w = yaml.safe_load(open(path, encoding="utf-8"))
        pil = np.array([[float(q["x"]), float(q["y"]), float(q["r"])] for q in w["pillars"]])
        cs = (w.get("mission") or {}).get("corners") or []
        # one lap only (the world repeats the ring laps times)
        laps = int((w.get("mission") or {}).get("laps", 1) or 1)
        ring = np.array([[float(c[0]), float(c[1])] for c in cs[:len(cs) // max(1, laps)]])
        print()
        print("== %s   (%d pillars, %d corners/lap, laps %d)"
              % (path, len(pil), len(ring), laps))
        print("   %-4s %16s %10s %10s %10s" % ("c", "xy", "R_free", "v@tilt", "v@max_acc"))
        rf = []
        for i, (cx, cy) in enumerate(ring):
            d = min(surface_dist(cx - p[0], cy - p[1], p[2]) for p in pil)
            rf.append(max(d, 1e-6))
            print("   c%-3d (%7.3f,%7.3f) %10.3f %10.2f %10.2f"
                  % (i + 1, cx, cy, d, math.sqrt(max(d, 0) * a_tilt),
                     math.sqrt(max(d, 0) * a.max_acc)))
        rf = np.array(rf)
        if len(ring) > 1:
            sides = [float(np.hypot(*(ring[(i + 1) % len(ring)] - ring[i]))) for i in range(len(ring))]
            pad = [float(np.hypot(*p)) for p in ring]
            print("   side lengths: min %.4f  max %.4f m   | pad distance: min %.4f max %.4f m"
                  % (min(sides), max(sides), min(pad), max(pad)))
        print("   corner free radius: p50 %.3f  min %.3f m  ->  v_corner ceiling %.2f m/s (tilt %.0f)"
              % (np.percentile(rf, 50), rf.min(), math.sqrt(rf.min() * a_tilt), a.tilt))


if __name__ == "__main__":
    main()
