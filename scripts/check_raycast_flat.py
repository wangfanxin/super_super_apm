#!/usr/bin/env python3
"""check_raycast_flat.py -- is the flattened trunk caster the SAME caster, only faster?

The simulator's scan cost is what sets the cloud rate, and the cloud rate is what the LIO lives on
(measured 2026-11-05: 85.7 ms/scan -> 7.0 Hz, 372 skipped scans, and two sorties whose sim clock
froze outright).  `_cast_trunks_flat` replaces 522 per-trunk Python iterations with one vectorized
pass over a flattened (trunk, ray) pair list -- same windows, same two tests.

This script is the honesty check, in three layers:
    A) flat vs the round-80 scalar reference `brute_force()` on the first 400 rays of each pose
    B) flat vs the per-trunk loop `cast(..., prune=False)` on ALL rays
    C) the timing of both

    python3 scripts/check_raycast_flat.py [--world worlds/forest_hex12.yaml] [--points 12000]
"""
import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, "/mnt/d/distill_self/lio_avoid/src/sim")
import raycast                                                   # noqa: E402


def hex_poses():
    """the pad, the six corners at 10.000 m, one mid-leg point, one random point in the field"""
    out = [(0.0, 0.0)]
    for k in range(6):
        a = np.deg2rad(60.0 * k)
        out.append((10.0 * np.cos(a), 10.0 * np.sin(a)))
    out.append((5.0, 4.33))
    out.append((3.1, -7.2))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="/mnt/d/distill_self/super_super_apm/worlds/forest_hex12.yaml")
    ap.add_argument("--points", type=int, default=12000)
    ap.add_argument("--z", type=float, default=2.0)
    ap.add_argument("--verbose", action="store_true")
    a = ap.parse_args()

    world = raycast.World.load(a.world)
    rays, times, src = raycast.load_pattern(raycast.default_pattern_path(), n=a.points)
    print("world %s: %d trunks, z_split %s, walls %s, floor/ceil %s/%s"
          % (os.path.basename(a.world), len(world.trunks), world.z_split, world.walls,
             world.floor_z, world.ceiling_z))
    print("pattern %s: %d rays, %.6f s span" % (src, len(rays), float(times[-1])))

    R = np.eye(3)
    worst_bf = 0.0
    n_bf_bad = 0
    worst_loop = 0.0
    n_loop_bad = 0
    t_old = t_flat = 0.0
    n_cmp = 0
    for (x, y) in hex_poses():
        o = np.array([x, y, a.z], dtype=float)
        t0 = time.perf_counter()
        _, _, bt_old = raycast.cast(o, R, rays, world, return_full=True, prune=False)
        t1 = time.perf_counter()
        _, _, bt_flat = raycast.cast(o, R, rays, world, return_full=True, flat=True)
        t2 = time.perf_counter()
        t_old += t1 - t0
        t_flat += t2 - t1
        d = np.abs(bt_old.astype(float) - bt_flat.astype(float))
        d[~np.isfinite(d)] = 0.0                                  # both miss
        one_sided = np.isfinite(bt_old) != np.isfinite(bt_flat)
        worst_loop = max(worst_loop, float(d.max()))
        n_loop_bad += int((d > 1e-3).sum() + one_sided.sum())
        # A) against the scalar reference, first 400 rays
        bf = raycast.brute_force(o, R, rays, world, limit=400)
        db = np.abs(bf - bt_flat[:400].astype(float))
        db[~np.isfinite(db)] = 0.0
        one_bf = np.isfinite(bf) != np.isfinite(bt_flat[:400])
        worst_bf = max(worst_bf, float(db.max()))
        n_bf_bad += int((db > 1e-3).sum() + one_bf.sum())
        n_cmp += 1
        if a.verbose:
            print("  pose (%6.2f,%6.2f): hits %5d/%d  old %6.1f ms  flat %6.1f ms  |d|max %.2e"
                  % (x, y, int(np.isfinite(bt_flat).sum()), len(rays),
                     1e3 * (t1 - t0), 1e3 * (t2 - t1), float(d.max())))

    print("\n A) flat vs brute_force (400 rays x %d poses): worst |d| = %.3e m, disagreements = %d"
          % (n_cmp, worst_bf, n_bf_bad))
    print(" B) flat vs per-trunk loop (%d rays x %d poses): worst |d| = %.3e m, disagreements = %d"
          % (len(rays), n_cmp, worst_loop, n_loop_bad))
    print(" C) timing per scan: loop %.1f ms, flat %.1f ms  (%.2fx)"
          % (1e3 * t_old / n_cmp, 1e3 * t_flat / n_cmp, t_old / max(t_flat, 1e-9)))
    ok = (n_bf_bad == 0) and (n_loop_bad == 0)
    print("\n%s" % ("PASS -- the flattened caster is the same caster" if ok else
                    "FAIL -- the flattened caster disagrees with the reference"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
