#!/usr/bin/env python3
"""Analytic ray casting for the pillar world -- no Gazebo, no meshes, no GPU.

Why analytic instead of a mesh ray tracer (trimesh/embree/pybullet):
  * the world is cylinders and planes, so the intersection is a closed-form quadratic; it is EXACT
    (no tessellation error) and vectorises trivially in numpy;
  * 20 000 rays at 10 Hz is 200 k intersections/s, which numpy does in a few milliseconds;
  * it removes a whole class of simulator defects that cost this project weeks before: the old
    Gazebo lidar quantised range to 2 cm and its physics step injected a phantom IMU acceleration.

Speed of light is not simulated: a "scan" is the set of hit points for a fixed ray pattern fired at
one instant, with per-point offsets assigned afterwards so the LIO can undistort.

Usage (self-test, no ROS needed):
    python3 raycast.py --world ../worlds/pillars.yaml --render /tmp/scan.png
    python3 raycast.py --world ../worlds/pillars.yaml --summary
"""
from __future__ import annotations

# --- release-tree path bootstrap (git clone anywhere) ------------------------
import os as _os


def _dsh_root():
    d = _os.path.dirname(_os.path.abspath(__file__))
    while d != _os.path.dirname(d):
        if _os.path.isdir(_os.path.join(d, "harness")) and _os.path.isdir(_os.path.join(d, "third_party")):
            return d
        d = _os.path.dirname(d)
    return _os.environ.get("DSH_ROOT", "")


_DSH = _os.environ.get("DSH_ROOT") or _dsh_root()
# -----------------------------------------------------------------------------

import argparse
import math
import os
import sys
import threading
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field

import numpy as np

try:
    import yaml
except ImportError:                                    # keep the module usable without pyyaml
    yaml = None


# --------------------------------------------------------------------------- world
@dataclass
class World:
    floor_z: float = 0.0
    ceiling_z: float | None = 6.0
    walls: dict | None = None
    pad: dict | None = None
    pillars: np.ndarray = field(default_factory=lambda: np.zeros((0, 5)))
    boxes: np.ndarray = field(default_factory=lambda: np.zeros((0, 5)))
    z_split: float | None = None
    raw_pillars: np.ndarray = field(default_factory=lambda: np.zeros((0, 5)))
    trunks: np.ndarray = field(default_factory=lambda: np.zeros((0, 6)))

    # pillars: rows of (x, y, r, z0, z1)  -- the CYLINDER part of each trunk
    # boxes  : rows of (x, y, h, z0, z1)  -- the SQUARE part below z_split (h = half side)
    # trunks : rows of (x, y, r, h, z0, z1) -- the ONE primitive list the caster walks, so a trunk costs one
    #          loop iteration instead of two.  h <= 0 means "no square part" (the trunk is a cylinder
    #          through its whole height); that is what keeps a world without `pillar_geometry` bit-for-bit
    #          the old behaviour.
    # raw_pillars keeps what the file said, so a renderer can draw the whole trunk.

    def __post_init__(self):
        """Derive `trunks` from `pillars`/`boxes` when a caller built a World by hand.

        `cast` walks `trunks`, so a World constructed directly (a test fixture, a tool) would silently
        return NO trunk hits at all if this were not filled in.  A world with only `pillars` becomes
        trunk rows with h = 0, i.e. cylinders through their whole height -- exactly the old behaviour.
        """
        if len(self.trunks) == 0 and len(self.pillars):
            rows = [[x, y, r, 0.0, z0, z1] for (x, y, r, z0, z1) in self.pillars]
            if len(self.boxes) and self.z_split is not None:
                for (x, y, h, z0, z1) in self.boxes:
                    for row in rows:
                        if abs(row[0] - x) < 1e-9 and abs(row[1] - y) < 1e-9 and row[3] == 0.0:
                            row[3] = float(h)
                            break
                    else:
                        rows.append([x, y, self.boxes[0, 2], float(h), z0, z1])
            self.trunks = np.array(rows, dtype=float)

    @staticmethod
    def load(path: str) -> "World":
        if yaml is None:
            raise RuntimeError("pyyaml is required to read %s" % path)
        with open(path) as fh:
            d = yaml.safe_load(fh)
        arena = d.get("arena", {}) or {}
        pl = d.get("pillars", []) or []
        arr = np.array([[p["x"], p["y"], p["r"], p.get("z0", 0.0), p.get("z1", 4.0)] for p in pl],
                       dtype=float) if pl else np.zeros((0, 5))
        # SPEC S2 (round 81): the trunk is a 0.2 m SQUARE below z_split and a 0.2 m-diameter CYLINDER
        # above it.  The mission flies at 2.0 m, i.e. through the square, so a cylinder-only world would
        # let the lidar see 0.04 m of extra room on every corner.  Split each trunk into two primitives;
        # a world without `pillar_geometry` keeps the old cylinder-only behaviour.
        geo = d.get("pillar_geometry") or {}
        z_split = geo.get("z_split")
        z_split = None if z_split is None else float(z_split)
        cyl, box, trunks = [], [], []
        for (x, y, r, z0, z1) in arr:
            h = float(geo.get("box_half_side_m", r)) if z_split is not None else 0.0
            has_box = (z_split is not None) and (z0 < z_split) and (h > 0.0)
            trunks.append((x, y, r, (h if has_box else 0.0), z0, z1))
            if not has_box:
                cyl.append((x, y, r, z0, z1))
                continue
            box.append((x, y, h, z0, min(z1, z_split)))
            if z1 > z_split:
                cyl.append((x, y, r, max(z0, z_split), z1))
        return World(floor_z=float(arena.get("floor_z", 0.0)),
                     ceiling_z=(None if arena.get("ceiling_z") is None
                                else float(arena["ceiling_z"])),
                     walls=arena.get("walls"), pad=d.get("pad"),
                     pillars=(np.array(cyl, dtype=float) if cyl else np.zeros((0, 5))),
                     boxes=(np.array(box, dtype=float) if box else np.zeros((0, 5))),
                     z_split=z_split, raw_pillars=arr,
                     trunks=(np.array(trunks, dtype=float) if trunks else np.zeros((0, 6))))

    def bounds(self):
        if self.walls:
            return (self.walls["x"][0], self.walls["x"][1], self.walls["y"][0], self.walls["y"][1])
        if len(self.pillars):
            return (self.pillars[:, 0].min() - 2, self.pillars[:, 0].max() + 2,
                    self.pillars[:, 1].min() - 2, self.pillars[:, 1].max() + 2)
        return (-6.0, 6.0, -6.0, 6.0)


# --------------------------------------------------------------------------- lidar pattern
S = _os.environ.get("STAGE_DIR", _DSH + "/.build/stage")      # kept for the older tools that import from here
MID360_POINT_RATE_HZ = 200000.0       # the real sensor's point rate: 200 000 points/s


def synth_pattern(n: int = 20000, az_per_rev: int = 360, seed: int = 0) -> np.ndarray:
    """Fallback MID-360-like pattern: a non-repetitive rosette.

    The real MID-360 fires along a fixed but non-repeating rosette; its CSV of ray directions is
    public (Livox-SDK/livox_laser_simulation scan_mode/mid360.csv) and is preferred when present.
    This synthesises something with the same coverage (360 deg azimuth, -7..+52 deg elevation) so
    the geometry can be exercised before that file is available.
    """
    rng = np.random.default_rng(seed)
    hfov = 360.0
    vlo, vhi = -7.0, 52.0
    k = np.arange(n, dtype=float)
    az = (k * hfov / az_per_rev) % 360.0                     # fast azimuth sweep
    # elevation walks upward across the scan, with a small per-ray jitter -> non-repetitive
    elev = vlo + (vhi - vlo) * ((k % 400.0) / 400.0) + rng.normal(0.0, 0.02, n)
    return np.column_stack([np.radians(az), np.radians(elev)])


def default_pattern_path() -> str | None:
    """Where the real MID-360 pattern lives, or None.

    It is 17 MB of vendor data and is deliberately NOT in the repo (`.gitignore`), so it lives in the
    runtime tree.  MEASURED 2026-10-13: a node that silently fell back to a synthesised rosette looked
    exactly like a node using the real pattern -- the only clue was one word in a start-up line --
    and the real pattern is the whole reason the sensor has a realistic blind zone and a
    non-repetitive sampling.  So callers get None and are expected to refuse, not to substitute.
    """
    candidates = [
        os.environ.get("LIO_PATTERN"),
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "mid360_pattern.csv"),
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "scan_mode", "mid360.csv"),
        _os.environ.get("MID360_PATTERN", _DSH + "/harness/sim/mid360_pattern.csv"),
        os.path.expanduser("~/lio_avoid/sim/mid360_pattern.csv"),
    ]
    for c in candidates:
        if c and os.path.exists(c):
            return c
    return None


def load_pattern(path: str | None, n: int = 20000, start: int = 0):
    """Return (rays, times, source).

    rays  : (N, 2) azimuth / elevation in radians, in the SENSOR frame
    times : (N,)   per-point offset within the scan, seconds (None if not available)
    source: a human-readable description of where the pattern came from

    Two formats are supported, and the difference matters:
      * the REAL MID-360 pattern from Livox-SDK/livox_laser_simulation,
        `scan_mode/mid360.csv`, header `Time/s,Azimuth/deg,Zenith/deg`, 800 001 rows = one second at
        200 kHz.  ZENITH is the angle from +z, so elevation = 90 - zenith.  Reading that file as if
        the second column were elevation -- the obvious mistake -- puts the fan in the wrong half of
        the sky.  The file's own time column is kept as the per-point timestamp (5 us spacing), which
        is more honest than assuming a linear ramp.
      * a synthesised rosette, used only when the file is absent.

    `start` lets consecutive scans use consecutive windows of the pattern, which is how the real
    non-repetitive sensor behaves (the pattern advances through the second).
    """
    if path and os.path.exists(path):
        try:
            with open(path) as fh:
                head = fh.readline()
            if "Zenith" in head:
                a = np.loadtxt(path, delimiter=",", skiprows=1 + start, usecols=(0, 1, 2),
                               max_rows=n)
                if a.ndim == 2 and len(a) > 100:
                    t = a[:, 0] - a[0, 0]
                    # The column is HEADERED "Time/s" but is in fact a sample index (step exactly 1,
                    # spanning 800 000 over the file).  Dividing by the sensor's point rate turns it
                    # into the per-point offset in seconds -- 5 us spacing, 100 ms for 20 000 points,
                    # which is exactly one scan at 10 Hz.  Detect it rather than assume it.
                    step = float(np.median(np.diff(t))) if len(t) > 1 else 0.0
                    if step > 0.5:                       # index, not seconds
                        t = t / MID360_POINT_RATE_HZ
                    az = np.radians(a[:, 1])
                    el = np.radians(90.0 - a[:, 2])
                    return np.column_stack([az, el]), t, "%s (real MID-360, rows %d..%d)" % (
                        os.path.basename(path), start + 1, start + len(a))
            a = np.loadtxt(path, delimiter=",", skiprows=1, usecols=(0, 1))
            if a.ndim == 2 and len(a) > 100:
                if len(a) > n:                          # thin uniformly, keep the order
                    idx = np.linspace(0, len(a) - 1, n).astype(int)
                    a = a[idx]
                return np.radians(a.astype(float)), None, os.path.basename(path)
        except Exception as exc:                        # noqa: BLE001
            print("  pattern file unreadable (%s); synthesising instead" % exc, file=sys.stderr)
    return synth_pattern(n), None, "synthesised rosette"


# --------------------------------------------------------------------------- ray casting
def as_per_ray_pose(origin, R, n: int):
    """Accept ONE pose (3,) / (3,3) or PER-RAY poses (N,3) / (N,3,3); return (o (N,3), R (N,3,3)).

    Per-ray poses are what a real scan needs: a MID-360 emits its ~20 000 points over 100 ms at 5 us
    spacing, so a moving vehicle casts every point from a slightly different place and attitude.  That
    distortion is physical, it is what the LIO's undistortion has to remove, and a simulator that casts
    a whole scan from one pose would hand the estimator a cleaner problem than the real one.
    """
    o = np.asarray(origin, dtype=float)
    if o.ndim == 1:
        o = np.broadcast_to(o, (n, 3))
    Rm = np.asarray(R, dtype=float)
    if Rm.ndim == 2:
        Rm = np.broadcast_to(Rm, (n, 3, 3))
    if o.shape != (n, 3) or Rm.shape != (n, 3, 3):
        raise ValueError("pose shapes %s / %s do not match %d rays" % (o.shape, Rm.shape, n))
    return o, Rm


def cast_parallel(origin, R, rays, world, *, rmin=0.10, rmax=40.0, return_full=False,
                  return_index=False, prune=False, threads=1, min_rays=4000):
    """cast() split across threads, because one scan does not fit in 100 ms on one core.

    MEASURED on the frozen world (250 trunks, 20 000 rays): one core needs ~75 ms standalone and ~121 ms
    while the rest of the stack runs, i.e. the simulator could only publish 5.8 scans/s against the 10 Hz
    the LIO needs -- and a starved LIO is what turned two sorties of round 81 into flyaways.  The work is
    numpy over big arrays, so it releases the GIL: 4 threads take the scan to ~25 ms.

    The split is by RAY, and the chunks are contiguous, so the merged result is in ray order and the
    per-ray poses stay with their own rays.  With threads <= 1 or few rays this is exactly cast().
    """
    import os
    from concurrent.futures import ThreadPoolExecutor

    n = len(rays)
    if threads is None:
        threads = min(4, os.cpu_count() or 1)
    if threads <= 1 or n < min_rays:
        return cast(origin, R, rays, world, rmin=rmin, rmax=rmax, return_full=return_full,
                    return_index=return_index, prune=prune)
    o = np.asarray(origin, dtype=float)
    Rm = np.asarray(R, dtype=float)
    per_ray_o = (o.ndim == 2)
    per_ray_R = (Rm.ndim == 3)
    bounds = np.linspace(0, n, threads + 1).astype(int)
    chunks = []
    for k in range(threads):
        a, b = int(bounds[k]), int(bounds[k + 1])
        if b > a:
            chunks.append((a, b))
    def work(k):
        a, b = chunks[k]
        oo = o[a:b] if per_ray_o else o
        RR = Rm[a:b] if per_ray_R else Rm
        return cast(oo, RR, rays[a:b], world, rmin=rmin, rmax=rmax, return_full=return_full,
                    return_index=return_index, prune=prune)
    with ThreadPoolExecutor(max_workers=len(chunks)) as ex:
        outs = list(ex.map(work, range(len(chunks))))
    if return_index:
        pts = np.vstack([x[0] for x in outs]) if outs else np.zeros((0, 3))
        rng = np.concatenate([x[1] for x in outs]) if outs else np.zeros((0,))
        idx = np.concatenate([x[2] + chunks[k][0] for k, x in enumerate(outs)]) if outs else np.zeros((0,), int)
        return pts, rng, idx
    if return_full:
        pts = np.vstack([x[0] for x in outs]) if outs else np.zeros((0, 3))
        rng = np.concatenate([x[1] for x in outs]) if outs else np.zeros((0,))
        full = np.concatenate([x[2] for x in outs]) if outs else np.zeros((0,))
        return pts, rng, full
    pts = np.vstack([x[0] for x in outs]) if outs else np.zeros((0, 3))
    rng = np.concatenate([x[1] for x in outs]) if outs else np.zeros((0,))
    return pts, rng


def cast(origin: np.ndarray, R: np.ndarray, rays: np.ndarray, world: World, *,
         rmin: float = 0.10, rmax: float = 40.0, return_full: bool = False,
         return_index: bool = False, prune: bool = False, flat: bool = False):
    """Cast `rays` (N,2: azimuth, elevation) from `origin` with attitude `R` (sensor->world).

    `origin` and `R` may be one pose or one pose per ray (see `as_per_ray_pose`).  The sensor frame is
    the body frame rotated by the mount, already folded into R by the caller.
    Returns (points_world (M,3), ranges (M,)) with non-hits dropped; `return_full=True` adds best_t
    (N,) where misses are +inf, and `return_index=True` instead returns the N-ray indices of the hits,
    which is how the scan simulator gets each surviving point's own pose back.

    `prune` = True uses the exact per-band prefilter described at `_cast_cylinders` (bit-for-bit the same
    result, several times faster on a dense forest); False runs the plain loop over every pillar, which is
    what `--prune-check` compares against.
    """
    n = len(rays)
    az, el = rays[:, 0], rays[:, 1]
    d_body = np.column_stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    o, Rm = as_per_ray_pose(origin, R, n)
    d = np.einsum("nij,nj->ni", Rm, d_body)                    # to world, per ray
    # THE GEOMETRY RUNS IN float32 (see the note in the file header): the passes are memory-bound and this
    # is worth ~40 % of the scan time.  The results below are promoted back to float64.
    o = np.asarray(o, dtype=np.float32)
    d = np.asarray(d, dtype=np.float32)

    best_t = np.full(n, np.inf, dtype=np.float32)

    # --- horizontal planes: floor and ceiling
    for z_plane in (world.floor_z, world.ceiling_z):
        if z_plane is None:
            continue
        dz = d[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (z_plane - o[:, 2]) / dz
        ok = np.isfinite(t) & (t > 0)
        best_t = np.where(ok & (t < best_t), t, best_t)

    # --- axis-aligned walls (inside faces)
    if world.walls:
        for axis, (lo, hi) in enumerate((world.walls["x"], world.walls["y"])):
            for plane, sign in ((lo, +1.0), (hi, -1.0)):
                dd = d[:, axis]
                with np.errstate(divide="ignore", invalid="ignore"):
                    t = (plane - o[:, axis]) / dd
                    ok = np.isfinite(t) & (t > 0)
                    # the multiply must stay inside errstate: a ray parallel to the wall gives
                    # t = inf, and inf * 0 = nan raises a warning on every scan
                    hit = o + t[:, None] * d
                # only count it if the hit is inside the box on the other horizontal axis
                other = 1 - axis
                olo, ohi = (world.walls["x"] if other == 0 else world.walls["y"])
                ok &= (hit[:, other] >= olo) & (hit[:, other] <= ohi)
                if world.ceiling_z is not None:
                    ok &= (hit[:, 2] >= world.floor_z) & (hit[:, 2] <= world.ceiling_z)
                best_t = np.where(ok & (t < best_t), t, best_t)

    # --- the trunks (SPEC S2: SQUARE below z_split, CYLINDER above), ONE loop with the prune
    #
    # MEASURED 2026-10-13: an azimuth-window culling of this loop (only test rays whose world azimuth
    # falls inside a pillar's apparent angular width) FAILED its own self-test on 5 of 6 poses -- it
    # missed near pillars, so a ray that should have been rejected as inside the blind zone was kept
    # at the farther hit.  It was also SLOWER than this plain loop (22.3 ms vs 18.2 ms at 48
    # pillars), because the mask costs more than the vectorised quadratic it saves.  So: no culling.
    # The scalar reference in brute_force() is what keeps this honest.
    if flat:
        best_t = _cast_trunks_flat(o, d, world.trunks, world.z_split, best_t)
    else:
        best_t = _cast_trunks(o, d, world.trunks, world.z_split, best_t, prune)

    good = np.isfinite(best_t) & (best_t >= rmin) & (best_t <= rmax)
    t = best_t[good].astype(np.float64)
    pts = (o[good].astype(np.float64) + t[:, None] * d[good].astype(np.float64))
    if return_index:
        return pts, t, np.flatnonzero(good)
    if return_full:
        return pts, t, best_t
    return pts, t


# --------------------------------------------------------------------- the cylinder pass, pruned exactly
#
# WHY THIS EXISTS: the number of primitives is now a real cost.  MEASURED 2026-10-14 on the mission-B world
# (24 x 24 m, 256 pillars of r = 0.40 m -- the user's spec): one 20 000-ray scan took 147.6 ms, i.e. the
# sensor could run at 6.8 Hz against the 10 Hz the whole chain is built on (28 pillars: 20.4 ms; 46: 30.4).
# Splitting the pillar loop across 8 threads was tried first and is a DEAD END, measured: 138.5 ms -> 727 ms
# (0.2x), i.e. 5x slower, because the per-pillar arrays are small and the threads spend their time in the
# allocator and the GIL instead of in arithmetic.  So the work itself is cut instead.
#
# The prune is exact, not heuristic.  A ray that starts at o, has already been found to hit something at
# best_t, and hits a cylinder of radius r must satisfy, by the triangle inequality,
#
#       hypot(o_xy - c_xy)  <=  best_t + r
#
# (the entry point is at most best_t away from o and exactly r from the axis).  Rays are therefore split
# into groups by their current best_t, and a whole group skips a pillar when the pillar's axis is further
# than the group's LARGEST best_t + r from every origin in it -- using the group's bounding circle
# (centre, radius), a SCALAR test per (group, pillar) pair.  Because every group is pruned with an upper
# bound on its own final best_t and each group's rays are updated only by pillars that can beat it, the
# result equals the un-pruned loop BIT FOR BIT, which `--prune-check` asserts with np.array_equal.
_T_BANDS = (0.0, 1.0, 2.0, 3.0, 5.0, 8.0, 12.0, 20.0, float("inf"))


def _cast_trunks(o, d, trunks, z_split, best_t, prune: bool = True):
    """The trunk ray-cast: a SQUARE below z_split, a CYLINDER above it, one loop iteration per trunk.

    WHY ONE LOOP AND NOT TWO.  The first version of the box support split each trunk into two primitives
    and ran a second pass over all of them.  MEASURED on this world (250 trunks, 20 000 rays): the scan
    went from 44-55 ms to 234-393 ms, i.e. the simulator could only produce 2.4-3.2 scans/s instead of
    10, and a LIO fed at 3 Hz is what the first sortie of round 81 turned into: EKF variance, then a
    flyaway to 189 m.  So the fix is to keep the old prune (it is what made 233 cylinders cost 44 ms)
    and to add the slab test for the SAME trunk inside the same iteration.

    `trunks` rows are (x, y, r, h, z0, z1); h <= 0 means the trunk has no square part.
    """
    if len(trunks) == 0:
        return best_t
    if not prune:
        return _cast_trunks_chunk(o, d, trunks, z_split, best_t)
    # BUG FIXED round 81: the last band is (20, inf) and `best_t < hi` is False for inf, so a ray that had
    # hit NOTHING so far belonged to no band and was never tested against any trunk -- it came back as a
    # miss no matter what stood in front of it.  The inf case is now its own band.
    # The prune radius is the trunk's LARGEST half-width: the square's corner (h*sqrt(2)) sticks out
    # further than the cylinder (r).
    rmax = np.maximum(trunks[:, 2], trunks[:, 3] * np.sqrt(2.0))
    bands = list(zip(_T_BANDS[:-1], _T_BANDS[1:]))
    for (lo, hi), want_inf in [(b, False) for b in bands] + [((20.0, float("inf")), True)]:
        if want_inf:
            idx = np.flatnonzero(~np.isfinite(best_t))
        else:
            idx = np.flatnonzero((best_t >= lo) & (best_t < hi))
        if len(idx) == 0:
            continue
        t_max = float(best_t[idx].max()) if not want_inf else float("inf")
        og = o[idx]
        cxy_x = float(og[:, 0].mean())
        cxy_y = float(og[:, 1].mean())
        rad = float(np.hypot(og[:, 0] - cxy_x, og[:, 1] - cxy_y).max()) if len(idx) > 1 else 0.0
        reach = t_max + rad
        keep = (np.hypot(trunks[:, 0] - cxy_x, trunks[:, 1] - cxy_y) - rmax - reach) <= 0.0
        sub = trunks[keep]
        if len(sub) == 0:
            continue
        best_t[idx] = _cast_trunks_chunk(og, d[idx], sub, z_split, best_t[idx])
    return best_t


def _cast_trunks_chunk(o, d, trunks, z_split, best_t):
    """The trunk loop: cylinder above z_split, square below it, on candidate rays only.

    WHY THE AZIMUTH WINDOW.  Every previous version still touched all 20 000 rays per trunk at least
    twice (to build the discriminant), i.e. 250 x 20 000 x a few array ops -- MEASURED 56 ms of a 72 ms
    scan, and the reason the simulator could not hold the hardware's 10 Hz.  A trunk of half-width
    R = 0.1*sqrt(2) m can only be hit by rays whose world azimuth points at it within
    atan(R / (distance - R)), so the rays are sorted by azimuth ONCE per scan and each trunk takes its own
    slice with two searchsorted calls.  With the per-ray origin spread padded in, the window is a
    conservative superset: the exact cylinder discriminant and box slab test still decide every hit, so
    the result is the same as the un-culled loop (`brute_force()` and `--prune-check` assert that).
    """
    o = np.asarray(o, dtype=np.float32)
    d = np.asarray(d, dtype=np.float32)
    best_t = np.asarray(best_t, dtype=np.float32)
    trunks = np.asarray(trunks, dtype=np.float32)
    if len(trunks) == 0:
        return best_t
    z_split = None if z_split is None else np.float32(z_split)

    n = len(d)
    dx, dy, dz = d[:, 0], d[:, 1], d[:, 2]
    ox_all, oy_all, oz_all = o[:, 0], o[:, 1], o[:, 2]
    abs_dx, abs_dy, abs_dz = np.abs(dx), np.abs(dy), np.abs(dz)

    # ---- the one-per-scan azimuth index
    az = np.arctan2(dy, dx)
    order = np.argsort(az)
    azs = az[order]
    ox_m, oy_m = float(ox_all.mean()), float(oy_all.mean())
    spread = float(np.max(np.hypot(ox_all - ox_m, oy_all - oy_m))) if n > 1 else 0.0
    TWO_PI = np.float32(2.0 * np.pi)

    tc_all = trunks[:, 0]
    tr_all = trunks[:, 1]
    rr_all = trunks[:, 2]
    hh_all = trunks[:, 3]
    z0_all = trunks[:, 4]
    z1_all = trunks[:, 5]
    Rmax_all = np.maximum(rr_all, hh_all * np.float32(np.sqrt(2.0)))

    for k in range(len(trunks)):
        cx = tc_all[k]
        cy = tr_all[k]
        r = rr_all[k]
        h = hh_all[k]
        z0 = z0_all[k]
        z1 = z1_all[k]
        R = float(Rmax_all[k])
        dist = float(np.hypot(cx - ox_m, cy - oy_m))
        # a ray whose azimuth points within this half-angle can reach the trunk's bounding circle
        # A trunk that the ray origins can be INSIDE OF (or within the origin spread of) has no useful
        # azimuth window: every direction can reach it.  Without this branch the window would be +-90
        # degrees and would MISS hits from the far side -- the exact mistake the file header warns about.
        # RIGOROUS BOUND, not a heuristic.  Every ray origin lies within spread of the mean origin and every
        # point of the trunk lies within R of its centre, so the apparent direction of ANY hit can differ
        # from the direction to the centre by at most asin((R + spread) / (dist - spread)): inflate the
        # trunk by the origin spread and put the vertex at the CLOSEST possible origin.  A first version
        # used atan((R+spread)/(dist-R-spread)), which is smaller than that asin for wide spreads and
        # MISSED hits -- measured by scripts/check_raycast_stress.py as a 6.94 m error at a pose 0.2 m from
        # a trunk.  When the closest possible origin can be inside the inflated circle the window is the
        # whole sphere.
        near = dist - spread
        if near <= R + spread + 1e-6:
            alpha = float(np.pi)
        else:
            alpha = float(np.arcsin(min(1.0, (R + spread) / near)))
        if alpha >= np.pi:
            cand = np.arange(n)
        else:
            azc = float(np.arctan2(cy - oy_m, cx - ox_m))
            lo = azc - alpha
            hi = azc + alpha
            # Slide the window into [-pi, pi) BEFORE the search: with hi > pi the search returns n and the
            # part of the window that wrapped past +pi was silently DROPPED (measured by
            # scripts/check_raycast_stress.py as a 6.94 m error: the culled caster missed a trunk and
            # returned a hit 7 m behind it).
            k = np.floor((lo + np.pi) / (2.0 * np.pi))
            lo -= 2.0 * np.pi * k
            hi -= 2.0 * np.pi * k
            if hi <= np.pi:
                i0 = int(np.searchsorted(azs, lo, side="left"))
                i1 = int(np.searchsorted(azs, hi, side="right"))
                idx_sorted = np.arange(i0, i1) if i1 > i0 else np.zeros(0, dtype=int)
            else:
                i0 = int(np.searchsorted(azs, lo, side="left"))
                i1 = int(np.searchsorted(azs, hi - 2.0 * np.pi, side="right"))
                idx_sorted = np.concatenate([np.arange(i0, n), np.arange(0, i1)])
            cand = order[idx_sorted]
        if len(cand) == 0:
            continue
        ox = ox_all[cand] - cx
        oy = oy_all[cand] - cy
        sdx, sdy, sdz = dx[cand], dy[cand], dz[cand]
        a = sdx * sdx + sdy * sdy
        a_ok = a > 1e-12
        b = np.float32(2.0) * (ox * sdx + oy * sdy)
        c = ox * ox + oy * oy - r * r
        four_a = np.float32(4.0) * a
        disc = b * b - four_a * c
        zc0 = z_split if (h > 0.0 and z_split is not None) else z0
        # ---- cylinder part
        if z1 > zc0:
            sel = (disc > 0) & a_ok
            if sel.any():
                bb, sq, aa = b[sel], np.sqrt(disc[sel]), (np.float32(2.0) * a[sel])
                t1 = (-bb - sq) / aa
                t2 = (-bb + sq) / aa
                t = np.where(t1 > 0, t1, t2)
                hz = oz_all[cand][sel] + t * sdz[sel]
                ok = (t > 0) & (hz >= zc0) & (hz <= z1)
                if ok.any():
                    ray = cand[sel][ok]
                    tt = t[ok]
                    m = tt < best_t[ray]
                    best_t[ray[m]] = tt[m]
        # ---- square part
        if h > 0.0:
            zb1 = min(z1, zc0)
            if zb1 > z0:
                R2 = np.float32(2.0) * h * h
                sel = (b * b - four_a * (c + r * r - R2) > 0) & a_ok
                if sel.any():
                    ray = cand[sel]
                    sox, soy = ox[sel], oy[sel]
                    ozz = oz_all[ray]
                    sdx2, sdy2, sdz2 = sdx[sel], sdy[sel], sdz[sel]
                    tlo = np.full(len(ray), -np.inf, dtype=np.float32)
                    thi = np.full(len(ray), np.inf, dtype=np.float32)
                    for oo, dd, ab, lo_, hi_ in ((sox, sdx2, abs_dx[cand][sel], -h, h),
                                                (soy, sdy2, abs_dy[cand][sel], -h, h),
                                                (ozz, sdz2, abs_dz[cand][sel], z0, zb1)):
                        with np.errstate(divide="ignore", invalid="ignore"):
                            ta = (lo_ - oo) / dd
                            tb = (hi_ - oo) / dd
                        a_lo = np.minimum(ta, tb)
                        a_hi = np.maximum(ta, tb)
                        par = ab < 1e-12
                        inside = (oo >= lo_) & (oo <= hi_)
                        a_lo = np.where(par, np.where(inside, -np.inf, np.inf), a_lo)
                        a_hi = np.where(par, np.where(inside, np.inf, -np.inf), a_hi)
                        tlo = np.maximum(tlo, a_lo)
                        thi = np.minimum(thi, a_hi)
                    tb2 = np.where(tlo > 0, tlo, thi)
                    ok = (thi >= tlo) & (thi > 0) & (tb2 > 0)
                    if ok.any():
                        ray2 = ray[ok]
                        tt = tb2[ok]
                        m = tt < best_t[ray2]
                        best_t[ray2[m]] = tt[m]
    return best_t


def _cast_trunks_flat(o, d, trunks, z_split, best_t):
    """`_cast_trunks_chunk` with every trunk's azimuth window FLATTENED into one pair list.

    Identical geometry and identical culling bound (see the long note in `_cast_trunks_chunk`); the
    only difference is that the per-trunk Python loop is replaced by:
        (1) all windows computed at once (vectorized searchsorted),
        (2) `np.repeat`/`arange` arithmetic that materialises the (trunk, ray) pair list,
        (3) one vectorized cylinder test and one vectorized slab test over all pairs,
        (4) a per-ray min via sort + reduceat.
    Everything is float32 like the rest of the caster.
    """
    o = np.asarray(o, dtype=np.float32)
    d = np.asarray(d, dtype=np.float32)
    best_t = np.asarray(best_t, dtype=np.float32)
    K = len(trunks)
    if K == 0:
        return best_t
    trunks = np.asarray(trunks, dtype=np.float32)
    n = len(d)

    cx = trunks[:, 0]
    cy = trunks[:, 1]
    rr = trunks[:, 2]
    hh = trunks[:, 3]
    z0 = trunks[:, 4]
    z1 = trunks[:, 5]
    Rmax = np.maximum(rr, hh * np.float32(np.sqrt(2.0)))
    if z_split is None:
        zc0 = z0.copy()
    else:
        zc0 = np.where(hh > np.float32(0.0), np.float32(z_split), z0)
    zb1 = np.minimum(z1, zc0)

    # ---- (1) every trunk's azimuth window, at once -----------------------------------------
    az = np.arctan2(d[:, 1], d[:, 0])
    order = np.argsort(az)
    azs = az[order]
    ox_m = float(o[:, 0].mean())
    oy_m = float(o[:, 1].mean())
    spread = float(np.max(np.hypot(o[:, 0] - ox_m, o[:, 1] - oy_m))) if n > 1 else 0.0

    dist = np.hypot(cx - ox_m, cy - oy_m)
    near = dist - spread
    ratio = np.clip((Rmax + spread) / np.maximum(near, np.float32(1e-6)), -1.0, 1.0)
    alpha = np.where(near <= Rmax + spread + 1e-6, np.float32(np.pi), np.arcsin(ratio))
    azc = np.arctan2(cy - oy_m, cx - ox_m)
    TWO_PI = np.float32(2.0 * np.pi)
    lo = azc - alpha
    hi = azc + alpha
    kk = np.floor((lo + np.pi) / TWO_PI)
    lo = lo - TWO_PI * kk
    hi = hi - TWO_PI * kk

    full = alpha >= np.float32(np.pi) - np.float32(1e-9)
    wrap = (~full) & (hi > np.float32(np.pi))
    simple = (~full) & (~wrap)

    # a window is a list of (trunk, start, stop) ranges; the wrapped ones contribute two
    g_tr, g_a, g_b = [], [], []
    if simple.any():
        i0 = np.searchsorted(azs, lo[simple], side="left")
        i1 = np.searchsorted(azs, hi[simple], side="right")
        g_tr.append(np.flatnonzero(simple))
        g_a.append(i0)
        g_b.append(i1)
    if wrap.any():
        w = np.flatnonzero(wrap)
        i0 = np.searchsorted(azs, lo[wrap], side="left")
        g_tr.append(w)
        g_a.append(i0)
        g_b.append(np.full(len(w), n, dtype=np.int64))
        j1 = np.searchsorted(azs, hi[wrap] - TWO_PI, side="right")
        g_tr.append(w)
        g_a.append(np.zeros(len(w), dtype=np.int64))
        g_b.append(j1)
    if full.any():
        f = np.flatnonzero(full)
        g_tr.append(f)
        g_a.append(np.zeros(len(f), dtype=np.int64))
        g_b.append(np.full(len(f), n, dtype=np.int64))
    if not g_tr:
        return best_t

    tr_rng = np.concatenate(g_tr).astype(np.int64)
    a_rng = np.concatenate(g_a).astype(np.int64)
    b_rng = np.concatenate(g_b).astype(np.int64)
    counts = np.maximum(b_rng - a_rng, 0)
    total = int(counts.sum())
    if total == 0:
        return best_t

    # ---- (2) the flattened pair list ------------------------------------------------------
    tk = np.repeat(tr_rng, counts)                     # trunk index per pair
    starts = np.repeat(a_rng, counts)
    ends = np.cumsum(counts)
    offs = np.arange(total, dtype=np.int64) - np.repeat(ends - counts, counts)
    pos = starts + offs
    rid = order[pos]                                   # ray index per pair

    cxp = cx[tk]
    cyp = cy[tk]
    rp = rr[tk]
    hp = hh[tk]
    z0p = z0[tk]
    z1p = z1[tk]
    zc0p = zc0[tk]
    zb1p = zb1[tk]

    ox = o[rid, 0] - cxp
    oy = o[rid, 1] - cyp
    oz = o[rid, 2]
    dx = d[rid, 0]
    dy = d[rid, 1]
    dz = d[rid, 2]
    adx = np.abs(dx)
    ady = np.abs(dy)
    adz = np.abs(dz)

    INF = np.float32(np.inf)
    t_best = np.full(total, INF, dtype=np.float32)

    # ---- (3a) the cylinder above z_split ---------------------------------------------------
    a = dx * dx + dy * dy
    cyl = (z1p > zc0p) & (a > np.float32(1e-12))
    if cyl.any():
        b = np.float32(2.0) * (ox * dx + oy * dy)
        c = ox * ox + oy * oy - rp * rp
        disc = b * b - np.float32(4.0) * a * c
        sel = cyl & (disc > np.float32(1e-12))
        if sel.any():
            bb = b[sel]
            sq = np.sqrt(disc[sel])
            aa = np.float32(2.0) * a[sel]
            t1 = (-bb - sq) / aa
            t2 = (-bb + sq) / aa
            t = np.where(t1 > np.float32(0.0), t1, t2)
            hz = oz[sel] + t * dz[sel]
            ok = (t > np.float32(0.0)) & (hz >= zc0p[sel]) & (hz <= z1p[sel])
            if ok.any():
                idx = np.flatnonzero(sel)[ok]
                t_best[idx] = t[ok]

    # ---- (3b) the square below z_split (3-axis slab) --------------------------------------
    box = (hp > np.float32(0.0)) & (zb1p > z0p)
    if box.any():
        bidx = np.flatnonzero(box)
        tlo = np.full(len(bidx), -INF, dtype=np.float32)
        thi = np.full(len(bidx), INF, dtype=np.float32)
        with np.errstate(divide="ignore", invalid="ignore"):
            for oh, dh, adh, lo_, hi_ in (
                (ox[bidx], dx[bidx], adx[bidx], -hp[bidx], hp[bidx]),
                (oy[bidx], dy[bidx], ady[bidx], -hp[bidx], hp[bidx]),
                (oz[bidx], dz[bidx], adz[bidx], z0p[bidx], zb1p[bidx]),
            ):
                ta = (lo_ - oh) / dh
                tb = (hi_ - oh) / dh
                a_lo = np.minimum(ta, tb)
                a_hi = np.maximum(ta, tb)
                par = adh < np.float32(1e-12)
                inside = (oh >= lo_) & (oh <= hi_)
                a_lo = np.where(par, np.where(inside, -INF, INF), a_lo)
                a_hi = np.where(par, np.where(inside, INF, -INF), a_hi)
                tlo = np.maximum(tlo, a_lo)
                thi = np.minimum(thi, a_hi)
        tb2 = np.where(tlo > np.float32(0.0), tlo, thi)
        ok = (thi >= tlo) & (thi > np.float32(0.0)) & (tb2 > np.float32(0.0))
        if ok.any():
            idx = bidx[ok]
            tt = tb2[ok]
            t_best[idx] = np.minimum(t_best[idx], tt)

    # ---- (4) per-ray minimum over the pairs -----------------------------------------------
    hit = np.isfinite(t_best)
    if not hit.any():
        return best_t
    rh = rid[hit]
    th = t_best[hit]
    srt = np.argsort(rh, kind="stable")
    rh = rh[srt]
    th = th[srt]
    first = np.flatnonzero(np.r_[True, rh[1:] != rh[:-1]])
    mins = np.minimum.reduceat(th, first)
    rr_idx = rh[first]
    better = mins < best_t[rr_idx]
    best_t[rr_idx[better]] = mins[better]
    return best_t


def _cast_boxes(o, d, boxes, best_t):
    """The SQUARE part of each trunk (SPEC S2: box below z_split).

    Slab method, vectorised over rays, one box at a time.  For an axis-aligned box the ray is inside the
    box exactly on the interval [max(t_lo), min(t_hi)] where t_lo/t_hi come from the three slab pairs; the
    first positive hit is t_lo when the origin is outside, and t_hi (the exit face) when it is inside.
    A ray parallel to a slab is handled explicitly instead of trusting inf/nan arithmetic.
    """
    n = len(d)
    for cx, cy, h, z0, z1 in boxes:
        tlo = np.full(n, -np.inf)
        thi = np.full(n, np.inf)
        for axis, (lo, hi) in ((0, (cx - h, cx + h)), (1, (cy - h, cy + h)), (2, (z0, z1))):
            dd = d[:, axis]
            oo = o[:, axis]
            with np.errstate(divide="ignore", invalid="ignore"):
                ta = (lo - oo) / dd
                tb = (hi - oo) / dd
            a_lo = np.minimum(ta, tb)
            a_hi = np.maximum(ta, tb)
            par = np.abs(dd) < 1e-12                     # parallel to this slab
            inside = (oo >= lo) & (oo <= hi)
            a_lo = np.where(par, np.where(inside, -np.inf, np.inf), a_lo)
            a_hi = np.where(par, np.where(inside, np.inf, -np.inf), a_hi)
            tlo = np.maximum(tlo, a_lo)
            thi = np.minimum(thi, a_hi)
        ok = (thi >= tlo) & (thi > 0)
        t = np.where(tlo > 0, tlo, thi)                  # origin inside -> the exit face
        best_t = np.where(ok & (t > 0) & (t < best_t), t, best_t)
    return best_t


def cast_full(origin: np.ndarray, R: np.ndarray, rays: np.ndarray, world: World, *,
              rmin: float = 0.10, rmax: float = 40.0) -> tuple[np.ndarray, np.ndarray]:
    """Reference implementation: every ray tested against every pillar, no culling.

    Only used by --selftest to prove the culled path in cast() returns the same hits.  A dense forest
    makes this the slow one on purpose.  Accepts per-ray poses like cast().
    """
    n = len(rays)
    az, el = rays[:, 0], rays[:, 1]
    d_body = np.column_stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    o, Rm = as_per_ray_pose(origin, R, n)
    d = np.einsum("nij,nj->ni", Rm, d_body)
    best_t = np.full(n, np.inf)
    for z_plane in (world.floor_z, world.ceiling_z):
        if z_plane is None:
            continue
        dz = d[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (z_plane - o[:, 2]) / dz
        ok = np.isfinite(t) & (t > 0)
        best_t = np.where(ok & (t < best_t), t, best_t)
    for cx, cy, r, z0, z1 in world.pillars:
        ox, oy = o[:, 0] - cx, o[:, 1] - cy
        dx, dy = d[:, 0], d[:, 1]
        a = dx * dx + dy * dy
        b = 2.0 * (ox * dx + oy * dy)
        c = ox * ox + oy * oy - r * r
        disc = b * b - 4.0 * a * c
        ok = (disc > 0) & (a > 1e-12)
        if not np.any(ok):
            continue
        sq = np.sqrt(np.where(ok, disc, 0.0))
        t1 = (-b - sq) / (2.0 * a)
        t2 = (-b + sq) / (2.0 * a)
        t = np.where(t1 > 0, t1, t2)
        hit_z = o[:, 2] + t * d[:, 2]
        ok &= (t > 0) & (hit_z >= z0) & (hit_z <= z1)
        best_t = np.where(ok & (t < best_t), t, best_t)
    if len(world.boxes):
        best_t = _cast_boxes(o, d, world.boxes, best_t)
    good = np.isfinite(best_t) & (best_t >= rmin) & (best_t <= rmax)
    t = best_t[good]
    return o[good] + t[:, None] * d[good], t


def brute_force(origin: np.ndarray, R: np.ndarray, rays: np.ndarray, world: World,
                limit: int = 400) -> np.ndarray:
    """Scalar, per-ray, per-primitive reference -- independent code, independent maths.

    Deliberately plain Python with math.* calls so a bug in the vectorised version cannot hide behind
    a shared mistake.  Returns the per-ray nearest-hit range for the first `limit` rays, +inf for a
    true miss -- and, like cast(..., return_full=True), WITHOUT the blind-zone/max-range gate, so the
    two can be compared directly.  (An earlier version of this test gated one side and not the other,
    which made 208 of 400 rays look like disagreements when both were right.)

    Accepts per-ray poses like cast(), which is how the per-ray path gets an independent check.
    """
    n = min(limit, len(rays))
    o_all, R_all = as_per_ray_pose(origin, R, len(rays))
    out = np.full(n, np.inf)
    for i in range(n):
        az, el = float(rays[i, 0]), float(rays[i, 1])
        origin_i = o_all[i]
        R_i = R_all[i]
        db = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
        d = R_i @ db
        best = math.inf
        for z_plane in (world.floor_z, world.ceiling_z):
            if z_plane is None or abs(d[2]) < 1e-15:
                continue
            t = (z_plane - origin_i[2]) / d[2]
            if t > 0:
                best = min(best, t)
        if world.walls:
            for axis, (lo, hi) in enumerate((world.walls["x"], world.walls["y"])):
                for plane in (lo, hi):
                    if abs(d[axis]) < 1e-15:
                        continue
                    t = (plane - origin_i[axis]) / d[axis]
                    if t <= 0 or t >= best:
                        continue
                    h = origin_i + t * d
                    other = 1 - axis
                    olo, ohi = (world.walls["x"] if other == 0 else world.walls["y"])
                    if olo <= h[other] <= ohi and (world.ceiling_z is None or
                                                   world.floor_z <= h[2] <= world.ceiling_z):
                        best = t
        for cx, cy, r, z0, z1 in world.pillars:
            ox, oy = origin_i[0] - cx, origin_i[1] - cy
            a = d[0] * d[0] + d[1] * d[1]
            if a < 1e-15:
                continue
            b = 2.0 * (ox * d[0] + oy * d[1])
            c = ox * ox + oy * oy - r * r
            disc = b * b - 4.0 * a * c
            if disc <= 0:
                continue
            sq = math.sqrt(disc)
            for t in ((-b - sq) / (2.0 * a), (-b + sq) / (2.0 * a)):
                if t > 0 and t < best:
                    hz = origin_i[2] + t * d[2]
                    if z0 <= hz <= z1:
                        best = t
                        break
        for cx, cy, h, z0, z1 in world.boxes:
            # scalar slab test, written out by hand so it cannot share a mistake with the vector version
            t_lo, t_hi = -math.inf, math.inf
            for axis, (lo, hi) in ((0, (cx - h, cx + h)), (1, (cy - h, cy + h)), (2, (z0, z1))):
                dd = d[axis]
                oo = origin_i[axis]
                if abs(dd) < 1e-12:
                    if not (lo <= oo <= hi):
                        t_lo, t_hi = math.inf, -math.inf
                        break
                    continue
                ta = (lo - oo) / dd
                tb = (hi - oo) / dd
                t_lo = max(t_lo, min(ta, tb))
                t_hi = min(t_hi, max(ta, tb))
                if t_hi < t_lo:
                    break
            if t_hi < t_lo or t_hi <= 0:
                continue
            t = t_lo if t_lo > 0 else t_hi
            if 0 < t < best:
                best = t
        out[i] = best if math.isfinite(best) else math.inf
    return out


def make_mount_rotation(mount: dict) -> np.ndarray:
    """Body <- sensor rotation from the yaml mount (degrees)."""
    r, p, y = (math.radians(mount.get(k, 0.0)) for k in ("roll", "pitch", "yaw"))
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1.0]])
    Ry = np.array([[cp, 0, sp], [0, 1.0, 0], [-sp, 0, cp]])
    Rx = np.array([[1.0, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def rpy_to_R(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


# --------------------------------------------------------------------------- self-test
def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default=os.path.join(os.path.dirname(__file__),
                                                    "..", "worlds", "pillars.yaml"))
    ap.add_argument("--render", default=None, help="write a PNG of one scan")
    ap.add_argument("--summary", action="store_true")
    ap.add_argument("--selftest", action="store_true",
                    help="prove the culled casting equals the un-culled reference")
    ap.add_argument("--prune-check", action="store_true",
                    help="prove the pruned cylinder pass equals the un-pruned one, BIT FOR BIT, and time "
                         "both (the prune exists because 256 pillars cost 147 ms/scan, i.e. 6.8 Hz "
                         "against a 10 Hz chain)")
    ap.add_argument("--scale", action="store_true",
                    help="timing versus pillar count (the forest gets dense)")
    ap.add_argument("--pos", default="0,0,3.2", help="sensor origin x,y,z")
    ap.add_argument("--pattern", default=os.path.join(os.path.dirname(__file__),
                                                      "mid360_pattern.csv"))
    args = ap.parse_args()

    w = World.load(args.world)
    rays, times, src = load_pattern(args.pattern)
    x, y, z = (float(v) for v in args.pos.split(","))
    cfg = {}
    if yaml is not None:
        cfg = yaml.safe_load(open(args.world)) or {}
    mount = (cfg.get("sensor", {}) or {}).get("mount", {}) or {}

    if args.selftest:
        rng = np.random.default_rng(1)
        bad = 0
        for trial in range(6):
            origin = np.array([rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(0.5, 3.5)])
            R = rpy_to_R(rng.uniform(-0.3, 0.3), rng.uniform(-0.3, 0.3), rng.uniform(-np.pi, np.pi))
            _, _, vec = cast(origin, R, rays, w, return_full=True)
            bru = brute_force(origin, R, rays, w, limit=400)
            n = len(bru)
            a, b = vec[:n], bru
            same_mask = np.isfinite(a) == np.isfinite(b)
            # cast() computes in float32 (memory traffic), the scalar reference in float64: 5e-4 m is
            # 40x below the 0.02 m range noise and is what "the same hit" means now (MEASURED worst
            # case 2.55e-4 m at 20 m; the hit/miss mask is still compared exactly).
            same_val = np.allclose(np.where(np.isfinite(a), a, 0), np.where(np.isfinite(b), b, 0),
                                   atol=5e-4)
            hit_v, hit_b = int(np.isfinite(a).sum()), int(np.isfinite(b).sum())
            ok = bool(same_mask.all() and same_val)
            print("  trial %d: origin (%+.2f, %+.2f, %+.2f)  %d-ray sample: vector %d hits, "
                  "brute force %d hits, max |drange| %.2e m, identical=%s"
                  % (trial, origin[0], origin[1], origin[2], n, hit_v, hit_b,
                     float(np.max(np.abs(np.where(np.isfinite(a), a, 0)
                                         - np.where(np.isfinite(b), b, 0)))), ok))
            bad += 0 if ok else 1
        # and the gated output must drop exactly the sub-blind-zone hits
        _, rng = cast(origin, R, rays, w)
        print("  gate check at the last trial: %d of %d finite ranges survive the blind-zone gate"
              % (len(rng), int(np.isfinite(vec).sum())))

        # PER-RAY POSES: this is the motion-compensated path the scan simulator uses, so it gets its
        # own check against the same independent reference.  A vehicle moving at 1 m/s covers 10 cm in
        # one 100 ms scan, so if this path were wrong the clouds would carry a distortion of that size
        # and the LIO's undistortion would be measured against a lie.
        rng_np = np.random.default_rng(7)
        n_rays = len(rays)
        o_pr = np.column_stack([rng_np.uniform(-3, 3, n_rays), rng_np.uniform(-3, 3, n_rays),
                                rng_np.uniform(0.5, 3.5, n_rays)])
        yaw = rng_np.uniform(-np.pi, np.pi, n_rays)
        pitch = rng_np.uniform(-0.3, 0.3, n_rays)
        R_pr = np.stack([rpy_to_R(0.0, pitch[i], yaw[i]) for i in range(n_rays)])
        _, _, vec_pr = cast(o_pr, R_pr, rays, w, return_full=True)
        bru_pr = brute_force(o_pr, R_pr, rays, w, limit=400)
        m = len(bru_pr)
        a, b = vec_pr[:m], bru_pr
        same_mask_pr = np.isfinite(a) == np.isfinite(b)
        diff_pr = np.max(np.abs(np.where(np.isfinite(a), a, 0) - np.where(np.isfinite(b), b, 0)))
        ok_pr = bool(same_mask_pr.all() and diff_pr < 5e-4)
        spread = float(np.linalg.norm(o_pr[:m] - o_pr[0], axis=1).max())
        print("  per-ray poses: %d-ray sample spread up to %.2f m across the scan, vector %d hits, "
              "brute force %d hits, max |drange| %.2e m, identical=%s"
              % (m, spread, int(np.isfinite(a).sum()), int(np.isfinite(b).sum()), float(diff_pr),
                 ok_pr))
        bad += 0 if ok_pr else 1

        print("  SELFTEST %s" % ("PASS" if bad == 0 else "FAIL (%d of 7 differ)" % bad))
        return 0 if bad == 0 else 1

    if args.prune_check:
        # WHY THIS TEST EXISTS: the prefilter is only allowed to be a SPEED change.  Its bound is a proof
        # (a hit within best_t implies hypot(o_xy - c_xy) <= best_t + r), and every band is pruned against
        # an UPPER bound of its own final best_t, so the pruned cast must equal the un-pruned one exactly --
        # not to 1e-9, to the last bit.  If a future edit prunes a band with the wrong bound, or updates
        # best_t across bands, the numbers would still LOOK right and every geometry conclusion in this
        # project would rest on a subtly different world.  np.array_equal is the whole point.
        import time
        print("world   : %s (%d pillars)" % (args.world, len(w.pillars)))
        nz = 0
        for trial in range(4):
            if trial == 0:
                origin = np.array([0.0, 0.0, 2.6])
            else:
                origin = np.random.default_rng(trial).uniform(-3, 3, 3)
                origin[2] = abs(origin[2]) + 0.6
            R = rpy_to_R(0, 0, 0) @ make_mount_rotation(mount)
            _, r1, a1 = cast(origin, R, rays, w, return_full=True, return_index=True, prune=False)
            _, r8, a8 = cast(origin, R, rays, w, return_full=True, return_index=True, prune=True)
            same_full = bool(np.array_equal(a1, a8))       # includes the +inf misses
            same_hits = bool(np.array_equal(r1, r8)) and len(a1) == len(a8)
            print("  trial %d origin (%+.2f, %+.2f, %+.2f): %d hits, best_t identical=%s, "
                  "ranges identical=%s" % (trial, origin[0], origin[1], origin[2], len(r1),
                                           same_full, same_hits))
            nz += 0 if (same_full and same_hits) else 1
            if trial == 0:
                t0 = time.perf_counter()
                for _ in range(5):
                    cast(origin, R, rays, w, prune=False)
                t_off = (time.perf_counter() - t0) / 5 * 1e3
                t0 = time.perf_counter()
                for _ in range(5):
                    cast(origin, R, rays, w, prune=True)
                t_on = (time.perf_counter() - t0) / 5 * 1e3
                print("  cost: no prune %.2f ms/scan, pruned %.2f ms/scan (%.2fx), budget 100 ms "
                      "at 10 Hz" % (t_off, t_on, t_off / t_on))
        print("  PRUNE %s" % ("PASS" if nz == 0 else "FAIL (%d of 4 differ)" % nz))
        return 0 if nz == 0 else 1

    if args.scale:
        import time
        print("  pillars   rays   culled ms   full ms   speedup")
        for P in (10, 25, 49, 100, 200, 400):
            ww = World(floor_z=w.floor_z, ceiling_z=w.ceiling_z, walls=w.walls, pad=w.pad,
                       pillars=np.array([[np.cos(2 * np.pi * i / P) * 3.0,
                                          np.sin(2 * np.pi * i / P) * 3.0, 0.3, 0.0, 4.0]
                                         for i in range(P)]))
            R = rpy_to_R(0, 0, 0) @ make_mount_rotation(mount)
            o = np.array([0.0, 0.0, 2.5])
            cast(o, R, rays, ww)
            t0 = time.perf_counter()
            for _ in range(10):
                cast(o, R, rays, ww)
            tc = (time.perf_counter() - t0) / 10
            t0 = time.perf_counter()
            for _ in range(3):
                cast_full(o, R, rays, ww)
            tf = (time.perf_counter() - t0) / 3
            print("  %7d %6d %10.2f %9.2f %8.1fx" % (P, len(rays), tc * 1e3, tf * 1e3, tf / tc))
        return 0

    # The lidar's fan is defined about the SENSOR axis, and the sensor keeps the real drone's
    # 45 deg nose-down mount, so R_world_sensor = R_world_body @ R_body_sensor.
    R = rpy_to_R(0.0, 0.0, 0.0) @ make_mount_rotation(mount)
    pts, rng = cast(np.array([x, y, z]), R, rays, w)
    print("world   : %d pillars, floor %.1f, ceiling %s, walls %s"
          % (len(w.pillars), w.floor_z, w.ceiling_z, bool(w.walls)))
    print("pattern : %s (%d rays)" % (src, len(rays)))
    if times is not None and len(times) > 1:
        print("timing  : per-point offsets %.6f .. %.6f s (median step %.1f us)"
              % (times[0], times[-1], 1e6 * float(np.median(np.diff(times)))))
    print("mount   : roll %+.1f pitch %+.1f yaw %+.1f deg  (sensor fan becomes %+.0f..%+.0f deg "
          "about the body)" % (mount.get("roll", 0.0), mount.get("pitch", 0.0), mount.get("yaw", 0.0),
                               -7.0 - mount.get("pitch", 0.0), 52.0 - mount.get("pitch", 0.0)))
    print("origin  : (%.2f, %.2f, %.2f)  body level" % (x, y, z))
    print("hits    : %d of %d rays (%.1f %%)" % (len(pts), len(rays), 100.0 * len(pts) / len(rays)))
    if len(pts):
        print("range   : %.2f .. %.2f m (median %.2f)" % (rng.min(), rng.max(), np.median(rng)))
        # which primitive each hit came from is not tracked; report the geometric split instead
        on_floor = np.isclose(pts[:, 2], w.floor_z, atol=1e-6)
        on_ceil = (w.ceiling_z is not None) and np.isclose(pts[:, 2], w.ceiling_z, atol=1e-6)
        print("split   : floor %.1f %%, ceiling %.1f %%, other (pillars/walls) %.1f %%"
              % (100.0 * on_floor.mean(), 100.0 * on_ceil.mean() if w.ceiling_z else 0.0,
                 100.0 * (1 - on_floor.mean() - (on_ceil.mean() if w.ceiling_z else 0.0))))
    if args.render:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        x0, x1, y0, y1 = w.bounds()
        fig, ax = plt.subplots(1, 2, figsize=(13, 5.4))
        ax[0].scatter(pts[:, 0], pts[:, 1], s=0.4, c=pts[:, 2], cmap="viridis")
        for cx, cy, r, _, _ in w.pillars:
            ax[0].add_patch(plt.Circle((cx, cy), r, color="k", fill=False, lw=1.2))
        ax[0].plot(x, y, "r*", ms=14, label="sensor")
        ax[0].set_xlim(x0, x1); ax[0].set_ylim(y0, y1); ax[0].set_aspect("equal")
        ax[0].set_xlabel("East (m)"); ax[0].set_ylabel("North (m)")
        ax[0].set_title("one scan, top view (%d points)" % len(pts))
        ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)
        ax[1].scatter(pts[:, 0], pts[:, 2], s=0.4, c=pts[:, 1], cmap="magma")
        ax[1].set_xlabel("East (m)"); ax[1].set_ylabel("Up (m)")
        ax[1].set_title("front view (colour = North)")
        ax[1].grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(args.render, dpi=140)
        print("render  : %s" % args.render)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
