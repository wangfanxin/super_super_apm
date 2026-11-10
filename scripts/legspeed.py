#!/usr/bin/env python3
"""legspeed.py -- what does the REFERENCE speed do inside one leg?

The retrace complaint ("在航点中间有往复") is a speed story: the reference sags mid-leg, the FCU
chases the sag, then the reference speeds up again and the aircraft runs past it.  `dip_context.py`
tells us the dips are NOT at trajectory installs (median 5.8 s after one), so the sag is inside the
single trajectory that a leg was planned as.  This script shows that profile directly, one leg at a
time, as v(s) along the reference's own path.

    python3 scripts/legspeed.py kwC-01-01            # summary: every leg
    python3 scripts/legspeed.py kwC-01-01 3 5        # ASCII profile for legs 3..5

Columns of the ASCII plot: s (distance along the leg, m), v (reference |v_xy|, m/s), and a marker
where the speed is BELOW 60 % of the leg's own p90 (i.e. a sag, not a normal start/stop ramp).
"""
import csv
import os
import sys

import numpy as np

ROOT = "/home/fanxin/lio_avoid/logs"


def load(tag):
    d = os.path.join(ROOT, "missionA-" + tag)
    sp = list(csv.DictReader(open(os.path.join(d, "setpoints.csv"))))
    wr = list(csv.DictReader(open(os.path.join(d, "wire.csv"))))
    tw = np.array([float(r["t_wall"]) for r in sp])
    cmd = np.array([[float(r["cmd_x"]), float(r["cmd_y"]), float(r["cmd_z"])] for r in sp])
    vel = np.array([[float(r["vel_x"]), float(r["vel_y"]), float(r["vel_z"])] for r in sp])
    ww = np.array([float(r["t_wall"]) for r in wr])
    tid = np.array([int(r["trajectory_id"]) for r in wr])
    # trajectory_id as a function of the setpoint clock (sample-and-hold)
    idx = np.clip(np.searchsorted(ww, tw) - 1, 0, len(tid) - 1)
    return tw, cmd, vel, tid[idx], ww, tid


def legs(tw, tid):
    """[(id, i0, i1)] -- contiguous runs of the same trajectory_id, >=1 s long."""
    out, i0 = [], 0
    for i in range(1, len(tid)):
        if tid[i] != tid[i - 1]:
            if tw[i - 1] - tw[i0] >= 1.0:
                out.append((int(tid[i - 1]), i0, i - 1))
            i0 = i
    if tw[-1] - tw[i0] >= 1.0:
        out.append((int(tid[-1]), i0, len(tid) - 1))
    return out


def one(tag, verbose=()):
    tw, cmd, vel, tid, ww, tid_raw = load(tag)
    L = legs(tw, tid)
    print("== %s: %d legs, %.1f s of reference" % (tag, len(L), tw[-1] - tw[0]))
    print("   leg   id   dur   path    v p50  v p90  v max   sag%%  n_dip  dip_dur  s_at_dip(%)")
    prof = {}
    for k, (i, i0, i1) in enumerate(L):
        t = tw[i0:i1 + 1] - tw[i0]
        p = cmd[i0:i1 + 1]
        v = np.hypot(vel[i0:i1 + 1, 0], vel[i0:i1 + 1, 1])
        ds = np.r_[0.0, np.linalg.norm(np.diff(p[:, :2], axis=0), axis=1)]
        s = np.cumsum(ds)
        if s[-1] < 3.0:
            continue
        p90 = float(np.percentile(v, 90))
        floor = 0.6 * p90
        # a sag: contiguous below-floor window that is NOT at the ends (first/last 8 % of the leg)
        inner = (s > 0.08 * s[-1]) & (s < 0.92 * s[-1])
        low = (v < floor) & inner & (v < 1.2)
        # group into windows
        dips, j = [], 0
        while j < len(low):
            if low[j]:
                j2 = j
                while j2 + 1 < len(low) and low[j2 + 1]:
                    j2 += 1
                if t[j2] - t[j] >= 0.06:
                    dips.append((s[j], s[j2], float(v[j:j2 + 1].min()), t[j], t[j2] - t[j]))
                j = j2 + 1
            else:
                j += 1
        sag = 100.0 * float(low.mean())
        dip_dur = float(np.median([d[4] for d in dips])) if dips else 0.0
        s_at = 100.0 * float(np.median([d[0] / s[-1] for d in dips])) if dips else 0.0
        print("   %3d  %3d  %5.1f  %6.1f   %5.2f  %5.2f  %5.2f   %5.1f   %4d   %5.2f    %5.1f"
              % (k + 1, i, t[-1], s[-1], np.median(v), p90, v.max(), sag, len(dips), dip_dur, s_at))
        prof[k + 1] = (t, s, v, p90)
    for k in verbose:
        if k not in prof:
            continue
        t, s, v, p90 = prof[k]
        print("\n   ---- leg %d: v(s)  (bar length = v / %.2f m/s)" % (k, p90))
        step = max(1, len(s) // 60)
        for j in range(0, len(s), step):
            bar = "#" * int(round(40.0 * v[j] / max(p90, 1e-6)))
            print("   s=%5.2f t=%5.2f v=%4.2f |%-40s|" % (s[j], t[j], v[j], bar))


if __name__ == "__main__":
    tag = sys.argv[1]
    verbose = [int(x) for x in sys.argv[2:]]
    one(tag, verbose)
