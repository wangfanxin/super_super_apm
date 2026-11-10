#!/usr/bin/env python3
"""dip_context.py -- WHEN does the reference dip mid-leg, and what was happening?

`midleg_stops.py` shows the dips; `reversal_table.py` shows they drive the retrace.  This asks the next
question, because the two obvious levers are different:

  * if a dip happens shortly AFTER a trajectory install (`trajectory_id` changed), it is the SEAM: the new
    plan starts slower than the old one ended, and the fix is on the hand-over (earlier arrival, or the
    goal carrying speed);
  * if it happens mid-trajectory, it is SUPER's own RE-TIMING of the remaining path, and the fix is on the
    planner side (or on how much of the path it is allowed to re-time).

Both clocks here are the bridge's own (`setpoints.csv` and `wire.csv` are written by the same process), so
no clock fit is needed.

    python3 scripts/dip_context.py kv1-02-01 kw1-01-01
"""
import csv
import os
import sys

import numpy as np

ROOT = "/home/fanxin/lio_avoid/logs"
CORNERS = [(10.0, 0.0), (5.0, 8.66), (-5.0, 8.66), (-10.0, 0.0), (-5.0, -8.66), (5.0, -8.66)]
CORNER_R = 1.5


def one(tag):
    d = os.path.join(ROOT, "missionA-" + tag)
    rows = list(csv.DictReader(open(os.path.join(d, "setpoints.csv"))))
    w = list(csv.DictReader(open(os.path.join(d, "wire.csv"))))
    tw = np.array([float(r["t_wall"]) for r in rows])
    tw = tw - tw[0]
    cx = np.array([float(r["cmd_x"]) for r in rows])
    cy = np.array([float(r["cmd_y"]) for r in rows])
    vx = np.array([float(r["vel_x"]) for r in rows])
    vy = np.array([float(r["vel_y"]) for r in rows])
    spd = np.hypot(vx, vy)
    dc = np.min(np.stack([np.hypot(cx - a, cy - b) for a, b in CORNERS]), axis=0)
    ww = np.array([float(r["t_wall"]) for r in w])
    ww = ww - ww[0]
    tid = np.array([int(r["trajectory_id"]) for r in w])
    # install times = where trajectory_id changes
    inst = ww[1:][np.diff(tid) != 0] if len(tid) > 1 else np.array([])
    dip = (spd < 0.25) & (dc > CORNER_R)
    edges = np.nonzero(np.diff(np.concatenate([[0], dip.astype(int), [0]])) == 1)[0]
    ends = np.nonzero(np.diff(np.concatenate([[0], dip.astype(int), [0]])) == -1)[0]
    print("== %s: %d dips (ref<0.25 m/s and >%.1f m from any corner)" % (tag, len(edges), CORNER_R))
    if not len(edges):
        return
    print("   %8s %5s %7s %8s %9s %9s %8s" %
          ("t", "dur", "d_corner", "spd_pre", "spd_post", "since_inst", "dist_at"))
    since, near = [], 0
    for a, b in zip(edges, ends):
        if a < 2 or b + 2 >= len(tw):
            continue
        pre = float(np.median(spd[max(0, a - 10):a]))
        post = float(np.median(spd[b:b + 10])) if b + 10 < len(spd) else float("nan")
        # time since the LAST install at or before this dip (a first-match search here is wrong:
        # `argmax` returns the FIRST True, i.e. the time since the FIRST install of the flight)
        k = np.nonzero(inst <= tw[a])[0]
        dt_inst = float(tw[a] - inst[k[-1]]) if k.size else 999.0
        since.append(dt_inst)
        if dt_inst < 2.0:
            near += 1
        print("   %8.2f %5.2f %7.2f %8.2f %9.2f %9.2f  (%6.2f,%6.2f)"
              % (tw[a], tw[b] - tw[a], dc[a], pre, post, dt_inst, cx[a], cy[a]))
    s = np.array(since)
    print("   -> %d of %d dips start within 2 s of a trajectory install; median gap %.2f s"
          % (near, len(s), float(np.median(s)) if s.size else float("nan")))
    print("      installs: %d ; gaps between installs (s): %s"
          % (len(inst), " ".join("%.1f" % x for x in np.diff(inst))))


for t in sys.argv[1:]:
    try:
        one(t)
    except Exception as exc:                                      # noqa: BLE001
        print("== %s: ERROR %s" % (t, exc))
    print()
