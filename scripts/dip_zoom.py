#!/usr/bin/env python3
"""dip_zoom.py -- what EXACTLY is published during a mid-leg dip?

`dip_context.py` proved the remaining dips are mid-leg (4-5 m from any corner) and not at a
trajectory install.  Before patching anything, this prints the published samples of a dip at full
rate, so the shape decides the fix:

  * position FROZEN and |v| exactly 0        -> nothing is being published; this is a hold / a clamp
                                                at the end of the committed trajectory
  * position still advancing, |v| ramps down  -> the planner re-timed the remaining path
  * |v| jumps between two levels              -> two trajectories are being interleaved

One clock only: `setpoints.csv` (`t_wall`), the reference the bridge actually put on the wire.

    python3 scripts/dip_zoom.py kx1-02-01              # every dip, 0.6 s of context each side
    python3 scripts/dip_zoom.py kx1-02-01 --n 3        # only the first 3
    python3 scripts/dip_zoom.py kx1-02-01 --t 51.9     # one specific dip
"""
import argparse
import csv
import math
import os

import numpy as np

ROOT = "/home/fanxin/lio_avoid/logs"
CORNERS = [(10.0, 0.0), (5.0, 8.66), (-5.0, 8.66), (-10.0, 0.0), (-5.0, -8.66), (5.0, -8.66)]


def load(tag):
    d = os.path.join(ROOT, "missionA-" + tag)
    rows = list(csv.DictReader(open(os.path.join(d, "setpoints.csv"))))
    t = np.array([float(r["t_wall"]) for r in rows])
    cmd = np.array([[float(r["cmd_" + c]) for c in "xyz"] for r in rows])
    v = np.array([[float(r["vel_" + c]) for c in "xyz"] for r in rows])
    spd = np.hypot(v[:, 0], v[:, 1])
    dc = np.min(np.stack([np.hypot(cmd[:, 0] - a, cmd[:, 1] - b) for a, b in CORNERS]), axis=0)
    t = t - t[0]
    return t, cmd, v, spd, dc


def find_dips(t, spd, dc, v_stop=0.25, d_min=1.5, min_dur=0.06, join=0.30):
    """same rule as dip_context.py, but episodes are merged across gaps < join seconds"""
    low = (spd < v_stop) & (dc > d_min)
    out, i = [], 0
    while i < len(low):
        if not low[i]:
            i += 1
            continue
        j = i
        while j + 1 < len(low) and (low[j + 1] or (t[j + 1] - t[j]) < 0.25 and low[min(j + 2, len(low) - 1)]):
            j += 1
        if t[j] - t[i] >= min_dur:
            out.append((i, j))
            i = j + 1
        else:
            i += 1
    # merge episodes closer than `join`
    merged = []
    for a, b in out:
        if merged and t[a] - t[merged[-1][1]] < join:
            merged[-1] = (merged[-1][0], b)
        else:
            merged.append((a, b))
    return merged


def show(t, cmd, v, spd, dc, i0, i1, pad=0.45):
    a = max(0, i0 - int(pad / max(1e-9, np.median(np.diff(t)))))
    b = min(len(t) - 1, i1 + int(pad / max(1e-9, np.median(np.diff(t)))))
    print("   ----- dip t=%.2f .. %.2f s (%.2f s), nearest corner %.2f m"
          % (t[i0], t[i1], t[i1] - t[i0], dc[i0]))
    print("   %7s %8s %8s %6s %6s %7s %s" % ("t", "cmd_x", "cmd_y", "|v|", "d_move", "d_corner", "note"))
    dt = np.median(np.diff(t))
    for k in range(a, b + 1):
        mv = np.hypot(*(cmd[k] - cmd[k - 1])[:2]) if k > 0 else 0.0
        note = ""
        if spd[k] <= 1e-9:
            note = "v==0"
        if k > 0 and mv <= 1e-9:
            note += " cmd-frozen"
        if i0 <= k <= i1:
            note += " <-- in-dip"
        print("   %7.2f %8.3f %8.3f %6.3f %6.3f %7.2f %s"
              % (t[k], cmd[k, 0], cmd[k, 1], spd[k], mv / dt, dc[k], note))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tag")
    ap.add_argument("--n", type=int, default=2)
    ap.add_argument("--t", type=float, default=None)
    ap.add_argument("--v-stop", type=float, default=0.25)
    ap.add_argument("--pad", type=float, default=0.45, help="seconds of context printed each side")
    a = ap.parse_args()
    t, cmd, v, spd, dc = load(a.tag)
    dips = find_dips(t, spd, dc, v_stop=a.v_stop)
    print("== %s: %d mid-leg dips at <%.2f m/s (>=1.5 m from a corner), %.1f s of reference"
          % (a.tag, len(dips), a.v_stop, t[-1]))
    print("   %6s %7s %8s %8s %8s %s" % ("t", "dur", "d_corner", "v_pre", "v_min", "d_move_m/s (in dip)"))
    for i0, i1 in dips:
        pre = spd[max(0, i0 - 10):i0]
        mv = np.hypot(*(cmd[i1] - cmd[i0])[:2]) / max(1e-9, t[i1] - t[i0])
        print("   %6.2f %7.2f %8.2f %8.2f %8.2f %8.2f"
              % (t[i0], t[i1] - t[i0], dc[i0], np.median(pre) if len(pre) else float("nan"),
                 spd[i0:i1 + 1].min(), mv))
    sel = dips
    if a.t is not None:
        sel = [(i0, i1) for i0, i1 in dips if abs(t[i0] - a.t) < 1.0][:1]
    else:
        sel = dips[:a.n]
    for i0, i1 in sel:
        show(t, cmd, v, spd, dc, i0, i1, pad=a.pad)


if __name__ == "__main__":
    main()
