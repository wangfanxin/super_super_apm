#!/usr/bin/env python3
"""prep_run.py -- pull the few columns the figures need out of a sortie's logs, at a plot-friendly rate.

WHY: the recordings live on the WSL filesystem (20-40 MB per sortie: tap.csv is 1.2 kHz x 190 s), and the
figures are drawn with a CJK font that only the Windows-side matplotlib has.  Copying whole recordings
around is slow and wasteful, so this writes three compact files a figure script can read anywhere:

    logs/<tag>/cmd.csv   t_sim_us, cmd_x, cmd_y, cmd_z, vel_x, vel_y, vel_z        (100 Hz, unchanged)
    logs/<tag>/tap.csv   t_us, pe, pn, pd, ve, vn                                  (downsampled to --rate)
    logs/<tag>/lio.csv   t_us, x, y, z                                             (downsampled, L frame)

`cmd_*` is in W (the bridge's log; velocity columns are W too since the round-82 fix -- for OLDER runs pass
--cmd-vel-frame l and the figure scripts will rotate them).  `lio_*` is the estimator pose in L on purpose.

    python3 scripts/prep_run.py --run /home/fanxin/lio_avoid/logs/missionA-d84-01-01 --out logs/d84-01-01
"""
import argparse
import csv
import os

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--run", required=True)
ap.add_argument("--out", required=True, help="an output directory, e.g. logs/<tag>")
ap.add_argument("--rate", type=float, default=100.0, help="the tap and the LIO are decimated to this rate")
a = ap.parse_args()

os.makedirs(a.out, exist_ok=True)

# ---- the command stream (100 Hz, W)
sp = list(csv.DictReader(open(os.path.join(a.run, "setpoints.csv"), encoding="utf-8")))
cols = ["t_wall", "t_sim_us", "cmd_x", "cmd_y", "cmd_z", "vel_x", "vel_y", "vel_z"]
have = sp[0].keys()
cols = [c for c in cols if c in have]
with open(os.path.join(a.out, "cmd.csv"), "w", encoding="utf-8", newline="") as f:
    w = csv.writer(f)
    w.writerow(cols)
    for r in sp:
        w.writerow([r[c] for c in cols])


def decimate(src, dst, cols, tcol, scale=1e-6):
    rows = list(csv.DictReader(open(src, encoding="utf-8")))
    t = np.array([float(r[tcol]) * scale for r in rows])
    keep = np.zeros(len(rows), dtype=bool)
    t_next = t[0]
    for k, tk in enumerate(t):
        if tk >= t_next:
            keep[k] = True
            t_next = tk + 1.0 / a.rate
    with open(dst, "w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for k in np.flatnonzero(keep):
            r = rows[k]
            w.writerow([r[c] for c in cols])
    return int(keep.sum()), len(rows)


n_tap = decimate(os.path.join(a.run, "tap.csv"), os.path.join(a.out, "tap.csv"),
                 ["t_us", "pe", "pn", "pd", "ve", "vn"], "t_us")
n_lio = decimate(os.path.join(a.run, "lio.csv"), os.path.join(a.out, "lio.csv"),
                 ["t_us", "x", "y", "z"], "t_us")

print("wrote %s/{cmd,tap,lio}.csv  (cmd %d rows; tap %d of %d; lio %d of %d at %.0f Hz)"
      % (a.out, len(sp), n_tap[0], n_tap[1], n_lio[0], n_lio[1], a.rate))
