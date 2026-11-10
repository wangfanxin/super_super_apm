#!/usr/bin/env python3
"""along_shape.py -- what does the ALONG-track error look like: a steady lag, or an oscillation?

`tracking_diag` says the residual of the delivered sorties is now dominated by the along-track term
(p90 0.64-1.10 m) while the lateral one is small (p50 0.11 / p90 0.28).  Two very different faults fit
that: (a) the aircraft flies a slightly different SPEED PROFILE (lag then catch-up = an oscillation), or
(b) a constant time-base offset.  This prints the two speeds against each other, plus the along-track
error, so the shape decides.

    python3 scripts/along_shape.py --tag ke1-01-01 [--t0 60 --t1 80]
"""
import argparse
import csv
import os

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--tag", default="ke1-01-01")
ap.add_argument("--root", default="/home/fanxin/lio_avoid/logs")
ap.add_argument("--t0", type=float, default=60.0)
ap.add_argument("--t1", type=float, default=84.0)
ap.add_argument("--step", type=float, default=0.5)
a = ap.parse_args()

R = os.path.join(a.root, "missionA-" + a.tag)
tap = list(csv.DictReader(open(os.path.join(R, "tap.csv"))))
tt = np.array([float(r["t_us"]) * 1e-6 for r in tap]); tt -= tt[0]
tp = np.array([[float(r["pe"]), float(r["pn"])] for r in tap])
sp = list(csv.DictReader(open(os.path.join(R, "setpoints.csv"))))
ts = np.array([float(r["t_sim_us"]) * 1e-6 for r in sp]); ts -= ts[0]
cmd = np.array([[float(r["cmd_x"]), float(r["cmd_y"])] for r in sp])
vel = np.array([[float(r["vel_x"]), float(r["vel_y"])] for r in sp])

# the truth is 1.2 kHz; smooth it over 0.3 s before differentiating
k = max(2, int(0.3 / np.median(np.diff(tt))))
sm = np.stack([np.convolve(tp[:, 0], np.ones(k) / k, "same"),
               np.convolve(tp[:, 1], np.ones(k) / k, "same")], 1)
v_tr = np.gradient(sm, axis=0) / np.median(np.diff(tt)[np.median(np.diff(tt)) > 0])

# clock fit (same convention as tracking_diag: one constant shift)
best = None
for sh in np.arange(20.0, 40.0, 0.02):
    q = np.stack([np.interp(ts + sh, tt, sm[:, 0]), np.interp(ts + sh, tt, sm[:, 1])], 1)
    d = np.median(np.linalg.norm(q - cmd, axis=1))
    if best is None or d < best[0]:
        best = (d, float(sh))
sh = best[1]
q = np.stack([np.interp(ts + sh, tt, sm[:, 0]), np.interp(ts + sh, tt, sm[:, 1])], 1)
vt = np.stack([np.interp(ts + sh, tt, v_tr[:, 0]), np.interp(ts + sh, tt, v_tr[:, 1])], 1)

ref_s = np.linalg.norm(vel, axis=1)
tr_s = np.linalg.norm(vt, axis=1)
dcmd = np.gradient(cmd, axis=0) / np.median(np.diff(ts))
u = dcmd / np.maximum(np.linalg.norm(dcmd, axis=1), 1e-9)[:, None]
along = np.sum((q - cmd) * u, axis=1)

print("== %s: clock shift %+.2f s, median |plan-truth| %.3f m" % (a.tag, sh, best[0]))
print("   reference speed p50 %.2f | truth speed p50 %.2f | ratio %.2f"
      % (np.median(ref_s), np.median(tr_s), np.median(tr_s) / max(np.median(ref_s), 1e-9)))
print("   along-track error p50 %+.3f  p90(|e|) %.3f m | correlation(along, truth speed) %+.2f"
      % (np.median(along), np.percentile(np.abs(along), 90),
         float(np.corrcoef(along, tr_s)[0, 1])))
print("\n     t    ref_v  truth_v   along   lateral")
tt0 = a.t0
while tt0 <= min(a.t1, ts[-1]):
    i = int(np.searchsorted(ts, tt0).clip(0, len(ts) - 1))
    lat = float((q[i] - cmd[i])[0] * (-u[i, 1]) + (q[i] - cmd[i])[1] * u[i, 0])
    print("  %5.1f  %5.2f   %5.2f   %+6.2f   %+6.2f" % (tt0, ref_s[i], tr_s[i], along[i], lat))
    tt0 += a.step
