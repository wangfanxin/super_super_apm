#!/usr/bin/env python3
"""why_fast.py -- the truth flies at 3-4 m/s while the reference never exceeds 1.5.  Which of these is it?

  (a) THE REFERENCE IS NOT AS SMOOTH AS ITS VELOCITY FIELD SAYS -- the position stream steps forward
      faster than the published |v| on some ticks (that is what the earlier `maxlag` clamp did).
  (b) THE POSITION LOOP OVERDRIVES -- the aircraft is behind, so the FCU adds speed on top of the
      feed-forward, and the sum exceeds the mission speed.
  (c) IT IS NOT THE REFERENCE AT ALL -- the bursts coincide with the corners, i.e. with the reference
      stopping/restarting (the measured 44 stop episodes).

Everything is measured on one clock (the setpoints log), with the truth interpolated onto it.

    python3 scripts/why_fast.py --tag ke1-01-01
"""
import argparse
import csv
import os

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--tag", default="ke1-01-01")
ap.add_argument("--root", default="/home/fanxin/lio_avoid/logs")
a = ap.parse_args()
R = os.path.join(a.root, "missionA-" + a.tag)

sp = list(csv.DictReader(open(os.path.join(R, "setpoints.csv"))))
t = np.array([float(r["t_wall"]) for r in sp]); t -= t[0]
cmd = np.array([[float(r["cmd_x"]), float(r["cmd_y"])] for r in sp])
vel = np.array([[float(r["vel_x"]), float(r["vel_y"])] for r in sp])
acc = np.array([[float(r["acc_x"]), float(r["acc_y"])] for r in sp])
fcu = np.array([[float(r["fcu_x"]), float(r["fcu_y"])] for r in sp])

tap = list(csv.DictReader(open(os.path.join(R, "tap.csv"))))
tt = np.array([float(r["t_us"]) * 1e-6 for r in tap]); tt -= tt[0]
tp = np.array([[float(r["pe"]), float(r["pn"])] for r in tap])

# clock: the plan log starts ~29-30 s after the tap (same convention as tracking_diag)
best = None
for sh in np.arange(20.0, 40.0, 0.02):
    q = np.stack([np.interp(t + sh, tt, tp[:, 0]), np.interp(t + sh, tt, tp[:, 1])], 1)
    d = np.median(np.linalg.norm(q - cmd, axis=1))
    if best is None or d < best[0]:
        best = (d, float(sh))
sh = best[1]
k = max(2, int(0.2 / np.median(np.diff(tt))))
sm = np.stack([np.convolve(tp[:, 0], np.ones(k) / k, "same"),
               np.convolve(tp[:, 1], np.ones(k) / k, "same")], 1)
q = np.stack([np.interp(t + sh, tt, sm[:, 0]), np.interp(t + sh, tt, sm[:, 1])], 1)
vtr = np.stack([np.interp(t + sh, tt, np.gradient(sm[:, 0], tt)),
                np.interp(t + sh, tt, np.gradient(sm[:, 1], tt))], 1)

dt = np.diff(t)
dp = np.linalg.norm(np.diff(cmd, axis=0), axis=1)
v_step = dp / np.maximum(dt, 1e-9)               # the speed the POSITION stream actually advances at
pub = np.linalg.norm(vel, axis=1)[1:]
tr = np.linalg.norm(vtr, axis=1)[1:]
err = np.linalg.norm(q - cmd, axis=1)[1:]
aref = np.linalg.norm(acc, axis=1)[1:]

print("== %s: %d ticks, clock shift %+.2f s" % (a.tag, len(t), sh))
print("   published |v|      p50 %.2f  p99 %.2f  max %.2f m/s" % (np.median(pub), np.percentile(pub, 99), pub.max()))
print("   cmd-position speed p50 %.2f  p99 %.2f  max %.2f m/s   <- (a) > published means the stream steps"
      % (np.median(v_step), np.percentile(v_step, 99), v_step.max()))
print("   truth |v| (0.2 s)  p50 %.2f  p99 %.2f  max %.2f m/s" % (np.median(tr), np.percentile(tr, 99), tr.max()))
print("   |position error|   p50 %.2f  p90 %.2f  max %.2f m" % (np.median(err), np.percentile(err, 90), err.max()))
over = tr > 2.0
print("\n   ticks with truth speed > 2.0 m/s: %d (%.1f %%)" % (int(over.sum()), 100.0 * over.mean()))
if over.any():
    print("     published |v| there : p50 %.2f  max %.2f m/s" % (np.median(pub[over]), pub[over].max()))
    print("     position error there: p50 %.2f  p90 %.2f m" % (np.median(err[over]), np.percentile(err[over], 90)))
    print("     reference |a| there : p50 %.2f  p90 %.2f m/s^2" % (np.median(aref[over]), np.percentile(aref[over], 90)))
print("     (whole flight)      : published %.2f, error %.2f, |a| %.2f"
      % (np.median(pub), np.median(err), np.median(aref)))

c = np.corrcoef(tr, err)[0, 1]
c2 = np.corrcoef(tr[:len(v_step)], v_step[:len(tr)])[0, 1]
print("\n   correlation(truth speed, position error)      %+.2f   <- (b) overdrive" % c)
print("   correlation(truth speed, cmd-position speed)  %+.2f   <- (a) reference steps" % c2)

# (c) are the bursts at the corners?
try:
    import yaml
    w = yaml.safe_load(open("worlds/forest_hex12.yaml", encoding="utf-8"))
    ring = np.array([[float(cc[0]), float(cc[1])] for cc in w["mission"]["corners"]][:6])
    dc = np.min(np.linalg.norm(cmd[1:][:, None, :] - ring[None, :, :], axis=2), axis=1)
    print("\n   truth speed > 2.0 m/s, binned by distance to the nearest corner:")
    for lo, hi in ((0, 2), (2, 5), (5, 99)):
        m = over & (dc >= lo) & (dc < hi)
        if m.sum():
            print("     %3d-%-3d m: %5d ticks, truth |v| p50 %.2f" % (lo, hi, int(m.sum()), np.median(tr[m])))
except Exception as e:                                       # noqa: BLE001
    print("   (corner binning skipped: %s)" % e)
