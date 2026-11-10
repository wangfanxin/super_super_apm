#!/usr/bin/env python3
"""Two questions from the owner, answered from the recording instead of from the code's intention.

1. WHERE DOES THE NOSE POINT IN FLIGHT?  (and therefore where does the 45-deg-down lidar look)
   Chain: SUPER's `traj_server` computes the plan's yaw from the direction to the point
   `time_forward` = 1 s ahead on its own spline (rate limited to `yaw_dot_max`, low-pass filtered);
   the thin layer forwards it (`m.yaw = s.yaw`); the bridge's `--yaw None` default no longer replaces
   it.  So the expectation is "along the path, ~1 s ahead".  This script compares the yaw that went ON
   THE WIRE (`wire.csv: yaw_out`, W frame) with the actual direction of travel in W.

2. WHAT DOES THAT LIDAR SEE?  `sim/mid360_pattern.csv` is `Time/s, Azimuth/deg, Zenith/deg`.

    python3 scripts/yaw_and_pattern.py --tag jd1-01-02
"""
import argparse
import csv
import math
import os

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--tag", default="jd1-01-02")
ap.add_argument("--run-root", default="/home/fanxin/lio_avoid/logs")
ap.add_argument("--pattern", default="/home/fanxin/lio_avoid/sim/mid360_pattern.csv")
a = ap.parse_args()

R = os.path.join(a.run_root, "missionA-" + a.tag)

# ---------------------------------------------------------------- 1) the yaw that went on the wire
wire = list(csv.DictReader(open(os.path.join(R, "wire.csv"))))
sp = list(csv.DictReader(open(os.path.join(R, "setpoints.csv"))))
n = min(len(wire), len(sp))
yaw = np.array([float(wire[i]["yaw_out"]) for i in range(n)])
pw = np.array([[float(wire[i]["p_w_x"]), float(wire[i]["p_w_y"])] for i in range(n)])
t = np.array([float(wire[i]["t_wall"]) for i in range(n)])

d = np.gradient(pw, axis=0)
spd = np.linalg.norm(d, axis=1) / np.maximum(np.gradient(t), 1e-9)
mv = spd > 0.30                                   # only while actually moving
psi = np.arctan2(d[mv, 1], d[mv, 0])
err = np.abs(np.angle(np.exp(1j * (psi - yaw[mv]))))
print("== %s: the yaw ON THE WIRE vs the direction of travel in W (%d moving ticks of %d)"
      % (a.tag, int(mv.sum()), n))
print("   |yaw_out - atan2(dy,dx)| : p50 %.1f  p90 %.1f  max %.1f deg"
      % (np.degrees(np.percentile(err, 50)), np.degrees(np.percentile(err, 90)),
         np.degrees(err.max())))
print("   within 30 deg: %.1f %%   within 60 deg: %.1f %%"
      % (100.0 * float(np.mean(err < np.radians(30))), 100.0 * float(np.mean(err < np.radians(60)))))
lat = np.abs(np.angle(np.exp(1j * (psi - yaw[mv]))))
print("   yaw_out span over the flight: %.2f rad (%.0f deg) -> %s"
      % (yaw.max() - yaw.min(), np.degrees(yaw.max() - yaw.min()),
         "it TURNS with the mission" if (yaw.max() - yaw.min()) > 2.0 else "it is nearly FIXED"))
print("   signed error p05/p50/p95: %+.1f / %+.1f / %+.1f deg"
      % tuple(np.degrees(np.percentile(np.angle(np.exp(1j * (psi - yaw[mv]))), [5, 50, 95]))))

# The wire speaks the FCU's local frame L; the travel direction above is in W.  L is W rotated by the
# calibration yaw (about +90.05 deg), so the honest comparison rotates one of them first.
try:
    import json
    cal = json.load(open("/mnt/d/distill_self/_stage/calib.json"))
    yawL = float(cal.get("yaw_deg", cal.get("yaw", 90.049771)))
except Exception:                                            # noqa: BLE001
    yawL = 90.049771
yaw_w = yaw[mv] + math.radians(yawL)   # the wire is L; W = L + calib yaw
err_w = np.abs(np.angle(np.exp(1j * (psi - yaw_w))))
print("   after undoing the L->W calibration rotation (%.2f deg):" % yawL)
print("     |yaw - travel dir| : p50 %.1f  p90 %.1f  max %.1f deg ; within 30 deg: %.1f %%"
      % (np.degrees(np.percentile(err_w, 50)), np.degrees(np.percentile(err_w, 90)),
         np.degrees(err_w.max()), 100.0 * float(np.mean(err_w < np.radians(30)))))

# Where do the big excursions live?  SUPER computes the plan's yaw from the point `time_forward` = 1 s
# AHEAD along its own spline and rate limits it, so the expectation is "small on the straights, large
# through the 120 deg corners" -- that distinction is what makes the number interpretable.
try:
    import yaml
    w = yaml.safe_load(open("worlds/forest_hex12.yaml", encoding="utf-8"))
    ring = np.array([[float(c[0]), float(c[1])] for c in w["mission"]["corners"]][:6])
    p_mv = pw[mv]
    dc = np.min(np.linalg.norm(p_mv[:, None, :] - ring[None, :, :], axis=2), axis=1)
    print("     |yaw - travel| binned by distance to the nearest hexagon corner:")
    for lo, hi in ((0.0, 2.0), (2.0, 5.0), (5.0, 99.0)):
        m = (dc >= lo) & (dc < hi)
        if m.sum() < 10:
            continue
        print("       %4.0f-%-4.0f m : n=%5d  p50 %5.1f  p90 %5.1f deg"
              % (lo, hi, int(m.sum()), np.degrees(np.percentile(err_w[m], 50)),
                 np.degrees(np.percentile(err_w[m], 90))))
except Exception as e:                                       # noqa: BLE001
    print("     (corner binning skipped: %s)" % e)

# ---------------------------------------------------------------- 2) the sensor's own coverage
P = a.pattern
if os.path.exists(P):
    pts = np.loadtxt(P, delimiter=",", skiprows=1, max_rows=20000)
    az_deg = pts[:, 1]
    zen_deg = pts[:, 2]
    h, _ = np.histogram(az_deg, bins=36, range=(0, 360))
    print("\n== %s: %d rays, columns (Time, Azimuth/deg, Zenith/deg)" % (os.path.basename(P), len(pts)))
    print("   azimuth: %.2f .. %.2f deg, nonzero 10-deg bins %d/36, per-bin %d..%d"
          % (az_deg.min(), az_deg.max(), int((h > 0).sum()), int(h.min()), int(h.max())))
    print("   zenith : %.2f .. %.2f deg  => %.1f deg above the sensor's own axis .. %.1f deg below"
          % (zen_deg.min(), zen_deg.max(), 90.0 - zen_deg.min(), zen_deg.max() - 90.0))
    pitch = 45.0
    print("   mounted at pitch %.0f deg DOWN, so relative to the body horizon it covers"
          " %+.1f .. %+.1f deg of elevation, over the FULL 360 deg of azimuth"
          % (pitch, 90.0 - zen_deg.min() - pitch, 90.0 - zen_deg.max() - pitch))
