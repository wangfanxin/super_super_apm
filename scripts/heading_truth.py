#!/usr/bin/env python3
"""Where does the NOSE point, measured from the flight's own attitude (not from the command).

The question "机头（激光雷达）朝向什么地方？是 yaw 始终对着下一航点吗？" cannot be answered from the
command alone, because the aircraft only follows a commanded yaw if it is actually on the wire.  So this
reads the SITL truth tap's quaternion, extracts the true heading, and compares it with:

  * the direction of travel (the path tangent, in W), and
  * the yaw the plan box put on the wire (`wire.csv: yaw_out`, in L, rotated to W with the calibration).

    python3 scripts/heading_truth.py --tag jd1-01-02
"""
import argparse
import csv
import json
import math
import os

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--tag", default="jd1-01-02")
ap.add_argument("--root", default="/home/fanxin/lio_avoid/logs")
a = ap.parse_args()
R = os.path.join(a.root, "missionA-" + a.tag)


def yaw_from_quat(qx, qy, qz, qw):
    return np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


tap = list(csv.DictReader(open(os.path.join(R, "tap.csv"))))
tt = np.array([float(r["t_us"]) * 1e-6 for r in tap])
tp = np.array([[float(r["pe"]), float(r["pn"])] for r in tap])
yaw_t = yaw_from_quat(*[np.array([float(r[k]) for r in tap]) for k in ("qx", "qy", "qz", "qw")])
# the tap is NED/FRD (yaw 0 = north, clockwise positive); the world W is ENU (yaw 0 = east,
# counter-clockwise positive):  psi_ENU = pi/2 - psi_NED
yaw_t = np.pi / 2.0 - yaw_t

wire = list(csv.DictReader(open(os.path.join(R, "wire.csv"))))
tw = np.array([float(r["t_wall"]) for r in wire])
yw = np.array([float(r["yaw_out"]) for r in wire])
pw = np.array([[float(r["p_w_x"]), float(r["p_w_y"])] for r in wire])

try:
    yawL = float(json.load(open("/mnt/d/distill_self/_stage/calib.json"))["yaw_deg"])
except Exception:                                            # noqa: BLE001
    yawL = 90.049771

# The two logs use two different clocks: the tap stamps SIM time from the SITL boot, the bridge stamps
# its own wall clock.  Rebase both to their first sample before searching (a raw search would need a
# shift of thousands of seconds and would never find the minimum inside a sane window).
tt = tt - tt[0]
tw = tw - tw[0]
# align the two clocks on the positions (they are two logs of the same flight)
best = None
for sh in np.arange(-60.0, 60.0, 0.02):
    q = np.interp(tw + sh, tt, tp[:, 0]), np.interp(tw + sh, tt, tp[:, 1])
    d = np.median(np.hypot(pw[:, 0] - q[0], pw[:, 1] - q[1]))
    if best is None or d < best[0]:
        best = (d, float(sh))
print("== %s: clock fit truth(tap) -> wire: shift %+.2f s (median position residual %.2f m)"
      % (a.tag, best[1], best[0]))
sh = best[1]

# heading of the aircraft, interpolated onto the wire's clock, in W
head_w = np.interp(tw, tt - sh, yaw_t)
# direction of travel from the truth, smoothed over 0.4 s so the 1.2 kHz jitter does not dominate
k = max(2, int(0.4 / max(np.median(np.diff(tt)), 1e-6)))
tp_s = np.stack([np.convolve(tp[:, 0], np.ones(k) / k, mode="same"),
                 np.convolve(tp[:, 1], np.ones(k) / k, mode="same")], 1)
tr_w = np.stack([np.interp(tw, tt - sh, tp_s[:, 0]), np.interp(tw, tt - sh, tp_s[:, 1])], 1)
d = np.gradient(tr_w, axis=0)
spd = np.linalg.norm(d, axis=1) / max(np.median(np.diff(tw)), 1e-9)
mv = spd > 0.30
psi = np.arctan2(d[mv, 1], d[mv, 0])

print("\n-- A) the AIRCRAFT's own heading (truth quaternion) vs the direction it travels")
e = np.abs(np.angle(np.exp(1j * (psi - head_w[mv]))))
print("   |heading - travel| : p50 %.1f  p90 %.1f deg ; within 30 deg %.1f %%"
      % (np.degrees(np.percentile(e, 50)), np.degrees(np.percentile(e, 90)),
         100.0 * float(np.mean(e < np.radians(30)))))
print("   heading span over the flight: %.0f deg -> %s"
      % (np.degrees(np.ptp(np.unwrap(head_w))),
         "it TURNS with the mission" if np.degrees(np.ptp(np.unwrap(head_w))) > 120 else "it is roughly FIXED"))

print("\n-- B) the plan's yaw on the wire vs the same travel direction")
# ★ TWO CONVENTIONS, BOTH MEASURED, SO AN OLD LOG AND A NEW ONE ARE BOTH READABLE ★
#   `wire.csv: yaw_out` is whatever `send_cmd` put in the message-84 yaw field.  The CONTRACT
#   (`planner/cmd_stream.py:83-91`) is "the FCU's compass heading" (0 = north, +90 = east), i.e. an
#   ENU yaw of `pi/2 - H`.  Until 2026-11-07 the SUPER thin layer violated that contract and sent an
#   L-frame yaw verbatim (the audit measured it as MINUS the true heading, rms 28.7 deg); it now
#   converts at publish (`common.frames.l_to_wire_yaw`).  So a log can carry either, and this script
#   prints BOTH interpretations instead of assuming one -- the one that fits is the one that was on
#   the wire, and the pair is the before/after evidence.
for name, psi_cmd in (("as an FCU compass heading (the contract)", np.pi / 2.0 - yw),
                      ("as an L-frame yaw, +calib (the pre-fix producer)", yw + math.radians(yawL))):
    ew = np.abs(np.angle(np.exp(1j * (psi - psi_cmd[mv]))))
    print("   |yaw_cmd - travel| %-42s : p50 %5.1f  p90 %5.1f deg ; within 30 deg %5.1f %%"
          % (name, np.degrees(np.percentile(ew, 50)), np.degrees(np.percentile(ew, 90)),
             100.0 * float(np.mean(ew < np.radians(30)))))

print("\n-- C) does the aircraft follow the commanded yaw?  (same two conventions)")
for name, psi_cmd in (("compass", np.pi / 2.0 - yw),
                      ("L + calib", yw + math.radians(yawL))):
    ed = np.abs(np.angle(np.exp(1j * (head_w[mv] - psi_cmd[mv]))))
    print("   |heading - yaw_cmd| %-12s : p50 %5.1f  p90 %5.1f deg" %
          (name, np.degrees(np.percentile(ed, 50)), np.degrees(np.percentile(ed, 90))))
