#!/usr/bin/env python3
"""jerk_measure.py -- how big is the reference's a_dot, and what body angular rate does it demand?

The owner's point: a_dot is not an abstract number -- a horizontal acceleration is produced by tilting
(a = g*tan(theta) ~ g*theta), so

        d(theta)/dt = a_dot / g          (rad/s)

and asking for a big a_dot is asking the airframe to SLEW ITS ATTITUDE at that rate.  So a_dot has to be
compared with the body's angular-rate limit, not only with the planner's `max_jerk`.

Reported here: the published acceleration's derivative (that is what the FCU actually sees) on the
delivered sortie, converted into the tilt rate it implies, next to the ArduPilot rate ceiling.
"""
import argparse
import csv
import math
import os
import re
import subprocess

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--tag", default="kh1-02-01")
ap.add_argument("--root", default="/home/fanxin/lio_avoid/logs")
a = ap.parse_args()
R = os.path.join(a.root, "missionA-" + a.tag)

w = list(csv.DictReader(open(os.path.join(R, "wire.csv"))))
t = np.array([float(r["t_wall"]) for r in w]); t -= t[0]
ax = np.array([float(r["a_l_x"]) for r in w])
ay = np.array([float(r["a_l_y"]) for r in w])
dt = np.median(np.diff(t))

jer = np.stack([np.gradient(ax, t), np.gradient(ay, t)], 1)
jer_mag = np.linalg.norm(jer, axis=1)
acc_mag = np.linalg.norm(np.stack([ax, ay], 1), axis=1)

print("== %s: %d wire samples at %.0f Hz" % (a.tag, len(t), 1 / dt))
print("   published |a|      p50 %.2f  p90 %.2f  p99 %.2f  max %.2f m/s^2"
      % (np.median(acc_mag), np.percentile(acc_mag, 90), np.percentile(acc_mag, 99), acc_mag.max()))
print("   published |a_dot|  p50 %.1f  p90 %.1f  p99 %.1f  max %.1f m/s^3"
      % (np.median(jer_mag), np.percentile(jer_mag, 90), np.percentile(jer_mag, 99), jer_mag.max()))
print("   the planner's max_jerk bound is 40 m/s^3 (but see the config: penna_jerk < 0 => not enforced)")
print("   the bridge clamps da/dt at 40 m/s^3 (--ff-acc-slew), so the published value cannot exceed it")
print()
print("   a_dot -> body tilt-rate demand  omega = a_dot / g:")
for q in (50, 90, 99):
    v = np.percentile(jer_mag, q)
    print("     p%-2d  %6.1f m/s^3  ->  %5.1f deg/s" % (q, v, math.degrees(v / 9.81)))
print("     max  %6.1f m/s^3  ->  %5.1f deg/s" % (jer_mag.max(), math.degrees(jer_mag.max() / 9.81)))

# ArduPilot's rate ceiling, from the source tree if it is there
for root in ("/home/fanxin/lio_avoid/src/ardupilot", "/home/fanxin/super_super_apm/upstream/ardupilot"):
    hit = subprocess.run(["grep", "-rn", "RATE_RLL_MAX", root + "/ArduCopter/Parameters.cpp"],
                         capture_output=True, text=True)
    if hit.returncode == 0 and hit.stdout.strip():
        print("\n== ArduPilot rate limit declaration (%s):" % root)
        for line in hit.stdout.strip().splitlines()[:4]:
            print("   ", line[:120])
        break
else:
    print("\n== (ArduPilot source not found; ATC_RATE_RLL_MAX default is the airframe's roll ceiling)")
