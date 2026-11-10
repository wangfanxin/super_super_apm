#!/usr/bin/env python3
"""What does the REFERENCE itself do?  A mean ground speed of 2.2 m/s cannot be blamed on the
controller until the reference's own velocity, which `setpoints.csv` carries per sample, is shown.

Reported:
  * the reference's speed distribution (this is the number the aircraft is asked to fly);
  * the number of times the reference comes to a full stop, and its period (a stop-and-go plan
    shows up as a sawtooth with a fixed period; a slow optimisation shows up as a low ceiling);
  * a one-second bin table, so the shape is visible without a figure.

  ref_speed_audit.py RUN [--t0 S] [--t1 S]
"""
import argparse
import os

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("run")
ap.add_argument("--t0", type=float, default=None, help="route window start (sim s)")
ap.add_argument("--t1", type=float, default=None, help="route window end (sim s)")
ap.add_argument("--bin", type=float, default=2.0, help="bin width for the table, s")
a = ap.parse_args()

sp = os.path.join(a.run, "setpoints.csv")
hdr = open(sp, encoding="utf-8").readline().strip().split(",")
d = np.loadtxt(sp, delimiter=",", skiprows=1, ndmin=2)
c = {n: i for i, n in enumerate(hdr)}
t = d[:, c["t_sim_us"]] * 1e-6
t -= t[0]
vx, vy, vz = d[:, c["vel_x"]], d[:, c["vel_y"]], d[:, c["vel_z"]]
v = np.hypot(vx, vy)
cmd = d[:, [c["cmd_x"], c["cmd_y"], c["cmd_z"]]]

sel = np.ones(len(t), bool)
if a.t0 is not None:
    sel &= t >= a.t0
if a.t1 is not None:
    sel &= t <= a.t1
t, v, vz, cmd = t[sel], v[sel], vz[sel], cmd[sel]
if len(t) < 10:
    raise SystemExit("no samples in the window")

dt = np.diff(t)
print("samples %d over %.1f s (%.1f Hz)" % (len(t), t[-1] - t[0], len(t) / max(1e-9, t[-1] - t[0])))
print()
print("REFERENCE horizontal speed |v_xy| m/s:")
print("   mean %.2f  median %.2f  p10 %.2f  p50 %.2f  p90 %.2f  p95 %.2f  max %.2f"
      % (v.mean(), np.median(v), np.percentile(v, 10), np.percentile(v, 50),
         np.percentile(v, 90), np.percentile(v, 95), v.max()))
print("   commanded path length %.1f m in %.1f s -> %.2f m/s along the path"
      % (float(np.sum(np.linalg.norm(np.diff(cmd, axis=0), axis=1))), t[-1] - t[0],
         float(np.sum(np.linalg.norm(np.diff(cmd, axis=0), axis=1))) / (t[-1] - t[0])))
print()
for lo, hi in ((0.0, 0.1), (0.1, 0.5), (0.5, 1.5), (1.5, 2.5), (2.5, 3.5), (3.5, 4.5),
               (4.5, 5.5), (5.5, 99)):
    frac = float(np.sum(dt * ((v[:-1] >= lo) & (v[:-1] < hi)))) / (t[-1] - t[0])
    print("   ref speed %4.1f-%-4.1f : %5.1f %% of the time" % (lo, hi, 100 * frac))
print()

# stops: contiguous runs below the threshold, and the period between their onsets
for thr in (0.10, 0.30, 0.60):
    below = v < thr
    edges = np.flatnonzero(np.diff(below.astype(int)) == 1)      # entering a stop
    if len(edges) < 2:
        print("   stops below %.2f m/s: %d" % (thr, len(edges)))
        continue
    onsets = t[edges]
    per = np.diff(onsets)
    print("   stops below %.2f m/s: %d, spacing median %.2f s (p10 %.2f p90 %.2f)"
          % (thr, len(edges), np.median(per), np.percentile(per, 10), np.percentile(per, 90)))
print()

nb = max(1, int(round((t[-1] - t[0]) / a.bin)))
print("one row per %.1f s:  mean |v| , max |v| , %% of the bin stopped (<0.3), cmd_z" % a.bin)
for k in range(nb):
    lo, hi = t[0] + k * a.bin, t[0] + (k + 1) * a.bin
    m = (t >= lo) & (t < hi)
    if not m.any():
        continue
    vm = v[m]
    bar = "#" * int(round(20.0 * float(vm.mean()) / 5.0))
    print("  t %6.1f-%6.1f  mean %.2f max %.2f  stopped %5.1f %%  z %.2f  %s"
          % (lo, hi, vm.mean(), vm.max(), 100.0 * float((vm < 0.3).mean()), cmd[m][-1, 2], bar))
