#!/usr/bin/env python3
"""plot_hex_vxyz_t.py -- v_xy, v_z, x, y and z against time, plan vs flight (owner's round-83 figure).

    python3 scripts/plot_hex_vxyz_t.py --tag d84-01-01 --world worlds/forest_v2_hex.yaml \\
        --out docs/evidence/vxyz-t.png

Four panels share one time axis:
  1. horizontal speed        (the plan's demand against what the aircraft did; S3 is about this)
  2. vertical speed          (the launch and any altitude commands)
  3. x and y                 (the plan's position against the truth's)
  4. z                       (the same)
Vertical lines mark the lap boundaries, measured the same way as in plot_hex_laps.py (the truth passing
within 1.2 m of corner 1), so the two figures agree about where a lap starts.
"""
import argparse
import csv
import os
import sys

import numpy as np
import yaml

import matplotlib                                     # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt                       # noqa: E402

# a font that has BOTH Han glyphs and Latin punctuation -- see scripts/_figfont.py
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _figfont                                              # noqa: E402
_figfont.apply()

ap = argparse.ArgumentParser()
ap.add_argument("--tag", required=True)
ap.add_argument("--prep", default="")
ap.add_argument("--world", default="worlds/forest_v2_hex.yaml")
ap.add_argument("--out", default="docs/evidence/vxyz-t.png")
ap.add_argument("--cmd-vel-frame", choices=["w", "l"], default="w")
ap.add_argument("--calib-yaw-deg", type=float, default=90.049771)
ap.add_argument("--title-suffix", default="")
a = ap.parse_args()

prep = a.prep or os.path.join("logs", a.tag)
w = yaml.safe_load(open(a.world, encoding="utf-8"))
ring = np.array([[float(c[0]), float(c[1])] for c in w["mission"]["corners"]])[:6]

tap = list(csv.DictReader(open(os.path.join(prep, "tap.csv"), encoding="utf-8")))
tt = np.array([float(r["t_us"]) * 1e-6 for r in tap])
T = np.array([[float(r["pe"]), float(r["pn"]), -float(r["pd"])] for r in tap])
Vt = np.array([[float(r["ve"]), float(r["vn"])] for r in tap])
cmd = list(csv.DictReader(open(os.path.join(prep, "cmd.csv"), encoding="utf-8")))
ts = np.array([float(r["t_sim_us"]) * 1e-6 for r in cmd])
C = np.array([[float(r["cmd_x"]), float(r["cmd_y"]), float(r["cmd_z"])] for r in cmd])
V = np.array([[float(r["vel_x"]), float(r["vel_y"]), float(r.get("vel_z", 0.0) or 0.0)] for r in cmd])
if a.cmd_vel_frame == "l":
    yaw = np.radians(a.calib_yaw_deg)
    Cc, Sc = np.cos(yaw), np.sin(yaw)
    V = np.column_stack([Cc * V[:, 0] - Sc * V[:, 1], Sc * V[:, 0] + Cc * V[:, 1], V[:, 2]])

# BOTH STREAMS ARE ON THE SAME SIMULATED CLOCK -- so they are plotted as recorded, with NO warp.
# Round 88 audit: the old version placed the plan by PROGRESS (`ts = interp(plan_arc, truth_arc, t)`)
# while the axis said "时间 (s)".  The plan's arc is 128.8 m and the truth's 145.6 m, so the two series
# were displaced by ~12 s and the figure showed |plan-truth| p50 8.0 m / max 11.5 m at the same plotted
# x, when the real tracking is 0.43 m p90.  It also erased the along-track lag the figure exists to show.
# `setpoints.csv: t_sim_us` is the tap stamp the bridge read, i.e. the same epoch as `tap.csv: t_us`
# (measured: setpoints 28.6-147.1 s inside tap 0-154.8 s), so a direct plot is exact.
print("plan %.1f..%.1f s, truth %.1f..%.1f s (same simulated clock, no alignment)"
      % (ts[0], ts[-1], tt[0], tt[-1]))

t0 = min(ts[0], tt[0])
# lap boundaries, same rule as the lap figure
d1 = np.hypot(T[:, 0] - ring[0, 0], T[:, 1] - ring[0, 1])
near = (T[:, 2] > 1.0) & (d1 < 1.2)
cuts = []
last = -1e9
for i in np.flatnonzero(near):
    if tt[i] - last > 10.0:
        cuts.append(tt[i] - t0)
    last = tt[i]

fig, axs = plt.subplots(4, 1, figsize=(13.2, 10.4), sharex=True)
ax = axs[0]
ax.plot(ts - t0, np.hypot(V[:, 0], V[:, 1]), lw=1.4, color="#e53935", label="规划 v_xy")
ax.plot(tt - t0, np.hypot(Vt[:, 0], Vt[:, 1]), lw=0.8, color="#1565c0", alpha=0.9, label="真值 v_xy")
ax.axhline(4.0, color="#2e7d32", ls="--", lw=1.0)
ax.text(0.5, 4.05, "4 m/s", color="#2e7d32", fontsize=8)
ax.set_ylabel("水平速度 (m/s)")
ax.legend(loc="upper right", fontsize=9)
ax.grid(alpha=0.25)

ax = axs[1]
ax.plot(ts - t0, V[:, 2], lw=1.4, color="#e53935", label="规划 v_z")
ax.plot(tt - t0, np.gradient(T[:, 2], tt), lw=0.8, color="#1565c0", alpha=0.9, label="真值 v_z")
ax.axhline(0.0, color="k", lw=0.6)
ax.set_ylabel("垂直速度 (m/s)")
ax.legend(loc="upper right", fontsize=9)
ax.grid(alpha=0.25)

ax = axs[2]
ax.plot(ts - t0, C[:, 0], lw=1.2, color="#e53935", ls="-", label="规划 x")
ax.plot(tt - t0, T[:, 0], lw=0.7, color="#1565c0", label="真值 x")
ax.plot(ts - t0, C[:, 1], lw=1.2, color="#fb8c00", ls="-", label="规划 y")
ax.plot(tt - t0, T[:, 1], lw=0.7, color="#00897b", label="真值 y")
ax.set_ylabel("x / y (m)")
ax.legend(loc="upper right", fontsize=8, ncol=4)
ax.grid(alpha=0.25)

ax = axs[3]
ax.plot(ts - t0, C[:, 2], lw=1.2, color="#e53935", label="规划 z")
ax.plot(tt - t0, T[:, 2], lw=0.7, color="#1565c0", label="真值 z")
ax.axhline(2.0, color="#2e7d32", ls="--", lw=1.0)
ax.set_ylabel("z (m)")
ax.set_xlabel("时间 (s)   （两条曲线同一仿真时钟，未做任何对齐）")
ax.legend(loc="upper right", fontsize=9)
ax.grid(alpha=0.25)

for c in cuts[1:]:
    for axx in axs:
        axx.axvline(c, color="#616161", ls=":", lw=0.9)
axs[0].set_title("%s：v_xy / v_z / x,y / z 对时间（点线 = 圈边界，实测；真值峰值 v_xy %.2f m/s，"
                 ">4 m/s 共 %.1f s）%s"
                 % (a.tag, float(np.max(np.hypot(Vt[:, 0], Vt[:, 1]))),
                    float(np.sum(np.diff(tt) * (np.hypot(Vt[:-1, 0], Vt[:-1, 1]) > 4.0))),
                    a.title_suffix), fontsize=11)
axs[3].set_xlim(0, min(ts[-1], tt[-1]) - t0)
fig.tight_layout()
os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
fig.savefig(a.out, dpi=140)
print("wrote %s" % a.out)
print("   v_xy plan p50 %.2f max %.2f | truth p50 %.2f max %.2f m/s"
      % (np.median(np.hypot(V[:, 0], V[:, 1])), float(np.max(np.hypot(V[:, 0], V[:, 1]))),
         np.median(np.hypot(Vt[:, 0], Vt[:, 1])), float(np.max(np.hypot(Vt[:, 0], Vt[:, 1])))))
