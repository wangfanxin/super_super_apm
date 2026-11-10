#!/usr/bin/env python3
"""plot_hex_lio_err_t.py -- the ESTIMATOR's error against time (owner's round-83 figure).

The LIO publishes its pose in L; the tap knows the truth in W; the calib that relates them
(`_stage/calib.json`, yaw +90.049771 deg, t = [-0.0008 -0.00409 +0.11557] m) is applied here and nowhere
else in this script.  What is plotted is |LIO_W - truth|: separately horizontal and vertical, because a
horizontal estimator error eats the pillar clearance while a vertical one only shows up in W3.

    python3 scripts/plot_hex_lio_err_t.py --tag d84-01-01 \\
        --out docs/evidence/lio-err-t.png

Two panels: the errors against time, and their cumulative distribution (a p50/p90 pair says more than a
noisy trace).  A clock offset is fitted first -- the LIO stamps and the tap stamps are two different clocks,
and calling that offset an estimator error would make the number meaningless.
"""
import argparse
import csv
import os
import sys

import numpy as np

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
ap.add_argument("--out", default="docs/evidence/lio-err-t.png")
ap.add_argument("--calib-yaw-deg", type=float, default=90.049771)
ap.add_argument("--calib-t", nargs=3, type=float, default=[-0.00080, -0.00409, 0.11557])
a = ap.parse_args()

prep = a.prep or os.path.join("logs", a.tag)
tap = list(csv.DictReader(open(os.path.join(prep, "tap.csv"), encoding="utf-8")))
tt = np.array([float(r["t_us"]) * 1e-6 for r in tap])
T = np.array([[float(r["pe"]), float(r["pn"]), -float(r["pd"])] for r in tap])
lio = list(csv.DictReader(open(os.path.join(prep, "lio.csv"), encoding="utf-8")))
tl = np.array([float(r["t_us"]) * 1e-6 for r in lio])
L = np.array([[float(r["x"]), float(r["y"]), float(r["z"])] for r in lio])

# L -> W: rotate by the calib yaw and add the translation
yaw = np.radians(a.calib_yaw_deg)
C, S = np.cos(yaw), np.sin(yaw)
LW = np.column_stack([C * L[:, 0] - S * L[:, 1] + a.calib_t[0],
                      S * L[:, 0] + C * L[:, 1] + a.calib_t[1],
                      L[:, 2] + a.calib_t[2]])


def fit_offset():
    best, bt = None, 0.0
    for d in np.arange(-1.0, 1.001, 0.01):
        x = np.interp(tl + d, tt, T[:, 0])
        y = np.interp(tl + d, tt, T[:, 1])
        e = float(np.median(np.hypot(LW[:, 0] - x, LW[:, 1] - y)))
        if best is None or e < best:
            best, bt = e, d
    return bt


dt = fit_offset()
t = tl + dt
x = np.interp(t, tt, T[:, 0])
y = np.interp(t, tt, T[:, 1])
z = np.interp(t, tt, T[:, 2])
air = z > 1.0
eh = np.hypot(LW[:, 0] - x, LW[:, 1] - y)
ev = np.abs(LW[:, 2] - z)
eh, ev, ta = eh[air], ev[air], t[air] - t[0]

fig, axs = plt.subplots(1, 2, figsize=(14.2, 5.6), gridspec_kw={"width_ratios": [2.3, 1.0]})
ax = axs[0]
ax.plot(ta, eh, lw=0.7, color="#c62828", label="水平 |LIO−真值|")
ax.plot(ta, ev, lw=0.7, color="#1565c0", alpha=0.85, label="垂直 |LIO−真值|")
for v, c, lab in ((np.percentile(eh, 50), "#c62828", "水平 p50 %.3f m" % np.percentile(eh, 50)),
                  (np.percentile(eh, 90), "#c62828", "水平 p90 %.3f m" % np.percentile(eh, 90))):
    ax.axhline(v, color=c, ls="--" if "p90" in lab else ":", lw=0.9)
    ax.text(0.4, v, "  " + lab, fontsize=8, color=c, va="bottom")
ax.set_xlabel("时间 (s)")
ax.set_ylabel("误差 (m)")
ax.grid(alpha=0.25)
ax.legend(loc="upper right", fontsize=9)
ax.set_title("%s：估计器误差对时间（时钟拟合 %+0.2f s，已按 calib 把 LIO 从 L 转到 W）\n"
             "水平 p50 %.3f / p90 %.3f / max %.3f m ｜ 垂直 p50 %.3f / p90 %.3f m"
             % (a.tag, dt, np.percentile(eh, 50), np.percentile(eh, 90), float(eh.max()),
                np.percentile(ev, 50), np.percentile(ev, 90)), fontsize=10.5)

ax = axs[1]
for e, c, lab in ((eh, "#c62828", "水平"), (ev, "#1565c0", "垂直")):
    v = np.sort(e)
    ax.plot(v, np.linspace(0, 100, len(v)), lw=1.6, color=c, label=lab)
    for q in (50, 90):
        ax.plot([np.percentile(e, q)], [q], marker="o", ms=4, color=c)
        ax.annotate("%.3f" % np.percentile(e, q), (np.percentile(e, q), q),
                    textcoords="offset points", xytext=(4, -10), fontsize=8, color=c)
ax.set_xlabel("误差 (m)")
ax.set_ylabel("累计百分比 (%)")
ax.grid(alpha=0.25)
ax.legend(loc="lower right", fontsize=9)
ax.set_title("误差分布", fontsize=10.5)

fig.tight_layout()
os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
fig.savefig(a.out, dpi=140)
print("wrote %s" % a.out)
print("   clock %+0.2f s | horizontal p50 %.3f p90 %.3f max %.3f | vertical p50 %.3f p90 %.3f"
      % (dt, np.percentile(eh, 50), np.percentile(eh, 90), float(eh.max()),
         np.percentile(ev, 50), np.percentile(ev, 90)))
