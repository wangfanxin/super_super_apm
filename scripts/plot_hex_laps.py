#!/usr/bin/env python3
"""plot_hex_laps.py -- one top view PER LAP: the path SUPER asked for and the trajectory actually flown.

This is the figure the owner asked for (round 83): 分圈数、路径-轨迹俯视图.  It is drawn from the compact
CSVs that `scripts/prep_run.py` makes, so it runs on the machine that has the CJK font.

WHAT IS IN IT, and there is nothing else: the trunks (as the 0.2 m squares they are at 2.0 m), the six
waypoints of that lap, the plan (red) and the flight (blue), plus the two numbers that lap can be judged on --
its minimum plan clearance and its minimum truth clearance, both measured on the box geometry.

THE LAP BOUNDARIES ARE MEASURED, NOT GUESSED.  A lap starts when the truth passes within 1.2 m of corner 1,
so the split comes from the recording rather than from a phase file whose leg numbering differs per mission.

    python3 scripts/plot_hex_laps.py --tag d84-01-01 --world worlds/forest_v2_hex.yaml \\
        --out docs/evidence --prefix lap
"""
import argparse
import csv
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src"))
from whtf.common.obstacles import surface_dist  # noqa: E402

import matplotlib                                     # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt                       # noqa: E402
from matplotlib.patches import Rectangle              # noqa: E402

# a font that has BOTH Han glyphs and Latin punctuation -- see scripts/_figfont.py
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _figfont                                              # noqa: E402
_figfont.apply()

ap = argparse.ArgumentParser()
ap.add_argument("--tag", required=True)
ap.add_argument("--prep", default="", help="default logs/<tag>")
ap.add_argument("--world", default="worlds/forest_hex12.yaml")
ap.add_argument("--out", default="docs/evidence")
ap.add_argument("--prefix", default="lap")
ap.add_argument("--pass-radius", type=float, default=1.2, help="a lap starts when the truth is this close to corner 1")
ap.add_argument("--laps", type=int, default=0,
                help="how many laps this TOUR has (default: the world file's mission.laps).  Round 87 "
                     "flies 2 laps of a frozen 3-lap world, so the figure must not say '/3'.")
ap.add_argument("--clip-m", type=float, default=0.0,
                help="drop truth samples farther than this from the pad (0 = keep everything).  A "
                     "sortie that ends in a model/estimator split keeps recording tens of metres away; "
                     "those samples are not part of the lap and would poison the clearance numbers.")
ap.add_argument("--title-suffix", default="")
a = ap.parse_args()

prep = a.prep or os.path.join("logs", a.tag)
w = yaml.safe_load(open(a.world, encoding="utf-8"))
pil = w["pillars"]
zs = float((w.get("pillar_geometry") or {}).get("z_split", 2.1))
P = np.array([[float(q["x"]), float(q["y"]), float(q["r"])] for q in pil])
_walls = (w.get("arena") or w.get("world") or {}).get("walls") or {}
_xl = _walls.get("x") or [-12.5, 12.5]
_yl = _walls.get("y") or [-12.5, 12.5]
X_LO, X_HI = float(_xl[0]), float(_xl[1])
Y_LO, Y_HI = float(_yl[0]), float(_yl[1])
corners = np.array([[float(c[0]), float(c[1])] for c in w["mission"]["corners"]])
lap_n_file = int(w["mission"].get("laps", 3))
laps_n = a.laps if a.laps > 0 else lap_n_file
ring = corners[:6]
z = float(w["mission"].get("takeoff_alt", 2.0))

tap = list(csv.DictReader(open(os.path.join(prep, "tap.csv"), encoding="utf-8")))
tt = np.array([float(r["t_us"]) * 1e-6 for r in tap])
T = np.array([[float(r["pe"]), float(r["pn"]), -float(r["pd"])] for r in tap])
if a.clip_m > 0:
    # a sortie that ends in a model/estimator split keeps recording far outside the field; those
    # samples are not part of any lap, and they would otherwise set the figure's clearance numbers
    infield = np.hypot(T[:, 0], T[:, 1]) <= a.clip_m
    if not np.all(infield):
        last = int(np.flatnonzero(infield)[-1])
        print("clip: truth leaves %.0f m of the pad at t=%.1f s; %.1f s of tail dropped"
              % (a.clip_m, tt[last] - tt[0], tt[-1] - tt[last]))
        tt = tt[:last + 1]
        T = T[:last + 1]
cmd = list(csv.DictReader(open(os.path.join(prep, "cmd.csv"), encoding="utf-8")))
ts = np.array([float(r["t_sim_us"]) * 1e-6 for r in cmd])
C = np.array([[float(r["cmd_x"]), float(r["cmd_y"]), float(r["cmd_z"])] for r in cmd])


def clearance(Pts, step=1):
    Pts = np.atleast_2d(np.asarray(Pts, float))[::step]
    out = np.full(len(Pts), np.inf)
    for k, (x, y, zz) in enumerate(Pts):
        out[k] = min(surface_dist(x - p[0], y - p[1], p[2], zz, zs) for p in P)
    return out


# ---- lap boundaries from the truth's passages of corner 1
d1 = np.hypot(T[:, 0] - ring[0, 0], T[:, 1] - ring[0, 1])
air = T[:, 2] > 1.0
near = air & (d1 < a.pass_radius)
cuts, k = [], 0
while k < len(tt):
    if near[k]:
        cuts.append(tt[k])
        k += int(20.0 * 100)                     # skip the next 20 s of samples (1 Hz decimated taps)
    k = k + 1 if k < 1 else k + 1
# the loop above is O(n) but the skip is crude: keep the FIRST time of each contiguous passage instead
near_idx = np.flatnonzero(near)
if len(near_idx):
    grp, last = [], -10 ** 9
    for i in near_idx:
        if tt[i] - last > 10.0:
            grp.append(i)
        last = tt[i]
    cuts = [tt[i] for i in grp]
print("lap starts (truth within %.1f m of corner 1): %s" % (a.pass_radius, ["%.1f" % c for c in cuts]))
if len(cuts) < 2:
    print("only %d passage(s) of corner 1 -- cannot split laps" % len(cuts))
    sys.exit(1)
cuts = cuts[:laps_n]
bounds = cuts + [tt[-1]]

os.makedirs(a.out, exist_ok=True)
made = []
for li in range(len(bounds) - 1):
    t0, t1 = bounds[li], bounds[li + 1]
    m = (tt >= t0) & (tt < t1)
    kref = (ts >= t0) & (ts < t1)
    if m.sum() < 100:
        print("  lap %d: too few samples, skipped" % (li + 1))
        continue
    sub_t = T[m][::max(1, int(m.sum() / 800))]
    sub_c = C[kref][::max(1, int(max(1, kref.sum()) / 800))]
    # never draw outside the arena: keep only the samples inside the walls (a flyaway tail would
    # otherwise paint across the margin and outside the frame)
    _in_t = (sub_t[:, 0] >= X_LO) & (sub_t[:, 0] <= X_HI) & (sub_t[:, 1] >= Y_LO) & (sub_t[:, 1] <= Y_HI)
    _in_c = (sub_c[:, 0] >= X_LO) & (sub_c[:, 0] <= X_HI) & (sub_c[:, 1] >= Y_LO) & (sub_c[:, 1] <= Y_HI)
    if not _in_t.all():
        print("  lap %d: %d truth samples outside the arena walls are not drawn"
              % (li + 1, int((~_in_t).sum())))
    if not _in_c.all():
        print("  lap %d: %d plan samples outside the arena walls are not drawn"
              % (li + 1, int((~_in_c).sum())))
    sub_t = sub_t[_in_t]
    sub_c = sub_c[_in_c]
    # FULL window, not a prefix (round 88 audit F8: [:400]/[:600] made lap-2 read +0.081 m "no
    # contact" when the real minimum is +0.019 m)
    clr_plan = clearance(sub_c, step=max(1, len(sub_c) // 400))
    clr_truth = clearance(sub_t, step=max(1, len(sub_t) // 600))
    # plan vs truth in time
    kk = np.clip(np.searchsorted(ts, tt[m]), 0, len(ts) - 1)
    dpt = np.hypot(T[m][:, 0] - C[kk, 0], T[m][:, 1] - C[kk, 1])

    fig, ax = plt.subplots(figsize=(9.6, 9.2))
    for (x, y, r) in P:
        ax.add_patch(Rectangle((x - r, y - r), 2 * r, 2 * r, facecolor="#455a64",
                               edgecolor="none", zorder=2))
    tour = np.vstack([ring, ring[0]])
    ax.plot(tour[:, 0], tour[:, 1], "--", lw=1.0, color="#1565c0", alpha=0.45, zorder=3)
    ax.plot(sub_c[:, 0], sub_c[:, 1], "-", lw=2.2, color="#e53935", alpha=0.95, zorder=5,
            label="SUPER 规划的路径（参考，100 Hz）")
    ax.plot(sub_t[:, 0], sub_t[:, 1], "-", lw=1.7, color="#1565c0", alpha=0.95, zorder=6,
            label="实飞轨迹（真值，1.2 kHz）")
    for i, (cx, cy) in enumerate(ring):
        ax.plot(cx, cy, marker="s", ms=11, mfc="none", mec="#0d47a1", mew=1.8, zorder=7)
        ax.annotate("c%d" % (i + 1), (cx, cy), textcoords="offset points", xytext=(8, 6),
                    fontsize=12, color="#0d47a1", fontweight="bold", zorder=8)
    ax.plot(0, 0, marker="o", ms=10, mfc="#ffca28", mec="k", mew=1.2, zorder=8, label="停机坪 (0,0)")
    # the arena boundary, drawn, and used as the frame (the field is +-12.0, the walls +-12.5)
    ax.add_patch(Rectangle((X_LO, Y_LO), X_HI - X_LO, Y_HI - Y_LO, fill=False,
                           edgecolor="#546e7a", lw=1.4, ls="-", zorder=1))
    ax.set_xlim(X_LO - 0.4, X_HI + 0.4)
    ax.set_ylim(Y_LO - 0.4, Y_HI + 0.4)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    ax.set_xlabel("西→东 (m)")
    ax.set_ylabel("南→北 (m)")
    ax.legend(loc="upper left", fontsize=9)
    ax.set_title("第 %d/%d 圈   t = %.1f–%.1f s%s\n"
                 "规划离柱面最小 %.3f m ｜ 真值离柱面最小 %.3f m ｜ |规划−真值| p50 %.2f / p90 %.2f m"
                 % (li + 1, laps_n, t0 - tt[0], t1 - tt[0], a.title_suffix,
                    clr_plan.min(), clr_truth.min(),
                    float(np.percentile(dpt, 50)), float(np.percentile(dpt, 90))), fontsize=11)
    out = os.path.join(a.out, "%s%d-paths.png" % (a.prefix, li + 1))
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)
    made.append(out)
    print("  lap %d: plan clearance %.3f m, truth clearance %.3f m, |plan-truth| p50 %.2f p90 %.2f -> %s"
          % (li + 1, clr_plan.min(), clr_truth.min(),
             float(np.percentile(dpt, 50)), float(np.percentile(dpt, 90)), out))
print("wrote %d lap figures" % len(made))
