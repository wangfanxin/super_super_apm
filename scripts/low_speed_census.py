#!/usr/bin/env python3
"""low_speed_census.py -- find EVERY place the aircraft (or the command) nearly stopped, and say what it was.

Owner's analysis rule (2026-11-26): "以后分析飞行效果的时候，你先找出所有速度<0.5 m/s 的地方，然后一一分析
是不是绕弯，这样应该比较精确。"  A corner-loop and a mid-leg deadlock look identical on a top-down plot, and
both can hide under a whole-flight p90; the census is the list of suspects, each one classified:

    LAUNCH/LAND   before the first / after the last waypoint (not a flight defect)
    CORNER-PIVOT  truth stopped near a corner AND turned >= PIVOT_DEG there   (the stop-and-pivot)
    CORNER-SLOW   truth stopped near a corner without much turning            (a plain corner slowdown)
    MID-LEG LOOP  truth stopped away from every corner and turned >= PIVOT_DEG (a real mid-path circle)
    MID-LEG STALL away from every corner, little turning                      (carrot deadlock / hold)

It also reports, per segment, whether the COMMAND stopped too (so a planning stall can be separated from a
tracking overshoot) and where the segment lies (lap index, nearest corner, along which leg).

    python3 scripts/low_speed_census.py --tag phase9f-01-01
    python3 scripts/low_speed_census.py --tag phase20a-01-01 --thr 0.5 --json /tmp/c.json
    python3 scripts/low_speed_census.py --tag phase9f-01-01 --no-figures

Figure: <tag>-lowspeed-census.png  (top-down with every segment coloured by class + the speed-norm timeline)
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import _paths as _P  # noqa: E402
import _figfont  # noqa: F401,E402

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

CLASS_COLOR = {
    "LAUNCH/LAND": "#6b7280",
    "CORNER-PIVOT": "#f87171",
    "CORNER-SLOW": "#fbbf24",
    "MID-LEG LOOP": "#a855f7",
    "MID-LEG STALL": "#22d3ee",
}


def _rows(p):
    with open(p, encoding="utf-8", errors="replace") as fh:
        return list(csv.DictReader(fh))


def _load_world(path):
    import yaml
    with open(path, encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _laps(t, p, corners, near_m=1.2):
    """lap boundaries = truth passes within near_m of corner 1 (same rule as the top-down plot)"""
    d = np.linalg.norm(p[:, :2] - corners[0], axis=1)
    near = d < near_m
    laps = []
    for i in range(1, len(t)):
        if near[i] and not near[i - 1]:
            laps.append(float(t[i]))
    return laps


def main() -> int:
    ap = argparse.ArgumentParser(description="low-speed segment census (owner's analysis rule)")
    ap.add_argument("--run", default="")
    ap.add_argument("--tag", default="")
    ap.add_argument("--world", default=os.path.join(ROOT, "sim", "worlds", "forest_hex12.yaml"))
    ap.add_argument("--thr", type=float, default=0.5, help="low-speed threshold on the truth speed 2-norm (m/s)")
    ap.add_argument("--cmd-thr", type=float, default=0.5, help="the commanded speed is 'also low' below this")
    ap.add_argument("--min-dur", type=float, default=0.15, help="a segment must last this long (s)")
    ap.add_argument("--corner-near-m", type=float, default=1.5)
    ap.add_argument("--pivot-deg", type=float, default=150.0)
    ap.add_argument("--pad-m", type=float, default=1.0)
    ap.add_argument("--json", default="")
    ap.add_argument("--no-figures", action="store_true")
    ap.add_argument("--outdir", default=os.path.join(ROOT, "docs", "evidence"))
    a = ap.parse_args()
    if a.run:
        run = os.path.abspath(a.run)
    elif a.tag:
        run = _P.sortie(a.tag)
    else:
        print("--tag or --run required")
        return 2
    tag = os.path.basename(run).replace("missionA-", "")
    short = tag[:-6] if tag.endswith("-01-01") else tag
    world = _load_world(a.world)
    corners = np.array([[float(c[0]), float(c[1])] for c in world["mission"]["corners"][:6]])

    tap = _rows(os.path.join(run, "tap.csv"))
    t = np.array([float(x["t_sim_us"]) * 1e-6 for x in tap])
    p = np.column_stack([[float(x["pe"]) for x in tap],
                         [float(x["pn"]) for x in tap],
                         [-float(x["pd"]) for x in tap]])
    v = np.column_stack([[float(x["ve"]) for x in tap],
                         [float(x["vn"]) for x in tap],
                         [-float(x["vd"]) for x in tap]])
    o = np.argsort(t)
    t, p, v = t[o], p[o], v[o]

    sp = _rows(os.path.join(run, "setpoints.csv"))
    tc = np.array([float(x["t_sim_us"]) * 1e-6 for x in sp])
    cv = np.hypot(np.array([float(x["vel_x"]) for x in sp]),
                  np.array([float(x["vel_y"]) for x in sp]))
    cp = np.column_stack([[float(x["cmd_x"]) for x in sp], [float(x["cmd_y"]) for x in sp]])

    laps = _laps(t, p, corners)
    spd = np.linalg.norm(v, axis=1)
    fly = p[:, 2] > 1.2
    low = (spd < a.thr) & fly
    idx = np.where(low)[0]
    segs = []
    i = 0
    while i < len(idx):
        j = i
        while j + 1 < len(idx) and idx[j + 1] == idx[j] + 1:
            j += 1
        k0, k1 = int(idx[i]), int(idx[j])
        dur = float(t[k1] - t[k0])
        if dur >= a.min_dur:
            pp, vv = p[k0:k1 + 1], v[k0:k1 + 1]
            path = float(np.linalg.norm(np.diff(pp[:, :2], axis=0), axis=1).sum())
            net = float(np.linalg.norm(pp[-1, :2] - pp[0, :2]))
            nz = np.linalg.norm(vv[:, :2], axis=1) > 0.15
            vh = vv[nz, :2]
            turn = 0.0
            if len(vh) > 2:
                ang = np.unwrap(np.arctan2(vh[:, 1], vh[:, 0]))
                turn = float(np.degrees(np.abs(np.diff(ang)).sum()))
            m = (tc >= t[k0]) & (tc <= t[k1])
            cmin = float(cv[m].min()) if m.any() else float("nan")
            cmean = float(cv[m].mean()) if m.any() else float("nan")
            cmdp = cp[m] if m.any() else np.zeros((0, 2))
            pos = pp[:, :2].mean(axis=0)
            d_corner = np.linalg.norm(corners - pos, axis=1)
            ci = int(np.argmin(d_corner))
            near_pad = float(np.linalg.norm(pos)) < a.pad_m
            lap = int(np.searchsorted(laps, t[k0]) + 1) if laps else 1
            if near_pad or t[k0] < (laps[0] if laps else t[0] + 5) - 3.0 or t[k0] > t[-1] - 4.0:
                cls = "LAUNCH/LAND"
            elif d_corner[ci] <= a.corner_near_m:
                cls = "CORNER-PIVOT" if turn >= a.pivot_deg else "CORNER-SLOW"
            else:
                cls = "MID-LEG LOOP" if turn >= a.pivot_deg else "MID-LEG STALL"
            segs.append(dict(t0=float(t[k0]), t1=float(t[k1]), dur=dur, lap=lap, cls=cls,
                             vmean=float(spd[k0:k1 + 1].mean()), vmin=float(spd[k0:k1 + 1].min()),
                             path=path, net=net, turn=turn, cmd_min=cmin, cmd_mean=cmean,
                             x=float(pos[0]), y=float(pos[1]), corner=ci + 1,
                             dcorner=float(d_corner[ci]),
                             cmd_speed_at=(float(np.linalg.norm(cmdp[-1] - cmdp[0]) / max(dur, 1e-9))
                                           if len(cmdp) > 1 else float("nan"))))
        i = j + 1

    print("== %s low-speed census (truth |v| < %.2f m/s, >= %.2f s)" % (short, a.thr, a.min_dur))
    print("   laps at: %s" % ", ".join("%.1f" % x for x in laps))
    counts = {}
    for s in segs:
        counts[s["cls"]] = counts.get(s["cls"], 0) + 1
    for k in sorted(counts):
        print("   %-13s x%d" % (k, counts[k]))
    print("   %-6s %-6s %-13s %5s %5s %5s %6s %5s %6s %6s  %s"
          % ("t0", "lap", "class", "dur", "vmean", "path", "net", "turn", "cmdmin", "dcorner", "where"))
    for s in segs:
        print("   %6.1f %6d %-13s %5.2f %5.2f %5.2f %6.2f %5.0f %6.2f %6.2f  (%.1f,%.1f) c%d"
              % (s["t0"], s["lap"], s["cls"], s["dur"], s["vmean"], s["path"], s["net"], s["turn"],
                 s["cmd_min"], s["dcorner"], s["x"], s["y"], s["corner"]))

    mid = [s for s in segs if s["cls"].startswith("MID")]
    corner = [s for s in segs if s["cls"].startswith("CORNER")]
    print("   SUMMARY: mid-path low-speed segments = %d (stalls %d, loops %d) | corner segments = %d"
          % (len(mid), sum(1 for s in mid if s["cls"] == "MID-LEG STALL"),
             sum(1 for s in mid if s["cls"] == "MID-LEG LOOP"), len(corner)))
    if a.json:
        json.dump(dict(tag=short, thr=a.thr, laps=laps, segments=segs,
                       summary=dict(mid_leg_stall=sum(1 for s in mid if s["cls"] == "MID-LEG STALL"),
                                    mid_leg_loop=sum(1 for s in mid if s["cls"] == "MID-LEG LOOP"),
                                    corner_pivot=sum(1 for s in corner if s["cls"] == "CORNER-PIVOT"),
                                    corner_slow=sum(1 for s in corner if s["cls"] == "CORNER-SLOW"))),
                  open(a.json, "w"), indent=1)
        print("   json: %s" % a.json)
    if a.no_figures:
        return 0

    os.makedirs(a.outdir, exist_ok=True)
    fig, axs = plt.subplots(1, 2, figsize=(15, 6.4), dpi=150)
    fig.patch.set_facecolor("#0f1116")
    for ax in axs:
        ax.set_facecolor("#0f1116")
    ax = axs[0]
    ax.plot(p[:, 0], p[:, 1], "-", color="#1f2937", lw=1.0, zorder=1)
    for s in segs:
        m = (t >= s["t0"]) & (t <= s["t1"])
        ax.plot(p[m, 0], p[m, 1], "-", color=CLASS_COLOR[s["cls"]], lw=2.4, zorder=3,
                label=s["cls"])
    for c in corners:
        ax.plot(c[0], c[1], "x", color="#fbbf24", ms=7, zorder=4)
    ax.set_aspect("equal")
    ax.set_title("%s  truth path; every low-speed (<%.1f m/s) segment coloured by class"
                 % (short, a.thr), color="#e5e7eb", fontsize=9)
    ax.grid(True, color="#374151", ls=":", lw=0.6, alpha=0.6)
    ax.tick_params(colors="#9ca3af", labelsize=8)
    for sp_ in ax.spines.values():
        sp_.set_color("#4b5563")
    h, l = ax.get_legend_handles_labels()
    uniq = {}
    for hh, ll in zip(h, l):
        uniq[ll] = hh
    ax.legend(uniq.values(), uniq.keys(), fontsize=7, loc="best", framealpha=0.25,
              facecolor="#111827", edgecolor="#374151", labelcolor="#e5e7eb")
    ax = axs[1]
    ax.plot(t, spd, "-", color="#38bdf8", lw=0.8, label="truth |v|")
    ax.plot(tc, cv, "--", color="#fb923c", lw=0.7, label="command |v_xy|")
    for s in segs:
        ax.axvspan(s["t0"], s["t1"], color=CLASS_COLOR[s["cls"]], alpha=0.35, lw=0)
    ax.axhline(a.thr, color="#f87171", ls="--", lw=0.8)
    for lap in laps:
        ax.axvline(lap, color="#6b7280", ls=":", lw=0.8)
    ax.set_xlabel("time (s, sim clock)", color="#d1d5db")
    ax.set_ylabel("speed (m/s)")
    ax.set_title("speed norm; shaded = low-speed segments", color="#e5e7eb", fontsize=9)
    ax.grid(True, color="#374151", ls=":", lw=0.6, alpha=0.6)
    ax.tick_params(colors="#9ca3af", labelsize=8)
    for sp_ in ax.spines.values():
        sp_.set_color("#4b5563")
    ax.legend(fontsize=8, framealpha=0.25, facecolor="#111827", edgecolor="#374151", labelcolor="#e5e7eb")
    fig.suptitle("%s  low-speed census (owner's rule: list every |v|<%.1f m/s, then classify the turn)"
                 % (short, a.thr), color="#e5e7eb", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    out = os.path.join(a.outdir, short + "-lowspeed-census.png")
    fig.savefig(out, facecolor=fig.get_facecolor())
    plt.close(fig)
    print("written:", out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
