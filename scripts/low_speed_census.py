#!/usr/bin/env python3
"""low_speed_census.py -- find EVERY low-speed point of a sortie and say what it is.

OWNER RULE (2026-11-27): a point whose speed is below HALF the configured speed (1.35/2 = 0.675 m/s
for the current band) is a problem that must be investigated, not just "a slow corner".  This script
lists every contiguous truth-speed and command-speed interval below the threshold, classifies it and
draws the map/time-series so the next flight starts from a list instead of a feeling.

CLASSES
  LAUNCH   before the first corner is reached (take-off transient is expected, but it is still listed)
  CORNER   the aircraft is within 1.8 m of a mission corner at the interval's slowest point
  END      within 8 s of the final corner (terminal deceleration before landing = legitimate stop)
  LOOP     net displacement much shorter than the path travelled around the interval (a circle/backtrack)
  MID      anything else -- the real "why did it slow down mid-leg?" bucket

USAGE
  python3 scripts/low_speed_census.py --tag phase117a-01-01
  python3 scripts/low_speed_census.py --tag phase117a-01-01 --fig docs/evidence/final/low-speed-117a.png
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import re
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))


def run_dir(tag):
    p = os.path.join(ROOT, ".runs", "missionA-%s" % tag)
    if not os.path.isdir(p):
        raise SystemExit("run not found: %s" % tag)
    return p


def load(tag):
    d = run_dir(tag)
    tap = list(csv.DictReader(open(os.path.join(d, "tap.csv"))))
    sp = list(csv.DictReader(open(os.path.join(d, "setpoints.csv"))))
    t = np.array([float(r["t_sim_us"]) * 1e-6 for r in tap])
    E = np.array([float(r["pe"]) for r in tap])
    N = np.array([float(r["pn"]) for r in tap])
    U = -np.array([float(r["pd"]) for r in tap])
    ts = np.array([float(r["t_sim_us"]) * 1e-6 for r in sp])
    ce = np.array([float(r["cmd_x"]) for r in sp])
    cn = np.array([float(r["cmd_y"]) for r in sp])
    cu = np.array([float(r["cmd_z"]) for r in sp])
    o = np.argsort(ts, kind="stable")
    ts, ce, cn, cu = ts[o], ce[o], cn[o], cu[o]
    k = np.concatenate([[True], np.diff(ts) > 1e-9])
    ts, ce, cn, cu = ts[k], ce[k], cn[k], cu[k]
    # corners parsed from the plan-box log ("target_w=[ ... ]")
    corners = []
    lp = os.path.join(d, "traj.log")
    if os.path.isfile(lp):
        for ln in open(lp, errors="replace"):
            m = re.search(r"target_w=\[\s*(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s*\]", ln)
            if m:
                p = (float(m.group(1)), float(m.group(2)))
                if not corners or float(np.hypot(p[0] - corners[-1][0], p[1] - corners[-1][1])) > 0.2:
                    corners.append(p)
    return dict(t=t, E=E, N=N, U=U, ts=ts, ce=ce, cn=cn, cu=cu, corners=corners)


def load_cmd_traj(tag):
    """The PUBLISHED command stream's own velocity (traj.csv.cmd.csv, L frame).

    The earlier version differentiated the bridge's 50 Hz position stream (setpoints.csv); that is an
    ALIASING ARTEFACT (sawtooth) and made the reference look 10x noisier than it is.  MEASURED: published
    |v| p50 1.22-1.24 m/s, only 11-12% of the flight below half speed, while the differentiated
    setpoints claimed 40-45 s below it.  Always take the command velocity from the message field.
    """
    d = run_dir(tag)
    rows = list(csv.DictReader(open(os.path.join(d, "traj.csv.cmd.csv"))))
    t = np.array([float(r["t_us"]) * 1e-6 for r in rows])
    o = np.argsort(t, kind="stable")
    t = t[o]
    x = np.array([float(r["x_l"]) for r in rows])[o]
    y = np.array([float(r["y_l"]) for r in rows])[o]
    vx = np.array([float(r["vx_l"]) for r in rows])[o]
    vy = np.array([float(r["vy_l"]) for r in rows])[o]
    k = np.concatenate([[True], np.diff(t) > 1e-9])
    return t[k], x[k], y[k], np.hypot(vx[k], vy[k])


def align_cmd_to_sim(tag, tt):
    """Coarse offset (s) between the traj.csv epoch clock and the sim clock, from speed correlation."""
    ts_, _, _, vs_ = load_cmd_traj(tag)
    d = load(tag)
    t, E, N = d["t"], d["E"], d["N"]
    vt = np.hypot(np.gradient(E, t), np.gradient(N, t))
    grid = np.arange(10.0, 90.0, 0.5)
    tr = np.interp(grid, t - t[0], vt)
    best, best_off = -1e9, 0.0
    for off in np.arange(-40.0, 200.0, 0.5):
        cs = np.interp(grid - off, ts_ - ts_[0], vs_, left=np.nan, right=np.nan)
        m = np.isfinite(cs)
        if m.sum() < 40:
            continue
        c = float(np.corrcoef(tr[m], cs[m])[0, 1])
        if c > best:
            best, best_off = c, float(off)
    return best_off, float(best)


def truth_cmd(d):
    t = d["t"]
    ve = np.gradient(d["E"], t)
    vn = np.gradient(d["N"], t)
    vt = np.hypot(ve, vn)
    # command speed: derivative of the command stream in sim time (same clock)
    vce = np.gradient(np.interp(t, d["ts"], d["ce"]), t)
    vcn = np.gradient(np.interp(t, d["ts"], d["cn"]), t)
    vcmd = np.hypot(vce, vcn)
    return vt, vcmd


def path_loops(tag, speed_thr=99.0, ratio_thr=1.8, win_s=1.2):
    """Find EVERY closed/backtracking loop of the truth path, independent of speed.

    OWNER NOTE (2026-11-27): a loop can be fast (102a has one on the right side that never drops below
    half speed), so a speed census alone misses it.  Sliding-window path/net ratio catches the geometry.
    """
    d = load(tag)
    t, E, N, U = d["t"], d["E"], d["N"], d["U"]
    air = (U > 1.2) & (t > t[0] + 1.0)
    vt, _ = truth_cmd(d)
    flags = np.zeros(len(t), bool)
    for i in range(len(t)):
        w = (t >= t[i] - win_s) & (t <= t[i] + win_s) & air
        if w.sum() < 15:
            continue
        idx = np.flatnonzero(w)
        pl = float(np.sum(np.hypot(np.diff(E[idx]), np.diff(N[idx]))))
        net = float(np.hypot(E[idx[-1]] - E[idx[0]], N[idx[-1]] - N[idx[0]]))
        if net < 0.35 and pl > 0.8:      # truly closed
            flags[idx] = True
        elif pl / max(net, 1e-6) > ratio_thr and pl > 1.0:
            flags[idx] = True
    out = []
    for i0, i1 in segments(flags, t, min_n=3):
        j = i0 + int(np.argmin(vt[i0:i1 + 1]))
        out.append(dict(t0=float(t[i0]), t1=float(t[i1]), dur=float(t[i1] - t[i0]),
                        pos=[float(E[j]), float(N[j])], vmin=float(vt[i0:i1 + 1].min()),
                        vmean=float(vt[i0:i1 + 1].mean())))
    return out


def segments(mask, t, min_n=3):
    out, i = [], 0
    while i < len(mask):
        if not mask[i]:
            i += 1
            continue
        j = i
        while j + 1 < len(mask) and mask[j + 1]:
            j += 1
        if j - i + 1 >= min_n:
            out.append((i, j))
        i = j + 1
    return out


def census(tag, threshold):
    d = load(tag)
    t, E, N, U = d["t"], d["E"], d["N"], d["U"]
    vt, vcmd = truth_cmd(d)
    corners = d["corners"]
    # nominal flight window: above 1.2 m, in the band, after 1 s of take-off, before the very end
    air = (U > 1.2) & (t > t[0] + 1.0)
    if air.sum() < 50:
        return {"tag": tag, "error": "no flight window"}
    t0_air, t1_air = float(t[air][0]), float(t[air][-1])
    # first corner arrival = first time truth is within 1.2 m of corner[1] (corner list includes pad=first?)
    first_arr = t0_air + 10.0
    last_arr = t1_air
    if len(corners) >= 2:
        for p in (corners[1], corners[-1]):
            dd = np.hypot(E - p[0], N - p[1])
            idx = np.flatnonzero((air) & (dd < 1.2))
            if idx.size:
                if p is corners[1]:
                    first_arr = float(t[idx[0]])
                else:
                    last_arr = float(t[idx[0]])
    if last_arr < first_arr + 10:
        last_arr = t1_air - 8.0
    rows = []
    # command stream: use the published velocity FIELD (traj.csv), aligned to the sim clock
    tc, cx, cy, vc = load_cmd_traj(tag)
    off, corr = align_cmd_to_sim(tag, t)
    streams = [("truth", vt, t)]
    # map command stream onto a coarse sim-time grid for classification
    tg = np.arange(t0_air, t1_air, 0.05)
    vcmd_g = np.interp(tg - off, tc - tc[0], vc, left=np.nan, right=np.nan)
    streams.append(("cmd", vcmd_g, tg))
    cmd_align = dict(offset_s=off, corr=corr)
    for name, v, tt in streams:
        if name == "truth":
            mask = air & (v < threshold)
            tt_use = t
        else:
            tm = np.isfinite(v) & (v < threshold)
            mask = tm
            tt_use = tt
        for i0, i1 in segments(mask, tt_use):
            dt = float(tt_use[i1] - tt_use[i0])
            seg = slice(i0, i1 + 1)
            vmin = float(np.min(v[seg]))
            tmin = float(tt_use[i0 + int(np.argmin(v[seg]))])
            # position at the slowest point (cmd rows: truth position at the aligned sim time)
            if name == "truth":
                j = i0 + int(np.argmin(v[seg]))
                pE, pN = float(E[j]), float(N[j])
            else:
                pE = float(np.interp(tmin, t, E))
                pN = float(np.interp(tmin, t, N))
            near = None
            dnear = 99.0
            for c in corners:
                dd = float(np.hypot(pE - c[0], pN - c[1]))
                if dd < dnear:
                    dnear, near = dd, c
            # loop test: path vs net over +-1.2 s around the interval
            w = (t >= tmin - 1.2) & (t <= tmin + 1.2)
            if w.sum() > 10:
                pl = float(np.sum(np.hypot(np.diff(E[w]), np.diff(N[w]))))
                net = float(np.hypot(E[w][-1] - E[w][0], N[w][-1] - N[w][0]))
                ratio = pl / max(net, 1e-6)
            else:
                ratio = 1.0
            if tmin < first_arr:
                cls = "LAUNCH"
            elif tmin > last_arr:
                cls = "END"
            elif ratio > 1.6:
                cls = "LOOP"
            elif dnear < 1.8:
                cls = "CORNER"
            else:
                cls = "MID"
            rows.append(dict(stream=name, t0=float(t[i0]), t1=float(t[i1]), dur=dt, vmin=vmin,
                             tmin=tmin, pos=[pE, pN], near_corner=[float(near[0]), float(near[1])] if near else None,
                             d_corner=dnear, class_=cls, path_net=ratio))
    out = dict(tag=tag, threshold=threshold, t_air=[t0_air, t1_air], first_arr=first_arr, last_arr=last_arr,
               corners=corners, rows=rows, loops=path_loops(tag), cmd_align=cmd_align)
    return out


def report(o):
    if "error" in o:
        print("%-20s ERROR %s" % (o["tag"], o["error"]))
        return
    rows = o["rows"]
    print("== %s  (thr %.2f m/s; launch<%.0f s, end>%.0f s)" %
          (o["tag"], o["threshold"], o["first_arr"], o["last_arr"]))
    for stream in ("truth", "cmd"):
        rr = [r for r in rows if r["stream"] == stream]
        print("   %s: %d low-speed segment(s), total %.1f s | %s" % (
            stream, len(rr), sum(r["dur"] for r in rr),
            ", ".join("%s %d (%.1fs)" % (c, sum(1 for r in rr if r["class_"] == c),
                                         sum(r["dur"] for r in rr if r["class_"] == c))
                      for c in ("LAUNCH", "CORNER", "LOOP", "MID", "END")
                      if any(r["class_"] == c for r in rr))))
        if stream == "truth" and o.get("loops"):
            print("      path loops (speed-independent): %d" % len(o["loops"]))
            for lp in sorted(o["loops"], key=lambda x: -x["dur"])[:6]:
                print("        LOOP t=%6.1f..%6.1f dur %.2f s @(%.1f,%.1f) vmin %.2f vmean %.2f" %
                      (lp["t0"], lp["t1"], lp["dur"], lp["pos"][0], lp["pos"][1], lp["vmin"], lp["vmean"]))
        bad = [r for r in rr if r["class_"] not in ("LAUNCH", "END")]
        for r in sorted(bad, key=lambda x: -x["dur"])[:8]:
            print("      %-6s t=%6.1f dur %.2f s vmin %.2f @(%.1f,%.1f) d_corner %.2f path/net %.2f" %
                  (r["class_"], r["tmin"], r["dur"], r["vmin"], r["pos"][0], r["pos"][1],
                   r["d_corner"], r["path_net"]))


def figure(tags, out, threshold):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    try:
        import _figfont
        cjk = _figfont.ensure_cjk_font(verbose=False)
        plt.rcParams["font.sans-serif"] = [cjk, "DejaVu Sans"]
    except Exception:  # noqa: BLE001
        pass
    plt.rcParams["axes.unicode_minus"] = False
    data = [(t, census(t, threshold)) for t in tags]
    data = [(t, o) for t, o in data if "error" not in o]
    if not data:
        print("no data")
        return
    n = len(data)
    fig, axes = plt.subplots(2, n, figsize=(6.4 * n, 11.0), squeeze=False)
    colmap = {"LAUNCH": "#9e9e9e", "CORNER": "#ff9800", "LOOP": "#d32f2f", "MID": "#7b1fa2", "END": "#90caf9"}
    for c, (tag, o) in enumerate(data):
        d = load(tag)
        t, E, N = d["t"], d["E"], d["N"]
        ax = axes[0, c]
        ax.plot(E, N, lw=0.7, color="#607d8b", alpha=0.8)
        for r in o["rows"]:
            if r["stream"] != "truth":
                continue
            ax.scatter(r["pos"][0], r["pos"][1], s=26, color=colmap[r["class_"]], zorder=5)
            if r["class_"] not in ("LAUNCH", "CORNER"):
                ax.annotate("%.2f m/s/%.1fs" % (r["vmin"], r["dur"]), r["pos"], fontsize=6,
                            xytext=(4, 4), textcoords="offset points", color=colmap[r["class_"]])
        for p in o["corners"][1:]:
            ax.plot(p[0], p[1], "x", color="#455a64", ms=6)
        for lp in o.get("loops", []):
            ax.plot(lp["pos"][0], lp["pos"][1], "o", mfc="none", mec="#d32f2f", ms=11, mew=1.2)
        for k, lab in (("LAUNCH", "launch"), ("CORNER", "corner"), ("LOOP", "loop"), ("MID", "mid"), ("END", "end")):
            pass
        ax.set_title("%s: truth |v| < %.2f m/s points" % (tag, threshold), fontsize=9)
        ax.set_aspect("equal", "box")
        ax.grid(alpha=0.2)
        ax = axes[1, c]
        vt, vcmd = truth_cmd(d)
        ax.plot(d["t"], vt, lw=0.7, color="#263238", label="truth |v|")
        ax.plot(d["t"], vcmd, lw=0.6, color="#1e88e5", alpha=0.8, label="cmd |v|")
        ax.axhline(threshold, color="#d32f2f", ls="--", lw=0.8, label="half configured speed")
        ax.axvline(o["first_arr"], color="#9e9e9e", lw=0.7, ls=":")
        ax.axvline(o["last_arr"], color="#9e9e9e", lw=0.7, ls=":")
        ax.set_ylim(0, 2.0)
        ax.set_xlabel("sim t (s)")
        ax.set_ylabel("m/s")
        ax.grid(alpha=0.2)
        ax.legend(fontsize=6)
    fig.tight_layout()
    fig.savefig(out, dpi=140)
    print("figure written:", out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", action="append", required=True)
    ap.add_argument("--threshold", type=float, default=0.675, help="默认 = 1.35/2")
    ap.add_argument("--json", default="")
    ap.add_argument("--fig", default="")
    a = ap.parse_args()
    outs = [census(t, a.threshold) for t in a.tag]
    for o in outs:
        report(o)
    if a.json:
        json.dump(outs, open(a.json, "w"), indent=1)
    if a.fig:
        figure(a.tag, a.fig, a.threshold)


if __name__ == "__main__":
    main()
