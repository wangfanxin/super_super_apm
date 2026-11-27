#!/usr/bin/env python3
"""guide_flip.py -- make the planner's guide/replan behaviour measurable from a sortie's logs.

WHY (owner round 14, after the independent audit): the earlier "guide flips its detour side" claim could not
be reproduced with the very criterion that was inserted into the planner: it was threshold-sensitive, and
the audit's better signal is that the guide BASE jumps between "whole committed trajectory is free -> use it"
and "retreat receding_dis metres + fresh A*" every 100 ms.  This script turns that into an event table:

  * GUIDE dumps (GUIDEDBG): wall time, n points, goal, length, astar flag, sampled polyline;
  * REPLAN decisions (REPLANDBG, WHTF_REPLAN_DEBUG=1): new_goal / keep / whole_free / split / remain;
  * events: astar-per-replan, base jumps between consecutive same-goal guides (max lateral difference to the
    previous polyline), side-flip counts at several thresholds (sensitivity table), whole_free transitions;
  * reconstruction rate inside vs outside the S2 path-loop windows (from low_speed_census).

USAGE
  python3 scripts/guide_flip.py --tag phase129a-01-01 --json /tmp/g129.json --fig docs/evidence/final/guide-flip-129a.png
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


def epoch_to_sim(tag):
    """Offset such that sim_time = epoch - offset, from traj.csv.cmd speed vs tap truth speed."""
    d = run_dir(tag)
    tr = list(csv.DictReader(open(os.path.join(d, "traj.csv.cmd.csv"))))
    te = np.array([float(r["t_us"]) * 1e-6 for r in tr])
    vc = np.array([np.hypot(float(r["vx_l"]), float(r["vy_l"])) for r in tr])
    o = np.argsort(te)
    te, vc = te[o], vc[o]
    tap = list(csv.DictReader(open(os.path.join(d, "tap.csv"))))
    ts = np.array([float(r["t_sim_us"]) * 1e-6 for r in tap])
    E = np.array([float(r["pe"]) for r in tap])
    N = np.array([float(r["pn"]) for r in tap])
    vt = np.hypot(np.gradient(E, ts), np.gradient(N, ts))
    grid = np.arange(10.0, 120.0, 0.2)
    tg = np.interp(grid, ts - ts[0], vt)
    best, best_off = -9.0, te[0]
    for off in np.arange(te[0] - 200.0, te[0] + 200.0, 0.25):
        cs = np.interp(grid, te - off, vc, left=np.nan, right=np.nan)
        m = np.isfinite(cs)
        if m.sum() < 50:
            continue
        c = float(np.corrcoef(tg[m], cs[m])[0, 1])
        if c > best:
            best, best_off = c, float(off)
    return best_off, best


def calib_l2w():
    p = os.path.join(ROOT, "calib", "calib.json")
    c = json.load(open(p, encoding="utf-8"))
    R = np.array(c["R_WL"], float)[:2, :2]
    t = np.array(c["t_WL"], float)[:2]

    def f(p_l):
        p_l = np.asarray(p_l, float)
        return p_l @ R.T + t

    return f


def parse_all(tag):
    d = run_dir(tag)
    off, corr = epoch_to_sim(tag)
    l2w = calib_l2w()
    guides, replans = [], []
    p = os.path.join(d, "traj.log")
    for ln in open(p, errors="replace"):
        if "GUIDEDBG wall=" in ln:
            m = re.search(r"GUIDEDBG wall=([\d.]+) n=(\d+) goal=\(([-\d.]+),([-\d.]+)\) len=([\d.]+) "
                          r"(?:astar=(\d+) )?pts=(.*) END=\(([-\d.]+),([-\d.]+)\)", ln)
            if not m:
                continue
            pts = re.findall(r"\(([-\d.]+),([-\d.]+)\)", m.group(7))
            pL = np.array([[float(a), float(b)] for a, b in pts] + [[float(m.group(8)), float(m.group(9))]])
            pw = l2w(pL)
            gl = l2w([[float(m.group(3)), float(m.group(4))]])[0]
            guides.append(dict(sim=float(m.group(1)) - off, n=int(m.group(2)),
                               goalL=[float(m.group(3)), float(m.group(4))], goalW=gl.tolist(),
                               length=float(m.group(5)), astar=int(m.group(6) or 0),
                               ptsW=pw))
        elif "REPLANDBG wall=" in ln:
            m = re.search(r"wall=([\d.]+) new_goal=(\d) keep=(\d) whole_free=(\d) split=([-\d.]+) remain=([-\d.]+)", ln)
            if m:
                replans.append(dict(sim=float(m.group(1)) - off, new_goal=int(m.group(2)),
                                    keep=int(m.group(3)), whole_free=int(m.group(4)),
                                    split=float(m.group(5)), remain=float(m.group(6))))
    return guides, replans, off, corr


def side_of(g, st=None):
    p = g["ptsW"]
    s = p[0] if st is None else np.asarray(st, float)
    e = np.asarray(g["goalW"], float)
    d = e - s
    n = np.array([-d[1], d[0]])
    nn = float(np.linalg.norm(n))
    if nn < 1e-6 or len(p) < 3:
        return 0.0
    mid = p[len(p) // 2]
    return float((mid - s) @ n / nn)


def base_jump(g_prev, g_new):
    """max distance from the previous polyline to the new guide points (same goal assumed)."""
    if g_prev is None or g_new is None:
        return 0.0
    from scipy.spatial import cKDTree
    tree = cKDTree(g_prev["ptsW"])
    d, _ = tree.query(g_new["ptsW"])
    return float(np.max(d))


def analyse(tag, side_thr=0.15):
    guides, replans, off, corr = parse_all(tag)
    from scipy.spatial import cKDTree
    events = []
    prev_same_goal = None
    for i, g in enumerate(guides):
        jump = 0.0
        if prev_same_goal is not None and np.linalg.norm(np.asarray(g["goalW"]) - np.asarray(prev_same_goal["goalW"])) < 0.8:
            jump = base_jump(prev_same_goal, g)
        g["base_jump_m"] = jump
        g["side"] = side_of(g)
        events.append(dict(sim=g["sim"], n=g["n"], goal=g["goalW"], len=g["length"],
                           astar=g["astar"], base_jump=jump, side=g["side"]))
        if np.linalg.norm(np.asarray(g["goalW"]) - np.asarray(g["goalW"])) < 1e9:
            prev_same_goal = g
    side = np.array([g["side"] for g in guides]) if guides else np.array([])
    sens = {}
    for thr in (0.0, 0.10, 0.15, 0.20, 0.30):
        n_flip = 0
        for i in range(1, len(side)):
            if side[i - 1] * side[i] < 0 and abs(side[i]) > thr and abs(side[i - 1]) > thr:
                n_flip += 1
        sens[str(thr)] = n_flip
    jumps = [e["base_jump"] for e in events]
    wf = np.array([r["whole_free"] for r in replans]) if replans else np.array([])
    wf_flips = int(np.sum(np.diff(wf) != 0)) if wf.size > 1 else 0
    # loop windows from the census
    loops = []
    try:
        import low_speed_census as L
        cen = L.census(tag, 0.675)
        loops = cen.get("loops", [])
    except Exception:  # noqa: BLE001
        pass
    def rate_in(ws):
        gs = [g for g in guides if any(w["t0"] <= g["sim"] <= w["t1"] for w in ws)]
        dur = sum(w["t1"] - w["t0"] for w in ws)
        return (len(gs) / max(dur, 1e-6), sum(1 for g in gs if g["astar"]) / max(dur, 1e-6))
    in_rate, in_astar = rate_in(loops) if loops else (0.0, 0.0)
    all_gs = len(guides)
    total_dur = (guides[-1]["sim"] - guides[0]["sim"]) if len(guides) > 1 else 1.0
    out_rate = (all_gs - len(loops) * 0 - 0) / total_dur
    out_astar = sum(g["astar"] for g in guides) / total_dur
    out = dict(tag=tag, n_guides=len(guides), n_replans=len(replans), offset=off, corr=corr,
               sim_span=[guides[0]["sim"], guides[-1]["sim"]] if guides else [0, 0],
               astar_frac=float(np.mean([g["astar"] for g in guides])) if guides else 0.0,
               base_jump_p50=float(np.median(jumps)) if jumps else 0.0,
               base_jump_p90=float(np.percentile(jumps, 90)) if jumps else 0.0,
               base_jump_max=float(max(jumps)) if jumps else 0.0,
               n_base_jump_gt035=int(sum(1 for j in jumps if j > 0.35)),
               side_flip_sensitivity=sens,
               whole_free=wf.tolist(), whole_free_flips=wf_flips,
               loops=[dict(t0=l["t0"], t1=l["t1"], pos=l["pos"]) for l in loops],
               rate_all=dict(guides_per_s=out_rate, astar_per_s=out_astar),
               rate_in_loops=dict(guides_per_s=in_rate, astar_per_s=in_astar),
               events=events[:2000])
    return out


def report(o):
    if "error" in o:
        print("%-20s ERROR %s" % (o.get("tag"), o["error"]))
        return
    print("== %s  guides=%d replans=%d sim %.1f..%.1f" % (o["tag"], o["n_guides"], o["n_replans"],
                                                          o["sim_span"][0], o["sim_span"][1]))
    print("   astar frac %.2f | base jump p50 %.2f p90 %.2f max %.2f | jumps>0.35 m: %d" %
          (o["astar_frac"], o["base_jump_p50"], o["base_jump_p90"], o["base_jump_max"], o["n_base_jump_gt035"]))
    print("   whole_free flips: %d | side-flip sensitivity: %s" % (o["whole_free_flips"], o["side_flip_sensitivity"]))
    print("   rebuild rate: all %.2f guides/s (astar %.2f) | in loops %.2f (astar %.2f)" %
          (o["rate_all"]["guides_per_s"], o["rate_all"]["astar_per_s"],
           o["rate_in_loops"]["guides_per_s"], o["rate_in_loops"]["astar_per_s"]))
    for l in o["loops"][:4]:
        print("   loop %.1f..%.1f @(%.1f,%.1f)" % (l["t0"], l["t1"], l["pos"][0], l["pos"][1]))


def figure(tags, outs, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["axes.unicode_minus"] = False
    n = len(outs)
    fig, axes = plt.subplots(n, 1, figsize=(12.5, 3.3 * n), squeeze=False)
    for ax, o in zip(axes[:, 0], outs):
        gs = np.array([e["sim"] for e in o["events"]])
        side = np.array([e["side"] for e in o["events"]])
        jumps = np.array([e["base_jump"] for e in o["events"]])
        ax.plot(gs, side, lw=0.8, color="#1e88e5", label="guide side (m)")
        ax2 = ax.twinx()
        ax2.plot(gs, jumps, lw=0.5, color="#d32f2f", alpha=0.6, label="base jump (m)")
        ax2.set_ylabel("base jump (m)", color="#d32f2f")
        ax.set_ylabel("side (m)", color="#1e88e5")
        ax.set_xlabel("sim t (s)")
        for l in o["loops"]:
            ax.axvspan(l["t0"], l["t1"], color="#ff9800", alpha=0.18)
        ax.set_title("%s: guide side / base jump (orange = S2 loop windows)" % o["tag"], fontsize=9)
        ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(out, dpi=140)
    print("figure written:", out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", action="append", required=True)
    ap.add_argument("--json", default="")
    ap.add_argument("--fig", default="")
    ap.add_argument("--side-thr", type=float, default=0.15)
    a = ap.parse_args()
    outs = []
    for t in a.tag:
        try:
            o = analyse(t, a.side_thr)
        except Exception as e:  # noqa: BLE001
            o = {"tag": t, "error": str(e)}
        outs.append(o)
        report(o)
    if a.json:
        json.dump(outs, open(a.json, "w"), indent=1)
        print("json written:", a.json)
    if a.fig:
        figure(a.tag, [o for o in outs if "error" not in o], a.fig)


if __name__ == "__main__":
    main()
