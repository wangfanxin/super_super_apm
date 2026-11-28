#!/usr/bin/env python3
"""plot_rest2rest.py -- where and how a rest2rest handover manifests in a sortie.

A rest2rest handover is PlanFromRest after the committed trajectory finished (FSM: FOLLOW_TRAJ -> TrajFinish ->
GENERATE_TRAJ).  This tool marks every such commit on the top-down path and against the speed traces, so the
question "does it show up as a stop, a loop, or a detour?" can be answered from data instead of narrative.

USAGE: python3 scripts/plot_rest2rest.py --tag phase180d-01-01
"""
import argparse, csv, os, re, sys
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import low_speed_census as L


def events(tag, run):
    sp = list(csv.DictReader(open(os.path.join(run, "setpoints.csv"))))
    tw = np.array([float(r["t_wall"]) for r in sp])
    ts = np.array([float(r["t_sim_us"]) * 1e-6 for r in sp])
    o = np.argsort(ts); ts, tw = ts[o], tw[o]
    k = np.concatenate([[True], np.diff(ts) > 1e-9]); ts, tw = ts[k], tw[k]
    ev, cur = [], None
    for ln in open(os.path.join(run, "traj.log"), encoding="utf-8", errors="replace"):
        m = re.search(r"PLANDBG wall=([0-9.]+)", ln)
        if m:
            cur = float(np.interp(float(m.group(1)), tw, ts))
        if "COMMIT " in ln and "rest2rest" in ln:
            mt = re.search(r"total=([0-9.]+)", ln)
            ev.append((cur if cur is not None else float("nan"),
                       float(mt.group(1)) if mt else float("nan")))
    return ev


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="phase180d-01-01")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    run = L.run_dir(a.tag)
    d = L.load(a.tag)
    t, E, N, ts, ce, cn = d["t"], d["E"], d["N"], d["ts"], d["ce"], d["cn"]
    corners = d["corners"]
    # truth / reference speed
    vt = np.hypot(np.gradient(E, t), np.gradient(N, t))
    vr = np.hypot(np.gradient(ce, ts), np.gradient(cn, ts))
    ker = np.ones(9) / 9.0
    vr = np.convolve(np.pad(vr, 4, mode="edge"), ker, mode="valid")
    ev = [e for e in events(a.tag, run) if np.isfinite(e[0])]
    print("== rest2rest events: %d" % len(ev))
    rows = []
    for i, (te, dur) in enumerate(ev, 1):
        j = int(np.argmin(np.abs(t - te)))
        p = np.array([E[j], N[j]])
        if corners:
            cc = np.array(corners)
            kbest = int(np.argmin(np.hypot(cc[:, 0] - p[0], cc[:, 1] - p[1])))
            dc = float(np.hypot(cc[kbest, 0] - p[0], cc[kbest, 1] - p[1]))
            lab = "c%d" % (kbest + 1)
        else:
            dc, lab = float("nan"), "?"
        w = (t >= te - 1.0) & (t <= te + 1.5)
        vmin = float(np.min(vt[w])) if w.any() else float("nan")
        rows.append((i, te, lab, dc, vmin, dur))
        print("  #%2d t=%7.2f s  %-4s d(corner)=%.2f m  truth v_min(±1s)=%.2f m/s  prev traj %.2f s"
              % (i, te, lab, dc, vmin, dur))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(15, 13))
    gs = fig.add_gridspec(3, 2, height_ratios=[1.25, 1.0, 1.0], hspace=0.32, wspace=0.18)
    ax = fig.add_subplot(gs[0, :])
    import yaml
    w = yaml.safe_load(open(os.path.join(ROOT, "sim", "worlds", "forest_hex12.yaml"), encoding="utf-8"))
    for p in w["pillars"]:
        r = float(p.get("r", 0.1))
        ax.add_patch(plt.Rectangle((float(p["x"]) - r, float(p["y"]) - r), 2 * r, 2 * r,
                                   color="0.75", zorder=1))
    ax.plot(E, N, "-", color="0.15", lw=1.0, zorder=2, label="truth")
    ax.plot(ce, cn, "-", color="#e08a1e", lw=0.8, alpha=0.7, zorder=2, label="reference")
    for i, c in enumerate(corners):
        ax.plot(c[0], c[1], "o", color="#1f6fd0", ms=7, zorder=4)
        ax.annotate("c%d" % (i + 1), (c[0] + 0.35, c[1] + 0.35), fontsize=9, color="#1f6fd0")
    for i, (te, dur) in enumerate(ev, 1):
        j = int(np.argmin(np.abs(t - te)))
        ax.plot(E[j], N[j], "x", color="#c02020", ms=11, mew=2.2, zorder=5)
        ax.annotate("#%d" % i, (E[j] + 0.3, N[j] - 0.55), fontsize=8, color="#c02020")
    ax.set_title("%s: truth/reference path and the %d rest2rest handovers (red X)" % (a.tag, len(ev)))
    ax.set_aspect("equal"); ax.grid(alpha=0.25); ax.legend(loc="upper right", fontsize=9)
    ax2 = fig.add_subplot(gs[1, :])
    ax2.plot(t, vt, "-", color="0.15", lw=0.9, label="truth |v|")
    ax2.plot(ts, vr, "-", color="#e08a1e", lw=0.8, alpha=0.8, label="reference |v|")
    for (te, _) in ev:
        ax2.axvline(te, color="#c02020", ls="--", lw=0.8, alpha=0.6)
    ax2.set_ylabel("speed [m/s]"); ax2.grid(alpha=0.25); ax2.legend(loc="upper right", fontsize=9)
    ax2.set_title("speed vs time (dashed = rest2rest commit)")
    # zoom: pick the event closest to a real corner
    pick = int(np.argmin([r[3] for r in rows])) if rows else 0
    te = rows[pick][1]; cc_idx = None
    if corners:
        j = int(np.argmin(np.abs(t - te)))
        cc = np.array(corners); kbest = int(np.argmin(np.hypot(cc[:, 0] - E[j], cc[:, 1] - N[j])))
        cc_idx = kbest
    ax3 = fig.add_subplot(gs[2, :])
    wsel = (t >= te - 4) & (t <= te + 4)
    ax3.plot(t[wsel], vt[wsel], "-", color="0.15", lw=1.2, label="truth |v|")
    wsel2 = (ts >= te - 4) & (ts <= te + 4)
    ax3.plot(ts[wsel2], vr[wsel2], "-", color="#e08a1e", lw=1.2, label="reference |v|")
    if corners and cc_idx is not None:
        c = corners[cc_idx]
        dt_ = np.hypot(E - c[0], N - c[1]); dr_ = np.hypot(ce - c[0], cn - c[1])
        ax3.plot(t[wsel], dt_[wsel], "--", color="#1f6fd0", lw=1.0, label="truth dist to corner")
        ax3.plot(ts[wsel2], dr_[wsel2], ":", color="#1f6fd0", lw=1.0, label="ref dist to corner")
    ax3.axvline(te, color="#c02020", ls="--", lw=1.2)
    ax3.set_xlabel("sim time [s]"); ax3.set_ylabel("m  /  m/s")
    ax3.set_title("zoom around rest2rest #%d (corner c%d): the reference stops at the corner, then replans from "
                  "the measured state" % (pick + 1, (cc_idx + 1) if cc_idx is not None else -1))
    ax3.grid(alpha=0.25); ax3.legend(loc="best", fontsize=9)
    out = a.out or os.path.join(ROOT, "docs", "evidence", "final", "%s_rest2rest.png" % a.tag)
    fig.savefig(out, dpi=110, bbox_inches="tight")
    print("written:", out)


if __name__ == "__main__":
    main()
