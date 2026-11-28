#!/usr/bin/env python3
"""plot_velocity_tracking.py -- position vs VELOCITY tracking of one sortie.

Position error can look small while the velocity VECTOR lags: the PSC closes the position loop eventually, but a
direction error at a corner is real (the aircraft rounds/cuts the turn).  This tool compares
  * truth velocity (tap vn/ve) with the SUPER reference velocity field (traj.csv.cmd vx_l/vy_l, L->W),
  * position error truth vs setpoints,
and splits both by distance to the mission corners.

USAGE: python3 scripts/plot_velocity_tracking.py --tag phase180d-01-01
"""
import argparse, csv, json, os, sys
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import low_speed_census as L


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="phase180d-01-01")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    run = L.run_dir(a.tag)
    d = L.load(a.tag)
    t, E, N = d["t"], d["E"], d["N"]
    tap = list(csv.DictReader(open(os.path.join(run, "tap.csv"))))
    vt = np.column_stack([[float(r["ve"]) for r in tap], [float(r["vn"]) for r in tap]])
    tr = list(csv.DictReader(open(os.path.join(run, "traj.csv.cmd.csv"))))
    tt = np.array([float(r["t_us"]) * 1e-6 for r in tr])
    vraw = np.column_stack([[float(r["vx_l"]) for r in tr], [float(r["vy_l"]) for r in tr]])
    cal = json.load(open(os.path.join(ROOT, "calib", "calib.json")))
    R = np.array(cal["R_WL"])
    vraw = vraw @ R[:2, :2].T
    o = np.argsort(tt); tt, vraw = tt[o], vraw[o]
    k = np.concatenate([[True], np.diff(tt) > 1e-9]); tt, vraw = tt[k], vraw[k]
    te = tt - tt[0]
    # align SUPER epoch clock to sim by matching the speed profile with the truth's
    vtn = np.hypot(vt[:, 0], vt[:, 1]); vrn = np.hypot(vraw[:, 0], vraw[:, 1])
    best = (1e9, 0.0)
    for s0 in np.arange(0.0, 60.0, 0.02):
        vv = np.interp(t, te + s0, vrn, left=np.nan, right=np.nan)
        m = np.isfinite(vv) & (t > 35) & (t < 140)
        if m.sum() < 500:
            continue
        e = np.nanmean(abs(vv[m] - vtn[m]))
        if e < best[0]:
            best = (e, s0)
    s0 = best[1]
    vref = np.column_stack([np.interp(t, te + s0, vraw[:, 0], left=0, right=0),
                            np.interp(t, te + s0, vraw[:, 1], left=0, right=0)])
    n = max(3, int(0.15 / np.median(np.diff(t))) | 1); ker = np.ones(n) / n
    sm = lambda v: np.column_stack([np.convolve(v[:, i], ker, "same") for i in range(2)])
    vs, vrs = sm(vt), sm(vref)
    fly = (t > 35) & (t < 140)
    dv = np.hypot(vrs[:, 0] - vs[:, 0], vrs[:, 1] - vs[:, 1])
    cross = vrs[:, 0] * vs[:, 1] - vrs[:, 1] * vs[:, 0]
    dot = vrs[:, 0] * vs[:, 0] + vrs[:, 1] * vs[:, 1]
    ang = np.abs(((np.degrees(np.arctan2(cross, dot)) + 180) % 360) - 180)
    sp = list(csv.DictReader(open(os.path.join(run, "setpoints.csv"))))
    ts = np.array([float(r["t_sim_us"]) * 1e-6 for r in sp])
    cE = np.array([float(r["cmd_x"]) for r in sp]); cN = np.array([float(r["cmd_y"]) for r in sp])
    o = np.argsort(ts); ts, cE, cN = ts[o], cE[o], cN[o]
    perr = np.hypot(np.interp(t, ts, cE) - E, np.interp(t, ts, cN) - N)
    corners = np.array(d["corners"]); dc = np.full(len(t), 1e9)
    for c in corners:
        dc = np.minimum(dc, np.hypot(E - c[0], N - c[1]))
    near = (dc < 1.5) & fly; mid = (dc >= 3.0) & fly
    print("== %s: velocity vs position tracking" % a.tag)
    for name, m in [("ALL", fly), ("near-corner(<1.5m)", near), ("mid-leg(>3m)", mid)]:
        print("  %-20s |dv| p50 %.3f p90 %.3f m/s | dir err p50 %.1f p90 %.1f deg | pos err p50 %.3f p90 %.3f m"
              % (name, np.median(dv[m]), np.percentile(dv[m], 90), np.median(ang[m]),
                 np.percentile(ang[m], 90), np.median(perr[m]), np.percentile(perr[m], 90)))
    speed_bias = np.mean((np.hypot(vrs[:, 0], vrs[:, 1]) - np.hypot(vs[:, 0], vs[:, 1]))[fly])
    print("  speed p50 ref %.2f truth %.2f m/s; bias(ref-truth) %+.3f m/s"
          % (np.median(np.hypot(vrs[fly, 0], vrs[fly, 1])), np.median(np.hypot(vs[fly, 0], vs[fly, 1])), speed_bias))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(3, 2, figsize=(15, 10), gridspec_kw=dict(hspace=0.35, wspace=0.22))
    ax = axs[0, 0]
    ax.plot(t, np.hypot(vs[:, 0], vs[:, 1]), "-", color="0.15", lw=1.0, label="truth |v|")
    ax.plot(t, np.hypot(vrs[:, 0], vrs[:, 1]), "-", color="#e08a1e", lw=1.0, label="reference |v|")
    ax.set_title("speed"); ax.grid(alpha=0.25); ax.legend(fontsize=8)
    ax = axs[0, 1]
    ax.plot(t, dv, "-", color="#c02020", lw=0.8, label="|v_ref - v_truth|")
    ax.plot(t, perr, "-", color="#1f6fd0", lw=0.8, label="|p_ref - p_truth|")
    ax.set_title("velocity vector error vs position error"); ax.grid(alpha=0.25); ax.legend(fontsize=8)
    ax = axs[1, 0]
    ax.plot(t, ang, "-", color="#7a30c0", lw=0.8)
    ax.set_title("velocity direction error [deg]"); ax.grid(alpha=0.25)
    ax = axs[1, 1]
    ax.plot(t, dc, "-", color="0.4", lw=0.8)
    ax.set_title("distance to nearest corner [m]"); ax.grid(alpha=0.25)
    ax = axs[2, 0]
    for name, m, col in [("near-corner", near, "#c02020"), ("mid-leg", mid, "#1f6fd0")]:
        ax.hist(dv[m], bins=np.arange(0, 1.2, 0.04), density=True, alpha=0.5, label=name, color=col)
    ax.set_title("|dv| distribution"); ax.legend(fontsize=8); ax.grid(alpha=0.25)
    ax = axs[2, 1]
    for name, m, col in [("near-corner", near, "#c02020"), ("mid-leg", mid, "#1f6fd0")]:
        ax.hist(ang[m], bins=np.arange(0, 90, 3), density=True, alpha=0.5, label=name, color=col)
    ax.set_title("direction error distribution [deg]"); ax.legend(fontsize=8); ax.grid(alpha=0.25)
    fig.suptitle("%s: position tracking is tight but the velocity VECTOR lags (worst near corners)" % a.tag)
    out = a.out or os.path.join(ROOT, "docs", "evidence", "final", "%s_velocity-tracking.png" % a.tag)
    fig.savefig(out, dpi=110, bbox_inches="tight")
    print("written:", out)


if __name__ == "__main__":
    main()
