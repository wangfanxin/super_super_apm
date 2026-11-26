#!/usr/bin/env python3
"""flight_forensics.py -- offline forensics for ONE sortie: where does the tracking error come from,
did the corners stop / double back, what is the estimator's own error, and why does the altitude move.

    python3 scripts/flight_forensics.py --tag phase9f-01-01
    python3 scripts/flight_forensics.py --tag phase9f-01-01 --world sim/worlds/forest_hex12.yaml
    python3 scripts/flight_forensics.py --run .runs/missionA-xxx --no-figures

WHAT IT USES (all recorded by the bridge already):
  tap.csv        truth in W        (pn/pe/pd -> x=pe, y=pn, z=-pd; verified by correlating with fcu_*)
  setpoints.csv  cmd in W + the FCU's own estimate (fcu_*) in W + the raw estimator pose (lio_*) in L
  nav.csv        the estimator samples fed to the EKF (L) and the FCU estimate (W)
  lio.csv        the raw LIO odometry (L, 200 Hz)
  calib/calib.json  R_WL, t_WL to put the LIO pose into W

THE DECOMPOSITION (the reason this script exists)
  The controller sees the FCU's own estimate, not the truth.  So with everything in W at the same time:

      cmd - truth  =  (cmd - fcu)  +  (fcu - truth)
      tracking err    command-following    localisation err

  If (cmd - fcu) is small and (fcu - truth) is the whole error, the aircraft is following its reference and
  the deviation is a LOCALISATION problem; if it is the other way round it is a CONTROL/reference problem.
  The script reports both, plus the same split for the velocity.

CORNERS
  For each mission corner, a +-CORNER_WIN_S window around the truth's first pass within CORNER_NEAR_M:
  commanded speed-norm minimum (did the plan ask to stop?), truth speed-norm minimum and time spent below
  STOP_MPS (did the aircraft stop?), the path length / net-displacement ratio (did it wander / double back),
  and the total |heading change| of the truth inside the window.

ALTITUDE
  The command z is constant, so any truth altitude motion is either (a) a constant estimator/calibration
  datum offset or (b) real dynamics.  The script splits it: median(truth - cmd) is the datum, the remaining
  distribution is the dynamic part, and it also reports the calib t_WL z term (the datum candidate).

Outputs (default --outdir docs/evidence, prefix <tag>):
  <tag>-vnorm.png          speed 2-norm (truth vs cmd), horizontal norm, signed v_z
  <tag>-lio-truth.png      estimator (raw LIO and FCU) error against the truth + CDF
  <tag>-tracking-decomp.png  cmd-truth vs (cmd-fcu) vs (fcu-truth) + along/cross
  <tag>-z-t.png            altitude: cmd / truth / FCU / LIO, and the dynamic residual
  <tag>-corner-forensics.png  6 zoomed corner panels coloured by truth speed norm
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

C_TRUTH = "#38bdf8"
C_CMD = "#fb923c"
C_FCU = "#a3e635"
C_LIO = "#f472b6"
C_BAD = "#f87171"
FG = "#e5e7eb"
GRID = "#374151"


def _rows(path):
    with open(path, encoding="utf-8", errors="replace") as fh:
        return list(csv.DictReader(fh))


def _style(ax):
    ax.set_facecolor("#0f1116")
    ax.grid(True, color=GRID, ls=":", lw=0.6, alpha=0.7)
    ax.tick_params(colors="#9ca3af", labelsize=8)
    for s in ax.spines.values():
        s.set_color("#4b5563")


def _cdf(v):
    v = np.sort(np.asarray(v, float))
    return v, np.arange(1, v.size + 1) / v.size


def interp_vec(t, Y, tq):
    """linear interpolation of an (N,k) array at tq (columns interpolated independently)"""
    Y = np.asarray(Y, float)
    out = np.empty((len(tq), Y.shape[1]))
    for j in range(Y.shape[1]):
        out[:, j] = np.interp(tq, t, Y[:, j])
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="offline forensics for one sortie")
    ap.add_argument("--run", default="")
    ap.add_argument("--tag", default="")
    ap.add_argument("--world", default="")
    ap.add_argument("--outdir", default=os.path.join(ROOT, "docs", "evidence"))
    ap.add_argument("--no-figures", action="store_true")
    ap.add_argument("--corner-win-s", type=float, default=2.0,
                    help="pre-corner window (s); the post-corner window is --corner-post-s")
    ap.add_argument("--corner-post-s", type=float, default=3.0)
    ap.add_argument("--corner-near-m", type=float, default=0.9,
                    help="the truth must come this close to the corner for the pass to count")
    ap.add_argument("--stop-mps", type=float, default=0.25)
    a = ap.parse_args()
    if a.run:
        run = os.path.abspath(a.run)
    elif a.tag:
        run = _P.sortie(a.tag)
    else:
        print("--tag or --run required")
        return 2
    tag = os.path.basename(run).replace("missionA-", "")
    # evidence figures are named by the SORTIE (phase9f), not by the attempt directory (phase9f-01-01)
    short = tag[:-6] if tag.endswith("-01-01") else tag
    world_path = a.world or os.path.join(ROOT, "sim", "worlds", "forest_hex12.yaml")
    world = json.load(open(world_path)) if world_path.endswith(".json") else _load_yaml(world_path)
    corners_w = np.array([[float(c[0]), float(c[1])] for c in world["mission"]["corners"][:6]])

    # ---- truth (W) ---------------------------------------------------------------------------
    tap = _rows(os.path.join(run, "tap.csv"))
    t = np.array([float(r["t_sim_us"]) * 1e-6 for r in tap])
    truth_p = np.column_stack([[float(r["pe"]) for r in tap],
                               [float(r["pn"]) for r in tap],
                               [-float(r["pd"]) for r in tap]])
    truth_v = np.column_stack([[float(r["ve"]) for r in tap],
                               [float(r["vn"]) for r in tap],
                               [-float(r["vd"]) for r in tap]])
    o = np.argsort(t)
    t, truth_p, truth_v = t[o], truth_p[o], truth_v[o]

    # ---- command (W) + FCU estimate (W) + raw LIO pose (L) ------------------------------------
    sp = _rows(os.path.join(run, "setpoints.csv"))
    tc = np.array([float(r["t_sim_us"]) * 1e-6 for r in sp])
    cmd_p = np.column_stack([[float(r["cmd_x"]) for r in sp],
                             [float(r["cmd_y"]) for r in sp],
                             [float(r["cmd_z"]) for r in sp]])
    cmd_v = np.column_stack([[float(r["vel_x"]) for r in sp],
                             [float(r["vel_y"]) for r in sp],
                             [float(r["vel_z"]) for r in sp]])
    cmd_a = np.column_stack([[float(r["acc_x"]) for r in sp],
                             [float(r["acc_y"]) for r in sp],
                             [float(r["acc_z"]) for r in sp]])
    fcu_p = np.column_stack([[float(r["fcu_x"]) for r in sp],
                             [float(r["fcu_y"]) for r in sp],
                             [float(r["fcu_z"]) for r in sp]])
    fcu_v = np.column_stack([[float(r["fcu_vx"]) for r in sp],
                             [float(r["fcu_vy"]) for r in sp],
                             [float(r["fcu_vz"]) for r in sp]])
    lio_L = np.column_stack([[float(r["lio_x"]) for r in sp],
                             [float(r["lio_y"]) for r in sp],
                             [float(r["lio_z"]) for r in sp]])

    # ---- calib L -> W ------------------------------------------------------------------------
    calib_path = os.path.join(ROOT, "calib", "calib.json")
    R_WL = np.eye(3)
    t_WL = np.zeros(3)
    if os.path.isfile(calib_path):
        d = json.load(open(calib_path))
        R_WL = np.array(d["R_WL"], float)
        t_WL = np.array(d["t_WL"], float)
    lio_W = (R_WL @ lio_L.T).T + t_WL

    # ---- truth at command/nav times ----------------------------------------------------------
    T = lambda tq: interp_vec(t, truth_p, tq)       # noqa: E731
    V = lambda tq: interp_vec(t, truth_v, tq)       # noqa: E731
    tp = T(tc)
    # flight window: truth above 1.0 m, plus 0.5 s inside
    alt = truth_p[:, 2]
    fly = alt > 1.0
    t_fly = (t[fly][0] + 0.5, t[fly][-1] - 0.5) if fly.any() else (t[0], t[-1])
    win = (tc >= t_fly[0]) & (tc <= t_fly[1])

    # ---- best lag of the command against the truth (position) --------------------------------
    best = (9e9, 0.0)
    for lag in np.arange(-0.6, 0.6 + 1e-9, 0.02):
        e = np.linalg.norm(cmd_p - T(tc + lag), axis=1)[win]
        p90 = float(np.percentile(e, 90)) if e.size else 9e9
        if p90 < best[0]:
            best = (p90, float(lag))
    lag = best[1]
    tp_l = T(tc + lag)
    tv_l = V(tc + lag)

    # ---- the decomposition -------------------------------------------------------------------
    e_track = cmd_p - tp_l                       # what the command asked vs what happened
    e_cmd_est = cmd_p - fcu_p                    # did the aircraft follow the reference IT saw?
    e_loc_fcu = fcu_p - tp_l                     # localisation error of the FCU estimate
    v_track = cmd_v - tv_l
    v_cmd_est = cmd_v - fcu_v
    v_loc = fcu_v - tv_l
    # along/cross of the tracking error (along = command velocity direction)
    sp_cmd = np.hypot(cmd_v[:, 0], cmd_v[:, 1])
    u = np.zeros((len(tc), 2))
    good = sp_cmd > 0.15
    u[good] = cmd_v[good, :2] / sp_cmd[good, None]
    e2 = e_track[:, :2]
    along = np.sum(e2 * u, axis=1)
    cross = u[:, 0] * e2[:, 1] - u[:, 1] * e2[:, 0]

    def P(x, q=90):
        x = np.asarray(x, float)
        return float(np.percentile(np.abs(x[np.isfinite(x)]), q)) if np.isfinite(x).any() else float("nan")

    def bias(x):
        x = np.asarray(x, float)
        return float(np.median(x[np.isfinite(x)])) if np.isfinite(x).any() else float("nan")

    # ---- raw LIO vs truth at the lio samples -------------------------------------------------
    lio_t = np.array([float(r["t_us"]) * 1e-6 for r in _rows(os.path.join(run, "lio.csv"))])
    lio_tl = np.array([[float(r["x"]), float(r["y"]), float(r["z"])]
                       for r in _rows(os.path.join(run, "lio.csv"))])
    lio_all_W = (R_WL @ lio_tl.T).T + t_WL
    lw = (lio_t >= t_fly[0]) & (lio_t <= t_fly[1])
    e_lio = lio_all_W - T(lio_t)

    # ---- corners ----------------------------------------------------------------------------
    corner_rows = []
    for i, c in enumerate(corners_w, start=1):
        d = np.linalg.norm(truth_p[:, :2] - c, axis=1)
        near = np.where((d < a.corner_near_m) & fly)[0]
        if near.size == 0:
            corner_rows.append(dict(corner=i, found=False))
            continue
        k0 = near[0]
        t0 = t[k0]
        lo, hi = t0 - a.corner_win_s, t0 + a.corner_post_s
        ws = (tc >= lo) & (tc <= hi)
        wt = (t >= lo) & (t <= hi)
        sp_t = np.linalg.norm(truth_v[wt], axis=1)
        sp_c = np.linalg.norm(cmd_v[ws], axis=1)
        dt_t = float(np.median(np.diff(t[wt]))) if wt.sum() > 2 else 0.05
        dt_c = float(np.median(np.diff(tc[ws]))) if ws.sum() > 2 else 0.01
        # path length vs net displacement (wander / double-back factor) in the window
        pp = truth_p[wt]
        seg = np.linalg.norm(np.diff(pp, axis=0), axis=1)
        plen = float(seg.sum())
        net = float(np.linalg.norm(pp[-1] - pp[0]))
        # dwell: how long the truth stays within 1.5 m of the corner
        dwell = float(np.sum(np.linalg.norm(truth_p[wt][:, :2] - c, axis=1) < 1.5) * dt_t)
        # backwards motion along the OUTGOING leg direction (the owner's "往复")
        nxt = corners_w[(i) % len(corners_w)]
        d_exit = nxt - c
        d_exit = d_exit / max(np.linalg.norm(d_exit), 1e-9)
        # only the POST-pass part: before the pass the aircraft is still on the incoming side and its
        # projection on the outgoing direction is naturally negative (that is not a "double back").
        post = t[wt] >= t0
        along_exit = (truth_p[wt][post][:, :2] - c) @ d_exit
        back = float(min(0.0, along_exit.min())) if along_exit.size else 0.0
        # cumulative heading change of the truth velocity in the window
        vh = truth_v[wt][:, :2]
        nz = np.linalg.norm(vh, axis=1) > 0.15
        vh = vh[nz]
        if len(vh) > 2:
            ang = np.unwrap(np.arctan2(vh[:, 1], vh[:, 0]))
            turn = float(np.degrees(np.abs(np.diff(ang)).sum()))
        else:
            turn = float("nan")
        corner_rows.append(dict(
            corner=i, found=True, t=t0,
            cmd_min=float(sp_c.min()) if sp_c.size else float("nan"),
            truth_min=float(sp_t.min()) if sp_t.size else float("nan"),
            truth_stop_s=float(np.sum(sp_t < a.stop_mps) * dt_t) if sp_t.size > 2 else 0.0,
            cmd_stop_s=float(np.sum(sp_c < a.stop_mps) * dt_c) if sp_c.size > 2 else 0.0,
            dwell_s=dwell, wander=float(plen / max(net, 1e-6)), net=net, path=plen,
            back_m=back, turn_deg=turn,
        ))

    # ---- altitude ---------------------------------------------------------------------------
    zb = float(np.median((tp_l[:, 2] - cmd_p[:, 2])[win]))     # truth - cmd datum
    dz = (tp_l[:, 2] - cmd_p[:, 2])[win] - zb                    # dynamic residual
    fz_b = float(np.median((fcu_p[:, 2] - tp_l[:, 2])[win]))
    lz_b = float(np.median((lio_W[:, 2] - T(tc)[:, 2])[win]))
    acc_n = np.linalg.norm(cmd_a[:, :2], axis=1)[win]
    corr_acc_z = float(np.corrcoef(acc_n, dz)[0, 1]) if acc_n.size > 4 else float("nan")
    vz_at_cmd = interp_vec(t, truth_v[:, 2:3], tc[win])[:, 0]
    z_dyn_vs_vz = float(np.corrcoef(np.abs(vz_at_cmd), np.abs(dz))[0, 1]) if len(dz) > 4 else float("nan")

    # ---- report -----------------------------------------------------------------------------
    print("== %s forensics" % tag)
    print("  flight window %.1f..%.1f s (%.1f s) | best lag %+0.2f s" % (t_fly[0], t_fly[1], t_fly[1] - t_fly[0], lag))
    def three(x):
        x = np.asarray(x, float)
        x = x[np.isfinite(x)]
        return (float(np.median(x)), float(np.percentile(x, 90)), float(np.percentile(x, 99)))
    tr = three(np.linalg.norm(e_track, axis=1)[win])
    fo = three(np.linalg.norm(e_cmd_est, axis=1)[win])
    lo = three(np.linalg.norm(e_loc_fcu, axis=1)[win])
    print("  TRACKING   cmd-truth : p50 %.3f p90 %.3f p99 %.3f m  (along p90 %.3f, cross p90 %.3f)"
          % (tr[0], tr[1], tr[2], P(along[win]), P(cross[win])))
    print("  FOLLOW     cmd-fcu   : p50 %.3f p90 %.3f p99 %.3f m  <-- what the controller could see"
          % (fo[0], fo[1], fo[2]))
    print("  LOCALISE   fcu-truth : p50 %.3f p90 %.3f p99 %.3f m  <-- estimator error (FCU fusion)"
          % (lo[0], lo[1], lo[2]))
    li = three(np.linalg.norm(e_lio, axis=1)[lw])
    print("  RAW LIO    lio-truth : p50 %.3f p90 %.3f p99 %.3f m  (bias x/y/z %+.3f %+.3f %+.3f)"
          % (li[0], li[1], li[2], bias(e_lio[:, 0]), bias(e_lio[:, 1]), bias(e_lio[:, 2])))
    print("  VELOCITY   cmd-truth p90 %.3f | cmd-fcu p90 %.3f | fcu-truth p90 %.3f m/s"
          % (P(v_track, 90), P(v_cmd_est, 90), P(v_loc, 90)))
    print("  share of tracking-error ENERGY explained by localisation: %.0f %%"
          % (100.0 * np.sum(np.sum(e_loc_fcu ** 2, axis=1)[win]) / max(np.sum(np.sum(e_track ** 2, axis=1)[win]), 1e-12)))
    print("  ALTITUDE   truth-cmd datum %+0.3f m (calib t_WL z %+0.3f) | dynamic residual p50 %.3f p90 %.3f m"
          % (zb, t_WL[2], float(np.percentile(np.abs(dz), 50)), float(np.percentile(np.abs(dz), 90))))
    print("             fcu-truth z bias %+0.3f | lio-truth z bias %+0.3f | corr(|a_xy|, dz_res) %+.2f"
          % (fz_b, lz_b, corr_acc_z))
    print("  CORNERS")
    for r in corner_rows:
        if not r.get("found"):
            print("    c%-2d no pass" % r["corner"])
            continue
        print("    c%-2d t=%6.1f  cmd |v|min %.2f  truth |v|min %.2f  truth<%.2f: %.2f s  cmd<%.2f: %.2f s  "
              "dwell<1.5m %.2f s  path/net %.2f  back %.3f m  cum turn %.0f deg"
              % (r["corner"], r["t"], r["cmd_min"], r["truth_min"], a.stop_mps, r["truth_stop_s"],
                 a.stop_mps, r["cmd_stop_s"], r["dwell_s"], r["wander"], r["back_m"], r["turn_deg"]))

    if a.no_figures:
        return 0
    os.makedirs(a.outdir, exist_ok=True)
    tt = tc[win] - tc[win][0]

    # ---- figure 1: speed 2-norm --------------------------------------------------------------
    sp3_t = np.linalg.norm(tv_l, axis=1)
    sp3_c = np.linalg.norm(cmd_v, axis=1)
    spxy_t = np.hypot(tv_l[:, 0], tv_l[:, 1])
    spxy_c = np.hypot(cmd_v[:, 0], cmd_v[:, 1])
    fig, axs = plt.subplots(3, 1, figsize=(13, 8.2), sharex=True, dpi=150)
    fig.patch.set_facecolor("#0f1116")
    w = win
    for ax in axs:
        _style(ax)
    axs[0].plot(tt, sp3_t[w], "-", color=C_TRUTH, lw=1.0, label="truth |v|=sqrt(vx²+vy²+vz²)")
    axs[0].plot(tt, sp3_c[w], "--", color=C_CMD, lw=1.0, label="command |v| (2-norm)")
    axs[0].set_ylabel("|v| (m/s)")
    axs[0].legend(fontsize=8, framealpha=0.25, facecolor="#111827", edgecolor="#374151", labelcolor=FG)
    axs[1].plot(tt, spxy_t[w], "-", color=C_TRUTH, lw=1.0, label="truth |v_xy|")
    axs[1].plot(tt, spxy_c[w], "--", color=C_CMD, lw=1.0, label="command |v_xy|")
    axs[1].set_ylabel("|v_xy| (m/s)")
    axs[1].legend(fontsize=8, framealpha=0.25, facecolor="#111827", edgecolor="#374151", labelcolor=FG)
    axs[2].plot(tt, tv_l[w, 2], "-", color=C_TRUTH, lw=1.0, label="truth v_z")
    axs[2].plot(tt, cmd_v[w, 2], "--", color=C_CMD, lw=1.0, label="command v_z")
    axs[2].axhline(0.0, color="#6b7280", lw=0.6)
    axs[2].set_ylabel("v_z (m/s)")
    axs[2].set_xlabel("time (s, from flight window start)")
    axs[2].legend(fontsize=8, framealpha=0.25, facecolor="#111827", edgecolor="#374151", labelcolor=FG)
    fig.suptitle("%s  speed: 2-norm (top), horizontal (middle), vertical (bottom) — truth vs command"
                 % tag, color=FG, fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    p = os.path.join(a.outdir, short + "-vnorm.png")
    fig.savefig(p, facecolor=fig.get_facecolor())
    plt.close(fig)
    print("written:", p)

    # ---- figure 2: estimator error -----------------------------------------------------------
    tL = lio_t[lw] - t_fly[0]
    fig, axs = plt.subplots(2, 2, figsize=(13, 7.2), dpi=150)
    fig.patch.set_facecolor("#0f1116")
    for ax in axs.flat:
        _style(ax)
    ax = axs[0, 0]
    for j, lab in enumerate("xyz"):
        ax.plot(tt, e_loc_fcu[w, j], "-", color=[C_TRUTH, C_FCU, C_LIO][j], lw=0.8, label="FCU est - truth %s" % lab)
    ax.axhline(0, color="#6b7280", lw=0.6)
    ax.set_ylabel("error (m)")
    ax.set_title("FCU estimate error vs truth (W frame)", color=FG, fontsize=9, loc="left")
    ax.legend(fontsize=7, framealpha=0.25, facecolor="#111827", edgecolor="#374151", labelcolor=FG)
    ax = axs[0, 1]
    ax.plot(tt, np.linalg.norm(e_loc_fcu, axis=1)[w], "-", color=C_FCU, lw=0.8, label="|FCU - truth|")
    ax.plot(tL, np.linalg.norm(e_lio, axis=1)[lw], "-", color=C_LIO, lw=0.6, alpha=0.8, label="|LIO - truth|")
    ax.set_ylabel("|error| (m)")
    ax.set_title("estimator error magnitude", color=FG, fontsize=9, loc="left")
    ax.legend(fontsize=7, framealpha=0.25, facecolor="#111827", edgecolor="#374151", labelcolor=FG)
    ax = axs[1, 0]
    v, c = _cdf(np.linalg.norm(e_loc_fcu, axis=1)[w])
    ax.plot(v, 100 * c, color=C_FCU, lw=1.2, label="FCU - truth")
    v2, c2 = _cdf(np.linalg.norm(e_lio, axis=1)[lw])
    ax.plot(v2, 100 * c2, color=C_LIO, lw=1.0, label="LIO - truth")
    for q in (50, 90, 99):
        ax.axvline(np.percentile(v, q), color=C_FCU, ls=":", lw=0.7)
    ax.set_xlabel("|error| (m)")
    ax.set_ylabel("CDF (%)")
    ax.set_title("error CDF (dotted: FCU p50/p90/p99)", color=FG, fontsize=9, loc="left")
    ax.legend(fontsize=7, framealpha=0.25, facecolor="#111827", edgecolor="#374151", labelcolor=FG)
    ax = axs[1, 1]
    for j, lab in enumerate("xyz"):
        ax.hist(e_loc_fcu[w, j], bins=60, color=[C_TRUTH, C_FCU, C_LIO][j], alpha=0.55, label=lab)
    ax.set_xlabel("per-axis FCU error (m)")
    ax.set_title("per-axis error histogram", color=FG, fontsize=9, loc="left")
    ax.legend(fontsize=7, framealpha=0.25, facecolor="#111827", edgecolor="#374151", labelcolor=FG)
    fig.suptitle("%s  LIO / FCU estimate vs truth (x=W-x, y=W-y, z=altitude)" % tag, color=FG, fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    p = os.path.join(a.outdir, short + "-lio-truth.png")
    fig.savefig(p, facecolor=fig.get_facecolor())
    plt.close(fig)
    print("written:", p)

    # ---- figure 3: tracking decomposition ----------------------------------------------------
    fig, axs = plt.subplots(3, 1, figsize=(13, 8.2), sharex=True, dpi=150)
    fig.patch.set_facecolor("#0f1116")
    for ax in axs:
        _style(ax)
    axs[0].plot(tt, np.linalg.norm(e_track, axis=1)[w], "-", color=C_TRUTH, lw=0.9, label="|cmd - truth| (tracking)")
    axs[0].plot(tt, np.linalg.norm(e_cmd_est, axis=1)[w], "-", color=C_CMD, lw=0.9, label="|cmd - FCU est| (following)")
    axs[0].plot(tt, np.linalg.norm(e_loc_fcu, axis=1)[w], "-", color=C_FCU, lw=0.9, label="|FCU est - truth| (localisation)")
    axs[0].set_ylabel("|error| (m)")
    axs[0].legend(fontsize=7.5, framealpha=0.25, facecolor="#111827", edgecolor="#374151", labelcolor=FG)
    axs[0].set_title("decomposition: tracking = following + localisation", color=FG, fontsize=9, loc="left")
    axs[1].plot(tt, along[w], "-", color=C_TRUTH, lw=0.8, label="along-track (cmd - truth)")
    axs[1].plot(tt, cross[w], "-", color=C_LIO, lw=0.8, label="cross-track")
    axs[1].axhline(0, color="#6b7280", lw=0.6)
    axs[1].set_ylabel("m")
    axs[1].legend(fontsize=7.5, framealpha=0.25, facecolor="#111827", edgecolor="#374151", labelcolor=FG)
    axs[2].plot(tt, np.linalg.norm(v_track, axis=1)[w], "-", color=C_TRUTH, lw=0.9, label="|cmd v - truth v|")
    axs[2].plot(tt, np.linalg.norm(v_cmd_est, axis=1)[w], "-", color=C_CMD, lw=0.9, label="|cmd v - FCU v|")
    axs[2].plot(tt, np.linalg.norm(v_loc, axis=1)[w], "-", color=C_FCU, lw=0.9, label="|FCU v - truth v|")
    axs[2].set_ylabel("m/s")
    axs[2].set_xlabel("time (s, from flight window start)")
    axs[2].legend(fontsize=7.5, framealpha=0.25, facecolor="#111827", edgecolor="#374151", labelcolor=FG)
    fig.suptitle("%s  tracking-error decomposition (best lag %+0.2f s)" % (tag, lag), color=FG, fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    p = os.path.join(a.outdir, short + "-tracking-decomp.png")
    fig.savefig(p, facecolor=fig.get_facecolor())
    plt.close(fig)
    print("written:", p)

    # ---- figure 4: altitude -------------------------------------------------------------------
    fig, axs = plt.subplots(2, 1, figsize=(13, 6.2), sharex=True, dpi=150)
    fig.patch.set_facecolor("#0f1116")
    for ax in axs:
        _style(ax)
    axs[0].plot(tt, cmd_p[w, 2], "--", color=C_CMD, lw=1.1, label="command z")
    axs[0].plot(tt, tp_l[w, 2], "-", color=C_TRUTH, lw=0.8, label="truth altitude")
    axs[0].plot(tt, fcu_p[w, 2], "-", color=C_FCU, lw=0.7, alpha=0.9, label="FCU estimate z")
    _lw_fig = (lio_t >= tc[win][0]) & (lio_t <= tc[win][-1])
    axs[0].plot(lio_t[_lw_fig] - tc[win][0], lio_all_W[_lw_fig, 2], "-", color=C_LIO, lw=0.6, alpha=0.8,
                label="raw LIO z (L->W)")
    axs[0].set_ylabel("z (m)")
    axs[0].set_title("altitude: command is constant %.2f m; the datum offset is the estimator/calibration, "
                     "not a control failure" % np.median(cmd_p[w, 2]), color=FG, fontsize=9, loc="left")
    axs[0].legend(fontsize=7.5, framealpha=0.25, facecolor="#111827", edgecolor="#374151", labelcolor=FG)
    axs[1].plot(tt, (tp_l[:, 2] - cmd_p[:, 2])[w], "-", color=C_TRUTH, lw=0.8, label="truth - commanded")
    axs[1].axhline(zb, color=C_CMD, ls="--", lw=0.9, label="datum (median) %+0.3f m" % zb)
    axs[1].axhline(0, color="#6b7280", lw=0.6)
    axs[1].set_ylabel("m")
    axs[1].set_xlabel("time (s, from flight window start)")
    axs[1].legend(fontsize=7.5, framealpha=0.25, facecolor="#111827", edgecolor="#374151", labelcolor=FG)
    fig.suptitle("%s  altitude: cmd / truth / FCU / LIO" % tag, color=FG, fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    p = os.path.join(a.outdir, short + "-z-t.png")
    fig.savefig(p, facecolor=fig.get_facecolor())
    plt.close(fig)
    print("written:", p)

    # ---- figure 5: corner forensics -----------------------------------------------------------
    fig, axs = plt.subplots(2, 3, figsize=(14, 8), dpi=150)
    fig.patch.set_facecolor("#0f1116")
    for i, ax in enumerate(axs.flat):
        _style(ax)
        r = corner_rows[i]
        c = corners_w[i]
        if not r.get("found"):
            ax.set_title("corner %d: no pass" % (i + 1), color=FG, fontsize=9)
            continue
        t0 = r["t"]
        wt = (t >= t0 - a.corner_win_s) & (t <= t0 + a.corner_post_s)
        ws = (tc >= t0 - a.corner_win_s) & (tc <= t0 + a.corner_post_s)
        pp = truth_p[wt]
        sp = np.linalg.norm(truth_v[wt], axis=1)
        sc = ax.scatter(pp[:, 0], pp[:, 1], c=sp, s=6, cmap="viridis", vmin=0, vmax=max(1.6, float(sp.max())))
        ax.plot(cmd_p[ws, 0], cmd_p[ws, 1], "--", color=C_CMD, lw=1.0, label="command")
        k = int(np.argmin(sp))
        ax.plot(pp[k, 0], pp[k, 1], "o", color=C_BAD, ms=5, label="truth slowest %.2f m/s" % sp[k])
        ax.plot(c[0], c[1], "x", color="#fbbf24", ms=7, label="corner")
        ax.set_aspect("equal")
        ax.set_title("c%d  cmd|v|min %.2f  truth|v|min %.2f  dwell %.1fs  back %.2fm"
                     % (i + 1, r["cmd_min"], r["truth_min"], r["dwell_s"], r["back_m"]),
                     color=FG, fontsize=8.5, loc="left")
        if i == 0:
            ax.legend(fontsize=6.5, framealpha=0.25, facecolor="#111827", edgecolor="#374151", labelcolor=FG)
        cb = fig.colorbar(sc, ax=ax, fraction=0.046, pad=0.02)
        cb.ax.tick_params(colors="#9ca3af", labelsize=7)
        cb.set_label("truth |v| (m/s)", color="#9ca3af", fontsize=7)
    fig.suptitle("%s  corner forensics: truth path coloured by speed (was it a stop? a double-back?)"
                 % tag, color=FG, fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    p = os.path.join(a.outdir, short + "-corner-forensics.png")
    fig.savefig(p, facecolor=fig.get_facecolor())
    plt.close(fig)
    print("written:", p)
    return 0


def _load_yaml(path):
    import yaml  # noqa: PLC0415
    with open(path, encoding="utf-8") as fh:
        return yaml.safe_load(fh)


if __name__ == "__main__":
    raise SystemExit(main())
