#!/usr/bin/env python3
"""traj_error.py -- standard trajectory error of the delivered streams against the SIM_STATE truth.

Phase 5 (optional deliverable): the 15 acceptance rows stay exactly as they are; this tool adds the
metric a SLAM paper would report, as an EXTRA line -- aligned APE (absolute pose error) and RPE (relative
pose error, 1 s apart), for the estimator that was fed to the FCU (`lio`) and for the FCU's own estimate
(`fcu`):

    python3 scripts/traj_error.py --run .runs/missionA-xxx [--json out.json]
    python3 scripts/traj_error.py --run ... --no-align      # raw errors, no Umeyama alignment

WHY ALIGNMENT: the LIO lives in its own world (`lio_world`, L) whose origin and yaw are only known through
`calib/calib.json`; comparing raw coordinates against the truth mixes that calibration error with the
estimator error.  APE is therefore reported after an SE(3) alignment (Umeyama without scale) on the flight
window, and the aligned rotation/translation are printed so the calibration part stays visible.  RPE is
computed from the SAME alignment but on one-second relative motions, which removes the global placement.

TIME BASE (D4): the truth is read from `tap.csv`'s `t_sim_us` when present (the explicit simulated clock);
LIO stamps are already on it.  Old sorties without `t_sim_us` fall back to `t_us`, exactly like accept.py.

EXIT CODE: 0 when metrics were computed, 2 when the inputs are missing (a tool, not a gate).
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys

import numpy as np


def _read_csv(path):
    with open(path, encoding="utf-8", errors="replace") as fh:
        return list(csv.DictReader(fh))


def load_truth(run):
    rows = _read_csv(os.path.join(run, "tap.csv"))
    tcol = "t_sim_us" if rows and "t_sim_us" in rows[0] else "t_us"
    t = np.array([float(r[tcol]) * 1e-6 for r in rows])
    p = np.array([[float(r["pe"]), float(r["pn"]), -float(r["pd"])] for r in rows])
    o = np.argsort(t)
    t, p = t[o], p[o]
    keep = np.concatenate([[True], np.diff(t) > 0])
    return t[keep], p[keep]


def load_lio(run):
    rows = _read_csv(os.path.join(run, "lio.csv"))
    t = np.array([float(r["t_us"]) * 1e-6 for r in rows])
    x = np.array([float(r["x"]) for r in rows])
    y = np.array([float(r["y"]) for r in rows])
    z = np.array([float(r["z"]) for r in rows])
    # L -> W:  W = (E, N, U) = (-y_l, x_l, z_l)
    p = np.column_stack([-y, x, z])
    o = np.argsort(t)
    return t[o], p[o]


def load_fcu(run):
    rows = _read_csv(os.path.join(run, "setpoints.csv"))
    t = np.array([float(r["t_sim_us"]) * 1e-6 for r in rows])
    p = np.array([[float(r["fcu_x"]), float(r["fcu_y"]), float(r["fcu_z"])] for r in rows])
    o = np.argsort(t)
    return t[o], p[o]


def flight_window(run):
    """(t_start, t_end) of the FLIGHT (take-off .. land/done) on the simulated clock, or None."""
    path = os.path.join(run, "phase.csv")
    if not os.path.isfile(path):
        return None
    rows = _read_csv(path)
    if not rows:
        return None
    t0 = None
    t1 = None
    for r in rows:
        ph = (r.get("phase") or "").strip()
        ts = float(r.get("t_sim_us") or 0) * 1e-6
        if ph in ("takeoff", "preflight", "align", "arm") and t0 is None:
            t0 = ts
        elif ph in ("land", "done", "disarm"):
            t1 = ts
    if t0 is None:
        t0 = min(float(r.get("t_sim_us") or 0) * 1e-6 for r in rows)
    if t1 is None:
        t1 = max(float(r.get("t_sim_us") or 0) * 1e-6 for r in rows)
    return (t0, t1)


def umeyama(src, dst):
    """Rotation+translation (no scale) mapping src -> dst, least squares."""
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    cov = (dst - mu_d).T @ (src - mu_s) / max(1, len(src))
    u, s, vt = np.linalg.svd(cov)
    d = np.ones(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        d[-1] = -1.0
    R = u @ np.diag(d) @ vt
    t = mu_d - R @ mu_s
    return R, t


def rpe(est_t, est_p, tru_t, tru_p, delta=1.0, max_pairs=400):
    """Relative pose error over `delta` seconds: translation RMSE of one-step relative motions."""
    errs = []
    n = len(est_t)
    step = max(1, n // max_pairs)
    for i in range(0, n - 1, step):
        j0 = i
        # find the first sample at least delta later
        j1 = np.searchsorted(est_t, est_t[j0] + delta)
        if j1 >= n:
            break
        a = est_p[j0]
        b = est_p[j1]
        # truth at the SAME simulated instants
        pa = np.array([np.interp(est_t[j0], tru_t, tru_p[:, k]) for k in range(3)])
        pb = np.array([np.interp(est_t[j1], tru_t, tru_p[:, k]) for k in range(3)])
        rel_est = b - a
        rel_tru = pb - pa
        errs.append(np.linalg.norm(rel_est - rel_tru))
    if not errs:
        return None
    e = np.array(errs)
    return {"delta_s": delta, "n": int(e.size), "rmse": float(np.sqrt(np.mean(e ** 2))),
            "p50": float(np.percentile(e, 50)), "p95": float(np.percentile(e, 95)),
            "max": float(e.max())}


def metrics(run, align=True):
    tru_t, tru_p = load_truth(run)
    win = flight_window(run)
    out = {"window_sim_s": win, "streams": {}}
    for name, loader in (("lio", load_lio), ("fcu", load_fcu)):
        try:
            est_t, est_p = loader(run)
        except FileNotFoundError:
            out["streams"][name] = None
            continue
        if win is not None:
            keep = (est_t >= win[0]) & (est_t <= win[1]) & (tru_t[0] <= est_t) & (est_t <= tru_t[-1])
            est_t, est_p = est_t[keep], est_p[keep]
        if est_t.size < 20:
            out["streams"][name] = {"n": int(est_t.size), "note": "too few samples in the flight window"}
            continue
        tru_at = np.column_stack([np.interp(est_t, tru_t, tru_p[:, k]) for k in range(3)])
        rec = {"n": int(est_t.size)}
        if align:
            R, t = umeyama(est_p, tru_at)
            est_al = (R @ est_p.T).T + t
            rec["align"] = {
                "yaw_deg": float(math.degrees(math.atan2(R[1, 0], R[0, 0]))),
                "t_m": [float(v) for v in t],
            }
        else:
            R, t = np.eye(3), np.zeros(3)
            est_al = est_p
        e = np.linalg.norm(est_al - tru_at, axis=1)
        rec["ape"] = {"rmse": float(np.sqrt(np.mean(e ** 2))),
                      "p50": float(np.percentile(e, 50)),
                      "p95": float(np.percentile(e, 95)),
                      "max": float(e.max()),
                      "mean_signed": [float(v) for v in (est_al - tru_at).mean(axis=0)]}
        # RPE on the ALIGNED trajectory
        rec["rpe"] = rpe(est_t, est_al, tru_t, tru_p, delta=1.0)
        out["streams"][name] = rec
    return out


def fmt(m):
    if m is None:
        return "n/a"
    lines = []
    w = "%s" % ("" if m.get("window_sim_s") is None else
                "(flight window %.1f..%.1f s sim)" % m["window_sim_s"])
    lines.append("trajectory error vs SIM_STATE truth %s" % w)
    for name in ("lio", "fcu"):
        rec = m["streams"].get(name)
        if not rec:
            lines.append("  %-4s: n/a" % name)
            continue
        if "ape" not in rec:
            lines.append("  %-4s: %s" % (name, rec.get("note", "n/a")))
            continue
        ape = rec["ape"]
        lines.append("  %-4s APE  rmse %7.3f m  p50 %7.3f  p95 %7.3f  max %7.3f   (n=%d)"
                     % (name, ape["rmse"], ape["p50"], ape["p95"], ape["max"], rec["n"]))
        r = rec.get("rpe")
        if r:
            lines.append("  %-4s RPE 1s rmse %5.3f m  p95 %5.3f  (n=%d)"
                         % (name, r["rmse"], r["p95"], r["n"]))
        if "align" in rec:
            lines.append("  %-4s alignment: yaw %+.2f deg, t = [%+.3f %+.3f %+.3f] m"
                         % (name, rec["align"]["yaw_deg"], *rec["align"]["t_m"]))
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description="aligned APE/RPE of the delivered streams vs SIM_STATE truth")
    ap.add_argument("--run", required=True)
    ap.add_argument("--json", default="")
    ap.add_argument("--no-align", action="store_true", help="raw errors without SE(3) alignment")
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args()
    if not os.path.isfile(os.path.join(a.run, "tap.csv")):
        print("traj_error: no tap.csv in %s (needs a sortie)" % a.run)
        return 2
    m = metrics(a.run, align=not a.no_align)
    if not a.quiet:
        print(fmt(m))
    if a.json:
        with open(a.json, "w") as fh:
            json.dump(m, fh, indent=2)
            fh.write("\n")
    n_ok = sum(1 for v in m["streams"].values() if v and "ape" in v)
    return 0 if n_ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
