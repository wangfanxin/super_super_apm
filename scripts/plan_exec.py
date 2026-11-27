#!/usr/bin/env python3
"""plan_exec.py -- planned-vs-executed path deviation (owner request, 2026-11-27).

WHY THIS EXISTS.  W6 measures command-vs-truth ZERO-LAG correlation (a time shift) and Q3 measures
mid-leg |cmd-truth| p90.  Neither answers the owner's question: "how far is the aircraft from the path
that was PLANNED, and does that difference have PEAKS?".  A run can have a small RMSE and still be
broken by short spikes (e.g. phase83a oscillated 0.01<->8 m/s while the time-aligned p50 looked fine).

WHAT IT MEASURES.
  * P1-a  time-aligned deviation  |truth - reference| at the same sim time, horizontal and vertical.
  * P1-b  geometric deviation     distance from each truth sample to the nearest point of the reference
          POLYLINE (subsampled).  This removes the time-lag component, so it is the true "distance from
          the planned path".
  * SPIKES: contiguous windows where the deviation exceeds SPIKE_M (default 0.5 m) and HARD_M (1.0 m),
          with count, longest duration and peak value.  A flight with a spike > HARD_M is not a valid
          "speed" result even if p90 is small.

DATA (all recorded per sortie, no new instrumentation):
  tap.csv        truth: pn/pe/pd (NED; up = -pd)
  setpoints.csv  reference/command as it went to the wire: cmd_x/cmd_y/cmd_z (W: north/east/up)

USAGE
  python3 scripts/plan_exec.py --tag phase44b-01-01
  python3 scripts/plan_exec.py --tag phase83a-01-01 --fig docs/evidence/final/planexec-83a.png
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys

import numpy as np


def _root():
    d = os.path.dirname(os.path.abspath(__file__))
    return os.path.dirname(d)


ROOT = _root()
sys.path.insert(0, os.path.join(ROOT, "scripts"))


def _run_dir(tag):
    cand = [os.path.join(ROOT, ".runs", "missionA-%s" % tag),
            os.path.join(os.environ.get("LIO_LOGS", ""), "missionA-%s" % tag)]
    for c in cand:
        if c and os.path.isdir(c):
            return c
    raise SystemExit("run not found for tag %s" % tag)


def _rows(path):
    with open(path, encoding="utf-8", errors="replace") as fh:
        return list(csv.DictReader(fh))


def _mono(t, x):
    """sort by t and collapse duplicate timestamps with a running mean."""
    o = np.argsort(t, kind="stable")
    t, x = t[o], x[o]
    keep = np.ones(len(t), bool)
    keep[1:] = np.diff(t) > 1e-9
    return t[keep], x[keep]


def _spikes(t, val, thresh):
    """contiguous runs above thresh -> (n, longest_dur, longest_t0, peak)."""
    m = val > thresh
    if not m.any():
        return 0, 0.0, None, 0.0
    idx = np.flatnonzero(m)
    runs = []
    i = 0
    while i < len(idx):
        j = i
        while j + 1 < len(idx) and idx[j + 1] - idx[j] <= 1:
            j += 1
        runs.append((t[idx[i]], t[idx[j]], float(val[idx[i]:idx[j] + 1].max())))
        i = j + 1
    longest = max(runs, key=lambda r: r[1] - r[0])
    return len(runs), float(longest[1] - longest[0]), float(longest[0]), float(max(r[2] for r in runs))


def plan_exec(tag, spike_m=0.5, hard_m=1.0, alt=2.0, z_band=0.9, t_start=35.0, t_end_trim=12.0):
    run = _run_dir(tag)
    tap = _rows(os.path.join(run, "tap.csv"))
    sp = _rows(os.path.join(run, "setpoints.csv"))
    t = np.array([float(r["t_sim_us"]) * 1e-6 for r in tap])
    pn = np.array([float(r["pn"]) for r in tap])
    pe = np.array([float(r["pe"]) for r in tap])
    up = -np.array([float(r["pd"]) for r in tap])

    ts = np.array([float(r["t_sim_us"]) * 1e-6 for r in sp])
    # FRAME: setpoints cmd_* are recorded in W = (East, North, Up); tap is NED (E=pe, N=pn, U=-pd).
    ce = np.array([float(r["cmd_x"]) for r in sp])     # cmd_x = EAST
    cn = np.array([float(r["cmd_y"]) for r in sp])     # cmd_y = NORTH
    cu = np.array([float(r["cmd_z"]) for r in sp])     # cmd_z = UP
    # ONE common sort/dedup so the four columns stay paired
    _o = np.argsort(ts, kind="stable")
    ts, ce, cn, cu = ts[_o], ce[_o], cn[_o], cu[_o]
    _k = np.concatenate([[True], np.diff(ts) > 1e-9])
    ts, ce, cn, cu = ts[_k], ce[_k], cn[_k], cu[_k]

    ok = (t > t_start) & (t < t[-1] - t_end_trim) & (np.abs(up - alt) < z_band)
    ok &= (t >= ts[0]) & (t <= ts[-1])
    if ok.sum() < 50:
        return {"tag": tag, "error": "no usable samples"}
    tt = t[ok]
    re = np.interp(tt, ts, ce)
    rn = np.interp(tt, ts, cn)
    ru = np.interp(tt, ts, cu)
    dn = pn[ok] - rn
    de = pe[ok] - re
    dh = np.hypot(dn, de)
    dv = up[ok] - ru

    # geometric deviation: truth point -> nearest point of the reference path, HORIZONTAL ONLY (E,N) and
    # only reference samples taken inside the level-flight altitude band.  The 3-D version is polluted by
    # the vertical climb/landing leg and the z datum offset, which are not "path" errors.
    lvl = np.abs(cu - alt) < z_band
    ref = np.column_stack([ce, cn])[lvl]
    if len(ref) > 4:
        step = max(1, len(ref) // 4000)
        refs = ref[::step]
        try:
            from scipy.spatial import cKDTree
            d3, _ = cKDTree(refs).query(np.column_stack([pe[ok], pn[ok]]))
            d_geo = d3
        except Exception:  # noqa: BLE001
            d_geo = np.full(len(tt), np.nan)
    else:
        d_geo = np.full(len(tt), np.nan)

    out = {"tag": tag, "n": int(ok.sum())}
    for name, v in (("time_aligned_h", dh), ("time_aligned_v", np.abs(dv)), ("geometric_h", d_geo)):
        out[name] = {"p50": float(np.nanpercentile(v, 50)), "p90": float(np.nanpercentile(v, 90)),
                     "p99": float(np.nanpercentile(v, 99)), "max": float(np.nanmax(v))}
        n, dur, t0, peak = _spikes(tt, v, spike_m)
        out[name]["spikes>%.1fm" % spike_m] = {"n": n, "longest_s": dur, "t0": t0, "peak": peak}
        n2, dur2, t02, peak2 = _spikes(tt, v, hard_m)
        out[name]["spikes>%.1fm" % hard_m] = {"n": n2, "longest_s": dur2, "t0": t02, "peak": peak2}
    return out


def report(o):
    if "error" in o:
        print("%-14s ERROR %s" % (o["tag"], o["error"]))
        return
    print("== %s  (usable samples %d)" % (o["tag"], o["n"]))
    for name in ("time_aligned_h", "time_aligned_v", "geometric_h"):
        d = o[name]
        s = d["spikes>0.5m"]
        h = d["spikes>1.0m"]
        print("  %-15s p50 %.3f  p90 %.3f  p99 %.3f  max %.3f m | spikes>0.5m: %d (longest %.2f s @%.1f s, peak %.2f)"
              % (name, d["p50"], d["p90"], d["p99"], d["max"], s["n"], s["longest_s"],
                 -1 if s["t0"] is None else s["t0"], s["peak"]))
        if h["n"]:
            print("  %-15s     >1.0m: %d (longest %.2f s @%.1f s, peak %.2f)  <-- HARD SPIKE"
                  % ("", h["n"], h["longest_s"], -1 if h["t0"] is None else h["t0"], h["peak"]))


def figure(tag, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    run = _run_dir(tag)
    tap = _rows(os.path.join(run, "tap.csv"))
    sp = _rows(os.path.join(run, "setpoints.csv"))
    t = np.array([float(r["t_sim_us"]) * 1e-6 for r in tap])
    pn = np.array([float(r["pn"]) for r in tap]); pe = np.array([float(r["pe"]) for r in tap])
    up = -np.array([float(r["pd"]) for r in tap])
    ts = np.array([float(r["t_sim_us"]) * 1e-6 for r in sp])
    ce = np.array([float(r["cmd_x"]) for r in sp]); cn = np.array([float(r["cmd_y"]) for r in sp])
    cu = np.array([float(r["cmd_z"]) for r in sp])
    _o = np.argsort(ts, kind="stable"); ts, ce, cn, cu = ts[_o], ce[_o], cn[_o], cu[_o]
    _k = np.concatenate([[True], np.diff(ts) > 1e-9]); ts, ce, cn, cu = ts[_k], ce[_k], cn[_k], cu[_k]
    ok = (t > 35) & (t < t[-1] - 12) & (np.abs(up - 2.0) < 0.9) & (t >= ts[0]) & (t <= ts[-1])
    tt = t[ok]
    dn = pn[ok] - np.interp(tt, ts, cn); de = pe[ok] - np.interp(tt, ts, ce)
    dh = np.hypot(dn, de)
    from scipy.spatial import cKDTree
    _lvl = np.abs(cu - 2.0) < 0.9
    refs = np.column_stack([ce, cn])[_lvl][::max(1, int(_lvl.sum()) // 4000)]
    d3, _ = cKDTree(refs).query(np.column_stack([pe[ok], pn[ok]]))
    fig, ax = plt.subplots(2, 2, figsize=(15, 10))
    a = ax[0, 0]
    a.plot(ce, cn, "-", color="0.6", lw=1.0, label="planned/command path")
    sc = a.scatter(pe[ok], pn[ok], c=np.minimum(d3, 2.0), s=3, cmap="turbo", label="truth (colour=geo dev)")
    plt.colorbar(sc, ax=a, label="plan-vs-exec deviation, m")
    a.set_xlabel("east (m)"); a.set_ylabel("north (m)"); a.set_title("%s plan vs exec (topdown)" % tag)
    a.axis("equal"); a.legend(loc="best", fontsize=8)
    a = ax[0, 1]
    a.plot(tt, dh, lw=0.8, label="|truth-plan| horizontal")
    a.axhline(0.5, color="orange", ls="--", lw=0.8); a.axhline(1.0, color="red", ls="--", lw=0.8)
    a.set_xlabel("sim t (s)"); a.set_ylabel("m"); a.set_title("time-aligned deviation"); a.legend(fontsize=8)
    a = ax[1, 0]
    a.plot(tt, d3, lw=0.8, color="purple", label="geometric distance to planned path")
    a.axhline(0.5, color="orange", ls="--", lw=0.8); a.axhline(1.0, color="red", ls="--", lw=0.8)
    a.set_xlabel("sim t (s)"); a.set_ylabel("m"); a.set_title("geometric deviation (path distance)")
    a.legend(fontsize=8)
    a = ax[1, 1]
    a.hist(np.minimum(d3, 2.0), bins=60, color="steelblue")
    a.set_xlabel("geometric deviation (clipped at 2 m)"); a.set_ylabel("samples"); a.set_title("distribution")
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print("figure written:", out)


def main():
    ap = argparse.ArgumentParser(description="planned-vs-executed path deviation + spike detection")
    ap.add_argument("--tag", action="append", required=True, help="one or more run tags")
    ap.add_argument("--fig", default="")
    ap.add_argument("--json", default="")
    a = ap.parse_args()
    outs = [plan_exec(t) for t in a.tag]
    for o in outs:
        report(o)
    if a.json:
        json.dump(outs, open(a.json, "w"), indent=1)
    if a.fig:
        figure(a.tag[0], a.fig)


if __name__ == "__main__":
    main()
