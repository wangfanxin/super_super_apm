#!/usr/bin/env python3
"""quality_rows.py -- the quality judgements we now actually care about (owner, 2026-11-26).

The 15-row acceptance table was built to answer "did the mission happen at all".  Three things we learned
this session are NOT in it, and one of its rows (H2, a 380 Hz TAP-ERA floor against a 200 Hz SIM_STATE feed)
says nothing about a flight:

  Q1 MID-PATH LOW-SPEED   the owner's rule: any truth |v| < 0.5 m/s lasting >= 0.15 s more than 1.5 m from
                          every corner is a mid-path stall/loop.  Gate: ZERO such segments.
  Q2 BODY-RADIUS CLEARANCE W4 grades a point against the pillar box; the planner itself uses
                          robot_r = 0.10 m.  Gate: min(body-edge margin) >= -0.02 m (truth noise 2 mm,
                          so this is "the body circle did not enter the trunk").
  Q3 MID-LEG TRACKING     p90 of |cmd - truth|_xy for samples more than 1.5 m from a corner.
                          Gate: <= 0.25 m (the delivered arm measures 0.19-0.25 m).
  Q4 CORNER DYNAMICS      per corner: back-motion |back| <= 0.35 m and dwell within 1.5 m <= 3.5 s.
                          Gate: every corner must pass.

Usage: python3 scripts/quality_rows.py --tag phase27b-01-01 [--world ... --json out.json]
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
import _plot_obstacles as OBS  # noqa: E402

C_LOW = 0.5
CORNER_NEAR = 1.5
MIN_DUR = 0.15


def _rows(p):
    with open(p, encoding="utf-8", errors="replace") as fh:
        return list(csv.DictReader(fh))


def main() -> int:
    ap = argparse.ArgumentParser(description="quality rows the acceptance table lacks")
    ap.add_argument("--run", default="")
    ap.add_argument("--tag", default="")
    ap.add_argument("--world", default=os.path.join(ROOT, "sim", "worlds", "forest_hex12.yaml"))
    ap.add_argument("--json", default="")
    a = ap.parse_args()
    run = os.path.abspath(a.run) if a.run else _P.sortie(a.tag)
    tag = os.path.basename(run).replace("missionA-", "")
    import yaml
    w = yaml.safe_load(open(a.world, encoding="utf-8"))
    pil = w.get("pillars") or []
    zs = float((w.get("pillar_geometry") or {}).get("z_split", 2.1))
    ring = np.array([[float(c[0]), float(c[1])] for c in w["mission"]["corners"][:6]])
    robot_r, _infl, _ = OBS.geometry(root=ROOT)

    tap = _rows(os.path.join(run, "tap.csv"))
    t = np.array([float(x["t_sim_us"]) * 1e-6 for x in tap])
    E = np.array([float(x["pe"]) for x in tap])
    N = np.array([float(x["pn"]) for x in tap])
    U = -np.array([float(x["pd"]) for x in tap])
    vE = np.array([float(x["ve"]) for x in tap])
    vN = np.array([float(x["vn"]) for x in tap])
    o = np.argsort(t)
    t, E, N, U, vE, vN = t[o], E[o], N[o], U[o], vE[o], vN[o]
    fly = (U >= 1.0) & (U <= 4.0)
    spd = np.hypot(vE, vN)
    dcorner = np.min(np.linalg.norm(ring[None, :, :] - np.column_stack([E, N])[:, None, :], axis=2), axis=1)
    rpad = np.hypot(E, N)                      # launch/landing happen at the pad (origin)
    mid = fly & (dcorner > CORNER_NEAR) & (rpad > 1.5)

    # ---- Q1 mid-path low-speed segments
    low = mid & (spd < C_LOW)
    idx = np.where(low)[0]
    segs = []
    i = 0
    while i < len(idx):
        j = i
        while j + 1 < len(idx) and idx[j + 1] == idx[j] + 1:
            j += 1
        if t[idx[j]] - t[idx[i]] >= MIN_DUR:
            segs.append((float(t[idx[i]]), float(t[idx[j]] - t[idx[i]]),
                         float(E[idx[i]]), float(N[idx[i]])))
        i = j + 1
    q1_ok = len(segs) == 0

    # ---- Q2 body-radius clearance (box below z_split, robot circle radius robot_r)
    bm = OBS.body_margin(np.column_stack([E[fly], N[fly]]), pil, U[fly], robot_r, zs)
    q2_min = float(np.nanmin(bm))
    q2_ok = q2_min >= -0.02

    # ---- Q3 mid-leg tracking p90 (cmd vs truth, horizontal)
    sp = _rows(os.path.join(run, "setpoints.csv"))
    ts = np.array([float(x["t_sim_us"]) * 1e-6 for x in sp])
    cmd = np.column_stack([[float(x[k]) for x in sp] for k in ("cmd_x", "cmd_y")])
    Tp = np.column_stack([np.interp(ts, t, E), np.interp(ts, t, N)])
    dc = np.min(np.linalg.norm(ring[None, :, :] - Tp[:, None, :], axis=2), axis=1)
    i0 = int(np.argmax(fly))
    i1 = int(len(fly) - 1 - np.argmax(fly[::-1]))
    wsel = (ts >= t[i0]) & (ts <= t[i1]) & (dc > CORNER_NEAR)
    err = np.linalg.norm(cmd - Tp, axis=1)
    q3_p90 = float(np.percentile(err[wsel], 90)) if wsel.sum() > 20 else float("nan")
    q3_ok = np.isfinite(q3_p90) and q3_p90 <= 0.25

    # ---- Q4 per-corner dynamics (back motion + dwell), same definitions as flight_forensics
    q4_rows = []
    q4_ok_all = True
    for k, c in enumerate(ring, 1):
        d = np.linalg.norm(np.column_stack([E, N]) - c, axis=1)
        near = np.where((d < CORNER_NEAR) & fly)[0]
        if near.size == 0:
            q4_rows.append((k, False, "no pass"))
            q4_ok_all = False
            continue
        # ONE row per VISIT; definitions copied from flight_forensics.py (owner's "往复"):
        # back = most negative projection on the OUTGOING leg direction AFTER the pass.
        nxt = ring[k % len(ring)]
        u_out = (nxt - c) / max(np.linalg.norm(nxt - c), 1e-9)
        visits = []
        start = 0
        for j in range(1, len(near)):
            if t[near[j]] - t[near[j - 1]] > 2.0:
                visits.append(near[start:j]); start = j
        visits.append(near[start:])
        dets, ok = [], True
        for vi in visits:
            t0 = t[vi[0]]
            wt = (t >= t0 - 1.5) & (t <= t0 + 4.0) & fly
            dwell = float(np.sum(np.linalg.norm(np.column_stack([E, N])[wt] - c, axis=1) < 1.5)
                          * float(np.median(np.diff(t[wt])) if wt.sum() > 2 else 0.05))
            # the owner's judgement is "不要再在角点停-转": the truth's minimum speed inside the visit
            vmin = float(np.min(spd[wt])) if wt.sum() else 0.0
            vok = (dwell <= 3.5) and (vmin >= 0.40)
            ok = ok and vok
            dets.append("dwell %.2f vmin %.2f%s" % (dwell, vmin, "" if vok else "!"))
        q4_ok_all = q4_ok_all and ok
        q4_rows.append((k, ok, "; ".join(dets)))

    # ---- P1 planned-vs-executed path deviation (owner request 2026-11-27)
    #   "规划路径和实际执行轨迹的差，如果这个值大（不仅是 rmse，如果有尖峰），也说明飞行有问题"
    #   Time-aligned horizontal |truth - reference| plus SPIKES; geometric_h = nearest distance from the
    #   truth to the reference path (horizontal, level band only).  Gate: p90 <= 0.35 m AND no spike > 1.0 m.
    _sp_ts = np.array([float(x["t_sim_us"]) * 1e-6 for x in sp])
    _sp_cmd = np.column_stack([[float(x[k]) for x in sp] for k in ("cmd_x", "cmd_y")])
    _o2 = np.argsort(_sp_ts, kind="stable")
    _sp_ts, _sp_cmd = _sp_ts[_o2], _sp_cmd[_o2]
    _k2 = np.concatenate([[True], np.diff(_sp_ts) > 1e-9])
    _sp_ts, _sp_cmd = _sp_ts[_k2], _sp_cmd[_k2]
    _lvl = (U >= 1.0) & (U <= 4.0) & (_sp_ts[0] <= t) & (_sp_ts[-1] >= t)
    _tt = t[_lvl]
    _refE = np.interp(_tt, _sp_ts, _sp_cmd[:, 0])
    _refN = np.interp(_tt, _sp_ts, _sp_cmd[:, 1])
    _dh = np.hypot(E[_lvl] - _refE, N[_lvl] - _refN)
    p1_p90 = float(np.percentile(_dh, 90)) if _dh.size else float("nan")
    p1_max = float(_dh.max()) if _dh.size else float("nan")
    def _spk(val, thr):
        m = val > thr
        if not m.any():
            return 0, 0.0, 0.0
        ii = np.flatnonzero(m)
        segl = []
        i0 = 0
        while i0 < len(ii):
            j0 = i0
            while j0 + 1 < len(ii) and ii[j0 + 1] - ii[j0] == 1:
                j0 += 1
            segl.append((_tt[ii[i0]], _tt[ii[j0]], float(val[ii[i0]:ii[j0] + 1].max())))
            i0 = j0 + 1
        longest = max(segl, key=lambda s: s[1] - s[0])
        return len(segl), float(longest[1] - longest[0]), float(max(s[2] for s in segl))
    n05, dur05, peak05 = _spk(_dh, 0.5) if _dh.size else (0, 0.0, 0.0)
    n10, dur10, peak10 = _spk(_dh, 1.0) if _dh.size else (0, 0.0, 0.0)
    # geometric distance to the planned path (horizontal, level band)
    try:
        from scipy.spatial import cKDTree
        _ref2 = _sp_cmd[np.abs(np.interp(_sp_ts, t, U) - 2.0) < 0.9] if t.size > 2 else _sp_cmd
        _dh_geo = cKDTree(_ref2[::max(1, len(_ref2) // 4000)]).query(np.column_stack([E[_lvl], N[_lvl]]))[0]
        p1_geo_p90 = float(np.percentile(_dh_geo, 90))
        p1_geo_max = float(_dh_geo.max())
    except Exception:  # noqa: BLE001
        p1_geo_p90 = float("nan")
        p1_geo_max = float("nan")
    p1_ok = bool(np.isfinite(p1_p90) and p1_p90 <= 0.35 and n10 == 0)

    rows = [("Q1", "mid-path low-speed stall/loop", q1_ok,
             "%d segment(s); %s" % (len(segs), ", ".join("t=%.1f dur %.2f at (%.1f,%.1f)" % s for s in segs[:4]))),
            ("Q2", "body-radius clearance (robot_r %.2f)" % robot_r, q2_ok,
             "min body-edge margin %+.3f m (gate -0.020)" % q2_min),
            ("Q3", "mid-leg tracking p90", q3_ok,
             "p90 %.3f m (gate 0.25) over %d samples" % (q3_p90, int(wsel.sum()))),
            ("Q4", "corner dynamics (dwell<=3.5 s, vmin>=0.40 m/s)", q4_ok_all,
             "; ".join("c%d %s %s" % (k, "ok" if ok else "FAIL", det) for k, ok, det in q4_rows)),
            ("P1", "planned-vs-executed path deviation (p90<=0.35, no >1.0m spike)", p1_ok,
             "time-aligned h: p90 %.3f m, max %.3f m, spikes>0.5m: %d (longest %.2f s, peak %.2f), "
             "spikes>1.0m: %d%s | geometric_h p90 %.3f max %.3f"
             % (p1_p90, p1_max, n05, dur05, peak05, n10,
                "" if n10 == 0 else " (longest %.2f s, peak %.2f)" % (dur10, peak10),
                p1_geo_p90, p1_geo_max))]
    print("== %s quality rows (owner's judgements, 2026-11-26)" % tag)
    for key, title, ok, det in rows:
        print("  [%s] %-2s %-52s %s" % ("PASS" if ok else "FAIL", key, title, det))
    n_bad = sum(1 for _, _, ok, _ in rows if not ok)
    print("QUALITY: %s (%d/%d fail)" % ("PASS" if n_bad == 0 else "FAIL", n_bad, len(rows)))
    if a.json:
        json.dump({"tag": tag, "rows": [{"id": k, "title": t_, "ok": ok, "detail": det}
                                        for k, t_, ok, det in rows]}, open(a.json, "w"), indent=1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
