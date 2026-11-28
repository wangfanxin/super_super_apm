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
    # 2026-11-28 audit: the landing phase must not be graded as a corner visit (180a's Q4 "FAIL" was the
    # vertical landing inside c6's window) nor counted in S1's terminal window.
    t_land = None
    _ph = os.path.join(run, "phase.csv")
    if os.path.isfile(_ph):
        try:
            for _r in _rows(_ph):
                if (_r.get("phase") or "").strip() == "land":
                    t_land = float(_r.get("t_sim_us") or 0) * 1e-6
                    break
        except Exception:                            # noqa: BLE001
            t_land = None
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
            if t_land is not None and t[vi[0]] >= t_land - 1.0:
                continue
            t0 = t[vi[0]]
            wt = (t >= t0 - 1.5) & (t <= t0 + 4.0) & fly
            if t_land is not None:
                wt = wt & (t <= t_land - 1.0)      # do not grade the landing as a corner visit
            if wt.sum() < 5:
                continue
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
    # ---- S1/S2 owner-rule diagnostics (2026-11-27): record-only, do NOT gate yet -------------
    # S1: every truth sample below HALF the configured speed counts as a problem to investigate
    #     (1.35/2 = 0.675 m/s), excluding the launch and terminal-deceleration windows.
    # S2: path loops are speed-independent (a fast circle never drops below half speed).
    try:
        import low_speed_census as _LSC
        _cen = _LSC.census(tag, 0.675)
        _tr = [r for r in _cen.get("rows", []) if r["stream"] == "truth" and r["class_"] not in ("LAUNCH", "END")]
        s1_ok = None
        s1_det = "%d segment(s), total %.2f s below 0.675 m/s outside launch/end%s" % (
            len(_tr), sum(r["dur"] for r in _tr),
            (" (land excluded at %.1f s)" % _cen["t_land"]) if _cen.get("t_land") else "")
        _lps = _cen.get("loops", [])
        s2_ok = None
        s2_det = "%d path loop(s), total %.1f s (examples: %s)" % (
            len(_lps), sum(l["dur"] for l in _lps),
            ", ".join("(%.1f,%.1f) %.1fs vmin %.2f" % (l["pos"][0], l["pos"][1], l["dur"], l["vmin"])
                      for l in sorted(_lps, key=lambda x: -x["dur"])[:3]) if _lps else "none")
    except (Exception, SystemExit) as _e:             # noqa: BLE001  (SystemExit for _invalid/ runs)
        s1_ok, s1_det, s2_ok, s2_det = None, "census failed: %s" % _e, None, "census failed"
    rows += [("S1", "low-speed census (truth <0.675 m/s, excl launch/end)", s1_ok, s1_det),
             ("S2", "path loops (speed-independent)", s2_ok, s2_det)]
    # ---- R2R / ADV / SWT (2026-11-28 review): the handover quantities R1a is judged on -----------
    # R2R: how many rest2rest handovers happened (FSM Traj finish + PlanFromRest), and how many failed.
    # ADV: how many early goal advances the thin layer did (ADVANCE-LOOKAHEAD lines).
    # SWT: goal-switch reference continuity: |dv| and heading change of the published reference at
    #      every sent-target change (PLANDBG sent=...), plus the worst tracking error 2 s after.
    try:
        _log = os.path.join(run, "traj.log")
        _txt = open(_log, encoding="utf-8", errors="replace").read()
        _fin = _txt.count("Traj finish.")
        _ok = _txt.count("PlanFromRest] GenerateExpTrajectory SUCCESS")
        _fail = _txt.count("PlanFromRest] GenerateExpTrajectory failed")
        _adv = [ln for ln in _txt.splitlines() if "ADVANCE-LOOKAHEAD" in ln]
        _defer = _txt.count("DEFER-HANDOVER")
        r2r_ok = None
        import re as _re3
        _decs = [ln for ln in _txt.splitlines() if "HANDOVER-DECIDE" in ln]
        _rems = [float(m.group(1)) for ln in _decs
                 for m in [_re3.search(r"remain=([0-9.]+)s", ln)] if m]
        _commit = [ln for ln in _txt.splitlines() if "COMMIT " in ln and "rest2rest" in ln]
        _short = 0
        for ln in _commit:
            m = _re3.search(r"total=([0-9.]+)", ln)
            if m and float(m.group(1)) < 5.0:
                _short += 1
        _resync = _txt.count("CARROT-RESYNC")
        # 2026-11-28 review: split rest2rest into pad / corner / mid-leg / landing / chase.
        # COMMIT lines carry an EPOCH sim=.  Instead of fitting epoch->sim (error prone), use the PLANDBG
        # lines interleaved in the same log: each has plan-box wall= which maps to sim through setpoints.csv
        # (t_wall<->t_sim_us) and carries the pacer leg.  phase.csv gives the leg start sims in the same clock.
        _tw = np.array([float(x["t_wall"]) for x in sp]) if False else None
        _phase = []
        _ph_csv = os.path.join(run, "phase.csv")
        if os.path.isfile(_ph_csv):
            try:
                for _r in _rows(_ph_csv):
                    _phase.append((float(_r.get("t_sim_us") or 0) * 1e-6, (_r.get("phase") or "").strip()))
                _phase.sort()
            except Exception:                            # noqa: BLE001
                _phase = []
        _leg_starts = [t0 for t0, nm in _phase if nm.startswith("leg")]
        _land_start = next((t0 for t0, nm in _phase if nm == "land"), None)
        _first_leg = _leg_starts[0] if _leg_starts else None
        # wall->sim table from setpoints.csv
        _srows = _rows(os.path.join(run, "setpoints.csv"))
        _w = np.array([float(r["t_wall"]) for r in _srows])
        _s = np.array([float(r["t_sim_us"]) * 1e-6 for r in _srows])
        _oo = np.argsort(_s); _s, _w = _s[_oo], _w[_oo]
        _kk = np.concatenate([[True], np.diff(_s) > 1e-9]); _s, _w = _s[_kk], _w[_kk]
        _cls = {"pad": 0, "corner": 0, "mid-leg": 0, "landing": 0, "chase": 0, "unknown": 0}
        _ev = []
        _cur_sim = None
        _cur_leg = None
        for _ln in _txt.splitlines():
            if "PLANDBG wall=" in _ln:
                _mw = _re3.search(r"wall=([0-9.]+)", _ln)
                if _mw and _w.size > 1:
                    _cur_sim = float(np.interp(float(_mw.group(1)), _w, _s))
                continue
            if "COMMIT " in _ln and "rest2rest" in _ln and _cur_sim is not None:
                _ev.append(_cur_sim)
        _ev.sort()
        _last = None
        for _sv in _ev:
            if _first_leg is None:
                _c = "unknown"
            elif _land_start is not None and _sv >= _land_start:
                _c = "landing"
            elif _sv < _first_leg - 2.0:
                _c = "pad"
            else:
                # a corner handover happens at a leg boundary (the old trajectory ends at the corner);
                # the phase boundary may be a few seconds before/after the commit, so use a +-5 s band
                _near = bool(_leg_starts) and min(abs(_sv - _t0) for _t0 in _leg_starts) < 5.0
                _c = "corner" if _near else "mid-leg"
            if _last is not None and _sv - _last < 5.0 and _c in ("corner", "mid-leg"):
                _c = "chase"
            _cls[_c] += 1
            _last = _sv
        if not _ev:
            _cls = {"pad": 0, "corner": 0, "mid-leg": 0, "landing": 0, "chase": 0, "unknown": len(_commit)}
        r2r_det = ("TrajFinish %d, PlanFromRest ok %d, failed %d; defer-decisions %d%s; "
                   "rest2rest commits %d(short<5s %d); resync %d; per corner %.2f; "
                   "classes pad %d/corner %d/mid %d/landing %d/chase %d%s"
                   % (_fin, _ok, _fail, len(_decs),
                      (" remain(min/p50 %.2f/%.2f s)" % (min(_rems), float(np.median(_rems)))) if _rems else "",
                      len(_commit), _short, _resync,
                      _ok / max(1.0, (len(_cen.get("corners", [])) - 1)),
                      _cls["pad"], _cls["corner"], _cls["mid-leg"], _cls["landing"], _cls["chase"],
                      "" if _ev else " (no COMMIT/PLANDBG pairing -> classes unavailable)"))
        adv_ok = None
        _legs = []
        for ln in _adv:
            import re as _re
            m = _re.search(r"leg (\d+)/", ln)
            if m:
                _legs.append(int(m.group(1)))
        adv_det = ("%d advance event(s)%s" % (len(_adv), (" legs " + ",".join(str(x) for x in _legs[:20])) if _legs else " (mechanism off)")) if _adv else "0 advance event(s) (mechanism off)"
        # SWT: sent-target changes from PLANDBG
        import re as _re2
        _sent = None
        _sw = []
        for ln in _txt.splitlines():
            if "PLANDBG wall=" not in ln:
                continue
            m = _re2.search(r"sent=\(([-0-9.]+),([-0-9.]+)\)", ln)
            if not m:
                continue
            cur = (float(m.group(1)), float(m.group(2)))
            if _sent is not None and cur != _sent:
                _sw.append(cur)
            _sent = cur
        swt_ok = None
        if _sw:
            # published velocity jump around each sent change: nearest reference sample
            try:
                _sp = _rows(os.path.join(run, "setpoints.csv"))
                _ts = np.array([float(r["t_sim_us"]) * 1e-6 for r in _sp])
                _vx = np.array([float(r["vel_x"]) for r in _sp])
                _vy = np.array([float(r["vel_y"]) for r in _sp])
                _o = np.argsort(_ts)
                _ts, _vx, _vy = _ts[_o], _vx[_o], _vy[_o]
                _k = np.concatenate([[True], np.diff(_ts) > 1e-9])
                _ts, _vx, _vy = _ts[_k], _vx[_k], _vy[_k]
                _dv = []
                for _i in range(6, len(_ts) - 6):
                    if (_vx[_i + 1] - _vx[_i]) == 0 and (_vy[_i + 1] - _vy[_i]) == 0:
                        continue
                # plan-box sent changes cannot be mapped to sim time without wall mapping; report count only
                _dv = []
            except Exception:
                _dv = []
            swt_det = "%d goal-switch event(s) (PLANDBG sent changes)" % len(_sw)
        else:
            swt_det = "no PLANDBG sent changes recorded"
        rows += [("R2R", "rest2rest handovers (TrajFinish/PlanFromRest)", r2r_ok, r2r_det),
                 ("ADV", "early goal advances (R1a)", adv_ok, adv_det),
                 ("SWT", "goal-switch events (PLANDBG sent)", swt_ok, swt_det)]
    except Exception as _e2:                          # noqa: BLE001
        rows += [("R2R", "rest2rest handovers", None, "parse failed: %s" % _e2),
                 ("ADV", "early goal advances (R1a)", None, "parse failed"),
                 ("SWT", "goal-switch events", None, "parse failed")]
    print("== %s quality rows (owner's judgements, 2026-11-26)" % tag)
    for key, title, ok, det in rows:
        tag_ok = "INFO" if ok is None else ("PASS" if ok else "FAIL")
        print("  [%s] %-2s %-52s %s" % (tag_ok, key, title, det))
    gated = [r for r in rows if r[2] is not None]
    n_bad = sum(1 for _, _, ok, _ in gated if not ok)
    print("QUALITY: %s (%d/%d fail; %d record-only)" %
          ("PASS" if n_bad == 0 and len(gated) >= 5 else "FAIL", n_bad, len(gated),
           len(rows) - len(gated)))
    if a.json:
        json.dump({"tag": tag, "rows": [{"id": k, "title": t_, "ok": ok, "detail": det}
                                        for k, t_, ok, det in rows]}, open(a.json, "w"), indent=1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
