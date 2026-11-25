#!/usr/bin/env python3
"""analyze_loops.py -- why does the aircraft sometimes circle while it tracks the reference?

Every flight is mostly a clean path-follow, but a few places show a small LOOP (a full-ish turn with little
net displacement).  This tool finds those places in the TRUTH, asks whether the PLAN was turning there too
(planner) or not (tracking/overshoot), and lines the events up with the planner's own log (SUPER's
`Opt failed` / `ReplanOnce` lines and the waypoint/trajectory switches).

    python3 scripts/analyze_loops.py --tag phase4p-01-01
    python3 scripts/analyze_loops.py --run .runs/missionA-xxx [--json out.json]

Definitions used here (all from the recording, no thresholds come from the controller):
  * heading psi(t) = atan2(dN, dE) of the truth, unwrapped;
  * a LOOP = a merged run of samples where the truth turns >= 300 deg inside a <= 8 s window AND the net
    displacement over that window is < 3 m (a corner turn is a rapid turn but the aircraft still translates;
    a loop comes back);
  * "plan also loops" = the command stream turns >= 200 deg in the same window;
  * along/cross-track error: e = truth - command at the same simulated instant, decomposed on the command's
    own velocity direction (along = ahead/behind the moving reference, cross = lateral).
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import _paths as _P  # noqa: E402


def _rows(path):
    with open(path, encoding="utf-8", errors="replace") as fh:
        return list(csv.DictReader(fh))


def load(run):
    rows = _rows(os.path.join(run, "tap.csv"))
    tcol = "t_sim_us" if rows and "t_sim_us" in rows[0] else "t_us"
    t = np.array([float(r[tcol]) * 1e-6 for r in rows])
    E = np.array([float(r["pe"]) for r in rows])
    N = np.array([float(r["pn"]) for r in rows])
    U = np.array([-float(r["pd"]) for r in rows])
    vE = np.array([float(r["ve"]) for r in rows])
    vN = np.array([float(r["vn"]) for r in rows])
    o = np.argsort(t)
    return t[o], np.column_stack([E[o], N[o], U[o]]), np.column_stack([vE[o], vN[o]])


def load_cmd(run):
    rows = _rows(os.path.join(run, "setpoints.csv"))
    t = np.array([float(r["t_sim_us"]) * 1e-6 for r in rows])
    p = np.column_stack([np.array([float(r["cmd_x"]) for r in rows]),
                         np.array([float(r["cmd_y"]) for r in rows]),
                         np.array([float(r["cmd_z"]) for r in rows])])
    v = np.column_stack([np.array([float(r["vel_x"]) for r in rows]),
                         np.array([float(r["vel_y"]) for r in rows]),
                         np.array([float(r["vel_z"]) for r in rows])])
    o = np.argsort(t)
    return t[o], p[o], v[o]


def load_traj_ids(run):
    """/plan/cmd trajectory_id history, when the bag was finalised into traj.csv.cmd.csv."""
    path = os.path.join(run, "traj.csv.cmd.csv")
    if not os.path.isfile(path):
        path = os.path.join(run, "derived", "traj.csv.cmd.csv")
    if not os.path.isfile(path):
        return None
    rows = _rows(path)
    t = np.array([float(r["t_us"]) * 1e-6 for r in rows])
    ids = np.array([int(float(r["trajectory_id"])) for r in rows])
    o = np.argsort(t)
    return t[o], ids[o]


def unwrap_heading(P):
    d = np.diff(P[:, :2], axis=0)
    seg = np.hypot(d[:, 0], d[:, 1])
    ok = seg > 1e-4
    psi = np.full(len(P), np.nan)
    psi[1:][ok] = np.arctan2(d[ok, 1], d[ok, 0])
    # forward-fill the nan gaps then unwrap
    idx = np.where(np.isfinite(psi))[0]
    if idx.size < 2:
        return np.zeros(len(P)), seg
    psi = np.interp(np.arange(len(P)), idx, psi[idx])
    return np.unwrap(psi), seg


def _smooth_resample(t, P, fs=10.0):
    """Uniform 10 Hz path with a 3-sample moving average -- loop detection on raw 200 Hz data is dominated
    by heading jitter at low speed (MEASURED: max |omega| 190 rad/s from position noise alone)."""
    tt = np.arange(t[0], t[-1], 1.0 / fs)
    Q = np.column_stack([np.interp(tt, t, P[:, k]) for k in range(P.shape[1])])
    if len(Q) >= 3:
        Q[1:-1] = (Q[:-2] + Q[1:-1] + Q[2:]) / 3.0
    return tt, Q


def detect_loops(t, P, closed_m=0.60, lap_s=(1.0, 6.0), turn_deg=300.0):
    """A LOOP = the path comes back within `closed_m` of an earlier point after 1-6 s, and it turned at
    least `turn_deg` in between.  That is "it went round", not just "it turned a corner" (a corner does not
    bring you back to your own track)."""
    tt, Q = _smooth_resample(t, P, 10.0)
    psi, _ = unwrap_heading(Q)
    n = len(tt)
    i_min = max(1, int(round(lap_s[0] * 10)))
    i_max = int(round(lap_s[1] * 10))
    cand = []
    last_end = -1.0
    for i in range(n):
        if tt[i] <= last_end:
            continue
        for j in range(i + i_min, min(n, i + i_max + 1)):
            if np.hypot(*(Q[j, :2] - Q[i, :2])) < closed_m:
                turn = abs(psi[j] - psi[i]) * 180.0 / math.pi
                if turn >= turn_deg:
                    cand.append({"t0": float(tt[i]), "t1": float(tt[j]), "turn_deg": float(turn)})
                    last_end = float(tt[j])
                break
    # merge events that overlap
    merged = []
    for c in cand:
        if merged and c["t0"] <= merged[-1]["t1"] + 0.3:
            merged[-1]["t1"] = max(merged[-1]["t1"], c["t1"])
            merged[-1]["turn_deg"] = max(merged[-1]["turn_deg"], c["turn_deg"])
        else:
            merged.append(dict(c))
    out = []
    for e in merged:
        i0 = int(np.searchsorted(t, e["t0"]))
        i1 = int(np.searchsorted(t, e["t1"]))
        if i1 - i0 < 10:
            continue
        e["i0"], e["i1"] = i0, i1
        e["dur_s"] = e["t1"] - e["t0"]
        e["net_m"] = float(np.hypot(*(P[i1, :2] - P[i0, :2])))
        out.append(e)
    return out


def classify(ev, t, P, V, tc, pc, vc):
    """plan-loops? along/cross-track errors, speed comparison."""
    m = (t >= ev["t0"]) & (t <= ev["t1"])
    if m.sum() < 5:
        return ev
    tt_, QQ = _smooth_resample(t[m], P[m], 10.0)
    psi_t, _ = unwrap_heading(QQ)
    turn_truth = abs(psi_t[-1] - psi_t[0]) * 180.0 / math.pi
    mc = (tc >= ev["t0"]) & (tc <= ev["t1"])
    turn_cmd = 0.0
    if mc.sum() >= 5:
        tc_, CC = _smooth_resample(tc[mc], pc[mc], 10.0)
        psi_c, _ = unwrap_heading(CC)
        turn_cmd = abs(psi_c[-1] - psi_c[0]) * 180.0 / math.pi
    # errors relative to the moving reference, at truth sample times
    cmd_at = np.column_stack([np.interp(t[m], tc, pc[:, k]) for k in range(3)])
    vel_at = np.column_stack([np.interp(t[m], tc, vc[:, k]) for k in range(2)])
    err = P[m] - cmd_at
    vnorm = np.linalg.norm(vel_at, axis=1)
    good = vnorm > 0.3
    along = cross = np.zeros(0)
    if good.sum() > 3:
        u = vel_at[good] / vnorm[good, None]
        e2 = err[good, :2]
        nvec = np.column_stack([-u[:, 1], u[:, 0]])
        along = np.sum(e2 * u, axis=1)
        cross = np.sum(e2 * nvec, axis=1)
    sp_truth = np.linalg.norm(V[m, :2], axis=1)
    ev.update({
        "turn_cmd_deg": float(turn_cmd),
        "plan_loops": bool(turn_cmd >= 200.0),
        "along_mean_m": float(np.mean(along)) if along.size else None,
        "along_p90_abs_m": float(np.percentile(np.abs(along), 90)) if along.size else None,
        "cross_mean_m": float(np.mean(cross)) if cross.size else None,
        "cross_p90_abs_m": float(np.percentile(np.abs(cross), 90)) if cross.size else None,
        "speed_max_mps": float(np.nanmax(sp_truth)) if sp_truth.size else None,
        "cmd_speed_max_mps": float(np.linalg.norm(vel_at, axis=1).max()) if mc.sum() else None,
        "truth_speed_mean_mps": float(np.nanmean(sp_truth)) if sp_truth.size else None,
    })
    return ev


def replan_log_events(run):
    """SUPER's own log lines: Opt failed / ReplanOnce succeed / state changes, with their sim times.

    The log's WARN lines carry a WALL epoch; setpoints.csv carries (t_wall, t_sim_us) pairs, so the wall ->
    sim mapping is measured per sortie (no assumption about clocks)."""
    path = os.path.join(run, "traj.log")
    if not os.path.isfile(path):
        return None
    rows = _rows(os.path.join(run, "setpoints.csv"))
    tw = np.array([float(r["t_wall"]) for r in rows])
    ts = np.array([float(r["t_sim_us"]) * 1e-6 for r in rows])
    o = np.argsort(tw)
    tw, ts = tw[o], ts[o]

    def to_sim(epoch):
        return float(np.interp(float(epoch), tw, ts))

    out = {"opt_failed": [], "replan_succeed": [], "states": [], "goals": []}
    n_prev_state = None
    for line in open(path, encoding="utf-8", errors="replace"):
        m = re.search(r"\[(\d+\.\d+)\] \[fsm_node\]:\s*-- \[(?:BackOpt|ExpOpt)\] Opt failed", line)
        if m:
            out["opt_failed"].append(to_sim(m.group(1)))
            continue
        m = re.search(r"\[(\d+\.\d+)\] \[fsm_node\]:\s*-- \[Fsm\] ReplanOnce succeed", line)
        if m:
            out["replan_succeed"].append(to_sim(m.group(1)))
            continue
        m = re.search(r"\[Fsm ([\d.]+)\] Current state: (\w+)", line)
        if m:
            st = m.group(2)
            if st != n_prev_state:
                out["states"].append({"t": float(m.group(1)), "state": st})
                n_prev_state = st
            continue
        m = re.search(r"GOAL (\d+)/(\d+) .*?(?:\[resend\])?$", line)
        if m:
            out["goals"].append({"t": None, "index": int(m.group(1)), "total": int(m.group(2))})
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="loop / overshoot events vs the plan and the planner log")
    ap.add_argument("--run", default="")
    ap.add_argument("--tag", default="")
    ap.add_argument("--json", default="")
    a = ap.parse_args()
    if a.run:
        run = os.path.abspath(a.run)
    elif a.tag:
        run = _P.sortie(a.tag if not a.tag.startswith("missionA-") else a.tag[len("missionA-"):])
    else:
        cands = sorted([p for p in os.listdir(_P.LOGS) if p.startswith("missionA-")],
                       key=lambda p: os.path.getmtime(os.path.join(_P.LOGS, p)), reverse=True)
        run = os.path.join(_P.LOGS, cands[0])
    tag = os.path.basename(run).replace("missionA-", "")
    t, P, V = load(run)
    tc, pc, vc = load_cmd(run)
    # flight window only (above 1 m), same rule as the top view
    m = P[:, 2] >= 1.0
    t, P, V = t[m], P[m], V[m]
    events = [classify(e, t, P, V, tc, pc, vc) for e in detect_loops(t, P)]
    log = replan_log_events(run)
    ids = load_traj_ids(run)
    if ids is not None and events:
        tid_t, tid_v = ids
        for e in events:
            sel = (tid_t >= e["t0"]) & (tid_t <= e["t1"])
            e["traj_ids"] = sorted(set(int(x) for x in tid_v[sel])) if sel.any() else []
    # nearest waypoint/corner
    world = os.path.join(ROOT, "sim", "worlds", "forest_hex12.yaml")
    try:
        man = json.load(open(os.path.join(run, "manifest.json")))
        world = (man.get("world") or {}).get("path") or world
    except Exception:  # noqa: BLE001
        pass
    ring = []
    try:
        import yaml
        d = yaml.safe_load(open(world))
        corners = ((d.get("mission") or {}).get("corners") or [])
        for c in corners:
            p = (float(c[0]), float(c[1]))
            if not ring or (abs(ring[-1][0] - p[0]) > 1e-9 or abs(ring[-1][1] - p[1]) > 1e-9):
                ring.append(p)
    except Exception:  # noqa: BLE001
        pass
    for e in events:
        mid = (e["i0"] + e["i1"]) // 2
        e["pos_mid"] = [float(P[mid, 0]), float(P[mid, 1])]
        if ring:
            dd = [float(np.hypot(P[mid, 0] - c[0], P[mid, 1] - c[1])) for c in ring]
            k = int(np.argmin(dd))
            e["nearest_corner"] = k + 1
            e["dist_to_corner_m"] = dd[k]
    out = {"run": run, "tag": tag, "n_events": len(events), "events": events,
           "planner": None if log is None else {
               "n_opt_failed": len(log["opt_failed"]),
               "n_replan_succeed": len(log["replan_succeed"]),
               "n_state_changes": len(log["states"]),
               "states": log["states"],
           }}
    print("== %s: %d loop event(s) (truth turns >=300 deg with <3 m net displacement)" % (tag, len(events)))
    for i, e in enumerate(events, 1):
        plan = "PLAN ALSO LOOPS" if e["plan_loops"] else "TRUTH-ONLY (tracking)"
        corner = ("near corner %d (%.2f m)" % (e["nearest_corner"], e["dist_to_corner_m"])
                  if "nearest_corner" in e else "n/a")
        print("\n%2d) t = %6.1f .. %6.1f s (%.1f s)  turn truth %5.0f deg / cmd %5.0f deg  -> %s"
              % (i, e["t0"], e["t1"], e["dur_s"], e["turn_deg"], e["turn_cmd_deg"], plan))
        print("    net displacement %.2f m; truth speed mean/max %.2f/%.2f m/s; cmd speed max %.2f m/s"
              % (e["net_m"], e["truth_speed_mean_mps"] or 0.0, e["speed_max_mps"] or 0.0,
                 e["cmd_speed_max_mps"] or 0.0))
        print("    along-track mean %s p90|.| %s m; cross-track mean %s p90|.| %s m"
              % ("%.2f" % e["along_mean_m"] if e["along_mean_m"] is not None else "n/a",
                 "%.2f" % e["along_p90_abs_m"] if e["along_p90_abs_m"] is not None else "n/a",
                 "%.2f" % e["cross_mean_m"] if e["cross_mean_m"] is not None else "n/a",
                 "%.2f" % e["cross_p90_abs_m"] if e["cross_p90_abs_m"] is not None else "n/a"))
        print("    position %.1f,%.1f m; %s; traj ids %s"
              % (e["pos_mid"][0], e["pos_mid"][1], corner, e.get("traj_ids", [])))
        if log:
            near = [x for x in log["opt_failed"] if e["t0"] - 3.0 <= x <= e["t1"] + 3.0]
            near_s = [x for x in log["replan_succeed"] if e["t0"] - 3.0 <= x <= e["t1"] + 3.0]
            print("    SUPER log: %d 'Opt failed' and %d 'ReplanOnce succeed' within +-3 s"
                  % (len(near), len(near_s)))
    if log:
        print("\nplanner totals: %d Opt failed, %d ReplanOnce succeed, %d state changes"
              % (len(log["opt_failed"]), len(log["replan_succeed"]), len(log["states"])))
        print("state sequence: " + " -> ".join("%s@%.0fs" % (s["state"], s["t"]) for s in log["states"]))
    if a.json:
        json.dump(out, open(a.json, "w"), indent=2)
        print("\njson: %s" % a.json)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
