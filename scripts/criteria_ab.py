#!/usr/bin/env python3
"""criteria_ab.py -- P0 decision evidence: replay the stored sorties under the CURRENT criteria (v1) and
under the PROPOSED ones (v2), and report exactly which flights would change verdict.

This tool changes NOTHING in scripts/accept.py and does not touch any flight.  It exists so the owner can
approve `docs/P0-CRITERIA-PROPOSAL.md` on data instead of prose:

    python3 scripts/criteria_ab.py --runs-dir /home/fanxin/sp_runs --out docs/metrics/p0-criteria-ab.txt

Rows compared (only the ones the proposal touches):
  H2  truth rate        v1: median rate >= 380 Hz
                        v2: rate == configured truth rate (200 Hz for SIM_STATE sorties) +-5%,
                            max gap <= 2 x 1/rate, >= 1000 samples
  W4  pillar clearance  v1: minimum distance > 0
                        v2: minimum >= -0.05 m AND inside-body fraction <= 0.5 %
  W6  tracking lag      v1: per-quarter offset spread <= 0.15 s AND worst lag-compensated p90 <= 0.60 m
                        v2: |median quarter offset| <= 0.35 s AND residual std after removing a LINEAR
                            time trend <= 0.12 s AND worst p90 <= 0.60 m; a quarter deviating > 0.25 s from
                            the median is reported as WARN (does not fail by itself)
  H6  scan budget       v1: cast MAX <= 100 ms AND skipped <= 20
                        v2: cast MAX <= 100 ms AND skipped <= max(20, 1 % of scans) (p99 reported)

Anything the stored data cannot answer (missing scans.csv, pre-SIM_STATE sortie for H2) is reported as
`n/a` for v2, never as a flip.
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import os
import re
import statistics
import sys

import numpy as np

# repo path authority (like every other tool): no author paths in the tool itself
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
import _paths as _P  # noqa: E402

SHIFT_MAX = 0.60
SHIFT_STEP = 0.01


# ---------------------------------------------------------------- loading ----
def _rows(path):
    with open(path, encoding="utf-8", errors="replace") as fh:
        return list(csv.DictReader(fh))


def load_truth(run):
    rows = _rows(os.path.join(run, "tap.csv"))
    has_tsim = bool(rows and "t_sim_us" in rows[0])
    tcol = "t_sim_us" if has_tsim else "t_us"
    t = np.array([float(r[tcol]) * 1e-6 for r in rows])
    p = np.array([[float(r["pe"]), float(r["pn"]), -float(r["pd"])] for r in rows])
    o = np.argsort(t)
    t, p = t[o], p[o]
    keep = np.concatenate([[True], np.diff(t) > 0])
    return t[keep], p[keep], has_tsim


def load_cmd(run):
    rows = _rows(os.path.join(run, "setpoints.csv"))
    t = np.array([float(r["t_sim_us"]) * 1e-6 for r in rows])
    p = np.array([[float(r["cmd_x"]), float(r["cmd_y"]), float(r["cmd_z"])] for r in rows])
    o = np.argsort(t)
    t, p = t[o], p[o]
    keep = np.concatenate([[True], np.diff(t) > 0])
    return t[keep], p[keep]


# ---------------------------------------------------------------- W6 model ----
def _best_shift(tq, tq_pos, st, st_pos, shifts):
    """Best temporal offset of the command against the truth, as accept.py's W6 does it (p90 metric,
    linear interpolation, parabola refinement).  Returns (offset_s, p90_m)."""
    best = (float("inf"), 0.0)
    ps = []
    for sh in shifts:
        j = tq + sh
        ins = (j >= st[0]) & (j <= st[-1])
        if ins.sum() < 40:
            ps.append(float("inf"))
            continue
        cmd = np.stack([np.interp(j[ins], st, st_pos[:, k]) for k in range(3)], axis=1)
        e = np.linalg.norm(cmd - tq_pos[ins], axis=1)
        p90 = float(np.percentile(e, 90))
        ps.append(p90)
        if p90 < best[0]:
            best = (p90, float(sh))
    # parabola refinement around the discrete minimum (same rule as accept.py)
    if ps:
        k = int(np.argmin(ps))
        if 0 < k < len(ps) - 1 and all(np.isfinite(ps[i]) for i in (k - 1, k, k + 1)):
            y0, y1, y2 = ps[k - 1], ps[k], ps[k + 1]
            den = y0 - 2.0 * y1 + y2
            if den > 1e-9:
                delta = 0.5 * (y0 - y2) / den
                best = (best[0], best[1] + max(-0.5, min(0.5, delta)) * SHIFT_STEP)
    return best


def w6_quarters(tt, tr, st, sc, nq=4):
    """Per-quarter (offset, p90) of the command against the truth, inside the common time window."""
    lo = max(tt[0], st[0]) + 0.5
    hi = min(tt[-1], st[-1]) - 0.5
    m = (tt >= lo) & (tt <= hi)
    tq = tt[m]
    tq_pos = tr[m]
    if tq.size < 200:
        return []
    qn = max(1, tq.size // nq)
    shifts = np.arange(-SHIFT_MAX, SHIFT_MAX + 1e-9, SHIFT_STEP)
    out = []
    for i in range(nq):
        sl = slice(i * qn, (i + 1) * qn) if i < nq - 1 else slice((nq - 1) * qn, None)
        p90, off = _best_shift(tq[sl], tq_pos[sl], st, sc, shifts)
        out.append((off, p90))
    return out


def w6_verdicts(run):
    tt, tr, has_tsim = load_truth(run)
    st, sc = load_cmd(run)
    q = w6_quarters(tt, tr, st, sc)
    if len(q) < 3:
        return None
    offs = np.array([o for o, _ in q])
    p90s = np.array([p for _, p in q])
    spread = float(offs.max() - offs.min())
    edge = bool(np.any(np.abs(offs) >= SHIFT_MAX - 1e-6))
    v1 = (not edge) and spread <= 0.15 and float(p90s.max()) <= 0.60
    # v2: remove a LINEAR time trend (clock ratio) and bound what is left
    x = np.arange(len(offs), dtype=float)
    try:
        slope, inter = np.polyfit(x, offs, 1)
        resid = offs - (slope * x + inter)
    except Exception:
        slope, resid = 0.0, offs - offs.mean()
    med = float(np.median(offs))
    dev = float(np.max(np.abs(offs - med)))
    v2 = (abs(med) <= 0.35 and float(np.std(resid)) <= 0.12
          and float(p90s.max()) <= 0.60 and not edge)
    if not has_tsim:
        # PRE-2026-11-25 SORTIE: tap.csv has no explicit simulated clock, so the command/truth comparison
        # carries a per-sortie constant from the recorder time base.  Spread is still comparable with v1,
        # but the v2 ABSOLUTE/DE-TRENDED bounds would be measured against a mixed clock -> undecidable.
        v2 = None
    return {"quarters": [{"offset_s": float(o), "p90_m": float(p)} for o, p in q],
            "legacy_timebase": not has_tsim,
            "spread_s": spread, "median_s": med, "max_dev_s": dev,
            "resid_std_s": float(np.std(resid)), "slope_s_per_q": float(slope),
            "p90_worst_m": float(p90s.max()), "edge": edge,
            "warn_quarter": bool(dev > 0.25), "v1": bool(v1),
            "v2": None if v2 is None else bool(v2)}


# ---------------------------------------------------------------- H2/W4/H6 from accept.json ----
def _detail(a, row):
    for r in a.get("rows", []):
        if r.get("id") == row:
            return r.get("detail") or ""
    return ""


def h2_verdicts(run, a):
    det = _detail(a, "H2")
    m = re.search(r"rate median ([0-9.]+) Hz", det)
    if not m:
        return None
    rate = float(m.group(1))
    v1 = rate >= 380.0
    simstate = os.path.isfile(os.path.join(run, "truth_timebase.json"))
    rows = _rows(os.path.join(run, "tap.csv"))
    tcol = "t_sim_us" if rows and "t_sim_us" in rows[0] else "t_us"
    t = np.array([float(r[tcol]) * 1e-6 for r in rows])
    gaps = np.diff(np.sort(t))
    max_gap = float(gaps.max()) if gaps.size else float("inf")
    cfg = 200.0 if simstate else rate
    v2 = None
    if simstate:
        v2 = (abs(rate - cfg) <= 0.05 * cfg and max_gap <= 2.0 / cfg and len(rows) >= 1000)
    return {"rate_hz": rate, "max_gap_s": max_gap, "configured_hz": cfg,
            "v1": bool(v1), "v2": None if v2 is None else bool(v2)}


def w4_verdicts(a):
    det = _detail(a, "W4")
    m = re.search(r"minimum truth-to-pillar-surface distance over the route: ([+-][0-9.]+) m", det)
    if not m:
        return None
    clear = float(m.group(1))
    mi = re.search(r"([0-9.]+) % of them inside a body", det)
    frac = float(mi.group(1)) / 100.0 if mi else None
    v1 = clear > 0.0
    v2 = None if frac is None else bool(clear >= -0.05 and frac <= 0.005)
    return {"clear_m": clear, "inside_frac": frac, "v1": bool(v1),
            "v2": None if v2 is None else bool(v2)}


def h6_verdicts(run, a):
    path = os.path.join(run, "scans.csv")
    if os.path.isfile(path):
        rows = _rows(path)
        if not rows:
            return None
        ok = [float(r["cast_ms"]) for r in rows if (r.get("reason") or "") == "ok"]
        if not ok:
            return None
        n_scans = len(rows)
        skipped = sum(1 for r in rows if (r.get("reason") or "") != "ok")
        p50 = statistics.median(ok)
        p99 = float(np.percentile(ok, 99))
        mx = max(ok)
        v1 = mx <= 100.0 and skipped <= 20
        v2 = mx <= 100.0 and skipped <= max(20, int(0.01 * n_scans))
        return {"source": "scans.csv", "n_scans": n_scans, "p50_ms": p50, "p99_ms": p99,
                "max_ms": mx, "skipped": skipped, "v1": bool(v1), "v2": bool(v2)}
    det = _detail(a, "H6")
    m = re.search(r"raycast p50 ([0-9.]+) ms, MAX ([0-9.]+) ms \(gate 100\), scans skipped ([0-9]+)", det)
    if not m:
        return None
    p50, mx, skipped = float(m.group(1)), float(m.group(2)), int(m.group(3))
    v1 = mx <= 100.0 and skipped <= 20
    return {"source": "aggregate(log)", "p50_ms": p50, "max_ms": mx, "skipped": skipped,
            "v1": bool(v1), "v2": None}


# ---------------------------------------------------------------- driver ----
def analyse(run):
    ap = os.path.join(run, "accept.json")
    if not os.path.isfile(ap):
        return None
    a = json.load(open(ap))
    rows = {r.get("id"): r for r in a.get("rows", [])}
    rec = {"tag": os.path.basename(run).replace("missionA-", ""),
           "v1_gate": a.get("pass") is not None and a.get("fail") == 0,
           "rows": {}}
    for name, fn in (("H2", lambda: h2_verdicts(run, a)),
                     ("W4", lambda: w4_verdicts(a)),
                     ("W6", lambda: w6_verdicts(run)),
                     ("H6", lambda: h6_verdicts(run, a))):
        try:
            rec["rows"][name] = fn()
        except Exception as exc:                                    # noqa: BLE001
            rec["rows"][name] = {"error": repr(exc)}
    return rec


def main() -> int:
    ap = argparse.ArgumentParser(description="v1 vs proposed-v2 criteria over the stored sorties")
    ap.add_argument("--runs-dir", default=_P.LOGS,
                    help="directory holding missionA-* sortie dirs (default: $LIO_LOGS or <repo>/.runs)")
    ap.add_argument("--out", default="")
    ap.add_argument("--limit", type=int, default=0)
    a = ap.parse_args()
    runs = sorted(glob.glob(os.path.join(a.runs_dir, "missionA-*")))
    if a.limit:
        runs = runs[-a.limit:]
    recs = [r for r in (analyse(d) for d in runs) if r]
    lines = []
    flips = {"H2": [], "W4": [], "W6": [], "H6": []}
    nan = {k: 0 for k in flips}
    for r in recs:
        for name in ("H2", "W4", "W6", "H6"):
            v = r["rows"].get(name) or {}
            if "error" in v:
                lines.append("%-26s %-3s ERROR %s" % (r["tag"], name, v["error"]))
                continue
            if v.get("v2") is None:
                nan[name] += 1
                continue
            if v["v1"] != v["v2"]:
                flips[name].append((r["tag"], bool(v["v1"]), bool(v["v2"]), v))
    print("sorties analysed: %d" % len(recs))
    for name in ("H2", "W4", "W6", "H6"):
        print("\n== %s flips: %d (v2 undecidable from stored data: %d)" % (name, len(flips[name]), nan[name]))
        for tag, v1, v2, v in flips[name]:
            print("  %-26s v1=%-5s -> v2=%-5s  %s" % (tag, v1, v2, _brief(name, v)))
    # write the evidence file
    out = a.out
    if out:
        os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
        with open(out, "w", encoding="utf-8") as fh:
            fh.write("# P0 criteria A/B over the stored sorties (v1 = current accept.py, v2 = proposal)\n")
            fh.write("# generated by scripts/criteria_ab.py -- NO thresholds were changed\n")
            fh.write("# sorties: %d\n\n" % len(recs))
            for name in ("H2", "W4", "W6", "H6"):
                fh.write("## %s -- %d flip(s); v2 undecidable for %d sortie(s)\n"
                         % (name, len(flips[name]), nan[name]))
                for tag, v1, v2, v in flips[name]:
                    fh.write("  %-26s v1=%-5s -> v2=%-5s  %s\n" % (tag, v1, v2, _brief(name, v)))
                fh.write("\n")
            fh.write("## per-sortie detail\n")
            for r in recs:
                fh.write("\n%s  (v1 gate: %s)\n" % (r["tag"], r["v1_gate"]))
                for name in ("H2", "W4", "W6", "H6"):
                    v = r["rows"].get(name) or {}
                    fh.write("  %-3s %s\n" % (name, _brief(name, v) if "error" not in v else "ERROR " + v["error"]))
        print("\nevidence written: %s" % out)
    return 0


def _brief(name, v):
    if not v:
        return "n/a"
    if name == "H2":
        return ("rate %.1f Hz (cfg %.0f), max gap %.4f s, v1=%s v2=%s"
                % (v["rate_hz"], v["configured_hz"], v["max_gap_s"], v["v1"], v["v2"]))
    if name == "W4":
        return ("clear %+.3f m, inside %.2f %%, v1=%s v2=%s"
                % (v["clear_m"], 100.0 * (v["inside_frac"] or 0.0), v["v1"], v["v2"]))
    if name == "W6":
        return ("median %+.3f s, spread %.3f, resid std %.3f, worst p90 %.2f m, %s%s"
                % (v["median_s"], v["spread_s"], v["resid_std_s"], v["p90_worst_m"],
                   "WARN quarter" if v["warn_quarter"] else "",
                   " [legacy time base: v2 n/a]" if v.get("legacy_timebase") else ""))
    if name == "H6":
        return ("p50 %.1f p99 %.1f max %.1f ms, skipped %d, v1=%s v2=%s"
                % (v["p50_ms"], v.get("p99_ms", float("nan")), v["max_ms"], v["skipped"], v["v1"], v["v2"]))
    return "?"


if __name__ == "__main__":
    raise SystemExit(main())
