#!/usr/bin/env python3
"""check_handover_via.py -- 0-flight audit: does the REFERENCE (and truth) actually pass each pacer corner?

The 2026-11-28 review showed the handover failure mode is deterministic: if the new plan's guide does not
reach the corner that the pacer is tracking, the aircraft cuts it, the pacer never fires pass-by, the leg
times out at 40 s and the target sequence shifts for the rest of the flight.  This script reconstructs the
pacer's leg timeline from traj.log's PLANDBG lines (wall -> sim through setpoints.csv) and, for every leg,
measures the reference and the truth against the pass-by box at that corner (lateral <= --pass-by-m,
0 <= along <= --pass-by-long-m, outgoing direction = corner -> next corner).

USAGE: python3 scripts/check_handover_via.py --tag phase180b-01-01
"""
import argparse, csv, importlib.util, os, re, sys
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _load_replay():
    p = os.path.join(ROOT, "scripts", "dispatch_replay.py")
    spec = importlib.util.spec_from_file_location("dispatch_replay", p)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="phase180b-01-01")
    ap.add_argument("--world", default=os.path.join(ROOT, "sim", "worlds", "forest_hex12.yaml"))
    ap.add_argument("--pass-by-m", type=float, default=0.6)
    ap.add_argument("--pass-by-long-m", type=float, default=1.0)
    ap.add_argument("--final-arrive-m", type=float, default=0.5,
                    help="the last target has no pass-by; grade its radius with this (final-arrive)")
    a = ap.parse_args()
    DR = _load_replay()
    run = os.path.join(ROOT, ".runs", "missionA-%s" % a.tag)
    if not os.path.isdir(run):
        run = os.path.join(ROOT, ".runs", "_invalid", "missionA-%s" % a.tag)
    if not os.path.isdir(run):
        raise SystemExit("run not found: %s" % a.tag)
    corners = DR.world_corners(a.world)
    n_targets = 13
    lp = os.path.join(run, "traj.log")
    for ln in open(lp, encoding="utf-8", errors="replace"):
        m = re.search(r"tour:.*=\s*(\d+)\s+targets", ln)
        if m:
            n_targets = int(m.group(1)); break
    tour = np.vstack([[0.0, 0.0, 2.0], corners[:max(1, n_targets - 1)]])
    # clocks: tap (sim truth), setpoints (cmd W + t_wall<->t_sim), PLANDBG (plan-box wall, pacer leg)
    tap = list(csv.DictReader(open(os.path.join(run, "tap.csv"))))
    tt = np.array([float(r["t_sim_us"]) * 1e-6 for r in tap])
    tE = np.array([float(r["pe"]) for r in tap]); tN = np.array([float(r["pn"]) for r in tap])
    sp = list(csv.DictReader(open(os.path.join(run, "setpoints.csv"))))
    ss = np.array([float(r["t_sim_us"]) * 1e-6 for r in sp])
    tw = np.array([float(r["t_wall"]) for r in sp])
    cE = np.array([float(r["cmd_x"]) for r in sp]); cN = np.array([float(r["cmd_y"]) for r in sp])
    o = np.argsort(ss); ss, tw, cE, cN = ss[o], tw[o], cE[o], cN[o]
    k = np.concatenate([[True], np.diff(ss) > 1e-9]); ss, tw, cE, cN = ss[k], tw[k], cE[k], cN[k]

    def w2s(w):
        return float(np.interp(w, tw, ss))

    legs = []  # (sim_t, leg_index_1based)
    for ln in open(lp, encoding="utf-8", errors="replace"):
        if "PLANDBG wall=" not in ln:
            continue
        mw = re.search(r"wall=([0-9.]+)", ln); ml = re.search(r"leg=(\d+)/(\d+)", ln)
        if mw and ml:
            legs.append((w2s(float(mw.group(1))), int(ml.group(1))))
    if not legs:
        raise SystemExit("no PLANDBG leg timeline in %s" % lp)
    # compress to leg intervals
    bounds = []
    cur_leg, t0 = legs[0][1], legs[0][0]
    for t, leg in legs[1:]:
        if leg != cur_leg:
            bounds.append((cur_leg, t0, t)); cur_leg, t0 = leg, t
    bounds.append((cur_leg, t0, legs[-1][0] + 0.05))

    def side_ok(cx, cy, nx, ny, px, py):
        ux, uy = nx - cx, ny - cy
        n = float(np.hypot(ux, uy))
        if n < 1e-9:
            return 99.0, 99.0
        ux, uy = ux / n, uy / n
        rx, ry = px - cx, py - cy
        along = rx * ux + ry * uy
        lat = abs(rx * (-uy) + ry * ux)
        return along, lat

    def min_at(idx, arrE, arrN, ci, is_last):
        cx, cy = tour[ci][:2]
        if is_last:
            d = np.hypot(arrE - cx, arrN - cy)
            j = int(np.argmin(d))
            return float(d[j]), 0.0, 0.0
        nx, ny = tour[min(ci + 1, len(tour) - 1)][:2]
        best = (1e9, 0.0, 0.0)
        for j in idx:
            al, la = side_ok(cx, cy, nx, ny, arrE[j], arrN[j])
            d = np.hypot(arrE[j] - cx, arrN[j] - cy)
            if d < best[0]:
                best = (float(d), float(al), float(la))
        return best

    print("== check_handover_via %s: %d legs, pass-by lat<=%.2f along<=%.2f (last: radius %.2f)"
          % (a.tag, len(bounds) - 1, a.pass_by_m, a.pass_by_long_m, a.final_arrive_m))
    n_ref_hit = n_truth_hit = n_legs = 0
    rows = []
    for (leg, t_lo, t_hi) in bounds:
        ci = leg - 1  # pacer.i_leg+1 -> corner index in tour
        if ci < 0 or ci >= len(tour):
            continue
        is_last = ci >= len(tour) - 1
        im = np.where((tt >= t_lo) & (tt <= t_hi))[0]
        ic = np.where((ss >= t_lo) & (ss <= t_hi))[0]
        if im.size < 10 or ic.size < 10:
            continue
        rd = min_at(ic, cE, cN, ci, is_last)
        td = min_at(im, tE, tN, ci, is_last)
        if is_last:
            ref_hit = rd[0] <= a.final_arrive_m
            truth_hit = td[0] <= a.final_arrive_m
        else:
            ref_hit = rd[0] <= a.pass_by_m or (rd[1] >= 0 and rd[2] <= a.pass_by_m and rd[1] <= a.pass_by_long_m)
            truth_hit = td[0] <= a.pass_by_m or (td[1] >= 0 and td[2] <= a.pass_by_m and td[1] <= a.pass_by_long_m)
        n_legs += 1; n_ref_hit += int(ref_hit); n_truth_hit += int(truth_hit)
        rows.append((leg, t_hi - t_lo, rd, td, ref_hit, truth_hit))
        print("  leg %2d [%6.1f-%6.1f s dur %5.1f] ref min %.2f (along %.2f lat %.2f) %s | "
              "truth min %.2f (along %.2f lat %.2f) %s"
              % (leg, t_lo, t_hi, t_hi - t_lo, rd[0], rd[1], rd[2], "HIT" if ref_hit else "MISS",
                 td[0], td[1], td[2], "HIT" if truth_hit else "MISS"))
    print("SUMMARY: legs %d | reference pass %d/%d | truth pass %d/%d"
          % (n_legs, n_ref_hit, n_legs, n_truth_hit, n_legs))
    return 0 if n_ref_hit == n_legs and n_truth_hit == n_legs else 1


if __name__ == "__main__":
    raise SystemExit(main())
