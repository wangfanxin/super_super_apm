#!/usr/bin/env python3
"""handover_audit.py -- offline audit of the COMMAND-STREAM handovers (trajectory_id switches).

WHY.  The thin layer publishes SUPER's reference; when the FSM switches trajectory (every corner, and at
restarts) the published stream can jump in position/velocity.  That discontinuity -- not MINCO's own
smoothness -- is what the aircraft cannot track, and it is what a "guidance layer" would have to mask.
This script quantifies it from the recorded `/plan/cmd` stream (traj.csv.cmd.csv, columns in L):
  * per-trajectory segment: start/end speed, min published speed, published-v vs dp/dt consistency
    (the plan-box governor / clamps show up as a large mismatch between the velocity FIELD and the
    position derivative);
  * per handover (trajectory_id k -> k+1): dt gap, position jump |dp|, velocity step |dv| and its
    direction change, implied |dv|/dt, whether it is a rest-to-rest handover (v~0 on either side),
    and the fraction of P1 spikes that land within +-0.35 s of a handover.

USAGE
  python3 scripts/handover_audit.py --tag phase44b-01-01 --tag phase94a-01-01 --fig docs/evidence/final/handover-audit.png
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


def run_dir(tag):
    p = os.path.join(ROOT, ".runs", "missionA-%s" % tag)
    if not os.path.isdir(p):
        p = os.path.join(os.environ.get("LIO_LOGS", ""), "missionA-%s" % tag)
    if not os.path.isdir(p):
        raise SystemExit("run not found: %s" % tag)
    return p


def load_cmd(run):
    for name in ("traj.csv.cmd.csv", os.path.join("derived", "traj.csv.cmd.csv")):
        p = os.path.join(run, name)
        if os.path.isfile(p):
            rows = list(csv.DictReader(open(p, encoding="utf-8")))
            break
    else:
        return None
    t = np.array([float(r["t_us"]) * 1e-6 for r in rows])
    p = np.column_stack([[float(r[k]) for r in rows] for k in ("x_l", "y_l", "z_l")])
    v = np.column_stack([[float(r[k]) for r in rows] for k in ("vx_l", "vy_l", "vz_l")])
    tid = np.array([int(r["trajectory_id"]) for r in rows])
    flag = np.array([int(r["trajectory_flag"]) for r in rows])
    o = np.argsort(t, kind="stable")
    t, p, v, tid, flag = t[o], p[o], v[o], tid[o], flag[o]
    k = np.concatenate([[True], np.diff(t) > 1e-9])
    return {"t": t[k], "p": p[k], "v": v[k], "tid": tid[k], "flag": flag[k]}


def segments(cmd):
    """list of (tid, i0, i1) with i1 exclusive, consecutive equal ids."""
    out = []
    i = 0
    while i < len(cmd["tid"]):
        j = i + 1
        while j < len(cmd["tid"]) and cmd["tid"][j] == cmd["tid"][i]:
            j += 1
        out.append((int(cmd["tid"][i]), i, j))
        i = j
    return out


def audited(run, tag):
    cmd = load_cmd(run)
    if cmd is None:
        return {"tag": tag, "error": "no traj.csv.cmd.csv"}
    t, p, v = cmd["t"], cmd["p"], cmd["v"]
    sp = np.linalg.norm(v[:, :2], axis=1)
    if len(t) < 10:
        return {"tag": tag, "error": "too few samples"}
    # dp/dt consistency (published velocity field vs position derivative, horizontal)
    dp = np.gradient(p[:, :2], t, axis=0)
    dsp = np.linalg.norm(dp, axis=1)
    mism = np.linalg.norm(v[:, :2] - dp, axis=1)
    segs = segments(cmd)
    seg_out = []
    for tid, i0, i1 in segs:
        if i1 - i0 < 3:
            continue
        ss = sp[i0:i1]
        jmin = int(np.argmin(ss))
        seg_out.append({"tid": tid, "t0": float(t[i0]), "t1": float(t[i1 - 1]), "n": int(i1 - i0),
                        "v_start": float(sp[i0]), "v_end": float(sp[i1 - 1]),
                        "vmin": float(ss.min()), "t_vmin": float(t[i0 + jmin]),
                        "mism_p90": float(np.percentile(mism[i0:i1], 90)),
                        "mism_p99": float(np.percentile(mism[i0:i1], 99)),
                        "flag_first": int(cmd["flag"][i0]), "flag_last": int(cmd["flag"][i1 - 1])})
    ho = []
    for a, b in zip(range(len(segs) - 1), range(1, len(segs))):
        ta, ia0, i1 = segs[a]
        tb, ib0, ib1 = segs[b]
        if i1 - ia0 < 2 or ib0 >= len(t):
            continue
        dt = float(t[ib0] - t[i1 - 1])
        dpj = float(np.linalg.norm(p[ib0, :2] - p[i1 - 1, :2]))
        va = v[i1 - 1, :2]
        vb = v[ib0, :2]
        dv = float(np.linalg.norm(vb - va))
        na, nb = float(np.linalg.norm(va)), float(np.linalg.norm(vb))
        cosang = float(np.dot(va, vb) / (na * nb)) if na > 1e-6 and nb > 1e-6 else 1.0
        ang = float(np.degrees(np.arccos(np.clip(cosang, -1, 1))))
        ho.append({"from_tid": int(ta), "to_tid": int(tb), "t": float(t[ib0]), "dt": dt,
                   "dp": dpj, "dv": dv, "v_old": na, "v_new": nb,
                   "amp_step": float(abs(nb - na)), "angle": ang,
                   "dv_rate": float(dv / max(dt, 1e-3)),
                   "rest_old": bool(na < 0.15), "rest_new": bool(nb < 0.15),
                   "flag_last_old": int(cmd["flag"][i1 - 1]), "flag_first_new": int(cmd["flag"][ib0])})
    # ---- within-segment audit: THIN-LAYER corruption (holds / velocity dips / accel glitches) ----
    # These are the things a "guidance layer" would have to mask; if they are inside segments they are
    # plan-box artefacts, not handovers.
    dp_step = np.linalg.norm(np.diff(p[:, :2], axis=0), axis=1)
    dv_step = np.linalg.norm(np.diff(v[:, :2], axis=0), axis=1)
    dt_step = np.diff(t)
    accel = dv_step / np.maximum(dt_step, 1e-4)
    # holds: consecutive samples with frozen position (<1 mm) for >=0.05 s
    frozen = dp_step < 1e-3
    holds = []
    i = 0
    while i < len(frozen):
        if not frozen[i]:
            i += 1
            continue
        j = i
        while j + 1 < len(frozen) and frozen[j + 1]:
            j += 1
        dur = float(t[j + 1] - t[i]) if j + 1 < len(t) else float(t[j] - t[i])
        if dur >= 0.05:
            holds.append({"t": float(t[i]), "dur": dur})
        i = j + 1
    glitches = [{"t": float(t[i + 1]), "dv": float(dv_step[i]), "dt": float(dt_step[i]),
                 "accel": float(accel[i])}
                for i in range(len(dv_step)) if accel[i] > 20.0 and dv_step[i] > 0.05]
    # dips: inside each segment, min published speed and how far from the segment END
    for s in seg_out:
        i0 = int(np.searchsorted(t, s["t0"])); i1 = int(np.searchsorted(t, s["t1"]))
        s["minv_to_end_s"] = float(s["t1"] - s["t_vmin"])
        s["dip_has_hold"] = bool(any(abs(h["t"] - s["t_vmin"]) < 0.15 for h in holds))
        s["minv_from_start_s"] = float(s["t_vmin"] - s["t0"])
        ss = sp[i0:i1]
        # time until the published speed exceeds 0.5 m/s after the segment start (corner restart cost)
        above = np.flatnonzero(ss > 0.5)
        s["t_to_0p5"] = float(t[i0 + above[0]] - t[i0]) if above.size else float("nan")
        # ---- Phase-1 offline gate: LEG-START DIP ------------------------------------------
        # what the aircraft actually sees in the first 0.8 s of a new leg, against that leg's own
        # cruise speed.  A continuous plan should keep dip08/cruise near 1.0.
        n08 = int(min(len(ss), max(3, int(round(0.8 / max(0.01, float(np.median(np.diff(t[i0:i1])))))))))
        if n08 >= 3:
            first = ss[:n08]
            jdip = int(np.argmin(first))
            cruise = float(np.percentile(ss, 90)) if ss.size else float("nan")
            target = 0.8 * cruise
            after = jdip + int(np.argmax(ss[jdip:] >= target)) if np.any(ss[jdip:] >= target) else -1
            s["dip08"] = float(first[jdip])
            s["dip08_ratio"] = float(first[jdip] / cruise) if cruise > 0.1 else float("nan")
            s["t_rec"] = float(t[i0 + after] - t[i0 + jdip]) if after >= 0 else float("nan")
            s["n08"] = n08
    # inner segments only (drop the pad/vertical first and the landing last)
    tids = sorted({s["tid"] for s in seg_out})
    inner = [s for s in seg_out if len(tids) > 2 and tids[1] <= s["tid"] <= tids[-2]]
    short_holds = [h for h in holds if h["dur"] < 5.0]
    out = {"tag": tag, "n_cmd": int(len(t)), "n_seg": len(seg_out), "n_ho": len(ho),
           "segments": seg_out, "handovers": ho,
           "inner": [{"tid": s["tid"], "v_start": s["v_start"], "t_to_0p5": s["t_to_0p5"],
                      "vmin": s["vmin"], "minv_from_start_s": s["minv_from_start_s"],
                      "dip08": s.get("dip08"), "dip08_ratio": s.get("dip08_ratio"),
                      "t_rec": s.get("t_rec")} for s in inner],
           "short_holds": short_holds,
           "holds": holds, "n_holds": len(holds),
           "glitches": glitches[:200], "n_glitches": len(glitches),
           "cmd_vs_dp": {"p50": float(np.percentile(mism, 50)), "p90": float(np.percentile(mism, 90)),
                         "p99": float(np.percentile(mism, 99))},
           "t_range": [float(t[0]), float(t[-1])]}
    # P1 spikes (time-aligned truth-vs-command deviation) -> nearest handover distance
    try:
        import plan_exec as PE
        pe = PE.plan_exec(tag)
        if "error" not in pe:
            # offset between tap (sim) clock and traj.csv (epoch) clock: cross-correlate speeds
            run2 = run_dir(tag)
            tap_rows = list(csv.DictReader(open(os.path.join(run2, "tap.csv"))))
            tt = np.array([float(r["t_sim_us"]) * 1e-6 for r in tap_rows])
            ve = np.array([float(r["ve"]) for r in tap_rows])
            vn = np.array([float(r["vn"]) for r in tap_rows])
            tspd = np.hypot(ve, vn)
            # resample both speed series to a common 20 Hz grid over relative time, find best shift
            dt = 0.05
            cmd_grid = np.arange(0, t[-1] - t[0], dt)
            cmd_s = np.interp(t[0] + cmd_grid, t, sp)
            best, best_shift = -1.0, 0.0
            for shift in np.arange(-120.0, 120.0, 0.5):
                tap_grid = np.arange(0, t[-1] - t[0], dt) + shift
                tap_s = np.interp(tt[0] + tap_grid, tt, tspd, left=0.0, right=0.0)
                m = np.isfinite(tap_s)
                if m.sum() < 200:
                    continue
                c = float(np.corrcoef(cmd_s[m], tap_s[m])[0, 1])
                if c > best:
                    best, best_shift = c, float(shift)
            out["p1"] = {"offset_s": best_shift, "corr": best}
            # spikes from quality rows-like computation on setpoints/tap are simpler: reuse plan_exec json
            # here we just record the P1 row summary via quality_rows if desired; spike times need recompute.
    except Exception as e:  # noqa: BLE001
        out["p1"] = {"error": str(e)}
    return out


def report(o):
    if "error" in o:
        print("%-16s ERROR %s" % (o["tag"], o["error"]))
        return
    h = o["handovers"]
    print("== %s : cmd samples %d, segments %d, handovers %d" % (o["tag"], o["n_cmd"], o["n_seg"], o["n_ho"]))
    print("   published |v| vs dp/dt mismatch: p50 %.3f p90 %.3f p99 %.3f m/s" %
          (o["cmd_vs_dp"]["p50"], o["cmd_vs_dp"]["p90"], o["cmd_vs_dp"]["p99"]))
    if h:
        dv = np.array([x["dv"] for x in h]); ang = np.array([x["angle"] for x in h])
        dpj = np.array([x["dp"] for x in h]); rate = np.array([x["dv_rate"] for x in h])
        rest = sum(1 for x in h if x["rest_old"] or x["rest_new"])
        print("   handover |dv|: p50 %.2f p90 %.2f max %.2f m/s | angle p50 %.1f max %.1f deg" %
              (np.percentile(dv, 50), np.percentile(dv, 90), dv.max(),
               np.percentile(ang, 50), ang.max()))
        print("   handover |dp|: p50 %.3f p90 %.3f max %.3f m | |dv|/dt p50 %.1f max %.1f m/s^2" %
              (np.percentile(dpj, 50), np.percentile(dpj, 90), dpj.max(),
               np.percentile(rate, 50), rate.max()))
        print("   rest-to-rest handovers: %d/%d | hard (|dv|>0.2 or angle>20 or |dp|>0.05): %d" %
              (rest, len(h),
               sum(1 for x in h if x["dv"] > 0.2 or x["angle"] > 20.0 or x["dp"] > 0.05)))
    minv = [s["vmin"] for s in o["segments"]]
    print("   per-segment min published |v|: median %.2f, segments<0.3 m/s: %d/%d, segments<0.1: %d" %
          (float(np.median(minv)), sum(1 for x in minv if x < 0.3), len(minv),
           sum(1 for x in minv if x < 0.1)))
    if "n_holds" in o:
        hd = [h["dur"] for h in o["holds"]]
        print("   FROZEN-POSITION holds: %d (longest %.2f s, total %.1f s) | accel glitches >20 m/s^2: %d (max %.0f)" %
              (o["n_holds"], max(hd) if hd else 0.0, sum(hd), o["n_glitches"],
               max([g["accel"] for g in o["glitches"]], default=0.0)))
        segs = [s for s in o["segments"] if s.get("dip_has_hold") is not None]
        print("   dips inside segments: median from segment START %.2f s / from END %.2f s, with a hold within 0.15 s: %d/%d" %
              (float(np.median([s["minv_from_start_s"] for s in segs])),
               float(np.median([s["minv_to_end_s"] for s in segs])),
               sum(1 for s in segs if s["dip_has_hold"]), len(segs)))
    if o.get("inner"):
        vs = [x["v_start"] for x in o["inner"]]
        tt = [x["t_to_0p5"] for x in o["inner"] if np.isfinite(x["t_to_0p5"])]
        mf = [x["minv_from_start_s"] for x in o["inner"]]
        print("   INNER leg starts (n=%d): v_start median %.2f (rest<0.15: %d, <0.5: %d) | "
              "time to reach 0.5 m/s: median %.2f s | dip position: median %.2f s after start" %
              (len(vs), float(np.median(vs)), sum(1 for x in vs if x < 0.15), sum(1 for x in vs if x < 0.5),
               float(np.median(tt)) if tt else float("nan"), float(np.median(mf)) if mf else float("nan")))
        sh = [h["dur"] for h in o.get("short_holds", [])]
        print("   short frozen-holds (<5 s): %d, total %.2f s, longest %.2f s" %
              (len(sh), sum(sh), max(sh) if sh else 0.0))
        dip = [x["dip08"] for x in o["inner"] if x.get("dip08") is not None]
        rat = [x["dip08_ratio"] for x in o["inner"] if x.get("dip08_ratio") is not None and np.isfinite(x["dip08_ratio"])]
        rec = [x["t_rec"] for x in o["inner"] if x.get("t_rec") is not None and np.isfinite(x["t_rec"])]
        if dip:
            print("   LEG-START DIP (first 0.8 s): min |v| median %.2f m/s, ratio to leg cruise median %.2f, "
                  "recovery to 0.8*cruise median %.2f s | legs with ratio<0.6: %d/%d" %
                  (float(np.median(dip)), float(np.median(rat)) if rat else float("nan"),
                   float(np.median(rec)) if rec else float("nan"),
                   sum(1 for r in rat if r < 0.6), len(rat)))


def figure(tags, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    try:
        import _figfont
        cjk = _figfont.ensure_cjk_font(verbose=False)
        plt.rcParams["font.sans-serif"] = [cjk, "DejaVu Sans"]
    except Exception:  # noqa: BLE001
        pass
    plt.rcParams["axes.unicode_minus"] = False
    data = [(t, audited(run_dir(t), t)) for t in tags]
    data = [(t, o) for t, o in data if "error" not in o]
    if not data:
        print("no data for figure")
        return
    n = len(data)
    fig, axes = plt.subplots(2, n, figsize=(6.2 * n, 7.4), squeeze=False)
    for c, (tag, o) in enumerate(data):
        run = run_dir(tag)
        cmd = load_cmd(run)
        t, v = cmd["t"], np.linalg.norm(cmd["v"][:, :2], axis=1)
        t0 = t[0]
        ax = axes[0, c]
        ax.plot(t - t0, v, lw=0.8, color="#455a64", label="published |v| (from /plan/cmd)")
        for h in o["handovers"]:
            ax.axvline(h["t"] - t0, color="#b71c1c", lw=0.6, alpha=0.5)
        for h in o.get("holds", []):
            ax.axvspan(h["t"] - t0, h["t"] - t0 + h["dur"], color="#ff9800", alpha=0.25)
        for g in o.get("glitches", []):
            ax.axvline(g["t"] - t0, color="#7b1fa2", lw=0.5, alpha=0.55)
        ax.axhline(0.3, color="orange", ls="--", lw=0.8)
        ax.set_title("%s: |v| + handovers(red) + holds(orange) + glitches(purple) n_ho=%d" % (tag, o["n_ho"]), fontsize=8.5)
        ax.set_xlabel("t in stream (s)"); ax.set_ylabel("m/s"); ax.set_ylim(bottom=0)
        ax = axes[1, c]
        h = o["handovers"]
        if h:
            hv = np.array([x["dv"] for x in h])
            ang = np.array([x["angle"] for x in h])
            sc = ax.scatter(np.array([x["v_old"] for x in h]), hv, c=ang, cmap="turbo", s=22)
            ax.axhline(0.2, color="orange", ls="--", lw=0.8)
            ax.set_xlabel("|v| before the handover (m/s)"); ax.set_ylabel("|dv| step (m/s)")
            ax.set_title("handover severity (colour = direction change, deg)")
            cb = plt.colorbar(sc, ax=ax); cb.set_label("angle (deg)")
        else:
            ax.text(0.5, 0.5, "no handovers", ha="center")
        ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(out, dpi=140)
    print("figure written:", out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", action="append", required=True)
    ap.add_argument("--json", default="")
    ap.add_argument("--fig", default="")
    a = ap.parse_args()
    outs = [audited(run_dir(t), t) for t in a.tag]
    for o in outs:
        report(o)
    if a.json:
        json.dump(outs, open(a.json, "w"), indent=1)
    if a.fig:
        figure(a.tag, a.fig)


if __name__ == "__main__":
    main()
