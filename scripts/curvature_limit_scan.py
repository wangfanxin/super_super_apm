#!/usr/bin/env python3
"""curvature_limit_scan.py -- 0-flight gate for a curvature/lateral-acceleration speed limit.

ODD-AT-CORNER HYPOTHESIS (owner, 2026-11-28): the velocity VECTOR cannot follow the reference through a
corner (measured: direction error p90 18 deg, 24 deg near corners), so the planned corner speed and the
true speed diverge; after the rest2rest switch the new plan starts from the TRUE velocity while the old plan
ended with the PLANNED one -- a real step.  A curvature speed limit v <= sqrt(a_lat_max / kappa) is the
mechanism-level way to ask the reference for less lateral acceleration, so the tracking (and the handover)
can actually follow.

This tool takes an already-flown sortie's SUPER reference velocity/acceleration field (traj.csv.cmd, L->W),
computes kappa = |v x a| / |v|^3, and reports -- for a range of a_lat_max -- how much speed the corners lose,
what the straights keep, and how much route time the limit costs.  It also measures the handover step:
old-reference end speed vs new-plan start speed vs truth speed at each rest2rest.

USAGE: python3 scripts/curvature_limit_scan.py --tag phase180d-01-01
"""
import argparse, csv, json, os, re, sys
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import low_speed_census as L


def world_path(a):
    return os.path.join(ROOT, "sim", "worlds", "forest_hex12.yaml")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="phase180d-01-01")
    ap.add_argument("--lat-accels", default="1.2,1.5,2.0,2.5,3.0")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    run = L.run_dir(a.tag)
    d = L.load(a.tag)
    t, E, N = d["t"], d["E"], d["N"]
    tr = list(csv.DictReader(open(os.path.join(run, "traj.csv.cmd.csv"))))
    tt = np.array([float(r["t_us"]) * 1e-6 for r in tr])
    vL = np.column_stack([[float(r["vx_l"]) for r in tr], [float(r["vy_l"]) for r in tr]])
    aL = np.column_stack([[float(r["ax_l"]) for r in tr], [float(r["ay_l"]) for r in tr]])
    pL = np.column_stack([[float(r["x_l"]) for r in tr], [float(r["y_l"]) for r in tr]])
    tr2 = tr
    cal = json.load(open(os.path.join(ROOT, "calib", "calib.json"))); R = np.array(cal["R_WL"])
    vW = vL @ R[:2, :2].T; aW = aL @ R[:2, :2].T
    o = np.argsort(tt); tt, vW, aW = tt[o], vW[o], aW[o]; pL = pL[o]
    k = np.concatenate([[True], np.diff(tt) > 1e-9]); tt, vW, aW = tt[k], vW[k], aW[k]; pL = pL[k]
    te = tt - tt[0]
    # align to sim by speed-profile matching
    vtn = np.hypot(np.array([float(r["ve"]) for r in csv.DictReader(open(os.path.join(run, "tap.csv")))]),
                   np.array([float(r["vn"]) for r in csv.DictReader(open(os.path.join(run, "tap.csv")))]))
    best = (1e9, 0.0)
    vs = np.hypot(vW[:, 0], vW[:, 1])
    for s0 in np.arange(0, 60, 0.02):
        vv = np.interp(t, te + s0, vs, left=np.nan, right=np.nan)
        m = np.isfinite(vv) & (t > 35) & (t < 140)
        if m.sum() < 500:
            continue
        e = np.nanmean(abs(vv[m] - vtn[m]))
        if e < best[0]:
            best = (e, s0)
    s0 = best[1]
    ts_ = te + s0
    # only the flight part
    m = (t > 35) & (t < 140)
    tq = t[m]
    vx = np.interp(tq, ts_, vW[:, 0], left=0, right=0); vy = np.interp(tq, ts_, vW[:, 1], left=0, right=0)
    ax = np.interp(tq, ts_, aW[:, 0], left=0, right=0); ay = np.interp(tq, ts_, aW[:, 1], left=0, right=0)
    v = np.hypot(vx, vy)
    # robust curvature: local circle fit (Kasa) on the SUPER reference POSITIONS over a 0.4 s window.
    # (v x a / |v|^3 on the raw field is destroyed by acceleration noise; heading differences are noisy at
    # low speed.  The committed trajectory's positions are smooth, so a circle fit is well conditioned.)
    PW = pL @ R[:2, :2].T   # same sorted order as vW / the aligned clock
    def circle_kappa(P, tarr, tq_, win=0.4):
        out = np.full(len(tq_), np.nan)
        for kk_ in range(len(tq_)):
            m = (tarr >= tq_[kk_] - win / 2) & (tarr <= tq_[kk_] + win / 2)
            if m.sum() < 8:
                continue
            Pw = P[m]
            A = np.column_stack([2 * Pw[:, 0], 2 * Pw[:, 1], np.ones(len(Pw))])
            b = Pw[:, 0] ** 2 + Pw[:, 1] ** 2
            try:
                cx, cy, c = np.linalg.lstsq(A, b, rcond=None)[0]
            except Exception:
                continue
            r2 = c + cx * cx + cy * cy
            if r2 <= 1e-6:
                continue
            out[kk_] = 1.0 / np.sqrt(r2)
        return out
    kappa = circle_kappa(PW, ts_, tq)
    # only where the fit has samples; interpolate over gaps, then smooth and clip implausible values
    okk = np.isfinite(kappa)
    kappa = np.interp(tq, tq[okk], kappa[okk]) if okk.any() else np.zeros_like(tq)
    n = max(3, int(0.2 / np.median(np.diff(tq))) | 1)
    kappa = np.convolve(kappa, np.ones(n) / n, "same")
    kappa = np.clip(kappa, 0.0, 3.0)          # a 1.35 m/s quad cannot need more than ~3 1/m in this world
    v = np.hypot(vx, vy)
    alat = v * v * kappa
    # arclength along the reference path (L->W), so retiming is exact and does not rely on differentiating time
    PWq = np.column_stack([np.interp(tq, ts_, PW[:, 0]), np.interp(tq, ts_, PW[:, 1])])
    dP = np.linalg.norm(np.diff(PWq, axis=0), axis=1)
    vmid = 0.5 * (v[:-1] + v[1:])
    ds = dP
    base_time = float(np.sum(dP / np.maximum(vmid, 1e-3)))
    print("== %s: reference curvature scan (flight 35-140 s, %.1f m/s p50)" % (a.tag, np.median(v)))
    print("   speed p50 %.2f max %.2f | kappa p50 %.3f p90 %.3f max %.3f 1/m | a_lat p50 %.2f p90 %.2f max %.2f m/s^2"
          % (np.median(v), v.max(), np.median(kappa), np.percentile(kappa, 90), kappa.max(),
             np.median(alat), np.percentile(alat, 90), alat.max()))
    print("   a_lat_max   time cost   v_min(corner)  v_p50(straight, kappa<0.2)  max a_lat after   frac time limited")
    res = []
    for amax in [float(x) for x in a.lat_accels.split(",")]:
        vlim = np.minimum(v, np.sqrt(amax / np.maximum(kappa, 1e-6)))
        vlim = np.maximum(vlim, 0.05)
        vlim_mid = 0.5 * (vlim[:-1] + vlim[1:])
        new_time = float(np.sum(ds / np.maximum(vlim_mid, 1e-3)))
        straights = kappa < 0.2
        limited = vlim < v - 1e-6
        res.append((amax, new_time / base_time, vlim.min(), np.median(vlim[straights]) if straights.any() else float("nan"),
                    (vlim ** 2 * kappa).max(), limited.mean()))
        _lim = (vlim < v - 0.02)
        _vmin = float(vlim[_lim].min()) if _lim.any() else float(vlim.min())
        print("   %6.2f      %+5.1f%%      %.2f m/s       %.2f m/s                 %.2f m/s^2       %4.1f%%"
              % (amax, 100.0 * (new_time / base_time - 1.0), _vmin,
                 np.median(vlim[straights]) if straights.any() else float("nan"),
                 (vlim ** 2 * kappa).max(), 100.0 * limited.mean()))
    # handover step: old-reference end vs new-plan start around each rest2rest
    sp = list(csv.DictReader(open(os.path.join(run, "setpoints.csv"))))
    tw = np.array([float(r["t_wall"]) for r in sp]); ts2 = np.array([float(r["t_sim_us"]) * 1e-6 for r in sp])
    oo = np.argsort(ts2); ts2, tw = ts2[oo], tw[oo]
    kk = np.concatenate([[True], np.diff(ts2) > 1e-9]); ts2, tw = ts2[kk], tw[kk]
    ev, cur = [], None
    for ln in open(os.path.join(run, "traj.log"), encoding="utf-8", errors="replace"):
        mm = re.search(r"PLANDBG wall=([0-9.]+)", ln)
        if mm:
            cur = float(np.interp(float(mm.group(1)), tw, ts2))
        if "COMMIT " in ln and "rest2rest" in ln and cur is not None:
            ev.append(cur)
    print("   rest2rest commit handover (old committed end vs new plan start vs truth):")
    steps = []
    for te_ in ev:
        if te_ < 40:
            continue
        old = float(np.interp(te_ - 0.15, ts_, vs))
        new = float(np.interp(te_ + 0.15, ts_, vs))
        tru = float(np.interp(te_, t, vtn))
        steps.append((te_, old, new, tru))
        print("     t=%6.1f  old_committed_end %.2f | new_plan_start %.2f | truth %.2f | plan step %+.2f | old_ref-truth %+.2f"
              % (te_, old, new, tru, old - new, old - tru))
    if steps:
        arr = np.array(steps)
        print("   mean |plan step| %.2f m/s; mean old_ref-truth %+.2f m/s; mean truth %.2f m/s"
              % (np.mean(np.abs(arr[:, 1] - arr[:, 2])), np.mean(arr[:, 1] - arr[:, 3]), np.mean(arr[:, 3])))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(3, 1, figsize=(14, 10), gridspec_kw=dict(hspace=0.35))
    axs[0].plot(tq, v, "-", color="0.15", label="SUPER reference |v|")
    axs[0].plot(tq, np.sqrt(np.array([float(x) for x in a.lat_accels.split(",")])[0] / np.maximum(kappa, 1e-6)),
                "--", color="#c02020", label="v <= sqrt(%.1f/kappa)" % float(a.lat_accels.split(",")[0]))
    axs[0].set_title("%s: reference speed and the curvature speed limit" % a.tag)
    axs[0].set_ylabel("m/s"); axs[0].legend(fontsize=8); axs[0].grid(alpha=0.25)
    axs[1].plot(tq, kappa, "-", color="#7a30c0"); axs[1].set_ylabel("kappa [1/m]"); axs[1].grid(alpha=0.25)
    axs[1].set_title("reference curvature")
    axs[2].plot(tq, alat, "-", color="#c02020"); axs[2].set_ylabel("a_lat [m/s^2]")
    axs[2].set_title("demanded lateral acceleration = v^2 * kappa"); axs[2].grid(alpha=0.25)
    axs[2].set_xlabel("sim time [s]")
    out = a.out or os.path.join(ROOT, "docs", "evidence", "final", "%s_curvature-limit.png" % a.tag)
    fig.savefig(out, dpi=110, bbox_inches="tight")
    print("written:", out)


if __name__ == "__main__":
    main()
