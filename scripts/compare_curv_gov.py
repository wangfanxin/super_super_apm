#!/usr/bin/env python3
"""compare_curv_gov.py -- did the curvature governor change the reference/velocity tracking?

Compares the PUBLISHED command (setpoints.csv cmd_x/cmd_y + vel_x/vel_y, which is what the FCU
receives) with the truth, split by distance to mission corners.  All rows are in the same sim clock.
"""
import argparse, csv, os, sys, json
import numpy as np
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import low_speed_census as L

def stats(tag):
    d = L.load(tag); run = L.run_dir(tag)
    t, E, N = d["t"], d["E"], d["N"]; corners = np.array(d["corners"])
    tap = list(csv.DictReader(open(os.path.join(run, "tap.csv"))))
    vt = np.column_stack([[float(r["ve"]) for r in tap], [float(r["vn"]) for r in tap]])
    sp = list(csv.DictReader(open(os.path.join(run, "setpoints.csv"))))
    ts = np.array([float(r["t_sim_us"]) * 1e-6 for r in sp])
    cE = np.array([float(r["cmd_x"]) for r in sp]); cN = np.array([float(r["cmd_y"]) for r in sp])
    vE = np.array([float(r["vel_x"]) for r in sp]); vN = np.array([float(r["vel_y"]) for r in sp])
    o = np.argsort(ts); ts, cE, cN, vE, vN = ts[o], cE[o], cN[o], vE[o], vN[o]
    k = np.concatenate([[True], np.diff(ts) > 1e-9]); ts, cE, cN, vE, vN = ts[k], cE[k], cN[k], vE[k], vN[k]
    fly = (t > 35) & (t < 140)
    vref = np.column_stack([np.interp(t, ts, vE), np.interp(t, ts, vN)])
    dv = np.hypot(vref[:, 0] - vt[:, 0], vref[:, 1] - vt[:, 1])
    cross = vref[:, 0] * vt[:, 1] - vref[:, 1] * vt[:, 0]
    dot = vref[:, 0] * vt[:, 0] + vref[:, 1] * vt[:, 1]
    ang = np.abs(((np.degrees(np.arctan2(cross, dot)) + 180) % 360) - 180)
    dc = np.full(len(t), 1e9)
    for c in corners: dc = np.minimum(dc, np.hypot(E - c[0], N - c[1]))
    near = (dc < 1.5) & fly; mid = (dc >= 3.0) & fly
    perr = np.hypot(np.interp(t, ts, cE) - E, np.interp(t, ts, cN) - N)
    # reference speed at corners (published) and truth corner minimum
    vr = np.hypot(vref[:, 0], vref[:, 1])
    return dict(tag=tag, dv_near=np.percentile(dv[near], 90), dir_near=np.percentile(ang[near], 90),
                dv_mid=np.percentile(dv[mid], 90), dir_mid=np.percentile(ang[mid], 90),
                perr_p90=np.percentile(perr[fly], 90), ref_speed_corner=np.percentile(vr[near], 10),
                truth_vmin_corner=np.percentile(np.hypot(vt[:, 0], vt[:, 1])[near], 5),
                T=float(t[fly][-1] - t[fly][0]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tags", default="phase180d-01-01,phase185a-01-01,phase185b-01-01")
    a = ap.parse_args()
    rows = [stats(x.strip()) for x in a.tags.split(",") if x.strip()]
    print("== published-command velocity tracking (corner<1.5 m / mid-leg>3 m)")
    for r in rows:
        print("  %-18s dv_near p90 %.3f | dir_near p90 %5.1f deg | dv_mid p90 %.3f | dir_mid p90 %5.1f | "
              "pos p90 %.3f | ref corner v10 %.2f | truth corner v5 %.2f | flight %.1f s"
              % (r["tag"], r["dv_near"], r["dir_near"], r["dv_mid"], r["dir_mid"], r["perr_p90"],
                 r["ref_speed_corner"], r["truth_vmin_corner"], r["T"]))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, 3, figsize=(15, 4.2))
    names = [r["tag"].replace("phase", "").replace("-01-01", "") for r in rows]
    cols = ["#888888", "#1f6fd0", "#c02020"]
    for i, (key, title) in enumerate([("dv_near", "corner |dv| p90 [m/s]"),
                                     ("dir_near", "corner direction error p90 [deg]"),
                                     ("truth_vmin_corner", "truth corner v5 [m/s]")]):
        ax = axs[i]
        ax.bar(names, [r[key] for r in rows], color=cols[:len(rows)])
        ax.set_title(title); ax.grid(alpha=0.25, axis="y")
        for j, r in enumerate(rows):
            ax.text(j, r[key], "%.2f" % r[key], ha="center", va="bottom", fontsize=9)
    fig.suptitle("curvature governor A/B: published-command vs truth (corner region)")
    out = os.path.join(ROOT, "docs", "evidence", "final", "curv-gov-ab.png")
    fig.savefig(out, dpi=110, bbox_inches="tight")
    print("written:", out)


if __name__ == "__main__":
    main()
