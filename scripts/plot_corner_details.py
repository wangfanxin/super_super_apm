#!/usr/bin/env python3
"""plot_corner_details.py -- one ZOOMED top view per mission corner, to see what happens at the handover.

    python3 scripts/plot_corner_details.py --tag phase4q-01-01
    python3 scripts/plot_corner_details.py --tag phase4q-01-01 --half 4.0 --out-prefix corner

For every distinct corner of the sortie's world it draws a window of +-HALF metres around that corner and,
per visit (lap 1, lap 2 in a two-lap mission),:
  * the pillars as the 0.2 m squares they are at the mission height,
  * the corner itself and the planner's arrival gate circle (the value the plan box printed on the banner),
  * the TRUTH path, colour-graded by TIME (dark = earlier) with an arrow every second so the direction of
    travel is readable,
  * the COMMAND path (dashed) and its handover point (where the trajectory_id changed = the new goal),
  * a small caption: the visit's closest approach, the truth's / command's minimum speed there, and whether a
    loop event from scripts/analyze_loops.py overlaps.
It is a DIAGNOSTIC figure (no acceptance numbers); the run's accept.json stays the authority.
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
import _figfont  # noqa: E402
import _plot_obstacles as OBS  # noqa: E402

OBS_R, OBS_INFL, OBS_ZSPLIT = OBS.geometry(root=os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
OBS_RING_M = 3.2

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, Rectangle  # noqa: E402


def _rows(path):
    with open(path, encoding="utf-8", errors="replace") as fh:
        return list(csv.DictReader(fh))


def load_world(run):
    path = os.path.join(ROOT, "sim", "worlds", "forest_hex12.yaml")
    try:
        man = json.load(open(os.path.join(run, "manifest.json")))
        path = (man.get("world") or {}).get("path") or path
    except Exception:  # noqa: BLE001
        pass
    import yaml
    d = yaml.safe_load(open(path))
    pillars = d.get("pillars") or []
    corners = ((d.get("mission") or {}).get("corners") or [])
    # the world's `mission.corners` repeats the cycle once per lap (18 entries for 3 laps); keep ONE cycle
    ring = []
    for c in corners:
        p = (float(c[0]), float(c[1]))
        if ring and len(ring) >= 3 and abs(ring[0][0] - p[0]) < 1e-9 and abs(ring[0][1] - p[1]) < 1e-9:
            break
        if not ring or (abs(ring[-1][0] - p[0]) > 1e-9 or abs(ring[-1][1] - p[1]) > 1e-9):
            ring.append(p)
    return pillars, ring, os.path.basename(path)


def load_truth(run):
    rows = _rows(os.path.join(run, "tap.csv"))
    tcol = "t_sim_us" if rows and "t_sim_us" in rows[0] else "t_us"
    t = np.array([float(r[tcol]) * 1e-6 for r in rows])
    E = np.array([float(r["pe"]) for r in rows])
    N = np.array([float(r["pn"]) for r in rows])
    U = np.array([-float(r["pd"]) for r in rows])
    vE = np.array([float(r["ve"]) for r in rows])
    vN = np.array([float(r["vn"]) for r in rows])
    o = np.argsort(t)
    return t[o], E[o], N[o], U[o], vE[o], vN[o]


def load_cmd(run):
    rows = _rows(os.path.join(run, "setpoints.csv"))
    t = np.array([float(r["t_sim_us"]) * 1e-6 for r in rows])
    cx = np.array([float(r["cmd_x"]) for r in rows])
    cy = np.array([float(r["cmd_y"]) for r in rows])
    spd = np.hypot(np.array([float(r["vel_x"]) for r in rows]),
                   np.array([float(r["vel_y"]) for r in rows]))
    o = np.argsort(t)
    return t[o], cx[o], cy[o], spd[o]


def traj_id_switches(run):
    for path in (os.path.join(run, "traj.csv.cmd.csv"), os.path.join(run, "derived", "traj.csv.cmd.csv")):
        if os.path.isfile(path):
            rows = _rows(path)
            t = np.array([float(r["t_us"]) * 1e-6 for r in rows])
            ids = np.array([int(float(r["trajectory_id"])) for r in rows])
            o = np.argsort(t)
            t, ids = t[o], ids[o]
            return [(float(t[i]), int(ids[i - 1]), int(ids[i])) for i in range(1, len(ids)) if ids[i] != ids[i - 1]]
    return []


def loop_events(run):
    try:
        import analyze_loops as A
        t, P, V = A.load(run)
        m = P[:, 2] >= 1.0
        return [(e["t0"], e["t1"]) for e in A.detect_loops(t[m], P[m])]
    except Exception:  # noqa: BLE001
        return []


def visits(t, E, N, corner, radius=1.5):
    """Contiguous groups of truth samples within `radius` of the corner -> one entry per visit."""
    d = np.hypot(E - corner[0], N - corner[1])
    inside = d < radius
    out = []
    i = 0
    n = len(t)
    while i < n:
        if not inside[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and inside[j + 1]:
            j += 1
        k = int(np.argmin(d[i:j + 1])) + i
        out.append({"i0": i, "i1": j, "k": k, "t": float(t[k]), "dmin": float(d[k])})
        i = j + 1
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="zoomed top view per mission corner")
    ap.add_argument("--run", default="")
    ap.add_argument("--tag", default="")
    ap.add_argument("--half", type=float, default=3.5, help="half window around the corner, m")
    ap.add_argument("--gate-m", type=float, default=0.5, help="arrival gate drawn around the corner, m")
    ap.add_argument("--out-prefix", default="")
    a = ap.parse_args()
    if a.run:
        run = os.path.abspath(a.run)
    elif a.tag:
        tag = a.tag if a.tag.startswith("missionA-") else "missionA-" + a.tag
        run = os.path.join(_P.LOGS, tag)
    else:
        cands = sorted([p for p in os.listdir(_P.LOGS) if p.startswith("missionA-")],
                       key=lambda p: os.path.getmtime(os.path.join(_P.LOGS, p)), reverse=True)
        run = os.path.join(_P.LOGS, cands[0])
    tag = os.path.basename(run).replace("missionA-", "")
    pillars, ring, world = load_world(run)
    t, E, N, U, vE, vN = load_truth(run)
    m = U >= 1.0
    t, E, N, vE, vN = t[m], E[m], N[m], vE[m], vN[m]
    tc, cx, cy, cspd = load_cmd(run)
    mc = (tc >= t[0]) & (tc <= t[-1])
    tc, cx, cy, cspd = tc[mc], cx[mc], cy[mc], cspd[mc]
    switches = traj_id_switches(run)
    loops = loop_events(run)
    _figfont.apply()
    prefix = a.out_prefix or os.path.join(run, "corner")
    n_visits_total = 0
    for ci, corner in enumerate(ring, 1):
        vs = visits(t, E, N, corner)
        if not vs:
            print("corner %d: no visit recorded" % ci)
            continue
        # one window per corner: expand to cover every visit's +-half (they are close in xy)
        cx0, cy0 = corner
        fig, ax = plt.subplots(figsize=(7.2, 7.2), dpi=160)
        fig.patch.set_facecolor("#0f1116")
        ax.set_facecolor("#0f1116")
        # ★ owner rule (2026-11-26): every trunk drawn as its real box PLUS the body-radius and the
        #   map-inflation rounded-square rings, so "did it fly inside the body radius?" is visible.
        _vs_xy = np.column_stack([E[vs[0]["i0"]:vs[0]["i1"] + 1], N[vs[0]["i0"]:vs[0]["i1"] + 1]])
        _z_med = float(np.median(U))
        _mmin, _imin, _obs_handles = OBS.draw_obstacles(
            ax, pillars, _z_med, robot_r=OBS_R, inflation=OBS_INFL, z_split=OBS_ZSPLIT,
            path_xy=_vs_xy, ring_m=OBS_RING_M, zorder=1)
        # the rest of the ring, faint, for orientation
        rx = [c[0] for c in ring] + [ring[0][0]]
        ry = [c[1] for c in ring] + [ring[0][1]]
        ax.plot(rx, ry, "--", color="#f59e0b", lw=0.9, alpha=0.5, zorder=2)
        for k, c in enumerate(ring, 1):
            ax.plot([c[0]], [c[1]], marker="D", ms=6, color="#fbbf24", zorder=6)
            ax.annotate(str(k), (c[0], c[1]), textcoords="offset points", xytext=(5, 5),
                        color="#fbbf24", fontsize=8, zorder=7)
        ax.add_patch(Circle(corner, a.gate_m, facecolor="none", edgecolor="#a3e635", ls=":",
                            lw=1.2, zorder=3))
        cap = ["corner %d  (%.1f, %.1f) m   gate %.1f m" % (ci, corner[0], corner[1], a.gate_m)]
        for li, v in enumerate(vs):
            sel = slice(v["i0"], v["i1"] + 1)
            tt = t[sel]
            color = ["#38bdf8", "#22d3ee", "#a78bfa", "#f472b6"][li % 4]
            # truth path coloured by the EXACT body-edge margin; a faint time-graded line keeps direction
            _xy_sel = np.column_stack([E[sel], N[sel]])
            _mmap = OBS.body_margin(_xy_sel, pillars, U[sel], OBS_R, OBS_ZSPLIT)
            _hc = OBS.plot_clearance_path(ax, _xy_sel, _mmap, lw=2.4, zorder=5)
            ax.plot(_xy_sel[:, 0], _xy_sel[:, 1], "-", color=color, alpha=0.35, lw=0.8, zorder=4)
            # direction arrow every ~1.2 s
            last = -1e9
            for q in range(1, len(tt) - 1):
                if tt[q] - last < 1.2:
                    continue
                last = tt[q]
                ax.annotate("", xy=(E[sel][q + 1], N[sel][q + 1]), xytext=(E[sel][q - 1], N[sel][q - 1]),
                            arrowprops=dict(arrowstyle="-|>", color=color, lw=1.0), zorder=5)
            # command in the same time window
            mm = (tc >= tt[0]) & (tc <= tt[-1])
            if mm.sum() > 3:
                ax.plot(cx[mm], cy[mm], "--", color="#fb923c", lw=1.0, alpha=0.9, zorder=3)
            tmin, tmax = float(tt[0]), float(tt[-1])
            sp_truth = float(np.min(np.hypot(vE[sel], vN[sel])))
            sp_cmd = float(np.min(cspd[(tc >= tmin - 1.0) & (tc <= tmax + 1.0)])) if (
                (tc >= tmin - 1.0) & (tc <= tmax + 1.0)).sum() else float("nan")
            ov = [e for e in loops if e[0] <= tmax and e[1] >= tmin]
            _mmin_v = float(np.nanmin(_mmap))
            cap.append("visit %d: t=%.1f..%.1f s, closest %.2f m, min speed truth %.2f / cmd %.2f m/s, "
                       "body margin min %+.3f m%s"
                       % (li + 1, tmin, tmax, v["dmin"], sp_truth, sp_cmd,
                          _mmin_v, ", LOOP" if ov else ""))
            n_visits_total += 1
        # trajectory-id handovers inside the window (new goal sent here)
        for (th, id0, id1) in switches:
            if t[0] <= th <= t[-1]:
                j = int(np.argmin(np.abs(tc - th)))
                if np.hypot(cx[j] - corner[0], cy[j] - corner[1]) < a.half + 1.0:
                    ax.plot([cx[j]], [cy[j]], "o", ms=4, mfc="none", mec="#f43f5e", mew=1.2, zorder=8)
        ax.set_xlim(corner[0] - a.half, corner[0] + a.half)
        ax.set_ylim(corner[1] - a.half, corner[1] + a.half)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("E (m)", color="#d1d5db", fontsize=9)
        ax.set_ylabel("N (m)", color="#d1d5db", fontsize=9)
        ax.grid(True, color="#374151", ls=":", lw=0.6, alpha=0.7)
        ax.tick_params(colors="#9ca3af", labelsize=8)
        for s in ax.spines.values():
            s.set_color("#4b5563")
        ax.set_title("missionA-%s   corner %d   ZOOM +-%.1f m" % (tag, ci, a.half),
                     color="#e5e7eb", fontsize=10, loc="left")
        # per-visit numbers in a box INSIDE the axes (a long title got cut off on the right)
        ax.text(0.01, 0.01, "\n".join(cap), transform=ax.transAxes, color="#e5e7eb", fontsize=6.6,
                va="bottom", ha="left", zorder=9,
                bbox=dict(facecolor="#111827", edgecolor="#374151", alpha=0.75, pad=3))
        fig.text(0.01, 0.005, "truth: dark->bright = time, arrows = direction | dashed orange = command | "
                              "red ring = new goal | green dotted = arrival gate",
                 color="#9ca3af", fontsize=6.5)
        fig.tight_layout(rect=(0, 0.02, 1, 1))
        out = "%s-%d.png" % (prefix, ci)
        fig.savefig(out, facecolor=fig.get_facecolor())
        plt.close(fig)
        print("corner %d: %d visit(s) -> %s" % (ci, len(vs), out))
    print("total visits drawn: %d" % n_visits_total)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
