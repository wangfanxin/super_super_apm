#!/usr/bin/env python3
"""plot_flight_topdown.py -- top view (bird's-eye) of one recorded sortie.

    python3 scripts/plot_flight_topdown.py --run .runs/missionA-phase4p-01-01
    python3 scripts/plot_flight_topdown.py --tag phase4p-01-01 --out /tmp/top.png
    python3 scripts/plot_flight_topdown.py --tag phase4p-01-01 --z-from 1.9 --z-to 2.1

What is drawn, and nothing else (so the figure answers one question: "where did it fly?"):
  * the pillars of the sortie's world, as the 0.2 m squares they are at the 2 m mission height
    (the box section below z_split; same geometry W4 grades against),
  * the mission ring (the world's own `mission.corners`, closed) and the pad,
  * the TRUTH (SITL model, `tap.csv`, E/N) as the thick line,
  * the COMMAND stream (`setpoints.csv` cmd_*, L->W) dashed,
  * the LIO estimate (`lio.csv`, L->W) thin,
  * start (o) / end (x) markers and the first/last truth time.
No acceptance numbers are computed here; the run's own accept.json remains the authority.
"""
from __future__ import annotations

import argparse
import csv
import glob
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import _paths as _P  # noqa: E402
import _figfont  # noqa: E402

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

try:
    import yaml
except Exception:  # noqa: BLE001
    yaml = None


def resolve_run(args):
    if args.run:
        return os.path.abspath(args.run)
    tag = args.tag
    if not tag:
        cands = sorted(glob.glob(os.path.join(_P.LOGS, "missionA-*")),
                       key=lambda p: os.path.getmtime(p), reverse=True)
        cands = [c for c in cands if os.path.isfile(os.path.join(c, "tap.csv"))]
        if not cands:
            raise SystemExit("no recorded sortie under %s" % _P.LOGS)
        return cands[0]
    if not tag.startswith("missionA-"):
        tag = "missionA-" + tag
    return os.path.join(_P.LOGS, tag)


def read_world(path):
    if not (yaml and os.path.isfile(path)):
        return None, None, None, args_zsplit_default()
    d = yaml.safe_load(open(path))
    pil = d.get("pillars") or []
    ring = ((d.get("mission") or {}).get("corners") or [])
    pad = d.get("pad") or {}
    zsplit = float(((d.get("W4") or {}).get("z_split")) or 2.1)
    # the mission ring repeats its corners for every lap; keep ONE cycle for the drawing
    ring2 = []
    for c in ring:
        x, y = float(c[0]), float(c[1])
        if ring2 and abs(ring2[0][0] - x) < 1e-9 and abs(ring2[0][1] - y) < 1e-9 and len(ring2) >= 3:
            break                                    # back to the first corner -> one lap closed
        if not ring2 or (abs(ring2[-1][0] - x) > 1e-9 or abs(ring2[-1][1] - y) > 1e-9):
            ring2.append((x, y))
    return pil, ring2, pad, zsplit


def args_zsplit_default():
    return 2.1


def load_truth(run):
    rows = list(csv.DictReader(open(os.path.join(run, "tap.csv"))))
    tcol = "t_sim_us" if rows and "t_sim_us" in rows[0] else "t_us"
    t = np.array([float(r[tcol]) * 1e-6 for r in rows])
    p = np.column_stack([np.array([float(r["pe"]) for r in rows]),
                         np.array([float(r["pn"]) for r in rows]),
                         np.array([-float(r["pd"]) for r in rows])])
    return t, p


def load_cmd(run):
    path = os.path.join(run, "setpoints.csv")
    rows = list(csv.DictReader(open(path)))
    t = np.array([float(r["t_sim_us"]) * 1e-6 for r in rows])
    p = np.column_stack([np.array([float(r["cmd_x"]) for r in rows]),
                         np.array([float(r["cmd_y"]) for r in rows])])
    return t, p


def load_lio(run):
    path = os.path.join(run, "lio.csv")
    rows = list(csv.DictReader(open(path)))
    t = np.array([float(r["t_us"]) * 1e-6 for r in rows])
    # L -> W:  W = (E, N) = (-y_l, x_l)
    p = np.column_stack([-np.array([float(r["y"]) for r in rows]),
                         np.array([float(r["x"]) for r in rows])])
    return t, p


def main() -> int:
    ap = argparse.ArgumentParser(description="top view of one recorded sortie")
    ap.add_argument("--run", default="")
    ap.add_argument("--tag", default="")
    ap.add_argument("--world", default="")
    ap.add_argument("--out", default="")
    ap.add_argument("--per-lap", action="store_true",
                    help="write ONE figure PER LAP (topdown-lap1.png, topdown-lap2.png, ...) instead of one "
                         "figure for the whole flight.  A lap boundary = the truth passing within 1.2 m of "
                         "corner 1 (the same measured rule as scripts/plot_hex_laps.py).")
    a = ap.parse_args()
    run = resolve_run(a)
    if not os.path.isfile(os.path.join(run, "tap.csv")):
        raise SystemExit("no tap.csv in %s" % run)
    world = a.world
    if not world:
        world = os.path.join(ROOT, "sim", "worlds", "forest_hex12.yaml")
        try:
            import json
            man = json.load(open(os.path.join(run, "manifest.json")))
            world = (man.get("world") or {}).get("path") or world
        except Exception:  # noqa: BLE001
            pass
    pil, ring, pad, zsplit = read_world(world)
    t, truth = load_truth(run)
    tc, cmd = load_cmd(run)
    tl, lio = load_lio(run)
    # FLIGHT WINDOW: the pad-parked tail (before arm / after land) is not "the flight" -- trim it from the
    # figure using the sortie's own phase log (takeoff..land).  Falls back to the whole recording.
    win = None
    try:
        ph = list(csv.DictReader(open(os.path.join(run, "phase.csv"))))
        t0 = t1 = None
        for r in ph:
            name = (r.get("phase") or "").strip()
            ts = float(r.get("t_sim_us") or 0) * 1e-6
            if name in ("takeoff", "preflight", "align", "arm") and t0 is None:
                t0 = ts
            elif name in ("land", "done", "disarm"):
                t1 = ts
        if t0 is not None and t1 is not None and t1 > t0:
            win = (t0, t1)
    except Exception:                               # noqa: BLE001
        win = None
    if win is not None:
        m = (t >= win[0]) & (t <= win[1]) & (truth[:, 2] >= 1.0)
        if m.sum() > 100:
            t, truth = t[m], truth[m]
        m = (tc >= win[0]) & (tc <= win[1])
        if m.sum() > 20:
            tc, cmd = tc[m], cmd[m]
        m = (tl >= win[0]) & (tl <= win[1])
        if m.sum() > 20:
            tl, lio = tl[m], lio[m]
    # per-lap boundaries: [start_of_flight, corner1, corner1, ..., end]; the first and last segments are the
    # LAUNCH (pad -> corner 1) and the RETURN (corner 1 -> pad/landing), not laps
    bounds = [0, len(truth) - 1]
    if ring and len(ring) >= 3:
        d1 = np.hypot(truth[:, 0] - ring[0][0], truth[:, 1] - ring[0][1])
        marks = [i for i in range(1, len(d1)) if d1[i] < 1.2 and d1[i - 1] >= 1.2]
        if marks:
            bounds = [0] + marks + [len(truth) - 1]

    # is the tour's final point the pad (a landing return) or a mission corner (mission A ends at a corner)?
    end_near_pad = False
    if pad:
        end_near_pad = float(np.hypot(truth[-1, 0] - float(pad.get("x", 0.0)),
                                      truth[-1, 1] - float(pad.get("y", 0.0)))) < 2.0

    def draw_truth_laps(ax, lo, hi, colour_by_lap):
        lap_cols = ["#38bdf8", "#22d3ee", "#a78bfa", "#f472b6"]
        if colour_by_lap and len(bounds) > 2:
            for li in range(len(bounds) - 1):
                a0, a1 = max(bounds[li], lo), min(bounds[li + 1], hi)
                if a1 - a0 < 5:
                    continue
                name = ("launch (pad -> corner1)" if li == 0 else
                        ("lap %d" % li if li < len(bounds) - 2 else
                         ("return (landing)" if end_near_pad else "lap %d" % li)))
                ax.plot(truth[a0:a1 + 1, 0], truth[a0:a1 + 1, 1], "-",
                        color=lap_cols[li % len(lap_cols)], lw=2.2, zorder=4, label="truth " + name)
        else:
            ax.plot(truth[lo:hi + 1, 0], truth[lo:hi + 1, 1], "-", color="#38bdf8", lw=2.2, zorder=4,
                    label="truth (SITL model, flight window)")

    _figfont.apply()
    fig, ax = plt.subplots(figsize=(9.0, 9.0), dpi=170)
    ax.set_facecolor("#0f1116")
    fig.patch.set_facecolor("#0f1116")
    # pillars: the 0.2 m square section they have at the mission height
    if pil:
        for p in pil:
            try:
                r = float(p.get("r", 0.1))
                x, y = float(p["x"]), float(p["y"])
            except Exception:  # noqa: BLE001
                continue
            ax.add_patch(Rectangle((x - r, y - r), 2 * r, 2 * r, facecolor="#6b7280",
                                   edgecolor="none", alpha=0.8, zorder=1))
    if ring:
        rx = [c[0] for c in ring] + [ring[0][0]]
        ry = [c[1] for c in ring] + [ring[0][1]]
        ax.plot(rx, ry, "--", color="#f59e0b", lw=1.2, alpha=0.9, zorder=2, label="mission ring")
        for i, c in enumerate(ring):
            ax.plot([c[0]], [c[1]], marker="D", ms=5, color="#fbbf24", zorder=5)
            ax.annotate(str(i + 1), (c[0], c[1]), textcoords="offset points", xytext=(4, 4),
                        color="#fbbf24", fontsize=7, zorder=6)
    if pad:
        px, py = float(pad.get("x", 0.0)), float(pad.get("y", 0.0))
        pr = float(pad.get("radius", 0.35))
        ax.add_patch(plt.Circle((px, py), pr, facecolor="none", edgecolor="#a3e635",
                                lw=1.2, ls=":", zorder=3, label="pad"))
    draw_truth_laps(ax, 0, len(truth) - 1, colour_by_lap=True)
    ax.plot(truth[0, 0], truth[0, 1], "o", ms=9, mfc="none", mec="#38bdf8", mew=1.6, zorder=7,
            label="start (1 m alt)")
    ax.plot(truth[-1, 0], truth[-1, 1], "X", ms=9, color="#f43f5e", zorder=7, label="end")
    ax.plot(cmd[:, 0], cmd[:, 1], "--", color="#fb923c", lw=1.1, alpha=0.9, zorder=3, label="command (plan)")
    ax.plot(lio[:, 0], lio[:, 1], "-", color="#22d3ee", lw=0.8, alpha=0.55, zorder=3, label="LIO estimate")
    span = max(np.ptp(truth[:, 0]), np.ptp(truth[:, 1]), 12.0)
    ax.set_xlim(min(truth[:, 0].min(), truth[:, 1].min(), -span / 2) - 1.2,
                max(truth[:, 0].max(), truth[:, 1].max(), span / 2) + 1.2)
    ax.set_ylim(min(truth[:, 1].min(), -span / 2) - 1.2, max(truth[:, 1].max(), span / 2) + 1.2)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("E (m)")
    ax.set_ylabel("N (m)")
    ax.grid(True, color="#374151", ls=":", lw=0.6, alpha=0.7)
    ax.tick_params(colors="#9ca3af", labelsize=8)
    for s in ax.spines.values():
        s.set_color("#4b5563")
    tag = os.path.basename(run).replace("missionA-", "")
    t0 = t[np.argsort(t)[0]]
    dur = float(t.max() - t.min())
    ax.set_title("missionA-%s  top view (E-N, z_split=%.1f m)\n"
                 "truth %.1f s, %d samples; command %d; LIO %d   |   world: %s"
                 % (tag, zsplit, dur, len(truth), len(cmd), len(lio), os.path.basename(world)),
                 color="#e5e7eb", fontsize=10, pad=10)
    leg = ax.legend(loc="upper left", fontsize=7.5, framealpha=0.25, facecolor="#111827",
                    edgecolor="#374151", labelcolor="#e5e7eb")
    if a.per_lap and len(bounds) > 2:
        plt.close(fig)
        outs = []
        for li in range(len(bounds) - 1):
            lo, hi = bounds[li], bounds[li + 1]
            if hi - lo < 5:
                continue
            name = ("launch" if li == 0 else
                    ("lap%d" % li if li < len(bounds) - 2 else
                     ("return" if end_near_pad else "lap%d" % li)))
            fig2, ax2 = plt.subplots(figsize=(8.0, 8.0), dpi=170)
            ax2.set_facecolor("#0f1116")
            fig2.patch.set_facecolor("#0f1116")
            if pil:
                for p in pil:
                    try:
                        r = float(p.get("r", 0.1)); x, y = float(p["x"]), float(p["y"])
                    except Exception:  # noqa: BLE001
                        continue
                    ax2.add_patch(Rectangle((x - r, y - r), 2 * r, 2 * r, facecolor="#6b7280",
                                            edgecolor="none", alpha=0.8, zorder=1))
            if ring:
                rx2 = [c[0] for c in ring] + [ring[0][0]]
                ry2 = [c[1] for c in ring] + [ring[0][1]]
                ax2.plot(rx2, ry2, "--", color="#f59e0b", lw=1.2, alpha=0.9, zorder=2, label="mission ring")
                for i, c in enumerate(ring):
                    ax2.plot([c[0]], [c[1]], marker="D", ms=5, color="#fbbf24", zorder=5)
                    ax2.annotate(str(i + 1), (c[0], c[1]), textcoords="offset points", xytext=(4, 4),
                                 color="#fbbf24", fontsize=7, zorder=6)
            if pad:
                px, py = float(pad.get("x", 0.0)), float(pad.get("y", 0.0))
                ax2.add_patch(plt.Circle((px, py), float(pad.get("radius", 0.35)), facecolor="none",
                                         edgecolor="#a3e635", lw=1.2, ls=":", zorder=3, label="pad"))
            ax2.plot(truth[lo:hi + 1, 0], truth[lo:hi + 1, 1], "-", color="#38bdf8", lw=2.4, zorder=4,
                     label="truth (SITL model)")
            # the command stream inside the same time window
            m = (tc >= t[lo]) & (tc <= t[hi])
            if m.sum() > 5:
                ax2.plot(cmd[m, 0], cmd[m, 1], "--", color="#fb923c", lw=1.0, alpha=0.85, zorder=3,
                         label="command (plan)")
            ax2.plot(truth[lo, 0], truth[lo, 1], "o", ms=8, mfc="none", mec="#38bdf8", mew=1.6, zorder=7)
            ax2.plot(truth[hi, 0], truth[hi, 1], "X", ms=8, color="#f43f5e", zorder=7, label="end")
            ax2.set_xlim(-12.4, 12.4)
            ax2.set_ylim(-11.4, 11.4)
            ax2.set_aspect("equal", adjustable="box")
            ax2.set_xlabel("E (m)")
            ax2.set_ylabel("N (m)")
            ax2.grid(True, color="#374151", ls=":", lw=0.6, alpha=0.7)
            ax2.tick_params(colors="#9ca3af", labelsize=8)
            for s in ax2.spines.values():
                s.set_color("#4b5563")
            ax2.set_title("missionA-%s  %s  top view (E-N)\nflight t = %.1f .. %.1f s (sim), "
                          "%.1f s, %d truth samples"
                          % (tag, name.replace("_", " "), float(t[lo]), float(t[hi]),
                             float(t[hi] - t[lo]), hi - lo + 1),
                          color="#e5e7eb", fontsize=10, pad=10)
            ax2.legend(loc="upper left", fontsize=7.5, framealpha=0.25, facecolor="#111827",
                       edgecolor="#374151", labelcolor="#e5e7eb")
            base = a.out or os.path.join(run, "topdown")
            if base.endswith(".png"):
                base = base[:-4]
            out2 = "%s-%s.png" % (base, name)
            fig2.tight_layout()
            fig2.savefig(out2, facecolor=fig2.get_facecolor())
            plt.close(fig2)
            outs.append(out2)
            print("per-lap figure written: %s  (%.1f s, t=%.1f..%.1f)" % (out2, float(t[hi] - t[lo]),
                                                                        float(t[lo]), float(t[hi])))
        return 0
    out = a.out or os.path.join(run, "topdown.png")
    fig.tight_layout()
    fig.savefig(out, facecolor=fig.get_facecolor())
    print("top view written: %s" % out)
    print("  truth samples %d (%.1f s), command %d, LIO %d, pillars %s, ring %s"
          % (len(truth), dur, len(cmd), len(lio), len(pil or []), len(ring or [])))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
