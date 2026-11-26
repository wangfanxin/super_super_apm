#!/usr/bin/env python3
"""plot_flight_timeseries.py -- the xyz-t and vxyz-t figures: what the plan asked vs what the aircraft did.

    python3 scripts/plot_flight_timeseries.py --tag phase4p-01-01
    python3 scripts/plot_flight_timeseries.py --run .runs/missionA-xxx --events ev.json

Three figures:
  xyz-t   E, N, U against time                (truth vs command)
  vnorm-t |v| = sqrt(vx^2+vy^2+vz^2) against time, with |v_xy| and signed v_z below
          (owner, 2026-11-26: "把 vxyz 改成速度平方和的根(2norm)来看" -- the per-axis traces hide how the
           speed collapses at a corner; the 2-norm is the number the controller and the acceptance rows use)
  speed-t |v_xy|, |v_z| against time          (same streams, the numbers the controller lives on)
The vertical dashed lines are the measured lap boundaries (truth passing within 1.2 m of corner 1, same rule
as the top view); red bands are the LOOP events from scripts/analyze_loops.py, so "the aircraft circled here"
can be read against the commands it was given at that instant.
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

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def _rows(path):
    with open(path, encoding="utf-8", errors="replace") as fh:
        return list(csv.DictReader(fh))


def load_truth(run):
    rows = _rows(os.path.join(run, "tap.csv"))
    tcol = "t_sim_us" if rows and "t_sim_us" in rows[0] else "t_us"
    t = np.array([float(r[tcol]) * 1e-6 for r in rows])
    # truth is NED on the wire: E = pe, N = pn, U = -pd ; velocity likewise
    E = np.array([float(r["pe"]) for r in rows])
    N = np.array([float(r["pn"]) for r in rows])
    U = np.array([-float(r["pd"]) for r in rows])
    vE = np.array([float(r["ve"]) for r in rows])
    vN = np.array([float(r["vn"]) for r in rows])
    vU = np.array([-float(r["vd"]) for r in rows])
    o = np.argsort(t)
    return t[o], E[o], N[o], U[o], vE[o], vN[o], vU[o]


def load_cmd(run):
    rows = _rows(os.path.join(run, "setpoints.csv"))
    t = np.array([float(r["t_sim_us"]) * 1e-6 for r in rows])
    return (t,
            np.array([float(r["cmd_x"]) for r in rows]),
            np.array([float(r["cmd_y"]) for r in rows]),
            np.array([float(r["cmd_z"]) for r in rows]),
            np.array([float(r["vel_x"]) for r in rows]),
            np.array([float(r["vel_y"]) for r in rows]),
            np.array([float(r["vel_z"]) for r in rows]))


def lap_boundaries(run, truth_t, E, N):
    """Times where the truth passes within 1.2 m of corner 1 (the same measured rule as the top views)."""
    world = os.path.join(ROOT, "sim", "worlds", "forest_hex12.yaml")
    try:
        man = json.load(open(os.path.join(run, "manifest.json")))
        world = (man.get("world") or {}).get("path") or world
    except Exception:  # noqa: BLE001
        pass
    try:
        import yaml
        d = yaml.safe_load(open(world))
        corners = (d.get("mission") or {}).get("corners") or []
    except Exception:  # noqa: BLE001
        return []
    if not corners:
        return []
    c1 = (float(corners[0][0]), float(corners[0][1]))
    d1 = np.hypot(E - c1[0], N - c1[1])
    marks = []
    for i in range(1, len(d1)):
        if d1[i] < 1.2 and d1[i - 1] >= 1.2:
            marks.append(float(truth_t[i]))
    return marks


def events_from(run, ev_path):
    if ev_path and os.path.isfile(ev_path):
        d = json.load(open(ev_path))
        return [(e["t0"], e["t1"]) for e in d.get("events", [])]
    # fall back: run the detector in-process (cheap)
    try:
        import analyze_loops as A
        t, P, V = A.load(run)
        m = P[:, 2] >= 1.0
        evs = A.detect_loops(t[m], P[m])
        return [(e["t0"], e["t1"]) for e in evs]
    except Exception:  # noqa: BLE001
        return []


def style_ax(ax, ylabel, title=None):
    ax.set_facecolor("#0f1116")
    ax.grid(True, color="#374151", ls=":", lw=0.6, alpha=0.7)
    ax.tick_params(colors="#9ca3af", labelsize=8)
    for s in ax.spines.values():
        s.set_color("#4b5563")
    ax.set_ylabel(ylabel, color="#d1d5db", fontsize=9)
    if title:
        ax.set_title(title, color="#e5e7eb", fontsize=9, loc="left")


def draw(ax, t, y_truth, t_cmd, y_cmd, ylabel, events, laps, title=None, speed=False):
    style_ax(ax, ylabel, title)
    for e0, e1 in events:
        ax.axvspan(e0, e1, color="#ef4444", alpha=0.22, lw=0, zorder=0)
    for lt in laps:
        ax.axvline(lt, color="#f59e0b", ls="--", lw=0.9, alpha=0.7, zorder=1)
    ax.plot(t, y_truth, "-", color="#38bdf8", lw=1.3, zorder=3, label="truth (SITL model)")
    ax.plot(t_cmd, y_cmd, "--", color="#fb923c", lw=1.0, alpha=0.9, zorder=2, label="command (plan)")


def main() -> int:
    ap = argparse.ArgumentParser(description="xyz-t / vxyz-t / speed-t: plan vs truth")
    ap.add_argument("--run", default="")
    ap.add_argument("--tag", default="")
    ap.add_argument("--events", default="", help="JSON from scripts/analyze_loops.py")
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
    t, E, N, U, vE, vN, vU = load_truth(run)
    tc, cx, cy, cz, cvx, cvy, cvz = load_cmd(run)
    # flight window (above 1 m) so the parked tail does not flatten the interesting part
    m = U >= 1.0
    t, E, N, U, vE, vN, vU = t[m], E[m], N[m], U[m], vE[m], vN[m], vU[m]
    mc = (tc >= t[0] - 1.0) & (tc <= t[-1] + 1.0)
    tc, cx, cy, cz, cvx, cvy, cvz = tc[mc], cx[mc], cy[mc], cz[mc], cvx[mc], cvy[mc], cvz[mc]
    laps = lap_boundaries(run, t, E, N)
    events = events_from(run, a.events)
    _figfont.apply()
    prefix = a.out_prefix or os.path.join(run, "timeseries")

    # ---- xyz-t ----
    fig, axs = plt.subplots(3, 1, figsize=(13, 8.2), sharex=True, dpi=150)
    fig.patch.set_facecolor("#0f1116")
    draw(axs[0], t, E, tc, cx, "E (m)", events, laps)
    draw(axs[1], t, N, tc, cy, "N (m)", events, laps)
    draw(axs[2], t, U, tc, cz, "U (m)", events, laps)
    axs[2].set_xlabel("time (s, sim clock)", color="#d1d5db")
    axs[0].legend(loc="upper left", fontsize=7.5, framealpha=0.25, facecolor="#111827",
                  edgecolor="#374151", labelcolor="#e5e7eb")
    fig.suptitle("missionA-%s  position vs time (truth solid, command dashed) — red bands = loop events, "
                 "orange dashed = lap boundary" % tag, color="#e5e7eb", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    p1 = prefix + "-xyz-t.png"
    fig.savefig(p1, facecolor=fig.get_facecolor())
    plt.close(fig)
    print("written: %s" % p1)

    # ---- vnorm-t: the 2-norm, not the per-axis traces (owner's round-326 ask) ----
    sp3_t = np.sqrt(vE ** 2 + vN ** 2 + vU ** 2)
    sp3_c = np.sqrt(cvx ** 2 + cvy ** 2 + cvz ** 2)
    spxy_t = np.hypot(vE, vN)
    spxy_c = np.hypot(cvx, cvy)
    fig, axs = plt.subplots(3, 1, figsize=(13, 8.2), sharex=True, dpi=150)
    fig.patch.set_facecolor("#0f1116")
    draw(axs[0], t, sp3_t, tc, sp3_c, "|v| = sqrt(vx^2+vy^2+vz^2)  (m/s)", events, laps)
    draw(axs[1], t, spxy_t, tc, spxy_c, "|v_xy| (m/s)", events, laps)
    draw(axs[2], t, vU, tc, cvz, "v_z (m/s)", events, laps)
    axs[2].set_xlabel("time (s, sim clock)", color="#d1d5db")
    axs[0].legend(loc="upper left", fontsize=7.5, framealpha=0.25, facecolor="#111827",
                  edgecolor="#374151", labelcolor="#e5e7eb")
    fig.suptitle("missionA-%s  SPEED 2-NORM vs time (truth solid, command dashed) — red bands = loop events"
                 % tag, color="#e5e7eb", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    p2 = prefix + "-vnorm-t.png"
    fig.savefig(p2, facecolor=fig.get_facecolor())
    plt.close(fig)
    print("written: %s" % p2)

    # ---- speed-t ----
    spy = np.hypot(vE, vN)
    spc = np.hypot(cvx, cvy)
    fig, ax = plt.subplots(figsize=(13, 4.0), dpi=150)
    fig.patch.set_facecolor("#0f1116")
    draw(ax, t, spy, tc, spc, "|v_xy| (m/s)", events, laps)
    ax.plot(t, np.abs(vU), "-", color="#a78bfa", lw=0.9, alpha=0.8, label="|v_z| truth")
    ax.set_xlabel("time (s, sim clock)", color="#d1d5db")
    ax.legend(loc="upper left", fontsize=7.5, framealpha=0.25, facecolor="#111827",
              edgecolor="#374151", labelcolor="#e5e7eb")
    ax.set_title("missionA-%s  speed vs time" % tag, color="#e5e7eb", fontsize=10, loc="left")
    fig.tight_layout()
    p3 = prefix + "-speed-t.png"
    fig.savefig(p3, facecolor=fig.get_facecolor())
    plt.close(fig)
    print("written: %s" % p3)
    print("laps at: %s" % ", ".join("%.1f" % x for x in laps))
    print("loop events: %s" % ", ".join("%.1f-%.1f" % e for e in events))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
