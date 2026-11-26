#!/usr/bin/env python3
"""plot_clearance_map.py -- the flight coloured by the EXACT body-edge clearance to the pillars.

Owner's rule (2026-11-26): a top-down plot must show the obstacle radii that matter, so "did it fly
inside the body radius?" is answerable by eye:

  * grey square            trunk box at the 2.0 m mission height (0.2 x 0.2 m, world z_split = 2.1 m)
  * red dashed rounded     trunk (+) body circle (planner robot_r = 0.10 m): body-edge contact ring
  * yellow dotted rounded  body ring (+) map inflation (inflation_step*inflation_resolution = 0.2 m):
                           the keep-out the planner's inflated map enforces
  * truth path colours     red = body edge inside the trunk, orange = touching band, yellow = inside
                           the inflation band, cyan = free (exact surface_dist(box) - robot_r)
  * annotation             W4's own point-vs-box number (accept.py --body box) next to the body margin

Usage: python3 scripts/plot_clearance_map.py --tag phase23c-01-01
"""
from __future__ import annotations

import argparse
import csv
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import _paths as _P  # noqa: E402
import _figfont  # noqa: F401,E402
import _plot_obstacles as OBS  # noqa: E402

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def load_world(run):
    import yaml
    world = os.path.join(ROOT, "sim", "worlds", "forest_hex12.yaml")
    try:
        import json
        man = json.load(open(os.path.join(run, "manifest.json")))
        world = (man.get("world") or {}).get("path") or world
    except Exception:  # noqa: BLE001
        pass
    with open(world, encoding="utf-8") as fh:
        w = yaml.safe_load(fh)
    pil = w.get("pillars") or []
    ring = [(float(c[0]), float(c[1])) for c in w["mission"]["corners"][:6]]
    z_split = float((w.get("pillar_geometry") or {}).get("z_split", 2.1))
    return pil, ring, z_split, os.path.basename(world)


def main() -> int:
    ap = argparse.ArgumentParser(description="whole flight coloured by exact body-edge clearance")
    ap.add_argument("--run", default="")
    ap.add_argument("--tag", default="")
    ap.add_argument("--outdir", default=os.path.join(ROOT, "docs", "evidence"))
    ap.add_argument("--out-prefix", default="")
    a = ap.parse_args()
    run = os.path.abspath(a.run) if a.run else _P.sortie(a.tag)
    tag = os.path.basename(run).replace("missionA-", "")
    short = tag[:-6] if tag.endswith("-01-01") else tag
    pil, ring, z_split, world_name = load_world(run)
    robot_r, inflation, _ = OBS.geometry(root=ROOT)

    tap = list(csv.DictReader(open(os.path.join(run, "tap.csv"), encoding="utf-8")))
    t = np.array([float(x["t_sim_us"]) * 1e-6 for x in tap])
    E = np.array([float(x["pe"]) for x in tap])
    N = np.array([float(x["pn"]) for x in tap])
    U = -np.array([float(x["pd"]) for x in tap])
    m = U >= 1.0
    t, E, N, U = t[m], E[m], N[m], U[m]
    xy = np.column_stack([E, N])
    margin = OBS.body_margin(xy, pil, U, robot_r, z_split)
    i_min = int(np.argmin(margin))
    w4 = OBS.surface_dist(xy, pil, U, z_split).min()
    fraction = float(np.mean(margin < 0.0))

    _figfont.apply()
    fig, ax = plt.subplots(figsize=(9.6, 9.6), dpi=170)
    fig.patch.set_facecolor("#0f1116")
    ax.set_facecolor("#0f1116")
    _, _, handles = OBS.draw_obstacles(ax, pil, float(np.median(U)), robot_r=robot_r, inflation=inflation,
                                       z_split=z_split, path_xy=xy, ring_m=2.2, zorder=1)
    h_all = handles + OBS.clearance_legend()
    # the path, coloured by the exact body-edge margin
    from matplotlib.collections import LineCollection
    segs = np.stack([xy[:-1], xy[1:]], axis=1)
    cols = [OBS.color_for(0.5 * (margin[k] + margin[k + 1])) for k in range(len(segs))]
    ax.add_collection(LineCollection(segs, colors=cols, linewidths=2.6, zorder=4))
    h_all.append(plt.Line2D([], [], color="#e5e7eb", lw=2.6, label="truth（颜色=机身边缘到躯干距离）"))
    # corner markers and the mission ring
    if ring:
        rx = [c[0] for c in ring] + [ring[0][0]]
        ry = [c[1] for c in ring] + [ring[0][1]]
        ax.plot(rx, ry, "--", color="#f59e0b", lw=1.0, alpha=0.7, zorder=2, label="mission ring")
        for k, c in enumerate(ring, 1):
            ax.plot([c[0]], [c[1]], marker="D", ms=5, color="#fbbf24", zorder=6)
            ax.annotate("%d" % k, (c[0], c[1]), textcoords="offset points", xytext=(4, 4),
                        color="#fbbf24", fontsize=7, zorder=7)
    # mark the worst body margin and the W4 point
    ax.plot([E[i_min]], [N[i_min]], "o", ms=11, mfc="none", mec="#f43f5e", mew=1.8, zorder=8)
    ax.annotate("body margin min %+.3f m\nW4 (point-vs-box) %+.3f m\nat (%.2f, %.2f), t=%.1f s"
                % (margin[i_min], w4, E[i_min], N[i_min], t[i_min]),
                (E[i_min], N[i_min]), textcoords="offset points", xytext=(8, -34),
                color="#fecdd3", fontsize=8, zorder=9,
                bbox=dict(boxstyle="round,pad=0.25", fc="#111827", ec="#f43f5e", alpha=0.85))
    ax.set_xlim(E.min() - 1.5, E.max() + 1.5)
    ax.set_ylim(N.min() - 1.5, N.max() + 1.5)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("E (m)", color="#d1d5db")
    ax.set_ylabel("N (m)", color="#d1d5db")
    ax.grid(True, color="#374151", ls=":", lw=0.6, alpha=0.6)
    ax.tick_params(colors="#9ca3af", labelsize=8)
    for s in ax.spines.values():
        s.set_color("#4b5563")
    ax.legend(handles=h_all, fontsize=7, loc="lower left", framealpha=0.25, facecolor="#111827",
              edgecolor="#374151", labelcolor="#e5e7eb", ncol=1)
    ax.set_title("%s  clearance map: trunk box + body circle (robot_r %.2f) + map inflation (%.2f m)\n"
                 "body-edge margin min %+.3f m  |  W4 point-vs-box %+.3f m  |  %s"
                 % (short, robot_r, inflation, margin[i_min], w4, world_name),
                 color="#e5e7eb", fontsize=9.5, loc="left")
    fig.tight_layout()
    os.makedirs(a.outdir, exist_ok=True)
    out = a.out_prefix or os.path.join(a.outdir, short + "-clearance.png")
    fig.savefig(out, facecolor=fig.get_facecolor())
    plt.close(fig)
    print("written:", out)
    print("  body-edge margin min %+.3f m at (%.2f, %.2f) t=%.1f s; W4 point-vs-box %+.3f m; %.2f %% samples body margin < 0"
          % (margin[i_min], E[i_min], N[i_min], t[i_min], w4, 100 * fraction))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
