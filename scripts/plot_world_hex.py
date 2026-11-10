#!/usr/bin/env python3
"""plot_world_hex.py -- the review figure for the SPEC S0/S2 field, in the language the owner asked for.

LEFT   the field as it will be built and simulated: 0.2 m SQUARES below z_split (the mission flies at 2.0 m,
       i.e. through the squares), the six waypoint cells that carry no pillar marked, the hexagon path, the
       six waypoints and the pad.
RIGHT  the same field with the clearance field shaded, and **the route SUPER's own A* would return** for
       each leg (shortest path on a grid with no map inflation, which is what round 81's config does), with
       that route's narrowest point marked.  That is the opposite of the previous figure, which drew the
       widest-possible route and therefore looked like there was no way through.

    python3 scripts/plot_world_hex.py [--world worlds/forest_v2_hex.yaml] [--out docs/evidence/world-v2-hex.png]
"""
import argparse
import heapq
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                      # noqa: E402
# the review figure is read by the owner, so the labels are Chinese: without an explicit CJK family
# matplotlib renders every glyph as a box (measured: "Glyph 27491 missing from current font").
matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Noto Sans CJK SC",
                                          "WenQuanYi Zen Hei", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False
import numpy as np                                   # noqa: E402
import yaml                                          # noqa: E402
from matplotlib.patches import Rectangle             # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--world", default="worlds/forest_v2_hex.yaml")
ap.add_argument("--out", default="docs/evidence/world-v2-hex.png")
ap.add_argument("--cfg", default="config/super_planner_hex.yaml")
ap.add_argument("--z", type=float, default=None)
a = ap.parse_args()

w = yaml.safe_load(open(a.world, encoding="utf-8"))
P = np.array([[float(q["x"]), float(q["y"])] for q in w["pillars"]])
mis = w["mission"]
geo = w.get("pillar_geometry", {}) or {}
r = float(geo.get("box_half_side_m", 0.10))
z_split = float(geo.get("z_split", 2.1))
z = float(a.z if a.z is not None else mis.get("takeoff_alt", 2.0))
corners = np.array([[float(c[0]), float(c[1])] for c in mis["corners"]])
hex6 = corners[:6]
laps = int(mis.get("laps", 3))
arr = float(mis.get("arrive_sphere_m", 0.5))
vexit = float(mis.get("corner_exit_speed_mps", 4.0))

# the yardsticks the round-81 pipeline actually has (see docs/INTEGRATION.md §56)
import re                                            # noqa: E402
txt = open(a.cfg, encoding="utf-8").read() if os.path.isfile(a.cfg) else ""


def cfg(key, default):
    m = re.search(r"^[ \t]*%s:[ \t]*([0-9.eE+-]+)" % key, txt, re.M)
    return float(m.group(1)) if m else default


robot_r = cfg("robot_r", 0.2)
inf_res = cfg("inflation_resolution", 0.1)
inf_step = cfg("inflation_step", 2)
# The A* searches the INFLATED map (inflation_step x inflation_resolution) on a grid of
# inflation_resolution, so its clearance is the inflation plus up to half a cell.
astar_clear = inf_step * inf_res
astar_clear_hi = astar_clear + inf_res / 2.0

g, HALF = 0.05, 11.0
N = int(round(2 * HALF / g)) + 1
ax = np.arange(N) * g - HALF
X, Y = np.meshgrid(ax, ax, indexing="ij")
DV = np.full(X.shape, np.inf)
for (px, py) in P:
    ox = np.abs(X - px) - r
    oy = np.abs(Y - py) - r
    DV = np.minimum(DV, np.hypot(np.maximum(ox, 0.0), np.maximum(oy, 0.0)))

NEI = [(di, dj) for di in (-1, 0, 1) for dj in (-1, 0, 1) if (di, dj) != (0, 0)]


def idx(x, y):
    return (int(round((x + HALF) / g)), int(round((y + HALF) / g)))


def astar(s, t, tmin):
    free = DV >= tmin
    if not (free[s] and free[t]):
        return None
    h = lambda n: np.hypot(n[0] - t[0], n[1] - t[1]) * g
    openq = [(h(s), 0.0, s)]
    best = {s: 0.0}
    prev = {}
    while openq:
        _, gc, n = heapq.heappop(openq)
        if n == t:
            path = [n]
            while n in prev:
                n = prev[n]
                path.append(n)
            return path[::-1]
        if gc > best.get(n, np.inf) + 1e-12:
            continue
        i, j = n
        for di, dj in NEI:
            ni, nj = i + di, j + dj
            if not (0 <= ni < N and 0 <= nj < N) or not free[ni, nj]:
                continue
            if di and dj and not (free[i + di, j] and free[i, j + dj]):
                continue
            m = (ni, nj)
            c = gc + g * (1.41421356237 if (di and dj) else 1.0)
            if c < best.get(m, np.inf) - 1e-12:
                best[m] = c
                prev[m] = n
                heapq.heappush(openq, (c + h(m), c, m))
    return None


_neigh = int((w.get("v2", {}) or {}).get("keepout_neigh", 0) or 0)
_rmid = float((w.get("mission", {}) or {}).get("radius_m", 5.0) or 5.0)
_side = float(np.hypot(*(hex6[1] - hex6[0]))) if len(hex6) > 1 else float("nan")
fig, (axL, axR) = plt.subplots(1, 2, figsize=(19.0, 9.2))

# ---------------------------------------------------------------- left: the field as built
for (px, py) in P:
    axL.add_patch(Rectangle((px - r, py - r), 2 * r, 2 * r, facecolor="#37474f",
                            edgecolor="none", zorder=2))
# the six waypoint cells carry no pillar (SPEC S2, round 81)
pitch = float(w.get("v2", {}).get("pitch", 1.0))
_keep_cells = ((w.get("v2", {}) or {}).get("keepout_cells_empty")
               or (w.get("v2", {}) or {}).get("waypoint_cells_empty") or [])
for cell in _keep_cells:
    i, j = int(cell[0]), int(cell[1])
    axL.add_patch(Rectangle((i * pitch, j * pitch), pitch, pitch, facecolor="#ffe082",
                            edgecolor="#f9a825", lw=1.0, alpha=0.55, zorder=1))
tour = np.vstack([hex6, hex6[0]])
axL.plot(tour[:, 0], tour[:, 1], "--", color="#1565c0", lw=1.8, zorder=3,
         label="六边形航线（%d 圈）" % laps)
axL.plot(hex6[:, 0], hex6[:, 1], "s", ms=9, mfc="#ffd54f", mec="#5d4037", zorder=4,
         label="航点（每个到 (0,0) = %.3f m）" % _rmid)
axL.plot(0, 0, "o", ms=11, mfc="#e53935", mec="#4e342e", zorder=5, label="停机坪 / 起点 (0,0)")
for k, (cx, cy) in enumerate(hex6):
    axL.annotate("c%d" % (k + 1), (cx, cy), textcoords="offset points", xytext=(6, 6), fontsize=10)
_half = max(9.0, float(np.max(np.abs(P))) + 1.0)
axL.set_xlim(-_half, _half)
axL.set_ylim(-_half, _half)
axL.set_aspect("equal")
axL.grid(alpha=0.25)
axL.set_xlabel("西→东 (m)")
axL.set_ylabel("南→北 (m)")
axL.legend(loc="upper left", fontsize=9, framealpha=0.9)
axL.set_title("规矩 S0/S2：%d 根柱、%.1f m 格、柱 %.2f×%.2f m 的**方柱**（z<%.1f）、柱心格内随机、**零车道**\n"
              "空格 = 航点周围 %s（黄，共 %d 格）；航点 %d 个 = 6 角 × %d 圈；每个角点到 (0,0) = %.3f m，边长 %.3f m"
              % (len(P), pitch, 2 * r, 2 * r, z_split,
                 ("3×3 格" if _neigh else "1 格"), len(_keep_cells), 6 * laps, laps, _rmid, _side),
              fontsize=10.5)

# ---------------------------------------------------------------- right: what the planner sees and flies
im = axR.imshow(np.clip(DV, 0, 0.60).T, origin="lower", extent=[-HALF, HALF, -HALF, HALF],
                cmap="Blues_r", vmin=0.0, vmax=0.60, alpha=0.85, zorder=0)
cb = fig.colorbar(im, ax=axR, fraction=0.035, pad=0.01)
cb.set_label("离柱面净空 d (m)", fontsize=9)

nm = ["pad→c1"] + ["c%d→c%d" % (i + 1, i + 2) for i in range(5)] + ["c6→c1"]
tourw = [(0.0, 0.0)] + [tuple(c) for c in hex6] + [tuple(hex6[0])]
info = []
for k in range(len(tourw) - 1):
    path = astar(idx(*tourw[k]), idx(*tourw[k + 1]), astar_clear)
    if path is None:
        info.append((nm[k], float("nan"), float("nan")))
        continue
    px = [ax[i] for (i, j) in path]
    py = [ax[j] for (i, j) in path]
    axR.plot(px, py, "-", color="#1e88e5", lw=2.0, zorder=4, alpha=0.95)
    dmin = min(DV[q] for q in path)
    ch = min(path, key=lambda q: DV[q])
    info.append((nm[k], dmin, float("nan")))
    axR.plot(ax[ch[0]], ax[ch[1]], "o", ms=5.5, mfc="#e53935", mec="none", zorder=6)
    axR.annotate("%s\n%.2f m" % (nm[k].replace("->", "→"), dmin),
                 (ax[ch[0]], ax[ch[1]]), textcoords="offset points", xytext=(7, -12),
                 fontsize=8.2, color="#b71c1c",
                 bbox=dict(boxstyle="round,pad=0.18", fc="white", ec="#ef9a9a", alpha=0.85, lw=0.8))
for (px_, py_) in P:
    axR.add_patch(Rectangle((px_ - r, py_ - r), 2 * r, 2 * r, facecolor="#eceff1",
                            edgecolor="#546e7a", lw=0.4, zorder=3))
axR.plot(tour[:, 0], tour[:, 1], "--", color="#37474f", lw=1.1, alpha=0.8, zorder=5)
axR.plot(hex6[:, 0], hex6[:, 1], "s", ms=8, mfc="#ffd54f", mec="#5d4037", zorder=6)
axR.plot(0, 0, "o", ms=10, mfc="#e53935", mec="#4e342e", zorder=6)
axR.set_xlim(-_half, _half)
axR.set_ylim(-_half, _half)
axR.set_aspect("equal")
axR.set_xlabel("西→东 (m)")
axR.set_title("规划器视角：底色 = 离柱面净空 d，蓝线 = **A\\* 实际会飞的最短路线**（无地图膨胀，%.2f m 栅格）\n"
              "红点 = 该路线最窄处；A* 判据 %.2f m（地图膨胀 %g×%.1f m）、CIRI 判据 robot_r = %.2f m、接触 = 0"
              % (inf_res, astar_clear, inf_step, inf_res, robot_r), fontsize=10.5)

n_astar = sum(1 for (_, d, _) in info if d == d and d >= astar_clear)
n_ciri = sum(1 for (_, d, _) in info if d == d and d >= robot_r)
txt_ = "  ".join("%s %.2f" % (k, v) for (k, v, _) in info if v == v)
axR.text(0.02, 0.02, "A* 路线最窄处：%s\n能过 A*(%.2f) 的腿 %d/7 · 能过 CIRI(%.2f) 的腿 %d/7"
         % (txt_, astar_clear, n_astar, robot_r, n_ciri),
         transform=axR.transAxes, fontsize=8.6, va="bottom",
         bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="#90a4ae", alpha=0.92))

fig.suptitle("%s 审阅：六边形柱林（%.1f m 格 / %.2f m 方柱 / 柱心格内随机 / 零车道 / 空格 = 航点周围 %s）"
             " + SUPER 真正会飞的路线"
             % (str((w.get("v2", {}) or {}).get("round_label", "第 87 轮")),
                float((w.get("v2", {}) or {}).get("pitch", 1.0)),
                2 * r,
                ("%dx%d 格" % (2 * _neigh + 1, 2 * _neigh + 1)) if _neigh else "航点格"),
             fontsize=12.5)
fig.tight_layout(rect=[0, 0, 1, 0.95])
os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
fig.savefig(a.out, dpi=140)
print("wrote %s" % a.out)
print("  pillars %d | box %.2f m | z_split %.1f | flight z %.2f | waypoint cells %d"
      % (len(P), 2 * r, z_split, z, len(w.get("v2", {}).get("waypoint_cells_empty") or [])))
for (k, v, _) in info:
    print("  %-8s A* route narrowest %s" % (k, "unreachable" if v != v else "%.3f m" % v))
print("  legs passing A*(%.2f): %d/7 | passing CIRI(%.2f): %d/7" % (astar_clear, n_astar, robot_r, n_ciri))
