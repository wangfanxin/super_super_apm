#!/usr/bin/env python3
"""Draw the pillar field with every radius that matters (owner's rule, 2026-11-26).

At the 2.0 m mission height the trunk is the **box** part (world `z_split = 2.1 m`, box half-side
`r = 0.10 m`), so "how close did we get" must be asked of the box, not of a circle around it.

Three boundaries are drawn for every pillar near the flown path:

    trunk box          physical obstacle (grey filled square)
    body-contact ring   Minkowski sum of the box and the BODY circle (planner `robot_r = 0.10 m`):
                        the instant the body EDGE touches the trunk the body CENTRE is on this ring.
    planning ring       body ring grown by the map inflation the planner applies
                        (`inflation_step * inflation_resolution = 2 x 0.1 m = 0.2 m`): the keep-out a
                        planner that inflates the map by 0.2 m sees.

The flown path is coloured with the EXACT body-edge margin
`surface_dist(box, centre) - robot_r`:

    red    margin < 0            body edge inside the trunk (contact)
    orange 0 <= margin < 0.10    body edge within one body radius of the trunk (touching band)
    yellow 0.10 <= margin < 0.30 inside the map inflation (planning margin)
    green  margin >= 0.30        free

`W4` itself is the stricter point-vs-box number (accept.py --body box, body as a POINT): it is printed
next to the minimum body-edge margin, because both answers matter.
"""
from __future__ import annotations

import os

import numpy as np

try:
    import yaml
except ImportError:                                  # pragma: no cover
    yaml = None

C_CONTACT = "#f43f5e"
C_TOUCH = "#fb923c"
C_INFLATE = "#facc15"
C_FREE = "#38bdf8"
C_BOX = "#6b7280"
C_BODY_RING = "#f87171"
C_INFL_RING = "#facc15"


def geometry(planner_config=None, root=None, robot_r=0.10, inflation=0.20, z_split=2.1):
    """(robot_r, inflation_m, z_split) -- read the delivered planner config when it is available."""
    if planner_config is None and root is not None:
        planner_config = os.path.join(root, "hku_super", "config", "super_planner_hex.yaml")
    rr, infl, zs = robot_r, inflation, z_split
    try:
        with open(planner_config, encoding="utf-8") as fh:
            cfg = (yaml.safe_load(fh) or {}).get("super_planner", {})
        rr = float(cfg.get("robot_r", rr))
        res = float(cfg.get("inflation_resolution", 0.1))
        step = float(cfg.get("inflation_step", 2))
        infl = res * step
    except Exception:                               # noqa: BLE001
        pass
    return rr, infl, zs


def surface_dist(xy, pillars, z, z_split=2.1):
    """Exact distance from a point to the nearest trunk surface (box below z_split, circle above).

    xy may be (2,) or (N,2); returns the same leading shape.
    """
    xy = np.asarray(xy, float)
    P = np.array([[float(p["x"]), float(p["y"])] for p in pillars], float)
    R = np.array([float(p.get("r", 0.1)) for p in pillars], float)
    single = xy.ndim == 1
    Q = xy[None, :] if single else xy
    zz = np.asarray(z, float)
    d = np.empty((len(Q), len(P)))
    for k, q in enumerate(Q):
        zk = float(zz) if zz.ndim == 0 else float(zz[k])
        dx = np.abs(q[0] - P[:, 0]) - R
        dy = np.abs(q[1] - P[:, 1]) - R
        inside = (dx <= 0) & (dy <= 0)
        box = np.where(inside, np.maximum(dx, dy), np.hypot(np.maximum(dx, 0.0), np.maximum(dy, 0.0)))
        r_cyl = np.hypot(q[0] - P[:, 0], q[1] - P[:, 1]) - R
        d[k] = np.where(zk < z_split, box, r_cyl)
    out = d.min(axis=1)
    return float(out[0]) if single else out


def body_margin(xy, pillars, z, robot_r=0.10, z_split=2.1):
    """Exact body-EDGE margin (m): <0 means the body circle overlaps the trunk."""
    return surface_dist(xy, pillars, z, z_split) - robot_r


def color_for(margin):
    if margin < 0.0:
        return C_CONTACT
    if margin < 0.10:
        return C_TOUCH
    if margin < 0.30:
        return C_INFLATE
    return C_FREE


def minkowski_ring(cx, cy, half, pad, n_arc=10):
    """Boundary of box(half) (+) disk(pad): the four straight edges and four corner arcs."""
    pts = []
    half = float(half)
    pad = float(pad)
    corners = [(1, 1), (-1, 1), (-1, -1), (1, -1)]           # upper/right, upper/left, lower/left, lower/right
    for (sx, sy) in corners:
        th_c = np.arctan2(sy, sx)                             # outward diagonal of this corner
        th = th_c - np.pi / 4.0 + np.linspace(0.0, np.pi / 2.0, n_arc)   # that quadrants +-45 deg
        pts.append(np.column_stack([cx + sx * half + pad * np.cos(th),
                                    cy + sy * half + pad * np.sin(th)]))
    # re-order into a closed CCW loop: right-top, top-left, bottom-left, bottom-right arcs already
    # run CCW because each arc sweeps +90 deg; concatenating them gives the full rounded square,
    # except the straight edges connect the arc end points, which is exactly the Minkowski sum.
    out = np.vstack(pts)
    return out


def draw_obstacles(ax, pillars, z, *, robot_r=0.10, inflation=0.20, z_split=2.1, path_xy=None,
                   ring_m=2.2, box_color=C_BOX, draw_rings=True, lw=0.7, n_arc=10, zorder=1):
    """Draw every trunk box; draw body/planning rings for the trunks near the path (or all if no path).

    Returns (min_margin, min_index, legend_handles): min_margin is the smallest body-edge margin over
    `path_xy` (None when no path given); the two proxies let the caller build one legend.
    """
    from matplotlib.patches import Rectangle, Polygon
    from matplotlib.lines import Line2D

    if not pillars:
        return None, None, []
    P = np.array([[float(p["x"]), float(p["y"])] for p in pillars], float)
    R = np.array([float(p.get("r", 0.1)) for p in pillars], float)
    for k in range(len(P)):
        ax.add_patch(Rectangle((P[k, 0] - R[k], P[k, 1] - R[k]), 2 * R[k], 2 * R[k],
                               facecolor=box_color, edgecolor="none", alpha=0.85, zorder=zorder))
    if path_xy is not None and len(path_xy) > 1:
        Q = np.asarray(path_xy, float)
        near = np.zeros(len(P), bool)
        for q in Q[::max(1, len(Q) // 400)]:
            near |= np.hypot(P[:, 0] - q[0], P[:, 1] - q[1]) <= ring_m
    else:
        near = np.ones(len(P), bool)
    if draw_rings:
        for k in np.where(near)[0]:
            x, y, r = P[k, 0], P[k, 1], R[k]
            if inflation > 0:
                ring = minkowski_ring(x, y, r, robot_r + inflation, n_arc=n_arc)
                ax.add_patch(Polygon(ring, closed=True, facecolor="none", edgecolor=C_INFL_RING,
                                     lw=lw * 0.9, ls=(0, (1, 3)), alpha=0.75, zorder=zorder + 1))
            ring = minkowski_ring(x, y, r, robot_r, n_arc=n_arc)
            ax.add_patch(Polygon(ring, closed=True, facecolor="none", edgecolor=C_BODY_RING,
                                 lw=lw, ls=(0, (3, 2)), alpha=0.95, zorder=zorder + 1))
    handles = []
    if draw_rings and near.any():
        handles = [Line2D([], [], color=C_BODY_RING, lw=lw, ls=(0, (3, 2)),
                          label="躯干 + 机身半径(r=%.2f)接触环" % robot_r),
                   Line2D([], [], color=C_INFL_RING, lw=lw, ls=(0, (1, 3)),
                          label="+ 地图膨胀(%.2f m) 规划环" % inflation)]
    if path_xy is None:
        return None, None, handles
    m = body_margin(np.asarray(path_xy, float), pillars, z, robot_r, z_split)
    i = int(np.argmin(m))
    return float(m[i]), i, handles


def clearance_legend(cmd=True):
    from matplotlib.lines import Line2D
    h = [Line2D([], [], color=C_CONTACT, lw=2.4, label="body margin < 0（接触）"),
         Line2D([], [], color=C_TOUCH, lw=2.4, label="0–0.10 m（擦边带）"),
         Line2D([], [], color=C_INFLATE, lw=2.4, label="0.10–0.30 m（膨胀带内）"),
         Line2D([], [], color=C_FREE, lw=2.4, label="> 0.30 m（自由）")]
    return h


def plot_clearance_path(ax, xy, margin, *, lw=2.4, zorder=4, alpha=1.0):
    """Colour a path segment-by-segment with its body margin; returns the used Line2D proxy handles."""
    from matplotlib.collections import LineCollection
    P = np.asarray(xy, float)
    M = np.asarray(margin, float)
    pts = P.reshape(-1, 2, 1) if False else None
    segs = np.stack([P[:-1], P[1:]], axis=1)
    cols = [color_for(0.5 * (M[i] + M[i + 1])) for i in range(len(segs))]
    ax.add_collection(LineCollection(segs, colors=cols, linewidths=lw, alpha=alpha, zorder=zorder))
    return clearance_legend()
