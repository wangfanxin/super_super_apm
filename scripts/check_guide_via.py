#!/usr/bin/env python3
"""check_guide_via.py -- offline A* gate for the R1a handover geometry.

After the 2026-11-28 review, an R1a advance keeps the OLD committed trajectory (including the corner it
ends at) as the guide prefix (`split_dis = planning_horizon - 4 m` for a new goal), so the corner IS an
interior point by construction.  What remains to check is that the A* that starts at that corner can still
reach the NEXT corner inside the remaining horizon, through the inflated pillar field.

Grid: 0.1 m, arena from the world file, pillars inflated by (half-side + 0.20 m) as the planner's
inflation_step=2 does.  For every consecutive corner pair (i -> i+1) this prints the shortest path length
and whether it fits horizon - advance_distance.  Exit code 1 if any pair does not fit.

USAGE: python3 scripts/check_guide_via.py [--world sim/worlds/forest_hex12.yaml] [--advance 3.9]
"""
import argparse, heapq, math, os, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load_world(path):
    import yaml
    w = yaml.safe_load(open(path, encoding="utf-8"))
    pil = w.get("pillars") or []
    corners = w["mission"]["corners"]
    hs = float((w.get("pillar_geometry") or {}).get("box_half_side_m", 0.1))
    return w, pil, corners, hs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default=os.path.join(ROOT, "sim", "worlds", "forest_hex12.yaml"))
    ap.add_argument("--advance", type=float, default=3.9, help="R1a advance distance (m), replay p50")
    ap.add_argument("--horizon", type=float, default=16.0)
    ap.add_argument("--cell", type=float, default=0.1)
    ap.add_argument("--inflate", type=float, default=0.20)
    a = ap.parse_args()
    w, pil, corners, hs = load_world(a.world)
    x0, x1 = -12.6, 12.6
    try:
        walls = w["arena"]["walls"]
        x0, x1 = float(walls["x"][0]), float(walls["x"][1])
    except Exception:
        pass
    n = int(round((x1 - x0) / a.cell))
    occ = bytearray(n * n)

    def idx(x, y):
        ix = int((x - x0) / a.cell); iy = int((y - x0) / a.cell)
        return ix, iy

    def mark(ix, iy):
        if 0 <= ix < n and 0 <= iy < n:
            occ[iy * n + ix] = 1

    r_eff = hs + a.inflate
    for p in pil:
        cx, cy, r = float(p["x"]), float(p["y"]), float(p.get("r", hs))
        rr = r_eff + a.cell
        i0, j0 = idx(cx - rr, cy - rr); i1, j1 = idx(cx + rr, cy + rr)
        for ix in range(max(0, i0), min(n, i1 + 1)):
            for iy in range(max(0, j0), min(n, j1 + 1)):
                if math.hypot((ix + 0.5) * a.cell + x0 - cx, (iy + 0.5) * a.cell + x0 - cy) <= rr:
                    mark(ix, iy)

    def free_at(x, y):
        ix, iy = idx(x, y)
        if not (0 <= ix < n and 0 <= iy < n):
            return None
        return occ[iy * n + ix] == 0

    def astar(sx, sy, gx, gy):
        si = idx(sx, sy); gi = idx(gx, gy)
        if not (0 <= si[0] < n and 0 <= si[1] < n and 0 <= gi[0] < n and 0 <= gi[1] < n):
            return None
        h = lambda ix, iy: math.hypot(ix - gi[0], iy - gi[1])
        best = {si: 0.0}
        pq = [(h(*si), 0.0, si)]
        while pq:
            f, g, cur = heapq.heappop(pq)
            if cur == gi:
                return g * a.cell
            if g > best.get(cur, 1e18):
                continue
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nxt = (cur[0] + dx, cur[1] + dy)
                    if not (0 <= nxt[0] < n and 0 <= nxt[1] < n):
                        continue
                    if occ[nxt[1] * n + nxt[0]]:
                        continue
                    ng = g + math.hypot(dx, dy)
                    if ng < best.get(nxt, 1e18):
                        best[nxt] = ng
                        heapq.heappush(pq, (ng + h(*nxt), ng, nxt))
        return None

    print("== check_guide_via: world=%s corners=%d advance=%.2f horizon=%.1f" % (
        os.path.basename(a.world), len(corners), a.advance, a.horizon))
    ok = True
    for i in range(len(corners) - 1):
        cx, cy = float(corners[i][0]), float(corners[i][1])
        nx, ny = float(corners[i + 1][0]), float(corners[i + 1][1])
        if not free_at(cx, cy):
            print("  %2d -> %2d: corner is NOT free on the inflated grid (planner would shift it)" % (i, i + 1))
        L = astar(cx, cy, nx, ny)
        need = math.hypot(nx - cx, ny - cy)
        avail = a.horizon - a.advance
        if L is None:
            print("  %2d -> %2d: NO A* PATH on the inflated grid -> FAIL" % (i, i + 1)); ok = False
        else:
            fit = L <= avail
            print("  %2d -> %2d: straight %.2f m, A* %.2f m, available horizon %.2f m -> %s"
                  % (i + 1, i + 2, need, L, avail, "OK" if fit else "DOES NOT FIT"))
            ok = ok and fit
    print("GUIDE-VIA: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
