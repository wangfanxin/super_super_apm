#!/usr/bin/env python3
"""为本任务构建确定性世界：18x18 m 点阵森林 + 每条腿上一根"拦路树"。

树干几何（本版的关键变化）
--------------------------
同一根树干**下半段是方盒、上半段是圆柱**：

    z <  z_split (=2.1 m)  ->  轴对齐方盒，半边长 r
    z >= z_split           ->  圆柱，半径 r

飞行高度 2.0 m 落在**方盒段**，所以二维避障必须用"到正方形的距离"，
不能用"到圆心的距离减半径" —— 后者在斜向上会低估阻挡。
距离函数统一在 `src/whtf/common/obstacles.py`，生成器/栅格/验收共用同一份。

为什么不用 make_world.py
------------------------
实测：`make_world.py --cells 12 --cell 1.5 --min-gap 0.90` 产出精确点阵
（`--seed 21..45` 逐字节相同），15 m 方形四条直腿到柱面恒为 0.350 m
→ 直线可飞 → "避障"这条验收项无从体现。本构建器在每条腿正中间压一根树，
让"必须绕行"成为硬需求，且完全可复现。

用法：python3 src/tools/build_world.py --out worlds/forest_square15.yaml
"""
from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from whtf.common.obstacles import DEFAULT_Z_SPLIT, min_clearance, seg_min_clearance  # noqa: E402

ARENA_HALF = 9.6
CELL = 2.4                    # 点阵间距：2.4 m 才有 1.6 m 的柱间缝，4 m/s 可飞
NPILL = 8                     # 8 x 8 = 64 根树，19.2 x 19.2 m 场地
R_TREE = 0.40
Z0, Z1 = 0.0, 4.2
Z_SPLIT = DEFAULT_Z_SPLIT     # 2.1 m：下半段方盒 / 上半段圆柱
Z_FLY = 2.0                   # 任务高度，落在方盒段

CORNER_CLEAR = 0.90           # 角点到树面的净空（转弯半径必须装得进去）
CORRIDOR_CLEAR = 0.00         # 不清腿两侧：2.4 m 点阵本身就有 1.6 m 缝，腿正好走在两排之间
PAD_CLEAR = 1.00              # 起飞点周围净空（否则起点被膨胀吃掉，A* 直接无解）
SPOKE_CLEAR = 0.00            # 点阵放宽后不需要专门通道（0 = 关闭）
CORNERS = [(-7.5, -7.5), (7.5, -7.5), (7.5, 7.5), (-7.5, 7.5)]
ORIGIN = (0.0, 0.0)


def lattice():
    c = [(-ARENA_HALF + CELL * 0.5) + i * CELL for i in range(NPILL)]
    return [(x, y) for x in c for y in c]


def legs():
    return [(CORNERS[i], CORNERS[(i + 1) % 4]) for i in range(4)]


def _tree_to_leg_clear(tx, ty, a, b):
    """单根树（轴心 tx,ty）到腿 a-b 的最小"树面到腿"距离（在腿采样）。"""
    d = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(2, int(d / 0.10))
    one = [{"x": tx, "y": ty, "r": R_TREE}]
    worst = float("inf")
    for k in range(n + 1):
        t = k / n
        px = a[0] + t * (b[0] - a[0])
        py = a[1] + t * (b[1] - a[1])
        worst = min(worst, min_clearance(px, py, one, Z_FLY, Z_SPLIT))
    return worst


def build():
    corner_probe = [{"x": cx, "y": cy, "r": R_TREE} for cx, cy in CORNERS]

    # 1) 去掉角点附近的树（盒必须能过）+ 起飞点净空
    pad_probe = [{"x": 0.0, "y": 0.0, "r": R_TREE}]
    kept = [(x, y) for (x, y) in lattice()
            if min_clearance(x, y, corner_probe, Z_FLY, Z_SPLIT) >= CORNER_CLEAR
            and min_clearance(x, y, pad_probe, Z_FLY, Z_SPLIT) >= PAD_CLEAR]

    # 2) 沿每条腿清带（否则拦路树与点阵树挤在一起，绕行走廊不可飞）
    for (a, b) in legs():
        kept = [(x, y) for (x, y) in kept if _tree_to_leg_clear(x, y, a, b) >= CORRIDOR_CLEAR]

    # 2b) 起飞点 -> 四个角点的通道（内部点阵在膨胀后是实心墙，没有通道就到不了第一个角）
    for c in CORNERS:
        kept = [(x, y) for (x, y) in kept
                if _tree_to_leg_clear(x, y, ORIGIN, c) >= SPOKE_CLEAR]

    pillars = [{"x": x, "y": y, "r": R_TREE, "z0": Z0, "z1": Z1} for (x, y) in kept]

    # 3) 每条腿正中间压一根树 -> 直线必被挡
    injected = []
    for i, (a, b) in enumerate(legs()):
        p = {"x": round(0.5 * (a[0] + b[0]), 3), "y": round(0.5 * (a[1] + b[1]), 3),
             "r": R_TREE, "z0": Z0, "z1": Z1}
        pillars.append(p)
        injected.append((i, p))

    audit = {}
    audit["corner_clear"] = [min_clearance(cx, cy, pillars, Z_FLY, Z_SPLIT) for cx, cy in CORNERS]
    audit["leg_clear"] = [seg_min_clearance(a, b, pillars, Z_FLY, Z_SPLIT) for (a, b) in legs()]
    audit["blocked"] = sum(1 for c in audit["leg_clear"] if c < 0.30)
    gaps = []
    for (i, p) in injected:
        a, b = legs()[i]
        dx, dy = b[0] - a[0], b[1] - a[1]
        n = math.hypot(dx, dy)
        nx, ny = -dy / n, dx / n
        if nx * p["x"] + ny * p["y"] > 0.0:      # 让 n 指向方形中心（原点）
            nx, ny = -nx, -ny
        sides = {+1: [], -1: []}
        for q in pillars:
            if q is p:
                continue
            rx, ry = q["x"] - p["x"], q["y"] - p["y"]
            s = rx * nx + ry * ny                 # 沿法向（内/外）
            t = abs(rx * (-ny) + ry * nx)         # 沿腿方向
            if abs(s) < 8.0 and t < 8.0:
                sides[1 if s > 0 else -1].append(math.hypot(rx, ry) - 2 * R_TREE)
        inner = min(sides[+1]) if sides[+1] else float("inf")
        outer = min(sides[-1]) if sides[-1] else float("inf")
        gaps.append(max(inner, outer))            # 可用的那一侧
        audit.setdefault("gate_sides", []).append((i, inner, outer))
    audit["gate_gap_min"] = min(gaps)
    audit["n_pillars"] = len(pillars)
    audit["injected"] = [(i, p["x"], p["y"]) for (i, p) in injected]
    return pillars, audit


def _footprint_gap(p, q):
    """两根树干在飞行高度上的面-面间隙：方盒用 L∞，圆柱用中心距。"""
    dx, dy = abs(p["x"] - q["x"]), abs(p["y"] - q["y"])
    if Z_FLY < Z_SPLIT:
        return max(dx, dy) - p["r"] - q["r"]
    return math.hypot(dx, dy) - p["r"] - q["r"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    pillars, a = build()
    d = os.path.dirname(args.out)
    if d:
        os.makedirs(d, exist_ok=True)

    with open(args.out, "w", encoding="utf-8") as fh:
        fh.write("# BUILT by src/tools/build_world.py -- deterministic, no seed\n")
        fh.write("# Frame: local ENU-like, x=East, y=North, z=Up, origin at the take-off pad\n")
        fh.write("# Trunk geometry: z <  z_split -> AXIS-ALIGNED BOX (half-side r)\n")
        fh.write("#                 z >= z_split -> CYLINDER (radius r)\n")
        fh.write("# z_split = %.2f m ; mission altitude = %.2f m (inside the BOX half)\n"
                 % (Z_SPLIT, Z_FLY))
        fh.write("# Arena %.1f x %.1f m, lattice %.2f m, r %.2f m, trunk z %.1f-%.1f m\n"
                 % (2 * ARENA_HALF, 2 * ARENA_HALF, CELL, R_TREE, Z0, Z1))
        fh.write("# Mission square corners: %s\n" % " ".join("%.1f,%.1f" % c for c in CORNERS))
        fh.write("#\n# audit (all measured at z=%.2f, i.e. the BOX half):\n" % Z_FLY)
        fh.write("#   trunks total           : %d\n" % a["n_pillars"])
        fh.write("#   corner clearance (min) : %.3f m  (contract %.2f)\n"
                 % (min(a["corner_clear"]), CORNER_CLEAR))
        for i, (c, cl) in enumerate(zip(CORNERS, a["corner_clear"])):
            fh.write("#     corner %d (%+.2f,%+.2f): %.3f m\n" % (i, c[0], c[1], cl))
        for i, cl in enumerate(a["leg_clear"]):
            fh.write("#   leg %d straight-line clearance: %+.3f m  %s\n"
                     % (i, cl, "BLOCKED(<0.30)" if cl < 0.30 else "clear"))
        fh.write("#   legs blocked           : %d / 4  (need >= 3 for a real avoidance test)\n"
                 % a["blocked"])
        fh.write("#   gate gap (min)         : %.3f m\n" % a["gate_gap_min"])
        for (i, x, y) in a["injected"]:
            fh.write("#     blocking trunk on leg %d at (%+.3f,%+.3f)\n" % (i, x, y))
        fh.write("z_split: %.2f\n" % Z_SPLIT)
        fh.write("pillars:\n")
        for p in pillars:
            fh.write("  - {x: %.3f, y: %.3f, r: %.2f, z0: %.1f, z1: %.1f}\n"
                     % (p["x"], p["y"], p["r"], p["z0"], p["z1"]))

    fp = "BOX" if Z_FLY < Z_SPLIT else "CIRCLE"
    print("build_world: %d trunks -> %s" % (a["n_pillars"], args.out))
    print("  z_split=%.2f m , fly z=%.2f m -> footprint %s" % (Z_SPLIT, Z_FLY, fp))
    print("  corner clearance min : %.3f m  %s"
          % (min(a["corner_clear"]), "OK" if min(a["corner_clear"]) >= CORNER_CLEAR else "FAIL"))
    print("  legs blocked         : %d / 4" % a["blocked"])
    print("  gate gap min         : %.3f m" % a["gate_gap_min"])
    for i, cl in enumerate(a["leg_clear"]):
        print("  leg %d clearance %+.3f m %s" % (i, cl, "BLOCKED" if cl < 0.30 else "clear"))
    ok = (min(a["corner_clear"]) >= CORNER_CLEAR and a["blocked"] >= 3
          and a["gate_gap_min"] >= 0.9)
    print("  VERDICT: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
