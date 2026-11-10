"""障碍物的几何语义：同一根树干，下半段是方盒，上半段是圆柱。

为什么要有这个模块
------------------
`worlds/forest_square15.yaml` 里每根树干的水平截面随高度变化：

    z <  z_split   ->  方盒（轴对齐正方形，半边长 r）
    z >= z_split   ->  圆柱（半径 r）

"到障碍表面的距离"因此不能到处各写一份：
`build_world.py` 的自检、`grid.py` 的自由空间、`check_mission.py` 的验收
必须用**同一个函数**，否则一个说能过、另一个说不能过。

距离定义
--------
* 圆柱：`hypot(dx,dy) - r`
* 方盒：`hypot(max(|dx|-r,0), max(|dy|-r,0))`（在盒内为负，
  内部取 `max(dx,dy) - r` 的近似不可用于负值，故分开处理）

返回值 < 0 表示点已在障碍内部。
"""
from __future__ import annotations

import math
from typing import List, Sequence, Tuple

# 默认的方/圆分界高度（m）。树干 z0..z1 为 0..4.2，"下半段"= 2.1。
DEFAULT_Z_SPLIT = 2.1


def surface_dist(dx: float, dy: float, r: float, z: float, z_split: float) -> float:
    """点到单根树干表面的距离（dx,dy 是相对树干轴心的水平偏移）。负值 = 在内部。"""
    if z < z_split:
        # 方盒
        ox = abs(dx) - r
        oy = abs(dy) - r
        if ox <= 0.0 and oy <= 0.0:
            # 在盒内：取到最近边的距离（负值）
            return max(ox, oy)
        return math.hypot(max(ox, 0.0), max(oy, 0.0))
    # 圆柱
    return math.hypot(dx, dy) - r


def min_clearance(px: float, py: float, pillars: Sequence[dict], z: float,
                  z_split: float = DEFAULT_Z_SPLIT) -> float:
    """该点到所有树干表面的最小距离。"""
    if not pillars:
        return float("inf")
    return min(surface_dist(px - p["x"], py - p["y"], p["r"], z, z_split) for p in pillars)


def seg_min_clearance(a: Tuple[float, float], b: Tuple[float, float],
                      pillars: Sequence[dict], z: float,
                      z_split: float = DEFAULT_Z_SPLIT, step: float = 0.05) -> float:
    """线段 a->b 到所有树干表面的最小距离（采样）。"""
    d = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(2, int(d / step))
    worst = float("inf")
    for k in range(n + 1):
        t = k / n
        x = a[0] + t * (b[0] - a[0])
        y = a[1] + t * (b[1] - a[1])
        worst = min(worst, min_clearance(x, y, pillars, z, z_split))
    return worst


def footprint_at(z: float, z_split: float = DEFAULT_Z_SPLIT) -> str:
    """给审计用：该高度上树干是 box 还是 circle。"""
    return "box" if z < z_split else "circle"


def load_world(path: str) -> Tuple[List[dict], float, dict]:
    """读取世界 YAML，返回 (pillars, z_split, meta)。

    只认本仓库 build_world.py 写出的固定格式，避免依赖 pyyaml 版本。
    """
    pillars: List[dict] = []
    z_split = DEFAULT_Z_SPLIT
    meta: dict = {}
    for raw in open(path, encoding="utf-8"):
        s = raw.strip()
        if s.startswith("#"):
            if "z_split" in s and "=" in s:
                try:
                    z_split = float(s.split("=")[-1].strip())
                except ValueError:
                    pass
            continue
        if s.startswith("z_split:"):
            z_split = float(s.split(":")[1].strip())
            continue
        if not s.startswith("- {"):
            continue
        d = {}
        for part in s[s.index("{") + 1 : s.rindex("}")].split(","):
            k, v = part.split(":")
            d[k.strip()] = float(v)
        pillars.append(d)
    meta["z_split"] = z_split
    meta["n"] = len(pillars)
    return pillars, z_split, meta
