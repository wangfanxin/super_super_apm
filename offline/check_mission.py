#!/usr/bin/env python3
"""M0 检查 1：任务盒 vs 世界 —— 几何可行性（形状感知）。

不需要 ROS、不需要飞控。回答四个问题：
  1. 每个盒是否在场地范围内？
  2. 每个盒是否与树干相交（盒内不能有树面）？
  3. 盒的净空余量是多少？
  4. 盒间直连段是否被树干挡住（给出"最差直飞"基线）？

★ 距离用 `whtf.common.obstacles`：树干 z<z_split 是方盒、否则圆柱。
  飞行高度 2.0 m 落在方盒段，所以必须用方盒距离。

用法：python3 offline/check_mission.py
"""
from __future__ import annotations

import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))

from whtf.common.obstacles import (  # noqa: E402
    footprint_at, load_world, min_clearance, seg_min_clearance,
)

MISSION = os.path.join(HERE, "..", "worlds", "mission_square_15m.yaml")
WORLD = os.path.join(HERE, "..", "worlds", "forest_square15.yaml")
ARENA_HALF = 9.0
Z_FLY = 2.0
BLOCK_THRESH = 0.30


def load_mission(path):
    """只认本仓库自己写出的固定格式（列表里含逗号，必须先用正则抠出来）。"""
    boxes, limits = [], {}
    for raw in open(path, encoding="utf-8"):
        s = raw.split("#")[0].strip()
        if not s:
            continue
        if s.startswith("- {id:"):
            body = s[s.index("{") + 1 : s.rindex("}")]
            lists = dict(re.findall(r"(\w+):\s*\[([^\]]*)\]", body))
            scalars = {k: v.strip() for k, v in re.findall(r"(\w+):\s*([^,\[\]]+)", body)
                       if k not in lists}
            boxes.append({
                "id": int(scalars["id"]),
                "center": [float(x) for x in lists["center"].split(",")],
                "half": [float(x) for x in lists["half"].split(",")],
                "kind": scalars.get("kind", "flyby"),
            })
        elif s.startswith(("v_max:", "a_max:", "a_lat_max:")):
            k, v = s.split(":")
            limits[k.strip()] = float(v)
    return boxes, limits


def main():
    boxes, limits = load_mission(MISSION)
    pillars, z_split, meta = load_world(WORLD)
    print("mission : %d boxes" % len(boxes))
    print("world   : %d trunks, arena +/-%.1f m, z_split %.2f m" %
          (len(pillars), ARENA_HALF, z_split))
    print("footprint@z=%.1f : %s" % (Z_FLY, footprint_at(Z_FLY, z_split).upper()))
    print("limits  : %s" % limits)
    print()

    fails = 0

    print("== 1. box inside arena ==")
    for b in boxes:
        cx, cy, cz = b["center"]
        ok = abs(cx) + b["half"][0] <= ARENA_HALF and abs(cy) + b["half"][1] <= ARENA_HALF
        fails += 0 if ok else 1
        print("  box %2d centre (%+6.2f,%+6.2f,%4.2f)  %s"
              % (b["id"], cx, cy, cz, "OK" if ok else "FAIL"))

    print("\n== 2/3. trunk vs box ==")
    for b in boxes:
        cx, cy, _ = b["center"]
        nearest = min_clearance(cx, cy, pillars, Z_FLY, z_split)
        margin = nearest - max(b["half"][:2])
        ok = margin >= 0.0
        fails += 0 if ok else 1
        print("  box %2d nearest trunk surface %6.3f m , half %.2f , margin %+6.3f  %s"
              % (b["id"], nearest, max(b["half"][:2]), margin, "OK" if ok else "FAIL"))

    print("\n== 4. straight-line baseline (only a lower bound; the plan must avoid) ==")
    blocked = 0
    for i in range(len(boxes) - 1):
        a = tuple(boxes[i]["center"][:2])
        c = tuple(boxes[i + 1]["center"][:2])
        worst = seg_min_clearance(a, c, pillars, Z_FLY, z_split)
        is_b = worst < BLOCK_THRESH
        blocked += 1 if is_b else 0
        print("  leg %2d->%2d  min trunk surface %+6.3f m  %s"
              % (i, i + 1, worst, "BLOCKED" if is_b else "clear"))

    print("\n== summary ==")
    print("  geometry fails        : %d" % fails)
    print("  straight legs blocked : %d / %d" % (blocked, len(boxes) - 1))
    print("  VERDICT: %s" % ("PASS" if fails == 0 else "FAIL"))
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
