#!/usr/bin/env python3
"""check_frozen.py -- 地图与航点一旦冻结，任何改动都必须被这里抓住。

用户第 81 轮的原话："地图从现在开始，不准变化。航点位置也不准变化。"
第 88 轮用户改了口径并给了新参数（"场地 [-12.0,+12.0]^2；六边形航点每点之间差 8m 以上（随后改为 10.000 m）；
航点附近 9 个格子不产生柱子"），于是 **SPEC 地图换成 worlds/forest_hex12.yaml**，
旧图 forest_v2_hex.yaml 归档为历史（见 docs/SPEC.md 的修改记录）。

冻结的是**内容**，不是"承诺"：世界文件的 SHA-256、柱子数、空格数、六个角点坐标、边长、场地墙
都记在下面的 FROZEN 里；`scripts/fly_hex2.sh` 在起飞前调用它，`docs/SPEC.md` 的 S5/S6 写着这条规矩。
换地图 = 先得到用户同意 + 在 SPEC 修改记录里留原句/新句 + 更新这里的指纹（用
`python3 scripts/fingerprint.py <world>` 生成），顺序不能反。

    python3 scripts/check_frozen.py [--world worlds/forest_hex12.yaml]
"""
import argparse
import hashlib
import io
import math
import sys

import yaml

# ---- the frozen fingerprint of the round-88 map --------------------------------------------------
FROZEN = {
    "sha256": "d30918efed8d5c47e5b2f076ee5eb3f201265be463c86b32f292d6738048beda",
    "bytes": 33307,
    "pillars": 522,
    "laps": 3,
    "z_m": 2.0,
    "radius_m": 10.0,
    "side_m": 10.0,
    "corners": [(10.0, 0.0),
                (5.0, 8.660254038),
                (-5.0, 8.660254038),
                (-10.0, 0.0),
                (-5.0, -8.660254038),
                (5.0, -8.660254038)],
    "keepout_cells": 54,
    "keepout_neigh": 1,
    "arena_m": 12.5,
    "generator": "--pitch 1.0 --cols 12 --r 0.1 --radius 10.0 --laps 3 --z 2.0 --z-split 2.1 "
                 "--seed 20240521 --overlap-m 0.02 --pad-clear 0.35 --keepout-wp 1 "
                 "--keepout-neigh 1 --arena-m 12.5",
}

ap = argparse.ArgumentParser()
ap.add_argument("--world", default="worlds/forest_hex12.yaml")
a = ap.parse_args()

ok = True


def row(good, name, detail):
    global ok
    ok = bool(good) and ok
    print("%-6s %-30s %s" % ("PASS" if good else "FAIL", name, detail))


raw = open(a.world, "rb").read()
sha = hashlib.sha256(raw).hexdigest()
w = yaml.safe_load(io.open(a.world, encoding="utf-8"))
mis = w["mission"]
v2 = w.get("v2") or {}
n_lap = int(mis.get("laps", 1) or 1)
corners = [(float(c[0]), float(c[1])) for c in mis["corners"][: len(mis["corners"]) // n_lap]]

print("== 地图与航点冻结检查：%s" % a.world)
row(sha == FROZEN["sha256"], "世界文件 SHA-256",
    "%s%s" % (sha[:32] + "…",
              "" if sha == FROZEN["sha256"] else "  ← 与冻结值 %s… 不同" % FROZEN["sha256"][:32]))
row(len(raw) == FROZEN["bytes"], "文件字节数", "%d（冻结 %d）" % (len(raw), FROZEN["bytes"]))
row(len(w["pillars"]) == FROZEN["pillars"], "柱子数", "%d（冻结 %d）" % (len(w["pillars"]), FROZEN["pillars"]))
row(int(mis.get("laps", 0)) == FROZEN["laps"], "圈数", "%s（冻结 %d）" % (mis.get("laps"), FROZEN["laps"]))
row(abs(float(mis.get("takeoff_alt", 0)) - FROZEN["z_m"]) < 1e-12, "任务高度",
    "%.2f m（冻结 %.2f）" % (float(mis.get("takeoff_alt", 0)), FROZEN["z_m"]))

dmax = max(max(abs(c[0] - f[0]), abs(c[1] - f[1])) for c, f in zip(corners, FROZEN["corners"]))
row(len(corners) == 6 and dmax < 1e-6, "六个航点坐标",
    "与冻结值最大偏差 %.2e m" % dmax)

# the owner's two geometric demands: pad distance = side = 10.000 m
r_all = [math.hypot(c[0], c[1]) for c in corners]
sides = [math.hypot(corners[(i + 1) % 6][0] - corners[i][0],
                    corners[(i + 1) % 6][1] - corners[i][1]) for i in range(6)] if len(corners) == 6 else []
row(bool(r_all) and all(abs(r - FROZEN["radius_m"]) < 1e-6 for r in r_all), "每个角点到 (0,0) = 10.000 m",
    "min %.6f / max %.6f m" % (min(r_all), max(r_all)) if r_all else "n/a")
row(bool(sides) and all(abs(s - FROZEN["side_m"]) < 1e-6 for s in sides), "相邻角点 = 10.000 m",
    "min %.6f / max %.6f m" % (min(sides), max(sides)) if sides else "n/a")

row(len(mis["corners"]) == 6 * FROZEN["laps"], "航点总数",
    "%d = 6 × %d" % (len(mis["corners"]), FROZEN["laps"]))

# the 3x3 keep-out around every waypoint: 54 empty cells, and no pillar inside them
keep = v2.get("keepout_cells_empty") or []
pitch = float(v2.get("pitch", 1.0) or 1.0)
cells_with_pillar = {(int(math.floor(float(q["x"]) / pitch)), int(math.floor(float(q["y"]) / pitch)))
                     for q in w["pillars"]}
row(len(keep) == FROZEN["keepout_cells"] and int(v2.get("keepout_neigh", -1)) == FROZEN["keepout_neigh"],
    "航点周围 3×3 格空格 = 54 格",
    "%d 格（keepout_neigh=%s）" % (len(keep), v2.get("keepout_neigh")))
n_bad = sum(1 for c in keep if tuple(c) in cells_with_pillar)
row(n_bad == 0, "空格里确实没有柱子", "54 格里 %d 格有柱" % n_bad)

walls = ((w.get("arena") or {}).get("walls") or {})
wx = walls.get("x") or [0, 0]
row(all(abs(abs(float(v)) - FROZEN["arena_m"]) < 1e-9 for v in wx), "场地墙 ±12.5 m",
    "x %s" % (walls.get("x"),))

hdr = io.open(a.world, encoding="utf-8").readline()
row("--seed 20240521" in hdr and "--pitch 1.0" in hdr and "--radius 10.0" in hdr,
    "生成器参数（文件头）", hdr.strip()[:96] + "…")

print()
if ok:
    print("== FROZEN AND UNCHANGED — safe to fly")
    print("   （换地图/换航点必须先得到用户同意并更新本文件的 FROZEN 指纹）")
else:
    print("== THE MAP OR THE WAYPOINTS CHANGED — DO NOT FLY")
sys.exit(0 if ok else 1)
