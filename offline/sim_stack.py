#!/usr/bin/env python3
"""M1 闭环仿真：L1 + L2 + L3 + L4 串起来跑 12 个航点盒。

不需要 ROS、不需要飞控。模型是点质量 + 加速度限幅 + 一阶执行器滞后，
够用来回答四个问题：

  1. 12 个盒能不能全部命中（`min‖p−c_k‖ ≤ half`）？
  2. 巡航速度能不能到 4 m/s 以上？
  3. 参考轨迹离树干最近多少（避障是否真的成立）？
  4. 换段瞬间的 ‖Δp‖ / ‖Δa‖（"缝"有多大）？

用法：python3 offline/sim_stack.py [--v 5.0] [--verbose]
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))

from whtf.common.obstacles import load_world, min_clearance, footprint_at  # noqa: E402
from whtf.planner.base import Box, run_backend  # noqa: E402
from whtf.planner.corridor import CorridorBackend  # noqa: E402
from whtf.stack import BoxScheduler, PidController, TrajServer  # noqa: E402

WORLD = os.path.join(HERE, "..", "worlds", "forest_square15.yaml")
Z_FLY = 2.0


def mission_boxes():
    """15 m 方形 x 3 圈 = 12 个盒（与 worlds/mission_square_15m.yaml 一致）。"""
    corners = [(-7.5, -7.5), (7.5, -7.5), (7.5, 7.5), (-7.5, 7.5)]
    out = []
    for lap in range(3):
        for (x, y) in corners:
            out.append(Box(center=np.array([x, y, Z_FLY]),
                           half=np.array([0.25, 0.25, 0.25]),
                           kind="flyby"))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--v", type=float, default=5.0, help="cruise speed (m/s)")
    ap.add_argument("--dt", type=float, default=0.01, help="sim step (s)")
    ap.add_argument("--tmax", type=float, default=80.0)
    ap.add_argument("--replan", type=float, default=0.20, help="replan period (s)")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    pillars, z_split, _ = load_world(WORLD)
    print("world: %d trunks, z_split %.2f, footprint@z=%.1f = %s"
          % (len(pillars), z_split, Z_FLY, footprint_at(Z_FLY, z_split).upper()))

    be = CorridorBackend(pillars, arena_half=9.0, res=0.10, margin=0.45,
                         v_cruise=args.v)
    boxes = mission_boxes()
    sched = BoxScheduler(boxes, v_cruise=args.v, a_lat=3.0, r_flyby=1.2,
                         v_max=6.0, a_max=3.0, window_n=3, advance_r=0.6)
    srv = TrajServer()
    pid = PidController(kp=4.0, kd=5.0, ki=0.0, a_max=3.0, v_max=6.0)

    # 初始状态：起飞点上方悬停
    p = np.array([0.0, 0.0, Z_FLY])
    v = np.zeros(3)
    a = np.zeros(3)
    now = 0.0
    next_replan = 0.0

    trace = {"t": [], "p": [], "v": [], "a": [], "aref": []}
    plan_fails = 0
    worst_clr = float("inf")

    # 第一次规划
    req = sched.make_request(p, v, a)
    res = run_backend(be, req)
    if not res.ok:
        print("FIRST PLAN FAILED:", res.kind, res.detail)
        return 1
    srv.set_trajectory(res.traj, now)
    print("first plan: %s" % res.detail)

    while now < args.tmax:
        # ---- L1：观察 + 到点就重规划（时间节拍，不是"到达才规划"）----
        sched.observe(p)
        if now >= next_replan or sched.ver != req.ver:
            if sched.done:
                pass
            req2 = sched.make_request(p, v, a)
            res2 = run_backend(be, req2)
            if res2.ok:
                srv.set_trajectory(res2.traj, now)
                req = req2
            else:
                plan_fails += 1
                if args.verbose:
                    print("  replan fail @%.2f %s %s" % (now, res2.kind, res2.detail))
            next_replan = now + args.replan

        # ---- L3：求值 ----
        p_ref, v_ref, a_ref, src = srv.sample(now)

        # ---- L4：PID ----
        a_cmd = pid.update(p_ref, v_ref, a_ref, p, v, args.dt)

        # ---- 点质量 + 一阶滞后 ----
        a = a + (a_cmd - a) * min(1.0, args.dt / 0.05)
        v = v + a * args.dt
        sp = float(np.linalg.norm(v))
        if sp > 6.0:
            v = v / sp * 6.0
        p = p + v * args.dt

        trace["t"].append(now); trace["p"].append(p.copy())
        trace["v"].append(v.copy()); trace["a"].append(a.copy())
        trace["aref"].append(a_ref.copy())

        worst_clr = min(worst_clr, min_clearance(p[0], p[1], pillars, Z_FLY, z_split))

        if sched.done and now > next_replan:
            break
        now += args.dt

    # ---------------- 评分 ----------------
    P = np.array(trace["p"]); V = np.array(trace["v"])
    T = np.array(trace["t"])
    rep = sched.hit_report()

    # 巡航速度：只统计"距最近盒 > 2 m"的样本，排除起飞/减速
    dmin = np.min(np.linalg.norm(
        P[:, None, :2] - np.array([b.center[:2] for b in boxes])[None, :, :], axis=2), axis=1)
    cruise = V[np.linalg.norm(V, axis=1)][dmin > 2.0]
    v_cruise_meas = float(np.percentile(np.linalg.norm(cruise, axis=1), 90)) if cruise.size else 0.0
    v_mean = float(np.mean(np.linalg.norm(cruise, axis=1))) if cruise.size else 0.0

    # 缝：相邻步的 p/a 跳变（用参考量更公平，这里用实测）
    dp = float(np.max(np.linalg.norm(np.diff(P, axis=0), axis=1))) if len(P) > 1 else 0.0
    A = np.array(trace["aref"])
    da = float(np.max(np.linalg.norm(np.diff(A, axis=0), axis=1))) if len(A) > 1 else 0.0

    print()
    print("== result ==")
    print("  sim time            : %.1f s (%.0f steps)" % (T[-1], len(T)))
    print("  box hits            : %d / %d" % (rep["hits"], rep["n"]))
    print("  min dist to boxes   : %s" % " ".join("%.2f" % d for d in rep["min_dist"]))
    print("  cruise speed p90    : %.2f m/s   (mean %.2f)" % (v_cruise_meas, v_mean))
    print("  min trunk clearance : %.3f m" % worst_clr)
    print("  max |dp| per step   : %.4f m" % dp)
    print("  max |da_ref| / step : %.4f m/s^2" % da)
    print("  replans             : %d (fails %d)" % (srv.n_replans, plan_fails))

    ok = (rep["hits"] == rep["n"] and v_cruise_meas >= 4.0
          and worst_clr > 0.0 and plan_fails <= max(2, srv.n_replans // 10))
    print("  VERDICT: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
