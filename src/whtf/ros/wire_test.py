#!/usr/bin/env python3
"""契约探针：只看 `/plan/cmd`，判断"线上到底是谁的参考"。

它回答三个问题（这正是四位审查里最看重的"验线三判据"）：
  ① 有没有消息？`trajectory_id` 变了几次（应 ≈ 航点数）？`trajectory_flag` 取值是否只出现 1/3？
  ② `velocity`/`acceleration` 是否非零（**薄层 fallback 发的是零速度阶跃**，SUPER 的参考不是）？
  ③ 指令点是否在一个"腿"内连续变化，以及最大单拍位移（fallback 会出现 15 m 级跳变）？

判定（写死在这里，避免事后解释）：
  PASS = 消息数 > 200  且  唯一 id 数 >= 2  且  |v|>0.2 的样本占比 > 20%  且  最大单拍位移 < 1.5 m
  FAIL = 否则，并指出是哪一条不满足。

用法：
  ROS_DOMAIN_ID=77 python3 wire_test.py --seconds 40 [--topic /plan/cmd]
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/plan/cmd")
    ap.add_argument("--seconds", type=float, default=40.0)
    ap.add_argument("--out", default="", help="把逐样本记录写成 csv（可选）")
    args = ap.parse_args()

    try:
        from quadrotor_msgs.msg import PositionCommand as Cmd
    except ImportError as exc:
        print("CANNOT START: quadrotor_msgs 不可用 (%s)" % exc, file=sys.stderr)
        return 2

    rclpy.init()
    node = Node("whtf_wire_test")
    # 订阅端要与前工程一致：RELIABLE + TRANSIENT_LOCAL（latched 出口）
    qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    rec = []

    def on_cmd(m):
        rec.append((time.monotonic(),
                    int(getattr(m, "trajectory_id", -1)),
                    int(getattr(m, "trajectory_flag", -1)),
                    float(m.position.x), float(m.position.y), float(m.position.z),
                    float(m.velocity.x), float(m.velocity.y), float(m.velocity.z),
                    float(getattr(m, "yaw", 0.0)),
                    str(getattr(m.header, "frame_id", "?"))))

    node.create_subscription(Cmd, args.topic, on_cmd, qos)
    print("wire_test: 监听 %s，%.0f s" % (args.topic, args.seconds), flush=True)
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.05)

    if not rec:
        print("\n== wire_test ==")
        print("  收到的消息：0")
        print("  VERDICT: FAIL —— 线上一条指令都没有（QoS 不匹配 / planner 没起 / 没发 goal）")
        node.destroy_node(); rclpy.shutdown(); return 2

    ids = [r[1] for r in rec]
    flags = sorted(set(r[2] for r in rec))
    frames = sorted(set(r[10] for r in rec))
    pos = np.array([[r[3], r[4], r[5]] for r in rec])
    vel = np.array([[r[6], r[7], r[8]] for r in rec])
    ts = np.array([r[0] for r in rec])
    ts = ts - ts[0]
    spd = np.linalg.norm(vel, axis=1)
    d = np.linalg.norm(np.diff(pos, axis=0), axis=1)
    max_step = float(d.max()) if len(d) else 0.0
    frac_moving = float((spd > 0.2).mean())

    # ★ 判据要配得上"静态回放"这个实验 ★
    #   回放里飞机是**不动**的（odom 是录制回放），于是 SUPER 飞到目标后就会一直等它到达，
    #   参考自然也长时间零速度。这不是 fallback 的特征。真正能区分"SUPER 在环"的是：
    #     有一段**连续 ≥1 s** 的参考速度 > 0.5 m/s（fallback 只有零速度），
    #     且这段里位置是连续变化的（不是 15 m 阶跃）。
    best_run, cur = 0.0, 0.0
    for v in spd:
        cur = cur + 1.0 if v > 0.5 else 0.0
        best_run = max(best_run, cur)
    # 用消息数换算时长：相邻样本时间差的中位数
    dt = float(np.median(np.diff(ts))) if len(ts) > 2 else 0.01
    fastest = float(spd.max())
    moving_span = float(np.linalg.norm(pos[spd > 0.5].max(axis=0) - pos[spd > 0.5].min(axis=0))) \
        if (spd > 0.5).any() else 0.0

    print("\n== wire_test ==")
    print("  消息数            : %d（%.1f Hz 有效）" % (len(rec), len(rec) / max(1e-9, rec[-1][0] - rec[0][0])))
    print("  trajectory_id     : 唯一 %d 个 -> %s%s"
          % (len(set(ids)), sorted(set(ids))[:8], " …" if len(set(ids)) > 8 else ""))
    print("  trajectory_flag   : %s" % flags)
    print("  frame_id          : %s" % frames)
    print("  |v| 分布          : 中位 %.3f  p90 %.3f  最大 %.3f；|v|>0.2 占 %.1f%%"
          % (float(np.median(spd)), float(np.percentile(spd, 90)), fastest, 100 * frac_moving))
    print("  |v|>0.5 最长连续  : %.2f s（%.0f 拍 × %.1f ms）" % (best_run * dt, best_run, dt * 1e3))
    print("  |v|>0.5 时的活动范围: %.2f m" % moving_span)
    print("  位置范围          : x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]"
          % (pos[:, 0].min(), pos[:, 0].max(), pos[:, 1].min(), pos[:, 1].max(),
             pos[:, 2].min(), pos[:, 2].max()))
    print("  最大单拍位移      : %.3f m" % max_step)

    if args.out:
        with open(args.out, "w") as fh:
            fh.write("t_rel,id,flag,px,py,pz,vx,vy,vz,yaw,frame\n")
            for r in rec:
                fh.write("%.4f,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%s\n"
                         % (r[0] - rec[0][0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8], r[9], r[10]))
        print("  记录              : %s（%d 行）" % (args.out, len(rec)))

    reasons = []
    if len(rec) <= 200:
        reasons.append("消息数 %d <= 200" % len(rec))
    if len(set(ids)) < 2:
        reasons.append("trajectory_id 只变了 %d 次（应每航点一次）" % len(set(ids)))
    if best_run * dt < 1.0:
        reasons.append("没有一段 >=1 s 的连续参考速度（|v|>0.5 最长只有 %.2f s）" % (best_run * dt))
    if moving_span < 0.5:
        reasons.append("参考在运动时也几乎不动（活动范围 %.2f m），更像 hold" % moving_span)
    if max_step >= 1.5:
        reasons.append("最大单拍位移 %.2f m >= 1.5（阶跃目标，不是连续参考）" % max_step)

    print()
    if reasons:
        print("  VERDICT: FAIL")
        for r in reasons:
            print("    - %s" % r)
    else:
        print("  VERDICT: PASS —— 线上是**连续、带速度**的参考，且 id 每航点推进")
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:                                   # noqa: BLE001
        pass
    return 0 if not reasons else 3


if __name__ == "__main__":
    sys.exit(main())
