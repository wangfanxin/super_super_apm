#!/usr/bin/env python3
"""无 Gazebo 的闭环：把 SUPER 当成被验证的黑盒，喂它 odom + 点云，收它的 PositionCommand。

它同时扮演三件事
----------------
  ① 传感器：把 worlds/forest_square15.yaml 的树干采样成点云，发 /cloud_registered（world 系）
  ② 里程计：把"当前状态"发成 /lidar_slam/odom
  ③ 被控对象：订 /planning/pos_cmd，按一个可配置的跟踪模型推进状态

跟踪模型两档
------------
  `--track perfect`  直接把 cmd 的 (p,v) 当成真实状态 —— 用来**孤立验证规划器**：
                     量出来的盒子命中/速度/净空就是 SUPER 轨迹本身的性质
  `--track lag`      一阶滞后跟随 —— 用来验证控制器存在时的鲁棒性

产出（`--out` 指定的 csv + 终端汇总）
---------------------------------
  12 个盒的最近距离、命中数、巡航速度、对树干的最近距离、命令跳变

用法（在 ws_super 里 source 过 overlay 之后）：
  python3 src/whtf/ros/headless_sim.py --seconds 90 --track perfect --out /tmp/run.csv
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion

HERE = os.path.dirname(os.path.abspath(__file__))          # .../src/whtf/ros
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))  # .../super_super_apm
sys.path.insert(0, os.path.join(ROOT, "src"))

from whtf.common.obstacles import footprint_at, load_world, min_clearance  # noqa: E402

WORLD = os.path.join(ROOT, "worlds", "forest_square15.yaml")
Z_FLY = 2.0
CORNERS = [(-7.5, -7.5), (7.5, -7.5), (7.5, 7.5), (-7.5, 7.5)]


def trunk_points(pillars, z_split, z_lo=0.05, z_hi=4.15, nz=10, per_face=6):
    """把树干采样成"表面点"：下半段方盒表面、上半段圆柱表面。"""
    pts = []
    for p in pillars:
        x, y, r = p["x"], p["y"], p["r"]
        for zi in range(nz + 1):
            z = z_lo + (z_hi - z_lo) * zi / nz
            if z < z_split:
                for k in range(per_face):
                    t = -r + 2 * r * k / max(per_face - 1, 1)
                    pts += [(x + r, y + t, z), (x - r, y + t, z),
                            (x + t, y + r, z), (x + t, y - r, z)]
            else:
                for k in range(4 * per_face):
                    a = 2 * math.pi * k / (4 * per_face)
                    pts.append((x + r * math.cos(a), y + r * math.sin(a), z))
    return np.array(pts, dtype=np.float32)


def make_cloud(points, frame="world"):
    msg = PointCloud2()
    msg.header = Header()
    msg.header.stamp = rclpy.clock.Clock().now().to_msg()
    msg.header.frame_id = frame
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.is_dense = True
    msg.data = points.tobytes()
    return msg


class HeadlessSim(Node):
    def __init__(self, args):
        super().__init__("whtf_headless_sim")
        self.args = args
        self.pillars, self.z_split, _ = load_world(WORLD)

        self.points = trunk_points(self.pillars, self.z_split)
        self.get_logger().info("cloud: %d pts (%s half at z=%.1f)"
                               % (len(self.points), footprint_at(Z_FLY, self.z_split), Z_FLY))

        be = QoSProfile(depth=10)
        be.reliability = ReliabilityPolicy.BEST_EFFORT
        rel = QoSProfile(depth=10)
        rel.reliability = ReliabilityPolicy.RELIABLE

        self.cloud_pub = self.create_publisher(PointCloud2, "/cloud_registered", be)
        self.odom_pub = self.create_publisher(Odometry, "/lidar_slam/odom", be)

        self.p = np.array([0.0, 0.0, Z_FLY])
        self.v = np.zeros(3)
        self.target_p = self.p.copy()
        self.target_v = np.zeros(3)
        self.got_cmd = False

        try:
            from mars_quadrotor_msgs.msg import PositionCommand
            self.cmd_type = PositionCommand
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().error("cannot import mars_quadrotor_msgs.msg.PositionCommand: %s" % exc)
            raise
        # SUPER 的 cmd_pub_ 用 best-effort；用 RELIABLE 订会静默丢光（实测报
        # "incompatible QoS ... RELIABILITY" 且一条都没收到）。
        self.create_subscription(self.cmd_type, "/planning/pos_cmd", self.on_cmd, be)

        self.trace = []
        self.dt = 1.0 / 100.0
        self.create_timer(self.dt, self.tick)
        self.create_timer(0.1, self.pub_cloud)

    # ---------------------------------------------------------------- callbacks
    def on_cmd(self, msg):
        try:
            p = np.array([msg.position.x, msg.position.y, msg.position.z])
            v = np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z])
        except AttributeError:
            p = np.array([msg.position.x, msg.position.y, msg.position.z])
            v = np.zeros(3)
        self.target_p = p
        self.target_v = v
        self.got_cmd = True

    def pub_cloud(self):
        self.cloud_pub.publish(make_cloud(self.points))

    def tick(self):
        # 被控对象
        if self.args.track == "perfect":
            if self.got_cmd:
                self.p = self.target_p.copy()
                self.v = self.target_v.copy()
        else:
            # 跟踪模型：代表"位置环跟不跟得住"。用 --kpos/--kvel/--amax 扫，
            # 就能算出 "SUPER 的参考需要多好的跟踪" —— 这是固件参数的目标。
            a = ((self.target_p - self.p) * self.args.kpos
                 + (self.target_v - self.v) * self.args.kvel)
            n = float(np.linalg.norm(a))
            if n > self.args.amax:
                a = a / n * self.args.amax
            self.v = self.v + a * self.dt
            self.p = self.p + self.v * self.dt
        self.publish_odom()
        self.trace.append((time.monotonic(), self.p.copy(), self.v.copy()))

    def publish_odom(self):
        m = Odometry()
        m.header.stamp = rclpy.clock.Clock().now().to_msg()
        m.header.frame_id = "world"
        m.child_frame_id = "body"
        m.pose.pose.position.x = float(self.p[0])
        m.pose.pose.position.y = float(self.p[1])
        m.pose.pose.position.z = float(self.p[2])
        m.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        m.twist.twist.linear.x = float(self.v[0])
        m.twist.twist.linear.y = float(self.v[1])
        m.twist.twist.linear.z = float(self.v[2])
        self.odom_pub.publish(m)

    # ---------------------------------------------------------------- scoring
    def report(self):
        if not self.trace:
            print("no trace")
            return 2
        P = np.array([t[1] for t in self.trace])
        V = np.array([t[2] for t in self.trace])
        T = np.array([t[0] for t in self.trace])
        T = T - T[0]

        boxes = CORNERS * 3
        d = np.array([[math.dist(p[:2], b) for b in boxes] for p in P])   # (N,12)
        min_d = d.min(axis=0)
        hits = int(np.sum(min_d <= 0.25))

        spd = np.linalg.norm(V, axis=1)
        near = d.min(axis=1) > 2.0
        cruise = float(np.percentile(spd[near], 90)) if near.any() else 0.0

        clr = min(min_clearance(p[0], p[1], self.pillars, Z_FLY, self.z_split) for p in P)
        dp = float(np.max(np.linalg.norm(np.diff(P, axis=0), axis=1))) if len(P) > 1 else 0.0

        print()
        print("== headless run (track=%s, %.1f s) ==" % (self.args.track, T[-1]))
        print("  box hits (<=0.25 m) : %d / 12" % hits)
        print("  min dist per box    : %s" % " ".join("%.2f" % x for x in min_d))
        print("  cruise v (p90, far) : %.2f m/s" % cruise)
        print("  min trunk clearance : %.3f m" % clr)
        print("  max |dp| per 10 ms  : %.4f m" % dp)
        ok = hits == 12 and cruise >= 4.0 and clr > 0.0
        print("  VERDICT: %s" % ("PASS" if ok else "FAIL"))

        if self.args.out:
            with open(self.args.out, "w", encoding="utf-8") as fh:
                fh.write("t,x,y,z,vx,vy,vz\n")
                for t, p, v in self.trace:
                    fh.write("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n"
                             % (t - T[0] + T[0], p[0], p[1], p[2], v[0], v[1], v[2]))
            print("  trace -> %s" % self.args.out)
        return 0 if ok else 2


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=90.0)
    ap.add_argument("--track", choices=["perfect", "lag"], default="perfect")
    ap.add_argument("--kpos", type=float, default=4.0, help="lag 模式的位置增益")
    ap.add_argument("--kvel", type=float, default=3.0, help="lag 模式的速度增益")
    ap.add_argument("--amax", type=float, default=8.0, help="lag 模式的加速度限幅 m/s^2")
    ap.add_argument("--out", default="")
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = HeadlessSim(args)
    t0 = time.monotonic()
    rc = 0
    try:
        while time.monotonic() - t0 < args.seconds and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    rc = node.report()
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
