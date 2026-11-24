#!/usr/bin/env python3
"""两个小节点合一个：把 SUPER 的输出接到 lio_avoid 的 bridge 上，并提供世界点云。

为什么需要它
------------
  ① **类型不匹配**：SUPER 发 `mars_quadrotor_msgs/msg/PositionCommand`，
     而 `lio_avoid/src/bridge` 订 `quadrotor_msgs/msg/PositionCommand`。
     两个包的字段几乎一样但类型不同，DDS 不会互认 → 必须显式转。
  ② **没有点云源**：ROG-Map 要 `/cloud_registered`（world 系）。
     这里把 `worlds/forest_square15.yaml` 的树干表面采样成点云发出去。

用法（两个 workspace 都要 source）：
  python3 src/whtf/ros/pc_adapter.py --cloud
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
sys.path.insert(0, os.path.join(ROOT, "src"))

from whtf.common.obstacles import load_world  # noqa: E402

WORLD = os.path.join(ROOT, "worlds", "forest_square15.yaml")


def trunk_points(pillars, z_split, z_lo=0.05, z_hi=4.15, nz=10, per_face=6):
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


def make_cloud(points, stamp):
    """★ 必须带 intensity 字段 ★
    ROG-Map 有 `intensity_thresh` 过滤；点云没有 intensity 字段时它读不到有效值，
    实测结果是整张图 0 个占据格子（/rog_map/occ width=0），规划器随即报
    "start point in obstacle"。加一列常数 intensity 即可。
    """
    n = len(points)
    data = np.empty((n, 4), dtype=np.float32)
    data[:, 0:3] = points
    data[:, 3] = 50.0                      # intensity
    msg = PointCloud2()
    msg.header = Header()
    msg.header.stamp = stamp
    msg.header.frame_id = "world"
    msg.height = 1
    msg.width = n
    msg.fields = [PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                  PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1)]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * n
    msg.is_dense = True
    msg.data = data.tobytes()
    return msg


class Adapter(Node):
    def __init__(self, args):
        super().__init__("whtf_pc_adapter")
        be = QoSProfile(depth=10)
        be.reliability = ReliabilityPolicy.BEST_EFFORT

        self.in_type = None
        self.out_type = None
        try:
            from mars_quadrotor_msgs.msg import PositionCommand as InT
            self.in_type = InT
        except Exception as exc:                    # noqa: BLE001
            self.get_logger().error("mars_quadrotor_msgs 不可用: %s" % exc)
        try:
            from quadrotor_msgs.msg import PositionCommand as OutT
            self.out_type = OutT
        except Exception as exc:                    # noqa: BLE001
            self.get_logger().error("quadrotor_msgs 不可用: %s" % exc)

        if self.in_type is not None and self.out_type is not None:
            # ★ 出口必须是 RELIABLE ★
            #   lio_avoid 的 bridge 用 reliable 订 /plan/cmd；用 best-effort 发会得到
            #   "New publisher discovered ... incompatible QoS ... No messages will be received"，
            #   表现就是 bridge 的 plan_rx=0、sent=0（实测踩了两次同类问题）。
            rel = QoSProfile(depth=10)
            rel.reliability = ReliabilityPolicy.RELIABLE
            # 还要 TRANSIENT_LOCAL：bridge 用的是 latched 订阅，
            # 只改 reliability 时错误会从 RELIABILITY 变成 DURABILITY（实测）。
            rel.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.pub = self.create_publisher(self.out_type, "/plan/cmd", rel)
            self.create_subscription(self.in_type, "/planning/pos_cmd", self.on_cmd, be)
            self.get_logger().info("adapter: mars PositionCommand -> quadrotor_msgs /plan/cmd")
        self.n = 0

        if args.cloud:
            pillars, z_split, _ = load_world(WORLD)
            self.pts = trunk_points(pillars, z_split)
            self.cloud_pub = self.create_publisher(PointCloud2, "/cloud_registered", be)
            self.create_timer(0.1, self.pub_cloud)
            self.get_logger().info("cloud: %d pts on /cloud_registered" % len(self.pts))

    def pub_cloud(self):
        self.cloud_pub.publish(make_cloud(self.pts, self.get_clock().now().to_msg()))

    def on_cmd(self, m):
        """mars PositionCommand -> quadrotor_msgs PositionCommand。

        ★ 两条防御，都是实测踩出来的 ★
        ① `frame_id` 必须显式写成 `lio_world`：lio_avoid 的 bridge 用白名单
           （只认 `lio_world` / `world`），空的或别的名字会被 silent 丢弃。
        ② 字段一律 getattr 兜底：mars 消息的字段名若与 quadrotor_msgs 不完全一致，
           直接访问会抛 AttributeError，回调静默死掉（表现就是 bridge 的 plan_rx=0）。
        """
        def vec(src, name):
            v = getattr(src, name, None)
            if v is None:
                return (0.0, 0.0, 0.0)
            try:
                return (float(v.x), float(v.y), float(v.z))
            except AttributeError:
                return (0.0, 0.0, 0.0)

        o = self.out_type()
        o.header = m.header
        o.header.frame_id = "lio_world"          # ★ ①
        px, py, pz = vec(m, "position")
        vx, vy, vz = vec(m, "velocity")
        ax, ay, az = vec(m, "acceleration")      # ★ ②
        o.position.x, o.position.y, o.position.z = px, py, pz
        o.velocity.x, o.velocity.y, o.velocity.z = vx, vy, vz
        o.acceleration.x, o.acceleration.y, o.acceleration.z = ax, ay, az
        o.yaw = float(getattr(m, "yaw", 0.0))
        o.yaw_dot = float(getattr(m, "yaw_dot", 0.0))
        for attr, default in (("trajectory_id", 1), ("trajectory_flag", 1)):
            setattr(o, attr, int(getattr(m, attr, default)))
        self.pub.publish(o)
        self.n += 1
        if self.n == 1:
            # 只打第一条：确认真实收到的 frame 与字段，避免再靠猜
            self.get_logger().info(
                "first cmd: frame_in=%r -> out=%r  p=(%.2f,%.2f,%.2f) v=(%.2f,%.2f,%.2f) yaw=%.2f"
                % (getattr(m.header, "frame_id", "?"), o.header.frame_id,
                   px, py, pz, vx, vy, vz, o.yaw))
        elif self.n % 100 == 0:
            self.get_logger().info("relayed %d cmds, last p=(%.2f,%.2f,%.2f)"
                                   % (self.n, px, py, pz))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cloud", action="store_true", help="同时发 /cloud_registered")
    args, _ = ap.parse_known_args()
    rclpy.init()
    node = Adapter(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
