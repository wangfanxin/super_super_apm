#!/usr/bin/env python3
"""离线回放：把一次**录制**里的 `/lio/robo/odom` 与 `/lio/cloud_world` 重新发出来。

为什么需要它
------------
这台机器上 SITL 是**单占用**的：两条 session 同时飞时实测 `rtf=0.51`，harness 自己的
host gate 会判 "THIS RUN'S NUMBERS ARE NOT GRADEABLE"。所以"SUPER 到底在不在环里"这个问题
不应该依赖一次飞行 —— 用录好的 odom + 点云回放，就能把**规划链**（SUPER + 薄层 + `/plan/cmd` 契约）
完全离线地验一遍。

用法
----
  ROS_DOMAIN_ID=77 python3 replay_lio.py --run /path/to/missionA-gate-01 [--speed 1.0] [--loop]

发布
----
  /lio/robo/odom    nav_msgs/Odometry  BEST_EFFORT depth 200，pose 来自 lio.csv（L 系）
  /lio/odom         nav_msgs/Odometry  同上 + diff 出来的 twist（与真机约定一致：
                                       Super-LIO 只填 pose，`/lio/odom` 才有速度）
  /lio/cloud_world  sensor_msgs/PointCloud2  BEST_EFFORT，来自 cloud.csv（scan,t_us,x,y,z）

注意：这是**链路**测试，不是感知测试 —— 点云是同一份静态地图重复发。
"""
from __future__ import annotations

import argparse
import csv
import os
import sys

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def read_lio(path):
    t, p, q = [], [], []
    with open(path) as fh:
        rdr = csv.DictReader(fh)
        for r in rdr:
            t.append(int(float(r["t_us"])))
            p.append([float(r["x"]), float(r["y"]), float(r["z"])])
            q.append([float(r["qw"]), float(r["qx"]), float(r["qy"]), float(r["qz"])])
    return np.array(t, dtype=np.int64), np.array(p), np.array(q)


def read_cloud(path):
    """cloud.csv: scan,t_us,x,y,z（同一 scan 多行）。返回 (scans[list of np.array], scan_t_us)。"""
    scans, times, cur, curt = [], [], [], None
    with open(path) as fh:
        rdr = csv.DictReader(fh)
        for r in rdr:
            s = int(r["scan"])
            if curt is None:
                curt = s
            if s != curt:
                if cur:
                    scans.append(np.array(cur, dtype=np.float32)); times.append(t)
                cur, curt = [], s
            cur.append([float(r["x"]), float(r["y"]), float(r["z"])])
            t = int(r["t_us"])
    if cur:
        scans.append(np.array(cur, dtype=np.float32)); times.append(t)
    return scans, np.array(times, dtype=np.int64)


class Replay(Node):
    def __init__(self, args):
        super().__init__("whtf_replay_lio")
        be = QoSProfile(depth=200, history=HistoryPolicy.KEEP_LAST,
                        reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub_robo = self.create_publisher(Odometry, "/lio/robo/odom", be)
        self.pub_lio = self.create_publisher(Odometry, "/lio/odom", be)
        self.pub_cloud = self.create_publisher(PointCloud2, "/lio/cloud_world", be)

        self.t, self.p, self.q = read_lio(os.path.join(args.run, "lio.csv"))
        self.scans, self.scan_t = read_cloud(os.path.join(args.run, "cloud.csv"))
        if args.z_offset:
            # ★ 只抬**位姿**，不抬点云 ★
            #   一开始把 cloud 也一起抬了，于是"地面"跟着到 1.9 m，飞机贴地 → SUPER 报
            #   "Local start point is deeply occupied"。只抬位姿 = 让飞机在**原来的世界**里从空中起步。
            self.p = self.p + np.array([0.0, 0.0, float(args.z_offset)])
            print("replay: 位姿 z 偏移 %+.2f m（点云不动；链路测试用）" % args.z_offset, flush=True)
        self.speed = float(args.speed)
        self.loop = bool(args.loop)
        self.i = 0
        # ★ 单位：lio.csv 的 t_us 是**微秒**，而墙钟差是**秒** ★
        #   一开始漏了这个换算，于是 `t0_rel[i] <= t_rel` 只在第一个样本成立 ——
        #   整段回放只发出 1 帧 odom（SUPER 于是打印 "No odom received"，薄层 odom 计数 0）。
        self.t0_rel = (self.t - self.t[0]) * 1e-6
        self.wall0 = self.get_clock().now().nanoseconds * 1e-9
        self.n_pub = 0
        self.n_scan = 0
        self.next_scan = 0.0
        self.done = False
        self.cloud_msg = self._mk_cloud(self.scans[0] if self.scans else np.zeros((2, 3), np.float32))
        self.create_timer(0.002, self.tick)          # 500 Hz 轮询，按录制时间戳节流
        print("replay: %d odom samples (%.1f s), %d cloud scans"
              % (len(self.t), float(self.t0_rel[-1]), len(self.scans)), flush=True)

    def _mk_cloud(self, pts):
        m = PointCloud2()
        m.header = Header()
        m.header.frame_id = "world"
        n = len(pts)
        d = np.empty((n, 4), dtype=np.float32)
        d[:, 0:3] = pts
        d[:, 3] = 50.0
        m.height = 1
        m.width = n
        m.fields = [PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1)]
        m.is_bigendian = False
        m.point_step = 16
        m.row_step = 16 * n
        m.is_dense = True
        m.data = d.tobytes()
        return m

    def tick(self):
        if self.done:
            return
        now = self.get_clock().now().nanoseconds * 1e-9 - self.wall0
        t_rel = now * self.speed
        # 发所有 <= t_rel 的 odom 样本（录制是 200 Hz，所以每拍 1 个左右）
        while self.i < len(self.t) and self.t0_rel[self.i] <= t_rel:
            k = self.i
            stamp = self.get_clock().now().to_msg()
            for pub in (self.pub_robo, self.pub_lio):
                m = Odometry()
                m.header.stamp = stamp
                m.header.frame_id = "world"
                m.child_frame_id = "body"
                m.pose.pose.position.x = float(self.p[k][0])
                m.pose.pose.position.y = float(self.p[k][1])
                m.pose.pose.position.z = float(self.p[k][2])
                m.pose.pose.orientation.w = float(self.q[k][0])
                m.pose.pose.orientation.x = float(self.q[k][1])
                m.pose.pose.orientation.y = float(self.q[k][2])
                m.pose.pose.orientation.z = float(self.q[k][3])
                if pub is self.pub_lio:
                    # 只有 /lio/odom 带速度：用录制位姿差分（与真机 `lio/odom` 的角色一致）
                    if k + 1 < len(self.t):
                        dt = (self.t[k + 1] - self.t[k]) * 1e-6
                        v = (self.p[k + 1] - self.p[k]) / dt if dt > 1e-6 else np.zeros(3)
                    else:
                        v = np.zeros(3)
                    m.twist.twist.linear.x = float(v[0])
                    m.twist.twist.linear.y = float(v[1])
                    m.twist.twist.linear.z = float(v[2])
                pub.publish(m)
            self.n_pub += 1
            self.i += 1
        # 点云按 10 Hz 循环发（同一份静态地图）
        if now >= self.next_scan:
            self.next_scan = now + 0.1
            self.cloud_msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_cloud.publish(self.cloud_msg)
            self.n_scan += 1
        if self.i >= len(self.t):
            if self.loop:
                self.i = 0
                self.wall0 = self.get_clock().now().nanoseconds * 1e-9
            elif not self.done:
                # 不要在 timer 里 raise SystemExit（会把进程打死、也是噪声）：
                # 打开 done 标志；点云继续发（ROG-Map 需要地图一直在）。
                self.done = True
                print("replay: odom 播完 — odom %d 帧, cloud %d 帧（继续发点云）"
                      % (self.n_pub, self.n_scan), flush=True)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True, help="一次 sortie 的目录（要含 lio.csv 与 cloud.csv）")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--loop", action="store_true")
    ap.add_argument("--z-offset", type=float, default=0.0,
                    help="把整个回放抬高多少米。用于**链路测试**：录制是从停机坪开始的，"
                         "而 SUPER 的地图把地面+膨胀判成占据 → 会报 'Local start point is deeply "
                         "occupied'。抬高 2 m 让它从空中起步，就能验规划链本身（与感知无关）。")
    args = ap.parse_args()
    for f in ("lio.csv", "cloud.csv"):
        if not os.path.exists(os.path.join(args.run, f)):
            print("missing %s in %s" % (f, args.run), file=sys.stderr)
            return 2
    rclpy.init()
    node = Replay(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:                                   # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
