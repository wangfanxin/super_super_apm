#!/usr/bin/env python3
"""fake_fcu_inputs.py -- synthetic ROS inputs for the merged-client bench (tests/check_fcu_client.sh).

Publishes what the merged client consumes, with NO simulator and NO lidar in the loop:

  * `/lio/robo/odom` (Odometry, 200 Hz, BEST_EFFORT): a slow 1 m/s circle in the plan frame `lio_world`;
  * `/sim/truth/odom` (Odometry, 100 Hz, BEST_EFFORT): the same motion in W;
  * `/plan/cmd` (PositionCommand, 100 Hz, RELIABLE/TRANSIENT_LOCAL, frame `lio_world`): a fixed target;
  * `/plan/trigger` (PoseStamped, latched once at 2 s): the go, which makes the relay start writing 84.

`--lio-stop-after SECONDS` stops the LIO stream without stopping the truth -- that is the extnav AGE GATE
case the bench checks (331 must stop).
"""
from __future__ import annotations

import argparse
import math
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=14.0)
    ap.add_argument("--lio-stop-after", type=float, default=6.0)
    ap.add_argument("--trigger-after", type=float, default=2.0)
    ap.add_argument("--rate-lio", type=float, default=200.0)
    ap.add_argument("--rate-truth", type=float, default=100.0)
    ap.add_argument("--rate-cmd", type=float, default=100.0)
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from quadrotor_msgs.msg import PositionCommand

    try:
        rclpy.init(args=[])
    except Exception:
        pass
    n = Node("fake_fcu_inputs")
    be = QoSProfile(depth=200, history=HistoryPolicy.KEEP_LAST,
                    reliability=ReliabilityPolicy.BEST_EFFORT)
    latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub_lio = n.create_publisher(Odometry, "/lio/robo/odom", be)
    pub_truth = n.create_publisher(Odometry, "/sim/truth/odom", be)
    pub_cmd = n.create_publisher(PositionCommand, "/plan/cmd", latched)
    pub_trigger = n.create_publisher(PoseStamped, "/plan/trigger", latched)

    def pose_at(t):
        r = 2.0
        w = 0.5                      # rad/s -> 1 m/s tangential: well inside the 5 m/s nav gate
        x = r * math.cos(w * t) - r
        y = r * math.sin(w * t)
        vx = -r * w * math.sin(w * t)
        vy = r * w * math.cos(w * t)
        yaw = w * t
        return x, y, vx, vy, yaw

    def q_from_yaw(yaw):
        return math.cos(yaw / 2.0), math.sin(yaw / 2.0)

    t0 = time.monotonic()
    next_lio = next_truth = next_cmd = 0.0
    triggered = False
    while time.monotonic() - t0 < args.seconds:
        now = time.monotonic()
        t = now - t0
        x, y, vx, vy, yaw = pose_at(t)
        qw, qz = q_from_yaw(yaw)
        stamp = n.get_clock().now().to_msg()
        if now >= next_lio and t < args.lio_stop_after:
            next_lio += 1.0 / args.rate_lio
            m = Odometry()
            m.header.stamp = stamp
            m.header.frame_id = "lio_world"
            m.child_frame_id = "base_link"
            m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = x, y, 2.0
            m.pose.pose.orientation.w, m.pose.pose.orientation.z = qw, qz
            m.twist.twist.linear.x, m.twist.twist.linear.y = vx, vy
            pub_lio.publish(m)
        if now >= next_truth:
            next_truth += 1.0 / args.rate_truth
            m = Odometry()
            m.header.stamp = stamp
            m.header.frame_id = "world_w"
            m.child_frame_id = "truth_base_link"
            m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = x, y, 2.0
            m.pose.pose.orientation.w, m.pose.pose.orientation.z = qw, qz
            m.twist.twist.linear.x, m.twist.twist.linear.y = vx, vy
            pub_truth.publish(m)
        if now >= next_cmd:
            next_cmd += 1.0 / args.rate_cmd
            c = PositionCommand()
            c.header.stamp = stamp
            c.header.frame_id = "lio_world"
            c.position.x, c.position.y, c.position.z = 0.0, 0.0, 2.0
            c.yaw = 0.0
            c.trajectory_id = 1
            c.trajectory_flag = 1
            pub_cmd.publish(c)
        if not triggered and t >= args.trigger_after:
            triggered = True
            p = PoseStamped()
            p.header.stamp = stamp
            p.header.frame_id = "lio_world"
            pub_trigger.publish(p)
        rclpy.spin_once(n, timeout_sec=0.0)
        time.sleep(0.0005)
    print("fake_fcu_inputs: done", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
