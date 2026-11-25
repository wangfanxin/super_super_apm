#!/usr/bin/env python3
"""viz_bridge.py -- the small adapter that turns the pipeline's own topics into what RViz can draw.

The pipeline deliberately publishes no "visualisation" messages: `/plan/cmd` is a `quadrotor_msgs/
PositionCommand` (RViz has no display for it), and odometry is a stream, not a path.  This node is the single
visualisation adapter, so the RViz config (`sim/rviz/pipeline.rviz`) can be the one place a human looks at
the whole chain -- world, truth, LIO, FCU estimate, the plan and the LIO's world cloud:

  in  /sim/truth/odom   (nav_msgs/Odometry, world_w)   -> /viz/truth_path  (nav_msgs/Path, its own frame)
  in  /lio/robo/odom    (nav_msgs/Odometry, lio_world) -> /viz/lio_path    (nav_msgs/Path, its own frame)
  in  /fcu/odom         (nav_msgs/Odometry, world_w)   -> /viz/fcu_path    (nav_msgs/Path, its own frame)
  in  /plan/cmd         (PositionCommand, lio_world)   -> /viz/plan_pose   (geometry_msgs/PoseStamped)
                                                          /viz/plan_path   (nav_msgs/Path)

Nothing here touches the flight path: it is read-only and only runs for human inspection / bag replay.
"""
from __future__ import annotations

import argparse
import math


def quat_from_R(R):
    """(w, x, y, z) from a 3x3 rotation (only the z-yaw matters for our frames, but keep it general)."""
    tr = float(R[0][0] + R[1][1] + R[2][2])
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (float(R[2][1]) - float(R[1][2])) / s
        y = (float(R[0][2]) - float(R[2][0])) / s
        z = (float(R[1][0]) - float(R[0][1])) / s
    elif R[0][0] > R[1][1] and R[0][0] > R[2][2]:
        s = math.sqrt(1.0 + float(R[0][0]) - float(R[1][1]) - float(R[2][2])) * 2.0
        w = (float(R[2][1]) - float(R[1][2])) / s
        x = 0.25 * s
        y = (float(R[0][1]) + float(R[1][0])) / s
        z = (float(R[0][2]) + float(R[2][0])) / s
    elif R[1][1] > R[2][2]:
        s = math.sqrt(1.0 + float(R[1][1]) - float(R[0][0]) - float(R[2][2])) * 2.0
        w = (float(R[0][2]) - float(R[2][0])) / s
        x = (float(R[0][1]) + float(R[1][0])) / s
        y = 0.25 * s
        z = (float(R[1][2]) + float(R[2][1])) / s
    else:
        s = math.sqrt(1.0 + float(R[2][2]) - float(R[0][0]) - float(R[1][1])) * 2.0
        w = (float(R[1][0]) - float(R[0][1])) / s
        x = (float(R[0][2]) + float(R[2][0])) / s
        y = (float(R[1][2]) + float(R[2][1])) / s
        z = 0.25 * s
    return w, x, y, z


def main() -> int:
    ap = argparse.ArgumentParser(description="pipeline -> RViz topic adapter (read-only)")
    ap.add_argument("--truth-topic", default="/sim/truth/odom")
    ap.add_argument("--lio-topic", default="/lio/robo/odom")
    ap.add_argument("--fcu-topic", default="/fcu/odom")
    ap.add_argument("--cmd-topic", default="/plan/cmd")
    ap.add_argument("--max-points", type=int, default=8000, help="points kept per path")
    args = ap.parse_args()

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry, Path
    from quadrotor_msgs.msg import PositionCommand
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init(args=[])
    node = Node("viz_bridge")
    best_effort = QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
    latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

    paths = {}

    def path_for(name):
        if name not in paths:
            p = Path()
            p.header.frame_id = "map"
            paths[name] = p
        return paths[name]

    def pub_for(name):
        if name.endswith("_path"):
            return node.create_publisher(Path, "/viz/" + name, 10)
        return node.create_publisher(PoseStamped, "/viz/" + name, 10)

    pubs = {n: pub_for(n) for n in ("truth_path", "lio_path", "fcu_path", "plan_path", "plan_pose")}

    def on_odom(name):
        def _cb(msg: Odometry):
            p = path_for(name)
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose = msg.pose.pose
            if not p.poses or p.header.frame_id != msg.header.frame_id:
                p.header.frame_id = msg.header.frame_id
            p.poses.append(pose)
            if len(p.poses) > args.max_points:
                del p.poses[: len(p.poses) - args.max_points]
            p.header.stamp = msg.header.stamp
            pubs[name].publish(p)
        return _cb

    def on_cmd(msg: PositionCommand):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = float(msg.position.x)
        pose.pose.position.y = float(msg.position.y)
        pose.pose.position.z = float(msg.position.z)
        yaw = float(msg.yaw)
        pose.pose.orientation.z = math.sin(0.5 * yaw)
        pose.pose.orientation.w = math.cos(0.5 * yaw)
        pubs["plan_pose"].publish(pose)
        p = path_for("plan_path")
        p.header.frame_id = msg.header.frame_id or p.header.frame_id
        p.poses.append(pose)
        if len(p.poses) > args.max_points:
            del p.poses[: len(p.poses) - args.max_points]
        p.header.stamp = msg.header.stamp
        pubs["plan_path"].publish(p)

    node.create_subscription(Odometry, args.truth_topic, on_odom("truth_path"), best_effort)
    node.create_subscription(Odometry, args.lio_topic, on_odom("lio_path"), best_effort)
    node.create_subscription(Odometry, args.fcu_topic, on_odom("fcu_path"), best_effort)
    try:
        node.create_subscription(PositionCommand, args.cmd_topic, on_cmd, latched)
    except Exception:                                       # noqa: BLE001
        pass

    node.get_logger().info("viz_bridge: %s -> /viz/truth_path, %s -> /viz/lio_path, %s -> /viz/plan_*"
                           % (args.truth_topic, args.lio_topic, args.cmd_topic))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
