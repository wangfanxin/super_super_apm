#!/usr/bin/env python3
"""bag_to_csv.py -- Phase 2: derive the CSV layer from the rosbag2 recording.

The pipeline's target state records ONE rosbag2 per sortie, and the CSVs the analysis/acceptance code
reads are DERIVED from it.  That keeps every number replayable while leaving scripts/accept.py untouched.
The column layouts are byte-for-byte the ones scripts/record-odom.py and scripts/record-traj.py write, so a
derived CSV can be compared line-by-line with the one the live recorder produced.

    python3 scripts/bag_to_csv.py --bag .runs/missionA-x/bag --derive-all --outdir /tmp/csv
    python3 scripts/bag_to_csv.py --bag DIR --topic /lio/robo/odom --kind odom --csv lio.csv
    python3 scripts/bag_to_csv.py --bag DIR --topic /plan/cmd --kind poscmd --csv traj.csv.cmd.csv

Kinds:
  odom    nav_msgs/Odometry            -> t_us,x,y,z,qw,qx,qy,qz,vx,vy,vz,wx,wy,wz,frame_id,child_frame_id
  poscmd  quadrotor_msgs/PositionCommand -> t_us,x_l,y_l,z_l,vx_l,vy_l,vz_l,ax_l,ay_l,az_l,yaw,yaw_dot,
                                          trajectory_id,trajectory_flag,frame_id
  any     prints the message count per topic only (discovery)
"""
from __future__ import annotations

import argparse
import os
import sys

ODOM_CSV = "t_us,x,y,z,qw,qx,qy,qz,vx,vy,vz,wx,wy,wz,frame_id,child_frame_id"
POSCMD_CSV = ("t_us,x_l,y_l,z_l,vx_l,vy_l,vz_l,ax_l,ay_l,az_l,yaw,yaw_dot,trajectory_id,"
              "trajectory_flag,frame_id")


def open_reader(bag_dir):
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

    reader = SequentialReader()
    storage_id = "sqlite3"
    reader.open(StorageOptions(uri=bag_dir, storage_id=storage_id),
                ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    return reader


def topic_types(reader):
    return {t.name: t.type for t in reader.get_all_topics_and_types()}


def t_us(header):
    return int(header.stamp.sec) * 1_000_000 + int(header.stamp.nanosec) // 1000


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True, action="append",
                    help="rosbag2 directory (metadata.yaml holder); repeat to merge a SPLIT recording")
    ap.add_argument("--topic", default="")
    ap.add_argument("--kind", default="", choices=["", "odom", "poscmd", "any"])
    ap.add_argument("--csv", default="")
    ap.add_argument("--derive-all", action="store_true",
                    help="derive lio.csv (/lio/robo/odom) and traj.csv.cmd.csv (/plan/cmd) into --outdir")
    ap.add_argument("--outdir", default="")
    args = ap.parse_args()

    from rclpy.serialization import deserialize_message

    bags = args.bag if isinstance(args.bag, list) else [args.bag]

    def derive_any(topic, kind, out):
        for bag in bags:
            types = topic_types(open_reader(bag))
            if topic in types:
                return derive([bag], topic, kind, out, deserialize_message)
        print("bag_to_csv: topic %s not found in any bag (%s)" % (topic, ", ".join(bags)), file=sys.stderr)
        return 2

    if args.derive_all:
        if not args.outdir:
            ap.error("--derive-all needs --outdir")
        os.makedirs(args.outdir, exist_ok=True)
        jobs = [("/lio/robo/odom", "odom", os.path.join(args.outdir, "lio.csv")),
                ("/plan/cmd", "poscmd", os.path.join(args.outdir, "traj.csv.cmd.csv"))]
        rc = 0
        for topic, kind, out in jobs:
            rc |= derive_any(topic, kind, out)
        return rc

    if not args.topic:
        # discovery mode: print every bag's topics and counts
        print("%-34s %-52s %s" % ("topic", "type", "messages"))
        for bag in bags:
            reader = open_reader(bag)
            types = topic_types(reader)
            counts = {}
            while reader.has_next():
                topic, _data, _stamp = reader.read_next()
                counts[topic] = counts.get(topic, 0) + 1
            print("-- %s" % bag)
            for name, typ in sorted(types.items()):
                print("%-34s %-52s %d" % (name, typ, counts.get(name, 0)))
        return 0

    if not args.csv or not args.kind:
        ap.error("give --kind and --csv (or --derive-all/--outdir)")
    sys.exit(derive_any(args.topic, args.kind, args.csv))


def derive(bag_dirs, topic, kind, csv_path, deserialize):
    if isinstance(bag_dirs, str):
        bag_dirs = [bag_dirs]
    reader = None
    for bag_dir in bag_dirs:
        r = open_reader(bag_dir)
        types = topic_types(r)
        if topic in types:
            reader = r
            break
    if reader is None:
        print("bag_to_csv: topic %s not in any of %s" % (topic, bag_dirs), file=sys.stderr)
        return 2
    msg_type = types[topic]
    if kind == "odom":
        from nav_msgs.msg import Odometry as Msg
        if "Odometry" not in msg_type:
            print("bag_to_csv: %s is %s, not Odometry" % (topic, msg_type), file=sys.stderr)
            return 2
        header = ODOM_CSV
    else:
        from quadrotor_msgs.msg import PositionCommand as Msg
        if "PositionCommand" not in msg_type:
            print("bag_to_csv: %s is %s, not PositionCommand" % (topic, msg_type), file=sys.stderr)
            return 2
        header = POSCMD_CSV

    n = 0
    with open(csv_path, "w", encoding="utf-8") as fh:
        fh.write(header + "\n")
        while reader.has_next():
            t_name, data, _stamp = reader.read_next()
            if t_name != topic:
                continue
            m = deserialize(data, Msg)
            if kind == "odom":
                p, q = m.pose.pose.position, m.pose.pose.orientation
                v, w = m.twist.twist.linear, m.twist.twist.angular
                fh.write("%d,%.6f,%.6f,%.6f,%.9f,%.9f,%.9f,%.9f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s,%s\n"
                         % (t_us(m.header), p.x, p.y, p.z, q.w, q.x, q.y, q.z,
                            v.x, v.y, v.z, w.x, w.y, w.z, m.header.frame_id, m.child_frame_id))
            else:
                fh.write("%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%s\n"
                         % (t_us(m.header), m.position.x, m.position.y, m.position.z,
                            m.velocity.x, m.velocity.y, m.velocity.z,
                            m.acceleration.x, m.acceleration.y, m.acceleration.z,
                            m.yaw, m.yaw_dot, m.trajectory_id, m.trajectory_flag, m.header.frame_id))
            n += 1
    print("bag_to_csv: %s -> %s (%d rows)" % (topic, csv_path, n))
    return 0


if __name__ == "__main__":
    sys.exit(main())
