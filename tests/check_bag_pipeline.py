#!/usr/bin/env python3
"""check_bag_pipeline.py -- the Phase 2 gate, no flight required.

Publishes a KNOWN Odometry stream and a KNOWN PositionCommand stream, records them with
`scripts/record_bag.sh` (split writers, the same script the flight uses), derives the CSV layer with
`scripts/bag_to_csv.py`, and checks the derived rows against the published values.

This is what makes the Phase 2 claim testable in `tests/run_all.sh`: the derivation is exercised on every
gate run, so a change to either side of the pipeline cannot silently break the CSVs the analysis reads.

  python3 tests/check_bag_pipeline.py            # PASS/FAIL + a one-line summary
  python3 tests/check_bag_pipeline.py --keep     # leave /tmp/sp_bagtest for inspection
"""
from __future__ import annotations

import os
import shutil
import signal
import subprocess
import sys
import time

R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = "/tmp/sp_bagtest"

PUB = r'''
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand

rclpy.init()
n = Node("bagtest_pub")
qos_be = QoSProfile(depth=200, history=HistoryPolicy.KEEP_LAST,
                    reliability=ReliabilityPolicy.BEST_EFFORT)
qos_rel = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
pub_o = n.create_publisher(Odometry, "/lio/robo/odom", qos_be)
pub_c = n.create_publisher(PositionCommand, "/plan/cmd", qos_rel)
t0 = time.monotonic()
k = 0
while time.monotonic() - t0 < 12.0:
    t = n.get_clock().now().to_msg()
    o = Odometry()
    o.header.stamp = t
    o.header.frame_id = "lio_world"
    o.child_frame_id = "base_link"
    o.pose.pose.position.x = 0.01 * k
    o.pose.pose.position.y = 0.02 * k
    o.pose.pose.position.z = 1.5
    o.pose.pose.orientation.w = 1.0
    o.twist.twist.linear.x = 1.0
    pub_o.publish(o)
    c = PositionCommand()
    c.header.stamp = t
    c.header.frame_id = "lio_world"
    c.position.x = 0.01 * k
    c.position.y = 0.02 * k
    c.position.z = 2.0
    c.velocity.x = 1.0
    c.yaw = 0.5
    c.yaw_dot = 0.1
    c.trajectory_id = k % 5
    c.trajectory_flag = 1
    pub_c.publish(c)
    k += 1
    # 30 Hz, not 100: two independent subscribers (live recorder + rosbag2) must not lose different
    # subsets, otherwise "bag-derived == live" cannot be judged sample-for-sample (MEASURED: at 100 Hz
    # the overlap was 77-91 % with ZERO value mismatches -- loss, not corruption).
    time.sleep(0.033)
print("PUBLISHED", k, flush=True)
'''

ROS_ENV = (". %s/setup/env.sh; set +u; . /opt/ros/humble/setup.bash; "
           ". \"$LIO_WS/install/setup.bash\" 2>/dev/null; "
           ". \"$WS_SUPER/install/setup.bash\" 2>/dev/null; " % R)


def shell(cmd, log=None):
    out = open(log, "w") if log else subprocess.DEVNULL
    return subprocess.Popen(["bash", "-lc", ROS_ENV + cmd], cwd=R,
                            stdout=out, stderr=subprocess.STDOUT,
                            start_new_session=True)


def _kill_stale():
    """Kill recorders left by an earlier run of this test (they write to the same tmpfs path).

    MEASURED 2026-11-25: four `ros2 bag record` processes from previous gate runs were still alive and
    subscribing to /lio/robo/odom etc.; the gate then failed intermittently.  The test must own and reap
    its own processes -- including their process group, because `ros2 bag record` is a child of the shell
    we start.
    """
    import signal

    def _ancestors():
        out, pid = set(), os.getpid()
        while pid > 1:
            out.add(pid)
            try:
                fields = open("/proc/%d/status" % pid).read().split()
                pid = int(fields[fields.index("PPid:") + 1])
            except Exception:
                break
        return out

    skip = _ancestors()
    pat = "/dev/shm/sp_bagtest/bag"
    for pid in os.listdir("/proc"):
        if not pid.isdigit() or int(pid) in skip:
            continue
        try:
            argv = open("/proc/%s/cmdline" % pid, "rb").read().decode("utf-8", "replace").split("\x00")
        except Exception:
            continue
        # ONLY a rosbag2 recorder writing THIS test's bag -- checked on argv, not on the joined command line:
        # the shell that launched the gate carries this test's own source text in ITS command line, and a
        # substring match there killed the gate itself (MEASURED 2026-11-25, exit 9).
        if (len(argv) > 3 and argv[0].endswith("python3") and argv[1].endswith("/ros2")
                and argv[2] == "bag" and argv[3] == "record" and pat in argv):
            try:
                os.kill(int(pid), signal.SIGKILL)
            except Exception:
                pass


def main():
    keep = "--keep" in sys.argv
    _kill_stale()
    shutil.rmtree(OUT, ignore_errors=True)
    os.makedirs(OUT, exist_ok=True)
    shutil.rmtree("/dev/shm/sp_bagtest", ignore_errors=True)
    with open(os.path.join(OUT, "pub.py"), "w") as fh:
        fh.write(PUB)
    pub = shell("python3 %s/pub.py" % OUT, log=os.path.join(OUT, "pub.log"))
    time.sleep(1.0)
    bag = shell("bash %s/scripts/record_bag.sh %s /dev/shm/sp_bagtest" % (R, OUT),
                log=os.path.join(OUT, "bag.log"))
    # PHASE 5-3 REPLAY CI: the SAME data is also written by the live CSV recorders the flight uses
    # (`record-odom.py` -> lio.csv, `record-traj.py --csv traj.csv` -> traj.csv.cmd.csv).  After the bag is
    # derived, the two layers must agree sample-for-sample: that is the "bag -> derive == live" claim made
    # executable on every gate run, without needing a flight.
    live = os.path.join(OUT, "live")
    os.makedirs(live, exist_ok=True)
    rec_o = shell("python3 %s/scripts/record-odom.py --topic /lio/robo/odom --csv %s/lio.csv --seconds 0"
                  % (R, live), log=os.path.join(live, "record_odom.log"))
    rec_t = shell("python3 %s/scripts/record-traj.py --topic /plan/cmd --msg position_cmd "
                  "--csv %s/traj.csv --world %s/sim/worlds/forest_hex12.yaml "
                  "--calib-file %s/calib/calib.json --seconds 0"
                  % (R, live, R, R), log=os.path.join(live, "record_traj.log"))
    time.sleep(11.0)
    pub.send_signal(signal.SIGINT)
    bag.send_signal(signal.SIGINT)
    for p in (rec_o, rec_t):
        try:
            p.send_signal(signal.SIGINT)
        except Exception:
            pass
    # wait for the recorder to CLOSE the bag (metadata.yaml appears when the storage is finalised)
    meta = "/dev/shm/sp_bagtest/bag/metadata.yaml"
    for _ in range(100):
        if os.path.isfile(meta):
            break
        time.sleep(0.1)
    # KILL THE PROCESS GROUP: `ros2 bag record` is a child of the shell we started, and a SIGINT to that shell
    # does not reliably reach it -- MEASURED 2026-11-25: every gate run left one recorder alive, subscribing
    # to the topics and writing into the tmpfs path, and after four of them the gate itself went flaky.
    import signal as _signal
    for p in (pub, bag, rec_o, rec_t):
        try:
            os.killpg(p.pid, _signal.SIGKILL)      # start_new_session=True => the group id IS the pid
        except Exception:
            pass
        try:
            p.kill()
        except Exception:
            pass
    _kill_stale()                               # and sweep for anything that escaped the group
    time.sleep(0.5)
    shmbag = "/dev/shm/sp_bagtest/bag"
    listing = sorted(os.listdir(shmbag)) if os.path.isdir(shmbag) else ["<no bag dir>"]
    print("bag dir contents:", listing, flush=True)
    log = os.path.join(OUT, "derive.log")
    with open(log, "w") as fh:
        rc = subprocess.call(["bash", "-lc", ROS_ENV +
                              "python3 %s/scripts/bag_to_csv.py --bag /dev/shm/sp_bagtest/bag "
                              "--derive-all --outdir %s/csv" % (R, OUT)], cwd=R,
                             stdout=fh, stderr=subprocess.STDOUT)
    ok = rc == 0
    details = []
    for name, header, checks in (
        ("lio.csv", "t_us,x,y,z,qw,qx,qy,qz,vx,vy,vz,wx,wy,wz,frame_id,child_frame_id",
         lambda r: abs(float(r[2]) - 2.0 * float(r[1])) < 1e-6 and r[4] == "1.000000000"),
        ("traj.csv.cmd.csv", "t_us,x_l,y_l,z_l,vx_l,vy_l,vz_l,ax_l,ay_l,az_l,yaw,yaw_dot,"
                             "trajectory_id,trajectory_flag,frame_id",
         lambda r: abs(float(r[2]) - 2.0 * float(r[1])) < 1e-6 and float(r[11]) == 0.1),
    ):
        path = os.path.join(OUT, "csv", name)
        if not os.path.isfile(path):
            ok = False
            details.append("%s missing" % name)
            continue
        lines = open(path).read().splitlines()
        good = lines and lines[0] == header and len(lines) > 20 and checks(lines[1].split(","))
        ok = ok and bool(good)
        details.append("%s %d rows %s" % (name, len(lines) - 1, "ok" if good else "BAD"))
    # ---- PHASE 5-3: bag-derived CSVs == live-recorder CSVs (sample for sample) --------------------
    def compare_layers(live_path, derived_path, label):
        import csv as _csv
        if not (os.path.isfile(live_path) and os.path.isfile(derived_path)):
            return None, "%s: missing live or derived file" % label
        live_rows = list(_csv.DictReader(open(live_path)))
        der_rows = list(_csv.DictReader(open(derived_path)))
        if not live_rows or not der_rows:
            return None, "%s: empty layer" % label
        by_t = {}
        for r in der_rows:
            by_t.setdefault(int(r["t_us"]), r)
        cols = [c for c in live_rows[0] if c != "t_us"]
        by_live = {}
        for r in live_rows:
            by_live.setdefault(int(r["t_us"]), r)
        matched = bad = 0
        # iterate the DERIVED rows: that is the layer accept.py reads, and two independent subscribers can
        # legitimately see different subsets under loss (the live recorder is a superset here: it starts
        # fractionally earlier).  What must hold is "every derived sample exists in the live layer, with the
        # same values" -- i.e. deriving the bag loses/changes nothing.
        for d in der_rows:
            r = by_live.get(int(d["t_us"]))
            if r is None:
                continue
            matched += 1
            for c in cols:
                a, b = r.get(c, ""), d.get(c, "")
                try:
                    if abs(float(a) - float(b)) > 1e-6:
                        bad += 1
                        break
                except ValueError:
                    if a != b:
                        bad += 1
                        break
        frac = matched / max(1, len(der_rows))
        okc = frac >= 0.95 and bad == 0
        return okc, ("%s: live %d / derived %d rows, matched %.1f%% of derived, value mismatches %d"
                     % (label, len(live_rows), len(der_rows), 100.0 * frac, bad))

    for label, live_name, der_name in (
        ("lio", os.path.join(live, "lio.csv"), os.path.join(OUT, "csv", "lio.csv")),
        ("traj.cmd", os.path.join(live, "traj.csv.cmd.csv"), os.path.join(OUT, "csv", "traj.csv.cmd.csv")),
    ):
        okc, msg = compare_layers(live_name, der_name, label)
        if okc is None:
            ok = False
            details.append(msg)
        else:
            details.append(msg)
            ok = ok and okc
    if not ok:
        tail = open(os.path.join(OUT, "derive.log")).read().strip().splitlines()[-3:]
        details.append("derive: " + " | ".join(tail))
    print("bag pipeline: %s (%s)" % ("PASS" if ok else "FAIL", ", ".join(details)))
    shutil.rmtree("/dev/shm/sp_bagtest", ignore_errors=True)
    if not keep:
        shutil.rmtree(OUT, ignore_errors=True)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
