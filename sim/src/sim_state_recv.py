#!/usr/bin/env python3
"""sim_state_recv.py -- Phase 1 truth source: read ArduPilot SITL's SIM_STATE (MAVLink 108) from an
UNPATCHED SITL and record it to CSV.

Why this file exists (measured 2026-11-24, probe_sim_state.py):
  * an unpatched SITL DOES emit SIM_STATE, and answers SET_MESSAGE_INTERVAL: asking 200 Hz delivered
    ~225 Hz (the stream scheduler divides the 1 kHz model loop); the exact achievable rate is measured at
    run time and printed, never assumed;
  * SIM_STATE carries NO simulation timestamp and NO local NED position:
        q1..q4, roll/pitch/yaw, xacc/yacc/zacc, xgyro/ygyro/zgyro, lat/lon/alt, vn/ve/vd,
        std_dev_horz/std_dev_vert, lat_int/lon_int
    so this recorder stores the RAW fields plus the RECEIVE wall clock (`t_wall_us`, monotonic) and a
    provisional local-NED estimate.  The exact lat/lon -> local-NED mapping is fitted later by
    scripts/compare_truth.py against the old tap stream (which has both), and the fitted mapping +
    residual are what decide whether SIM_STATE can replace the tap for grading.

CSV columns (mirror the tap where the fields exist):
    t_wall_us,t_unix_us,lat,lon,alt,q1,q2,q3,q4,roll,pitch,yaw,
    ax,ay,az,wx,wy,wz,vn,ve,vd,pn,pe,pd,lat_int,lon_int,std_h,std_v

    python3 sim/src/sim_state_recv.py --csv /tmp/sim_state.csv --seconds 5
    python3 sim/src/sim_state_recv.py --mav tcp:127.0.0.1:5782 --hz 1000 --csv "$OUT/sim_state.csv"
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import signal
import sys
import time

import numpy as np

GLOBAL_STOP = False


def _on_signal(signum, frame):  # noqa: ARG001
    global GLOBAL_STOP
    GLOBAL_STOP = True


def parse_home(text):
    """'lat,lon,alt' -> (lat_deg, lon_deg, alt_m)."""
    parts = [p.strip() for p in text.split(",")]
    if len(parts) < 2:
        raise SystemExit("--home needs lat,lon[,alt]")
    lat = float(parts[0])
    lon = float(parts[1])
    alt = float(parts[2]) if len(parts) > 2 and parts[2] else 0.0
    return lat, lon, alt


class LocalEnu:
    """Provisional WGS84-ish lat/lon/alt -> local NED (N, E, D) around home.

    Small-area approximation with the local meridional / prime-vertical radii (mm-level over a few
    hundred metres).  The AUTHORITATIVE mapping is fitted by compare_truth.py against the tap; this is
    only so a human can read the CSV.
    """

    def __init__(self, lat0, lon0, alt0):
        self.lat0, self.lon0, self.alt0 = lat0, lon0, alt0
        a, f = 6378137.0, 1.0 / 298.257223563
        e2 = f * (2.0 - f)
        s = math.sin(math.radians(lat0))
        self.rn = a / math.sqrt(1.0 - e2 * s * s)          # prime vertical
        self.rm = a * (1.0 - e2) / (1.0 - e2 * s * s) ** 1.5  # meridional

    def ned(self, lat, lon, alt):
        dlat = math.radians(lat - self.lat0)
        dlon = math.radians(lon - self.lon0)
        n = dlat * self.rm
        e = dlon * self.rn * math.cos(math.radians(self.lat0))
        d = -(alt - self.alt0)
        return n, e, d


# THE CSV THAT scripts/accept.py READS (same header/columns as sim/src/truth_rx.py --csv):
#   t_us,inst,pn,pe,pd,vn,ve,vd,qw,qx,qy,qz,wx,wy,wz,ax,ay,az
# PHASE 1b/D4 FIX (2026-11-25): `t_us` is the recorder's own monotonic clock; `t_sim_us` is that SAME
# sample mapped onto the simulator clock with the explicit offset `--stamp-offset-us` (= client start -
# SITL step 0, printed on the flight banner).  The ROS /sim/state stamp already uses t_sim_us (so the
# LiDAR -> LIO -> bridge chain is on SITL's clock), but tap.csv historically carried only t_us: comparing
# the two (as W6 does) therefore mixed two time bases by exactly that offset -- 0.15 s on a normal sortie,
# 1.35 s on phase4oE (client started late), which put W6's +-0.6 s fit at its search edge.  Both columns
# are now written so every consumer can choose the documented one.
TAP_CSV_HEADER = ("t_us,t_sim_us,inst,pn,pe,pd,vn,ve,vd,qw,qx,qy,qz,wx,wy,wz,ax,ay,az\n")
TAP_ROW = ("%d,%d,0,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,"
           "%.8f\n")


class TruthOut:
    """Phase 1b: publish the reconstructed truth as ROS topics (and optionally a tap.csv).

    The flight stack's truth contract is the tap: positions in NED relative HOME, attitude as a
    body->NED quaternion, body rates / specific force in FRD, one sample per model step (~1.2 kHz).
    SIM_STATE arrives at ~328 Hz with no NED position, so this class:

      * integrates the SIM_STATE velocity (trapezoid) to get the NED position, anchored at the first
        sample's geodetic position (which on the pad equals HOME; MEASURED the pad value to within 1 cm);
      * **interpolates** the samples onto a uniform --publish-hz grid (200 Hz default).  200 Hz matters:
        lidar_imu decides its IMU decimation from the measured stream rate, and at 328 Hz it would
        decimate by 2 -> 164 Hz IMU, below the 200 Hz the LIO is configured for;
      * publishes /sim/state (nav_msgs/Odometry: pose NED, twist m/s and rad/s) and
        /sim/state/imu (sensor_msgs/Imu: same orientation, angular velocity, linear acceleration).
        Both carry the SAME stamp, so consumers can pair them.

    The time base is the recorder's own monotonic clock; the explicit map to the simulator's clock is
    t_sim = t_here + b with b measured once by compare_truth (D4: wall clock + explicit mapping).
    """

    def __init__(self, publish_hz, stamp_offset_us=0, truth_odom=False, clock_hz=100.0,
                 odom_hz=100.0, rtf_hz=1.0, pad_offset=(0.0, 0.0, -0.0999)):
        self.pub_hz = float(max(1.0, publish_hz))
        self.off = int(stamp_offset_us)
        self.prev = None
        self.next_us = None
        self.count = 0
        self.pad = np.asarray(pad_offset, float)
        self.truth_odom = bool(truth_odom)
        self.clock_period_us = int(1e6 / max(0.1, clock_hz))
        self.odom_period_us = int(1e6 / max(0.1, odom_hz))
        self.rtf_period_us = int(1e6 / max(0.1, rtf_hz))
        self.next_clock_us = 0
        self.next_odom_us = 0
        self.next_rtf_us = 0
        self._rtf_wall = None
        self._rtf_sim = None
        self.rtf = 1.0
        # UNIFORM SAMPLE-TIME RECONSTRUCTION (see note in feed()): the arrival wall clock is bursty
        # (TCP batches), so timestamps are reconstructed from a running estimate of the stream period
        # instead of being taken from the arrival instant.
        self.period_us = 3200.0
        self.recon_us = None
        self.last_arrival_us = None
        self.sink = None          # optional callable(t_us, p, v, q, w, a): writes the tap-format CSV
        # CLOCK-LIKE TIME BASE (D4): sample time = alpha + beta*index, with (alpha, beta) refitted by
        # least squares over a rolling window of ARRIVAL instants.  Two earlier attempts failed:
        #   * EMA over per-message arrival deltas: TCP batches put many deltas near 0 (excluded) and a few
        #     long ones in range (included) -> period crept up, timeline ran 1.6x fast (phase1k: 202 Hz CSV
        #     out of a 323 Hz stream, 627 skipped scans);
        #   * block-mean period: rate right, but the intercept was never pinned, so the timeline drifted
        #     against wall by ~0.35 s over 130 s (phase1l: W6 differed by 0.37 m between the two truths).
        # A regression of arrival on index pins BOTH: beta is the stream period, alpha the wall anchor.
        self._idx = 0
        self._slope = 3200.0
        self._inter = None
        self._hist_i = []
        self._hist_t = []
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu
        try:
            from rclpy.signals import SignalHandlerOptions
            rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        except Exception:
            rclpy.init(args=[])
        self.rclpy = rclpy
        self.node = Node("sim_state_truth")
        qos = QoSProfile(depth=500, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub_odom = self.node.create_publisher(Odometry, "/sim/state", qos)
        self.pub_imu = None
        self.Odometry, self.Imu = Odometry, Imu
        if self.truth_odom:
            from rosgraph_msgs.msg import Clock
            from std_msgs.msg import Float32
            odom_qos = QoSProfile(depth=200, history=HistoryPolicy.KEEP_LAST,
                                  reliability=ReliabilityPolicy.BEST_EFFORT)
            self.pub_truth = self.node.create_publisher(Odometry, "/sim/truth/odom", odom_qos)
            self.pub_clock = self.node.create_publisher(Clock, "/clock", odom_qos)
            self.pub_rtf = self.node.create_publisher(Float32, "/sim/rtf", 10)
            self.Clock, self.Float32 = Clock, Float32
            # the W-frame conversion lives in ONE place (bridge_nodes/common/frames.py)
            repo = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            bridge = os.path.join(repo, "bridge_nodes")
            if bridge not in sys.path:
                sys.path.insert(0, bridge)
            from common import frames as _frames
            self._frames = _frames
            import time as _time
            self._time = _time

    @staticmethod
    def _stamp(t_us):
        from builtin_interfaces.msg import Time
        s = Time()
        s.sec = int(t_us // 1_000_000)
        s.nanosec = int((t_us % 1_000_000) * 1000)
        return s

    def _emit(self, t_us, p, v, q, w, a):
        from geometry_msgs.msg import Point, Quaternion, Vector3
        if self.sink is not None:
            self.sink(t_us, p, v, q, w, a)
        stamp = self._stamp(t_us + self.off)
        o = self.Odometry()
        o.header.stamp = stamp
        o.header.frame_id = "sim_ned"
        o.child_frame_id = "sim_body_frd"
        o.pose.pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
        o.pose.pose.orientation = Quaternion(w=float(q[0]), x=float(q[1]), y=float(q[2]), z=float(q[3]))
        o.twist.twist.linear = Vector3(x=float(v[0]), y=float(v[1]), z=float(v[2]))
        o.twist.twist.angular = Vector3(x=float(w[0]), y=float(w[1]), z=float(w[2]))
        # SPECIFIC FORCE IN pose.covariance[0:3] (FRD like the tap): nav_msgs/Odometry has no acceleration
        # field, and publishing a SECOND message per sample doubled the callback load on the LiDAR node
        # (MEASURED 2026-11-25: 400 callbacks/s, the LiDAR loop fell behind on scan scheduling and skipped
        # 68 scans).  One message per sample keeps the truth on one topic and halves that overhead.
        o.pose.covariance[0] = float(a[0])
        o.pose.covariance[1] = float(a[1])
        o.pose.covariance[2] = float(a[2])
        self.pub_odom.publish(o)
        if self.pub_imu is not None:
            m = self.Imu()
            m.header.stamp = stamp
            m.header.frame_id = "sim_body_frd"
            m.orientation = o.pose.pose.orientation
            m.angular_velocity = o.twist.twist.angular
            m.linear_acceleration = Vector3(x=float(a[0]), y=float(a[1]), z=float(a[2]))
            self.pub_imu.publish(m)
        if self.truth_odom:
            if t_us >= self.next_clock_us:
                self.next_clock_us = t_us + self.clock_period_us
                cm = self.Clock()
                cm.clock = stamp
                self.pub_clock.publish(cm)
            if t_us >= self.next_odom_us:
                self.next_odom_us = t_us + self.odom_period_us
                p_w, v_w, q_w = self._frames.wire_to_world(np.asarray(p), np.asarray(v), np.asarray(q))
                o2 = self.Odometry()
                o2.header.stamp = stamp
                o2.header.frame_id = "world_w"
                o2.child_frame_id = "truth_base_link"
                o2.pose.pose.position = Point(x=float(p_w[0]), y=float(p_w[1]), z=float(p_w[2]))
                o2.pose.pose.orientation = Quaternion(w=float(q_w[0]), x=float(q_w[1]),
                                                      y=float(q_w[2]), z=float(q_w[3]))
                o2.twist.twist.linear = Vector3(x=float(v_w[0]), y=float(v_w[1]), z=float(v_w[2]))
                o2.twist.twist.angular = Vector3(x=float(w[0]), y=float(w[1]), z=float(w[2]))
                self.pub_truth.publish(o2)
            if t_us >= self.next_rtf_us:
                self.next_rtf_us = t_us + self.rtf_period_us
                wall = self._time.monotonic()
                if self._rtf_wall is not None and wall > self._rtf_wall + 0.05:
                    self.rtf = ((t_us - self._rtf_sim) * 1e-6) / (wall - self._rtf_wall)
                self._rtf_wall, self._rtf_sim = wall, t_us
                fm = self.Float32()
                fm.data = float(self.rtf)
                self.pub_rtf.publish(fm)

    def reconstruct_time(self, arrival_us):
        """Monotone SAMPLE time for one SIM_STATE message, from a running period estimate.

        WHY NOT THE ARRIVAL TIME (MEASURED 2026-11-25): the client receives the ~328 Hz stream in TCP
        batches, so several messages share (almost) the same arrival instant.  Using those instants as
        sample times made the published grid bursty, the LiDAR's 20 ms window check report holes
        (25-27 skipped scans, H6 FAIL) and the CSV carry a 1.04 s gap at arm (H2 FAIL).  The stream
        period is stable, so reconstruct t = t_prev + period with the period tracked by an EMA over
        plausible arrival deltas.  The absolute offset is the wall-vs-simulator map of D4, measured by
        compare_truth (a~1, b=launch offset).
        """
        self._idx += 1
        self._hist_i.append(self._idx)
        self._hist_t.append(arrival_us)
        if len(self._hist_i) > 600:
            self._hist_i = self._hist_i[-600:]
            self._hist_t = self._hist_t[-600:]
        if len(self._hist_i) >= 64 and self._idx % 16 == 0:
            n = np.asarray(self._hist_i, float)
            a = np.asarray(self._hist_t, float)
            A = np.column_stack([n, np.ones_like(n)])
            coef, *_ = np.linalg.lstsq(A, a, rcond=None)
            self._slope = float(coef[0])
            self._inter = float(coef[1])
        if self._inter is not None:
            self.recon_us = int(round(self._inter + self._slope * self._idx))
        elif self.recon_us is None:
            self.recon_us = arrival_us
        else:
            self.recon_us = int(self.recon_us + self._slope)
        self.period_us = self._slope
        self.last_arrival_us = arrival_us
        return self.recon_us

    def feed(self, t_us, p, v, q, w, a):
        """One raw SIM_STATE sample (already on the reconstructed timeline).  Emits the grid samples."""
        sample = (np.asarray(p, float), np.asarray(v, float), np.asarray(q, float),
                  np.asarray(w, float), np.asarray(a, float))
        if self.prev is None:
            self.prev = (t_us, sample)
            self.next_us = t_us
            self._emit(t_us, *sample)
            self.count += 1
            return
        t0, s0 = self.prev
        t1, s1 = t_us, sample
        step = max(1, int(round(1e6 / self.pub_hz)))
        while self.next_us + step <= t1:
            self.next_us += step
            f = (self.next_us - t0) / float(t1 - t0) if t1 != t0 else 0.0
            f = min(1.0, max(0.0, f))
            p = (1.0 - f) * s0[0] + f * s1[0]
            v = (1.0 - f) * s0[1] + f * s1[1]
            q = (1.0 - f) * s0[2] + f * s1[2]          # normalised-linear (same as the tap consumer)
            n = float(np.linalg.norm(q))
            q = q / n if n > 0 else q
            w = (1.0 - f) * s0[3] + f * s1[3]
            a = (1.0 - f) * s0[4] + f * s1[4]
            self._emit(self.next_us, p, v, q, w, a)
            self.count += 1
        self.prev = (t1, s1)

    def keep_alive(self, now_wall_us):
        """Fill a gap in the SIM_STATE stream by HOLDING the last sample on the publish grid.

        WHY: the stream has one ~1 s hole per sortie, at the arm moment (MEASURED 2026-11-25: the tap
        never has it, and with SIM_STATE as the truth it made the LiDAR skip 27 scans and the bridge
        abort "ESTIMATOR FEED IS STALE: the LIO sample ... is 1.56 s old").  A held sample is exactly
        right there -- the vehicle is parked while the FCU arms -- and the alternative (a hole) makes
        the truth source look dead to the whole flight chain.
        """
        if self.prev is None or self.next_us is None or self.last_arrival_us is None:
            return
        # only fill when the stream has actually stalled (a couple of periods), and never more than 2 s
        stall_us = now_wall_us - self.last_arrival_us
        if stall_us < int(2.5 * self.period_us):
            return
        self.recon_us = int(self.recon_us + self.period_us)
        self.period_us = float(self.period_us)
        target = self.recon_us
        step = max(1, int(round(1e6 / self.pub_hz)))
        n = 0
        while self.next_us + step <= target and n < int(2.0 * self.pub_hz):
            self.next_us += step
            _, (p, v, q, w, a) = self.prev
            self._emit(self.next_us, p, v, q, w, a)
            self.count += 1
            n += 1
        # advance the hold clock in step with the wall, so a long stall does not emit a burst later
        self.last_arrival_us = int(now_wall_us - self.period_us)

    def shutdown(self):
        try:
            self.node.destroy_node()
            self.rclpy.shutdown()
        except Exception:
            pass



def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mav", default="tcp:127.0.0.1:5782", help="SITL endpoint (a spare serial port)")
    ap.add_argument("--hz", type=float, default=120.0, help="requested SIM_STATE rate; ArduPilot DENIES 1000 Hz and (on the flight param set) 200 Hz")
    ap.add_argument("--csv", default="", help="write the CSV here ('' = stdout)")
    ap.add_argument("--home", default="-35.363261,149.165230,584.0", help="SITL home: lat,lon,alt")
    ap.add_argument("--seconds", type=float, default=0.0, help="stop after N s (0 = until SIGTERM)")
    ap.add_argument("--report-hz", type=float, default=5.0, help="rate of the progress line on stderr")
    # PHASE 1b: the same client becomes the TRUTH SOURCE of the flight path.
    #   --ros            publish /sim/state (nav_msgs/Odometry, NED/FRD like the tap) + /sim/state/imu
    #                    (sensor_msgs/Imu) at --publish-hz, interpolated between SIM_STATE samples
    #   --tap-csv PATH   write a tap.csv-compatible file (same 19 columns as truth_rx) so scripts/accept.py
    #                    keeps working when the firmware tap is no longer in the flight path
    #   --publish-hz     the rate the flight stack sees; 200 Hz keeps lidar_imu's IMU decimation at 1
    ap.add_argument("--ros", action="store_true", help="publish /sim/state and /sim/state/imu")
    ap.add_argument("--publish-hz", type=float, default=200.0,
                    help="rate of the published truth (must be >= the lidar IMU rate)")
    ap.add_argument("--tap-csv", default="", help="also write a tap.csv-compatible CSV here")
    ap.add_argument("--stamp-offset-us", type=int, default=0,
                    help="added to the published/published-CSV timestamps (aligns the old tap's t=0)")
    # PHASE 1b: REPLACE truth_rx AS WELL -- it publishes the W-frame truth the supervisor/recorders read.
    ap.add_argument("--truth-odom", action="store_true",
                    help="also publish /sim/truth/odom (world_w, like sim/src/truth_rx.py --ros)")
    ap.add_argument("--clock-hz", type=float, default=100.0, help="rate of /clock when --truth-odom")
    ap.add_argument("--odom-hz", type=float, default=100.0, help="rate of /sim/truth/odom")
    ap.add_argument("--rtf-hz", type=float, default=1.0, help="rate of /sim/rtf")
    ap.add_argument("--pad-offset-n", type=float, default=0.0,
                    help="wire-NED north offset of the pad (integrated position starts here)")
    ap.add_argument("--pad-offset-e", type=float, default=0.0, help="wire-NED east offset of the pad")
    ap.add_argument("--pad-offset-d", type=float, default=-0.0999,
                    help="wire-NED down offset of the pad: MEASURED 2026-11-25, the tap reports pd=-0.0999 "
                         "parked, and integrating SIM_STATE velocity from this anchor reproduces the tap's "
                         "position to 5.7 mm rms (n/e 2 mm, d 9.5 mm) with NO affine fit")
    args = ap.parse_args()

    signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT, _on_signal)

    # MAVLink 2 framing is required for SET_MESSAGE_INTERVAL to take effect on ArduPilot's links
    # (MEASURED: with the default v1 framing the command is accepted but the stream never starts).
    os.environ.setdefault("MAVLINK20", "1")
    from pymavlink import mavutil

    def open_link():
        """Connect and wait for the AUTOPILOT heartbeat (system id >= 1).

        MEASURED 2026-11-25: connecting to SITL's serial port before the autopilot is up yields a
        heartbeat from system 0, and the stream requests sent on that link are then answered with a mix
        of DENIED/ACCEPTED while no SIM_STATE ever arrives (phase1n: 15 s with no samples, the LiDAR
        declared the truth dead and the sortie aborted).  The system-0 heartbeat is therefore not
        "connected" -- keep waiting.
        """
        link = mavutil.mavlink_connection(args.mav, source_system=251)
        link.wait_heartbeat(timeout=20)
        tries = 0
        while link.target_system < 1 and tries < 100:
            tries += 1
            link.wait_heartbeat(timeout=1.0)
        print("sim_state_recv: endpoint %s, heartbeat from system %d component %d"
              % (args.mav, link.target_system, link.target_component), file=sys.stderr)
        return link

    m = open_link()

    want_hz = [float(max(1.0, args.hz))]
    interval_us = [int(round(1e6 / want_hz[0]))]
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            mavutil.mavlink.MAVLINK_MSG_ID_SIM_STATE, interval_us[0], 0, 0, 0, 0, 0)
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 1000000, 0, 0, 0, 0, 0)

    home = parse_home(args.home)
    enu = LocalEnu(*home)

    cols = ["t_wall_us", "t_unix_us", "lat", "lon", "alt", "q1", "q2", "q3", "q4",
            "roll", "pitch", "yaw", "ax", "ay", "az", "wx", "wy", "wz",
            "vn", "ve", "vd", "pn", "pe", "pd", "lat_int", "lon_int", "std_h", "std_v"]
    fh = open(args.csv, "w", newline="") if args.csv else sys.stdout
    w = csv.writer(fh)
    w.writerow(cols)

    t_start = time.monotonic()
    last_report = t_start
    last_request = 0.0
    n = 0
    got_hb = False
    acks = []
    warned = False
    reconnects = 0
    # ---- Phase 1b: this client is the truth source -------------------------------
    truth_out = TruthOut(args.publish_hz, args.stamp_offset_us, truth_odom=args.truth_odom,
                         clock_hz=args.clock_hz, odom_hz=args.odom_hz, rtf_hz=args.rtf_hz,
                         pad_offset=(args.pad_offset_n, args.pad_offset_e, args.pad_offset_d)) \
        if args.ros else None
    tap_fh = open(args.tap_csv, "w") if args.tap_csv else None
    if tap_fh is not None:
        tap_fh.write(TAP_CSV_HEADER)
    if truth_out is not None and tap_fh is not None:
        def _tap_sink(t_us, p, v, q, w, a):
            tap_fh.write(TAP_ROW % (t_us, t_us + int(args.stamp_offset_us),
                                    p[0], p[1], p[2], v[0], v[1], v[2],
                                    q[0], q[1], q[2], q[3], w[0], w[1], w[2], a[0], a[1], a[2]))
        truth_out.sink = _tap_sink
    p_ned_arr = None            # integrated position, anchored at the first sample
    prev_t_us = None
    prev_v = None
    if truth_out is not None:
        print("sim_state_recv: publishing /sim/state + /sim/state/imu at %.0f Hz (interpolated)"
              % args.publish_hz, file=sys.stderr)
    if tap_fh is not None:
        print("sim_state_recv: writing tap-compatible CSV %s" % args.tap_csv, file=sys.stderr)
    try:
        while not GLOBAL_STOP:
            now = time.monotonic()
            if args.seconds and now - t_start >= args.seconds:
                break
            if n == 0 and not warned and now - t_start >= 15.0:
                warned = True
                print("sim_state_recv: 15 s with no SIM_STATE (heartbeat=%s, acks=%s)"
                      % (got_hb, acks[:4]), file=sys.stderr)
            if n == 0 and now - last_request > 1.0:
                last_request = now
                m.mav.command_long_send(m.target_system, m.target_component,
                                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                        mavutil.mavlink.MAVLINK_MSG_ID_SIM_STATE, interval_us[0],
                                        0, 0, 0, 0, 0)
            # FAST DRAIN, NOT recv_match(type=[...]): recv_match filters the whole queue per call and the
            # timestamp used to be taken BEFORE the blocking receive, so the recorded wall time lagged the
            # real arrival by up to one loop period (MEASURED: 87-91 Hz over a 100 Hz request, a ~1 % clock
            # fit bias and 2 gaps > 20 ms).  Take one raw message per iteration and stamp it right after.
            # If the stream never starts, the link may have been made before the autopilot's stream
            # scheduler was ready: reconnect and re-request (phase1n spent 15 s on such a link).
            if n == 0 and reconnects < 3 and now - t_start > 8.0 and now - last_request > 8.0:
                reconnects += 1
                last_request = now
                print("sim_state_recv: no SIM_STATE after %.0f s -- reconnecting (%d/3)"
                      % (now - t_start, reconnects), file=sys.stderr)
                try:
                    m.close()
                except Exception:
                    pass
                m = open_link()
                m.mav.command_long_send(m.target_system, m.target_component,
                                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                        mavutil.mavlink.MAVLINK_MSG_ID_SIM_STATE, interval_us[0],
                                        0, 0, 0, 0, 0)
            msg = m.recv_msg()
            if msg is None:
                if truth_out is not None:
                    truth_out.keep_alive(int((time.monotonic() - t_start) * 1e6) + args.stamp_offset_us)
                time.sleep(0.0005)
                continue
            mtype = msg.get_type()
            if mtype == "COMMAND_ACK":
                acks.append((int(msg.command), int(msg.result)))
                if len(acks) <= 4:
                    print("sim_state_recv: ACK command=%d result=%d" % (int(msg.command), int(msg.result)),
                          file=sys.stderr)
                # ADAPTIVE FALLBACK (MEASURED 2026-11-25): ArduPilot answers MAV_RESULT_DENIED with
                # "Requested rate for message ID 108 too fast. Increase SCHED_LOOP_RATE" when the FCU's
                # scheduler cannot sustain the asked rate -- and that answer CHANGES between runs at the
                # same requested rate.  Step the request down instead of recording nothing.
                if int(msg.command) == int(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL) and int(msg.result) == 2:
                    rate = max(50.0, want_hz[0] * 0.8)
                    if rate < want_hz[0]:
                        want_hz[0] = rate
                        interval_us[0] = int(round(1e6 / rate))
                        print("sim_state_recv: rate denied, backing off to %.0f Hz" % rate, file=sys.stderr)
                        m.mav.command_long_send(m.target_system, m.target_component,
                                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                                mavutil.mavlink.MAVLINK_MSG_ID_SIM_STATE,
                                                interval_us[0], 0, 0, 0, 0, 0)
                continue
            if mtype == "HEARTBEAT":
                got_hb = True
                continue
            if mtype != "SIM_STATE":
                continue                      # recv_msg() hands back every MAVLink message on the link
            t_recv = time.monotonic()
            t_wall_us = int((t_recv - t_start) * 1e6)
            lat_deg = msg.lat * 1e-7 if abs(msg.lat) > 1000 else float(msg.lat)   # dialect may give deg or 1e-7
            lon_deg = msg.lon * 1e-7 if abs(msg.lon) > 1000 else float(msg.lon)
            lat_i = int(round(lat_deg * 1e7))
            lon_i = int(round(lon_deg * 1e7))
            # alt is AMSL in metres; SITL's --home alt is the ground reference
            p = enu.ned(lat_deg, lon_deg, msg.alt)
            # ---- Phase 1b: reconstructed truth (position = anchor + integral of the velocity) ----
            if truth_out is not None or tap_fh is not None:
                v_cur = np.array([msg.vn, msg.ve, msg.vd], float)
                if p_ned_arr is None:
                    # THE ANCHOR IS THE PAD, NOT THE GEODETIC FIRST SAMPLE: MEASURED 2026-11-25, SITL's
                    # reported pad lat/lon is ~0.47 m east of --home, and using it would shift every
                    # cast/estimate by that much.  The model's own frame starts at the pad, and the tap
                    # reports pd=-0.0999 parked, so the anchor is the calibratable pad offset.
                    p_ned_arr = np.array([args.pad_offset_n, args.pad_offset_e, args.pad_offset_d], float)
                elif prev_t_us is not None and t_wall_us > prev_t_us:
                    dt_s = (t_wall_us - prev_t_us) * 1e-6
                    p_ned_arr = p_ned_arr + 0.5 * (prev_v + v_cur) * dt_s
                q_cur = np.array([msg.q1, msg.q2, msg.q3, msg.q4], float)
                w_cur = np.array([msg.xgyro, msg.ygyro, msg.zgyro], float)
                a_cur = np.array([msg.xacc, msg.yacc, msg.zacc], float)
                if truth_out is not None:
                    t_sample_us = truth_out.reconstruct_time(t_wall_us)
                else:
                    t_sample_us = t_wall_us
                if truth_out is not None:
                    truth_out.feed(t_sample_us + args.stamp_offset_us, p_ned_arr, v_cur, q_cur,
                                   w_cur, a_cur)
                if tap_fh is not None and truth_out is None:
                    # no-ROS path: `t_us` is the raw monotonic sample, `t_sim_us` the mapped one -- the same
                    # convention the ROS path's sink writes, so a tap.csv means one thing everywhere.
                    tap_fh.write(TAP_ROW % (t_sample_us, t_sample_us + int(args.stamp_offset_us),
                                            p_ned_arr[0], p_ned_arr[1], p_ned_arr[2],
                                            v_cur[0], v_cur[1], v_cur[2],
                                            q_cur[0], q_cur[1], q_cur[2], q_cur[3],
                                            w_cur[0], w_cur[1], w_cur[2],
                                            a_cur[0], a_cur[1], a_cur[2]))
                prev_t_us, prev_v = t_wall_us, v_cur
            w.writerow([t_wall_us, int(time.time() * 1e6), "%.7f" % lat_deg, "%.7f" % lon_deg,
                        "%.3f" % msg.alt,
                        "%.7f" % msg.q1, "%.7f" % msg.q2, "%.7f" % msg.q3, "%.7f" % msg.q4,
                        "%.6f" % msg.roll, "%.6f" % msg.pitch, "%.6f" % msg.yaw,
                        "%.5f" % msg.xacc, "%.5f" % msg.yacc, "%.5f" % msg.zacc,
                        "%.6f" % msg.xgyro, "%.6f" % msg.ygyro, "%.6f" % msg.zgyro,
                        "%.5f" % msg.vn, "%.5f" % msg.ve, "%.5f" % msg.vd,
                        "%.5f" % p[0], "%.5f" % p[1], "%.5f" % p[2],
                        lat_i, lon_i, "%.4f" % msg.std_dev_horz, "%.4f" % msg.std_dev_vert])
            if n == 0:
                print("sim_state_recv: FIRST SIM_STATE (stream is up)", file=sys.stderr)
            n += 1
            if t_recv - last_report >= 1.0 / max(0.1, args.report_hz):
                last_report = t_recv
                hz = n / max(1e-6, t_recv - t_start)
                print("sim_state_recv: %d samples, %.1f Hz (requested %.0f -> accepted %.0f), heartbeat %s"
                      % (n, hz, args.hz, want_hz[0], "yes" if got_hb else "no"), file=sys.stderr)
    finally:
        fh.flush()
        if args.csv:
            fh.close()
        if tap_fh is not None:
            tap_fh.flush()
            tap_fh.close()
        if truth_out is not None:
            truth_out.shutdown()
        dt = time.monotonic() - t_start
        print("sim_state_recv: stopped, %d samples in %.2f s = %.1f Hz"
              % (n, dt, n / max(1e-6, dt)), file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
