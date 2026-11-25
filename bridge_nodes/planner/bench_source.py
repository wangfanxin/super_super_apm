#!/usr/bin/env python3
"""bench_source.py -- the AIRCRAFT/TRACKING bench reference: smooth PVA sines, no obstacles, no planner.

Owner direction (2026-11-26): "首先有一个几乎完美的飞机设置，再考虑飞路径" and "固件相关估计跟踪性能相关
ff/pid 也得调整".  This plan box publishes a KNOWN, analytic position/velocity/acceleration reference so the
control chain (84 setpoints -> ArduPilot GUIDED -> airframe) can be measured on its own:

    phase 0  climb + hold at (0, 0, Z)                        (the plan's vertical leg)
    phase k  x(t) = A_k * sin(omega * (t - t_k))  for one --period per amplitude
             peak speed = A_k * omega, peak accel = A_k * omega^2
             (A = 1,2,3,4,5,6,8 m with omega = 1.0 rad/s -> 1..8 m/s and 1..8 m/s^2)
    final    return to (0, 0, Z), hold, trajectory_flag = COMPLETED -> the supervisor lands

Everything is published in L on /plan/cmd as PositionCommand at --cmd-rate, and the L->W/W->L conversion uses
the SAME calib file as the bridge.  The world is the open bench world (bench_low.yaml / bench_ring.yaml), so
nothing except the aircraft is in the loop.
"""
from __future__ import annotations

import argparse
import math
import os
import signal
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from common import frames          # noqa: E402
from planner import cmd_stream    # noqa: E402
import rclpy                       # noqa: E402
from rclpy.node import Node        # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # noqa: E402

try:
    from quadrotor_msgs.msg import PositionCommand
except ImportError:                # pragma: no cover
    from mars_quadrotor_msgs.msg import PositionCommand

FLAG_READY = 1
FLAG_COMPLETED = 3


def parse_amps(s):
    return [float(x) for x in str(s).replace(",", " ").split() if x.strip()]


def main() -> int:
    ap = argparse.ArgumentParser(description="PVA tracking bench reference (smooth sines)")
    ap.add_argument("--calib-file", default="")
    ap.add_argument("--z", type=float, default=2.0)
    ap.add_argument("--omega", type=float, default=1.0, help="sine angular frequency, rad/s")
    ap.add_argument("--amps", default="1,2,3,4,5,6,8", help="sine amplitudes (m); peak speed = A*omega")
    ap.add_argument("--periods", type=float, default=1.0, help="sine periods per amplitude")
    ap.add_argument("--climb-s", type=float, default=6.0)
    ap.add_argument("--hold-s", type=float, default=3.0)
    ap.add_argument("--return-s", type=float, default=4.0)
    ap.add_argument("--lead-s", type=float, default=0.0,
                    help="publish the reference this many seconds AHEAD (predictive lead): compensates the "
                         "measured 0.05 s pipeline/estimator lag.  0 = no lead (raw reference)")
    ap.add_argument("--cmd-rate", type=float, default=100.0)
    ap.add_argument("--cmd-topic", default="/plan/cmd")
    args = ap.parse_args()

    if args.calib_file and os.path.isfile(args.calib_file):
        import json as _json
        _d = _json.load(open(args.calib_file))
        calib_obj = frames.Calib(R_WL=np.array(_d["R_WL"], dtype=float),
                                 t_WL=np.array(_d["t_WL"], dtype=float))
        print("bench_source: L->W from %s" % args.calib_file, flush=True)
    else:
        print("bench_source: NO --calib-file -> identity L->W (bench only)", flush=True)
        calib_obj = frames.Calib(R_WL=np.eye(3), t_WL=np.zeros(3))
    amps = parse_amps(args.amps)
    # ONE continuous amplitude-modulated sine instead of hard amplitude switches: amplitude A(t) slides from
    # amp[k] to amp[k+1] with a smoothstep over ONE period, so the reference velocity has no jumps.  MEASURED
    # phase7f: switching A at a constant phase produced a 1-2 m/s velocity STEP at every segment boundary
    # (path_continuity |Δv| max 8.0 m/s), which the aircraft correctly refused to follow and the tracking
    # metric then blamed on the aircraft.
    T_sine = 2.0 * math.pi / args.omega
    sine_dur = len(amps) * T_sine
    t_sine0 = args.climb_s + args.hold_s
    t_ret0 = t_sine0 + sine_dur
    t_done0 = t_ret0 + args.return_s
    segs = [("climb", 0.0, args.climb_s, None),
            ("sine", t_sine0, sine_dur, None),
            ("return", t_ret0, args.return_s, None),
            ("done", t_done0, args.hold_s, None)]

    rclpy.init(args=[])
    node = Node("bench_source")
    # the bridge's /plan/cmd subscription is RELIABLE + TRANSIENT_LOCAL (bridge_node.Relay.attach); a plain
    # VOLATILE publisher is reported as an incompatible QoS and NOTHING reaches the wire (measured phase7d).
    from rclpy.qos import DurabilityPolicy
    qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(PositionCommand, args.cmd_topic, qos)
    t0 = time.monotonic()
    stop = {"flag": False}

    def on_term(_s, _f):  # noqa: ARG001
        stop["flag"] = True

    signal.signal(signal.SIGTERM, on_term)
    signal.signal(signal.SIGINT, on_term)

    def ref_at(tt):
        """(p_W, v_W, a_W) at plan time tt, or None when finished."""
        if tt < 0:
            p = np.array([0.0, 0.0, args.z])
            return p, np.zeros(3), np.zeros(3), FLAG_READY
        for kind, t_k, dur, A in segs:
            if not (t_k <= tt < t_k + dur):
                continue
            tau = tt - t_k
            if kind == "climb":
                # smooth 5th-order climb z: 0 -> z, then hold
                s = tau / max(1e-6, args.climb_s)
                s = min(1.0, max(0.0, s))
                z = args.z * (10 * s**3 - 15 * s**4 + 6 * s**5)
                zd = args.z * (30 * s**2 - 60 * s**3 + 30 * s**4) / args.climb_s
                zdd = args.z * (60 * s - 180 * s**2 + 120 * s**3) / args.climb_s**2
                return np.array([0.0, 0.0, z]), np.array([0.0, 0.0, zd]), np.array([0.0, 0.0, zdd]), FLAG_READY
            if kind == "sine":
                w = args.omega
                ph = w * tau
                k = min(int(tau // T_sine), len(amps) - 1)
                s = (tau - k * T_sine) / T_sine
                A0 = amps[k]
                A1 = amps[k + 1] if k + 1 < len(amps) else amps[k]
                sm = 10 * s**3 - 15 * s**4 + 6 * s**5           # smoothstep(0..1), ds/ds=0 at both ends
                A_t = A0 + (A1 - A0) * sm
                Adot = (A1 - A0) * (30 * s**2 - 60 * s**3 + 30 * s**4) / T_sine
                x = A_t * math.sin(ph)
                vx = Adot * math.sin(ph) + A_t * w * math.cos(ph)
                ax = Adot * w * math.cos(ph) - A_t * w * w * math.sin(ph)  # Adotdot term negligible
                return np.array([x, 0.0, args.z]), np.array([vx, 0.0, 0.0]), np.array([ax, 0.0, 0.0]), FLAG_READY
            if kind == "return":
                # smooth 5th-order move from A_last*sin(2*pi*periods...) back to 0: use the last x
                x_from = 0.0
                for kk, (k2, t2, d2, A2) in enumerate(segs):
                    if k2 == "sine":
                        x_from = 0.0
                s = min(1.0, max(0.0, tau / args.return_s))
                sm = 10 * s**3 - 15 * s**4 + 6 * s**5
                # move only in x, from 0 (sine ends at ~0 after whole periods) to 0: hold z
                x = 0.0
                zd = 0.0
                return np.array([x, 0.0, args.z]), np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]), FLAG_READY
            # done: hold
            return np.array([0.0, 0.0, args.z]), np.zeros(3), np.zeros(3), FLAG_COMPLETED
        return np.array([0.0, 0.0, args.z]), np.zeros(3), np.zeros(3), FLAG_COMPLETED

    print("bench_source: %.0f Hz, z %.2f, omega %.2f rad/s, amps %s, segments %d, lead %.3f s"
          % (args.cmd_rate, args.z, args.omega, amps, len(segs), args.lead_s), flush=True)
    for kind, t_k, dur, A in segs:
        print("   seg %-6s t=%6.2f..%6.2f %s" % (kind, t_k, t_k + dur,
                                                 "" if A is None else "A=%.2f (peak v %.2f, a %.2f)"
                                                 % (A, A * args.omega, A * args.omega ** 2)), flush=True)

    def tick():
        tt = time.monotonic() - t0
        p_w, v_w, a_w, flag = ref_at(tt + float(args.lead_s))
        # W -> L for the command stream (the bridge converts L -> W again with the same calib)
        p_l = calib_obj.R_WL.T @ (p_w - calib_obj.t_WL)
        v_l = calib_obj.R_WL.T @ v_w
        a_l = calib_obj.R_WL.T @ a_w
        m = PositionCommand()
        m.header.stamp = node.get_clock().now().to_msg()
        m.header.frame_id = cmd_stream.FRAME_L
        m.position.x, m.position.y, m.position.z = float(p_l[0]), float(p_l[1]), float(p_l[2])
        m.velocity.x, m.velocity.y, m.velocity.z = float(v_l[0]), float(v_l[1]), float(v_l[2])
        m.acceleration.x, m.acceleration.y, m.acceleration.z = float(a_l[0]), float(a_l[1]), float(a_l[2])
        m.kx = [0.0, 0.0, 0.0]      # PositionCommand (quadrotor_msgs) has no jerk field; kx/kv are unused
        m.kv = [0.0, 0.0, 0.0]
        m.yaw = 0.0
        m.yaw_dot = 0.0
        m.trajectory_id = 1
        m.trajectory_flag = flag
        pub.publish(m)

    period = 1.0 / args.cmd_rate
    while rclpy.ok() and not stop["flag"]:
        rclpy.spin_once(node, timeout_sec=0.0)
        tick()
        time.sleep(period)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
