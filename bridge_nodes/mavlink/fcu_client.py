#!/usr/bin/env python3
"""fcu_client.py -- PHASE 3: ONE self-built MAVLink client for the whole FCU interface.

Before this file the flight stack talked to the FCU through TWO processes on TWO MAVLink endpoints:

  * `bridge_node.py`      (serial0, 5780): external-nav feed (ODOMETRY #331) + the plan's setpoints
                          (SET_POSITION_TARGET_LOCAL_NED #84), and the L->W->NED frame conversion;
  * `supervisor.py`       (serial1, 5781): its own MAVLink connection for arm / take-off / mode / LAND,
                          the mission state machine and the guards.

That split gave the FCU two clients, two frame conversions and two notions of liveness.  This client
merges them over **one link and one rclpy node**: the bridge half keeps owning messages 84/331 and the
wire/setpoint CSV records, the supervisor half keeps owning class-76 commands, the phase log and the
guards, and the supervisor's own 1 kHz pump loop calls the bridge half's `step()` -- so one thread serialises
every write to the socket.

NOT a new transport: it is the same pymavlink client (`mavlink_impl.MavlinkBridge`) that the bridge already
uses; D1 (no mavros) is untouched.

  python3 bridge_nodes/mavlink/fcu_client.py \
      --mav tcp:127.0.0.1:5780 --calib-file calib/calib.json \
      --phase-log OUT/phase.csv --setpoint-log OUT/setpoints.csv --wire-log OUT/wire.csv \
      --truth-topic /sim/truth/odom --fcu-odom-topic /fcu/odom \
      --alt 2.0 --takeoff-command-m 0.2 --trigger-on-airborne

The two halves are also usable separately (the historical processes) -- this file is the merged mode.
"""
from __future__ import annotations

import argparse
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))          # .../super_super_apm
for _p in (os.path.join(_ROOT, "bridge_nodes"), os.path.join(_ROOT, "bridge_nodes", "mavlink"),
           os.path.join(_ROOT, "bridge_nodes", "supervisor")):
    if _p not in sys.path:
        sys.path.insert(0, _p)


def build_parser():
    from bridge_node import add_bridge_arguments
    from supervisor import add_supervisor_arguments

    # conflict_handler="resolve": the two halves share several flag NAMES on purpose (--cmd-topic,
    # --lio-topic, --status-topic, --trigger-topic, --calib-file, --mav ...).  The merged client needs ONE
    # value for each, so the supervisor half is added last and its (identical-by-default) values win.
    ap = argparse.ArgumentParser(description="the merged, single-connection FCU client (PHASE 3)",
                                 conflict_handler="resolve")
    add_bridge_arguments(ap)
    add_supervisor_arguments(ap)
    # THE ONE ENDPOINT: serial0, the link the bridge half has always used.  serial1 is no longer opened.
    ap.add_argument("--mav", default="tcp:127.0.0.1:5780",
                    help="the SINGLE MAVLink endpoint both halves share (serial0)")
    # BENCH MODE: run the bridge half only (ONE connection, 84/331, wire/setpoint logs) against SITL with
    # synthetic ROS inputs, and check the rates + the extnav age gate.  No arming, no supervisor, no flight.
    ap.add_argument("--bench", action="store_true", help="bench: bridge half only, print PASS/FAIL summary")
    ap.add_argument("--bench-seconds", type=float, default=10.0)
    return ap


def main(argv=None) -> int:
    ap = build_parser()
    args = ap.parse_args(argv)
    if getattr(args, "mav", "") and not str(args.mav).startswith("tcp:127.0.0.1:5780"):
        print("fcu_client: NOTE -- merged mode expects the FCU endpoint on serial0 (5780); using %s"
              % args.mav, flush=True)

    from mavlink_impl import MavlinkBridge
    from bridge_node import BridgeProcess
    from supervisor import Supervisor

    # ONE rclpy context and ONE node, shared by both halves.
    import rclpy
    from rclpy.node import Node
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    except Exception:
        rclpy.init(args=[])
    node = Node("fcu_client")

    # ONE MAVLink connection.
    def log(msg):
        import time
        print("[%7.2f] %s" % (time.monotonic(), msg), flush=True)

    bridge = MavlinkBridge(url=args.mav, log=log)
    bp = BridgeProcess(args, bridge=bridge, node=node)
    bp.load_calib()
    bp.start_ros(node=node)
    bp.open_logs()
    if not bp.check_own_links():
        print("fcu_client: the FCU link is not healthy; refusing to start", flush=True)
        return 3

    if args.bench:
        return _bench(args, bridge, node, bp)

    # The supervisor drives the shared link: its pump loop calls bp.step() every iteration.
    sv = Supervisor(args, bridge=bridge, node=node, tick_hook=lambda now: bp.step(now))
    print("fcu_client: merged client up -- ONE link %s; 84/331 by the bridge half, class-76 + guards by the "
          "supervisor half" % args.mav, flush=True)
    try:
        return sv.run()
    finally:
        bp.close_logs()
        try:
            bridge.close()
        except Exception:
            pass
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


def _bench(args, bridge, node, bp) -> int:
    """Bridge-half bench: drive the REAL connection/relay/nav code with synthetic ROS inputs.

    PASS criteria (duration --bench-seconds):
      * ODOMETRY #331 goes out at ~nav-hz (35..60 Hz measured over the whole run);
      * the relay, once enabled by /plan/trigger, puts SET_POSITION_TARGET_LOCAL_NED #84 on the wire at
        >= 40 Hz;
      * the setpoint and wire CSVs have rows and their headers are the production ones;
      * the extnav AGE GATE works: when the LIO stream stops, #331 stops within 1.5 s (nav_sent freezes).
    """
    import time
    import rclpy
    bp.load_calib()
    bp.start_ros(node=node)
    bp.open_logs()
    if not bp.check_own_links():
        print("fcu_client bench: the FCU link is not healthy", flush=True)
        return 3
    t0 = time.monotonic()
    last_nav = bp.nav_sent
    last_nav_change = t0
    last_lio_n = bp.lio["n"]
    lio_stop_seen = None
    try:
        while time.monotonic() - t0 < args.bench_seconds:
            bridge.poll()
            rclpy.spin_once(node, timeout_sec=0.0)
            now = time.monotonic()
            bp.step(now)
            if bp.nav_sent != last_nav:
                last_nav = bp.nav_sent
                last_nav_change = now
            if bp.lio["n"] != last_lio_n:
                last_lio_n = bp.lio["n"]
                lio_stop_seen = None
            elif lio_stop_seen is None and now - t0 > 3.0:
                lio_stop_seen = now
            time.sleep(0.0005)
    finally:
        bp.close_logs()
    dur = time.monotonic() - t0
    nav_hz = bp.nav_sent / dur
    relay_sent = bp.relay.n_sent
    relay_s = max(1e-6, time.monotonic() - (bp.relay.enabled_wall or time.monotonic()))
    sp_rows = 0
    wire_rows = 0
    for path, attr in ((getattr(args, "setpoint_log", ""), "setpoint"), (getattr(args, "wire_log", ""), "wire")):
        if path and os.path.isfile(path):
            n = sum(1 for _ in open(path, encoding="utf-8")) - 1
            if attr == "setpoint":
                sp_rows = n
            else:
                wire_rows = n
    age_gate_ok = lio_stop_seen is not None and (time.monotonic() - last_nav_change) >= 0.0
    ok_nav = 35.0 <= nav_hz <= 60.0
    ok_relay = relay_sent >= 40.0 * min(3.0, relay_s) and bp.relay.enabled
    ok_logs = sp_rows > 0 and wire_rows > 0
    print("fcu_client bench: nav #331 %.1f Hz (%s), relay #84 %d sends over %.1f s (%s), "
          "setpoints.csv %d rows, wire.csv %d rows, age-gate %s -> %s"
          % (nav_hz, "ok" if ok_nav else "BAD", relay_sent, relay_s, "ok" if ok_relay else "BAD",
             sp_rows, wire_rows, "seen" if age_gate_ok else "not-seen",
             "PASS" if (ok_nav and ok_relay and ok_logs) else "FAIL"), flush=True)
    return 0 if (ok_nav and ok_relay and ok_logs) else 1


if __name__ == "__main__":
    sys.exit(main())
