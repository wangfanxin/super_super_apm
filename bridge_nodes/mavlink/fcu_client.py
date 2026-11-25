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
    # PHASE 3 CLOSEOUT (round 20): `merged` (default) and `bench` are the only modes left.  The Phase-3
    # bisect modes that lived here (bridge_only / bridge_thread / supervisor_only / legacy-shape) did their job
    # -- they proved that the bridge half has to keep its own thread and that the supervisor must own its own
    # node -- and have been REMOVED so there is exactly one control implementation in the flight path.
    # `FCU_CLIENT=legacy` in scripts/run-mission.sh is the rollback (the historical TWO processes).
    ap.add_argument("--mav", default="tcp:127.0.0.1:5780",
                    help="the bridge half's MAVLink endpoint (serial0: 84/331)")
    ap.add_argument("--mav-supervisor", default="tcp:127.0.0.1:5781",
                    help="the supervisor half's MAVLink endpoint (serial1: class-76 commands)")
    ap.add_argument("--bench", action="store_true",
                    help="bench: bridge half only against SITL with synthetic ROS inputs; prints PASS/FAIL")
    ap.add_argument("--bench-seconds", type=float, default=10.0)
    return ap


def main(argv=None) -> int:
    ap = build_parser()
    args = ap.parse_args(argv)
    args.client_mode = "bench" if getattr(args, "bench", False) else "merged"
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

    def log(msg):
        import time
        print("[%7.2f] %s" % (time.monotonic(), msg), flush=True)

    # TWO MAVLink connections, one per half (the layout the Phase-3 bisect proved necessary): serial0 carries
    # 84/331, serial1 carries the class-76 commands.  In `--bench` only serial0 is opened.
    bridge = MavlinkBridge(url=args.mav, log=log)
    # ★ TWO NODES, TWO CONNECTIONS, TWO THREADS — THE SHAPE THAT WORKS (PHASE 3, 2026-11-25) ★
    #   The Phase-3 bisect established that the bridge half must keep its own cadence (its own thread) and
    #   that it must NOT share an rclpy node with the supervisor: any shape with a shared node diverged in the
    #   EKF (XKF4.SH 1-3 m) right after take-off.  The merged client therefore keeps each half's legacy serial
    #   structure: the bridge thread owns serial0 AND spins the bridge's node; the supervisor owns serial1 AND
    #   spins its own node.  One program, one build, one place for the FCU interface -- two transports.
    merged = args.client_mode == "merged"
    bridge_sup = bridge
    node_sup = node
    if merged:
        bridge_sup = MavlinkBridge(url=args.mav_supervisor, log=log)
        node_sup = Node("fcu_client_sup")
        print("fcu_client: MERGED -- 84/331 on %s (bridge thread + its node), class-76 on %s (supervisor + its "
              "own node)" % (args.mav, args.mav_supervisor), flush=True)
    bp = BridgeProcess(args, bridge=bridge, node=node)
    bp.load_calib()
    bp.start_ros(node=node)
    bp.open_logs()
    if not bp.check_own_links():
        print("fcu_client: the FCU link is not healthy; refusing to start", flush=True)
        return 3

    if args.bench:
        return _bench(args, bridge, node, bp)

    import threading as _threading
    import time as _time
    from rclpy.executors import SingleThreadedExecutor as _Exec
    _run = {"stop": False, "err": None}
    _node_for_bridge = node
    # ONE EXECUTOR PER THREAD: `rclpy.spin_once()` is process-global and not reentrant (phase4e crashed with
    # "generator already executing" when both threads used it).
    _ex_bridge = _Exec()
    _ex_sup = _Exec()
    _ex_bridge.add_node(node)
    _ex_sup.add_node(node_sup)

    def _bridge_loop():
        while not _run["stop"]:
            try:
                bridge.poll()                           # this thread owns serial0's reads
                _ex_bridge.spin_once(timeout_sec=0.0)
                bp.step(_time.monotonic())
            except Exception as exc:                     # noqa: BLE001
                if _run["err"] is None:
                    _run["err"] = repr(exc)
                    print("fcu_client: bridge thread EXCEPTION %r" % (exc,), flush=True)
                _time.sleep(0.01)
                continue
            _time.sleep(0.0005)

    _bt = _threading.Thread(target=_bridge_loop, name="bridge-half", daemon=True)
    _bt.start()

    # SUPERVISOR half: its own node and its own connection
    sv = Supervisor(args, bridge=bridge_sup, node=node_sup, executor=_ex_sup)
    print("fcu_client: merged client up -- bridge thread owns 84/331 on %s; supervisor owns class-76 + "
          "guards on %s" % (args.mav, args.mav_supervisor), flush=True)
    try:
        return sv.run()
    finally:
        _run["stop"] = True
        _bt.join(timeout=2.0)
        bp.close_logs()
        try:
            bridge.close()
        except Exception:
            pass
        if bridge_sup is not bridge:
            try:
                bridge_sup.close()
            except Exception:
                pass
        try:
            node_sup.destroy_node()
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
