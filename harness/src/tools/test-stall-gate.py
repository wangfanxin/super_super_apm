#!/usr/bin/env python3
"""The stall gate, tested WITHOUT a flight and without a ROS graph -- now the SUPERVISOR's.

WHY A TEST AND NOT A SORTIE: the gate decides whether a healthy mission gets aborted, and both directions of
that decision are cheap to check here and expensive to discover in the air.  `Supervisor._stall_reason` is
pure logic over attributes, so it can be called on a stub object.

CARRIED FROM `fsm/mission_fsm.py` WHEN F2 RETIRED IT, and the reason it was carried is design section 8
item 8: it is the only NAMED cause for "the trajectory source is dead" that ANY source can raise by itself,
and the gate that replaced it for our own plan box (`--plan-silence-s`, which turns silence into a NAMED
abort because the end of the tour is now a FIELD) is new.  Two named causes, both of them tested.

The three cases are exactly the three states a sortie can be in:
  A. the planner says "stalled" but plans are still arriving (it is waiting for the aircraft) -> NO abort;
  B. the planner says "stalled" and nothing has arrived for --stall-abort-s                    -> ABORT, named;
  C. the signal cleared (a new plan was published)                                             -> NO abort.

usage: python3 tools/test-stall-gate.py
"""
from __future__ import annotations

import argparse
import importlib.util
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)


def load_supervisor():
    """Import the class without starting anything (rclpy is imported inside start_ros)."""
    spec = importlib.util.spec_from_file_location(
        "supervisor_under_test", os.path.join(ROOT, "supervisor", "supervisor.py"))
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def main() -> int:
    mod = load_supervisor()
    ok = True

    def check(name, good, detail=""):
        nonlocal ok
        ok = ok and bool(good)
        print("%-72s %s %s" % (name, "ok" if good else "FAIL", detail))

    def stub(stalled, up_s, quiet_s):
        s = object.__new__(mod.Supervisor)
        s.args = argparse.Namespace(stall_abort_s=3.0, stall_topic="/planner/stalled",
                                    cmd_topic="/plan/cmd")
        s.planner_stalled = stalled
        now = time.monotonic()
        s.planner_stall_since = None if not stalled else now - up_s
        s.plan = {"last_wall": now - quiet_s, "n": 5, "last_tid": 2, "ids": [1, 2], "flag": 1}
        return s

    r = mod.Supervisor._stall_reason(stub(True, 10.0, 0.2))
    check("A. stalled signal up 10 s but a command arrived 0.2 s ago -> NO abort", r is None, str(r)[:40])

    r = mod.Supervisor._stall_reason(stub(True, 10.0, 10.0))
    check("B. stalled signal up 10 s and nothing for 10 s -> ABORT, and it NAMES the planner",
          r is not None and "THE PLANNER IS NOT PLANNING" in r, (r or "")[:70])

    r = mod.Supervisor._stall_reason(stub(True, 1.0, 1.0))
    check("C. stalled, but for less than --stall-abort-s -> NO abort (a 2 s hiccup is not a death)",
          r is None, str(r)[:40])

    r = mod.Supervisor._stall_reason(stub(False, 0.0, 30.0))
    check("D. signal down (a new plan was published) -> NO abort even after a long silence", r is None,
          str(r)[:40])

    # ---- THE GATE F2 ADDED: silence from a source whose END-OF-TOUR IS A FIELD -------------------------
    print()
    print("   and the plan-box liveness gate F2 added (a dead plan box is no longer a finished tour):")

    def live(quiet_s, relay=1, n=5, nav_age=0.02, status_age=0.0, alt=1.0):
        s = object.__new__(mod.Supervisor)
        s.args = argparse.Namespace(geofence_m=6.60, nav_floor_hz=30.0, plan_silence_s=3.0,
                                    cmd_topic="/plan/cmd",
                                    # the fields the 2026-11-07 gates added (audit findings 3, 10, 12).
                                    # `nav_age_max_s` is pinned to 0.5 HERE so the arithmetic in the
                                    # cases below is unambiguous; the SHIPPED default is 1.5 s (a 0.54 s
                                    # stall was flown through and must not abort).
                                    alt=2.0, climb_guard_m=4.0, status_stale_s=2.0,
                                    nav_age_max_s=0.5, nav_age_warn_s=0.25,
                                    status_topic="/bridge/status")
        s.tap = type("T", (), {"n": 1, "p": [0.0, 0.0, alt], "last_jump_wall": None, "n_jumps": 0})()
        s.bridge = type("B", (), {"state": staticmethod(
            lambda: type("S", (), {"armed": True, "mode": "GUIDED", "messages": []})())})()
        s.plan = {"n": n, "last_wall": time.monotonic() - quiet_s, "last_tid": 2, "ids": [1, 2], "flag": 1}
        s.bridge_status = {"nav_hz": 48.0, "nav_age": nav_age, "relay": relay, "sent": 10,
                           "last_wall": time.monotonic() - status_age, "mode": "GUIDED", "armed": 1}
        s.route_started = True
        s.planner_stalled = False
        s.planner_stall_since = None
        s.abort_reason = None
        s.gates_armed = True
        return s

    s = live(0.2)
    mod.Supervisor.check_gates(s)
    check("E. the plan box spoke 0.2 s ago -> NO abort", s.abort_reason is None, str(s.abort_reason)[:60])

    s = live(5.0)
    mod.Supervisor.check_gates(s)
    check("F. the plan box has been SILENT for 5 s (> the 3 s limit) -> ABORT, and it says why",
          s.abort_reason is not None and "STOPPED TALKING" in s.abort_reason
          and "SILENCE IS NOT COMPLETION" in s.abort_reason, (s.abort_reason or "")[:80])

    s = live(5.0, relay=0)
    mod.Supervisor.check_gates(s)
    check("G. ... and it aborts even if the relay never enabled: after the ROUTE has started, a silent plan "
          "box is an abort whatever the bridge reports", s.abort_reason is not None
          and "STOPPED TALKING" in s.abort_reason, (s.abort_reason or "")[:60])

    s = live(5.0)
    s.route_started = False
    mod.Supervisor.check_gates(s)
    check("G2. BEFORE the route starts, silence is not an abort (the plan box is up and waiting: the whole "
          "point of the trigger)", s.abort_reason is None, str(s.abort_reason)[:60])

    s = live(0.2)
    s.bridge_status["nav_hz"] = 3.0
    mod.Supervisor.check_gates(s)
    check("H. the bridge reports the estimator feed at 3 Hz -> ABORT naming the feed",
          s.abort_reason is not None and "ESTIMATOR FEED HAS STOPPED" in s.abort_reason,
          (s.abort_reason or "")[:70])

    # ---- THE THREE GATES THE 2026-11-07 AUDIT ADDED (findings 3, 10, 12) ----------------------------
    print()
    print("   the gates the 2026-11-07 audit added:")

    # finding 3: the RATE cannot see a dead LIO, because the bridge republishes the last sample at
    # --nav-hz.  `nav_age` (the age of the sample being fed to the EKF) is the number that moves.
    s = live(0.2, nav_age=1.2)
    mod.Supervisor.check_gates(s)
    check("I. nav_hz is fine but the LIO sample being fed is 1.2 s old -> ABORT naming the STALE feed",
          s.abort_reason is not None and "ESTIMATOR FEED IS STALE" in s.abort_reason,
          (s.abort_reason or "")[:70])

    # finding 12: a dead bridge freezes every number it last sent, including nav_hz=48.
    s = live(0.2, status_age=5.0)
    mod.Supervisor.check_gates(s)
    check("J. no /bridge/status for 5 s -> ABORT naming the dead BRIDGE (a frozen echo is not health)",
          s.abort_reason is not None and "BRIDGE HAS STOPPED REPORTING" in s.abort_reason,
          (s.abort_reason or "")[:70])

    # finding 10: the climb guard used to live in hold() only, so nothing bounded the altitude in flight.
    s = live(0.2, alt=7.0)                     # 2.00 m mission + 4.00 m guard = 6.00 m
    mod.Supervisor.check_gates(s)
    check("K. the truth climbs to 7.0 m on a 2.00 m mission -> ABORT naming the ALTITUDE GUARD",
          s.abort_reason is not None and "ALTITUDE GUARD" in s.abort_reason,
          (s.abort_reason or "")[:70])

    s = live(0.2, alt=5.9)                     # just under the 6.00 m guard
    mod.Supervisor.check_gates(s)
    check("K2. ... and 5.9 m is inside the guard, so no abort (the guard is not a hair trigger)",
          s.abort_reason is None, (s.abort_reason or "")[:70])

    # ---- THE ROUND-89 ESTIMATOR GATES ---------------------------------------------------------------
    # INTEGRATION §84-88: on 30 % of the sorties the LIO's own WORLD POSE ran away (position to 117 m,
    # yaw steps up to 154 deg).  The FCU follows it, so two gates were added: the bridge counts the frames
    # it refuses as physically impossible (`nav_skip`), and the supervisor aborts on the RATE of those
    # refusals, plus on |FCU estimate - truth| holding above 2 m.  Neither had a test until now.
    print()
    print("   the round-89 estimator gates (the LIO's own pose ran away on 30 % of the sorties):")

    def est(nav_skip_hist=None, fcu_gap=0.0, div_since=None):
        s = live(0.2)
        s.args.fcu_odom_topic = "/fcu/odom"
        s.args.fcu_fresh_s = 1.0
        s.args.truth_fcu_max_m = 2.0
        s.args.truth_fcu_hold_s = 2.0
        s.args.nav_skip_max_per_s = 10.0
        s.bridge_status["nav_skip"] = (nav_skip_hist[-1][1] if nav_skip_hist else 0)
        s.nav_skip_hist = nav_skip_hist or []
        now = time.monotonic()
        s.fcu = {"p": [fcu_gap, 0.0, 1.0], "wall": now, "n": 5}
        s.fcu_div_since = div_since
        return s

    now = time.monotonic()
    s = est(nav_skip_hist=[(now - 2.0, 0), (now - 1.0, 50), (now, 100)])
    mod.Supervisor.check_gates(s)
    check("L. the bridge refused 100 external-nav frames in 2 s (50/s) -> ABORT naming the DIVERGING estimator",
          s.abort_reason is not None and "ESTIMATOR IS DIVERGING" in s.abort_reason, (s.abort_reason or "")[:70])

    s = est(nav_skip_hist=[(now - 2.0, 0), (now - 1.0, 1), (now, 1)])
    mod.Supervisor.check_gates(s)
    check("L2. ... and ONE refused frame in the window is not an abort (a take-off glitch is not a divergence)",
          s.abort_reason is None, str(s.abort_reason)[:60])

    s = est(fcu_gap=3.0, div_since=time.monotonic() - 3.0)
    mod.Supervisor.check_gates(s)
    check("M. the FCU's own estimate is 3 m from the truth and has been for 3 s -> ABORT naming the lost aircraft",
          s.abort_reason is not None and "ESTIMATOR HAS LOST THE AIRCRAFT" in s.abort_reason,
          (s.abort_reason or "")[:70])

    s = est(fcu_gap=3.0, div_since=time.monotonic() - 0.2)
    mod.Supervisor.check_gates(s)
    check("M2. ... and the same gap for 0.2 s only -> NO abort (the gate waits --truth-fcu-hold-s)",
          s.abort_reason is None, str(s.abort_reason)[:60])

    s = est(fcu_gap=1.5, div_since=None)
    mod.Supervisor.check_gates(s)
    check("M3. ... and a 1.5 m gap is ordinary tracking, inside the 2 m limit -> NO abort",
          s.abort_reason is None, str(s.abort_reason)[:60])

    # ---- THE RULE THE KILL SORTIES ADDED: THE GATES ARE FOR A MISSION IN PROGRESS --------------------
    # MEASURED (`RECT27-kill-plan` and `RECT27b-kill-plan`, 2026-10-15): the abort's own landing pumps
    # through `check_gates`, every gate that fired is STILL TRUE while the aircraft comes down, and the
    # re-fired reason makes `pump` -- whose exit condition it is -- return in one iteration.  Both sorties
    # exited 0.05 s after the LAND with the aircraft armed at 2.0 m.
    print()
    print("   and WHILE THE SORTIE IS ENDING the gates must NOT re-fire (the landing has to be waited for):")

    s = live(5.0)                      # a condition that DID fire, as it is during the abort's landing
    s.gates_armed = False
    mod.Supervisor.check_gates(s)
    check("I. a silent plan box while the sortie is ENDING -> NO new abort (the landing proceeds)",
          s.abort_reason is None, str(s.abort_reason)[:60])

    s = live(5.0)                      # the negative control: the SAME condition with the gates live
    mod.Supervisor.check_gates(s)
    check("I'. ... and the same condition with the gates live DOES abort (the case above is not vacuous)",
          s.abort_reason is not None and "STOPPED TALKING" in s.abort_reason,
          (s.abort_reason or "")[:60])

    print()
    print("stall-gate test:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
