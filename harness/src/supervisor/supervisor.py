#!/usr/bin/env python3
"""THE SUPERVISOR: mission A's authority, as an INDEPENDENT PROCESS with its OWN MAVLink endpoint.

WHAT IT IS, AND WHY IT IS NOT PART OF THE PLAN BOX.  `_stage/refactor-bridge-plan.md` section 4: the PLAN
box is the box a whole-replacement swaps out (amendment A1), so anything left inside it disappears with the
swap.  Preflight, arming, the take-off, the modes, LAND and the ABORT are none of them functions of a plan
-- checked against the code, none of them reads a plan quantity -- so they live here, in the one process
whose inputs are the TRUTH and the LINK and never the LIO in flight.

ITS INPUTS, exactly:
  * `/sim/truth/odom`  (W) -- the truth-fed geofence and the airborne test.  SIM-ONLY, and that is the
    point: it is the one box allowed to read it, and it reads it as a GATE, never as a control input;
  * `/plan/cmd`        -- the plan box's own statements: is it still talking, and has it declared COMPLETED.
    This is not the LIO; it is the plan, and the design names this topic as one of the supervisor's inputs;
  * `/bridge/status`   -- the BRIDGE's own report of the streams it is carrying (its LIO input rate, the
    estimator feed it has sent, the relay's state);
  * its OWN endpoint   -- heartbeat/position age, armed, mode, landed_state, statustext.
  It does NOT subscribe to `/lio/robo/odom` or `/lio/odom`: in flight, the LIO is the plan's and the
  bridge's business.  (Before the aircraft is armed it does read the LIO, in the preflight liveness gate --
  the design's own qualification of the rule, section 4.3.)

ITS AUTHORITY, by MESSAGE CLASS and by nothing else (section 4.2): class 76 -- arm, take-off, mode, LAND,
RTL, the EKF origin and the accelerometer calibration.  It NEVER sends a setpoint: message 84 belongs to
`bridge/bridge_node.py`, and a supervisor that streamed setpoints would be a second, silent plan authority.
The split needs no runtime token because the two writers cannot name the same message; that rule is also
free to a dropped-in plan box, because ego's own source never writes to a flight controller at all.

WHAT IT FIXES.  MEASURED, and it is the reason the split is worth its risk: until F2 a DEAD PLAN BOX and a
FINISHED TOUR were the same fact to the consumer -- an empty path meant "no more legs", a timeout meant the
same thing, the mission landed and `tools/metrics.py` scored it COMPLETE with no `abort` phase (design
section 2(e), risk R3).  The tour's end is a FIELD now (`trajectory_flag = COMPLETED`) and the plan box
keeps publishing; so SILENCE is an error, and this process turns it into a NAMED abort.

usage: supervisor/supervisor.py --mav tcp:127.0.0.1:5761 --calib-file _stage/calib.json
       --phase-log logs/<run>/phase.csv [--alt 2.5] [--hold 3.0] [--geofence-m 6.60]
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from common import frames                    # noqa: E402
from bridge.base import Mode                 # noqa: E402
from planner.cmd_stream import TRAJECTORY_STATUS_COMPLETED   # noqa: E402

HOME_LAT, HOME_LON, HOME_ALT = -35.363261, 149.165230, 584.0
GATES = {
    "lio_rate_min_hz": 5.0,
    "rate_hz": 50.0,
}


class TapTruth:
    """The truth, as the supervisor needs it: the geofence and the airborne test.  Nothing else.

    PLUS ONE THING THE GEOFENCE CANNOT DO WITHOUT: whether this stream just TELEPORTED.

    WHY (round 85, measured): the tap is emitted from SITL's model state in NED "with the simulator origin,
    i.e. the EKF origin" (`src/patches/apm-0001-sitl-truth-tap.patch`), so when the EKF re-seats its origin
    or goes failsafe the tap's numbers move by hundreds of metres while the vehicle does not.  MEASURED on
    this harness: `pva2-02-02` tap vertical span 410.6 m against the FCU's own 3.6 m, `pva2-01-01` 591 m
    horizontal against 9.8 m, `az0-01-01` 80.9 m of altitude on a 2.00 m mission.  Four `ABORT: geofence`
    in round 85, at least two of them provably false alarms.  A position that jumps at an implied speed no
    airframe can produce is a COORDINATE EVENT, and the supervisor must not abort a mission on it.
    """

    #: an implied speed above this cannot be the aircraft (the enforcer's own limit is 6-10 m/s)
    JUMP_MPS = 30.0

    def __init__(self):
        self.t_us = 0
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.n = 0
        #: teleport bookkeeping (see the class docstring)
        self.p_prev = None
        self.t_us_prev = 0
        self.n_jumps = 0
        self.last_jump_wall = None

    def on_odom(self, m):
        self.t_us = int(m.header.stamp.sec) * 1_000_000 + int(m.header.stamp.nanosec) // 1000
        self.p = np.array([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z])
        self.v = np.array([m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z])
        self.q = np.array([m.pose.pose.orientation.w, m.pose.pose.orientation.x,
                           m.pose.pose.orientation.y, m.pose.pose.orientation.z])
        if self.p_prev is not None:
            dt = (self.t_us - self.t_us_prev) * 1e-6
            if dt > 0:
                implied = float(np.linalg.norm(self.p - self.p_prev)) / dt
                if implied > self.JUMP_MPS:
                    self.n_jumps += 1
                    self.last_jump_wall = time.monotonic()
        self.p_prev = self.p.copy()
        self.t_us_prev = self.t_us
        self.n += 1


def geofence_verdict(tap, geofence_m, now, grace_s, disagree_m=2.0, ignore_m=50.0):
    """(breach, note) for the geofence gate -- with the teleport guard folded in.

    `note` is "" when the breach is real and nothing more needs saying, and a short explanation otherwise
    (why a breach was suppressed, or what corroborated it).  Pure arithmetic on the tap object plus
    `tap.fcu_p`, so `tools/test-teleport-guard.py` can drive it with no ROS and no MAVLink.

    ★ WHAT THE SECOND OPINION MAY AND MAY NOT DO (2026-11-07 audit, finding 4) ★
    The FCU's own estimate was allowed to SUPPRESS the fence on a disagreement of more than
    `disagree_m` (2 m), and that is indistinguishable from the failure it must catch: MEASURED on
    `kp1-01-01`, four suppressions of 3.8-5.8 m in the 8 s before the verdict, whose own phase row records
    the truth at (16.07, 7.92) -- outside a 13 m fence -- followed by `SIM Hit ground at 10.55 m/s`.
    A false abort costs a sortie; a suppressed fence costs the aircraft, so the rules are now:

      1. the tap is inside                     -> no breach;
      2. the tap JUMPED within `grace_s`       -> suppressed (a coordinate event: the numbers moved, the
                                                  aircraft did not) -- unless the FCU independently puts it
                                                  outside, in which case it is out either way;
      3. the FCU is fresh and puts it outside  -> breach (both sources agree);
      4. the two disagree by `ignore_m` or more AND the FCU has it inside -> suppressed: a frame change of
         that size (MEASURED: 338 m, 410 m, 591 m) is not a flight, and this is the persistent case the
         grace window cannot cover;
      5. any OTHER disagreement (a few metres)  -> BREACH, with the disagreement named in the note.  That
         is the kp1 shape: a diverging estimate, not a frame.

    `ignore_m` is deliberately far above any tracking or estimator error this stack has ever measured
    (the worst EKF-vs-truth divergence in `logs/` is 18.8 m) and far below the frame-change distances.
    """
    if tap.n <= 0:
        return False, ""
    inside = abs(tap.p[0]) <= geofence_m and abs(tap.p[1]) <= geofence_m
    if inside:
        return False, ""
    fcu = getattr(tap, "fcu_p", None)
    disagree = None
    fcu_outside = None
    if fcu is not None:
        disagree = float(np.linalg.norm(np.asarray(tap.p[:2]) - np.asarray(fcu[:2])))
        fcu_outside = bool(abs(fcu[0]) > geofence_m or abs(fcu[1]) > geofence_m)
    # (2) a coordinate EVENT: the whole stream moved by a distance no airframe could fly in one tick
    if tap.last_jump_wall is not None and (now - tap.last_jump_wall) <= grace_s:
        if fcu_outside:
            return True, ("the truth stream jumped %.1f s ago, but the FCU also puts the vehicle outside "
                          "the fence (%.1f m away)" % (now - tap.last_jump_wall, disagree))
        return False, ("geofence suppressed: the truth stream jumped at an impossible speed %.1f s ago "
                       "(%d such jumps) -- a coordinate-frame change, not a flight" %
                       (now - tap.last_jump_wall, tap.n_jumps))
    if fcu is not None:
        if fcu_outside:
            return True, "the FCU's own estimate is outside the fence too (%.1f m apart)" % disagree
        if disagree >= ignore_m:
            return False, ("geofence suppressed: the truth is %.1f m from the FCU's own estimate, which has "
                           "the vehicle INSIDE the fence -- the tap is describing another frame (a "
                           "coordinate change of that size is not a flight)" % disagree)
        if disagree > disagree_m:
            return True, ("the FCU disagrees by %.1f m and has the vehicle inside the fence -- NOT "
                          "suppressed: an unexplained disagreement is a diverging estimate, and the fence "
                          "is the only lateral bound this aircraft has" % disagree)
    return True, ""


class Supervisor:
    def __init__(self, args):
        self.args = args
        self.t_wall0 = time.monotonic()
        self.tap = TapTruth()
        self.bridge = None
        self.node = None
        self.phase_pub = None
        self.phase = "init"
        self.abort_reason = None
        self.next_stream = time.monotonic()
        # the LIO, read ONLY by the preflight liveness gate (design section 4.3)
        self.lio = {"t_us": 0, "p": np.zeros(3), "q": np.array([1.0, 0.0, 0.0, 0.0]),
                    "n": 0, "rate_slow": 0.0, "last_wall": 0.0}
        # the plan box's statements, from /plan/cmd -- NOT the LIO
        self.plan = {"n": 0, "last_wall": None, "ids": [], "flag": None, "last_tid": None}
        # the bridge's own report
        self.bridge_status = {"nav_hz": 0.0, "nav_age": float("inf"), "relay": 0, "sent": 0,
                              "last_wall": None, "mode": "", "armed": 0, "nav_skip": 0}
        #: (wall, nav_skip) samples for the "estimator is diverging" gate -- see check_gates
        self.nav_skip_hist = []
        #: |FCU estimate - truth| and since when it has exceeded --truth-fcu-max-m (see check_gates)
        self.fcu_div_m = 0.0
        self.fcu_div_since = None
        self.calib = None
        #: the FCU's own position, as the second opinion for the geofence (see on_fcu_odom)
        self.fcu = {"p": np.zeros(3), "wall": None, "n": 0}
        self.cmd = np.zeros(3)
        self.log_fh = None
        self.trigger_sent = False
        #: THE FOUR GATES JUDGE A MISSION IN PROGRESS.  `finish` clears this before it lands, because the
        #: conditions that ended the sortie are still true while the aircraft comes down, and a gate that
        #: re-fires during the landing makes the landing wait return immediately (`check_gates`).  It is set
        #: HERE rather than in `run` because `preflight` reaches `check_gates` through `pump`, and a gate
        #: that is read before it exists is an AttributeError on a code path nothing tests (MEASURED: the
        #: first version of this fix put it in `run` and `tools/test-stall-gate.py` failed).
        self.gates_armed = True
        #: AND THE SAME ARGUMENT APPLIES TO `route_started`, which `check_gates` also reads (`:281`) and which
        #: used to be born in `run()` (`:592`).  `check_gates` is reachable from `preflight` through `pump`,
        #: so an object built WITHOUT `run()` -- which is how `tools/test-stall-gate.py` and
        #: `tools/test-planner-swap.py` drive these gates, with no ROS and no MAVLink -- hit an
        #: AttributeError on the very gate it was about to check.  MEASURED: writing the supervisor half of
        #: the planner-swap test crashed on exactly this line, the second time this file has been bitten by
        #: state born in the wrong place.  `run()` still sets it False before preflight; this only makes the
        #: object usable without it.
        self.route_started = False
        #: THE NORMAL LANDING'S OWN OUTCOME, AND IT IS NOT A MISSION ABORT.  `land()` used to return
        #: "still armed after LAND" into `run()`'s step loop, which grades every non-None return as
        #: `abort_reason` -- so 14 completed tours in `logs/` are stamped `ABORT` and exit rc=1, and on the
        #: cut-short path the reason OVERWRITES whatever actually ended the sortie (2026-11-07 audit,
        #: finding 2).  The tour's verdict and the landing's verdict are now separate facts with separate
        #: names (and a separate return code, 2).
        self.land_outcome = None

    def log(self, msg):
        print("[%7.2f] %s" % (time.monotonic() - self.t_wall0, msg), flush=True)

    def _geofence_note(self, note):
        """Log a suppressed geofence ONCE per episode instead of every 50 Hz tick."""
        last = getattr(self, "_geo_note_last", None)
        if last is not None and time.monotonic() - last < 2.0:
            return
        self._geo_note_last = time.monotonic()
        self.log(note)

    def _nav_age_note(self, note):
        """Log a stale-but-survivable estimator feed once per 2 s, not on every 50 Hz tick."""
        last = getattr(self, "_nav_age_note_last", None)
        if last is not None and time.monotonic() - last < 2.0:
            return
        self._nav_age_note_last = time.monotonic()
        self.log("NAV AGE: " + note)

    # ------------------------------------------------------------------ plumbing
    def start_ros(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        from geometry_msgs.msg import PoseStamped
        try:
            from rclpy.signals import SignalHandlerOptions
            rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        except Exception:
            rclpy.init(args=[])
        self.node = Node("supervisor")
        best_effort = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST,
                                 reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(Odometry, self.args.truth_topic,
                                      lambda m: self.tap.on_odom(m), best_effort)
        # THE FCU'S OWN ESTIMATE, AS A SECOND OPINION ON THE TRUTH (round 85).
        # The geofence reads the tap, and the tap is emitted in "the simulator origin, i.e. the EKF origin",
        # so an EKF re-seat moves it while the vehicle stays put (measured 410.6 m of altitude against the
        # FCU's 3.6 m on pva2-02-02, 591 m horizontal on pva2-01-01).  The bridge publishes the FCU's own
        # LOCAL_POSITION_NED on /fcu/odom; a large disagreement between the two is the cheapest live test
        # that the tap is describing another frame.  Optional: if the topic is absent the geofence stays
        # exactly as it was.
        if self.args.fcu_odom_topic:
            try:
                self.node.create_subscription(Odometry, self.args.fcu_odom_topic,
                                              self.on_fcu_odom, best_effort)
            except Exception as exc:                                    # noqa: BLE001
                self.log("cannot subscribe %s (%s): the geofence stays tap-only"
                         % (self.args.fcu_odom_topic, exc))
        self.node.create_subscription(Odometry, self.args.lio_topic, self.on_lio, best_effort)
        self.node.create_subscription(String, self.args.status_topic, self.on_bridge_status, 10)
        try:
            from quadrotor_msgs.msg import PositionCommand
            self.node.create_subscription(PositionCommand, self.args.cmd_topic, self.on_cmd, latched)
        except ImportError as exc:
            self.log("cannot subscribe %s (%s): the plan's liveness gate is DISABLED, and a dead plan box "
                     "would look like a finished tour again" % (self.args.cmd_topic, exc))
        from std_msgs.msg import Bool as _Bool
        try:
            self.node.create_subscription(_Bool, self.args.stall_topic, self.on_stall, 10)
        except Exception as exc:                                        # noqa: BLE001
            self.log("cannot subscribe %s (%s)" % (self.args.stall_topic, exc))
        self.trigger_pub = self.node.create_publisher(PoseStamped, self.args.trigger_topic, latched)
        self.phase_pub = self.node.create_publisher(String, "/fsm/phase", 10)
        self.rclpy = rclpy

    def on_lio(self, m):
        """THE PREFLIGHT'S LIVENESS GATE ONLY.  Nothing in flight reads this (design section 4.3)."""
        now = time.monotonic()
        dt = max(1e-3, now - self.lio["last_wall"])
        self.lio["t_us"] = int(m.header.stamp.sec) * 1_000_000 + int(m.header.stamp.nanosec) // 1000
        self.lio["p"] = np.array([m.pose.pose.position.x, m.pose.pose.position.y,
                                  m.pose.pose.position.z])
        self.lio["q"] = np.array([m.pose.pose.orientation.w, m.pose.pose.orientation.x,
                                  m.pose.pose.orientation.y, m.pose.pose.orientation.z])
        self.lio["n"] += 1
        self.lio["last_wall"] = now
        self.lio["rate_slow"] = 1.0 / dt if self.lio["n"] == 1 else \
            (0.9 * self.lio["rate_slow"] + 0.1 * (1.0 / dt))

    def on_fcu_odom(self, m):
        """The FCU's own LOCAL_POSITION_NED (W), as a second opinion on the truth stream.

        Stored raw with its arrival wall clock; `geofence_verdict` reads it only while it is fresh
        (`--fcu-fresh-s`), so a dead bridge degrades to the tap-only geofence instead of a wrong one.
        """
        self.fcu = {"p": np.array([m.pose.pose.position.x, m.pose.pose.position.y,
                                   m.pose.pose.position.z]),
                    "wall": time.monotonic(), "n": self.fcu["n"] + 1}

    def on_cmd(self, m):
        """The plan box is talking.  THIS is the fact that used to be indistinguishable from "finished"."""
        self.plan["n"] += 1
        self.plan["last_wall"] = time.monotonic()
        self.plan["flag"] = m.trajectory_flag
        if m.trajectory_id not in self.plan["ids"]:
            self.plan["ids"].append(m.trajectory_id)
            self.plan["last_tid"] = m.trajectory_id
            if self.phase.startswith("leg"):
                self.set_phase("leg%d" % m.trajectory_id)
        p_l = np.array([m.position.x, m.position.y, m.position.z])
        self.plan["pos"] = p_l
        self.plan["frame"] = m.header.frame_id
        # THE PHASE LOG'S `cmd_*` COLUMN, in W -- the same meaning it has had since before F1 ("the command
        # in force when the phase begins"), computed through the SAME calibration the bridge converts with.
        # The supervisor sends no setpoint (class 84 is the bridge's), so this is where the column comes
        # from now: one conversion, on the command the plan box is publishing.
        if m.header.frame_id in ("lio_world", "world"):
            self.cmd = self.calib.R_WL @ p_l + self.calib.t_WL
        else:
            self.cmd = p_l

    def on_bridge_status(self, m):
        """`bridge/bridge_node.py`'s own report -- the streams it is carrying, from the process that carries
        them.  The bridge does not DECIDE anything here; it reports, and this process decides."""
        for kv in m.data.split():
            if "=" not in kv:
                continue
            k, v = kv.split("=", 1)
            if k in ("nav_hz", "nav_age"):
                try:
                    self.bridge_status[k] = float(v)
                except ValueError:
                    pass
            elif k in ("relay", "sent", "armed", "nav_skip"):
                try:
                    self.bridge_status[k] = int(v)
                except ValueError:
                    pass
            elif k == "mode":
                self.bridge_status["mode"] = v
        self.bridge_status["last_wall"] = time.monotonic()
        #: keep a short history of the bridge's refusal counter: the RATE is what says "the estimator is
        #: diverging", not the lifetime total (a single glitch at take-off must not abort the mission).
        self.nav_skip_hist.append((self.bridge_status["last_wall"], self.bridge_status["nav_skip"]))
        if len(self.nav_skip_hist) > 10:
            del self.nav_skip_hist[:-10]

    def nav_skip_rate(self):
        """Refused external-nav frames per second over the last pair of bridge reports (0 when unknown).

        `getattr` on purpose: `tools/test-stall-gate.py` builds a Supervisor-shaped stub without `__init__`,
        and a gate must not raise on a code path that has not been flown yet (the same lesson as the
        `finish`/`land` flag at line ~192).
        """
        h = getattr(self, "nav_skip_hist", None) or []
        if len(h) < 2:
            return 0.0
        (t0, n0), (t1, n1) = h[0], h[-1]
        dt = t1 - t0
        if dt <= 1e-6:
            return 0.0
        return max(0.0, (n1 - n0) / dt)

    def on_stall(self, m):
        self.planner_stalled = bool(m.data)
        if self.planner_stalled and getattr(self, "planner_stall_since", None) is None:
            self.planner_stall_since = time.monotonic()
            self.log("PLANNER STALL SIGNALLED on %s" % self.args.stall_topic)
        elif not self.planner_stalled:
            self.planner_stall_since = None

    # ------------------------------------------------------------------ the phases
    def set_phase(self, phase):
        if phase == self.phase:
            return
        self.phase = phase
        self.log("phase -> %s" % phase)
        self.log_fh.write("%.3f,%d,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n" % (
            time.monotonic(), self.tap.t_us, phase, self.cmd[0], self.cmd[1], self.cmd[2],
            self.lio["p"][0], self.lio["p"][1], self.lio["p"][2],
            self.tap.p[0], self.tap.p[1], self.tap.p[2]))
        self.log_fh.flush()
        if self.phase_pub is not None:
            from std_msgs.msg import String
            m = String()
            m.data = phase
            self.phase_pub.publish(m)

    # ------------------------------------------------------------------ the loop
    def pump(self, seconds=None, until=None):
        period = 1.0 / GATES["rate_hz"]
        t_end = None if seconds is None else time.monotonic() + seconds
        while True:
            if t_end is not None and time.monotonic() >= t_end:
                break
            if until is not None and until():
                break
            self.bridge.poll()
            self.rclpy.spin_once(self.node, timeout_sec=0.0)
            self.check_gates()
            if self.abort_reason:
                return
            time.sleep(0.001)
        return None

    def check_gates(self):
        """THE FOUR GATES, all of them the supervisor's (design section 1.3's table).

        THE GATES ARE FOR A MISSION IN PROGRESS, so they are not evaluated once the sortie is ENDING.
        `finish` sets `gates_armed = False` before it lands, and this returns at once.  THAT IS THE FIX TO
        A MEASURED LOOP, not a tidy-up: the abort's own landing pumps through here, and every gate that
        fired is STILL TRUE while the aircraft comes down -- the plan box is still dead, so the tick after
        the landing starts sets `abort_reason` again and `pump` (whose exit condition that is) returns in
        one iteration.  MEASURED on `RECT27-kill-plan` and `RECT27b-kill-plan`, 2026-10-15: both exited with
        `landed: armed=True mode=LAND` 0.05 s after the LAND, the aircraft still at 2.0 m and still armed,
        and the second one did it with the reason explicitly cleared first -- the gate put it back.  An
        abort that keeps re-firing while it lands can never wait for the landing.
        """
        if not self.gates_armed:
            return
        st = self.bridge.state()
        reasons = []
        # ★★ THE ESTIMATOR HAS LOST THE AIRCRAFT (round 89) ★★
        #   The truth tap exists only in this simulator -- which is exactly why it can arbitrate, and the
        #   fence gate already uses it.  Metres of disagreement between the FCU's own estimate and the truth
        #   is not a gust: the position loop is chasing a place the aircraft is not, and every second it
        #   continues carries the aircraft further out.
        #   MEASURED over 30 sorties tonight: every fence escape printed `the FCU disagrees by 5.0-15.3 m`
        #   only when the truth had ALREADY crossed the 13 m fence, i.e. 3.4 m outside the 10 m mission ring
        #   (`kxC-02-01`: 6.6 m of disagreement, abort at the fence).  A 2 m disagreement held for 2 s is far
        #   outside normal tracking (|fcu-truth| p50 0.16 m, p90 0.24 m on the delivered sorties) and would
        #   end the sortie while the aircraft is still near the reference.
        if (getattr(self.args, "fcu_odom_topic", None) and self.fcu["wall"] is not None and self.tap.n > 0
                and (time.monotonic() - self.fcu["wall"]) <= self.args.fcu_fresh_s):
            # np.asarray on both sides: the real streams are arrays, but `tools/test-stall-gate.py` builds
            # a Supervisor-shaped stub with plain lists, and a gate must not raise on a stub (the same
            # lesson as the `finish`/`land` flag at line ~192).
            self.fcu_div_m = float(np.linalg.norm(np.asarray(self.fcu["p"], float)[:2]
                                                  - np.asarray(self.tap.p, float)[:2]))
            if self.fcu_div_m > float(getattr(self.args, "truth_fcu_max_m", 2.0)):
                if self.fcu_div_since is None:
                    self.fcu_div_since = time.monotonic()
                    self.log("ESTIMATOR DISAGREEMENT: |FCU - truth| = %.1f m (limit %.1f m) -- timing it"
                             % (self.fcu_div_m, float(getattr(self.args, "truth_fcu_max_m", 2.0))))
                elif (time.monotonic() - self.fcu_div_since) >= float(getattr(self.args, "truth_fcu_hold_s", 2.0)):
                    reasons.append("THE ESTIMATOR HAS LOST THE AIRCRAFT: the FCU's own estimate is %.1f m from "
                                   "the truth (limit %.1f m) and has been for %.1f s -- the position loop is "
                                   "chasing a place the aircraft is not"
                                   % (self.fcu_div_m, float(getattr(self.args, "truth_fcu_max_m", 2.0)),
                                      time.monotonic() - self.fcu_div_since))
            else:
                self.fcu_div_since = None
        if self.tap.n > 0 and (abs(self.tap.p[0]) > self.args.geofence_m or
                               abs(self.tap.p[1]) > self.args.geofence_m):
            # hand the FCU's own position to the verdict, but ONLY while it is fresh: a dead bridge must
            # degrade to the tap-only geofence, not to a permanent "suppressed"
            self.tap.fcu_p = None
            if (self.args.fcu_odom_topic and self.fcu["wall"] is not None and
                    (time.monotonic() - self.fcu["wall"]) <= self.args.fcu_fresh_s):
                self.tap.fcu_p = self.fcu["p"]
            breach, note = geofence_verdict(self.tap, self.args.geofence_m, time.monotonic(),
                                            self.args.geo_grace_s, self.args.truth_disagree_m,
                                            self.args.truth_ignore_m)
            if breach:
                msg = "geofence: truth at [%+.2f %+.2f]" % (self.tap.p[0], self.tap.p[1])
                if note:
                    msg += " -- " + note
                reasons.append(msg)
            elif note:
                self._geofence_note(note)
        # THE ALTITUDE IS BOUNDED FOR THE WHOLE SORTIE, NOT ONLY FOR THE PRE-TRIGGER HOLD.  The
        # `--climb-guard-m` check used to live in `hold()` alone (2026-11-07 audit, finding 10), so after
        # the trigger NOTHING bounded the altitude: the RECT25 class -- ~9.8 m reached on a 2.00 m command
        # -- would have escaped the guard the moment the go went out.  This is the same number, read at the
        # same 50 Hz, for the whole flight.
        if self.tap.n > 0 and self.tap.p[2] > self.args.alt + self.args.climb_guard_m:
            reasons.append("ALTITUDE GUARD: the truth is at %.2f m for a %.2f m mission (guard %.2f m)"
                           % (self.tap.p[2], self.args.alt, self.args.climb_guard_m))
        if st.armed and st.mode == "LAND" and self.phase not in ("land", "done"):
            reasons.append("the FCU left the mission mode (%s)" % st.mode)
        # THE LIO'S LIVENESS IS NOT READ HERE ANY MORE.  It arrives as the BRIDGE's own report of the
        # stream it is carrying ("the stream I am transporting has stopped" is transport health, design
        # section 4.3), and the firmware's own EKF failsafe is the backstop.
        if self.plan["n"] > 0 and self.bridge_status["relay"] == 1:
            if self.bridge_status["nav_hz"] < self.args.nav_floor_hz:
                reasons.append("THE ESTIMATOR FEED HAS STOPPED: the bridge reports nav_hz=%.1f "
                               "(floor %.1f Hz, age %.2f s) -- the FCU is flying on a stale estimate"
                               % (self.bridge_status["nav_hz"], self.args.nav_floor_hz,
                                  self.bridge_status["nav_age"]))
            # THE RATE IS NOT THE WHOLE STORY (audit finding 3).  The bridge REPUBLISHES the last LIO
            # sample at --nav-hz, so `nav_hz` stays at 47 Hz after the LIO dies -- the packets keep flowing
            # and every one of them carries the same frozen pose.  `nav_age` is the age of that sample
            # (the bridge prints it on every status line and NOTHING read it), so it is the number that
            # moves: a 200 Hz estimator that is 0.5 s old has missed 100 samples.
            if self.bridge_status["last_wall"] is None:
                pass                        # no bridge report yet: nothing to gate on
            elif (time.monotonic() - self.bridge_status["last_wall"]) > self.args.status_stale_s:
                # AND THE BRIDGE ITSELF (audit finding 12).  A dead bridge freezes every number above at
                # its last value, including `nav_hz=47`, so "no news" must not read as "good news".
                reasons.append("THE BRIDGE HAS STOPPED REPORTING: nothing on %s for %.1f s (limit %.1f s)"
                               " -- the transport is dead and every stream number it last sent is a "
                               "frozen echo"
                               % (self.args.status_topic,
                                  time.monotonic() - self.bridge_status["last_wall"],
                                  self.args.status_stale_s))
            # ★★ THE ESTIMATOR IS DIVERGING (round 89) ★★
            #   The bridge refuses to forward an external-nav pose the aircraft could not physically reach
            #   and counts the refusals (`nav_skip`).  MEASURED over 32 sorties (scripts/lio_step_census.py):
            #   a clean sortie reports `nav_skip=0` for the whole flight; the two fence escapes of `kxB`
            #   reported **1312 and 2174** refusals -- 21-28 % of the feed, i.e. the LIO's own world pose was
            #   running away for 28-46 s before the aircraft reached the fence.
            #   Refusing the frames protects the EKF, but the PLANNER reads the same estimator, so the plan
            #   is being built on a state that is metres away from the vehicle.  The sequence to avoid is
            #   "keep flying until the truth crosses the fence": abort by name while it is still inside.
            rate = self.nav_skip_rate()
            _skip_max = float(getattr(self.args, "nav_skip_max_per_s", 10.0))
            if rate > _skip_max:
                reasons.append("THE ESTIMATOR IS DIVERGING: the bridge refused %.0f external-nav frames per "
                               "second (limit %.0f/s, nav_skip=%d total) -- the LIO's own pose is not physically "
                               "possible, so the plan is being built on a broken state"
                               % (rate, _skip_max,
                                  self.bridge_status.get("nav_skip", 0)))
            elif self.bridge_status["nav_age"] > self.args.nav_age_max_s:
                reasons.append("THE ESTIMATOR FEED IS STALE: the LIO sample the bridge is feeding the EKF "
                               "is %.2f s old (limit %.2f s) -- the FCU is integrating a frozen pose"
                               % (self.bridge_status["nav_age"], self.args.nav_age_max_s))
            elif self.bridge_status["nav_age"] > self.args.nav_age_warn_s:
                # THE SURVIVABLE BAND.  MEASURED kv1-01-02 (rtf 0.93, loaded host): 0.54 s of staleness,
                # aircraft still flying; the abort threshold is 1.5 s so this band only speaks, once per
                # episode, and the log then says how the sortie ended rather than leaving a gap.
                self._nav_age_note("the LIO sample the bridge is feeding the EKF is %.2f s old "
                                   "(abort above %.2f s)" % (self.bridge_status["nav_age"],
                                                             self.args.nav_age_max_s))
        # THE PLAN BOX'S LIFELINE: silence is an error now that the end of the tour is a FIELD.
        if self.route_started and self.plan["last_wall"] is not None and self.args.plan_silence_s > 0:
            age = time.monotonic() - self.plan["last_wall"]
            if age > self.args.plan_silence_s:
                reasons.append("THE PLAN BOX HAS STOPPED TALKING: nothing on %s for %.1f s (limit %.1f s) "
                               "after %d messages, trajectory_id %s -- a plan box that is DONE says so in "
                               "the message (trajectory_flag = COMPLETED); SILENCE IS NOT COMPLETION"
                               % (self.args.cmd_topic, age, self.args.plan_silence_s, self.plan["n"],
                                  self.plan["last_tid"]))
        stall = self._stall_reason()
        if stall:
            reasons.append(stall)
        if reasons:
            self.abort_reason = "; ".join(reasons)

    def _stall_reason(self):
        """The legacy `--stall-topic` gate, CARRIED, not dropped (design section 8 item 8): it is the only
        NAMED cause for a dead source that any source could raise, and its replacement (the command stream's
        own liveness, above) is new."""
        if not getattr(self, "planner_stalled", False):
            return None
        up = time.monotonic() - getattr(self, "planner_stall_since", time.monotonic())
        quiet = time.monotonic() - (self.plan["last_wall"] or 0.0)
        if up < self.args.stall_abort_s or quiet < self.args.stall_abort_s:
            return None
        return ("THE PLANNER IS NOT PLANNING: %s has been up for %.1f s and nothing has arrived on %s for "
                "%.1f s -- this is NOT a plan-box timeout" % (self.args.stall_topic, up,
                                                              self.args.cmd_topic, quiet))

    # ------------------------------------------------------------------ preflight / takeoff / land
    def check_own_link(self):
        """CONDITION 1 OF THE F2 GATE, on THIS process's own channel: request, then MEASURE.

        TWO STAGES, because the bench measured that they are different facts.  MEASURED
        (`docs/metrics/f2-second-endpoint-20261016.txt`): the FCU ACCEPTS a 511 request, READS IT BACK as
        exactly what was asked (50000 us) and then sends NOTHING -- its intervals are served from a fixed set
        of buckets (`GCS_Common.cpp:1734-1770`), so the rate you asked for is not the rate you get.  And
        MEASURED while building the bench: before the EKF origin exists, Copter has no position to send and
        LOCAL_POSITION_NED does not exist at all (90 s of nothing), so a position check BEFORE the origin
        would refuse a healthy stack.  Hence: stage 1 = the heartbeat (always broadcast); stage 2 = the
        position stream, MEASURED AFTER the origin is set (`check_position_stream`).
        """
        st = self.bridge.state()
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.args.link_wait_s:
            self.bridge.poll()
            st = self.bridge.state()
            if st.last_heartbeat_us:
                break
            time.sleep(0.05)
        if not st.last_heartbeat_us:
            self.log("REFUSING: no heartbeat on %s (this process's OWN endpoint)" % self.args.mav)
            return False
        self.log("supervisor: stage 1 ok -- a heartbeat on %s (armed=%s mode=%s landed_state=%s, "
                 "alt AMSL %.1f m)"
                 % (self.args.mav, st.armed, st.mode, st.landed_state, st.alt_amsl_m))
        return True

    def check_position_stream(self, what):
        """Stage 2: the LOCAL_POSITION_NED this endpoint ASKED for, counted as it arrives.

        IT RE-ASKS.  MEASURED on this very endpoint (`logs/missionA-RECT25-kill-plan/supervisor.log`): at
        0.6 s of this process's life the 511 request reached an FCU that was still booting, and the stream
        was 0.0 Hz nine seconds later -- while the F2 bench measured the identical request delivering
        50.2 Hz when it was made after the FCU was up.  So the loop is request -> measure -> request again,
        and it refuses only when the rate is STILL below the floor after `--link-retries`.
        """
        for attempt in range(1, self.args.link_retries + 1):
            n0 = self.bridge.state().last_position_us
            n, t0 = 0, time.monotonic()
            while time.monotonic() - t0 < 2.0:
                self.bridge.poll()
                if self.bridge.state().last_position_us != n0:
                    n0 = self.bridge.state().last_position_us
                    n += 1
                time.sleep(0.002)
            hz = n / (time.monotonic() - t0)
            self.log("supervisor: stage 2 attempt %d/%d (%s) -- LOCAL_POSITION_NED %.1f Hz MEASURED on "
                     "this endpoint (floor %.1f Hz)"
                     % (attempt, self.args.link_retries, what, hz, self.args.link_floor_hz))
            if hz >= self.args.link_floor_hz:
                return True
            if attempt < self.args.link_retries:
                self.log("      re-asking this endpoint for its rates (a 511 sent into the FCU's boot is "
                         "accepted by nobody -- MEASURED)")
                self.bridge.request_rates()
                time.sleep(1.0)
        self.log("REFUSING TO CONTINUE: this endpoint's own position stream is still below the %.1f Hz "
                 "floor after %d attempts.  MEASURED on the bench, a 511 request can be ACCEPTED, read back "
                 "exactly as asked, and deliver nothing -- so 'I asked for it' is not evidence."
                 % (self.args.link_floor_hz, self.args.link_retries))
        return False

    def preflight(self):
        self.set_phase("preflight")
        self.log("supervisor: navigation=%s, this process's endpoint %s, the bridge's %s"
                 % (self.args.nav_source, self.args.mav, self.args.cmd_topic))
        if not self.check_own_link():
            return "this process's own MAVLink endpoint did not verify"
        self.log("waiting for the LIO on %s (PREFLIGHT ONLY: in flight the LIO is the plan box's and the "
                 "bridge's, never the supervisor's)" % self.args.lio_topic)
        self.pump(self.args.lio_wait_s, until=lambda: self.lio["n"] > 20)
        if self.lio["n"] <= 20:
            return "no LIO odometry on %s" % self.args.lio_topic
        self.log("LIO live: %d messages, rate %.1f Hz" % (self.lio["n"], self.lio["rate_slow"]))
        # the parked alignment, MEASURED AND PRINTED -- a CHECK now, not the frame the bridge converts with
        # (the bridge loads the same --calib-file the plan box publishes with, so the L->W->wire round trip
        # is exact; see `bridge/bridge_node.py::load_calib` for why that moved).
        samples = []
        t_end = time.monotonic() + 3.0
        while time.monotonic() < t_end:
            self.pump(0.02)
            if self.tap.n > 0 and self.lio["n"] > 0:
                samples.append((self.lio["p"].copy(), self.tap.p.copy()))
        if len(samples) < 20:
            return "not enough parked samples to check the alignment"
        L = np.array([s[0] for s in samples])
        W = np.array([s[1] for s in samples])
        t = W.mean(axis=0) - self.calib.R_WL @ L.mean(axis=0)
        resid = (self.calib.R_WL @ L.T).T + t - W
        self.log("      parked alignment CHECK: translation measured from %d samples is t = "
                 "[%+.4f %+.4f %+.4f] m, rms %.4f m; the bridge converts with %s"
                 % (len(samples), t[0], t[1], t[2],
                    float(np.sqrt(np.mean(np.sum(resid ** 2, axis=1)))), self.calib.summary()))
        self.log("      (the parked mismatch to the file the bridge uses is [%+.3f %+.3f %+.3f] m in W; the "
                 "rotation gets checked on the flight data, where the path shape identifies it)"
                 % tuple(t - self.calib.t_WL))
        self.log("setting the EKF origin")
        if not self.bridge.set_ekf_origin(HOME_LAT, HOME_LON, HOME_ALT, pump=self.pump):
            return "EKF origin never accepted"
        self.pump(0.5)
        if not self.check_position_stream("after the EKF origin, which is what makes Copter able to send "
                                          "a position at all"):
            return "this process's own position stream never arrived after the EKF origin"
        st = self.bridge.state()
        self.log("FCU: armed=%s mode=%s pos=[%+.3f %+.3f %+.3f] alt AMSL %.1f m gps_fix=%d"
                 % (st.armed, st.mode, *st.pos, st.alt_amsl_m, st.gps_fix))
        self.log("      (this line is the stack's assertion that the EKF origin IS the pad, which is what "
                 "makes W and the FCU's local frame the same up to axis labels)")
        if st.gps_fix > 1:
            return "the FCU has a GPS fix (%d) -- mission A is defined as no-GPS" % st.gps_fix
        if not self.bridge.set_mode(Mode.GUIDED):
            return "could not enter GUIDED"
        if not self.bridge.calibrate_accel_simple():
            self.log("note: simple accel calibration not ACKed (may already be calibrated)")
        self.pump(0.5)
        if not self.bridge.arm(timeout_s=25.0):
            return "arming refused: %s" % _texts(self.bridge.state())
        self.pump(2.0)
        st = self.bridge.state()
        if not st.armed:
            return "the FCU does not report itself armed"
        self.log("armed; landed_state=%d" % st.landed_state)
        return None

    def takeoff(self):
        """THE TAKE-OFF, and WHICH CONTRACT IS FLOWN is decided by `--takeoff-command-m` and stated below.

        TWO MECHANISMS, BOTH IN THE CODE, ONE OF THEM FLOWN, and the log says which:

        * THE COMMAND HEIGHT.  `--takeoff-command-m` is what goes out as `packet.z` -- VERBATIM, with no
          datum added (`bridge/mavlink_impl.py::takeoff`; the bench settled that a 2.500 m request is
          ACCEPTED and ADOPTED as 2.5 m above the pad, CTUN DAlt peak +2.525).  The FCU's take-off
          controller then flies the aircraft to that height and its `landed_state` becomes flying.
        * THE SETPOINT-DRIVEN CLIMB (the OWNER'S design change, 2026-10-15: "起飞命令就飞20cm高，然后立马进定点").
          With `--takeoff-command-m 0.2` the command is only "leave the ground cleanly" and the CLIMB TO THE
          MISSION ALTITUDE IS THE PLAN'S: `--trigger-on-airborne` puts the go out the moment the vehicle is
          airborne, and the plan box's first target is a VERTICAL leg to the mission altitude
          (`rectangle_source.py --vertical-first-m`), held for the A4 window.  Position setpoints cancel
          GUIDED's take-off SubMode by design (STATUS row 25, trap 9), which is what hands the climb over.
        * `--climb-guard-m` STAYS ON either way and bounds the climb against the MISSION altitude
          (`--alt`), not against the command, so a 0.2 m command with a 2.5 m mission does not trip it.
        """
        self.set_phase("takeoff")
        if not self.bridge.takeoff(self.args.takeoff_command_m):
            return "takeoff refused: %s" % _texts(self.bridge.state())
        z0 = float(self.tap.p[2])
        if abs(self.args.takeoff_command_m - self.args.alt) < 1e-9:
            self.log("takeoff commanded to %.2f m, the mission altitude, VERBATIM (mechanism: the FCU's "
                     "own take-off controller flies the whole climb); the plan's setpoints are NOT on the "
                     "wire yet -- position setpoints cancel GUIDED's take-off SubMode"
                     % self.args.takeoff_command_m)
        else:
            self.log("takeoff commanded to only %.2f m (mechanism: SETPOINT-DRIVEN CLIMB -- the command "
                     "is just 'leave the ground', the mission altitude %.2f m is the PLAN's; the go goes "
                     "out on airborne=%s and the plan's first target is a vertical leg)"
                     % (self.args.takeoff_command_m, self.args.alt, self.args.trigger_on_airborne))
        # The airborne threshold is BELOW the command: with a 0.2 m command the FCU reaches its own target
        # and stops, so a threshold at the command height would never be met.  It is a "has it left the pad"
        # test, not a "has it reached the target" one.
        #
        # TWO CLOCKS AND TWO NAMED HARNESS FAILURES (2026-10-25).  This guard used to be ONE signal -- a
        # 0.10 m rise of the truth -- and 12 s of WALL time.  Measured on a nominal flight
        # (logs/missionA-mA-repro2): the FCU's vertical datum is ~0.10 m, so a 0.20 m command gives a TRUTH
        # RISE OF 0.107 m and the old threshold passed by **8 mm**.  And the truth advances in SIMULATION
        # seconds while `pump()` measures `time.monotonic()`: on a starved host (rtf 0.40-0.57) four
        # 2026-10-25 sorties died as "never left the ground" while the FCU's own log (SIM2) shows the
        # aircraft climbing (`docs/metrics/handoff-audit-20261025.txt`).  So:
        #   * the threshold is HALF the command (0.05 m for a 0.2 m command), because the datum eats the
        #     other half: the 0.107 m rise measured on a nominal flight is 93 % of a 0.10 m threshold;
        #   * the deadline is 12 s of SIM time from the tap's own stamps, with a 60 s WALL cap that
        #     NAMES the host when the host is the problem;
        #   * a tap that stops advancing is a named HARNESS failure after 2 s of wall time, not an
        #     aircraft verdict.
        # WHAT IS DELIBERATELY *NOT* CHANGED: the DEFINITION of airborne is still "the TRUTH rose above
        # the pad", not the FCU's `landed_state`.  Accepting `landed_state` as an OR was tried and flown
        # (`logs/missionA-guard-01`): it fires at truth 0.100 m (landed_state 3 = TAKING_OFF), 3.3 s
        # before the truth crosses, which moves the A4 window and the whole climb profile -- A2 went
        # 1.85 -> 4.02 mm with THAT as the only change.  A harness fix must not move the numbers the
        # frozen release is compared against, so `landed_state` stays out of the pass condition.
        thr = min(0.25, max(0.05, 0.25 * self.args.takeoff_command_m))
        t_sim0 = self.tap.t_us
        n_tap0 = self.tap.n
        t_wall0 = time.monotonic()
        why = None
        while True:
            if self.tap.p[2] > z0 + thr:
                why = "truth rise > %.2f m" % thr
                break
            wall_s = time.monotonic() - t_wall0
            sim_s = ((self.tap.t_us - t_sim0) / 1e6) if t_sim0 > 0 else wall_s
            # THE TIMEOUT IS WALL TIME.  "Did it leave the ground within N seconds" is a question
            # about DURATION, and the simulator's clock is not a duration: truth_rx gates its
            # publishes on the tap's own timestamp, so when it falls behind it flushes a backlog
            # whose stamps are 10 s old, and this loop sees the sim clock jump.  MEASURED
            # (missionA-rp-02): rtf 1.0019 (the host gate PASSED) yet "never left the ground
            # ... 15.1 s of SIM time in 5.9 s of wall time" -- the window expired because a
            # catch-up arrived, while the aircraft had not had 6 s of real flight.  sim_s is
            # still computed and reported; it is simply no longer the judge.
            if wall_s >= self.args.airborne_timeout_s:
                break
            if self.tap.n == n_tap0 and wall_s >= 2.0:
                return ("the TRUTH FEED IS STALE: no new tap sample in %.1f s of wall time (n=%d). "
                        "The airborne test cannot be judged -- this is a HARNESS failure (tap/fanout/"
                        "recorder), NOT the aircraft" % (wall_s, self.tap.n))
            if wall_s >= 60.0:
                return ("never left the ground AND the tap advanced only %.3f s of sim time in "
                        "60.0 s of wall time (rtf %.2f): the HOST is not keeping real time, so neither "
                        "the guard nor any metric of this run is gradeable (truth %.3f m, threshold "
                        "%.2f m above %.3f m)"
                        % (sim_s, sim_s / wall_s, self.tap.p[2], thr, z0))
            self.pump(0.02)
            if self.abort_reason:
                return self.abort_reason
        wall_s = time.monotonic() - t_wall0
        sim_s = ((self.tap.t_us - t_sim0) / 1e6) if t_sim0 > 0 else wall_s
        if why is None:
            return ("never left the ground (truth %.3f m after %.1f s of WALL time / %.1f s of SIM "
                    "time, tap/sim ratio %.2f; threshold %.2f m above %.3f m)"
                    % (self.tap.p[2], wall_s, sim_s, (sim_s / wall_s if wall_s > 0 else 0.0),
                       thr, z0))
        self.log("airborne (%s): truth altitude %.3f m (threshold %.2f m; the take-off command was "
                 "%.2f m); %.2f s of sim time in %.2f s of wall time"
                 % (why, self.tap.p[2], thr, self.args.takeoff_command_m, sim_s, wall_s))
        return None

    def hold(self, seconds):
        """The hold, and then THE GO.  Until the trigger, NO setpoint is on the wire, so the aircraft is
        held by the FCU's own take-off target and NOTHING the mission commands bounds it.

        THAT IS A DESIGN SMELL, MEASURED.  On `RECT25-kill-plan` the FCU's own take-off target was ~9.8 m
        while the supervisor had asked for 2.5 m, and for the 3 s of this hold plus the pre-trigger slack
        the only thing between the aircraft and that target was the FCU (`CTUN alt +9.757 / dalt +9.797`).
        The aircraft came back to 2.0 m the moment the plan's setpoints reached it.  Two responses, both
        implemented and NEITHER YET FLOWN:
          * `--climb-guard-m` (default 1.0, ON): if the TRUTH climbs more than this above the commanded
            take-off altitude before the trigger, the supervisor STOPS waiting and takes over with a NAMED
            abort -- this box is the one allowed to read the truth, so this is where the bound belongs;
          * `--trigger-on-airborne`: publish the go as soon as the vehicle is airborne instead of after this
            hold, so the plan's setpoints pin the altitude for the whole climb (at the cost of the A4
            window being a climb-and-transit rather than a hover; the default keeps the flown profile).
        """
        self.set_phase("hold")
        if self.args.trigger_on_airborne:
            self.log("the go goes out NOW (--trigger-on-airborne): the plan's setpoints, not the FCU's "
                     "take-off target, bound the climb")
            self.publish_trigger()
        self.log("holding for %.1f s before the trigger (the A4 window); the climb guard is %.2f m above "
                 "%.2f m" % (seconds, self.args.climb_guard_m, self.args.alt))
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            self.pump(0.05)
            if self.abort_reason:
                return self.abort_reason
            if self.tap.n > 0 and self.tap.p[2] > self.args.alt + self.args.climb_guard_m:
                return ("THE TAKE-OFF ALTITUDE DID NOT MATCH THE REQUEST: the truth is at %.2f m for a "
                        "commanded %.2f m (guard %.2f m).  ArduPilot's GUIDED take-off GUARD compares the "
                        "requested number against its GLOBAL altitude while its EXECUTOR ADDS that same "
                        "number to the current up-estimate (`ArduCopter/takeoff.cpp`: "
                        "`do_user_takeoff_U_m`'s `takeoff_alt_m <= copter.current_loc.alt * 0.01` vs "
                        "`_TakeOff::start_m`'s `take_off_complete_alt_m = take_off_start_alt_m + alt_m`), "
                        "so a datum-compensated request can pass the guard and command a climb of that "
                        "whole datum.  MEASURED once: `RECT25-kill-plan` reached ~9.8 m on a 2.50 m command "
                        "(docs/metrics/f2-supervisor-20261016.txt section 4) -- this gate would have named "
                        "it in the first second"
                        % (self.tap.p[2], self.args.alt, self.args.climb_guard_m))
        return None

    def publish_trigger(self):
        from geometry_msgs.msg import PoseStamped
        m = PoseStamped()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.header.frame_id = "world_w"
        m.pose.orientation.w = 1.0
        self.trigger_pub.publish(m)
        self.trigger_sent = True
        self.log("THE GO: %s published (latched, once, when the vehicle is airborne and holding) -- the "
                 "bridge may now put the plan's setpoints on the wire" % self.args.trigger_topic)

    def route(self):
        """Watch the plan's statements until it declares COMPLETED, and window the phases it implies."""
        if not self.trigger_sent:
            self.publish_trigger()
        t0 = time.monotonic()
        while self.plan["n"] == 0:
            self.pump(0.05)
            if self.abort_reason:
                return self.abort_reason
            if time.monotonic() - t0 > self.args.plan_wait_s:
                return ("THE PLAN BOX IS NOT TALKING: nothing on %s within %.1f s of the trigger"
                        % (self.args.cmd_topic, self.args.plan_wait_s))
        self.route_started = True
        self.log("the plan box is talking: %d messages, trajectory_id %s, frame %s"
                 % (self.plan["n"], self.plan["last_tid"], self.plan.get("frame")))
        self.set_phase("leg%d" % (self.plan["last_tid"] or 1))
        while True:
            self.pump(0.05)
            if self.abort_reason:
                return self.abort_reason
            if self.plan["flag"] == TRAJECTORY_STATUS_COMPLETED:
                self.log("the plan box has DECLARED THE TOUR COMPLETE (trajectory_flag = COMPLETED after "
                         "waypoint %s): %.1f s of route, %d setpoints relayed by the bridge"
                         % (self.plan["last_tid"], time.monotonic() - t0, self.bridge_status["sent"]))
                return None

    def land(self, settle_s=5.0):
        """NORMAL LAND.  The plan is already off the wire -- the BRIDGE takes it off when the stream says
        COMPLETED, and again when the mode leaves GUIDED -- so this is the authority acting alone.

        `settle_s` IS NOT A DELAY FOR ITS OWN SAKE; it is the answer to a MEASURED false end.  The wait
        below is "until the FCU says it is not armed", and `armed` is a LEVEL in the heartbeat, not an
        event: the 1 Hz heartbeat that was in flight when LAND was commanded can still carry `armed=True`
        while the mode has already changed.  MEASURED on `RECT27b-kill-plan` (2026-10-15): the abort's LAND
        was followed by `landed: armed=True mode=LAND` 0.05 s later and the process exited with the
        aircraft at 2.0 m, so a wait that starts by sampling the flag immediately can end on a STALE
        sample.  Settling first costs at most `settle_s` on a landing that cannot finish faster than that
        anyway (a LAND from 2.5 m takes ~12 s), and removes the whole class of "it said not-armed once".

        ★ THE LANDING'S OWN VERDICT IS NOT THE MISSION'S (2026-11-07 audit, finding 2) ★
        It returns None when the aircraft is down and disarmed, and a NAMED string otherwise; `run()` keeps
        that string in `land_outcome` and exits 2, so a completed tour with a failed landing is no longer
        recorded as `ABORT` -- and the reason that ended the sortie (on the abort path) is never overwritten
        by a landing outcome.  The gates are disarmed for the duration for the same reason `finish()` does
        it: the conditions that ended the mission are still true while the aircraft comes down, and a gate
        that re-fires during the landing makes `pump` return in one iteration -- which is how a landing
        gets cut short to 26.8 s and the aircraft is found still armed (measured `kp1-01-01`).

        AND IT DISARMS.  `MavlinkBridge.disarm()` existed and had no caller, `DISARM_DELAY 0` switches off
        the FCU's own timer fallback, so a LAND that the FCU declines to finish left the motors spinning
        until the SITL process was killed.
        """
        self.set_phase("land")
        was_armed = self.gates_armed
        self.gates_armed = False
        try:
            self.bridge.land()
            self.pump(settle_s)                 # let the commanded mode take effect before it is trusted
            self.pump(40.0, until=lambda: not self.bridge.state().armed)
            st = self.bridge.state()
            if st.armed:
                self.log("LANDING: still armed %.1f s after LAND (mode=%s, alt AMSL %.1f m) -- sending "
                         "DISARM" % (settle_s + 40.0, st.mode, st.alt_amsl_m))
                try:
                    self.bridge.disarm()
                except Exception as exc:                            # noqa: BLE001
                    self.log("LANDING: the DISARM command failed: %s" % exc)
                self.pump(5.0, until=lambda: not self.bridge.state().armed)
                st = self.bridge.state()
            self.log("landed: armed=%s mode=%s" % (st.armed, st.mode))
        finally:
            self.gates_armed = was_armed
        self.set_phase("done")
        if st.armed:
            return ("LANDING INCOMPLETE: the FCU is still armed %.1f s after LAND (mode=%s, alt AMSL %.1f "
                    "m) even after DISARM -- the tour's own rows are unaffected, but the aircraft has to be "
                    "disarmed before the next sortie"
                    % (settle_s + 40.0, st.mode, st.alt_amsl_m))
        return None

    def run(self):
        from bridge.mavlink_impl import MavlinkBridge
        if self.args.calib_file and os.path.exists(self.args.calib_file):
            import json
            d = json.load(open(self.args.calib_file))
            self.calib = frames.Calib(R_WL=np.array(d["R_WL"], dtype=float),
                                      t_WL=np.array(d["t_WL"], dtype=float))
        else:
            self.calib = frames.Calib.predicted(spawn_heading_deg=0.0, world_axes="body")
        self.log_fh = open(self.args.phase_log, "w")
        self.log_fh.write("t_wall,t_sim_us,phase,cmd_x,cmd_y,cmd_z,lio_x,lio_y,lio_z,truth_x,truth_y,"
                          "truth_z\n")
        self.route_started = False
        self.planner_stalled = False
        self.planner_stall_since = None
        self.gates_armed = True                    # set in __init__ too; see the comment there
        self.start_ros()
        self.bridge = MavlinkBridge(url=self.args.mav, log=self.log)
        try:
            steps = [self.preflight, self.takeoff, lambda: self.hold(self.args.hold), self.route]
            for step in steps:
                err = step()
                if err:
                    self.abort_reason = err
                    break
            else:
                # ★ THE NORMAL LAND IS NOT A STEP OF THE MISSION (audit finding 2) ★
                #   It used to be the fifth entry in `steps`, so its "still armed after LAND" string was
                #   graded as a mission abort: a perfect 13/13 tour exited rc=1 with phase `abort` and the
                #   message `ABORT: still armed after LAND`.  MEASURED: 14 sorties in `logs/`, every one of
                #   them after a completed tour.  The landing now has its own field, its own name in the log
                #   and its own return code (2), and the tour's verdict stays clean.
                self.land_outcome = self.land()
        finally:
            self.finish()
        if self.abort_reason is not None:
            return 1
        return 2 if self.land_outcome else 0

    def finish(self):
        if self.abort_reason:
            # THE REASON IS CARRIED OUT OF THE PUMP LOOP, NOT LEFT IN IT.  `pump` STOPS as soon as
            # `abort_reason` is set (it is the loop's own exit), so a pump that runs while the reason is
            # still set is a ZERO-SECOND no-op.  The reason is restored before the final read, so the
            # summary and the process rc still name what ended the sortie.  WHERE ELSE IT HAS TO BE CLEARED
            # is a different occurrence of the same signature and is explained at the landing below.
            reason = self.abort_reason
            self.abort_reason = None
            self.gates_armed = False
            self.set_phase("abort")
            self.log("ABORT: %s" % reason)
            try:
                # RTL is available to the supervisor (it is a class-76 mode); the mission's own abort has
                # used LAND since S3, and the choice is a parameter rather than a rewrite.
                if self.args.abort_mode == "rtl":
                    self.bridge.set_mode(Mode.RTL)
                    self.pump(self.args.abort_land_s, until=lambda: not self.bridge.state().armed)
                else:
                    # THE LANDING IS THE SAME ACT AS THE NORMAL ONE, so it is the same code: `self.land()`
                    # commands LAND, pumps until the FCU reports DISARMED (bounded by the landing's own
                    # 40 s), names the phase `land` -- the phase log's ending is therefore the same on an
                    # abort as on a completed tour -- and logs `landed: armed=... mode=...`.
                    #
                    # MEASURED ON `RECT27-kill-plan`, 2026-10-15, WHICH IS WHY THIS IS NOT A
                    # COMMAND-AND-EXIT: the old abort path sent LAND and fell straight through to the final
                    # read 0.05 s later --
                    #     [  28.94] bridge/mavlink: mode -> LAND
                    #     [  28.94] final: armed=True mode=LAND alt AMSL 586.0 m
                    # -- and the FCU's own log for that sortie ends with the aircraft STILL ARMED AT
                    # 2.002 m (no MODE 9 until t 41.72, the FCU still in mode 4 when the log closed).  The
                    # abort had taken over the MODE and let go of the AIRCRAFT.  A process that exits before
                    # the vehicle is down has not ended the sortie cleanly, whatever its phase log says.
                    #
                    # WHAT IS DELIBERATELY NOT DONE: the landing's own "still armed after LAND" verdict is
                    # DISCARDED.  `reason` is why this sortie ended and what a reader must see; overwriting
                    # it with a post-abort outcome would lose the finding.  That outcome is not hidden --
                    # `landed: armed=...` and the `final:` line below carry it, and `tools/metrics.py`'s B4
                    # reads it off the phase log.
                    #
                    # AND THE REASON IS CLEARED FOR THE DURATION, WHICH IS NOT BOOKKEEPING.  `pump` returns
                    # the moment `abort_reason` is set, and `check_gates` sets it AGAIN during the landing:
                    # the tick after LAND arrives reads `st.armed and st.mode == "LAND" and self.phase not
                    # in ("land", "done")` -- "the FCU left the mission mode" -- which is TRUE of the very
                    # landing this block just commanded.  MEASURED on `RECT27b-kill-plan`: with the reason
                    # left set, the landing pump ran ONE iteration and the process again exited with the
                    # aircraft armed at 2.0 m.  The parking brake of the abort was switching itself back on.
                    self.abort_reason = None
                    self.land_outcome = self.land()
            except Exception as e:                                      # pragma: no cover
                self.log("could not command the abort mode: %s" % e)
            self.abort_reason = reason
        # THE LANDING'S OWN VERDICT, NAMED AS SUCH (audit finding 2).  It is reported even when an abort
        # is why we are here: the abort names the mission, this names the aircraft's state.
        if self.land_outcome:
            self.log("LANDING INCOMPLETE: %s" % self.land_outcome)
        st = self.bridge.state()
        ctr = self.bridge.counters()
        self.log("final: armed=%s mode=%s alt AMSL %.1f m; %d setpoints, %d external-nav messages "
                 "(the SUPERVISOR sent none of them: class 84 is the bridge's)"
                 % (st.armed, st.mode, st.alt_amsl_m, ctr["positions"], ctr["external_nav"]))
        self.log("the FCU's last words: %s" % _texts(st))
        self.log_fh.close()
        try:
            self.bridge.close()
        except Exception:
            pass


def _texts(st):
    return " | ".join(t for _, t in st.messages[-4:]) or "(nothing)"


def main(argv=None) -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--mav", default="tcp:127.0.0.1:5761",
                    help="THIS process's own MAVLink endpoint.  The bridge is on serial0 (5760) and this is "
                         "serial1 (5761): one writer per message class, no arbitration (design section 4.2)")
    ap.add_argument("--truth-topic", default="/sim/truth/odom")
    ap.add_argument("--lio-topic", default="/lio/robo/odom",
                    help="read ONLY by the preflight liveness gate; never in flight")
    ap.add_argument("--cmd-topic", default="/plan/cmd")
    ap.add_argument("--status-topic", default="/bridge/status")
    ap.add_argument("--trigger-topic", default="/plan/trigger")
    ap.add_argument("--stall-topic", default="/planner/stalled")
    ap.add_argument("--stall-abort-s", type=float, default=3.0)
    ap.add_argument("--nav-source", default="lio", choices=["lio"],
                    help="the truth-fed calibration flight is NOT ported to F2 -- see the evidence file, "
                         "section 'what this round could not do'")
    ap.add_argument("--calib-file", default="")
    ap.add_argument("--alt", type=float, default=2.5, help="MISSION altitude, m above the pad.  This is the "
                    "height the guard and the A4 window are judged against; it is NOT automatically what "
                    "the take-off command asks for (see --takeoff-command-m)")
    ap.add_argument("--takeoff-command-m", type=float, default=0.2,
                    help="WHAT GOES OUT AS packet.z, verbatim, m above the pad -- the OWNER'S take-off "
                         "design (2026-10-15): 0.2 m is 'just leave the ground cleanly', and the climb to "
                         "--alt is flown by the PLAN's setpoints once the go is out.  Set it equal to "
                         "--alt for the other mechanism, in which the FCU's own take-off controller flies "
                         "the whole climb (BENCH: a 2.50 m command is ACCEPTED and ADOPTED as 2.50 m above "
                         "the pad, CTUN DAlt peak +2.525).  THE FLOWN ONE IS PRINTED AT START-UP")
    ap.add_argument("--hold", type=float, default=3.0, help="seconds at the take-off point before the go")
    ap.add_argument("--climb-guard-m", type=float, default=1.0,
                    help="ABORT if the truth climbs this far above the commanded take-off altitude "
                         "before the trigger: until the plan setpoints reach the FCU, nothing the "
                         "mission commands bounds the climb (MEASURED: 9.6 m on a 2.50 m command)")
    ap.add_argument("--airborne-timeout-s", type=float, default=20.0,
                    help="WALL-clock seconds to wait for the truth to rise.  It used to be a "
                         "SIM-time window of 12 s, which a burst of queued tap samples can expire "
                         "in a fraction of a second: MEASURED missionA-rp-02 reported 'never left "
                         "the ground ... 15.1 s of SIM time in 5.9 s of wall time' while its own "
                         "host gate read rtf 1.0019, i.e. the window was eaten by a 2.6x tap "
                         "catch-up, not by the aircraft.  Wall time cannot be faked by the "
                         "simulator's clock, so this is STRICTER than the window it replaces: it "
                         "removes a way to fail spuriously without adding a way to pass falsely")
    ap.add_argument("--trigger-on-airborne", dest="trigger_on_airborne", action="store_true",
                    default=True,
                    help="DEFAULT: publish the go the moment the vehicle is airborne, so the plan's "
                         "setpoints bound the climb.  MEASURED on RECT25-kill-plan: with the go held "
                         "back for the take-off hold, the FCU's own take-off target ran to ~9.8 m on "
                         "a 2.50 m command and only the arrival of the plan's setpoints stopped it "
                         "(docs/metrics/f2-supervisor-20261016.txt section 4).  NOT YET FLOWN in this "
                         "form: it changes the A4 window from a hover at the take-off point to a "
                         "climb-and-transit")
    ap.add_argument("--trigger-after-hold", dest="trigger_on_airborne", action="store_false",
                    help="the F1 sequencing (go after the hold), kept for the A/B -- it is the "
                         "sequence that let the take-off target run away")
    ap.add_argument("--geofence-m", type=float, default=4.60)
    #: how long after a truth-stream TELEPORT the geofence stays suppressed (see TapTruth).  A frame change
    #: moves the stream once; 2 s is longer than the one tick it takes the geofence to re-read it and much
    #: shorter than any real excursion (the enforcer needs ~1 s to cross 10 m at its own limit).
    ap.add_argument("--geo-grace-s", type=float, default=2.0)
    #: the FCU's own position, as the second opinion for the geofence.  Empty = tap-only (old behaviour).
    ap.add_argument("--fcu-odom-topic", default="/fcu/odom",
                    help="Odometry in W, the FCU's own LOCAL_POSITION_NED; the geofence is suppressed while "
                         "the truth stream is farther than --truth-disagree-m from it")
    ap.add_argument("--fcu-fresh-s", type=float, default=0.5,
                    help="only use the FCU opinion if it arrived this recently (a dead bridge must not "
                         "suppress the geofence forever)")
    ap.add_argument("--truth-disagree-m", type=float, default=2.0,
                    help="horizontal disagreement above which the truth stream and the FCU are treated as "
                         "DIFFERENT SOURCES worth naming.  It no longer suppresses a breach by itself: a "
                         "disagreement of a few metres is a diverging estimate, which is exactly what the "
                         "fence must catch (MEASURED kp1-01-01: 3.8-5.8 m suppressions, then a crash)")
    ap.add_argument("--truth-ignore-m", type=float, default=50.0,
                    help="horizontal disagreement at or above which the TAP is treated as being in another "
                         "frame and the breach is suppressed (provided the FCU has the vehicle inside the "
                         "fence).  The measured frame changes were 338/410/591 m; the worst measured "
                         "EKF-vs-truth divergence was 18.8 m, so 50 m separates them")
    ap.add_argument("--abort-mode", default="land", choices=["land", "rtl"])
    ap.add_argument("--abort-land-s", type=float, default=40.0,
                    help="wait up to this long for the FCU to DISARM on an RTL abort.  A LAND abort does "
                         "not use it: it goes through `self.land()`, which owns the same 40 s wait the "
                         "normal landing uses -- MEASURED on RECT27-kill-plan, where the old abort path "
                         "sent LAND and exited 0.05 s later with the aircraft armed at 2.0 m")
    ap.add_argument("--link-wait-s", type=float, default=30.0)
    ap.add_argument("--link-floor-hz", type=float, default=10.0,
                    help="REFUSE TO ARM below this MEASURED position rate on this endpoint")
    ap.add_argument("--link-retries", type=int, default=5,
                    help="ask -> measure -> ask AGAIN this many times before refusing (a 511 request sent "
                         "into the FCU's boot delivers 0 Hz forever; MEASURED, twice)")
    ap.add_argument("--nav-floor-hz", type=float, default=30.0,
                    help="abort when the bridge reports less than this estimator feed")
    ap.add_argument("--nav-age-max-s", type=float, default=1.5,
                    help="abort when the LIO sample the bridge is FEEDING the EKF is older than this.  The "
                         "rate alone cannot catch a dead LIO: the bridge republishes the last sample at "
                         "--nav-hz, so nav_hz stays at 47 Hz while every packet carries the same frozen "
                         "pose (2026-11-07 audit, finding 3).  1.5 s = 300 missed samples at 200 Hz: MEASURED "
                         "on `kv1-01-02` (a loaded host, rtf 0.93) the feed went 0.54 s stale and the "
                         "aircraft was still flying the mission -- 0.5 s, which is what this shipped with "
                         "first, aborted that sortie at waypoint 6.  A gate must fire on a DEAD feed, not on "
                         "a slow host")
    ap.add_argument("--nav-age-warn-s", type=float, default=0.5,
                    help="print a one-per-episode warning when the fed LIO sample is older than this (the "
                         "survivable band between 'slow host' and 'dead feed')")
    ap.add_argument("--truth-fcu-max-m", type=float, default=2.0,
                    help="abort when the FCU's own position estimate is further than this from the truth "
                         "(SIMULATION-ONLY gate: on a real aircraft there is no truth).  Delivered sorties "
                         "track at |fcu-truth| p50 0.16 m / p90 0.24 m; every fence escape tonight reached "
                         "5.0-15.3 m of disagreement but was only stopped at the 13 m fence, i.e. 3.4 m "
                         "outside the 10 m mission ring.  2 m is 8x the p90 and well inside the ring.")
    ap.add_argument("--truth-fcu-hold-s", type=float, default=2.0,
                    help="... and it must hold that long, so a single outlier sample cannot end a sortie")
    ap.add_argument("--nav-skip-max-per-s", type=float, default=10.0,
                    help="abort when the bridge is REFUSING external-nav frames faster than this.  The "
                         "bridge refuses a pose the aircraft could not physically reach (step speed > 5 m/s "
                         "or yaw rate > 900 deg/s) and counts it as nav_skip.  MEASURED over 32 sorties: a "
                         "clean sortie reports nav_skip=0 all flight; the two fence escapes of `kxB` had "
                         "1312 and 2174 refusals (21-28 % of a 47 Hz feed, i.e. 28-46 s of a divergent "
                         "estimator) and were only stopped by the fence itself.  10/s = 21 % of the feed.")
    ap.add_argument("--status-stale-s", type=float, default=2.0,
                    help="abort when nothing has arrived on /bridge/status for this long: a dead bridge "
                         "freezes nav_hz/nav_age at their last values, so silence must not read as health")
    ap.add_argument("--lio-wait-s", type=float, default=25.0)
    ap.add_argument("--plan-wait-s", type=float, default=20.0)
    ap.add_argument("--plan-silence-s", type=float, default=3.0,
                    help="A DEAD PLAN BOX IS AN ABORT, not a finished tour: the tour's end is a FIELD "
                         "(trajectory_flag = COMPLETED), so silence is an error.  0 disables")
    ap.add_argument("--phase-log", required=True)
    args = ap.parse_args(argv)
    GATES["geofence_m"] = args.geofence_m
    print("supervisor: endpoint %s (its OWN), phase log %s" % (args.mav, args.phase_log), flush=True)
    print("supervisor: gates -- geofence %.2f m on TRUTH, nav floor %.1f Hz, plan silence %.1f s, "
          "class 76 only (it never sends a setpoint)" % (args.geofence_m, args.nav_floor_hz,
                                                         args.plan_silence_s), flush=True)
    # WHICH TAKE-OFF CONTRACT IS FLOWN, IN THE RUN'S OWN WORDS.  Two mechanisms exist and they fly very
    # different profiles, so a reader must not have to infer which one from a number in the log.
    if abs(args.takeoff_command_m - args.alt) < 1e-9:
        print("supervisor: TAKE-OFF = FCU-TARGET: packet.z = %.3f m VERBATIM (= --alt); the FCU's own "
              "take-off controller flies the whole climb, the go goes out %s"
              % (args.takeoff_command_m,
                 "ON AIRBORNE (--trigger-on-airborne)" if args.trigger_on_airborne
                 else "AFTER THE HOLD (--trigger-after-hold)"), flush=True)
    else:
        print("supervisor: TAKE-OFF = SETPOINT-DRIVEN CLIMB (the owner's design): packet.z = %.3f m only "
              "('leave the ground'); the mission altitude %.3f m is the PLAN's, the go goes out %s, and the "
              "plan's first target must be a VERTICAL leg (rectangle_source.py --vertical-first-m)"
              % (args.takeoff_command_m, args.alt,
                 "ON AIRBORNE (--trigger-on-airborne)" if args.trigger_on_airborne
                 else "AFTER THE HOLD (--trigger-after-hold)"), flush=True)
    print("supervisor: the take-off's own guard -- abort if the TRUTH passes %.2f m (mission altitude "
          "%.2f + climb guard %.2f); nothing else bounds the flight before the trigger"
          % (args.alt + args.climb_guard_m, args.alt, args.climb_guard_m), flush=True)
    os.makedirs(os.path.dirname(os.path.abspath(args.phase_log)), exist_ok=True)
    return Supervisor(args).run()


if __name__ == "__main__":
    sys.exit(main())
