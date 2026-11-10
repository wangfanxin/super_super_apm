#!/usr/bin/env bash
# One sortie, then the three measurements that answer "is it faster, and did it touch anything".
#
# The speed question has to be answered on the REFERENCE (setpoints.csv vel_*), not on the truth,
# because the mean ground speed is set by what the plan asks for; the collision question has to be
# answered on the TRUTH (tap.csv vs the world file), because the plan is collision-free by
# construction and only the flown path can hit a pillar.
#
#   fly_speed_run.sh TAG [ATTEMPTS]
#
# Everything the flight depends on is printed BEFORE it starts, so a later reader can tell which
# arm produced the log:
#   * SUPER config file + the four numbers that decide the plan's shape
#   * the plan box's argv (arrive / settle / pass-by / rate limits)
#   * the FCU's WP_SPD, read back from the parm file that is appended LAST
set -u
ROOT=${DSH_ROOT}
TAG=${1:?tag, e.g. pm}
ATT=${2:-3}

# ---- the arm --------------------------------------------------------------------------------
export PROFILE=mission
export SUPER_CONFIG=${SUPER_CONFIG:-super_lio_15m.yaml}
export WORLD_NAME=${WORLD_NAME:-square15_forest.yaml}
export ARRIVE_M=${ARRIVE_M:-1.5}
export HOLD_S=${HOLD_S:--1}
# ACCEPTANCE ARM (default).  This is the arm missionA-pc-02 / pg-01 / ph-01 / pi-01 flew to
# GATE: PASS: the waypoint is declared reached on the radius test after the reference has SETTLED to
# 1.5 m/s (or 0.5 s inside the box), no pass-by, no retract clamp.  It is slower (2.17-2.25 m/s mean)
# than the speed arm below, and it is the arm the gate accepts.
export SETTLE_SPEED=${SETTLE_SPEED:-1.5}
export SETTLE_S=${SETTLE_S:-0.5}
export PASS_BY_M=${PASS_BY_M:-0.0}
# PASS_BY_LONG_M: the pass-by test's ALONG-TRACK bound.  0 = the old half-plane (which declared an
# arrival 13.8 m from the waypoint on cp-01); >0 turns the test into a box.
export PASS_BY_LONG_M=${PASS_BY_LONG_M:-0.0}
export GOAL_LOOKAHEAD_M=${GOAL_LOOKAHEAD_M:-0.0}
# GOAL_LEAD: 把发给 SUPER 的 goal 提前 k 个航点，让当前角点成为轨迹的**内部点**（0=老行为）。
export GOAL_LEAD=${GOAL_LEAD:-0}
# PRE_GOAL_M: send the NEXT corner's goal this many metres before the current one, so the
# current corner becomes an interior point of the trajectory (no stop there).  0 = off.
export PRE_GOAL_M=${PRE_GOAL_M:-0}
# POLY_CHECK=1: evaluate SUPER's own polynomial at the current time and compare it with the
# 100 Hz reference (no behaviour change; validates the layout/clock for --time-scale).
export POLY_CHECK=${POLY_CHECK:-0}
# CLR_GOV_M: clearance governor in the thin layer -- v <= (c - CLR_GOV_M)/0.154 where c is the
# reference's minimum pillar clearance ahead.  Only slows down, never changes the path.
export CLR_GOV_M=${CLR_GOV_M:-0}
# TIME_SCALE>1: replay SUPER's own polynomial k times faster (same path, compressed time).
export TIME_SCALE=${TIME_SCALE:-1.0}
# GOAL_PVA / CORNER_SPEED: PVA goals -- the waypoint stays the goal, but the trajectory ends moving
# along the outgoing leg at CORNER_SPEED instead of at rest (needs patch_goal_pva.py + goal_pva_en).
export GOAL_PVA=${GOAL_PVA:-0}
export CORNER_SPEED=${CORNER_SPEED:-2.0}
# REF_MAX_MPS is a RATE limit on the command, which makes it a real speed cap.  6.0 is the accepted
# value; the plan never asked for more than ~5 in the acceptance regime, so it does not bind there.
export REF_MAX_MPS=${REF_MAX_MPS:-6.0}
# The reference must never move BACKWARD: when SUPER replans, `PlanFromRest` starts the new trajectory
# at the vehicle's measured pose (super_planner.cpp:409-410), which trails the commanded point by
# v*latency, so the commanded point jumps back by that much on every replan.  0 here because the
# acceptance arm settles at the box, so the jump is small -- measured on the SPEED arm (pq-01) it cut
# the commanded path from 215 m to 187 m and lifted the mean from 2.33 to 2.49 m/s.
export REF_NO_RETRACT=${REF_NO_RETRACT:-0}
# ★★ BRIDGE_FIELDS: DEFAULT `pos`, AND THAT IS A MEASURED DECISION, NOT AN OVERSIGHT ★★
# `run-mission.sh:462-467` defaults this to `posvelacc` for PLANNER=ego and to `pos` for everything
# else, and a custom planner path takes the `else` -- so every sortie of this repo fed ArduPilot
# POSITION ONLY (`wire.csv`: `fields=pos, type_mask=2552`) while the thin layer published a full
# (p, v, a) command.  Round 46 tried to fix that and the fix FAILED FOUR TIMES:
#
#   pz-01  posvel + --ref-ff 1 : "ABORT: geofence: truth at [-7.49 -10.51]" at 65.4 s
#   qa-01  posvel + --ref-ff 0 : 13/13 waypoints, but the TRUTH reached 7.12 m/s against a 5.00 m/s
#                                reference, altitude 3.58 m, path 211 m vs 184 m commanded
#   qb-01  posvel + governor   : "ABORT: geofence: truth at [-7.00 -10.52]" at 43.9 s
#   qb-02  posvel + governor   : "ABORT: geofence: truth at [+10.50 +7.16]" at 74.2 s
#
# The mechanism is documented by the harness itself (lio_avoid/docs/ARCHITECTURE.md:507-512 and
# run-mission.sh's own comments): ArduPilot's `posvel` path does not fly the position, it feeds
# (p, v) into an acceleration-limited reference model, and a feed-forward that is not continuous
# there throws the vehicle out of the arena -- which is why the harness's ego arm needed a bounding
# shim for it (`EGO_REF_ACC_MAX`), not a flag.  Our reference IS a per-frame-limited slide, so its
# (p, v) cannot be made continuous enough by any of the three variants tried.
# The FLAG IS KEPT because the finding is worth reproducing: `BRIDGE_FIELDS=posvel bash
# scripts/fly_governor.sh <tag>` re-runs the last of them.
export BRIDGE_FIELDS=${BRIDGE_FIELDS:-pos}
export REF_GOV_KP=${REF_GOV_KP:-0.0}
export REF_GOV_ALONG=${REF_GOV_ALONG:-0}
export REF_MAX_Z_MPS=${REF_MAX_Z_MPS:-1.2}
# REF_Z_HOLD>0: after the vehicle is up (--ref-z-latch-m), the reference's z is clamped to
# REF_Z_HOLD +- REF_Z_TOL.  A level mission does not need a 3D reference: MEASURED on v2d-01 the
# reference z swung 0.50..3.50 m in a level square, which both broke W3 and dragged the lateral
# tracking with it (cmd-vs-truth p50 1.66 m, and the truth ended up in the pillar rows).
export REF_Z_HOLD=${REF_Z_HOLD:-0.0}
export REF_Z_TOL=${REF_Z_TOL:-0.20}
export REF_Z_LATCH_M=${REF_Z_LATCH_M:-1.60}
export CLIMB_GUARD_M=${CLIMB_GUARD_M:-4.0}
export EXTRA_PARMS=${EXTRA_PARMS:-$ROOT/params/guided_limits_flight.parm}
export EKF_PARMS=${EKF_PARMS-$ROOT/params/ekf_trust_lio_flight.parm}
# GEOFENCE 10.5, not 9.0: docs/INTEGRATION.md row 11 of the milestone table -- the pillars sit ON the
# ±7.5 legs, the legal avoidance needs ~0.9 m of lateral room, and the planner will route up to ~1.6 m
# outside the waypoint box to pass a pillar.  The arena walls are at ±11.  9.0 aborted pn-01/pn-02 at
# truth x +9.00 while the COMMAND box itself was at x 9.07 -- i.e. it was following a legal plan.
export GEOFENCE_M=${GEOFENCE_M:-10.5}

echo "==================== arm for this sortie ===================="
echo "  tag              : $TAG"
echo "  world            : $WORLD_NAME"
echo "  SUPER config     : $SUPER_CONFIG"
for d in ${WS_SUPER}/src/super_planner/config; do
  python3 "$ROOT/scripts/patch_yaml.py" "$d/$SUPER_CONFIG" --dump 2>/dev/null | grep -E \
    '^(super_planner\.(hot_start_en|receding_dis|planning_horizon|robot_r|yaw_mode)|traj_opt\.boundary\.max_(vel|acc|omg))' \
    | sed 's/^/    /'
done
echo "  plan box         : arrive ${ARRIVE_M}  settle ${SETTLE_SPEED} m/s / ${SETTLE_S} s  pass-by ${PASS_BY_M}  lookahead ${GOAL_LOOKAHEAD_M}"
echo "  corner speed     : a 0 km/h stop is replaced by a handover at ${ARRIVE_M} m, i.e. v ~ sqrt(2*max_acc*${ARRIVE_M})"
echo "  reference limits : ${REF_MAX_MPS} m/s horizontal, ${REF_MAX_Z_MPS} m/s vertical"
echo "  FCU arm          : $EXTRA_PARMS"
grep -E '^(WP_SPD|WP_ACC|ATC_ANGLE_MAX|PSC_NE_JERK)\b' "$EXTRA_PARMS" 2>/dev/null | sed 's/^/    /'
echo

# ---- the flight ------------------------------------------------------------------------------
START_EPOCH=$(date +%s)
bash "$ROOT/scripts/run_until_healthy.sh" "$TAG" super lio "$ATT"
RC=$?
echo
echo "==================== the three measurements ===================="
# THE TAG NAMESPACE IS SHARED WITH OTHER SESSIONS.  MEASURED: a stale `missionA-rp-02` from another
# session (Oct 25) was picked as "the newest" for a round-4 flight and got scored instead of the flight
# that had just been graded, which reported "the vehicle never took off".  So the directory has to be
# CREATED BY THIS RUN: mtime at or after the moment the runner started.
NEW=""
for d in ${LIO_LOGS}/missionA-"$TAG"-*; do
  [ -d "$d" ] || continue
  # a directory touched by this run has a phase.csv or tap.csv written after START_EPOCH
  if [ -n "$(find "$d" -maxdepth 1 -name '*.csv' -newermt "@$START_EPOCH" 2>/dev/null | head -1)" ]; then
    NEW="$d"
  fi
done
if [ -z "$NEW" ]; then
  echo "no sortie directory for tag $TAG was created by this run (stale directories ignored)"
  exit $RC
fi
T=$(basename "$NEW" | sed 's/^missionA-//')
echo
echo "---- 1) speed, on the REFERENCE"
python3 "$ROOT/scripts/ref_speed_audit.py" "$NEW" --bin 6 2>/dev/null | head -24
echo
echo "---- 2) speed, on the TRUTH"
python3 "$ROOT/scripts/speed_audit.py" "$NEW" 2>/dev/null | head -14
echo
echo "---- 3) clearance + the whole acceptance gate"
SQUARE_HALF=${SQUARE_HALF:-$(python3 -c "print(${SQUARE_M:-15}/2.0)")}
bash "$ROOT/scripts/run_accept.sh" "$T" "$WORLD_NAME" "${Z_M:-2.0}" "$SQUARE_HALF" "${BODY:-circle}" \
  "${WBOX:-}" "${LAPS:-}" \
  >/tmp/accept-$T.log 2>&1
python3 "$ROOT/scripts/show_gate.py" "$NEW" --full W4 2>/dev/null | sed 's/^/  /'
echo
echo "(full rows: sed -n '1,200p' /tmp/accept-$T.log)"
exit $RC
