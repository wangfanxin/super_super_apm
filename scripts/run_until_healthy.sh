#!/usr/bin/env bash
# Fly the SUPER mission and judge it by the RIGHT evidence.
#
# Why this file was rewritten (all four were my defects, each one enough to kill the sortie):
#
#  1. GEOFENCE_M was 6.60.  supervisor.py:275 aborts on |truth x| or |truth y| > geofence_m, and the
#     mission is a 15 m square, i.e. |x|,|y| reach 7.5.  Flight kg-02 died on exactly this
#     ("ABORT: geofence: truth at [+2.46 +5.01]") and no future sortie could have survived it.
#
#  2. WORLD was worlds/pillars.yaml -- a 10 x 10 m arena with its mission square placed inside a
#     CLEAR RECTANGLE.  A 15 m square does not even fit in it, and it contains nothing to avoid.
#     Now: worlds/square15_forest.yaml, generated and audited by scripts/make_square15_world.py
#     (4 of 4 legs provably blocked, every corner reachable on the planner's own inflated grid).
#
#  3. The thin layer got NO --calib-file.  run-mission.sh only adds it to the *bridge*'s argv
#     (line 182); for a path planner it passes ${CMD_ARGS} verbatim (line 415).  So the bridge
#     converted L->W with the MEASURED (-0.8, -4.1, +115.6) mm while my planner used its PREDICTED
#     fallback (0, +50, +100) mm -- a 54 mm y error wired straight into every goal.  Measured in
#     missionA-sup-03: traj.log line 1 says "NO --calib-file -> ... predicted fallback" while the
#     supervisor prints "the bridge converts with ... t = [-0.0008 -0.0041 +0.1156]".
#
#  4. WP_SPD was 2.0 (phase_a_mission_fast.parm).  mode_guided.cpp's shaper takes its horizontal
#     speed limit from WP_SPD, so >= 4 m/s was IMPOSSIBLE no matter what SUPER planned.  Fixed with
#     EXTRA_PARM=params/guided_limits_fast.parm -- the arm that repository already flew and
#     measured (C2 +0.038/+0.016 m, 0.00 % of samples inside a body, docs/.../guided_limits_fast).
#     run-mission.sh appends EXTRA_PARM LAST in --defaults, and ArduPilot takes the last value.
#
# The health gate now asserts what actually has to be true for the numbers to mean anything:
# LIO alive, setpoints on the wire, SUPER's own reference present (not just my hold fallback),
# no supervisor abort, and the host gate (rtf 0.98-1.02).  A sortie that passes all five is gradeable.
set -u
PREFIX=${1:-h}
WHAT=${2:-super}
ARM=${3:-lio}
ATTEMPTS=${4:-4}
TAKEOFF_M=${TAKEOFF_M:-0.2}
CALIB=${DSH_ROOT}/calib/calib.json

# ---- PROFILE: which configuration to fly ------------------------------------------------
# `repro` exists because "the SUPER layer does not fly" and "this harness's flight
# configuration does not fly" were being confused with each other.  `missionA-supertest-01`
# is a COMPLETE, healthy sortie of this harness (take-off, hold, leg2..leg6, land; C2 min
# pillar distance 0.538 m) whose banner (/tmp/supertest01.log) records its exact arm:
#
#     config    : apm_lio_only,sitl_basics,baro_posz,phase_a_mission,firmware defaults
#     override  : none
#     world     : worlds/pillars.yaml          (2.0 x 1.2 m mission square)
#     geofence  : 5.00 m, take-off altitude 2.50 m
#     planner   : rectangle
#
# So `repro` copies ALL of that and changes exactly ONE thing: PLANNER.  If that flies, the
# thin layer is proven end to end with no other variable in play.  `mission` is the real
# target (15 m square x 3 laps, forest world, >= 4 m/s) and differs from `repro` in several
# variables at once -- which is why it has to come second.
PROFILE=${PROFILE:-mission}
# NOTE: ROOT/CODE/WS/SUP must exist BEFORE the profile block, because the mission profile names
# a parm file by absolute path inside this repo (and `set -u` makes an unbound $ROOT fatal:
# MEASURED, "line 74: ROOT: unbound variable", which is a clean abort but a wasted sortie slot).
CODE=${DSH_ROOT}/harness
ROOT=${DSH_ROOT}
WS=${LIO_WS}
SUP=$ROOT/src/whtf/ros/planner_source_super.py
case "$PROFILE" in
  repro)
    WORLD_NAME=pillars.yaml
    Z_M=2.5; SQUARE_M=2.0; LAPS=1; HOLD_S=0
    GEOFENCE_M=5.0
    MISSION_PARMS=params/phase_a_mission.parm     # WP_SPD 0.25 / WP_ACC 0.3 (the slow precision arm)
    EKF_PARMS=                                    # supertest-01: "firmware defaults", NO EKF parm
    EXTRA_PARMS=                                  # and no override
    ARM=baro
    ;;
  mission)
    WORLD_NAME=${WORLD_NAME:-square15_forest.yaml}
    Z_M=${Z_M:-2.0}; SQUARE_M=${SQUARE_M:-15}; LAPS=${LAPS:-3}; HOLD_S=${HOLD_S:--1}
    GEOFENCE_M=${GEOFENCE_M:-9.0}
    MISSION_PARMS=${MISSION_PARMS:-params/phase_a_mission_fast.parm}
    # ARRIVE_M IS THE WAYPOINT BOX RADIUS, and it is also what decides whether the aircraft STOPS
    # at a waypoint.  SUPER has a warm-started replan (`ReplanOnce` takes pos_init_state from the
    # committed trajectory, i.e. WITH its velocity, and only recedes `receding_dis` when a new goal
    # arrives), but the FSM is forced back to `PlanFromRest` -- which zeroes the initial velocity --
    # the moment `generateExpTraj` reports NEW_TRAJ, i.e. once the old trajectory has RUN OUT.
    # So if the pacer only declares arrival 0.25 m before the corner, the old trajectory is already
    # spent and the handover is necessarily rest2rest: measured on pe-01/pg-01 the speed drops to 0
    # at every corner (0 <-> 4.2 m/s with a ~5 s period).
    # Declaring arrival at the EDGE OF THE BOX (1.5 m) hands the next goal over while the aircraft
    # is still moving, so the replan is warm-started and it flies THROUGH the corner.
    ARRIVE_M=${ARRIVE_M:-1.5}
    MISSION_PARMS=${MISSION_PARMS:-params/phase_a_mission_fast.parm}
    # MY OWN flight-trust file, not the harness's calibration one: the calibration file assumes
    # the LIO is accurate to 3 cm, which turns a normal in-flight LIO transient into an
    # 'EKF variance' -> 'EKF Failsafe' (measured on missionA-na-02, see params/ekf_trust_lio_flight.parm)
    EKF_PARMS=${EKF_PARMS-$ROOT/params/ekf_trust_lio_flight.parm}
    EXTRA_PARMS=${EXTRA_PARMS-params/guided_limits_fast.parm}
    ;;
  *) echo "PROFILE must be repro or mission, not '$PROFILE'" >&2; exit 2 ;;
esac
TIMEOUT_S=${TIMEOUT_S:-300}

# THE TAP MUST NOT HAVE HOLES.  MEASURED: missionA-supertest-01 (the one flight that worked)
# recorded the tap at 1200 Hz with ZERO holes; missionA-ne-02 recorded it at 902 Hz with 13 holes
# up to 5.28 s, and 25.6 % of its 100 ms scan windows straddled one.  A window that straddles a
# hole cannot be cast (lidar_imu refuses it), so a quarter of that flight produced no usable scan,
# and the LIO's z diverged by 10.7 m.  truth_rx already asks for a big SO_RCVBUF, but the kernel
# caps it at net.core.rmem_max, which defaults to 4 MB here (~22 s of tap at 180 kB/s -- not
# enough for a long stall).  This raises the cap when sudo is available without a prompt, and
# says so; without sudo the sim still works, the buffer just stays smaller.
if command -v sudo >/dev/null 2>&1 && sudo -n true 2>/dev/null; then
  sudo -n sysctl -w net.core.rmem_max=33554432 >/dev/null 2>&1 && \
    echo "  tap buffer cap raised: net.core.rmem_max=32 MB"
else
  echo "  note: no passwordless sudo -- net.core.rmem_max stays at $(sysctl -n net.core.rmem_max 2>/dev/null)"
fi

# Port / lock isolation.  The harness is single-occupancy and another session flies the same
# binaries; these are the ONLY thing that keeps the two sorties apart (INTEGRATION.md section 14).
export TAP=14580 FAN=14581 PORT0=5780 PORT1=5781 PORT2=5782 PORT3=5783
export LOCK=/tmp/lio_avoid_sortie_super.lock

for i in $(seq 1 "$ATTEMPTS"); do
  TAG="$PREFIX-$(printf '%02d' "$i")"
  echo "=========== attempt $i/$ATTEMPTS  tag=$TAG  what=$WHAT  arm=$ARM  world=$WORLD_NAME  geofence=${GEOFENCE_M}m"
  if [ -f "$LOCK" ]; then
    p=$(cut -d: -f1 "$LOCK" 2>/dev/null || echo "")
    if [ -n "$p" ] && kill -0 "$p" 2>/dev/null; then echo "  ABORT: my own sortie is still flying ($p)"; exit 3; fi
    rm -f "$LOCK"
  fi
  bash "$ROOT/scripts/clean_harness.sh" | sed 's/^/  /'

  if [ "$ARM" = "lio" ]; then
    NAV=lio; ALT=lio
    # the planner MUST convert with the same file the bridge uses, or every goal is skewed
    CALIB_ARG="--calib-file $CALIB"
  else
    NAV=truth; ALT=baro; CALIB_ARG=""
  fi
  EKF=""
  if [ -n "${EKF_PARMS:-}" ]; then
    case "$EKF_PARMS" in
      /*) EKF="$EKF_PARMS" ;;                      # absolute: a file in MY repo
      *)  EKF="$CODE/$EKF_PARMS" ;;               # relative: a file in the harness's params/
    esac
    [ -f "$EKF" ] || { echo "  ABORT: no EKF parm at $EKF" >&2; exit 2; }
  fi

  if [ "$WHAT" = "super" ]; then
    PLANNER=$SUP
    # SUPER_CONFIG selects the planner's ceiling: `super_lio_15m.yaml` is the graded file
    # (max_vel 7 / max_acc 6.5); `super_lio_15m_gentle.yaml` differs ONLY in that ceiling
    # (max_vel 3 / max_acc 3) and exists to prove the 12-waypoint mission completes before the
    # speed is raised.  See scripts/make_super_gentle_config.sh.
    SUPER_CONFIG=${SUPER_CONFIG:-super_lio_15m.yaml}
    # `--vertical-first-m` MUST be here.  run-mission.sh builds $VERTICAL_ARG and appends it only to
    # the BUILT-IN rectangle planner's argv (line 381); for a path planner it passes ${CMD_ARGS}
    # verbatim (line 415).  Without it the banner says "NO vertical leg in custom's argv" and the
    # aircraft never gets the pad-above target the pacer's _arm needs (z > 0.5), so it never leaves
    # the ground: MEASURED, setpoints.csv 0 rows, "ABORT: never left the ground".
    CMD_ARGS="--super-config $SUPER_CONFIG --square-m $SQUARE_M --laps $LAPS --z $Z_M \
--tour ${TOUR:-square} \
--arrive-m $ARRIVE_M --hold-s $HOLD_S --first-hold-s 3.0 --leg-timeout-s 40 \
--settle-speed ${SETTLE_SPEED:-1.5} --settle-s ${SETTLE_S:-0.5} \
--vertical-first-m $Z_M --vertical-hold-s 3.0 \
--ref-max-speed-mps ${REF_MAX_MPS:-6.0} --ref-max-speed-z-mps ${REF_MAX_Z_MPS:-1.5} \
--ref-z-hold ${REF_Z_HOLD:-0} --ref-z-tol ${REF_Z_TOL:-0.20} --ref-z-latch-m ${REF_Z_LATCH_M:-1.60} \
--entry-max-speed-mps ${ENTRY_MAX_MPS:-0} \
--goal-lookahead-m ${GOAL_LOOKAHEAD_M:-0.0} \
--goal-lead ${GOAL_LEAD:-0} \
--pre-goal-m ${PRE_GOAL_M:-0} \
--poly-check ${POLY_CHECK:-0} \
--clr-gov-margin ${CLR_GOV_M:-0} \
--time-scale ${TIME_SCALE:-1.0} \
--pass-by-m ${PASS_BY_M:-0.0} \
--pass-by-long-m ${PASS_BY_LONG_M:-0.0} \
--goal-pva ${GOAL_PVA:-0} --corner-speed ${CORNER_SPEED:-2.0} \
--corner-mode ${CORNER_MODE:-outgoing} \
--corner-speed-list "${CORNER_SPEED_LIST:-${CORNER_SPEED:-2.0}}" \
--ref-no-retract ${REF_NO_RETRACT:-0} \
--ref-ff ${REF_FF:-0} \
--ref-gov-kp ${REF_GOV_KP:-0} \
--ref-gov-acc ${REF_GOV_ACC:-0} \
--ref-gov-maxlag ${REF_GOV_MAXLAG:-0} \
--ref-gov-along ${REF_GOV_ALONG:-0} \
--ref-gov-ks ${REF_GOV_KS:-4.0} \
--ref-gov-dec ${REF_GOV_DEC:-2.0} \
--ref-along-floor ${REF_ALONG_FLOOR:-0} \
--ref-catchup-mps ${REF_CATCHUP_MPS:-0.5} \
--ref-lead-s ${REF_LEAD_S:-0} \
--ref-lp-tau ${REF_LP_TAU:-0} \
--ref-ff-acc-max ${REF_FF_ACC_MAX:-0} \
--ref-lead-turn-rad-s ${REF_LEAD_TURN_RAD_S:-1.5} \
--ref-ff-gain ${REF_FF_GAIN:-1.0} \
--super-odom /lio/robo/odom --super-cloud /lio/cloud_world \
--world "$ROOT/worlds/${WORLD_NAME:-square15_forest.yaml}" $CALIB_ARG"
  else
    PLANNER=rectangle
    CMD_ARGS="$CALIB_ARG"
  fi

  EXTRA_ARG=""
  if [ -n "${EXTRA_PARMS:-}" ]; then
    case "$EXTRA_PARMS" in
      /*) EXTRA_ARG="$EXTRA_PARMS" ;;              # absolute: a file in MY repo
      *)  EXTRA_ARG="$CODE/$EXTRA_PARMS" ;;       # relative: a file in the harness's params/
    esac
    [ -f "$EXTRA_ARG" ] || { echo "  ABORT: no EXTRA parm at $EXTRA_ARG" >&2; exit 2; }
  fi

  ( cd "$CODE" || exit 1
    set +u; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"
    [ "$WHAT" = "super" ] && source ${WS_SUPER}/install/setup.bash
    set -u
    # ★★ THE STALE COPY OF THE LAUNCH COMMAND USED TO SIT HERE ★★
    #   Round 89's first fix added the guard and a corrected command BELOW this point but left the original
    #   backslash-continued line in place, with a `#` comment line inside the continuation.  Bash joined the
    #   lines, the comment swallowed the rest, so that copy ran as `timeout N env A=1 ... VERTICAL_HOLD_S=3.0`
    #   -- `env` with no command, which PRINTS THE WHOLE ENVIRONMENT on the console (the dump in every
    #   `log` of tags kxF..kxI) -- and started nothing.  The flight itself worked because the corrected
    #   command below also ran; the dump was the fossil.  Deleted, and the rule is in the block below.
    # CLIMB_GUARD: run-mission.sh forwards ${SUP_ARGS} to the supervisor.  The guard exists to
    # catch the FCU's take-off target running away (MEASURED once at 9.8 m on a 2.50 m command),
    # not to fail on a normal step response.  With the thin layer's vertical slew limiter the
    # overshoot on a 2.0 m climb is a transient; 2.5 m still catches the pathology by 4x.
    #   The raycast is the only part of the simulator that can overrun its 100 ms scan budget, and when
    #   it does the scan is SKIPPED, the cloud rate collapses, Super-LIO scan-matches across the gap,
    #   diverges (yaw steps up to 154 deg were measured) and the FCU follows the bad estimate out of the
    #   fence.  MEASURED: 20000 points -> cast p50 21-25 ms; 12000 points -> cast p50 11-14 ms, i.e. the
    #   budget margin doubles (2x the work per scan removed).  The estimator is not starved by it:
    #   `kxE` H5 span test PASS on every flight, W1b worst 0.11-0.16 m, and its best sortie
    #   (`kxE-01-02`) is the project's best acceptance yet (12 pass / 2 fail, W6 p90 0.332).
    #   It does NOT remove a host-scheduling stall (cast still hit 327 ms once) -- that needs CPU
    #   isolation, which is why §87 made the stall a validity gate (H6) as well.
    # ★ NO COMMENT MAY SIT INSIDE THE CONTINUATION BELOW ★
    #   MEASURED (round 89, tag `kxF`): a `#` line placed between two backslash-continued lines ENDS the
    #   command, so `timeout ... env ...` ran with no command of its own (printing the whole environment
    #   on the console) and `bash src/tools/run-mission.sh` ran as a SEPARATE command WITHOUT `PLANNER`,
    #   so the harness fell back to `PLANNER=ego` and started `ego_contract_shim.py`, which rejected the
    #   plan box's argv.  Six attempts died as "THE PLAN BOX IS NOT TALKING" with `super_ref=0`.
    #   `bash -n` does not catch it.  Keep this block comment BEFORE this line, never inside it.
    #
    # ★ A GUARD FOR THE kxF DEFECT ★
    #   When a stray comment truncated the launch command, the runner started with `PLANNER` UNSET, the
    #   harness fell back to `PLANNER=ego`, `ego_contract_shim.py` ran instead of the plan box, and six
    #   attempts burned 20 s each before the supervisor said "THE PLAN BOX IS NOT TALKING".
    #   These two lines make that state impossible to miss on the first attempt.
    [ -n "${PLANNER:-}" ] && [ -f "$PLANNER" ] || {
      echo "  ABORT: PLANNER='${PLANNER:-<unset>}' is not a file -- refusing to start." >&2
      echo "         (the harness would fall back to PLANNER=ego, run ego_contract_shim.py, and the sortie" >&2
      echo "          would die as 'THE PLAN BOX IS NOT TALKING'; see docs/INTEGRATION.md §89)" >&2
      exit 2
    }
    echo "  launch         : PLANNER=$PLANNER LIDAR_POINTS=${LIDAR_POINTS:-12000} tag=$TAG"
    timeout "$TIMEOUT_S" env NAV_SOURCE="$NAV" ALT_MODE="$ALT" PLANNER="$PLANNER" \
      TAP="$TAP" FAN="$FAN" PORT0="$PORT0" PORT1="$PORT1" PORT2="$PORT2" PORT3="$PORT3" LOCK="$LOCK" \
      WORLD="$ROOT/worlds/$WORLD_NAME" \
      MISSION_PARM="$CODE/$MISSION_PARMS" \
      ${EXTRA_ARG:+EXTRA_PARM="$EXTRA_ARG"} \
      ${EKF:+EKF_PARM="$EKF"} \
      GEOFENCE_M="$GEOFENCE_M" TAKEOFF_COMMAND_M="$TAKEOFF_M" VERTICAL_FIRST=1 VERTICAL_HOLD_S=3.0 \
      LIDAR_POINTS="${LIDAR_POINTS:-12000}" \
      SUP_ARGS="--climb-guard-m ${CLIMB_GUARD_M:-2.5}" \
      CMD_ARGS="$CMD_ARGS" \
      bash src/tools/run-mission.sh "$TAG" > "/tmp/$TAG.log" 2>&1 )
  echo "  RUN_EXIT=$?"

  D=${LIO_LOGS}/missionA-$TAG
  [ -d "$D" ] || D=${DSH_ROOT}/harness/logs/missionA-$TAG

  LIO=$(wc -l < "$D/lio.csv" 2>/dev/null || echo 0)
  SET=$(wc -l < "$D/setpoints.csv" 2>/dev/null || echo 0)
  REF=$(grep -o 'SUPER ref [0-9]*' "$D/traj.log" 2>/dev/null | awk '{print $3}' | sort -n | tail -1)
  REF=${REF:-0}
  AB=$(grep -h 'ABORT' "$D/supervisor.log" 2>/dev/null | tail -1 | cut -c1-150)
  RTF=$(grep -o 'rtf [0-9.]*' "$D/truth_report.txt" 2>/dev/null | head -1 | awk '{print $2}')
  RTF=${RTF:-0}
  # ★ the gate is on the FLIGHT window, not the whole tap window (round 89) ★
  #   The tap is started before the SITL begins stepping; its own report divides the whole window and
  #   therefore carries the startup transient.  MEASURED: four consecutive sorties read 0.9765-0.9792
  #   there while every flown phase measured 0.999-1.002, and a genuinely starved sortie
  #   (kwC-01-02, aborted at 37 s) reads 0.358 on this same measure -- so the gate still bites.
  RTFF=$(python3 "$ROOT/scripts/flight_rtf.py" "$D" 2>/dev/null)
  RTFF=${RTFF:-n/a}
  # ★ the LiDAR SIMULATOR's own budget (round 89, docs/INTEGRATION.md §87) ★
  #   A raycast that takes 200-570 ms (host contention) makes `src/sim/lidar_imu.py` SKIP that scan; the
  #   cloud gaps then make Super-LIO diverge (yaw steps up to 154 deg) and the FCU follows it out of the
  #   fence.  Every one of the 14 fence escapes measured tonight had cast_max >= 184 ms and skipped >= 70,
  #   while every clean flight had cast_max <= 60 ms and skipped <= 3 -- so this is a VALIDITY gate on the
  #   same footing as the rtf gate: a starved-sensor sortie's control numbers describe the starvation.
  LIDAR=$(python3 "$ROOT/scripts/lidar_stall.py" "$D" 2>/dev/null)
  LIDAR=${LIDAR:-n/a}
  WAYP=$(grep -c 'ARRIVED at waypoint' "$D/traj.log" 2>/dev/null || echo 0)

  echo "  health: lio=$LIO setpoints=$SET super_ref=$REF waypoints=$WAYP rtf=$RTF flight_rtf=$RTFF lidar[$LIDAR]"
  [ -n "$AB" ] && echo "  abort : $AB"
  bash "$ROOT/scripts/peek.sh" "$TAG" 2>/dev/null | grep -E 'max U|起飞|max \|v\|' | sed 's/^/  /'
  bash "$ROOT/scripts/clean_harness.sh" >/dev/null 2>&1

  OK=1
  [ "$LIO" -ge 1000 ] || { echo "  NO: the LIO delivered $LIO rows (<1000)"; OK=0; }
  [ "$SET" -ge 100 ] || { echo "  NO: only $SET setpoints reached the wire (<100)"; OK=0; }
  [ "$REF" -ge 1 ] || { echo "  NO: SUPER published NO reference -- everything on the wire was my hold fallback"; OK=0; }
  [ -z "$AB" ] || { echo "  NO: the supervisor aborted"; OK=0; }
  awk "BEGIN{exit !($RTFF>=0.98 && $RTFF<=1.02)}" || { echo "  NO: host gate flight_rtf=$RTFF (take-off..land) is outside 0.98-1.02, numbers not gradeable"; OK=0; }
  python3 "$ROOT/scripts/lidar_stall.py" "$D" >/dev/null 2>&1 \
    || { echo "  NO: the LiDAR SIMULATOR starved the sortie [$LIDAR] -- check was >100 ms/scan or >20 scans skipped, numbers not gradeable (see docs/INTEGRATION.md §87)"; OK=0; }
  if [ "$OK" = "1" ]; then
    echo "  ==> GRADEABLE: LIO alive, SUPER reference present, no abort, host kept real time. tag=$TAG"
    exit 0
  fi
  echo "  ==> retrying"
done
echo "== $ATTEMPTS attempts, none gradeable =="
exit 1
