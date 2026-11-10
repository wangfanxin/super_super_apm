#!/usr/bin/env bash
# Give the FCU the velocity feed-forward (BRIDGE_FIELDS=posvel) AND make the reference a real trajectory
# first (--ref-gov-kp), which is the precondition nobody had.
#
# THE TWO FAILURES THAT LED HERE, both recorded:
#   pz-01  posvel + --ref-ff 1 : "ABORT: geofence: truth at [-7.49 -10.51]" at 65.4 s.  --ref-ff makes
#          the published velocity the limiter's slide rate, which switches between ~0 and ~5 m/s at
#          every clamp boundary; the FCU feeds that into an acceleration-limited reference model and a
#          discontinuous feed-forward there throws the vehicle out of the arena (the harness measured
#          the same thing on its ego arm).
#   qa-01  posvel + --ref-ff 0 : completed 13/13 but the TRUTH reached 7.12 m/s against a 5.00 m/s
#          reference, altitude 3.58 m, path 211 m (longer than the 184 m commanded).  Position and
#          velocity disagreed: the position was a per-frame slide, the velocity was the spline's own
#          value, so the reference model was driven by two inconsistent inputs.
#          ... but the position correspondence improved a lot: zero-lag p50 0.85 m vs 2.0 m before.
#
# So: keep the feed-forward (it is why the aircraft is near its reference at all) and fix the input it
# is fed with.  The governor integrates ONE velocity per tick and derives BOTH outputs from it, so
# (p, v) are consistent by construction and continuous by construction, and |v| is bounded.
set -u
ROOT=/mnt/d/distill_self/super_super_apm
cd "$ROOT" || exit 1
export SUPER_CONFIG=${SUPER_CONFIG:-super_lio_15m_fast.yaml}
export ARRIVE_M=${ARRIVE_M:-2.5}
export SETTLE_SPEED=${SETTLE_SPEED:-1.5} SETTLE_S=${SETTLE_S:-0.5}
export PASS_BY_M=${PASS_BY_M:-0.0} GOAL_LOOKAHEAD_M=${GOAL_LOOKAHEAD_M:-0.0}
export REF_MAX_MPS=${REF_MAX_MPS:-5.0} REF_MAX_Z_MPS=${REF_MAX_Z_MPS:-1.2}
export REF_NO_RETRACT=${REF_NO_RETRACT:-0} REF_FF=${REF_FF:-0}
export BRIDGE_FIELDS=${BRIDGE_FIELDS:-posvel}
export REF_GOV_KP=${REF_GOV_KP:-4.0}
export CLIMB_GUARD_M=${CLIMB_GUARD_M:-4.0} GEOFENCE_M=${GEOFENCE_M:-10.5}
exec bash "$ROOT/scripts/fly_speed_run.sh" "${1:-qb}" "${2:-3}"
