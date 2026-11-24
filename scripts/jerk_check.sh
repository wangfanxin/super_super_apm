#!/usr/bin/env bash
# --- repo path bootstrap (round 21) ------------------------------------------
if [ -z "${DSH_ROOT:-}" ]; then
  _dsh="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  while [ "$_dsh" != "/" ] && [ ! -f "$_dsh/setup/env.sh" ]; do _dsh="$(dirname "$_dsh")"; done
  [ -f "$_dsh/setup/env.sh" ] && . "$_dsh/setup/env.sh"
fi
# -----------------------------------------------------------------------------
# Is a_dot bounded anywhere in the chain, and what does it demand from the body?
set -u
echo "== 1) SUPER: jerk is a bound, but is the penalty ON?"
U=${SUPER_PLANNER_ROOT}
grep -n 'weightJer > 0' "$U/super_planner/src/traj_opt/exp_traj_optimizer_s4.cpp" | head -2
grep -n 'weightOmg > 0\|weightAccThr > 0\|weightVel > 0\|weightAcc > 0\|weightPos > 0' \
  "$U/super_planner/src/traj_opt/exp_traj_optimizer_s4.cpp" | head -6
echo "   our weights:"
grep -nE 'penna_(pos|vel|acc|jerk|omg|thr):' "${DSH_ROOT}/config/super_planner_hex.yaml" | head -8
echo
echo "== 2) the hard-reject gates in the optimizer"
grep -n -B2 -A8 'violation\|max_pena(POS_IDX)\|reject' "$U/super_planner/src/traj_opt/exp_traj_optimizer_s4.cpp" | sed -n '1,24p'
echo
echo "== 3) ArduPilot rate limits (the body's angular-rate ceiling)"
AP=${ARDUPILOT_ROOT}
grep -rn 'ATC_RATE_RLL_MAX\|RATE_RLL_MAX' "$AP/ArduCopter/Parameters.cpp" 2>/dev/null | head -4
grep -rn 'ATC_RATE_RLL_MAX' "$AP/libraries/AP_Motors/AP_MotorsMulticopter.cpp" 2>/dev/null | head -3
