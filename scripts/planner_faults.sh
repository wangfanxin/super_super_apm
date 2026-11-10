#!/usr/bin/env bash
# planner_faults.sh -- one line per sortie: which fault dominates SUPER's log?
#   occupied  "Local start point is deeply occupied"      (robot inside the inflated map)
#   belowgnd  "Odom below virtual ground"                 (odom z under the map's floor)
#   pfr       "PlanFromRest failed"
#   minco     "Minco exp traj opt failed"
#
#   bash scripts/planner_faults.sh hx1-01-01 hx3-01-01 oldctl-01
set -u
D=/home/fanxin/lio_avoid/logs
printf '%-14s %9s %9s %9s %9s\n' sortie occupied belowgnd pfr minco
for t in "$@"; do
  f="$D/missionA-$t/traj.log"
  if [ ! -f "$f" ]; then printf '%-14s %9s\n' "$t" "(no log)"; continue; fi
  a=$(grep -c 'deeply occupied' "$f" || true)
  b=$(grep -c 'below virtual ground' "$f" || true)
  c=$(grep -c 'PlanFromRest failed' "$f" || true)
  d=$(grep -c 'Minco exp_traj opt failed' "$f" || true)
  printf '%-14s %9s %9s %9s %9s\n' "$t" "${a:-0}" "${b:-0}" "${c:-0}" "${d:-0}"
done
