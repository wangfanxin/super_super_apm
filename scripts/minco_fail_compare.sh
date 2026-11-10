#!/usr/bin/env bash
# How often did SUPER's own optimizer fail on a sortie?  That single number separates "the plan is
# infeasible here" from "the aircraft cannot track it": the reference only freezes when MINCO keeps
# rejecting, and a frozen reference makes W1b unwinnable whatever the controller does.
#
#   bash scripts/minco_fail_compare.sh mx8-01-01 v3pv-02-01 v3s-01-01
set -u
D=/home/fanxin/lio_avoid/logs
printf '%-12s %10s %10s %10s %10s\n' tag MINCO_fail PlanFromRest polytope lines
for t in "$@"; do
  f="$D/missionA-$t/traj.log"
  if [ ! -f "$f" ]; then printf '%-12s %10s\n' "$t" "(no traj.log)"; continue; fi
  m=$(grep -c "TrajOpt failed" "$f" || true)
  p=$(grep -c "PlanFromRest failed" "$f" || true)
  g=$(grep -c "GeneratePolytopeFromLine failed" "$f" || true)
  n=$(wc -l < "$f")
  printf '%-12s %10s %10s %10s %10s\n' "$t" "$m" "$p" "$g" "$n"
done
