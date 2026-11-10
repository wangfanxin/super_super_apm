#!/usr/bin/env bash
# sim_rates.sh -- the lidar sim's measured rate/cost per sortie (the estimator's food supply).
#
#   bash scripts/sim_rates.sh hx4-01-01 oldctl2-01 v3h132-01-01
set -u
D=/home/fanxin/lio_avoid/logs
printf '%-14s %10s %10s %12s %10s\n' sortie cloud_hz cast_ms scans skipped
for t in "$@"; do
  f="$D/missionA-$t/lidar_imu.log"
  if [ ! -f "$f" ]; then printf '%-14s %10s\n' "$t" "(no log)"; continue; fi
  hz=$(grep -o 'cloud [0-9.]* Hz' "$f" | tail -1 | awk '{print $2}')
  cast=$(grep -o 'raycast *[0-9.]* ms/scan' "$f" | tail -1 | awk '{print $2}')
  sc=$(grep -o 'scans *[0-9]* in' "$f" | tail -1 | awk '{print $2}')
  sk=$(grep -o 'skipped scans *[0-9]*' "$f" | tail -1 | awk '{print $3}')
  printf '%-14s %10s %10s %12s %10s\n' "$t" "${hz:--}" "${cast:--}" "${sc:--}" "${sk:--}"
done
