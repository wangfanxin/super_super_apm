#!/usr/bin/env bash
# Compare the lidar sim's measured scan rate between sorties -- the one hardware-adjacent number
# (the lidar is 10 Hz, imu 200 Hz: "雷达10hz，imu200hz，这是硬件一样的不要乱改") that could explain why
# the estimator copes at 2 m/s and not at 3 m/s.
set -u
D=/home/fanxin/lio_avoid/logs
for t in "$@"; do
  echo "== $t"
  f="$D/missionA-$t/lidar_imu.log"
  [ -f "$f" ] || { echo "   (no lidar_imu.log)"; continue; }
  # the per-20-scans progress lines carry the running cloud/imu rates
  grep "cloud" "$f" | tail -3 | sed 's/^/   /'
  # and the final summary block
  tail -14 "$f" | grep -e "scans" -e "imu" -e "raycast" -e "skipped" -e "GATE" | sed 's/^/   /'
done
