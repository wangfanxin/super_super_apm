#!/usr/bin/env bash
# --- repo path bootstrap (round 21) ------------------------------------------
if [ -z "${DSH_ROOT:-}" ]; then
  _dsh="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  while [ "$_dsh" != "/" ] && [ ! -f "$_dsh/setup/env.sh" ]; do _dsh="$(dirname "$_dsh")"; done
  [ -f "$_dsh/setup/env.sh" ] && . "$_dsh/setup/env.sh"
fi
# -----------------------------------------------------------------------------
# Compare the lidar sim's measured scan rate between sorties -- the one hardware-adjacent number
# (the lidar is 10 Hz, imu 200 Hz: "雷达10hz，imu200hz，这是硬件一样的不要乱改") that could explain why
# the estimator copes at 2 m/s and not at 3 m/s.
set -u
D=${LIO_LOGS}
for t in "$@"; do
  echo "== $t"
  f="$D/missionA-$t/lidar_imu.log"
  [ -f "$f" ] || { echo "   (no lidar_imu.log)"; continue; }
  # the per-20-scans progress lines carry the running cloud/imu rates
  grep "cloud" "$f" | tail -3 | sed 's/^/   /'
  # and the final summary block
  tail -14 "$f" | grep -e "scans" -e "imu" -e "raycast" -e "skipped" -e "GATE" | sed 's/^/   /'
done
