#!/usr/bin/env bash
# --- repo path bootstrap (git clone anywhere) --------------------------------
if [ -z "${DSH_ROOT:-}" ]; then
  _dsh="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  while [ "$_dsh" != "/" ] && [ ! -f "$_dsh/setup/env.sh" ]; do _dsh="$(dirname "$_dsh")"; done
  [ -f "$_dsh/setup/env.sh" ] && . "$_dsh/setup/env.sh"
fi
# -----------------------------------------------------------------------------
# record_bag.sh -- Phase 2 recording base: ONE rosbag2 for the whole sortie.
#
# Replaces (in the target state) the per-topic CSV recorders.  The CSV files stay available as a DERIVED
# layer (scripts/bag_to_csv.py), so the analysis and acceptance code does not change when the base does.
#
#   bash scripts/record_bag.sh <run-dir>
#
# Writes <run-dir>/bag/ and <run-dir>/bag.log.  Stops on SIGINT/SIGTERM (ros2 bag flushes on SIGINT).
set -u
R="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT=${1:?usage: record_bag.sh <run-dir>}
[ -f "$R/setup/env.sh" ] && . "$R/setup/env.sh"
set +u
[ -f /opt/ros/humble/setup.bash ] && . /opt/ros/humble/setup.bash
[ -f "${LIO_WS:-}/install/setup.bash" ] && . "$LIO_WS/install/setup.bash"
[ -f "${WS_SUPER:-}/install/setup.bash" ] && . "$WS_SUPER/install/setup.bash"
set -u

# TWO TOPIC SETS.  `core` is what scripts/bag_to_csv.py needs for the CSV layer and is small enough to run
# while the CSV recorders are still on (MEASURED 2026-11-25: the full set plus 4 CSV recorders on 4 shared
# cores starved the LiDAR simulator -- cast 106 ms instead of 10 -- and the flight aborted).  `full` is the
# target state, when the CSVs are derived and the per-topic recorders are gone.
if [ "${BAG_TOPICS:-core}" = "full" ]; then
  TOPICS=(
    /lio/robo/odom /lio/odom /lio/cloud_world
    /livox/lidar /livox/imu
    /plan/cmd /planning/pos_cmd /planning_cmd/poly_traj
    /sim/truth/odom /clock /sim/rtf
    /fcu/odom /rosout
  )
else
  TOPICS=(/lio/robo/odom /plan/cmd /sim/truth/odom /clock /fcu/odom)
fi
# NOTE: do NOT create the bag directory yourself -- `ros2 bag record -o DIR` refuses to start when DIR
# already exists (MEASURED: "[ERROR] [ros2bag]: Output folder ... already exists"), so the parent only.
mkdir -p "$OUT"
echo "record_bag: ${#TOPICS[@]} topics -> $OUT/bag"
exec ros2 bag record -o "$OUT/bag" \
  --qos-profile-overrides-path "$R/scripts/bag_qos.yaml" \
  --max-bag-size 2147483648 \
  "${TOPICS[@]}"
# COMPRESSION IS OFF ON PURPOSE: rosbag2_py's SequentialReader in Humble fails with
# "file is not a database" on a zstd-compressed .db3 (measured 2026-11-25), while `ros2 bag info` reads it.
# The Phase 2 derivation uses rosbag2_py, so the base recorder writes plain sqlite until the reader path is
# proven; scripts/record_bag.sh can then gain `--compression-mode file` behind a flag.
