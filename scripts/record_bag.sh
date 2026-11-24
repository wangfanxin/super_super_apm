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
OUT=${1:?usage: record_bag.sh <run-dir> [bag-parent]}
BAGPARENT=${2:-$OUT}          # e.g. /dev/shm/sp_bags/<tag>: tmpfs, see finalize_bag.sh for why
[ -f "$R/setup/env.sh" ] && . "$R/setup/env.sh"
# FAST START WHEN THE CALLER ALREADY HAS THE ROS ENVIRONMENT: re-sourcing two workspaces costs ~2-4 s, and
# that delay is exactly the recording window the bag loses at the start of a sortie (the derived layer then
# cannot match the old CSV recorders row-for-row).  The launcher sources everything before starting us.
if [ -z "${ROS_VERSION:-}" ] || [ "${BAG_FORCE_SOURCE:-0}" = "1" ]; then
    set +u
    [ -f /opt/ros/humble/setup.bash ] && . /opt/ros/humble/setup.bash
    [ -f "${LIO_WS:-}/install/setup.bash" ] && . "$LIO_WS/install/setup.bash"
    [ -f "${WS_SUPER:-}/install/setup.bash" ] && . "$WS_SUPER/install/setup.bash"
    set -u
fi

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
# SPLIT WRITERS (MEASURED 2026-11-25): a single rosbag2 recorder on this WSL/ext4 box sustains only
# ~350-450 messages/s end-to-end; the flight's core topics offer ~600/s (200 Hz odometry + 4 x 100 Hz),
# so a single recorder captured ~35 Hz per 200 Hz topic.  Two recorders with DISJOINT topic lists halve
# the load per writer and both stay under the cap.
if [ "${BAG_SPLIT:-0}" = "1" ]; then
  # BALANCED BY RATE, NOT BY TOPIC COUNT: /lio/robo/odom is 200 Hz alone, so it gets its own writer;
  # the three 100 Hz topics share the other.  /clock is dropped from the bag on purpose -- D4's time base
  # is the truth client's own stamps, and /clock adds 100 msg/s for a derivable quantity.
  case "${BAG_GROUP:-a}" in
    a) TOPICS=(/lio/robo/odom) ;;
    b) TOPICS=(/sim/truth/odom /plan/cmd /fcu/odom) ;;
    *) echo "BAG_GROUP must be a or b" >&2; exit 2 ;;
  esac
fi
# NOTE: do NOT create the bag directory yourself -- `ros2 bag record -o DIR` refuses to start when DIR
# already exists (MEASURED: "[ERROR] [ros2bag]: Output folder ... already exists"), so the parent only.
if [ "${BAG_SPLIT:-0}" = "1" ]; then BAGDIR="$BAGPARENT/bag_${BAG_GROUP:-a}"; else BAGDIR="$BAGPARENT/bag"; fi
mkdir -p "$BAGPARENT"
echo "record_bag: ${#TOPICS[@]} topics -> $BAGDIR ($(df -h "$BAGPARENT" | tail -1 | awk '{print $4}') free)"
exec ros2 bag record -o "$BAGDIR" \
  --qos-profile-overrides-path "$R/scripts/bag_qos.yaml" \
  --max-bag-size 2147483648 \
  "${TOPICS[@]}"
# COMPRESSION IS OFF ON PURPOSE: rosbag2_py's SequentialReader in Humble fails with
# "file is not a database" on a zstd-compressed .db3 (measured 2026-11-25), while `ros2 bag info` reads it.
# The Phase 2 derivation uses rosbag2_py, so the base recorder writes plain sqlite until the reader path is
# proven; scripts/record_bag.sh can then gain `--compression-mode file` behind a flag.
