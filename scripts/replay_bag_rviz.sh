#!/usr/bin/env bash
# replay_bag_rviz.sh -- PHASE 4: one command to SEE a sortie.
#
#   bash scripts/replay_bag_rviz.sh .runs/missionA-phase4h-01-01            # 1x
#   bash scripts/replay_bag_rviz.sh .runs/missionA-phase4h-01-01 --rate 0.5 # slow motion
#   bash scripts/replay_bag_rviz.sh .runs/missionA-phase4h-01-01 --no-rviz  # headless (bag + adapters only)
#
# What it does:
#   1. publishes the static TF that the recording needs: `world_w -> lio_world`, read from the same
#      calib/calib.json the flight used (p_W = R_WL p_L + t_WL).  Without it RViz cannot put the truth
#      (world_w) and the LIO/plan (lio_world) in one picture;
#   2. runs scripts/viz_bridge.py, which turns odometry into Paths and /plan/cmd into a Pose/Path;
#   3. plays every bag directory the sortie has (bag/ or bag_a/ + bag_b/), if any, and opens RViz with
#      sim/rviz/pipeline.rviz.  The bag's clock is whatever was recorded; RViz's time panel is left in
#      "wall" mode so a bag without /clock still animates.
#
# NOT part of the automated acceptance chain: this is the human debug entry point.
set -uo pipefail
R="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$R" || exit 1
RUN=${1:?usage: replay_bag_rviz.sh <sortie-dir> [--rate R] [--no-rviz] [--loop]}
shift || true
RATE=1.0
NO_RVIZ=0
LOOP=""
while [ $# -gt 0 ]; do
    case "$1" in
        --rate) RATE=${2:?--rate needs a value}; shift 2 ;;
        --no-rviz) NO_RVIZ=1; shift ;;
        --loop) LOOP="--loop"; shift ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done
case "$RUN" in
    /*) ;;
    *) RUN="$R/$RUN" ;;
esac
[ -d "$RUN" ] || { echo "no such sortie dir: $RUN" >&2; exit 2; }

. setup/env.sh
set +u
. /opt/ros/humble/setup.bash
. "$LIO_WS/install/setup.bash" 2>/dev/null
. "$WS_SUPER/install/setup.bash" 2>/dev/null
set -u

BAGS=()
for d in "$RUN/bag" "$RUN/bag_a" "$RUN/bag_b"; do
    [ -f "$d/metadata.yaml" ] && BAGS+=("$d")
done
if [ "${#BAGS[@]}" -eq 0 ]; then
    echo "replay_bag_rviz: no bag in $RUN (Phase 2 records bag/ or bag_a+bag_b; BAG=1 on the flight)" >&2
    exit 2
fi

CALIB=${CALIB_FILE:-$R/calib/calib.json}
PIDS=()
cleanup() {
    for p in "${PIDS[@]:-}"; do [ -n "$p" ] && kill "$p" 2>/dev/null; done
    return 0
}
trap cleanup EXIT

if [ -f "$CALIB" ]; then
    TF_LINE=$(python3 - "$CALIB" <<'PY'
import json, math, sys
d = json.load(open(sys.argv[1]))
R = d["R_WL"]; t = d["t_WL"]
yaw = math.atan2(R[1][0], R[0][0])
print("%f %f %f 0 0 %f 1 world_w lio_world" % (t[0], t[1], t[2], math.sin(yaw / 2.0), math.cos(yaw / 2.0)))
PY
)
    # yaw-only quaternion: our calibrations are a z-rotation + translation, and printing the full
    # quaternion from a matrix here would be three more lines for no extra truth.
    set -- $TF_LINE
    ros2 run tf2_ros static_transform_publisher "$1" "$2" "$3" "$4" "$5" "$6" "$7" "$8" "$9" \
        > "$RUN/rviz_tf.log" 2>&1 &
    PIDS+=($!)
else
    echo "replay_bag_rviz: no $CALIB -- the lio_world cloud/TF will be missing in RViz" >&2
fi

python3 "$R/scripts/viz_bridge.py" > "$RUN/rviz_viz_bridge.log" 2>&1 &
PIDS+=($!)

for d in "${BAGS[@]}"; do
    ros2 bag play "$d" --rate "$RATE" $LOOP > "$RUN/rviz_bag_play_$(basename "$d").log" 2>&1 &
    PIDS+=($!)
    echo "replay_bag_rviz: playing $d at ${RATE}x"
done

sleep 2
if [ "$NO_RVIZ" = "0" ]; then
    if [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
        echo "replay_bag_rviz: no DISPLAY/WAYLAND_DISPLAY; RViz not started (WSLg or an X server needed)."
        echo "                 the bag + adapters keep running; Ctrl-C to stop."
    else
        exec ros2 run rviz2 rviz2 -d "$SIM_ROOT/rviz/pipeline.rviz"
    fi
fi
wait
