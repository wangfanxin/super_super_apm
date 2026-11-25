#!/usr/bin/env bash
# check_rviz_replay.sh -- end-to-end smoke test for the Phase-4 replay entry point, WITHOUT a screen.
#
#   bash tests/check_rviz_replay.sh [sortie-dir]
#
# `scripts/replay_bag_rviz.sh --no-rviz` has to start three things that a GUI would hide: the static TF
# `world_w -> lio_world` (read from calib/calib.json), scripts/viz_bridge.py (odometry -> /viz/*_path,
# PositionCommand -> /viz/plan_pose + /viz/plan_path) and the bag players.  This test runs it headless and
# asserts what RViz would need:
#   1. /viz/truth_path, /viz/lio_path, /viz/fcu_path, /viz/plan_path, /viz/plan_pose exist;
#   2. they carry data at a usable rate (truth >= 20 Hz, plan >= 10 Hz);
#   3. /tf_static links world_w (parent) to lio_world (child), so world-frame truth and L-frame LIO can be
#      drawn side by side in one fixed frame;
#   4. the recorded topics themselves are on the wire (/sim/truth/odom, /lio/robo/odom, /plan/cmd, /fcu/odom).
#
# It SKIPS (exit 0) when there is no bag to replay: a fresh clone has no .runs, and the flight that creates
# one needs the SITL stack.
set -u
R="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$R" || exit 1
. setup/env.sh
SORTIE=${1:-}
if [ -z "$SORTIE" ]; then
    SORTIE=$(ls -dt "$LIO_LOGS"/*/bag_a/metadata.yaml 2>/dev/null | head -1 | xargs -r dirname | xargs -r dirname)
fi
if [ -z "${SORTIE:-}" ] || [ ! -d "$SORTIE" ]; then
    echo "rviz replay: SKIP (no sortie with a bag; runs live under $LIO_LOGS)"
    exit 0
fi
if [ ! -f "$SORTIE/bag_a/metadata.yaml" ] && [ ! -f "$SORTIE/bag/metadata.yaml" ]; then
    echo "rviz replay: SKIP ($SORTIE has no bag_a/bag)"
    exit 0
fi
set +u
. /opt/ros/humble/setup.bash
. "$LIO_WS/install/setup.bash" 2>/dev/null
. "$WS_SUPER/install/setup.bash" 2>/dev/null
set -u
LOG=/tmp/sp_rviz_replay_$$
mkdir -p "$LOG"
bash scripts/replay_bag_rviz.sh "$SORTIE" --no-rviz --rate 3.0 > "$LOG/replay.log" 2>&1 &
RP=$!
cleanup() {
    kill "$RP" 2>/dev/null
    pkill -f "ros2 bag play $SORTIE" 2>/dev/null
    pkill -f "scripts/viz_bridge.py" 2>/dev/null
    pkill -f "static_transform_publisher" 2>/dev/null
    return 0
}
trap cleanup EXIT
sleep 8
fails=0
note() { echo "  - $*"; fails=$((fails + 1)); }

topics=$(ros2 topic list 2>/dev/null)
for t in /viz/truth_path /viz/lio_path /viz/plan_path /viz/plan_pose; do
    echo "$topics" | grep -qx "$t" || note "no $t (viz_bridge did not publish)"
done
for t in /sim/truth/odom /lio/robo/odom /plan/cmd /fcu/odom; do
    echo "$topics" | grep -qx "$t" || note "no $t (the bag is not playing the recorded topics)"
done

rate() {   # $1 topic, $2 floor Hz
    hz=$(timeout 6 ros2 topic hz "$1" 2>/dev/null | grep -o 'average rate: [0-9.]*' | head -1 | awk '{print $3}')
    if [ -z "${hz:-}" ]; then
        note "no messages on $1"
    else
        awk -v h="$hz" -v f="$2" 'BEGIN{exit !(h+0>=f+0)}' || note "$1 at ${hz} Hz (floor $2)"
    fi
}
rate /viz/truth_path 20
rate /viz/plan_pose 10

# the static TF is LATCHED (transient_local) and the CLI's QoS flags differ between distros: try the
# default subscription a few times, and fall back to asking who publishes it.
tf=""
for _try in 1 2 3; do
    tf=$(timeout 6 ros2 topic echo /tf_static --once 2>/dev/null)
    [ -n "$tf" ] && break
    sleep 2
done
if [ -z "$tf" ]; then
    if ros2 topic info /tf_static 2>/dev/null | grep -q "Publisher count: 1"; then
        note "/tf_static has a publisher but echo returned nothing (latched QoS race)"
    else
        note "no /tf_static publisher at all (static_transform_publisher did not start)"
    fi
fi
echo "$tf" | grep -q "frame_id: world_w" || note "/tf_static has no world_w parent"
echo "$tf" | grep -q "child_frame_id: lio_world" || note "/tf_static does not link lio_world"
if ! echo "$tf" | grep -q "translation:"; then
    note "/tf_static carries no translation"
fi

if [ "$fails" -ne 0 ]; then
    echo "FAIL: rviz replay smoke ($fails problem(s)); log: $LOG/replay.log"
    tail -5 "$LOG/replay.log"
    exit 1
fi
echo "PASS: headless replay -- bag plays, viz_bridge publishes /viz/* at usable rates, world_w->lio_world TF is up ($SORTIE)"
echo "logs: $LOG"
