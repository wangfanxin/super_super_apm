#!/usr/bin/env bash
# 编译 SUPER（ROS 2 Humble）。
#
# SUPER 是 catkin + ament 双制式：每个包都有 ros1.* / ros2.* 模板，
# 由 scripts/select_ros_version.sh 把对应的一份拷成 CMakeLists.txt / package.xml。
# 所以先切到 ROS2，再在独立工作区里 colcon build。
set -uo pipefail

R=/home/fanxin/super_super_apm/upstream/SUPER
WS=/home/fanxin/super_super_apm/ws_super
LOG=${1:-/tmp/super_build.log}

echo "== select ROS2 templates =="
cd "$R"
bash scripts/select_ros_version.sh ROS2 2>&1 | tail -12 || {
  echo "select_ros_version failed; falling back to manual copy"; }

echo
echo "== verify templates applied =="
for p in super_planner rog_map mission_planner; do
  f="$R/$p/package.xml"
  [ -f "$f" ] || { echo "  $p: NO package.xml"; continue; }
  printf '  %-18s format=%s  ' "$p" "$(grep -o 'format="[0-9]"' "$f" | head -1)"
  if grep -q ament_cmake "$f"; then echo AMENT; elif grep -q catkin "$f"; then echo CATKIN; else echo "?"; fi
done

echo
echo "== workspace =="
mkdir -p "$WS/src"
for p in super_planner rog_map mission_planner; do
  ln -sfn "$R/$p" "$WS/src/$p"
done
ln -sfn "$R/mars_uav_sim/mars_quadrotor_msgs" "$WS/src/mars_quadrotor_msgs"
ls "$WS/src"

echo
echo "== colcon build (log $LOG) =="
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
cd "$WS"
colcon build --symlink-install --parallel-workers 6 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DDISTRO_ROS=humble > "$LOG" 2>&1 || true

echo
echo "== summary =="
grep -E 'Summary:|Failed|Aborted|not processed|error:' "$LOG" | tail -20 || echo "(no summary)"
echo
echo "== installed =="
ls "$WS/install" 2>/dev/null | sed 's/^/  /'
