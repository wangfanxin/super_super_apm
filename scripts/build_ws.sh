#!/usr/bin/env bash
# 全量构建自己的工作区。
#
# 关键：livox_ros_driver2 的 CMakeLists 引用了一个**从未被设置**的变量 DISTRO_ROS，
# 所以永远走 else 分支，用 get_target_property 去取一个在 generate 阶段还没生成的
# 目标属性 -> CMake Error: LIVOX_INTERFACES_INCLUDE_DIRECTORIES ... NOTFOUND。
# 它的 humble/jazzy 分支用的是正确的 rosidl_get_typesupport_target。
# 因此这里只需要 -DDISTRO_ROS=humble，不需要改它的源码。
set -euo pipefail

WS=/home/fanxin/super_super_apm/ws
LOG="${1:-/tmp/whtf_build.log}"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

cd "$WS"
echo "== colcon build (log: $LOG) =="
colcon build --symlink-install --parallel-workers 10 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DDISTRO_ROS=humble \
  > "$LOG" 2>&1 || true

echo
echo "== summary =="
grep -E 'Summary:|Failed|Aborted|not processed' "$LOG" | tail -15 || echo "  (no summary)"

echo
echo "== installed =="
ls "$WS/install" 2>/dev/null | grep -v -E '^(COLCON_IGNORE|local_setup|setup\.|_local_setup)' | sed 's/^/  /'
