#!/usr/bin/env bash
# --- repo path bootstrap (round 21) ------------------------------------------
if [ -z "${DSH_ROOT:-}" ]; then
  _dsh="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  while [ "$_dsh" != "/" ] && [ ! -f "$_dsh/setup/env.sh" ]; do _dsh="$(dirname "$_dsh")"; done
  [ -f "$_dsh/setup/env.sh" ] && . "$_dsh/setup/env.sh"
fi
# -----------------------------------------------------------------------------
# 起 RViz 看这套栈的内部状态（复用的 lio_avoid 的看板思路，话题换成 SUPER 的）。
#
# 需要图形显示：WSL2 + WSLg 直接可用；老环境要自己起 X server 并 export DISPLAY。
# 本仓库的 SITL 验收是**无头**跑的（scripts/run_and_score.sh），RViz 只用于人工看，
# 所以它不属于自动化链路，也不会影响验收数字。
#
# 用法：
#   bash scripts/run_rviz.sh          # 只看
#   bash scripts/run_rviz.sh -d ...   # 换配置文件
set -uo pipefail
ROOT=${DSH_ROOT}
# PHASE 4: the single replay/debug entry point.  For a recorded sortie prefer
#   bash scripts/replay_bag_rviz.sh <sortie-dir>      (bag + TF + adapters + this config)
# The live view below shows the same topics when the stack is running.
CFG=${1:-"$SIM_ROOT/rviz/pipeline.rviz"}

if [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
  echo "没有 DISPLAY/WAYLAND_DISPLAY：WSL 里先用 WSLg，或在 Windows 侧起 X server 后"
  echo "  export DISPLAY=<windows-host>:0"
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source ${DSH_ROOT}/ws/install/setup.bash
source ${WS_SUPER}/install/setup.bash
set -u

echo "== 当前话题（先确认 SITL 那套在跑）=="
ros2 topic list 2>/dev/null | grep -E "cloud_registered|rog_map|visualization|lidar_slam|plan" | head -20

echo "== rviz2 -d $CFG =="
exec ros2 run rviz2 rviz2 -d "$CFG"
