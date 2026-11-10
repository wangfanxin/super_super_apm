#!/usr/bin/env bash
# 单独验证 config/rviz_super.rviz 能不能被 rviz2 正常加载（不需要飞行栈）。
# 起 12 s、截一张图、退出。日志里有 error/failed 就是配置有问题。
set -uo pipefail
ROOT=/mnt/d/distill_self/super_super_apm
LOG=/tmp/rviz_standalone.log
SHOT=${1:-/tmp/rviz_standalone.png}

set +u
source /opt/ros/humble/setup.bash
set -u

echo "== 起 rviz2（12 s）=="
ros2 run rviz2 rviz2 -d "$ROOT/config/rviz_super.rviz" > "$LOG" 2>&1 &
RVIZ=$!
sleep 12

echo "== 窗口 =="
command -v xdotool >/dev/null 2>&1 && xdotool search --name rviz 2>/dev/null | head -3
scrot -o "$SHOT" 2>/dev/null && echo "  screenshot -> $SHOT ($(stat -c%s "$SHOT" 2>/dev/null) B)" || echo "  scrot failed"

kill $RVIZ 2>/dev/null
sleep 1
pkill -f 'rviz2' 2>/dev/null

echo
echo "== 日志里有没有错误 =="
grep -inE "error|fail|critical|does not exist|no plugin|invalid" "$LOG" | grep -viE "no error" | head -15 || echo "  (没有 error)"
echo
echo "== 日志头 =="
head -12 "$LOG"
