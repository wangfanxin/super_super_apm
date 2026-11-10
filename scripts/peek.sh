#!/usr/bin/env bash
# 看任意一轮的健康度：sensor 链、LIO、bridge、supervisor 的关键行。
# 用法： bash peek.sh <tag>
set -u
T=${1:?tag}
D=/home/fanxin/lio_avoid/logs/missionA-$T
[ -d "$D" ] || D=/mnt/d/distill_self/lio_avoid/logs/missionA-$T
echo "== run dir: $D =="
[ -d "$D" ] || { echo MISSING; exit 2; }
echo "-- 行数 --"
for f in tap.csv lio.csv setpoints.csv lidar_imu.log lio.log truth_rx.log supervisor.log bridge.log; do
  printf '   %-16s %s\n' "$f" "$(wc -l < "$D/$f" 2>/dev/null || echo MISSING)"
done
echo "-- lidar_imu 尾 --"
tail -4 "$D/lidar_imu.log" 2>/dev/null | sed 's/^/   /'
echo "-- lio 尾 --"
tail -3 "$D/lio.log" 2>/dev/null | sed 's/^/   /'
echo "-- bridge final --"
grep -h 'final:' "$D/bridge.log" 2>/dev/null | tail -1 | sed 's/^/   /'
echo "-- supervisor 关键 --"
grep -hE 'airborne: truth|ABORT|THE GO' "$D/supervisor.log" 2>/dev/null | tail -3 | sed 's/^/   /'
