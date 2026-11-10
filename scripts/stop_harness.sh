#!/usr/bin/env bash
set -u
pkill -9 -x arducopter 2>/dev/null
pkill -9 -x fsm_node 2>/dev/null
pkill -9 -x super_lio_node 2>/dev/null
pkill -9 -f 'lidar_imu.py' 2>/dev/null
pkill -9 -f 'fly_hex2.sh' 2>/dev/null
pkill -9 -f 'fly_speed_run.sh' 2>/dev/null
pkill -9 -f 'run_until_healthy.sh' 2>/dev/null
pkill -9 -f 'planner_source_super' 2>/dev/null
sleep 2
rm -f /tmp/lio_avoid_sortie_super.lock
echo "processes left:"
ps -eo comm | grep -E 'arducopter|fsm_node|super_lio|lidar' | grep -v grep || echo "   none"
echo -n "lock: "; test -e /tmp/lio_avoid_sortie_super.lock && echo present || echo free
