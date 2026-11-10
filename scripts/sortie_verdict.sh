#!/usr/bin/env bash
# Did the flight complete, and what are its per-lap numbers?
set -u
cd /mnt/d/distill_self/super_super_apm || exit 1
T=${1:?tag}
R=/home/fanxin/lio_avoid/logs/missionA-$T
echo "=== $T"
echo "ABORT lines : $(grep -c ABORT "$R/supervisor.log" 2>/dev/null)"
grep -E 'phase ->' "$R/supervisor.log" 2>/dev/null | tail -3
echo "waypoints   : $(grep -o 'waypoints=[0-9]*' "$R/supervisor.log" 2>/dev/null | tail -1)"
echo
echo "=== per-lap numbers (re-render, text only) ==="
python3 scripts/plot_hex_laps.py --tag "$T" --laps 2 --world worlds/forest_hex12.yaml \
        --out /tmp/evidence_stage --prefix lap 2>&1 | grep -E '^  lap|^lap starts'
