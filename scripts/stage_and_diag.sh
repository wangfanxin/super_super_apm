#!/usr/bin/env bash
# Stage a sortie and print the tracking decomposition (lateral vs along) for the FF comparison.
set -u
cd /mnt/d/distill_self/super_super_apm || exit 1
for T in "$@"; do
  echo "===================== $T"
  python3 scripts/prep_run.py --run "/home/fanxin/lio_avoid/logs/missionA-$T" --out "logs/$T" 2>&1 | tail -1
  python3 scripts/tracking_diag.py "$T" 2>&1 | grep -E 'plan stream|steps > 0.30|frozen below|clock shift|along-track p50|lateral   p50|vertical  p50'
done
