#!/usr/bin/env bash
# --- repo path bootstrap (round 21) ------------------------------------------
if [ -z "${DSH_ROOT:-}" ]; then
  _dsh="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  while [ "$_dsh" != "/" ] && [ ! -f "$_dsh/setup/env.sh" ]; do _dsh="$(dirname "$_dsh")"; done
  [ -f "$_dsh/setup/env.sh" ] && . "$_dsh/setup/env.sh"
fi
# -----------------------------------------------------------------------------
# Did the flight complete, and what are its per-lap numbers?
set -u
cd ${DSH_ROOT} || exit 1
T=${1:?tag}
R=${LIO_LOGS}/missionA-$T
echo "=== $T"
echo "ABORT lines : $(grep -c ABORT "$R/supervisor.log" 2>/dev/null)"
grep -E 'phase ->' "$R/supervisor.log" 2>/dev/null | tail -3
echo "waypoints   : $(grep -o 'waypoints=[0-9]*' "$R/supervisor.log" 2>/dev/null | tail -1)"
echo
echo "=== per-lap numbers (re-render, text only) ==="
python3 scripts/plot_hex_laps.py --tag "$T" --laps 2 --world worlds/forest_hex12.yaml \
        --out /tmp/evidence_stage --prefix lap 2>&1 | grep -E '^  lap|^lap starts'
