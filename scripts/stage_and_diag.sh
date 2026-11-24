#!/usr/bin/env bash
# --- repo path bootstrap (round 21) ------------------------------------------
if [ -z "${DSH_ROOT:-}" ]; then
  _dsh="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  while [ "$_dsh" != "/" ] && [ ! -f "$_dsh/setup/env.sh" ]; do _dsh="$(dirname "$_dsh")"; done
  [ -f "$_dsh/setup/env.sh" ] && . "$_dsh/setup/env.sh"
fi
# -----------------------------------------------------------------------------
# Stage a sortie and print the tracking decomposition (lateral vs along) for the FF comparison.
set -u
cd ${DSH_ROOT} || exit 1
for T in "$@"; do
  echo "===================== $T"
  python3 scripts/prep_run.py --run "${LIO_LOGS}/missionA-$T" --out "logs/$T" 2>&1 | tail -1
  python3 scripts/tracking_diag.py "$T" 2>&1 | grep -E 'plan stream|steps > 0.30|frozen below|clock shift|along-track p50|lateral   p50|vertical  p50'
done
