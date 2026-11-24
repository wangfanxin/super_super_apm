#!/usr/bin/env bash
# --- repo path bootstrap (round 21) ------------------------------------------
if [ -z "${DSH_ROOT:-}" ]; then
  _dsh="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  while [ "$_dsh" != "/" ] && [ ! -f "$_dsh/setup/env.sh" ]; do _dsh="$(dirname "$_dsh")"; done
  [ -f "$_dsh/setup/env.sh" ] && . "$_dsh/setup/env.sh"
fi
# -----------------------------------------------------------------------------
# Where does the goal advance happen, and on what condition?
set -u
cd ${DSH_ROOT} || exit 1
F=$(grep -rln 'class TourPacer' apm_thin/ | head -1)
echo "== TourPacer in: $F"
START=$(grep -n 'class TourPacer' "$F" | head -1 | cut -d: -f1)
END=$((START + 80))
sed -n "${START},${END}p" "$F"
