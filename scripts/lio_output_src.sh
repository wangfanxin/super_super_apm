#!/usr/bin/env bash
# --- repo path bootstrap (round 21) ------------------------------------------
if [ -z "${DSH_ROOT:-}" ]; then
  _dsh="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  while [ "$_dsh" != "/" ] && [ ! -f "$_dsh/setup/env.sh" ]; do _dsh="$(dirname "$_dsh")"; done
  [ -f "$_dsh/setup/env.sh" ] && . "$_dsh/setup/env.sh"
fi
# -----------------------------------------------------------------------------
set -u
S=${DSH_ROOT}/slam-fork/super_lio_local/src/lio/super_lio.cpp
echo "== where Output() transforms a cloud"
grep -n 'Output\|transformPointCloud' "$S" | head -20
echo
echo "== the Output() body"
START=$(grep -n 'void SuperLIO::Output' "$S" | head -1 | cut -d: -f1)
[ -n "$START" ] && sed -n "${START},$((START+45))p" "$S"
