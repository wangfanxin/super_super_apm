#!/usr/bin/env bash
# --- repo path bootstrap (round 21) ------------------------------------------
if [ -z "${DSH_ROOT:-}" ]; then
  _dsh="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  while [ "$_dsh" != "/" ] && [ ! -f "$_dsh/setup/env.sh" ]; do _dsh="$(dirname "$_dsh")"; done
  [ -f "$_dsh/setup/env.sh" ] && . "$_dsh/setup/env.sh"
fi
# -----------------------------------------------------------------------------
# Bring the harness's own worlds into my repo so a PROFILE=repro sortie can name them by the
# same path every other world uses.  A copy, not a move: the reference worlds stay where the
# harness's own tools and the other session expect them.
set -eu
SRC=${DSH_ROOT}/harness/worlds
DST=${DSH_ROOT}/worlds
for w in pillars.yaml pillars_fast.yaml forest6x4.yaml rect68.yaml forest_north.yaml; do
  [ -f "$SRC/$w" ] || { echo "missing in the harness: $w" >&2; continue; }
  cp -f "$SRC/$w" "$DST/$w"
  echo "copied $w  ($(grep -c '^  - {x:' "$DST/$w") pillars)"
done
echo
echo "== worlds now available to PROFILE=* =="
ls -1 "$DST"
