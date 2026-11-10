#!/usr/bin/env bash
# Bring the harness's own worlds into my repo so a PROFILE=repro sortie can name them by the
# same path every other world uses.  A copy, not a move: the reference worlds stay where the
# harness's own tools and the other session expect them.
set -eu
SRC=/mnt/d/distill_self/lio_avoid/worlds
DST=/mnt/d/distill_self/super_super_apm/worlds
for w in pillars.yaml pillars_fast.yaml forest6x4.yaml rect68.yaml forest_north.yaml; do
  [ -f "$SRC/$w" ] || { echo "missing in the harness: $w" >&2; continue; }
  cp -f "$SRC/$w" "$DST/$w"
  echo "copied $w  ($(grep -c '^  - {x:' "$DST/$w") pillars)"
done
echo
echo "== worlds now available to PROFILE=* =="
ls -1 "$DST"
