#!/usr/bin/env bash
# Every figure the docs name must exist, and the figures the owner asked to delete must NOT be back.
#
# Round 83 (owner): keep ONLY (a) the per-lap path/trajectory top views, (b) v_xy/v_z/x/y/z against time,
# (c) the LIO error against time.  Everything else is archived outside the repo, and every later round
# that produced a new set deleted the previous one ("旧图记得一定要删除").
#
# ROUND 88 (workspace clean + new map): the map was redesigned (worlds/forest_hex12.yaml), the flight
# figures of the OLD map were removed on purpose, and the new campaign then landed its three families on
# the new map -- so this file now REQUIRES them:
#     lap1-paths.png / lap2-paths.png   the per-lap path-vs-trajectory top views (owner's round-83 ask)
#     vxyz-t.png                        v_xy / v_z / x / y / z against time
#     lio-err-t.png                     the estimator's error against time + its CDF
# plus the map review figure and this README.  Nothing else belongs in docs/evidence.
set -u
E=/mnt/d/distill_self/super_super_apm/docs/evidence
REQUIRED="README.md world-hex12.png lap1-paths.png lap2-paths.png vxyz-t.png lio-err-t.png"
LEGACY_PATTERNS='pv1-* d84-* d85-* r15b-* v4a-01-* i1a-* v3pv-* v3d-* v3h132-* cmp1-* cmp2-* frontier.png deviation.png pipeline-states.png *-lap1-paths.png *-lap2-paths.png *-vxyz-t.png *-lio-err-t.png mx8-* pm-01-* pgf-* hx*-* hy0-* hz*-* ja1-01-* jb1-* jc1-*'

echo "== evidence directory"
ls -1 "$E" | sort | sed 's/^/  /'
echo
echo "== required now"
MISS=0
for f in $REQUIRED; do
  if [ -f "$E/$f" ]; then
    printf '  OK    %-28s %9s bytes\n' "$f" "$(stat -c%s "$E/$f")"
  else
    printf '  MISS  %s\n' "$f"
    MISS=1
  fi
done
echo
echo "== legacy figures that must stay deleted"
LEG=0
for pat in $LEGACY_PATTERNS; do
  for f in "$E"/$pat; do
    [ -e "$f" ] || continue
    printf '  PRESENT (should be gone) %s\n' "$(basename "$f")"
    LEG=1
  done
done
[ "$LEG" -eq 0 ] && echo "  none of the legacy figure families are present"
echo
if [ "$MISS" -ne 0 ] || [ "$LEG" -ne 0 ]; then
  echo "== FAIL"
  exit 1
fi
echo "== OK: the evidence directory matches the documented state"
