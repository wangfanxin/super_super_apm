#!/usr/bin/env bash
# =============================================================================
#  setup/run_hex.sh -- fly the 2-lap hexagon mission and grade it.
#
#  One command, three phases (each one is also usable on its own):
#     1. static gates: frozen map, SPEC self-check, patches present        (scripts/check_static.sh)
#     2. N sorties                                                        (scripts/fly_hex2.sh <tag>)
#     3. accept.py per sortie                                             (scripts/run_accept.sh)
#
#  Prerequisites: setup/build.sh has produced fsm_node and super_lio, and ArduPilot SITL exists at
#  $ARDUPILOT_ROOT (see setup/fetch_ardupilot.sh).
#
#    bash setup/run_hex.sh [TAG] [SORTIES] [ATTEMPTS]
#    e.g. bash setup/run_hex.sh kx7 2 2
#
#  Everything is overridable from the environment (see setup/env.sh); the most useful ones:
#    ARDUPILOT_ROOT  path to an existing ArduPilot tree      (default .build/ardupilot)
#    LIO_LOGS        where the sortie recordings go          (default .runs)
#    RVIZ=1          start rviz with the harness config
# =============================================================================
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
. "$HERE/env.sh"

TAG="${1:-hex}"
N="${2:-2}"
ATT="${3:-2}"

cd "$DSH_ROOT" || exit 1
dsh_env_summary
echo

echo "== 1) static gates"
bash scripts/check_static.sh || { echo "STATIC GATE FAILED -- not flying" >&2; exit 6; }

echo
echo "== 2) $N sortie(s), tag prefix $TAG"
mkdir -p "$LIO_LOGS"
SORTIES="$N" bash scripts/fly_hex2.sh "$TAG" "$ATT"

echo
echo "== 3) grading"
for d in "$LIO_LOGS"/missionA-"$TAG"-*; do
  [ -d "$d" ] || continue
  t=$(basename "$d" | sed 's/^missionA-//')
  echo "---- $t"
  KNOWN="${KNOWN:-W4}" bash scripts/run_accept.sh "$t" forest_hex12.yaml 2.0 10.0 box 0.5 2 \
    2>&1 | grep -E '^\[|RESULT|GATE' | tail -20
done

echo
echo "recordings: $LIO_LOGS"
