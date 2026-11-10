#!/usr/bin/env bash
# --- release-tree path bootstrap (git clone anywhere) -----------------------
if [ -z "${DSH_ROOT:-}" ]; then
  _dsh="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  while [ "$_dsh" != "/" ] && [ ! -f "$_dsh/setup/env.sh" ]; do _dsh="$(dirname "$_dsh")"; done
  [ -f "$_dsh/setup/env.sh" ] && . "$_dsh/setup/env.sh"
fi
# -----------------------------------------------------------------------------
# One command to reproduce the whole evaluation for a sortie.
#
#   run_accept.sh                 # the newest missionA-* sortie
#   run_accept.sh sf-02           # by tag
#   run_accept.sh sf-02 square15_forest.yaml 2.0
#
# Prints the acceptance rows (scripts/accept.py) and archives them as JSON next to the run,
# so the numbers a report quotes can always be regenerated from the recorded files.
set -u
ROOT=${DSH_ROOT}
LOGS=${LIO_LOGS}

TAG=${1:-}
WORLD=${2:-square15_forest.yaml}
ALT=${3:-2.0}
# HALF: the mission square's half extent, when the world's square is not the 15 m default (a 14.4 m
# square puts the corners at +-7.2, and every W1/W1b tolerance is scaled off that number).
# BODY: W4's body model -- `circle` (the old rule) or `box` (the v2 trunk below z_split).
HALF=${4:-}
BODY=${5:-circle}
# BOX: the W1b "was the waypoint actually approached" tolerance.  1.5 m is the legacy square-mission
# default; SPEC S1 (round 81) defines the arrival as a 0.5 m SPHERE, so a hexagon run is scored at 0.5
# unless the caller says otherwise.
WBOX=${6:-}
# LAPS: how many laps THIS TOUR flies.  accept.py has always taken `--laps`; the runner never passed
# it, so the scorer kept assuming the world file's own count (3) and would have scored a 2-lap flight
# against 19 targets (18 corners + pad) -- 6 corners the aircraft never flies.  ROUND 87: the arm's
# LAPS is now forwarded as the 7th argument.  The mission LENGTH is declared by the arm; no tolerance
# in accept.py is touched.
LAPS=${7:-}
# KNOWN: rows the user has accepted as KNOWN DEFECTS for this delivery (round 20: W4).  Passed through
# to accept.py's --known-defect; it does NOT move any threshold, it only lets the gate line name the
# defect instead of pretending it is absent.  Example: KNOWN="W4" bash scripts/run_accept.sh ...
KNOWN=${KNOWN:-}
EXTRA=""
[ -n "$HALF" ] && EXTRA="$EXTRA --square-half $HALF"
[ -n "$WBOX" ] && EXTRA="$EXTRA --waypoint-box $WBOX"
[ -n "$LAPS" ] && EXTRA="$EXTRA --laps $LAPS"
EXTRA="$EXTRA --body $BODY"
if [ -n "$KNOWN" ]; then
  for k in $KNOWN; do EXTRA="$EXTRA --known-defect $k"; done
fi

if [ -n "$TAG" ]; then
  RUN="$LOGS/missionA-$TAG"
else
  RUN=$(ls -dt "$LOGS"/missionA-* 2>/dev/null | head -1)
  TAG=$(basename "$RUN" | sed 's/^missionA-//')
fi
[ -d "$RUN" ] || { echo "no such run: $RUN" >&2; exit 2; }

echo "tag   : $TAG"
echo "run   : $RUN"
echo "world : $ROOT/worlds/$WORLD"
echo "alt   : $ALT m"
echo "half  : ${HALF:-15 m default}   body: $BODY"
echo

python3 "$ROOT/scripts/accept.py" --run "$RUN" --world "$ROOT/worlds/$WORLD" --alt "$ALT" \
    $EXTRA --json "$RUN/accept.json"
RC=$?

echo
echo "== how to regenerate the sortie itself =="
echo "  cd $ROOT && PROFILE=mission bash scripts/run_until_healthy.sh sup super lio 3"
echo "  (needs a QUIET host: the gate is rtf 0.98-1.02 on the truth tap, and the flight stack"
echo "   and the LiDAR simulator compete for the same cores -- see docs/INTEGRATION.md 17.7)"
exit $RC
