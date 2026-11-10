#!/usr/bin/env bash
# =============================================================================
#  setup/fetch_ardupilot.sh -- get the ArduPilot SITL tree this project was flown against.
#
#  WHY this is not in the repo: the checkout is ~1.5 GB and the build output is another few hundred MB;
#  GitHub rejects big files and nobody wants that in a diff.  The commit below is the one the recorded
#  flights used (see third_party/pins.env).
#
#  It also patches nothing by itself: the truth-tap the harness reads is a small build-time patch that
#  this project's flights used; if you start from a clean upstream ArduPilot you will need that patch to
#  get tap.csv.  Run `harness/firmware/README.md` for what it is (kept in the harness history), or use
#  your own SITL build and pass TAP=0 to run-mission via the flight scripts.
#
#    bash setup/fetch_ardupilot.sh            # clone + checkout pinned commit (no build)
#    bash setup/fetch_ardupilot.sh --build    # ... and build the SITL binary
# =============================================================================
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
. "$HERE/env.sh"

REPO="${ARDUPILOT_REPO:-https://github.com/ArduPilot/ardupilot}"
COMMIT="${ARDUPILOT_COMMIT:-dbe792162d06cab66c3475fd5556bf7a120f119e}"
DO_BUILD=0
[ "${1:-}" = "--build" ] && DO_BUILD=1

echo "==================== ArduPilot SITL"
echo "  repo   : $REPO"
echo "  commit : $COMMIT"
echo "  target : $ARDUPILOT_ROOT"

if [ ! -d "$ARDUPILOT_ROOT/.git" ]; then
  mkdir -p "$(dirname "$ARDUPILOT_ROOT")"
  echo "== cloning (this takes a few minutes)"
  git clone --filter=blob:none "$REPO" "$ARDUPILOT_ROOT"
fi

echo "== checking out $COMMIT"
git -C "$ARDUPILOT_ROOT" fetch --filter=blob:none origin "$COMMIT" 2>/dev/null || git -C "$ARDUPILOT_ROOT" fetch --all
git -C "$ARDUPILOT_ROOT" checkout -q "$COMMIT"

echo "== submodules"
git -C "$ARDUPILOT_ROOT" submodule update --init --recursive --depth 1 2>&1 | tail -3

if [ "$DO_BUILD" -eq 1 ]; then
  echo "== building SITL (copter)"
  ( cd "$ARDUPILOT_ROOT" && ./waf configure --board sitl >/tmp/ap_conf.log 2>&1 && ./waf copter 2>&1 | tail -5 )
  ls -l "$ARDUPILOT_ROOT/build/sitl/bin/arducopter" || true
fi

echo
echo "ARDUPILOT_ROOT=$ARDUPILOT_ROOT"
echo "next:  bash setup/build.sh     # the two colcon workspaces"
