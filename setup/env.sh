#!/usr/bin/env bash
# =============================================================================
#  setup/env.sh -- the ONLY place in this repository that knows where things are.
#
#  Every script under scripts/ and harness/ sources this file (directly, or through the small
#  bootstrap the release build inserted: they walk up to the directory holding setup/env.sh).
#  Nothing else needs to know the clone path, so the repo runs from any directory.
#
#  Override anything by exporting it before you call a script, e.g.
#     ARDUPILOT_ROOT=/opt/ardupilot bash setup/build.sh
#     LIO_LOGS=/data/runs bash setup/run_hex.sh kx6 3
# =============================================================================

# the repo root = parent of this file's directory
DSH_ROOT="${DSH_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
export DSH_ROOT

# ---- what lives inside the repo -------------------------------------------------
export SUPER_PLANNER_ROOT="${SUPER_PLANNER_ROOT:-$DSH_ROOT/third_party/SUPER}"
export HARNESS_ROOT="${HARNESS_ROOT:-$DSH_ROOT/harness}"
export LIO_ROOT="${LIO_ROOT:-$HARNESS_ROOT}"                  # the thin layer's --lio-root
export CALIB_FILE="${CALIB_FILE:-$DSH_ROOT/calib/calib.json}"  # L->W calibration
export MID360_PATTERN="${MID360_PATTERN:-$HARNESS_ROOT/sim/mid360_pattern.csv}"

# ---- build outputs (outside git) ------------------------------------------------
export WS_SUPER="${WS_SUPER:-$DSH_ROOT/.build/ws_super}"
export LIO_WS="${LIO_WS:-$DSH_ROOT/.build/lio_ws}"
export LIO_LOGS="${LIO_LOGS:-$DSH_ROOT/.runs}"                # sortie recordings
export ARDUPILOT_ROOT="${ARDUPILOT_ROOT:-$DSH_ROOT/.build/ardupilot}"

# ---- ROS 2 ---------------------------------------------------------------------
if [ -f /opt/ros/humble/setup.bash ] && [ -z "${ROS_DISTRO:-}" ]; then
  set +u
  # shellcheck disable=SC1091
  . /opt/ros/humble/setup.bash
  set -u
fi

export PYTHONPATH="$DSH_ROOT/src/whtf/ros:$HARNESS_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"

# A short, human-readable line -- printed by setup/build.sh and by a flight's first line.
dsh_env_summary() {
  echo "DSH_ROOT         = $DSH_ROOT"
  echo "SUPER (planner)  = $SUPER_PLANNER_ROOT"
  echo "harness          = $HARNESS_ROOT"
  echo "ArduPilot SITL   = $ARDUPILOT_ROOT"
  echo "ws_super         = $WS_SUPER"
  echo "lio_ws           = $LIO_WS"
  echo "recordings       = $LIO_LOGS"
}
