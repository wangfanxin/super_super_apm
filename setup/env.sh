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

# ---- the five modules -----------------------------------------------------------
export BRIDGE_ROOT="${BRIDGE_ROOT:-$DSH_ROOT/bridge_nodes}"            # our thin layer + shared libs
export SUPER_PLANNER_ROOT="${SUPER_PLANNER_ROOT:-$DSH_ROOT/hku_super}"   # hku-mars SUPER
export SLAM_ROOT="${SLAM_ROOT:-$DSH_ROOT/slam}"                # Super-LIO + livox + LIO config
export SIM_ROOT="${SIM_ROOT:-$DSH_ROOT/sim}"                   # simulator, worlds, rviz, frozen
export ARDUPILOT_DIR="${ARDUPILOT_DIR:-$DSH_ROOT/fcu_interface}"  # FCU parameters + message packages
# kept for scripts that predate the split: the thin layer's --lio-root is the shared-lib module
export LIO_ROOT="${LIO_ROOT:-$BRIDGE_ROOT}"
export CALIB_FILE="${CALIB_FILE:-$DSH_ROOT/calib/calib.json}"  # L->W calibration
export MID360_PATTERN="${MID360_PATTERN:-$SIM_ROOT/pattern/mid360_pattern.csv}"

# ---- build outputs (outside git) ------------------------------------------------
export WS_SUPER="${WS_SUPER:-$DSH_ROOT/.build/ws_super}"
export LIO_WS="${LIO_WS:-$DSH_ROOT/.build/lio_ws}"
export LIO_LOGS="${LIO_LOGS:-$DSH_ROOT/.runs}"                # sortie recordings
export ARDUPILOT_ROOT="${ARDUPILOT_ROOT:-$DSH_ROOT/.build/ardupilot}"

# ---- optional machine-local overrides -------------------------------------------
# On this machine the repo lives on the Windows drive (D:) but the builds and the recordings deliberately
# stay on the WSL ext4 filesystem (fast, and ArduPilot's 1.5 GB clone does not belong on /mnt/d).
# setup/runtime.env is NOT committed (see .gitignore); a fresh clone simply has none and uses .build/.
if [ -f "$DSH_ROOT/setup/runtime.env" ]; then
  # shellcheck disable=SC1090,SC1091
  . "$DSH_ROOT/setup/runtime.env"
fi

# ---- ROS 2 ---------------------------------------------------------------------
if [ -f /opt/ros/humble/setup.bash ] && [ -z "${ROS_DISTRO:-}" ]; then
  set +u
  # shellcheck disable=SC1091
  . /opt/ros/humble/setup.bash
  set -u
fi

export PYTHONPATH="$DSH_ROOT/bridge_nodes:$DSH_ROOT/scripts${PYTHONPATH:+:$PYTHONPATH}"

# A short, human-readable line -- printed by setup/build.sh and by a flight's first line.
dsh_env_summary() {
  echo "DSH_ROOT         = $DSH_ROOT"
  echo "SUPER (planner)  = $SUPER_PLANNER_ROOT"
  echo "thin layer       = $BRIDGE_ROOT"
  echo "SUPER module     = $SUPER_PLANNER_ROOT"
  echo "SLAM module      = $SLAM_ROOT"
  echo "sim module       = $SIM_ROOT"
  echo "ArduPilot driver = $ARDUPILOT_DIR"
  echo "ArduPilot SITL   = $ARDUPILOT_ROOT"
  echo "ws_super         = $WS_SUPER"
  echo "lio_ws           = $LIO_WS"
  echo "recordings       = $LIO_LOGS"
}
