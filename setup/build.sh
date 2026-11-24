#!/usr/bin/env bash
# =============================================================================
#  setup/build.sh -- create the two colcon workspaces inside .build/ and build them.
#
#  What it does (idempotent):
#    1. .build/ws_super/src/{super_planner,rog_map,mission_planner,mars_quadrotor_msgs}
#         -> symlinks into hku_super/...              (the planner we integrate)
#    2. .build/upstream/SUPER -> hku_super           (the layout scripts/install_config_hex.sh expects)
#    3. .build/lio_ws/src/{super_lio,basic,livox_ros_driver2,quadrotor_msgs}
#         -> symlinks into slam/... + fcu_interface/msgs/quadrotor_msgs
#    4. colcon build (Release) in both workspaces
#
#  ArduPilot SITL is NOT built here -- it is a 1.5 GB checkout.  Run setup/fetch_ardupilot.sh first
#  (or point ARDUPILOT_ROOT at an existing checkout); this script only reports whether it is present.
#
#    bash setup/build.sh [--no-lio]
# =============================================================================
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
. "$HERE/env.sh"
set +u
[ -f /opt/ros/humble/setup.bash ] && . /opt/ros/humble/setup.bash
set -u

BUILD_LIO=1
[ "${1:-}" = "--no-lio" ] && BUILD_LIO=0

echo "==================== release build"
dsh_env_summary
echo

# ---- 1/2. super workspace -------------------------------------------------------
echo "== linking the planner into $WS_SUPER"
mkdir -p "$WS_SUPER/src" "$DSH_ROOT/.build/upstream"
ln -sfn "$SUPER_PLANNER_ROOT" "$DSH_ROOT/.build/upstream/SUPER"
for p in super_planner rog_map mission_planner mars_uav_sim/mars_quadrotor_msgs; do
  ln -sfn "$SUPER_PLANNER_ROOT/$p" "$WS_SUPER/src/$(basename "$p")"
done
ls -l "$WS_SUPER/src" | sed 's/^/   /'

echo "== colcon build (super_planner)"
# TWO PHASES on purpose: `super_planner/CMakeLists.txt` calls find_package(mars_quadrotor_msgs) but its
# package.xml does NOT declare that dependency, so colcon is free to configure super_planner before the
# message package is installed -- measured: "Could not find a package configuration file provided by
# mars_quadrotor_msgs".  Phase 1 builds the dependencies, phase 2 (with their install/ sourced) builds
# the planner.  The ordered build is what a human does by hand the second time.
set +u
( cd "$WS_SUPER" && colcon build --packages-select mars_quadrotor_msgs rog_map mission_planner \
      --cmake-args -DCMAKE_BUILD_TYPE=Release ) 2>&1 | tail -4
# shellcheck disable=SC1091
. "$WS_SUPER/install/setup.bash"
( cd "$WS_SUPER" && colcon build --packages-select super_planner \
      --cmake-args -DCMAKE_BUILD_TYPE=Release ) 2>&1 | tail -6
set -u
if [ -x "$WS_SUPER/install/super_planner/lib/super_planner/fsm_node" ]; then
  echo "   fsm_node: $(ls -l "$WS_SUPER/install/super_planner/lib/super_planner/fsm_node" | awk '{print $5, $6, $7, $8}')"
else
  echo "   !! fsm_node was not produced -- see the build log above" >&2
  exit 1
fi

# ---- 3/4. lio workspace ---------------------------------------------------------
if [ "$BUILD_LIO" -eq 1 ]; then
  echo
  echo "== linking Super-LIO into $LIO_WS"
  mkdir -p "$LIO_WS/src"
  for p in super_lio basic; do
    ln -sfn "$SLAM_ROOT/$p" "$LIO_WS/src/$p"
  done
  ln -sfn "$SLAM_ROOT/livox_ros_driver2" "$LIO_WS/src/livox_ros_driver2"
  ln -sfn "$ARDUPILOT_DIR/msgs/quadrotor_msgs" "$LIO_WS/src/quadrotor_msgs"
  ls -l "$LIO_WS/src" | sed 's/^/   /'

  echo "== colcon build (lio: super_lio + livox driver + msgs)"
  set +u
  ( cd "$LIO_WS" && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DDISTRO_ROS=humble -DROS_EDITION=ROS2 ) 2>&1 | tail -8
  set -u
fi

echo
echo "== ArduPilot SITL"
if [ -x "$ARDUPILOT_ROOT/build/sitl/bin/arducopter" ]; then
  echo "   found: $ARDUPILOT_ROOT/build/sitl/bin/arducopter"
  if [ -x "$ARDUPILOT_ROOT/build/sitl/bin/arducopter.tap-patched" ]; then
    echo "   patched (legacy-tap) binary also present: arducopter.tap-patched"
  fi
  # PHASE 1b (2026-11-25): the DEFAULT binary is the UNPATCHED SITL -- truth comes from MAVLink SIM_STATE via
  # sim/src/sim_state_recv.py.  The old tap path is archived:
  #   apply:  cd "$ARDUPILOT_ROOT" && patch -p1 < "$DSH_ROOT/sim/legacy/apm-0001-sitl-truth-tap.patch"
  #           ./waf copter && cp build/sitl/bin/arducopter build/sitl/bin/arducopter.tap-patched
  #   revert: cd "$ARDUPILOT_ROOT" && patch -R -p1 < "$DSH_ROOT/sim/legacy/apm-0001-sitl-truth-tap.patch"
  #           ./waf configure --board sitl && ./waf copter
  echo "   truth source: SIM_STATE (unpatched SITL).  Tap patch archived in sim/legacy/ (see its README)."
else
  echo "   NOT built.  Run:  bash setup/fetch_ardupilot.sh      (1.5 GB clone + build)"
  echo "   or:               ARDUPILOT_ROOT=/existing/ardupilot bash setup/run_hex.sh <tag>"
fi
echo
echo "BUILD: done"
