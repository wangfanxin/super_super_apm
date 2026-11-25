#!/usr/bin/env bash
# check_super_velocity_wiring.sh -- the vehicle VELOCITY has to reach the planner (2026-11-25 fix).
#
# MEASURED defect (docs/FLIGHT-LOOP-ANALYSIS.md, phase4p): SUPER's replanner injects `robot_state_.v` as the
# initial velocity of every rest2rest replan (config `hot_start_en: true`) so the reference does not brake to a
# stop at each goal handover.  The velocity never arrived:
#   * Super-LIO's robot state had `// todo: v,w,a` -> /lio/robo/odom twist was all zeros;
#   * rog_map's odometry callback kept only the POSE, and ROGMap::updateRobotState never wrote v.
# Observed effect: 22 command "stops" per flight (11 corner + 11 mid-leg) and 5 aircraft loops, each at
# "command speed 0.00-0.10 m/s while truth 1.0-1.5 m/s".  This test pins the FIVE links of the fix, so a rebuild
# from a stale source cannot silently undo it.
set -u
R="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$R" || exit 1
fails=0
note() { echo "  - $*"; fails=$((fails + 1)); }

grep -q "state_robot.v = new_R.R_.transpose() \* fw_v_;" slam/super_lio/src/lio/ESKF.cpp \
    || note "Super-LIO ESKF no longer fills state_robot.v (robot-frame velocity)"
grep -q "state_robot.w = gyr;" slam/super_lio/src/lio/ESKF.cpp \
    || note "Super-LIO ESKF no longer fills state_robot.w"
grep -q "odom_robo.twist.twist.linear.x = robo_state.v(0);" slam/super_lio/src/ros/ROSWrapper.cpp \
    || note "/lio/robo/odom lost its published twist (ROSWrapper)"
grep -q "void updateRobotState(const Pose& pose, const Vec3f& v_world);" hku_super/rog_map/include/rog_map/rog_map.h \
    || note "ROGMap lost the updateRobotState(pose, velocity) overload"
grep -q "robot_state_.v = v_world;" hku_super/rog_map/src/rog_map/rog_map.cpp \
    || note "ROGMap no longer stores the velocity into robot_state_.v"
grep -q "q_odom \* v_body" hku_super/rog_map/include/rog_map_ros/rog_map_ros2.hpp \
    || note "rog_map's odometry callback no longer rotates the twist into the world frame"
# the planner must actually use that state (hot_start injection) -- config + code
grep -q "hot_start_en: true" hku_super/config/super_planner_hex.yaml \
    || note "super_planner_hex.yaml no longer enables hot_start_en (the whole point of the wiring)"
grep -q "Vec3f v0 = robot_state_.v;" hku_super/super_planner/src/super_core/super_planner.cpp \
    || note "super_planner.cpp no longer injects robot_state_.v as the rest2rest initial velocity"
# a clean checkout has to build: the upstream CMakeLists hardcoded the author's absolute include path
if grep -q 'CMAKE_CXX_FLAGS.*-include /' hku_super/super_planner/CMakeLists.txt; then
    note "super_planner/CMakeLists.txt still uses an ABSOLUTE -include path (not portable)"
fi
grep -q 'include ${CMAKE_CURRENT_SOURCE_DIR}/ros/super_compat.hpp' hku_super/super_planner/CMakeLists.txt \
    || note "super_planner/CMakeLists.txt lost the portable super_compat include"

if [ "$fails" -ne 0 ]; then
    echo "FAIL: SUPER velocity wiring ($fails problem(s))"
    exit 1
fi
echo "PASS: LIO publishes robot velocity -> rog_map stores it -> SUPER hot_start can inject it (5 links + build path)"
