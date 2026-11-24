#!/usr/bin/env bash
# --- repo path bootstrap (round 21) ------------------------------------------
if [ -z "${DSH_ROOT:-}" ]; then
  _dsh="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  while [ "$_dsh" != "/" ] && [ ! -f "$_dsh/setup/env.sh" ]; do _dsh="$(dirname "$_dsh")"; done
  [ -f "$_dsh/setup/env.sh" ] && . "$_dsh/setup/env.sh"
fi
# -----------------------------------------------------------------------------
# Verify the rebuilt LIO: the guard is in the binary, and the node still initialises.
set -u
L=${LIO_WS}/install/super_lio/lib/liblio.so
echo "== guard string present in the rebuilt library?"
strings "$L" 2>/dev/null | grep -c 'empty undistorted cloud' || echo "   0"
echo
echo "== node binary"
ls -la ${LIO_WS}/install/super_lio/lib/super_lio/super_lio_node
echo
echo "== does the node start?  (5 s, no data -> it should initialise and just wait)"
source /opt/ros/humble/setup.bash
cd ${LIO_WS}
( export ROS_DOMAIN_ID=0
  timeout 8 ./install/super_lio/lib/super_lio/super_lio_node --ros-args \
    --params-file ${DSH_ROOT}/harness/src/lio/config/livox_360.yaml 2>&1 | head -12 ) || true
echo
echo "== did it die with SIGFPE?  (expect no 'Floating point' line above)"
