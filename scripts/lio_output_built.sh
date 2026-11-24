#!/usr/bin/env bash
# --- repo path bootstrap (round 21) ------------------------------------------
if [ -z "${DSH_ROOT:-}" ]; then
  _dsh="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  while [ "$_dsh" != "/" ] && [ ! -f "$_dsh/setup/env.sh" ]; do _dsh="$(dirname "$_dsh")"; done
  [ -f "$_dsh/setup/env.sh" ] && . "$_dsh/setup/env.sh"
fi
# -----------------------------------------------------------------------------
# Read Output() in the source tree the build actually compiles (Super-LIO/src/super_lio).
set -u
S=${SLAM_ROOT}/super_lio/src/lio/super_lio.cpp
echo "== $S"
ls -la "$S"
echo
echo "== the transform sites"
grep -n 'transformPointCloud' "$S" | head -6
echo
echo "== the Output() body"
START=$(grep -n 'void SuperLIO::Output' "$S" | head -1 | cut -d: -f1)
echo "   (starts at line $START)"
sed -n "${START},$((START+28))p" "$S"
echo
echo "== the shared library to back up"
ls -la ${LIO_WS}/install/super_lio/lib/liblio.so
ls -la ${LIO_WS}/install/super_lio/lib/super_lio/ | head -4
