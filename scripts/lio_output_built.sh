#!/usr/bin/env bash
# Read Output() in the source tree the build actually compiles (Super-LIO/src/super_lio).
set -u
S=/home/fanxin/lio_avoid/src/Super-LIO/src/super_lio/src/lio/super_lio.cpp
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
ls -la /home/fanxin/lio_avoid/lio_ws/install/super_lio/lib/liblio.so
ls -la /home/fanxin/lio_avoid/lio_ws/install/super_lio/lib/super_lio/ | head -4
