#!/usr/bin/env bash
# Where does the built liblio.so come from, and what are the build flags?
set -u
WS=/home/fanxin/lio_avoid/lio_ws
echo "== workspace layout"
ls -la "$WS" | head -8
echo
echo "== the source package the build actually uses"
readlink -f "$WS/src/super_lio" 2>/dev/null
ls -la "$WS/src/" | head -8
echo
echo "== is it a symlink or a copy?"
if [ -L "$WS/src/super_lio" ]; then echo "   SYMLINK -> $(readlink -f "$WS/src/super_lio")"; else echo "   REAL DIRECTORY (a copy)"; fi
cmp -s "$WS/src/super_lio/src/lio/super_lio.cpp" \
       /home/fanxin/lio_avoid/src/lio-fork/super_lio_local/src/lio/super_lio.cpp \
  && echo "   source identical to src/lio-fork/super_lio_local" || echo "   source DIFFERS from src/lio-fork/super_lio_local"
echo
echo "== build config"
grep -E 'CMAKE_BUILD_TYPE|CMAKE_HOME_DIRECTORY' "$WS/build/super_lio/CMakeCache.txt" 2>/dev/null | head -4
echo
echo "== installed artefacts"
ls -la "$WS/install/super_lio/lib/" 2>/dev/null | head -6
ls -la "$WS/install/super_lio/lib/super_lio/" 2>/dev/null | head -4
