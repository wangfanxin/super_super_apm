#!/usr/bin/env bash
set -u
S=/home/fanxin/lio_avoid/src/lio-fork/super_lio_local/src/lio/super_lio.cpp
echo "== where Output() transforms a cloud"
grep -n 'Output\|transformPointCloud' "$S" | head -20
echo
echo "== the Output() body"
START=$(grep -n 'void SuperLIO::Output' "$S" | head -1 | cut -d: -f1)
[ -n "$START" ] && sed -n "${START},$((START+45))p" "$S"
