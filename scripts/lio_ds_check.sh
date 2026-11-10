#!/usr/bin/env bash
# What is `ds_undistort_` (the cloud the non-dense branch transforms) and how big is the downsample?
set -u
S=/home/fanxin/lio_avoid/src/lio-fork/super_lio_local/src/lio/super_lio.cpp
CFG=/mnt/d/distill_self/lio_avoid/src/lio/config/livox_360.yaml
echo "== ds_undistort_ uses"
grep -n 'ds_undistort_' "$S" 2>/dev/null | head -12
echo
echo "== downsample / filter params in the config the node loads"
grep -nE 'filter|downsample|ds_|voxel|pub_step|dense|map' "$CFG" 2>/dev/null | head -16
