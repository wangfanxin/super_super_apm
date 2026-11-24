#!/usr/bin/env bash
# --- repo path bootstrap (round 21) ------------------------------------------
if [ -z "${DSH_ROOT:-}" ]; then
  _dsh="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  while [ "$_dsh" != "/" ] && [ ! -f "$_dsh/setup/env.sh" ]; do _dsh="$(dirname "$_dsh")"; done
  [ -f "$_dsh/setup/env.sh" ] && . "$_dsh/setup/env.sh"
fi
# -----------------------------------------------------------------------------
# What is `ds_undistort_` (the cloud the non-dense branch transforms) and how big is the downsample?
set -u
S=${DSH_ROOT}/harness/src/lio-fork/super_lio_local/src/lio/super_lio.cpp
CFG=${DSH_ROOT}/harness/src/lio/config/livox_360.yaml
echo "== ds_undistort_ uses"
grep -n 'ds_undistort_' "$S" 2>/dev/null | head -12
echo
echo "== downsample / filter params in the config the node loads"
grep -nE 'filter|downsample|ds_|voxel|pub_step|dense|map' "$CFG" 2>/dev/null | head -16
