#!/usr/bin/env bash
# --- release-tree path bootstrap (git clone anywhere) -----------------------
if [ -z "${DSH_ROOT:-}" ]; then
  _dsh="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  while [ "$_dsh" != "/" ] && [ ! -f "$_dsh/setup/env.sh" ]; do _dsh="$(dirname "$_dsh")"; done
  [ -f "$_dsh/setup/env.sh" ] && . "$_dsh/setup/env.sh"
fi
# -----------------------------------------------------------------------------
# Static gate before any takeoff (round 87): SPEC self-check + frozen-map check.
# Kept as a script because passing "|" inside a grep pattern through the Windows
# shell to bash keeps getting mangled -- see docs/FLIGHT-ANALYSIS.md §41 note 3.
set -u
cd "$(dirname "$0")/.." || exit 1

echo "===== spec_check.py"
python3 scripts/spec_check.py > /tmp/static_spec.txt 2>&1
rc_spec=$?
grep -n -e FAIL /tmp/static_spec.txt || echo "(no FAIL rows)"
tail -2 /tmp/static_spec.txt
echo "spec rc=$rc_spec"

echo
echo "===== check_frozen.py"
python3 scripts/check_frozen.py > /tmp/static_frozen.txt 2>&1
rc_frozen=$?
grep -n -e FAIL /tmp/static_frozen.txt || echo "(no FAIL rows)"
tail -3 /tmp/static_frozen.txt
echo "frozen rc=$rc_frozen"

echo
echo "===== the patches that remain in the BUILT source (10/15/17/20/22 + 9/18)"
rc_patch=0
PATCHES=hku_super/patches
for p in patch_super_replan_time_limit patch_super_pva_tail patch_super_general_bc patch_super_adaptive_tail patch_super_keep_old_goal_guide; do
  out=$(python3 "$PATCHES/$p.py" --check 2>&1)
  case "$out" in
    *"is APPLIED"*) echo "  $(printf '%-34s' "$p") APPLIED" ;;
    *) echo "  $(printf '%-34s' "$p") NOT APPLIED -- the flight binary would not have it"; rc_patch=1 ;;
  esac
done
echo "patch rc=$rc_patch"


echo
if [ "$rc_spec" -eq 0 ] && [ "$rc_frozen" -eq 0 ] && [ "${rc_patch:-1}" -eq 0 ]; then
  echo "STATIC GATE: PASS (clear to fly)"
  exit 0
fi
echo "STATIC GATE: FAIL (do not fly)"
exit 1
