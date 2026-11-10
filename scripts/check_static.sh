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
# shell to bash keeps getting mangled -- see docs/INTEGRATION.md §41 note 3.
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
echo "===== patches 10/11 (they live in the BUILT source, so check them, not the docs)"
rc_patch=0
for p in patch_super_replan_time_limit patch_super_no_backward_head; do
  out=$(python3 "scripts/$p.py" --check 2>&1)
  case "$out" in
    *"is APPLIED"*) echo "  $(printf '%-34s' "$p") APPLIED" ;;
    *) echo "  $(printf '%-34s' "$p") NOT APPLIED -- the flight binary would not have it"; rc_patch=1 ;;
  esac
done
echo "patch rc=$rc_patch"

echo
echo "===== patch 12 (sfc_margin) -- OPTIONAL, and REJECTED on evidence (§94/§95)"
out=$(python3 scripts/patch_super_sfc_margin.py --check 2>&1 | tail -1)
rc_sfc=0
case "$out" in
  *"is APPLIED"*)
    echo "  patch_super_sfc_margin              APPLIED -- default 0.0 is a no-op, but a config can turn"
    echo "                                      it on; measured harmful at 0.12 (W1b 3/13, §95)"
    echo "===== is its knob read by the header the flight links?"
    bash scripts/test_sfc_margin_load.sh > /tmp/static_sfc.txt 2>&1
    rc_sfc=$?
    grep -n -e FAIL -e "loader test" /tmp/static_sfc.txt || true
    ;;
  *)
    echo "  patch_super_sfc_margin              not applied (expected: the delivered tree is patches 10+11)"
    ;;
esac
echo "sfc-margin rc=$rc_sfc"

echo
if [ "$rc_spec" -eq 0 ] && [ "$rc_frozen" -eq 0 ] && [ "${rc_patch:-1}" -eq 0 ] && [ "${rc_sfc:-1}" -eq 0 ]; then
  echo "STATIC GATE: PASS (clear to fly)"
  exit 0
fi
echo "STATIC GATE: FAIL (do not fly)"
exit 1
