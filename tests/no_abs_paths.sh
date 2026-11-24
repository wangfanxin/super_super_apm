#!/usr/bin/env bash
# Path policy: the flight/grade chain must not contain the author's absolute paths; historical analysis
# tools are reported but do not fail the gate (they are listed in docs/PATHS.md §4).
#   bash tests/no_abs_paths.sh
set -u
R="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$R" || exit 1
PAT='/mnt/d/distill_self\|/home/fanxin'

REQUIRED="
setup
tests
apm_thin
ardupilot/bridge
ardupilot/supervisor
sim/src
scripts/fly_hex.sh
scripts/fly_hex2.sh
scripts/fly_speed_run.sh
scripts/run_until_healthy.sh
scripts/run_accept.sh
scripts/check_static.sh
scripts/accept.py
scripts/check_frozen.py
scripts/spec_check.py
scripts/install_config_hex.sh
scripts/render_evidence.sh
scripts/run-mission.sh
"

echo "== must be path-free"
bad=0
for p in $REQUIRED; do
  [ -e "$p" ] || { echo "   MISSING $p"; bad=1; continue; }
  hits=$(grep -rn "$PAT" "$p" --include=*.py --include=*.sh 2>/dev/null | grep -v '^\s*#' \
         | grep -v "tests/no_abs_paths.sh" | wc -l)
  if [ "$hits" -gt 0 ]; then
    echo "   FAIL $p ($hits author path reference(s))"
    grep -rn "$PAT" "$p" --include=*.py --include=*.sh 2>/dev/null | head -3 | sed 's/^/        /'
    bad=1
  fi
done

echo
echo "== historical analysis tools still carrying author paths (informational)"
n=0
for f in scripts/*.py scripts/*.sh scripts/harness/*.py ardupilot/bridge/*.py ardupilot/supervisor/*.py sim/src/*.py; do
  [ "$f" = "tests/no_abs_paths.sh" ] && continue
  [ -f "$f" ] || continue
  if grep -q "$PAT" "$f" 2>/dev/null; then n=$((n + 1)); fi
done
echo "   $n file(s) (see docs/PATHS.md §4)"

if [ "$bad" -ne 0 ]; then echo; echo "PATH POLICY: FAIL"; exit 1; fi
echo
echo "PATH POLICY: PASS"
