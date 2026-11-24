#!/usr/bin/env bash
# The no-flight gate.  Everything here must pass before a commit that touches the chain.
#   bash tests/run_all.sh [--only-syntax]
set -u
R="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$R" || exit 1
ONLY_SYNTAX=0
[ "${1:-}" = "--only-syntax" ] && ONLY_SYNTAX=1
fail=0
step() { echo; echo "===== $*"; }
note_fail() { echo "   FAILED: $*" >&2; fail=1; }

step "bash -n (every .sh)"
n=0
while IFS= read -r f; do
  n=$((n + 1)); bash -n "$f" || note_fail "bash -n $f"
done < <(find . -name '*.sh' -not -path './.git/*' -not -path './.build/*' -not -path './.runs/*' -not -path './third_party/*')
echo "   $n file(s)"

step "py_compile (every .py)"
m=0
while IFS= read -r f; do
  m=$((m + 1)); python3 -m py_compile "$f" || note_fail "py_compile $f"
done < <(find . -name '*.py' -not -path './.git/*' -not -path './.build/*' -not -path './.runs/*' -not -path './third_party/*')
echo "   $m file(s)"

if [ "$ONLY_SYNTAX" -eq 1 ]; then
  [ "$fail" -eq 0 ] && { echo; echo "SYNTAX: PASS"; exit 0; }
  echo; echo "SYNTAX: FAIL"; exit 1
fi

step "static gate"
bash scripts/check_static.sh || note_fail "check_static.sh"

step "sensor ray pattern (harness/sim/mid360_pattern.csv)"
bash scripts/check_pattern.sh || note_fail "check_pattern.sh (see harness/sim/README.md)"

step "unit tests"
python3 scripts/test_ref_lp.py || note_fail "test_ref_lp.py"
python3 harness/src/tools/test-stall-gate.py || note_fail "test-stall-gate.py"

step "path policy"
bash tests/no_abs_paths.sh || note_fail "no_abs_paths.sh"

step "evidence links"
bash scripts/check_evidence_links.sh || note_fail "check_evidence_links.sh"

step "tool --help smoke (python tools in scripts/)"
k=0
for f in scripts/*.py; do
  case "$(basename "$f")" in
    accept.py|show_gate.py) continue ;;
  esac
  timeout 20 python3 "$f" --help >/dev/null 2>&1 || { echo "   --help failed: $f"; k=$((k + 1)); }
done
echo "   $k tool(s) failed --help"
[ "$k" -gt 3 ] && note_fail "more than 3 tools cannot print --help"

echo
if [ "$fail" -eq 0 ]; then echo "GATE: PASS"; exit 0; fi
echo "GATE: FAIL"; exit 1
