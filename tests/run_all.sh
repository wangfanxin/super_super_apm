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
done < <(find . -name '*.sh' -not -path './.git/*' -not -path './.build/*' -not -path './.runs/*' -not -path './hku_super/*' -not -path './slam/*')
echo "   $n file(s)"

step "py_compile (every .py)"
m=0
while IFS= read -r f; do
  m=$((m + 1)); python3 -m py_compile "$f" || note_fail "py_compile $f"
done < <(find . -name '*.py' -not -path './.git/*' -not -path './.build/*' -not -path './.runs/*' -not -path './hku_super/*' -not -path './slam/*')
echo "   $m file(s)"

if [ "$ONLY_SYNTAX" -eq 1 ]; then
  [ "$fail" -eq 0 ] && { echo; echo "SYNTAX: PASS"; exit 0; }
  echo; echo "SYNTAX: FAIL"; exit 1
fi

step "static gate"
bash scripts/check_static.sh || note_fail "check_static.sh"

step "sensor ray pattern (sim/pattern/mid360_pattern.csv)"
bash scripts/check_pattern.sh || note_fail "check_pattern.sh (see sim/pattern/README.md)"

step "unit tests"
python3 scripts/test_ref_lp.py || note_fail "test_ref_lp.py"
python3 scripts/harness/test-stall-gate.py || note_fail "test-stall-gate.py"

step "path policy"
bash tests/no_abs_paths.sh || note_fail "no_abs_paths.sh"

step "evidence links"
bash scripts/check_evidence_links.sh || note_fail "check_evidence_links.sh"

step "tool smoke: every python tool IMPORTS cleanly (a broken import is the real failure here)"
# Not every historical tool has a CLI (`--help` legitimately exits 2 for positional-only tools), so the
# check is: the interpreter must not die with a traceback on import/startup.  A tool that prints its usage
# and exits 2 is fine; a tool that raises ImportError/NameError is not.
k=0
guard() {   # md5 of every SPEC-frozen input; a tool must never change them
  find config params worlds calib -type f 2>/dev/null | sort | xargs md5sum 2>/dev/null | md5sum | cut -d' ' -f1
}
before_g=$(guard)
skipped=0
for f in scripts/*.py; do
  case "$(basename "$f")" in
    _paths.py) continue ;;
  esac
  # positional-only tools (no argparse) cannot answer --help; running them with it makes them read
  # "--help" as a filename, which looks like a crash and tells us nothing.  Skip them by construction.
  if ! grep -q "argparse.ArgumentParser" "$f"; then skipped=$((skipped + 1)); continue; fi
  out=$(timeout 20 python3 "$f" --help 2>&1)
  if printf '%s' "$out" | grep -q -e "Traceback (most recent call last)" -e "ModuleNotFoundError" -e "ImportError"; then
    echo "   broken on startup: $f"
    printf '%s\n' "$out" | grep -e "Error" | head -1 | sed 's/^/        /'
    k=$((k + 1))
  fi
  if [ "$(guard)" != "$before_g" ]; then
    echo "   !! $f MODIFIED a SPEC-frozen input (config/params/worlds/calib) -- restoring"
    git checkout -- config params worlds calib 2>/dev/null || true
    before_g=$(guard)
    k=$((k + 1))
  fi
done
echo "   $k tool(s) broken, $skipped skipped (no argparse)"
[ "$k" -gt 0 ] && note_fail "$k tool(s) fail to start (or write to a frozen input)"

echo
if [ "$fail" -eq 0 ]; then echo "GATE: PASS"; exit 0; fi
echo "GATE: FAIL"; exit 1
