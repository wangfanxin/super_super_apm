#!/usr/bin/env bash
# One session = the SPEC S6 unit: TWO sorties in a row, each with its own tag, both scored.
#
#   bash scripts/fly_session.sh TAG [ATTEMPTS] [POINTS]
#     TAG       e.g. ke1  -> sortie tags ke1-01, ke1-02
#     ATTEMPTS  attempts per sortie (default 2)
#     POINTS    lidar points per scan (default 20000 = the hardware pattern)
#
# WHY THIS WRAPPER (each line is a mistake that cost a flight):
#   * it cleans the harness and the lock file FIRST -- a stale SITL on tcp:5780 makes the next sortie
#     abort with "有残留进程";
#   * it checks the frozen world before the first take-off (SPEC S5: the map may not change);
#   * it keeps the whole session log in /tmp so a long run can be read without the terminal;
#   * it tells you the ONE thing to do while it runs: nothing.  Every analysis or plot competing for a
#     core shows up as rtf < 0.98, which turns the sortie's numbers into "not gradeable".
set -u
cd "$(dirname "$0")/.." || exit 1
TAG=${1:?usage: fly_session.sh TAG [ATTEMPTS] [POINTS]}
ATT=${2:-2}
export LIDAR_POINTS=${3:-${LIDAR_POINTS:-20000}}
# ★ core dumps ★ Super-LIO dies with SIGFPE in ~5 % of sorties.  Without this line nothing is written
#   (WSL ships core_pattern=|/wsl-capture-crash and ulimit -c 0).  With it the next crash leaves
#   /tmp/dsh_core.super_lio_node.<pid>, and `gdb -batch -ex bt <node> <core>` names the exact line.
#   The pattern itself is set once by scripts/lio_core_setup.sh (needs root, passwordless here).
ulimit -c unlimited 2>/dev/null || true
# The pattern lives in /proc and WSL restores its own value when the last session closes, so it is
# (re)set here rather than once by hand -- otherwise the next crash writes nothing, which is exactly
# what happened to ki1-01-01 (LIO SIGFPE, no core, pattern back to wsl-capture-crash).
sudo -n sh -c 'echo /tmp/dsh_core.%e.%p > /proc/sys/kernel/core_pattern' 2>/dev/null || true
echo "== core dumps: pattern=$(cat /proc/sys/kernel/core_pattern) ulimit_c=$(ulimit -c)"
LOG=/tmp/${TAG}_session.log

echo "== session $TAG : $LIDAR_POINTS points, $ATT attempts per sortie"
python3 scripts/check_frozen.py 2>&1 | tail -2 || exit 1
bash scripts/clean_harness.sh 2>&1 | tail -1
rm -f /tmp/lio_avoid_sortie_super.lock
echo "== launching (log: $LOG).  DO NOT run anything heavy until it finishes."
bash scripts/fly_hex2.sh "$TAG" "$ATT" > "$LOG" 2>&1
rc=$?
echo "== fly_hex2 exit=$rc"
echo "== last acceptance block:"
grep -E '^\s+\[(PASS|FAIL|INFO)\]|RESULT|GATE|not gradeable' "$LOG" | tail -18
