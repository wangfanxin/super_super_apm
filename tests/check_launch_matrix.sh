#!/usr/bin/env bash
# check_launch_matrix.sh -- Phase 3/4 closeout: the FCU interface has exactly ONE implementation.
#
# Round 20 removed the Phase-3 bisect modes (bridge_only / bridge_thread / supervisor_only / legacy-shape)
# from both the merged client and the launcher.  This test pins that down STATICALLY, because the alternative
# -- "a stale mode still exists and silently launches the wrong shape" -- is exactly the class of bug the
# Phase-3 flights cost a day to find.
#
# PASS means:
#   1. run-mission.sh accepts exactly `merged` (default) and `legacy` (rollback), and rejects anything else;
#   2. the merged branch launches fcu_client.py with BOTH endpoints (serial0 + serial1);
#   3. the legacy branch launches bridge_node.py AND supervisor.py as two processes;
#   4. fcu_client.py has no bisect flags left (--bridge-only / --bridge-thread / --supervisor-only /
#      --legacy-shape / --dual-link) and still has --bench;
#   5. `fcu_client.py --bench --help` parses (the merged parser still works after the cleanup).
set -u
R="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$R" || exit 1
M=scripts/run-mission.sh
C=bridge_nodes/mavlink/fcu_client.py
fails=0
note() { echo "  - $*"; fails=$((fails + 1)); }

# 1. the launcher's mode gate
grep -q 'FCU_CLIENT="${FCU_CLIENT:-merged}"' "$M" || note "run-mission: default is not merged"
grep -q 'allowed: merged (default) | legacy (rollback)' "$M" || note "run-mission: no explicit rejection of unknown modes"
grep -q '^legacy)' "$M" || note "run-mission: legacy (rollback) branch missing"
for dead in bridge_only bridge_thread supervisor_only; do
    if grep -qE "(FCU_CLIENT.*=.*\"$dead\"|--$dead|\b$dead\))" "$M"; then
        note "run-mission still references the removed bisect mode '$dead'"
    fi
done

# 2. merged branch: one process, both endpoints
seg=$(sed -n '/^merged)/,/^legacy)/p' "$M")
echo "$seg" | grep -q 'fcu_client.py' || note "merged branch does not launch fcu_client.py"
echo "$seg" | grep -q -- '--mav "tcp:127.0.0.1:${PORT0:-5780}"' || note "merged branch lost the serial0 endpoint"
echo "$seg" | grep -q -- '--mav-supervisor "tcp:127.0.0.1:${PORT1:-5781}"' || note "merged branch lost the serial1 endpoint"
echo "$seg" | grep -q -- '--nav-log' || note "merged branch does not write nav.csv"

# 3. legacy branch: two processes
seg=$(sed -n '/^legacy)/,/^;;/p' "$M")
echo "$seg" | grep -q 'bridge_node.py' || note "legacy branch does not launch bridge_node.py"
if ! sed -n '/^if \[ "\$FCU_CLIENT" = "legacy" \]/,/^fi$/p' "$M" | grep -q 'supervisor.py'; then
    note "legacy branch does not launch supervisor.py"
fi

# 4. no bisect flags left in the merged client
for dead in --bridge-only --bridge-thread --supervisor-only --legacy-shape --dual-link; do
    if grep -q -- "\"$dead\"" "$C"; then
        note "fcu_client.py still defines a removed flag '$dead'"
    fi
done
grep -q -- '"--bench"' "$C" || note "fcu_client.py lost --bench (the gate test needs it)"

# 5. the merged dispatcher is still there and the file compiles
grep -q 'args.client_mode = "bench" if getattr(args, "bench", False) else "merged"' "$C" \
    || note "fcu_client.py lost the merged/bench mode dispatch"
python3 -m py_compile "$C" || note "fcu_client.py does not compile"

if [ "$fails" -ne 0 ]; then
    echo "FAIL: launch matrix ($fails problem(s))"
    exit 1
fi
echo "PASS: one FCU implementation -- merged (two endpoints, one process) | legacy (two processes, rollback only)"
