#!/usr/bin/env bash
# warmup.sh -- put the HOST out of its idle/low-power state before a sortie.
#
# WHY THIS EXISTS (MEASURED 2026-11-25): the same arm, same world, same binaries failed H6 (LiDAR simulator
# starved: cast p50 100+ ms, scans skipped) on an idle machine and passed (cast p50 15 ms, 0 skipped) when
# every vCPU had been busy for the preceding minute.  The simulator gives each 10 Hz scan a 100 ms budget
# and its casts are memory-bandwidth-bound; on a freshly-idle laptop the host's power management and the
# wake-up latency of the SITL/sensor threads blow that budget.  Long compile jobs had masked this all
# evening -- the passes happened right after builds.
#
#   bash scripts/warmup.sh [seconds]     # default 45
#
# It is a plain CPU burn on every vCPU, then exit; the sortie starts after it returns.
set -u
R="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SECS=${1:-45}
NPROC=$(nproc)
echo "warmup: $NPROC vCPU busy for ${SECS}s (host p-state / wake-up latency)"
python3 - "$NPROC" "$SECS" <<'PY'
import multiprocessing as mp
import sys
import time

n = int(sys.argv[1])
secs = float(sys.argv[2])


def burn(_):
    t = time.monotonic()
    x = 0.0
    while time.monotonic() - t < secs:
        x += 1.0
    return x


with mp.Pool(n) as pool:
    pool.map(burn, range(n))
PY
echo "warmup: done"
