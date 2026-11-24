#!/usr/bin/env bash
# Is the LiDAR simulator's vendor ray pattern present and sane?  The flight refuses to start without it,
# and a wrong file is worse than a missing one -- this prints what the loader actually sees.
#
#   bash scripts/check_pattern.sh
set -u
R="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
. "$R/setup/env.sh"

python3 - "$R" <<'PY'
import os, sys
R = sys.argv[1]
sys.path.insert(0, os.path.join(R, "harness", "src", "sim"))
import raycast  # noqa: E402

p = raycast.default_pattern_path()
print("default_pattern_path : %s" % (p or "NONE (the simulator will refuse to run)"))
if not p:
    print("  fix:  curl -fsSL -o %s/harness/sim/mid360_pattern.csv \\" % R)
    print("          https://raw.githubusercontent.com/Livox-SDK/livox_laser_simulation/main/scan_mode/mid360.csv")
    sys.exit(1)
try:
    with open(p) as fh:
        head = fh.readline().strip()
    nrows = sum(1 for _ in open(p))
except OSError as exc:
    print("  unreadable: %s" % exc)
    sys.exit(1)
print("header              : %s" % head)
print("rows                : %d (expect 800001)" % nrows)
rays, times, src = raycast.load_pattern(p, n=20000)
az = rays[:, 0].min(), rays[:, 0].max()
el = rays[:, 1].min(), rays[:, 1].max()
print("loader              : %s" % src)
print("azimuth   deg       : %.2f .. %.2f (expect ~0..360)" % (az[0] * 57.2958, az[1] * 57.2958))
print("elevation deg       : %.2f .. %.2f (expect ~-7..+52)" % (el[0] * 57.2958, el[1] * 57.2958))
ok = (("Zenith" in head) and nrows >= 800000 and el[0] < 0 and (el[1] * 57.2958) > 1.0)
print("RESULT              : %s" % ("OK" if ok else "SUSPECT -- see harness/sim/README.md"))
sys.exit(0 if ok else 1)
PY
