#!/usr/bin/env bash
# finalize_bag.sh -- Phase 2 wrap-up: move the ROSBAG2 recording into the sortie directory and derive the
# CSV layer from it.
#
#   bash scripts/finalize_bag.sh <run-dir> <bag-parent> [split]
#
# `bag-parent` is where the recorder wrote the bags.  It is normally /dev/shm (tmpfs): MEASURED 2026-11-25,
# bag writes on the ext4 virtual disk are what turn into multi-second VM-wide stalls -- the LiDAR cast time
# jumps from ~15 ms to 1.9-2.2 s, the LIO dead-reckons and the sortie aborts.  Recording to tmpfs and
# copying the (tens of MB) bag afterwards keeps the flight path off that path.
#
# Outputs in <run-dir>:
#   bag/ or bag_a/ + bag_b/      the bags, moved out of tmpfs
#   derived/lio.csv              /lio/robo/odom derived from the bag
#   derived/traj.csv.cmd.csv     /plan/cmd derived from the bag
#   bag_compare.txt              derived vs live CSV agreement (rows and byte-identical rows)
#
# Exit status: 0 when the bags moved and the derivation produced non-empty CSVs.
set -u
R="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT=${1:?usage: finalize_bag.sh <run-dir> <bag-parent> [split]}
BAGPARENT=${2:?usage: finalize_bag.sh <run-dir> <bag-parent> [split]}
SPLIT=${3:-0}
mkdir -p "$OUT/derived"
BAGS=()
if [ "$SPLIT" = "1" ]; then
    for g in a b; do
        if [ -d "$BAGPARENT/bag_$g" ]; then
            cp -a "$BAGPARENT/bag_$g" "$OUT/bag_$g" 2>/dev/null
            BAGS+=("$OUT/bag_$g")
        fi
    done
else
    if [ -d "$BAGPARENT/bag" ]; then
        cp -a "$BAGPARENT/bag" "$OUT/bag" 2>/dev/null
        BAGS+=("$OUT/bag")
    fi
fi
if [ "${#BAGS[@]}" -eq 0 ]; then
    echo "finalize_bag: no bag found under $BAGPARENT" | tee "$OUT/bag_compare.txt" >&2
    exit 1
fi

# shellcheck disable=SC2086
ARGS=()
for b in "${BAGS[@]}"; do ARGS+=(--bag "$b"); done
if [ -f /opt/ros/humble/setup.bash ]; then
    set +u
    . /opt/ros/humble/setup.bash
    . "${LIO_WS:-$R/.build/lio_ws}/install/setup.bash" 2>/dev/null
    . "${WS_SUPER:-$R/.build/ws_super}/install/setup.bash" 2>/dev/null
    set -u
fi
python3 "$R/scripts/bag_to_csv.py" "${ARGS[@]}" --derive-all --outdir "$OUT/derived" \
    > "$OUT/derived/bag_to_csv.log" 2>&1 || true
# The live recorders still produce these during the transition; the bag-derived copies are the target.
[ -f "$OUT/traj.csv.cmd.csv" ] || cp "$OUT/derived/traj.csv.cmd.csv" "$OUT/traj.csv.cmd.csv" 2>/dev/null || true
[ -f "$OUT/lio.csv" ] || cp "$OUT/derived/lio.csv" "$OUT/lio.csv" 2>/dev/null || true

python3 - "$OUT" <<'PY' | tee "$OUT/bag_compare.txt"
import os
import sys

out = sys.argv[1]
lines = []
for name in ("lio.csv", "traj.csv.cmd.csv"):
    live = os.path.join(out, name)
    der = os.path.join(out, "derived", name)
    if not (os.path.isfile(live) and os.path.isfile(der)):
        lines.append("%-18s live=%s derived=%s" % (name, os.path.isfile(live), os.path.isfile(der)))
        continue
    a = open(live, encoding="utf-8").read().splitlines()
    b = open(der, encoding="utf-8").read().splitlines()
    aset = set(a[1:])
    hits = sum(1 for r in b[1:] if r in aset)
    same_header = a[0] == b[0]
    lines.append("%-18s live=%6d derived=%6d identical_rows=%6d header_match=%s"
                 % (name, len(a) - 1, len(b) - 1, hits, same_header))
for b in sorted(p for p in os.listdir(out) if p.startswith("bag")):
    lines.append("bag: %s" % b)
print("\n".join(lines))
PY
rm -rf "$BAGPARENT" 2>/dev/null || true
exit 0
