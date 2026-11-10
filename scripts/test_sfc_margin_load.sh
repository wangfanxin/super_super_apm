#!/usr/bin/env bash
# Prove that `traj_opt/boundary/sfc_margin` (patch 12) is actually READ from a config, without flying.
#
# Why this exists: a config knob that is silently not loaded looks exactly like a knob that has no
# effect -- and this project has already paid for that mistake twice (`VISO_*_M_NSE` missing from a
# params dump; `ref-ff-acc-max` defaulting to 0 and doing nothing).  This compiles a 12-line program
# against the SAME header the flight links, constructs traj_opt::Config from a config file, and prints
# the value.
#
#   bash scripts/test_sfc_margin_load.sh
#   bash scripts/test_sfc_margin_load.sh config/super_planner_hex_sfc12.yaml
set -u
cd "$(dirname "$0")/.." || exit 1
ROOT=$(pwd)
SRC=/tmp/sfc_margin_probe.cpp
BIN=/tmp/sfc_margin_probe
INC="-I$ROOT/../../_stage/super_src/include"
# The live upstream headers are the ones the flight binary was built from, so use those first.
UP=/home/fanxin/super_super_apm/upstream/SUPER/super_planner/include
[ -d "$UP" ] && INC="-I$UP"

cat > "$SRC" <<'CPP'
#include <iostream>
#include <traj_opt/config.hpp>
int main(int argc, char **argv) {
    if (argc < 2) { std::cerr << "usage: probe <config.yaml>" << std::endl; return 2; }
    traj_opt::Config cfg(argv[1], "traj_opt");
    std::cout << cfg.sfc_margin << std::endl;
    return 0;
}
CPP

if ! g++ -std=c++17 -O0 -o "$BIN" "$SRC" $INC -I/usr/include/eigen3 -lyaml-cpp 2>/tmp/sfc_probe_build.log; then
  echo "PROBE BUILD FAILED -- see /tmp/sfc_probe_build.log" >&2
  sed -n '1,12p' /tmp/sfc_probe_build.log >&2
  exit 1
fi

rc=0
check() {  # check <config> <expected>
  local cfg=$1 want=$2 got
  if [ ! -f "$cfg" ]; then echo "MISSING $cfg" >&2; rc=1; return; fi
  # The loader prints one line per parameter (loud on purpose upstream); the probe's own answer is the
  # last line of stdout, so keep only that.
  got=$("$BIN" "$cfg" 2>/dev/null | tail -1)
  if [ "$got" = "$want" ]; then
    printf "  OK   %-46s sfc_margin = %s\n" "$cfg" "$got"
  else
    printf "  FAIL %-46s sfc_margin = %s (expected %s)\n" "$cfg" "$got" "$want"
    rc=1
  fi
}

echo "== the key is read by the same header the flight links ($INC)"
check "$ROOT/config/super_planner_hex.yaml" "0"
for f in "$@"; do
  # the variant's expected value is whatever its own line says
  want=$(python3 - "$f" <<'PY'
import re, sys
t = open(sys.argv[1], encoding="utf-8").read()
m = re.search(r"^[ \t]*sfc_margin:[ \t]*([0-9.eE+-]+)", t, re.M)
print(m.group(1).rstrip("0").rstrip(".") if m else "0")
PY
)
  check "$f" "$want"
done
echo "sfc-margin loader test: $([ $rc -eq 0 ] && echo PASS || echo FAIL)"
exit $rc
