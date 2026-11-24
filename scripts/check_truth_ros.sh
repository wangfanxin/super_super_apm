#!/usr/bin/env bash
# check_truth_ros.sh -- Phase 1b pipeline check WITHOUT a flight.
#
# Proves the flight path can be driven by the self-built MAVLink truth client instead of the patched-firmware
# tap:  SITL -> sim/src/sim_state_recv.py --ros  ->  /sim/state (+/sim/state/imu, /sim/truth/odom, /clock,
# /sim/rtf)  ->  sim/src/lidar_imu.py --truth-topic  ->  /livox/lidar + /livox/imu  ->  Super-LIO.
#
# PASS criteria (printed as GATE): lidar cloud >= 9 Hz, IMU >= 190 Hz, raycast p50 < 100 ms,
# skipped scans == 0.  On an idle host MEASURED 2026-11-25: cloud 9.88 Hz, IMU 195.3 Hz, cast 15.8 ms, 0 skipped.
#
#   bash scripts/check_truth_ros.sh [seconds]
set -u
R="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$R" || exit 1
SECS=${1:-18}
. setup/env.sh
set +u
. /opt/ros/humble/setup.bash
. "$LIO_WS/install/setup.bash"
. "$WS_SUPER/install/setup.bash" 2>/dev/null
set -u
D=/tmp/sp_truth_ros
mkdir -p "$D"
BIN=$ARDUPILOT_ROOT/build/sitl/bin/arducopter
SITL=$CLI=$SENSOR=$LIO=""
cleanup() {
    [ -n "$LIO" ] && kill -INT "$LIO" 2>/dev/null
    [ -n "$SENSOR" ] && kill -INT "$SENSOR" 2>/dev/null
    [ -n "$CLI" ] && kill -INT "$CLI" 2>/dev/null
    [ -n "$SITL" ] && kill "$SITL" 2>/dev/null
    sleep 1
    [ -n "$LIO" ] && kill -9 "$LIO" 2>/dev/null
    [ -n "$SENSOR" ] && kill -9 "$SENSOR" 2>/dev/null
    [ -n "$CLI" ] && kill -9 "$CLI" 2>/dev/null
    [ -n "$SITL" ] && kill -9 "$SITL" 2>/dev/null
    return 0
}
trap cleanup EXIT

"$BIN" --model x --speedup 1 -I0 --home -35.363261,149.165230,584,0 \
    --serial0 tcp:5890 --serial1 tcp:5891 --serial2 tcp:5892 --serial3 tcp:5893 \
    > "$D/sitl.log" 2>&1 &
SITL=$!
for _ in $(seq 1 40); do grep -q "bind port 5892" "$D/sitl.log" 2>/dev/null && break; sleep 0.3; done
grep -q "bind port 5892" "$D/sitl.log" || { echo "FAIL: SITL did not start"; tail -5 "$D/sitl.log"; exit 1; }

python3 sim/src/sim_state_recv.py --mav tcp:127.0.0.1:5892 --hz 1000 \
    --home=-35.363261,149.165230,584.0 --csv "$D/sim_state.csv" --ros --publish-hz 200 \
    --tap-csv "$D/tap_from_simstate.csv" --truth-odom --report-hz 5 > "$D/client.log" 2>&1 &
CLI=$!
sleep 3
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1 \
    python3 sim/src/lidar_imu.py --truth-topic /sim/state --world "$SIM_ROOT/worlds/forest_hex12.yaml" \
    --points 12000 --scan-hz 10.0 --imu-hz 200.0 --duration "$SECS" > "$D/lidar.log" 2>&1 &
SENSOR=$!
sleep 2
setsid nohup ros2 run super_lio super_lio_node --ros-args \
    --params-file "$SLAM_ROOT/config/livox_360.yaml" > "$D/lio.log" 2>&1 &
LIO=$!
wait "$SENSOR"
tail -8 "$D/lidar.log"
python3 - "$D/lidar.log" <<'PY'
import re
import sys

text = open(sys.argv[1], "r", errors="replace").read()
scans = re.findall(r"scans\s+(\d+) in [\d.]+ s wall \(([\d.]+) Hz\)", text)
imu = re.search(r"imu messages\s+\d+ \(([\d.]+) Hz\)", text)
cast = re.search(r"raycast\s+([\d.]+) ms/scan", text)
skip = re.search(r"skipped scans\s+(\d+)", text)
cloud = float(scans[-1][1]) if scans else 0.0
imu_hz = float(imu.group(1)) if imu else 0.0
cast_ms = float(cast.group(1)) if cast else 1e9
skipped = int(skip.group(1)) if skip else 999
ok = cloud >= 9.0 and imu_hz >= 190.0 and cast_ms < 100.0 and skipped == 0
print("cloud %.2f Hz (>=9), imu %.1f Hz (>=190), cast %.1f ms (<100), skipped %d (==0) -> %s"
      % (cloud, imu_hz, cast_ms, skipped, "GATE: PASS" if ok else "GATE: FAIL"))
sys.exit(0 if ok else 1)
PY
