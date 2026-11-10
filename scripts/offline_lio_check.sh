#!/usr/bin/env bash
# Does the LIO track a MOVING truth?  Isolated: no SITL, no flight controller, no geofence, no
# arena -- a truth tap (recorded, or synthetic) is replayed over UDP into the real lidar_imu, the
# real Super-LIO consumes it, and the result is compared with the tap it was made from.
#
# ★ ORDERING MATTERS AND IT COST A WRONG VERDICT ★
#   Super-LIO needs ~12 s to come up ("KF init done", then "Map init done").  The first version of
#   this script started the replay FIRST and the LIO last, so on a short tap the LIO only ever saw
#   the tail of it: MEASURED on a 20 s synthetic climb, lio.csv spanned t=12.43..20.00 s -- i.e. the
#   climb (t=6..10 s) was never fed to the estimator at all, while the summary still said
#   "PARKED case ... GATE: PASS".  A check that silently measures the wrong window is worse than
#   no check, so this now WAITS for the estimator to be ready before a single sample is replayed.
#
#   offline_lio_check.sh [tap.csv] [world.yaml] [seconds] [scan_hz] [imu_hz]
set -u
ROOT=/mnt/d/distill_self/super_super_apm
LIO=/mnt/d/distill_self/lio_avoid
CSV=${1:-/home/fanxin/lio_avoid/logs/missionA-supertest-01/tap.csv}
WORLD=${2:-$ROOT/worlds/pillars.yaml}
SECS=${3:-55}
SCAN_HZ=${4:-10}
IMU_HZ=${5:-200}
PORT=14590
OUT=/tmp/offline_lio_$(date +%H%M%S)
mkdir -p "$OUT"

[ -f "$CSV" ] || { echo "no tap csv: $CSV" >&2; exit 2; }
[ -f "$WORLD" ] || { echo "no world: $WORLD" >&2; exit 2; }

set +u
source /opt/ros/humble/setup.bash
source /home/fanxin/lio_avoid/lio_ws/install/setup.bash
set -u
export ROS_LOCALHOST_ONLY=1

echo "tap     : $CSV"
echo "world   : $WORLD"
echo "rates   : ${SCAN_HZ} Hz scan, ${IMU_HZ} Hz IMU, 20000 points (hardware spec)"
echo "out     : $OUT"
echo "loadavg : $(cat /proc/loadavg)"
echo

cleanup() {
  for p in ${REC:-} ${LIO_PID:-} ${SIM_PID:-} ${REP:-}; do
    [ -n "$p" ] && kill "$p" 2>/dev/null
  done
  sleep 1
  for p in ${REC:-} ${LIO_PID:-} ${SIM_PID:-} ${REP:-}; do
    [ -n "$p" ] && kill -9 "$p" 2>/dev/null
  done
}
trap cleanup EXIT INT TERM

# ---- 1. the sensor stream FIRST: Super-LIO cannot initialise without data (it reports
#         "KF init done" / "Map init done" only once samples are flowing), so waiting for it
#         before replaying deadlocks.  The RECORDER is what gets gated on readiness instead, so
#         the comparison never includes the pre-init transient.  MEASURED: with the recorder
#         started blindly 8 s in, lio.csv held only the tail of the tap.
python3 "$ROOT/scripts/tap_replay.py" --csv "$CSV" --port $PORT --rate 1.0 \
    --seconds $((SECS + 20)) > "$OUT/replay.log" 2>&1 &
REP=$!
sleep 1

python3 "$LIO/src/sim/lidar_imu.py" --port $PORT --world "$WORLD" \
    --points 20000 --scan-hz "$SCAN_HZ" --imu-hz "$IMU_HZ" > "$OUT/lidar_imu.log" 2>&1 &
SIM_PID=$!
sleep 2

ros2 run super_lio super_lio_node --ros-args --params-file \
    "$LIO/src/lio/config/livox_360.yaml" > "$OUT/lio.log" 2>&1 &
LIO_PID=$!
echo "waiting for the LIO to report 'Map init done' ..."
ready=0
for i in $(seq 1 90); do
  if grep -q 'Map init done' "$OUT/lio.log" 2>/dev/null; then
    echo "  LIO ready after $((i / 2)) s of data"
    ready=1
    break
  fi
  kill -0 "$LIO_PID" 2>/dev/null || { echo "  the LIO died:"; tail -15 "$OUT/lio.log"; exit 1; }
  sleep 0.5
done
[ "$ready" = 1 ] || { echo "  the LIO never became ready"; tail -15 "$OUT/lio.log"; exit 1; }

python3 "$LIO/src/tools/record-odom.py" --topic /lio/robo/odom \
    --csv "$OUT/lio.csv" --seconds "$SECS" > "$OUT/rec.log" 2>&1 &
REC=$!
wait $REC 2>/dev/null
sleep 1

echo
echo "--- the sim's own rate report (hardware spec is 10 Hz / 200 Hz) ---"
grep -E '^  (scans|points|imu messages|raycast|skipped)' "$OUT/lidar_imu.log" | tail -6 || true
echo
if [ -f "$OUT/lio.csv" ]; then
  f=$(sed -n '2p' "$OUT/lio.csv" | cut -d, -f1)
  l=$(tail -1 "$OUT/lio.csv" | cut -d, -f1)
  n=$(wc -l < "$OUT/lio.csv")
  echo "--- LIO output: $n rows, t_us $f .. $l ($(awk "BEGIN{printf \"%.1f\", ($l-$f)/1e6}") s) ---"
else
  echo "--- no lio.csv (the estimator published nothing) ---"
fi
echo
echo "--- LIO vs the truth it was fed ---"
python3 "$LIO/src/tools/lio-vs-truth.py" --tap "$CSV" --odom "$OUT/lio.csv" 2>&1 | tail -25
echo
echo "(artefacts in $OUT)"
