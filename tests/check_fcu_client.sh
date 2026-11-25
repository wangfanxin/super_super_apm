#!/usr/bin/env bash
# check_fcu_client.sh -- no-flight acceptance for the PHASE 3 merged MAVLink client.
#
#   SITL (unpatched, no flight)  <-- ONE MAVLink link -->  bridge_nodes/mavlink/fcu_client.py --bench
#                                                                    ^
#                                        tests/fake_fcu_inputs.py (lio/truth/plan + trigger)
#
# PASS means: ODOMETRY #331 at ~nav-hz over the REAL connection, SET_POSITION_TARGET_LOCAL_NED #84 at
# >=40 Hz once the trigger arrives, the production CSV headers/rows are written, and the extnav AGE GATE
# stops #331 when the LIO stream stops.  No arming, no lidar, no planner: this test is about the merged
# client's transport, frames and gates.
set -u
R="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$R" || exit 1
. setup/env.sh
set +u
. /opt/ros/humble/setup.bash
. "$LIO_WS/install/setup.bash" 2>/dev/null
. "$WS_SUPER/install/setup.bash" 2>/dev/null
set -u
D=/tmp/sp_fcu_client
mkdir -p "$D"
BIN=$ARDUPILOT_ROOT/build/sitl/bin/arducopter
if [ ! -x "$BIN" ]; then
    echo "fcu client bench: SKIP (no SITL binary at $BIN)"
    exit 0
fi
SITL="" ; FAKE=""
cleanup() {
    for p in "$FAKE" "$SITL"; do [ -n "$p" ] && kill "$p" 2>/dev/null; done
    return 0
}
trap cleanup EXIT
"$BIN" --model x --speedup 1 -I0 --home -35.363261,149.165230,584,0 \
    --serial0 tcp:5890 --serial1 tcp:5891 --serial2 tcp:5892 --serial3 tcp:5893 \
    > "$D/sitl.log" 2>&1 &
SITL=$!
for _ in $(seq 1 300); do grep -q "bind port 5890" "$D/sitl.log" 2>/dev/null && break; sleep 0.1; done
python3 tests/fake_fcu_inputs.py --seconds 13 --lio-stop-after 6 > "$D/fake.log" 2>&1 &
FAKE=$!
sleep 1
python3 bridge_nodes/mavlink/fcu_client.py --bench --bench-seconds 10 \
    --mav tcp:127.0.0.1:5890 \
    --calib-file "$R/calib/calib.json" \
    --setpoint-log "$D/setpoints.csv" --wire-log "$D/wire.csv" \
    --phase-log "$D/phase.csv" \
    > "$D/fcu.log" 2>&1
RC=$?
tail -3 "$D/fcu.log"
echo "logs: $D (fcu.log, setpoints.csv, wire.csv)"
exit $RC
