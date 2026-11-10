#!/usr/bin/env bash
# When does the LIO die, and what was the last thing it saw?
set -u
for T in "$@"; do
  R=/home/fanxin/lio_avoid/logs/missionA-$T
  echo "===================== $T"
  crash=$(grep -cE 'Floating point exception|Segmentation fault' "$R/lio.log" 2>/dev/null || true)
  echo "  crash lines in lio.log : $crash"
  # when did the odom stream stop, relative to the sortie?
  python3 - "$R" <<'PY'
import csv, os, sys
R = sys.argv[1]
p = os.path.join(R, "lio.csv")
t_end = None
n = 0
if os.path.exists(p):
    with open(p) as f:
        for row in csv.DictReader(f):
            n += 1
            if n == 1:
                t0 = float(row["t_us"])
            t_end = float(row["t_us"])
print("  lio.csv : %d rows, span %.1f s (if 200 Hz: %.1f s of data)" % (n, (t_end - t0) * 1e-6 if n else 0, n / 200.0))
# the supervisor's abort time, for reference
sup = os.path.join(R, "supervisor.log")
if os.path.exists(sup):
    for line in open(sup):
        if "ABORT" in line or "phase -> done" in line:
            print("  supervisor:", line.strip()[:100])
PY
  echo "  lidar_imu.log tail:"
  grep -E 'scans|GATE|points' "$R/lidar_imu.log" 2>/dev/null | tail -3 | sed 's/^/    /'
  echo "  lio.log tail:"
  tail -3 "$R/lio.log" 2>/dev/null | sed 's/^/    /'
done
