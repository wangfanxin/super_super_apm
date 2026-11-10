#!/usr/bin/env bash
set -u
T=${1:?tag}
R=/home/fanxin/lio_avoid/logs/missionA-$T
echo "=== $T phases ==="
grep -E 'phase ->|ABORT' "$R/supervisor.log" 2>/dev/null | tail -4
echo
echo "=== GOAL lines ==="
grep -E '^GOAL' "$R/traj.log" 2>/dev/null | tail -3
echo
echo "=== arc-sync debug ==="
grep 'arc-sync' "$R/traj.log" 2>/dev/null | tail -8
echo
echo "=== traj.log tail ==="
tail -5 "$R/traj.log" 2>/dev/null
echo
echo "=== setpoints: how far did the plan get ==="
python3 - "$R" <<'PY'
import csv, sys, os
p = os.path.join(sys.argv[1], "setpoints.csv")
if not os.path.exists(p):
    print("  (no setpoints.csv yet)"); raise SystemExit
rows = list(csv.DictReader(open(p)))
if not rows:
    print("  (empty)"); raise SystemExit
xs = [float(r["cmd_x"]) for r in rows]; ys = [float(r["cmd_y"]) for r in rows]
vs = [(float(r["vel_x"]) ** 2 + float(r["vel_y"]) ** 2) ** 0.5 for r in rows]
print("  rows %d, cmd x[%.2f,%.2f] y[%.2f,%.2f], |v| p50 %.2f max %.2f, last t=%.1f s"
      % (len(rows), min(xs), max(xs), min(ys), max(ys),
         sorted(vs)[len(vs)//2], max(vs), float(rows[-1]["t_wall"]) - float(rows[0]["t_wall"])))
PY
