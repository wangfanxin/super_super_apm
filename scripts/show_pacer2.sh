#!/usr/bin/env bash
set -u
cd /mnt/d/distill_self/super_super_apm || exit 1
echo "== where cmd_stream lives"
python3 - <<'PY'
import importlib.util, inspect, sys
spec = importlib.util.spec_from_file_location("ps", "src/whtf/ros/planner_source_super.py")
src = open("src/whtf/ros/planner_source_super.py", encoding="utf-8").read()
import re
m = re.search(r"^(?:from|import)\s+([\w\.]+)\s+as\s+cmd_stream", src, re.M)
print("   import line:", m.group(0) if m else "(not found)")
m2 = re.search(r"(?:from|import)\s+.*cmd_stream.*", src)
print("   any cmd_stream import:", m2.group(0) if m2 else "(none)")
PY
echo
echo "== TourPacer.tick / advance"
F=$(find /mnt/d/distill_self -name 'cmd_stream.py' 2>/dev/null | head -1)
echo "   file: $F"
grep -n 'class TourPacer' -A 70 "$F" 2>/dev/null | sed -n '1,75p'
