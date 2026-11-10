#!/usr/bin/env bash
# Where does the goal advance happen, and on what condition?
set -u
cd /mnt/d/distill_self/super_super_apm || exit 1
F=$(grep -rln 'class TourPacer' src/whtf/ | head -1)
echo "== TourPacer in: $F"
START=$(grep -n 'class TourPacer' "$F" | head -1 | cut -d: -f1)
END=$((START + 80))
sed -n "${START},${END}p" "$F"
