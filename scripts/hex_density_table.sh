#!/usr/bin/env bash
# Per-leg bottleneck b for a few (seed, keep-prob) combinations of the hex generator.
# b = the widest route's minimum surface clearance on that leg; it is the number that decides whether
# SUPER can plan inside the corridor at all (docs/INTEGRATION.md 71.2 / the tracking diagnosis).
set -u
cd /mnt/d/distill_self/super_super_apm || exit 1
for p in 1.0 0.8 0.6 0.4; do
  for s in 4 20240521; do
    out=$(python3 scripts/make_world_v2_hex.py --seed "$s" --keep-prob "$p" --out /tmp/pp_${s}_${p}.yaml 2>&1)
    n=$(printf '%s\n' "$out" | grep -m1 'pillars ' | sed 's/.*: *//; s/ in .*//')
    mb=$(printf '%s\n' "$out" | grep -e 'pad->c1' -e 'c1->c2' -e 'c2->c3' -e 'c3->c4' \
         -e 'c4->c5' -e 'c5->c6' -e 'c6->c1' | awk '{print $4}' | sort -g | head -1)
    am=$(printf '%s\n' "$out" | grep -e 'pad->c1' -e 'c1->c2' -e 'c2->c3' -e 'c3->c4' \
         -e 'c4->c5' -e 'c5->c6' -e 'c6->c1' | awk '{print $8}' | sort -g | head -1)
    printf 'seed %-9s keep %-4s pillars %-4s  min_b %-8s  min_astar %s\n' "$s" "$p" "$n" "$mb" "$am"
  done
done
