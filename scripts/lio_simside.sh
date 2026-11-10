#!/usr/bin/env bash
# Can the LIO crash be avoided from the SIMULATION side (no change to the LIO source)?
#   a) does the simulator ever publish an empty / near-empty cloud?
#   b) which params file does the node load, and what is `lio.output.pub_step` there?
#   c) what does the LIO's own downsample produce (filter_size_*)?
set -u
echo "== a) min/max points per scan across every sortie log we still have"
for f in /home/fanxin/lio_avoid/logs/missionA-*/lidar_imu.log; do
  v=$(grep -o 'points/scan [0-9]*' "$f" 2>/dev/null | awk '{print $2}' | sort -n | head -1)
  [ -n "${v:-}" ] && printf "   %8s  %s\n" "$v" "$(basename "$(dirname "$f")")"
done | sort -n | head -8
echo "   (a published cloud with 0 points would be the strongest sim-side lever; the smallest we have"
echo "    is above -- the simulator does not emit empty scans in these logs)"
echo
echo "== b) the params file the RUNNING node loads"
ps -eo args | grep '[s]uper_lio_node' | head -1 | tr ' ' '\n' | grep -A1 'params-file' | head -4
for f in /mnt/d/distill_self/lio_avoid/src/lio/config/livox_360.yaml \
         /mnt/d/distill_self/lio_avoid/src/lio-fork-local/config/livox_360.yaml; do
  [ -f "$f" ] || continue
  echo "   -- $f"
  grep -nE 'pub_step|visual_map|visual_dense|filter_size|output' "$f" | head -10
done
echo
echo "== c) the downsample the node runs (ds_undistort_ comes from here)"
for f in /mnt/d/distill_self/lio_avoid/src/lio/config/livox_360.yaml; do
  [ -f "$f" ] || continue
  sed -n '/lio:/,/^[a-z]/p' "$f" | grep -nE 'filter|ds_|downsample|pub_step|visual' | head -12
done
