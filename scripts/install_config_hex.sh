#!/usr/bin/env bash
# Install (and prove) the round-81 SUPER config: robot_r 0.15, no inflation, PVA goals on.
#
#   bash scripts/install_config_hex.sh
#
# THREE copies, and getting only one right fails silently -- so this script now COMPARES them and
# exits non-zero on any mismatch (round 88 audit F7: the old version printed md5sums and always
# returned 0, and `fly_hex.sh` never called it).
#
#   * upstream/SUPER/super_planner/config   -- what the thin layer's --super-config existence check reads
#   * ws_super/src/super_planner/config     -- what fsm_node ACTUALLY loads (it prints the path)
set -eu
ROOT=${DSH_ROOT}
USER_HOME=${DSH_ROOT}/.build
CFG=super_planner_hex.yaml
DIRS="$USER_HOME/upstream/SUPER/super_planner/config $USER_HOME/ws_super/src/super_planner/config"

for d in $DIRS; do
  [ -d "$d" ] || { echo "MISSING config dir: $d" >&2; exit 1; }
  cp "$ROOT/config/$CFG" "$d/$CFG"
  echo "installed -> $d/$CFG"
done

echo "== read back the values that decide behaviour, from the LOADED copy"
for d in $DIRS; do
  echo "-- $d"
  python3 "$ROOT/scripts/patch_yaml.py" "$d/$CFG" --dump | grep -E \
    '^(super_planner\.(robot_r|goal_pva_en|hot_start_en|goal_vel_en|backup_traj_en|planning_horizon)|rog_map\.(resolution|inflation_resolution|inflation_step)|traj_opt\.boundary\.(max_vel|max_acc))'
done

echo "== checksums (repo vs the two loaded copies must agree)"
md5sum "$ROOT/config/$CFG" "$USER_HOME/upstream/SUPER/super_planner/config/$CFG" \
       "$USER_HOME/ws_super/src/super_planner/config/$CFG" 2>/dev/null | sed 's/^/   /' || true

# audit F7: hard comparison of the three copies
cmp -s "$ROOT/config/$CFG" "$USER_HOME/upstream/SUPER/super_planner/config/$CFG" || { echo "MISMATCH upstream copy" >&2; exit 1; }
cmp -s "$ROOT/config/$CFG" "$USER_HOME/ws_super/src/super_planner/config/$CFG" || { echo "MISMATCH ws copy" >&2; exit 1; }
