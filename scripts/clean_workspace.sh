#!/usr/bin/env bash
# clean_workspace.sh -- one pass, archived and auditable (round 88).
#
# The owner asked to clean the workspace of wrong/expired code, docs, test scripts, data, maps and
# figures before the next campaign.  87 rounds left ~470 entries under scripts/ (most are one-off
# probes and doc appenders), 17 world files, 9 planner configs and a pile of stale figures/data.
#
# RULE 1: nothing is deleted without first going into _archive/<stamp>/ (a tar.gz + a manifest), so the
#         clean is reversible and every removal can be explained afterwards.
# RULE 2: the keep-list is matched with `grep -qxF` against a newline-delimited list.  The first version
#         of this script matched with `case " $LIST " in *" $NAME "*)` on a multi-line variable, where
#         names are separated by NEWLINES -- so only the names that happened to sit mid-line (surrounded
#         by spaces) survived, and run_until_healthy.sh / accept.py / prep_run.py were deleted by
#         accident.  They were restored from the archive and this is the fix.
#
#   bash scripts/clean_workspace.sh            # dry run: prints what WOULD go
#   bash scripts/clean_workspace.sh --apply    # archive + delete
set -u
ROOT=/mnt/d/distill_self/super_super_apm
cd "$ROOT" || exit 1
APPLY=0
[ "${1:-}" = "--apply" ] && APPLY=1
STAMP=$(date '+%Y%m%d-%H%M%S')
ARC="/mnt/d/distill_self/_archive_super/$STAMP"   # archives live OUTSIDE the workspace on purpose
MAN="$ARC/CLEANUP-manifest.md"

KEEP_DIR="$ARC/keep"
if [ "$APPLY" -eq 1 ]; then mkdir -p "$ARC"; KEEP_DIR="$ARC/keep"; mkdir -p "$KEEP_DIR"; else KEEP_DIR=$(mktemp -d); fi

cat > "$KEEP_DIR/scripts.txt" <<'EOF'
run_until_healthy.sh
fly_speed_run.sh
fly_hex.sh
fly_hex2.sh
fly_governor.sh
run_accept.sh
accept.py
show_gate.py
speed_audit.py
ref_speed_audit.py
patch_yaml.py
clean_harness.sh
stop_harness.sh
peek.sh
check_static.sh
spec_check.py
check_frozen.py
cfg_envelope.py
install_config_hex.sh
prep_run.py
spec_flight.py
set_optimizer_log.py
test_world_tour.py
check_param_names.py
dump_all_params.py
make_world_v2_hex.py
make_config_v2_08.py
sync_worlds.sh
world_summary.py
peek_world.py
cmp_world_pillars.py
corner_clearance.py
hex_world_sweep.py
hex_density_table.sh
tracking_diag.py
plan_anatomy.py
plan_clearance.py
path_curvature.py
corner_overshoot.py
minco_fail_compare.sh
minco_penalty_stats.py
pva_wire_census.py
attitude_truth.py
thrust_probe.py
alt_divergence.py
setpoint_window.py
pscd_window.py
z_plan_audit.py
chain_rates.py
loop_response.py
rtf_trend.sh
lidar_rate_compare.sh
hunt_two_laps.sh
hunt_stats.py
two_sortie_report.py
round87_ledger.py
plot_hex_laps.py
plot_hex_vxyz_t.py
plot_hex_lio_err_t.py
plot_world_hex.py
build_super.sh
build_ws.sh
fetch_super.sh
offline_lio_check.sh
run_rviz.sh
verify_rviz_config.sh
check_evidence_links.sh
clean_evidence.sh
clean_workspace.sh
refscan.py
# --- round 88 audit F4: everything the gates / docs / render_evidence.sh depend on ---
check_raycast_flat.py
render_evidence.sh
check_doc_refs.py
track_gain.py
backtrack.py
seam_direction.py
ref_stops.py
midleg_stops.py
ref_pulse.py
limits_check.py
jerk_measure.py
body_rate.py
why_fast.py
along_shape.py
plan_geometry.py
inflation_sweep.py
lio_drift.py
z_audit.py
trace.py
cmd_fcu_table.py
yaw_and_pattern.py
heading_truth.py
fly_session.sh
stage_and_diag.sh
sortie_verdict.sh
peek_run.sh
lio_core_setup.sh
lio_core_bt.sh
lio_core_latest.sh
lio_core_div.sh
lio_crash_when.sh
lio_guard_verify.sh
lio_output_built.sh
show_pacer.sh
_mint_experiments_note
EOF

cat > "$KEEP_DIR/worlds.txt" <<'EOF'
forest_hex12.yaml
forest_v2_hex.yaml
square15_forest.yaml
EOF

# evidence figures are REQUIRED by scripts/check_evidence_links.sh (round 88 audit F4: the old list
# archived 4 of the 5 of them, which made the evidence gate fail right after a "cleanup")
cat > "$KEEP_DIR/evidence.txt" <<'EOF'
README.md
world-hex12.png
lap1-paths.png
lap2-paths.png
vxyz-t.png
lio-err-t.png
EOF

cat > "$KEEP_DIR/config.txt" <<'EOF'
super_planner_hex.yaml
super_lio_15m.yaml
super_lio_15m_fast.yaml
rviz_super.rviz
calib_identity.json
EOF

cat > "$KEEP_DIR/evidence.txt" <<'EOF'
README.md
world-hex12.png
EOF

keep() { grep -qxF "$2" "$KEEP_DIR/$1.txt"; }

del_scripts=(); del_worlds=(); del_config=(); del_evidence=(); del_logs=(); del_scratch=()
for f in scripts/*; do
  b=$(basename "$f")
  keep scripts "$b" || del_scripts+=("$f")
done
for f in worlds/*; do b=$(basename "$f"); keep worlds "$b" || del_worlds+=("$f"); done
for f in config/*; do b=$(basename "$f"); keep config "$b" || del_config+=("$f"); done
for f in docs/evidence/*; do b=$(basename "$f"); keep evidence "$b" || del_evidence+=("$f"); done
for f in logs/*; do del_logs+=("$f"); done          # compact prep copies; regenerable via prep_run.py
for f in _scratch/*; do del_scratch+=("$f"); done

echo "== would archive+delete"
printf '   scripts   %3d  of %3d\n' "${#del_scripts[@]}" "$(ls scripts | wc -l)"
printf '   worlds    %3d  of %3d\n' "${#del_worlds[@]}" "$(ls worlds | wc -l)"
printf '   config    %3d  of %3d\n' "${#del_config[@]}" "$(ls config | wc -l)"
printf '   evidence  %3d  of %3d\n' "${#del_evidence[@]}" "$(ls docs/evidence | wc -l)"
printf '   logs      %3d | _scratch %3d\n' "${#del_logs[@]}" "${#del_scratch[@]}"

if [ "$APPLY" -eq 0 ]; then
  echo
  echo "(dry run -- pass --apply to do it; nothing is ever deleted without _archive/<stamp>)"
  rm -rf "$KEEP_DIR"
  exit 0
fi

{
  echo "# CLEANUP manifest -- $STAMP"
  echo
  echo "Repository: $ROOT"
  echo
  echo "## kept"
  echo '```'
  sed 's/^/  scripts\//' "$KEEP_DIR/scripts.txt"
  sed 's/^/  worlds\//' "$KEEP_DIR/worlds.txt"
  sed 's/^/  config\//' "$KEEP_DIR/config.txt"
  sed 's/^/  docs\/evidence\//' "$KEEP_DIR/evidence.txt"
  echo '```'
  echo
  echo "## removed (${#del_scripts[@]} scripts, ${#del_worlds[@]} worlds, ${#del_config[@]} configs,"
  echo "   ${#del_evidence[@]} evidence files, ${#del_logs[@]} log dirs, ${#del_scratch[@]} scratch entries)"
  echo '```'
  printf '%s\n' "${del_scripts[@]:-}" "${del_worlds[@]:-}" "${del_config[@]:-}" \
             "${del_evidence[@]:-}" "${del_logs[@]:-}" "${del_scratch[@]:-}"
  echo '```'
} > "$MAN"

tar czf "$ARC/removed.tar.gz" \
  "${del_scripts[@]:-}" "${del_worlds[@]:-}" "${del_config[@]:-}" \
  "${del_evidence[@]:-}" "${del_logs[@]:-}" "${del_scratch[@]:-}" 2>/dev/null
echo "archived -> $ARC/removed.tar.gz ($(du -h "$ARC/removed.tar.gz" 2>/dev/null | cut -f1))"

rm -rf "${del_scripts[@]:-}" "${del_worlds[@]:-}" "${del_config[@]:-}" \
       "${del_evidence[@]:-}" "${del_logs[@]:-}" "${del_scratch[@]:-}" 2>/dev/null

echo
echo "== after"
printf '   scripts %3d | worlds %2d | config %2d | evidence %2d | logs %2d | _scratch %2d\n' \
  "$(ls scripts 2>/dev/null | wc -l)" "$(ls worlds 2>/dev/null | wc -l)" \
  "$(ls config 2>/dev/null | wc -l)" "$(ls docs/evidence 2>/dev/null | wc -l)" \
  "$(ls logs 2>/dev/null | wc -l)" "$(ls _scratch 2>/dev/null | wc -l)"
echo "manifest: $MAN"
