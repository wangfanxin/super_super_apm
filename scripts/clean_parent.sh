#!/usr/bin/env bash
# clean_parent.sh -- remove the junk that earlier PowerShell quoting accidents created next to the
# project (files literally named `|`, `=130`, `done|still`, a directory named `;`, ...) plus the
# one-off probe scripts from those rounds.  Archived first, same rule as clean_workspace.sh.
#
#   bash scripts/clean_parent.sh [--apply]
set -u
P=/mnt/d/distill_self
STAMP=$(date '+%Y%m%d-%H%M%S')
ARC=/mnt/d/distill_self/_archive_super/parent-$STAMP
APPLY=0
[ "${1:-}" = "--apply" ] && APPLY=1

# exact names produced by the accidents (kept literally; several contain | ; = { } or newlines)
items=(
  "$P/1 wire.csv | head -3; echo ===; tail -4 bridge.log; echo ===; cat phase.csv"
  "$P/1.7:,sum(1 for v in z if v>1.7))"
  "$P/1{print"
  "$P/2.0:,sum(1 for v in z if v>2.0),z"
  "$P/26{exit} GCS_Common.cpp"
  "$P/;"
  "$P/=130"
  "$P/cd"
  "$P/done|still"
  "$P/odom|truth_odom|nav_source|NAV_SOURCE"
  "$P/|"
  "$P/_state_probe.py"
  "$P/_v3_apply.py"
  "$P/_v3_probe.py"
  "$P/eeprom.bin"
)

echo "== candidates"
for f in "${items[@]}"; do
  if [ -e "$f" ]; then printf '   %-60s %s\n' "$(basename -- "$f")" "$(du -sh -- "$f" 2>/dev/null | cut -f1)"; fi
done

if [ "$APPLY" -eq 0 ]; then echo; echo "(dry run -- pass --apply)"; exit 0; fi

mkdir -p "$ARC"
: > "$ARC/CLEANUP-parent-manifest.md"
echo "# parent cleanup $STAMP" >> "$ARC/CLEANUP-parent-manifest.md"
echo >> "$ARC/CLEANUP-parent-manifest.md"
for f in "${items[@]}"; do
  [ -e "$f" ] || continue
  echo "  $f" >> "$ARC/CLEANUP-parent-manifest.md"
  tar czf "$ARC/junk.tar.gz" --append --absolute-names "$f" 2>/dev/null \
    || { tar czf "$ARC/junk.tar.gz" --absolute-names "$f"; }
  rm -rf -- "$f"
done
echo "archived -> $ARC/junk.tar.gz"
echo "remaining in $P:"
ls -1 "$P" | sed 's/^/  /'
