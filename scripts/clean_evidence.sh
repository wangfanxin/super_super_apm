#!/usr/bin/env bash
# --- repo path bootstrap (round 21) ------------------------------------------
if [ -z "${DSH_ROOT:-}" ]; then
  _dsh="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  while [ "$_dsh" != "/" ] && [ ! -f "$_dsh/setup/env.sh" ]; do _dsh="$(dirname "$_dsh")"; done
  [ -f "$_dsh/setup/env.sh" ] && . "$_dsh/setup/env.sh"
fi
# -----------------------------------------------------------------------------
# Round 72: docs/evidence/ had 31 files (8.5 MB) after the per-run figure work.
# Keep only what the current conclusions rest on; archive the rest OUTSIDE the repo first.
set -eu
E=${DSH_ROOT}/docs/evidence
ARC=${DSH_ROOT}/../_evidence_archive_round72.tar.gz
cd "$E"

KILL="cp-01-lap1.png cp-01-lap2.png cp-01-lap3.png cp-01-speed-t.png
pgd-01-lap1.png pgd-01-lap2.png pgd-01-lap3.png pgd-01-speed-t.png
pgf-01-lap1.png pgf-01-lap2.png pgf-01-lap3.png pgf-01-speed-t.png
sc-01-lap1.png sc-01-lap2.png sc-01-lap3.png sc-01-speed-t.png
sc-01-lap1-paths.png sc-01-lap2-paths.png sc-01-lap3-paths.png"

echo "== archiving to $ARC"
tar czf "$ARC" $KILL
echo "   archive: $(du -h "$ARC" | cut -f1)"
rm -f $KILL
echo "== what remains"
ls -la
echo "== count / size"
ls -1 | wc -l; du -sh .
