#!/usr/bin/env bash
# Regenerate the whole evidence set (the owner's three figure families) from one sortie, in one command.
#
#   bash scripts/render_evidence.sh [TAG]        default TAG = kx1-02-01 (the sortie the docs cite)
#
# TWO THINGS THIS WRAPPER EXISTS FOR
#  1. THE PNGs ARE LOCKED WHILE SOMEONE IS LOOKING AT THEM.  Writing straight into docs/evidence/ fails
#     with `OSError: [Errno 22] Invalid argument` whenever the file is open in a viewer (this is a
#     DrvFs/Windows handle, not a permissions problem).  So everything is rendered into /tmp and then
#     RENAMED into place -- a rename only touches the directory entry and works on an open file.
#  2. THE PLOT SCRIPTS NEED THE COMPACT CSVs.  `prep_run.py` stages them into logs/<tag>/; this does it
#     first so the three commands below are always consistent with each other.
#
# The three families are the only figures allowed (owner, round 83): per-lap path-vs-trajectory top
# views, v_xy/v_z/x/y/z against time, and the estimator error against time.  `world-hex12.png` (the map
# review figure) is drawn by `scripts/plot_world_hex.py` and is not regenerated here.
set -u
cd "$(dirname "$0")/.." || exit 1
TAG=${1:-kx1-02-01}   # 与 docs/evidence/README.md 里的"最好一趟"同一个常量
RUN=${RUN:-${LIO_LOGS}/missionA-$TAG}
STAGE=/tmp/evidence_stage

[ -d "$RUN" ] || { echo "no such sortie: $RUN"; exit 1; }
echo "== staging $TAG"
python3 scripts/prep_run.py --run "$RUN" --out "logs/$TAG" 2>&1 | tail -1

rm -rf "$STAGE"; mkdir -p "$STAGE"
echo "== per-lap top views"
python3 scripts/plot_hex_laps.py --tag "$TAG" --laps 2 --world worlds/forest_hex12.yaml \
        --out "$STAGE" --prefix lap 2>&1 | grep -E '^  lap|^wrote'
echo "== vxyz against time"
python3 scripts/plot_hex_vxyz_t.py --tag "$TAG" --world worlds/forest_hex12.yaml \
        --out "$STAGE/vxyz-t.png" 2>&1 | grep -E '^wrote|^   v_xy'
echo "== estimator error against time"
python3 scripts/plot_hex_lio_err_t.py --tag "$TAG" --out "$STAGE/lio-err-t.png" 2>&1 | grep -E '^wrote|clock'

echo "== installing into docs/evidence (rename, so a viewer holding the old file cannot block it)"
for f in "$STAGE"/*.png; do
  [ -f "$f" ] || continue
  mv -f "$f" "docs/evidence/$(basename "$f")" && echo "   docs/evidence/$(basename "$f")"
done
echo
bash scripts/check_evidence_links.sh 2>&1 | tail -3
