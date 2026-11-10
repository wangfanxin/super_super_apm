#!/usr/bin/env bash
# --- release-tree path bootstrap (git clone anywhere) -----------------------
if [ -z "${DSH_ROOT:-}" ]; then
  _dsh="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  while [ "$_dsh" != "/" ] && [ ! -f "$_dsh/setup/env.sh" ]; do _dsh="$(dirname "$_dsh")"; done
  [ -f "$_dsh/setup/env.sh" ] && . "$_dsh/setup/env.sh"
fi
# -----------------------------------------------------------------------------
# 一次会话 = 两趟飞行（SPEC S6："每次飞行飞两趟"），地图与航点先过冻结检查（SPEC S5）。
#
#   fly_hex2.sh TAG [ATTEMPTS]        e.g.  bash scripts/fly_hex2.sh h2a
#                                     -> sortie tags: h2a-01 (第一趟), h2a-02 (第二趟)
#
# 两趟用**同一套臂、同一张地图**，只换 tag，目的就是看同一件事能不能重复：
# 第 79 轮同一条臂 9 趟里 7 趟 11/0、2 趟 W6 0.61/0.65，所以"一趟通过"不算数。
#
# 任何一条不满足就**不起飞**：
#   * scripts/check_frozen.py  地图/航点没被动过（SHA-256 + 六个坐标）
#   * 单飞锁没被占（/tmp/lio_avoid_sortie_super.lock）
#   * 没有残留的 arducopter / fsm_node / super_lio 进程
set -u
ROOT=${DSH_ROOT}
# THE SCRIPT'S PATHS ARE RELATIVE TO THE PROJECT ROOT (`worlds/forest_v2_hex.yaml`), so it must not
# depend on the caller's cwd.  MEASURED round 85: launched from `_stage/` (via a wrapper that did not cd),
# `check_frozen.py` died on FileNotFoundError and the session refused to fly -- a safe failure, but a
# confusing one, and the same class of bug would have made a *non*-frozen run look like a frozen check.
cd "$ROOT" || exit 1
TAG=${1:-hex}
ATT=${2:-3}
N=${SORTIES:-2}                      # SPEC S6: two sorties per session
LOCK=/tmp/lio_avoid_sortie_super.lock

echo "==================== 第 81 轮会话：${N} 趟   tag 前缀 ${TAG}"
echo "== S5 冻结检查（不过就不飞）"
python3 "$ROOT/scripts/check_frozen.py" || { echo "ABORT: 地图或航点变了，拒绝起飞（SPEC S5）" >&2; exit 2; }
# The frozen check above is ALWAYS about the SPEC map.  A diagnostic sortie may fly a DIFFERENT world
# (round 86: `compare_open.yaml`, the same mission in empty space, to test whether the pillars were ever
# the binding constraint).  Say so loudly, so nobody later reads those rows as a SPEC result.
if [ "${WORLD_NAME:-forest_v2_hex.yaml}" != "forest_v2_hex.yaml" ]; then
  echo
  echo "== DIAGNOSTIC WORLD: ${WORLD_NAME} -- THIS IS NOT THE SPEC MISSION MAP."
  echo "   the frozen map above is intact (that check always runs on it); these rows are a CONTROL"
  echo "   experiment and must not be scored against SPEC S0/S2.  tag ${TAG} is diagnostic."
  echo
fi

if [ -e "$LOCK" ]; then
  echo "ABORT: 单飞锁 $LOCK 还在，另一个 sortie 正在跑" >&2
  exit 3
fi
if pgrep -fa 'build/sitl/bin/arducopter|bin/arducopter|fsm_node|super_lio_node|lidar_imu[.]py' >/dev/null 2>&1; then
  echo "ABORT: 有残留进程，先跑 scripts/clean_harness.sh" >&2
  pgrep -fa 'build/sitl/bin/arducopter|bin/arducopter|fsm_node|super_lio_node|lidar_imu[.]py' | sed 's/^/  /' >&2
  exit 4
fi

RC_ALL=0
for i in $(seq 1 "$N"); do
  T=$(printf "%s-%02d" "$TAG" "$i")
  echo
  echo "==================== 第 ${i}/${N} 趟  tag=${T}  $(date '+%H:%M:%S')"
  bash "$ROOT/scripts/fly_hex.sh" "$T" "$ATT"
  RC=$?
  echo "  sortie ${T} exit=${RC}"
  [ "$RC" -ne 0 ] && RC_ALL=1
  sleep 5
done

echo
echo "==================== 会话结束  $(date '+%H:%M:%S')"
echo "== 两趟的十一行（世界与几何固定，数字变化只可能来自飞机）"
for i in $(seq 1 "$N"); do
  T=$(printf "%s-%02d" "$TAG" "$i")
  D=$(ls -d ${LIO_LOGS}/missionA-"$T"-* 2>/dev/null | head -1)
  echo "-- ${T}  ${D:-<无记录>}"
  [ -n "$D" ] && [ -f "$D/accept.json" ] && python3 - "$D/accept.json" <<'PY'
import json, sys
a = json.load(open(sys.argv[1]))
rows = a.get("rows") or a.get("results") or []
for r in rows:
    print("   %-5s %-46s %s" % (r.get("id", ""), r.get("name", "")[:46], r.get("status", r.get("ok"))))
print("   GATE:", a.get("gate", a.get("verdict", "?")))
PY
done
exit "$RC_ALL"
