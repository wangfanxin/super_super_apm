#!/usr/bin/env bash
# --- release-tree path bootstrap (git clone anywhere) -----------------------
if [ -z "${DSH_ROOT:-}" ]; then
  _dsh="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  while [ "$_dsh" != "/" ] && [ ! -f "$_dsh/setup/env.sh" ]; do _dsh="$(dirname "$_dsh")"; done
  [ -f "$_dsh/setup/env.sh" ] && . "$_dsh/setup/env.sh"
fi
# -----------------------------------------------------------------------------
# 只清**我自己这份副本**的残留，按端口而不是按程序名匹配。
#
# ★ 为什么必须这样 ★ 这台机器上同时存在两份 harness 副本：
#     <DSH_ROOT>/harness                （我的；端口已改成 14580/14581/5780/5781）
#     （另一条线的旧树 lio_nav_apm；仍是 14560/14561/5760/5761）
#   它们共用**同一个** SITL 二进制。以前我用 `pkill -f 'bin/arducopter'` 按程序名清，
#   会把对方正在飞的 sortie 一起杀掉（对方那几次 "never left the ground" 里就有我造成的）。
#   现在只按"命令行里带我的端口/我的 planner 路径"来匹配。
set -u
MYPORTS='14580|14581|5780|5781'
MINE='planner_source_super\.py'
# ★ LIO 的命令行里**没有端口**：只有 `--params-file <DSH_ROOT>/slam/config/livox_360.yaml`。
#   只按端口匹配会漏掉它（实测残留两个 super_lio_node 一直占着图）。按"我这副本的配置路径"匹配。
MYLIO="${DSH_ROOT}/slam/config"
MYFSM='super_planner/lib/super_planner/fsm_node|super_planner/fsm_node'
F=${DSH_ROOT}/scripts/run-mission.sh
MYLOCK=$(grep -m1 '^LOCK=' "$F" 2>/dev/null | cut -d= -f2- | tr -d '"' || true)

n=0
for pat in "$MYPORTS" "$MINE" "$MYLIO" "$MYFSM"; do
  # -f 全命令行匹配；把自己排除在外（$$ 与父 shell）
  for pid in $(pgrep -f "$pat" 2>/dev/null); do
    [ "$pid" = "$$" ] && continue
    [ "$pid" = "$PPID" ] && continue
    kill "$pid" 2>/dev/null && n=$((n+1))
  done
done
sleep 2
# 二次强杀：还在的用 -9
# ★ 必须包含 MYFSM（2026-11-26，MEASURED）★  rclcpp 的 fsm_node 捕获 SIGTERM 后走 ROS 关闭，在 rclpy
#   客户端已死的残留图里**不会退出**（实测 pid 5642 一直 Sl，下一次 sortie 的"残留进程"检查永不通过）。
for pat in "$MYPORTS" "$MINE" "$MYLIO" "$MYFSM"; do
  for pid in $(pgrep -f "$pat" 2>/dev/null); do
    [ "$pid" = "$$" ] && continue
    [ "$pid" = "$PPID" ] && continue
    kill -9 "$pid" 2>/dev/null && n=$((n+1))
  done
done
sleep 1
left=$( { pgrep -f "$MYPORTS"; pgrep -f "$MYLIO"; } 2>/dev/null | grep -v -e "^$$\$" -e "^$PPID\$" | sort -u | wc -l )
echo "clean_mine: 杀了 $n 个，残留 $left 条（端口 $MYPORTS + 我的 LIO 配置路径）"
{ pgrep -fa "$MYPORTS"; pgrep -fa "$MYLIO"; } 2>/dev/null | grep -v pgrep | head -3

# 顺手把我自己的锁（如果是死的）清掉；活的锁不动 —— 那是"我正在飞"的标记
if [ -n "${MYLOCK:-}" ] && [ -f "$MYLOCK" ]; then
  pid=$(cut -d: -f1 "$MYLOCK" 2>/dev/null)
  if [ -n "$pid" ] && ! kill -0 "$pid" 2>/dev/null; then
    rm -f "$MYLOCK"; echo "clean_mine: 清掉死锁 $MYLOCK (pid $pid)"
  fi
fi

# ★ DDS 共享内存清理（2026-11-26，MEASURED）★
#   phase16b/17c 的失败共同点：LIO 进程初始化完成、lidar 正常发云，但 LIO **一条点云都没收到**
#   （supervisor 报 "the LIO sample ... is 3.99 s old"，H3 挂、飞机不起飞）。手动 `ros2 daemon stop` +
#   删除 /dev/shm/fastrtps_* 后下一次飞行即恢复正常（16c、13a 等）。Fast-DDS 的共享内存段/端口在
#   大量 sortie 之后会残留，导致新进程之间发现失败。清理它们是幂等的、只影响我自己的会话。
ros2 daemon stop >/dev/null 2>&1 || true
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* /dev/shm/fastdatachannel_* 2>/dev/null || true
