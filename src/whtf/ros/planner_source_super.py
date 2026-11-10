#!/usr/bin/env python3
"""`planner_source_super.py` —— 把 SUPER 接进 lio_avoid 的 `/plan/cmd` 契约（薄层）。

位置
----
它**不替代**前工程的任何基础设施：SITL、truth tap、LiDAR 仿真、Super-LIO、bridge、supervisor、
录像、指标全部照旧。它只替换"plan box"那一层：

    run-mission.sh 的 PLANNER=<这个文件>
        ├─ 订阅 /lio/robo/odom (L, 200 Hz 位姿) + /lio/odom (L, twist)   ← 与前工程一致
        ├─ 起 super_planner/fsm_node（SUPER 原生 FSM：ROG-Map→A*→CIRI→GCOPTER）
        │     它的输入就是上面两路：rog_map 直接吃 /lio/robo/odom + /lio/cloud_world
        ├─ 航点推进：用前工程的 `cmd_stream.TourPacer`（进盒即切、dwell、超时、完成）
        │     每换一个航点 → 一次 `/planning/click_goal`（W→L，显式用 calib）
        └─ 输出：SUPER 的 `/planning/pos_cmd`(mars) → 转成 `quadrotor_msgs/PositionCommand`
                → `/plan/cmd`，自带 100 Hz 时钟，`trajectory_id` 每航点一次、
                `trajectory_flag` 1 飞行中 / 3 完成（完成后**继续发**，与前工程一致）

为什么是"薄"的
--------------
复用（不重写）：
  · `common.frames.Calib`            L↔W 的唯一换算（`_stage/calib.json`，实测值）
  · `planner.cmd_stream.TourPacer`   到达/悬停/推进/超时的全部判定
  · 前工程的 QoS 习惯（RELIABLE+TRANSIENT_LOCAL 的 5 深度 latched 出口；
    BEST_EFFORT depth 200 的 odom 入口）
新写的只有三件：
  ① 把 SUPER 的参考接进这条流（含"SUPER 还没出参考/断流时发 hold"）
  ② 把航点盒在 W→L 之后发给 SUPER 的 goal 话题
  ③ fsm_node 的起停

★ 帧 ★：**航点在 W 定义，`/plan/cmd` 与 LIO 在 L**。两者只经 `calib` 互转，
**不做任何"拟合到对得上"**（`metrics.py` 的 A1 就是拟合出来的，已确认不能当帧正确性的证据）。

用法（由 run-mission.sh 以 PLANNER= 调用，或手工）：
  python3 planner_source_super.py --calib-file _stage/calib.json \
      --square-m 15 --laps 3 --z 2.0 --arrive-m 1.0 \
      --super-config forest_lio_15m.yaml \
      --super-odom /lio/robo/odom --super-cloud /lio/cloud_world
  python3 planner_source_super.py --selftest        # 离线自测（不需要 ROS）
"""
from __future__ import annotations

# --- release-tree path bootstrap (git clone anywhere) ------------------------
import os as _os


def _dsh_root():
    d = _os.path.dirname(_os.path.abspath(__file__))
    while d != _os.path.dirname(d):
        if _os.path.isdir(_os.path.join(d, "harness")) and _os.path.isdir(_os.path.join(d, "third_party")):
            return d
        d = _os.path.dirname(d)
    return _os.environ.get("DSH_ROOT", "")


_DSH = _os.environ.get("DSH_ROOT") or _dsh_root()
# -----------------------------------------------------------------------------

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time

import numpy as np

# 参考低通（--ref-lp-tau，默认关）：与本文件同目录的纯 numpy 实现，便于单测
# （scripts/test_ref_lp.py 直接 import 它，不需要 rclpy）。
# ★ 必须先把本文件自己的目录放进 sys.path ★：spec_check.py 是用 importlib 按路径导入本模块的
#   （它要调 world_tour() 核对航点），那种导入方式**不会**把本目录放进 sys.path —— 实测
#   第 20 轮第一次这么做时 spec_check 报 `No module named 'ref_lp'`，静态闸门当场拒飞
#   （`S0 两圈航线 ... No module named 'ref_lp'`）。闸门工作正常，修的是这里。
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
import ref_lp

#: 前工程根目录（只为复用它的 common/planner 两个包；本文件不属于它）
LIO_ROOT_DEFAULT = _DSH + "/harness"

#: SUPER 的 mars 消息与 bridge 要的 quadrotor 消息是**两个不同的包**，必须显式转。
MARS_PKG = "mars_quadrotor_msgs"


def add_lio_path(root):
    src = os.path.join(root, "src")
    if src not in sys.path:
        sys.path.insert(0, src)


# --------------------------------------------------------------------------------------- 航点几何
def world_tour(world_path, laps, z, verbose=True):
    """世界文件里的 `mission.corners` × `laps` 圈，**W 系**。

    ★ 第 87 轮的 bug，和它为什么在这里 ★
      这个函数原来只有一种行为：把世界文件的 `mission.corners` **全部**返回。第 87 轮把飞行臂的
      `LAPS` 从 3 改成 2（用户："飞完两圈"），臂把 `--laps 2` 老老实实传了进来，可这一层**根本没读它**
      —— 它照旧发 18 个角点 + 停机坪 = **19 个目标**，`accept.py` 的 H4 也就一直报 "19 goals issued"。
      最刺眼的是那行打印：它自己算 `len(_cs) // laps` 当"每圈角点数"来显示，于是打出
      "18 waypoints (6 per lap × 2 laps)" —— 一句话里同时写着 6×2=12 和 18。
      所以第 87 轮那些"两圈"的趟，其实一直是三圈。

      现在按世界文件自己声明的圈数截断：世界文件里有 `mission.laps = 3`（那是**冻结的地图**，
      SPEC S5 不许改），`mission.corners` 是 3 圈 × 6；飞行臂要 2 圈就取前 2 × 6 = 12 个，
      再加停机坪竖直腿 = 13 个目标。**地图不动，圈数在臂这边。**

      世界文件没声明 `mission.laps` 时保持旧行为（全部角点）——老的世界文件不受影响。

    返回 (corners_w: (N,3) float, info: dict)。
    """
    import yaml as _yaml
    _wd = _yaml.safe_load(open(world_path, encoding="utf-8"))
    _mis = _wd.get("mission") or {}
    _cs = _mis.get("corners") or []
    if not _cs:
        raise ValueError("%s has no mission.corners" % world_path)
    declared = int(_mis.get("laps", 0) or 0)
    n_all = len(_cs)
    n_want = n_all
    info = {"declared_laps": declared, "corners_in_world": n_all, "corners_per_lap": 0,
            "laps_requested": int(laps), "corners_flown": n_all, "truncated": False}
    if declared > 0 and n_all % declared == 0:
        per = n_all // declared
        info["corners_per_lap"] = per
        if int(laps) < declared:
            n_want = per * max(1, int(laps))
            info["truncated"] = True
        elif int(laps) > declared:
            if verbose:
                print("   WARNING: --laps %d > the world's %d laps; flying the %d corners the "
                      "world file declares (a lap cannot be invented)" % (int(laps), declared, n_all),
                      flush=True)
    elif declared > 0 and verbose:
        print("   WARNING: mission.corners has %d entries, not a multiple of the declared %d laps; "
              "flying all of them" % (n_all, declared), flush=True)
    corners_w = np.array([[float(c[0]), float(c[1]), float(z)] for c in _cs[:n_want]], dtype=float)
    info["corners_flown"] = len(corners_w)
    if verbose:
        if info["truncated"]:
            print("   TOUR TRUNCATED: world declares %d laps (%d corners, %d per lap); flying "
                  "%d laps = %d corners (+ the pad vertical leg)"
                  % (declared, n_all, info["corners_per_lap"], int(laps), len(corners_w)), flush=True)
        else:
            print("   tour from the world file: %d corners%s"
                  % (len(corners_w), "" if declared <= 0 else " (%d laps)" % declared), flush=True)
    return corners_w, info


def square_tour(side_m, laps, z, cx=0.0, cy=0.0, first_hold_s=3.0, vertical_first_m=None):
    """15 m 正方形 × laps 圈的 12 个角点，**W 系**（x=E, y=N, z=U，原点=起飞点）。

    ★ `vertical_first_m` 不是可选项，是这个 harness 的起飞机制 ★
      `run-mission.sh` 的 banner 会检查 planner 的 argv 里有没有"竖直腿"：
          NO vertical leg in custom's argv: the first setpoint is the planner's own first target
      supervisor 的说明也写着 "the go goes out on airborne=True and **the plan's first target is a
      vertical leg**"。也就是说：第一条指令必须是**停机坪正上方**的一个点，飞机爬上去 → truth 离地 →
      supervisor 才发 `/plan/trigger` → bridge 才开始发定点 → 才有闭环。
      我第一版没给这一段，结果 `setpoints.csv` 一行数据都没有，supervisor 报
      "ABORT: never left the ground (truth 0.100 m after 12.0 s)"。
    给了之后：targets = [正上方 (0,0,vertical_first_m)] + 12 个角点。
    """
    h = side_m / 2.0
    ring = np.array([[cx - h, cy - h], [cx + h, cy - h], [cx + h, cy + h], [cx - h, cy + h]])
    pts = [ring for _ in range(max(1, laps))]
    corners = np.vstack(pts)
    corners = np.hstack([corners, np.full((len(corners), 1), float(z))])
    if vertical_first_m is not None:
        pad_above = np.array([[cx, cy, float(vertical_first_m)]])
        corners = np.vstack([pad_above, corners])
    return corners, first_hold_s


# --------------------------------------------------------------------------------------- 离线自测
def selftest() -> int:
    bad = []
    print("== 1) 航点生成 ==")
    c, hold0 = square_tour(15.0, 3, 2.0)
    print("   corners %d, 前 4 个：\n%s" % (len(c), np.round(c[:4], 3)))
    if len(c) != 12:
        bad.append("15 m × 3 圈应是 12 个角点，得到 %d" % len(c))
    if abs(np.linalg.norm(c[0][:2] - c[2][:2]) - 15.0 * math.sqrt(2)) > 1e-9:
        bad.append("对角点距离不是 15√2")
    for i in range(1, len(c)):
        if abs(np.linalg.norm(c[i][:2] - c[i - 1][:2]) - 15.0) > 1e-9:
            bad.append("第 %d 段不是 15 m" % i)

    print("== 2) W→L→W 往返（用 README pin 的那个标定）==")
    yaw = math.radians(90.049771)
    R_WL = np.array([[math.cos(yaw), -math.sin(yaw), 0.0],
                     [math.sin(yaw), math.cos(yaw), 0.0],
                     [0.0, 0.0, 1.0]])
    t_WL = np.array([-0.00080, -0.00409, 0.11557])
    R_LW = R_WL.T
    cl = (R_LW @ (c - t_WL).T).T
    back = (R_WL @ cl.T).T + t_WL
    err = float(np.abs(back - c).max())
    print("   L 系里 x∈[%.2f,%.2f] y∈[%.2f,%.2f]  往返最大误差 %.3e m"
          % (cl[:, 0].min(), cl[:, 0].max(), cl[:, 1].min(), cl[:, 1].max(), err))
    if err > 1e-9:
        bad.append("W→L→W 往返误差 %.3e m 太大" % err)

    print("== 3) 帧方向自检：W 的 +x（东）在 L 里应转到 -y ==")
    e_w = np.array([1.0, 0.0, 0.0])
    e_l = R_LW @ e_w
    print("   W+x → L (%.4f, %.4f, %.4f)" % tuple(e_l))
    if abs(e_l[1] + 1.0) > 1e-6:
        bad.append("+90° 标定下 W 的 +x 应落到 L 的 -y，得到 %s" % np.round(e_l, 4))

    print("== 3b) 与 bridge 兜底的一致性（这条是那次飞歪的根因）==")
    # bridge 无 --calib-file 时用 Calib.predicted(0.0, world_axes="body")；薄层必须用同一份。
    Rp = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])   # R_z(+90°)
    tp = np.array([0.0, 0.05, 0.10])
    c0 = square_tour(15.0, 1, 2.0)[0][0]                                  # W 的第一个角点 (-7.5,-7.5,2)
    l0 = Rp.T @ (c0 - tp)                                                 # 薄层发给 SUPER/bridge 的 L 点
    back0 = Rp @ l0 + tp                                                  # bridge 的 L→W
    print("   W 角点 %s → 薄层发 L %s → bridge 还原 %s  (误差 %.2e m)"
          % (np.round(c0, 3), np.round(l0, 3), np.round(back0, 3),
             float(np.linalg.norm(back0 - c0))))
    if float(np.linalg.norm(back0 - c0)) > 1e-9:
        bad.append("薄层与 bridge 的兜底标定互逆性不成立")

    print("== 4) 契约字段 ==")
    print("   trajectory_id：每航点 +1（0..%d）；trajectory_flag：飞行中 1 / 完成 3；frame_id=L" % (len(c) - 1))
    print()
    if bad:
        print("SELFTEST FAIL")
        for b in bad:
            print("  -", b)
        return 1
    print("SELFTEST PASS")
    return 0


# --------------------------------------------------------------------------------------- 主程序
def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--lio-root", default=LIO_ROOT_DEFAULT, help="前工程根目录（复用其 common/planner）")
    ap.add_argument("--world", default="", help="仅用于打印；SUPER 的云由 LIO 提供")
    ap.add_argument("--calib-file", default="", help="L→W 标定（_stage/calib.json）；不给则用单位阵并告警")
    ap.add_argument("--square-m", type=float, default=15.0)
    ap.add_argument("--laps", type=int, default=3)
    ap.add_argument("--z", type=float, default=2.0, help="任务高度（W，米）")
    ap.add_argument("--first-hold-s", type=float, default=3.0)
    ap.add_argument("--vertical-first-m", type=float, default=None,
                    help="★ 起飞机制：先在停机坪正上方给出这个高度的竖直腿（不给=这条 harness 起不来）")
    ap.add_argument("--hold-s", type=float, default=-1.0, help="每个角点的 dwell；-1 = 不 dwell")
    ap.add_argument("--arrive-m", type=float, default=1.0, help="进盒即切的半径（盒半径 0.25 的宽松版）")
    ap.add_argument("--settle-speed", type=float, default=1.5)
    ap.add_argument("--settle-s", type=float, default=0.5)
    ap.add_argument("--leg-timeout-s", type=float, default=30.0)
    ap.add_argument("--odom", default="/lio/robo/odom")
    ap.add_argument("--odom-vel-topic", default="/lio/odom")
    ap.add_argument("--cmd-topic", default="/plan/cmd")
    ap.add_argument("--cmd-rate", type=float, default=100.0)
    ap.add_argument("--goal-topic", default="/planning/click_goal")
    ap.add_argument("--super-cmd-topic", default="/planning/pos_cmd")
    ap.add_argument("--super-config", default="super_lio_15m.yaml",
                    help="SUPER 的 config_name（必须在 super_planner/config/ 下真实存在）")
    ap.add_argument("--super-config-dir", default=_DSH + "/third_party/SUPER/super_planner/config",
                    help="启动前检查 --super-config 是否存在的目录")
    ap.add_argument("--ref-clamp-m", type=float, default=1.2,
                    help="单拍参考位移上限（两个参考源切换时防跳变；0=不限）")
    ap.add_argument("--ref-no-retract", type=int, default=0,
                    help="1 = 参考**永不后退**：若新参考落在当前指令点的后方（沿当前前进方向的分量为负），"
                         "就保持当前指令点不动，直到新规划追上来。0=关（老行为）。")
    ap.add_argument("--ref-gov-kp", type=float, default=0.0,
                    help=">0 时启用参考调速器：把参考变成真正的一条轨迹（位置与速度自洽、处处连续），"
                         "而不是每拍夹一下。状态 p_ref，每拍 v = clamp(v_super + kp*(p_super - p_ref), "
                         "REF_MAX)，p_ref += v*dt，同时发 (p_ref, v)。这一项是为 BRIDGE_FIELDS=posvel "
                         "准备的前提：实测 qa-01 在这种组合下真值冲到 7.12 m/s、高度到 3.58 m"
                         "（飞控的加速度受限参考模型被自相矛盾的前馈推起来了）。"
                         "kp 是收敛速率（1/s），kp=4 对应 0.25 s 时间常数。0=关（老行为）。")
    ap.add_argument("--ref-gov-acc", type=float, default=0.0,
                    help="参考调速器输出的加速度上限，m/s^2（0=不限）。这就是前馈钳位壳那一层："
                         "ArduPilot 的 posvel 通道把 (p,v) 喂进一个加速度受限参考模型，前馈的一阶导"
                         "（即加速度）跳变会把飞机踢出去 —— pz/qb 四次摔出围栏都是这个机制。"
                         "限住 dv/dt 之后，前馈才既能用、又不会把参考模型推飞。"
                         "只有 --ref-gov-kp>0 时才有意义。")
    ap.add_argument("--ref-gov-along", type=int, default=0,
                    help="1 = 调速器只做**沿航迹**调速（推荐）：每拍取 SUPER 自己的前进方向 u，"
                         "只允许 p_ref 沿 u 前进（速度 = clamp(kp*(Δ·u), 0, vmax)），因此"
                         "**横向偏差恒为 0、永不切角**；0 = 老的「朝规划点追」（一阶滤波，会切角）。"
                         "实测依据：chase 模式在 kp=6 时滞后 v/kp=0.75 m 是纯横向的，指令净空 p1 掉到 "
                         "0.084 m 而规划给了 0.325 m，于是接触；kp=20 时滞后只剩 0.22 m 但指令过于忠实、"
                         "飞机跟不上（missionA-rp-01：W6 残差 0.84-1.11，时间基从 -0.85 s 缩到 -0.15 s，"
                         "说明那 0.85 s 本来就是滤波器滞后）。沿航迹模式同时拿到不切角和跟得上。")
    ap.add_argument("--ref-ff-gain", type=float, default=1.0,
                    help="速度前馈的放大系数（0=完全不用前馈，1=原样）。实测前馈在这里会与位置环"
                         "**叠加**：指令峰值 ≤4.50 m/s、真值峰值却是 6.18~7.31 m/s，而零延迟 "
                         "|cmd−truth| 是 0.67~0.89 m（p90 1.1~1.7 m）—— 位置误差被位置环乘成一个速度"
                         "修正，压在前馈之上。增益 0.5 就是「只借一半前馈」，用来判断超速是否这个叠加。")
    ap.add_argument("--ref-along-floor", type=float, default=0.0,
                    help="沿航迹模式下参考速度的下限，取规划速度的这个比例（0 = 关）。ke1-01-01 实测："
                         "投影律每趟停 44 次（19 次贴角点），每次之后飞机冲到 2-3 m/s 再追。")
    ap.add_argument("--ref-gov-dec", type=float, default=0.0,
                    help="参考**减速**的加速度上限（m/s²，0 = 与 --ref-gov-acc 相同）。实测 plan 会在腿中间"
                         "把速度掉到 0.2 m/s，参考一拍跟下去、飞机就冲过头；把下降沿限到 2.0 后同样的"
                         "下降变成 0.6 s 的斜坡，而加速不受影响。")
    ap.add_argument("--ref-catchup-mps", type=float, default=0.5,
                    help="参考最多可以比规划自己快多少（m/s）。这是 ke2-01-01 那个'追缺口追到 6 m/s、"
                         "飞出场地'的直接修复：参考只许追赶，不许超过它追的那条轨迹太多。")
    ap.add_argument("--ref-gov-ks", type=float, default=4.0,
                    help="弧长同步的增益（1/s），只在 --ref-gov-along 2 时用。参考速度 = 规划速度 + "
                         "ks×(规划弧长 − 参考弧长)；规划在动时它就不会归零（老的一阶投影律在拐弯必停，"
                         "实测 44 次/趟、集中在六个角点）。")
    ap.add_argument("--ref-ff-acc-max", type=float, default=0.0,
                    help="参考加速度前馈的上限（m/s²）。0 = 用规划器的 max_acc（由飞行臂按配置传入）。"
                         "为什么需要它：本文件以前把 a 一律发 0（实测 wire.csv 全程 |a| = 0.000），"
                         "于是飞控只能靠位置/速度误差去'制造'加速度，沿航迹残差因此 p90 0.61 m。"
                         "发布 a = dv/dt 后 (p, v, a) 仍是同一条轨迹（自洽），且天然被 --ref-gov-acc 限住。")
    ap.add_argument("--ref-lead-s", type=float, default=0.0,
                    help="命令超前量（秒）：把 SUPER 的点沿它自己的速度方向外推这么多秒再喂给调速器。"
                         "为什么要它：本机位置环实测滞后 0.3-0.4 s，参考若就是'现在'，飞机永远"
                         "落后 v*0.4 m（ja1-01-01 的 W6 p50 0.418 m）。0 = 关（旧行为）。")
    ap.add_argument("--ref-lead-turn-rad-s", type=float, default=1.5,
                    help="规划转向角速度超过这个值（rad/s）时把超前量压到 0 —— 拐角处不超前，"
                         "免得把'抄近道'引回来（柱子就在拐角）。")
    ap.add_argument("--ref-lp-tau", type=float, default=0.0,
                    help="发布的参考 (p, v, a) 经二阶临界阻尼低通，时间常数 τ 秒（0 = 关，默认）。"
                         "依据 docs/INTEGRATION.md §99.3–§102：九趟飞控手臂都压不下 W6 p90 0.34 m，"
                         "而离线把命令流低通到 τ=0.05–0.10 s 时真值-命令 p90 可到 ~0.26 m。"
                         "τ=0 严格直通（scripts/test_ref_lp.py 断言），实现见 ref_lp.py。")
    ap.add_argument("--ref-gov-maxlag", type=float, default=0.0,
                    help="参考调速器允许落后规划点多少米（0=不限）。一阶滤波器在拐弯处必然内切 —— "
                         "而柱子就在拐弯处。实测 missionA-qi-01/qh-01：调速器输出的**指令**净空 p1 只剩 "
                         "0.084 m、8.3%% 的样本进了规划自己的 0.325 m 余量（验收档 pw-01 是 0.19%%），"
                         "于是飞机在指令已经贴柱子的地方再偏 0.8 m 就撞了。限住滞后 = 限住切角。")
    ap.add_argument("--ref-ff", type=int, default=0,
                    help="1 = 被限幅的那一拍把速度前馈改成限幅后的滑移速率 (p_new−p_prev)/dt；"
                         "0 = 保留 SUPER 自己给的速度（默认）。实测：1 会让参考速度**超过规划器自己的上限**"
                         "（pv-01：max_vel 5.0 的配置里参考峰值 6.00 m/s），而 W6 的残差是滞后×速度，"
                         "所以默认 0。")
    ap.add_argument("--ref-max-speed-mps", type=float, default=6.0,
                    help="参考的**滑移速率**上限，m/s。这才是真正的限幅：100 Hz 下 1.2 m/拍 = "
                         "120 m/s 的目标滑移，等于没限（实测 missionA-ms-03：参考跳 4.3 m，"
                         "飞机压坡度追它、掉高度触地）。6 m/s 把 4.3 m 抹成 0.7 s；"
                         "4 m/s 的真实巡航只走 0.04 m/拍，不会被削。0=不限。")
    ap.add_argument("--ref-max-speed-z-mps", type=float, default=1.5,
                    help="参考**竖直**滑移速率上限，m/s。竖直不能靠压坡度补救，油门一顶就冲过头："
                         "实测 mz-02/03 在 6 m/s 三维限幅下 U 仍冲到 3.13/3.35 m（指令 2.0）。"
                         "1.5 m/s 把 1.8 m 垂直腿摊到 1.2 s，落在 WP_SPD_UP 2.5 m/s 之内。0=不限。")
    ap.add_argument("--ref-z-hold", type=float, default=0.0,
                    help="把参考的 z 限制在 --ref-z-hold ± --ref-z-tol 里（0=关）。平方航线是定高飞行，"
                         "而 SUPER 的三维优化器在窄走廊里会让 z 上下摆：实测 v2d-01 的参考 z 在"
                         "0.50~3.50 m 之间摆（真值 p95 偏高 +0.82 m，W3 差 0.02 m 不过），"
                         "而任务高度恒为 2.0 m。只在飞机已经爬到位（--ref-z-latch-m）后才生效，"
                         "所以垂直起飞段不受影响。")
    ap.add_argument("--ref-z-tol", type=float, default=0.20,
                    help="--ref-z-hold 的窗口半宽，m。要大于实测跟踪误差、小于 W3 的 ±0.30 m 门。")
    ap.add_argument("--ref-z-latch-m", type=float, default=1.60,
                    help="飞机（估计）高度超过这个值后锁存 --ref-z-hold，m。")
    ap.add_argument("--entry-max-speed-mps", type=float, default=0.0,
                    help="发射段（pad 与第一条腿）的参考限速，m/s；0=不限。起飞后的第一段是从悬停直接"
                         "接 4-5 m/s，飞机滞后 0.3-0.5 s，实测开头 10-20 s 的 |cmd−truth| p90 是"
                         "0.71-0.92 m 而全程只有 0.55-0.60（v3h-01，scripts/residual_segment.py），"
                         "W6 的四等分判据正是被这一段顶掉的（第一段 0.81 vs 0.56/0.51/0.58）。"
                         "发射段没有速度指标，慢一点不损失什么。")
    ap.add_argument("--goal-lookahead-m", type=float, default=0.0,
                    help="把发给 SUPER 的目标点沿**去向**前推这么多米，使轨迹的终点不在角点上 —— "
                         "SUPER 每条轨迹都把终点速度设为 0（pos_fina_state.setZero()），"
                         "所以『把角点当目标』必然要求停在角点上。实测 ph-01：离角点 2 m 内占 40%%"
                         "的路程、速度只有 1.77 m/s；max_acc 4.5→9.0 只把峰值 4.43→6.44、均值仍 2.45。"
                         "必须配合 --pass-by-m，否则切角会让 pacer 永远判不到到达。0=关（老行为）。")
    ap.add_argument("--pass-by-m", type=float, default=0.0,
                    help="『越过航点』判据的横向容差，m。>0 时到达 = **已越过航点所在平面**且横向偏差"
                         "≤该值（不是『进入半径』）。半径判据在高速切角时永远触发不了：90° 角、"
                         "转弯半径 R 的切角量是 R*(sqrt2-1)，5 m/s / max_acc 9 时约 1.2 m。"
                         "实测 missionA-pl-01：目标前推 6 m 后，飞机飞了 300 s 只算到 1 个航点。"
                         "0=关（老行为）。")
    ap.add_argument("--pass-by-long-m", type=float, default=0.0,
                    help="『越过航点』判据的**沿航迹**上界，m。0=老行为（半平面：along>=0 无上界）——"
                         "这个形状在目标与出弯方向共线时会误判：实测 missionA-cp-01 里"
                         "『ARRIVED at waypoint 2: the vehicle is 13.780 m from it』，一圈误判两次，"
                         "自报 13/13 而真值只到 7/13（scripts/verify_waypoints.py）。>0 时判据变成"
                         "一个盒子：0 <= along <= 该值 且 lateral <= pass-by-m。")
    ap.add_argument("--goal-pva", type=int, default=0,
                    help="1 = 用 PVA goal（mars PositionCommand 到 <goal-topic>_pva）：goal 仍是角点，"
                         "但把**离开角点的速度**一起给 SUPER，于是轨迹终点不是静止点、转角不停车。"
                         "需要 SUPER 打过 scripts/patch_goal_pva.py 并配置 super_planner/goal_pva_en=true。")
    ap.add_argument("--clr-gov-margin", type=float, default=0.0,
                    help="★ 净空调速 ★ >0 时启用：v ≤ (c − margin)/0.154，c = 参考点在前进方向上一小段内的"
                         "最小柱面距离（0.154 是实测的横向偏差-速度斜率，§37.2）。只降速、不改路径。"
                         "推荐 0.10–0.15；0=关（老行为）。世界几何从 --world 读。")
    ap.add_argument("--clr-gov-look-m", type=float, default=0.40,
                    help="净空调速的前视距离（m）：在前视段内取最小净空，避免进洞才发现。"
                         "0.4 m ≈ 0.1 s @ 4 m/s。")
    ap.add_argument("--time-scale-vmax", type=float, default=0.0,
                    help="时间缩放后参考速度的上限（m/s）。0 = 用薄层限幅。k>1 会让参考整体变快，"
                         "超过规划自己的上限就是失控的另一种写法。")
    ap.add_argument("--time-scale-drift", type=float, default=0.35,
                    help="锁相漂移保护（s）：缩放后的参考在轨迹时间上领先实际参考超过该值就重新锁定。"
                         "太大 → 参考跑到另一段路径上；太小 → 每次锁定都回到 1× 速度，白做。")
    ap.add_argument("--time-scale-gate", type=float, default=0.10,
                    help="同步门槛（m）：先按 k=1 求值多项式，与 SUPER 这条 100 Hz 参考比；"
                         "差超过该值就这一拍回退到参考（换目标瞬间的旧轨迹保护）。")
    ap.add_argument("--time-scale", type=float, default=1.0,
                    help="★ 时间缩放 ★ >1 时按倍速求值 SUPER 自己的多项式（`--poly-topic`），即**同一条路径、"
                         "时间压缩 k 倍**：路径不变，所以净空与 W1b 不变。实测同路径的时间最优是 3.53 m/s"
                         "而规划只给 2.89（§46.2）。1.0=关（老行为）。依赖 --poly-dt 的时钟校准。")
    ap.add_argument("--poly-dt", type=float, default=0.30,
                    help="多项式求值的时间偏移（s）。第 69 轮实测：在 +0.30 s 处 |poly − 100Hz 参考| = "
                         "1.2–3.5 cm（`POLYFIT` 日志），其它偏移都在分米级以上。")
    ap.add_argument("--poly-topic", default="/planning_cmd/poly_traj",
                    help="SUPER 自己提交的多项式轨迹（mars PolynomialTrajectory）。--poly-check / "
                         "--time-scale 用它求值；路径不改，只改时间参数化。")
    ap.add_argument("--poly-check", type=int, default=0,
                    help="1 = 每秒把多项式在当前时刻求值，与 100 Hz 参考对比并打印误差（零行为改动，"
                         "用来确认系数排布与时钟对得上）。")
    ap.add_argument("--pre-goal-m", type=float, default=0.0,
                    help="★ 离当前角点还剩该距离时，把**下一个**角点发给 SUPER ★ 于是当前角点成为轨迹的"
                         "内部点、不在那里停车；而 A* 只有这么点余量，切不出大弯（与 --goal-lead 1 不同，"
                         "后者在 15 m 外换目标，实测切掉 8-9 m、W1b 7/13）。到达判据不变。0=关。")
    ap.add_argument("--corner-mode", default="outgoing",
                    choices=("outgoing", "bisector", "incoming"),
                    help="PVA goal 的终点速度方向：outgoing=沿出弯腿（实测 90° 角不可行，pv2-01 95 次优化失败）；"
                         "bisector=来向与去向的角平分线（90° 角 ⇒ 45°，推荐试）；incoming=沿来向（对照）。")
    ap.add_argument("--corner-speed-list", default="",
                    help="每个角的安全速度，逗号分隔，按圈循环（例如 \"2.0,2.4,2.0,2.4\"）。"
                         "空=用 --corner-speed。来源：scripts/hex_corner_speeds.py 的 v_safe。")
    ap.add_argument("--corner-speed", type=float, default=2.0,
                    help="PVA goal 的出弯速度大小，m/s。由几何决定：本世界两个紧角净空圆角 R=0.95 m，"
                         "v=sqrt(a_lat*R)≈2.0（scripts/corner_room.py）；2.5 m/s 时深偏差 0.40 m，"
                         "仍小于紧角的参考净空 0.45 m，但余量很薄。")
    ap.add_argument("--goal-lead", type=int, default=0,
                    help="★ 让当前角点成为轨迹的**内部点** ★ 发给 SUPER 的 goal 提前 k 个航点，"
                         "pacer 仍然量到当前角点（到达判据不变），而 SUPER 规划的是一条跨过该角点、"
                         "终点在下一个角点的轨迹 —— 终点不在角点上，参考就不会在那里降到 0。"
                         "转角速度由几何决定：实测本世界两个紧角的净空圆角 R=0.95 m，"
                         "v=sqrt(a_lat*R)≈2.0 m/s，宽角 3.5-4.3 m/s。0=老行为（goal=当前角点，必停）。"
                         "⚠ 实测 missionA-cp-01：lead=1 会**抄近角**（真值到角点 8.3-9.2 m），"
                         "W1b 只有 7/13 —— 角点必须仍然是 goal，见 docs/INTEGRATION.md §41。")
    ap.add_argument("--super-odom", default="/lio/robo/odom")
    ap.add_argument("--super-cloud", default="/lio/cloud_world")
    ap.add_argument("--tour", choices=["square", "world"], default="square",
                    help="square = the legacy --square-m tour; world = read mission.corners from --world "
                         "(the hexagon arm uses this, so the waypoints and the map have one source)")
    ap.add_argument("--no-super", action="store_true", help="不自己起 fsm_node（调试用）")
    ap.add_argument("--stale-s", type=float, default=1.0, help="SUPER 断流多久后改发 hold")
    ap.add_argument("--log-period-s", type=float, default=5.0)
    ap.add_argument("--selftest", action="store_true")
    args, _unknown = ap.parse_known_args()

    if args.selftest:
        return selftest()

    # 启动横幅：把"这一趟到底开了什么"写进日志。一个没被读到的开关和一个没效果的开关长得一样
    # （本项目已为此付过两次学费），所以凡是被命令行打开的参考整形项都在这里报一行。
    print("ref shaping: gov_kp=%.2f gov_acc=%.2f gov_dec=%.2f gov_along=%d cap=%.2f lead=%.2f "
          "lp_tau=%.3f"
          % (float(getattr(args, "ref_gov_kp", 0.0) or 0.0),
             float(getattr(args, "ref_gov_acc", 0.0) or 0.0),
             float(getattr(args, "ref_gov_dec", 0.0) or 0.0),
             int(getattr(args, "ref_gov_along", 0) or 0),
             float(getattr(args, "ref_catchup_mps", 0.0) or 0.0),
             float(getattr(args, "ref_lead_s", 0.0) or 0.0),
             float(getattr(args, "ref_lp_tau", 0.0) or 0.0)), flush=True)

    add_lio_path(args.lio_root)
    from common import frames                      # noqa: E402  前工程：唯一的 L↔W 换算
    from planner import cmd_stream                 # noqa: E402  前工程：到达/悬停/推进判定

    # ★ 航线来源 ★
    #   --tour world（默认 in the hex arm）: 用世界文件里的 mission.corners（一 lap 的角点 × laps），
    #     这样**航点与地图同源**，世界文件冻结（SPEC S5）就等于航点冻结；hexagon 只能这么喂。
    #     第 87 轮：`laps` 现在真的生效（见 world_tour 的注释——在那之前它被忽略，一直飞 18 个角点）。
    #   --tour square: 老行为，用 --square-m 生成正方形巡游（v1/v2 的方形任务）。
    if args.tour == "world":
        try:
            corners_w, tour_info = world_tour(args.world, args.laps, args.z, verbose=True)
        except ValueError as _exc:
            print("FATAL: --tour world: %s" % _exc, flush=True)
            return 2
        if args.vertical_first_m is not None:
            corners_w = np.vstack([np.array([[0.0, 0.0, float(args.vertical_first_m)]]), corners_w])
        hold0 = float(args.first_hold_s)
        print("   tour: %d corners + %s = %d targets (%d laps requested, %d flown)"
              % (tour_info["corners_flown"],
                 "a vertical first leg" if args.vertical_first_m is not None else "no vertical leg",
                 len(corners_w), tour_info["laps_requested"],
                 tour_info["corners_flown"] // max(1, tour_info["corners_per_lap"])),
              flush=True)
    else:
        corners_w, hold0 = square_tour(args.square_m, args.laps, args.z, first_hold_s=args.first_hold_s,
                                       vertical_first_m=args.vertical_first_m)
    holds = [hold0] + [args.hold_s] * (len(corners_w) - 1)

    # ---- 净空调速需要世界几何：从 --world 读 84 根柱子（薄层本来只拿它打印） ----
    # 只有 --clr-gov-a-lat > 0 时才用；读不到就自动关闭（不影响任何旧臂）。
    pillars = None
    if args.world and os.path.isfile(args.world):
        try:
            import yaml as _yaml
            _wd = _yaml.safe_load(open(args.world, encoding="utf-8"))
            _pl = _wd.get("pillars") or []
            if _pl:
                pillars = np.array([[float(q["x"]), float(q["y"]), float(q.get("r", 0.12))]
                                    for q in _pl], dtype=float)
                print("   clearance governor: %d pillars from %s" % (len(pillars), args.world),
                      flush=True)
        except Exception as _exc:                        # noqa: BLE001
            print("   clearance governor: could not read %s (%s); disabled"
                  % (args.world, _exc), flush=True)
            pillars = None

    def clr_min(p_w, u_dir, horizon_m, nsamp=6):
        """参考点附近（并沿前进方向 horizon_m 米内）最小柱面距离，W 系。"""
        if pillars is None:
            return None
        best = np.inf
        for k in range(nsamp):
            frac = 0.0 if nsamp == 1 else k / (nsamp - 1.0)
            q = np.asarray(p_w, float)[:2] + (u_dir[:2] * horizon_m * frac if u_dir is not None else 0.0)
            d = np.hypot(q[0] - pillars[:, 0], q[1] - pillars[:, 1]) - pillars[:, 2]
            best = min(best, float(d.min()))
        return best

    # ---- calib：**显式**，没有"拟合一下"这条后路 ----
    # ★ 默认值必须与 bridge 的兜底**逐位一致** ★
    #   bridge（`bridge_node.load_calib()`）在没有 `--calib-file` 时用
    #       frames.Calib.predicted(spawn_heading_deg=0.0, world_axes="body")
    #   即 L→W = R_z(+90°) + t=(0, +0.05, +0.10) m。
    #   我一开始在这里用单位阵，于是在 `NAV_SOURCE=truth`（harness 不传 calib）时：
    #       薄层把 W 的点当 L 发 → bridge 又按 +90° 转一次 → 飞机去的是**旋转 90° 的正方形**。
    #   现场证据：setpoints.csv 第一条 cmd=(7.50, −7.45, 2.10)，而薄层发的第一个角点是 (−7.5, −7.5)，
    #   (−7.5,−7.5) 绕 +90° 正是 (7.5,−7.5) —— 同一批命令被转了两次。
    if args.calib_file and os.path.exists(args.calib_file):
        with open(args.calib_file) as fh:
            d = json.load(fh)
        calib = frames.Calib(R_WL=np.array(d["R_WL"], dtype=float),
                             t_WL=np.array(d["t_WL"], dtype=float))
        print("calib: L->W from %s: %s" % (args.calib_file, calib.summary()), flush=True)
    else:
        calib = frames.Calib.predicted(spawn_heading_deg=0.0, world_axes="body")
        print("calib: NO --calib-file -> using the SAME predicted fallback as the bridge (%s).\n"
              "       这是 bridge 的兜底值；只要两边一致，W 的点就会被 bridge 还原回 W。\n"
              "       LIO 臂（NAV_SOURCE=lio）harness 会给两边同一份 _stage/calib.json，那时用实测值。"
              % calib.summary(), flush=True)
    R_LW = calib.R_WL.T
    corners_l = (R_LW @ (corners_w - calib.t_WL).T).T

    print("tour : %d 个航点（%g m 正方形 × %d 圈），W 系, z=%.2f" %
          (len(corners_w), args.square_m, args.laps, args.z), flush=True)
    print("       W 范围 x[%.2f,%.2f] y[%.2f,%.2f] → L 范围 x[%.2f,%.2f] y[%.2f,%.2f]" % (
        corners_w[:, 0].min(), corners_w[:, 0].max(), corners_w[:, 1].min(), corners_w[:, 1].max(),
        corners_l[:, 0].min(), corners_l[:, 0].max(), corners_l[:, 1].min(), corners_l[:, 1].max()),
        flush=True)
    print("       arrive %.2f m, hold %.1f s, cmd %.0f Hz, SUPER cmd topic %s" %
          (args.arrive_m, args.hold_s, args.cmd_rate, args.super_cmd_topic), flush=True)

    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import PoseStamped, Quaternion
    from nav_msgs.msg import Odometry

    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    except Exception:                                   # noqa: BLE001
        rclpy.init(args=[])
    node = Node("planner_source_super")

    try:
        from quadrotor_msgs.msg import PositionCommand as CmdMsg
    except ImportError as exc:
        print("CANNOT START: quadrotor_msgs 不可用 (%s)" % exc, flush=True)
        return 2
    try:
        from mars_quadrotor_msgs.msg import PositionCommand as MarsMsg
    except ImportError as exc:
        print("CANNOT START: mars_quadrotor_msgs 不可用 (%s)" % exc, flush=True)
        return 2
    try:
        from mars_quadrotor_msgs.msg import PolynomialTrajectory as PolyTrajMsg
    except ImportError:
        PolyTrajMsg = None                              # 老版本没有这条消息时，--time-scale 不可用

    # 出口 QoS 与前工程一致：latched，深 5，RELIABLE
    qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    cmd_pub = node.create_publisher(CmdMsg, args.cmd_topic, qos)
    goal_pub = node.create_publisher(PoseStamped, args.goal_topic, 10)
    # PVA goal topic must be <click_goal_topic> + "_pva": that is what fsm_ros2.hpp subscribes when
    # click goals are enabled (patch_goal_pva.py).
    goal_pva_pub = node.create_publisher(MarsMsg, args.goal_topic + "_pva", 10)
    best_effort = QoSProfile(depth=200, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.BEST_EFFORT)

    def now_goal():
        return time.monotonic()

    state = {"p_l": None, "v_l": None, "n": 0, "prev_p": None, "prev_wall": 0.0,
             "ref": None, "ref_wall": 0.0, "n_ref": 0, "goal_i": -1,
             "last_pub": None, "last_pub_t": None, "last_pace_wall": 0.0, "hold_yaw": 0.0}

    def here_w():
        if state["p_l"] is None:
            return None
        return calib.R_WL @ state["p_l"] + calib.t_WL

    # ---------------- SUPER 的多项式轨迹（可求值） ----------------
    # 需求来源：实测 `pgf-01` 的参考均速 2.89 m/s，而**同一条路径的时间最优是 3.53 m/s**
    # （scripts/path_time_optimal.py）—— 也就是说路径没问题，是规划器给的时间分配保守。
    # SUPER 会把自己的多项式轨迹发出来（/planning_cmd/poly_traj，order 7、每段 time_pos），
    # 求值它就能按倍速重放（--time-scale），把参考推到接近时间最优，而不改路径（净空/W1b 不变）。
    def _ros_now():
        return float(node.get_clock().now().nanoseconds) * 1e-9

    def on_poly(m):
        state["poly_msg_n"] = state.get("poly_msg_n", 0) + 1
        try:
            # ⚠ 同一个 topic 上还发 **HEARTBEAT**（`type` 只有 HEART_BEAT 位、piece_num=0）。
            #   第一版把 n<=0 当成"坏消息"并把 state["poly"] 清空 —— 于是每个心跳都抹掉刚收到的
            #   轨迹，POLYCHECK 一行都打不出来（实测 missionA-pgt-01：POLYMSG #1 是 trajectory
            #   piece_num=2 order=7 len(coef_x)=16，第 2、3 条全是 piece_num=0 的心跳）。
            #   正确做法：只认 POSITION_TRAJ 位；心跳/其它类型直接忽略，保留上一条有效轨迹
            #   （陈旧与否由 eval_poly 的时间区间检查兜底）。
            typ = int(getattr(m, "type", 0))
            POS = getattr(PolyTrajMsg, "POSITION_TRAJ", 2) if PolyTrajMsg is not None else 2
            n = int(m.piece_num_pos)
            o = int(m.order_pos) + 1
            if state["poly_msg_n"] <= 3:
                print("POLYMSG #%d type=%d piece_num=%s order=%s len(coef_x)=%d len(time)=%d"
                      % (state["poly_msg_n"], typ, n, m.order_pos,
                         len(m.coef_pos_x), len(m.time_pos)), flush=True)
            if not (typ & POS) or n <= 0 or o < 2:
                return
            cx = np.asarray(m.coef_pos_x, float).reshape(n, o)
            cy = np.asarray(m.coef_pos_y, float).reshape(n, o)
            cz = np.asarray(m.coef_pos_z, float).reshape(n, o)
            dur = np.asarray(m.time_pos, float)
            if dur.size != n:
                return
            # ⚠ 字段名随版本不同：装好的 mars_quadrotor_msgs 里是 `start_wt_pos`（小写 wt），
            #   而 upstream 的 .msg 写的是 `start_WT_pos`。实测：按大写取会抛 AttributeError，
            #   被下面的 except 吞掉，多项式恒为 None —— 校验飞行 POLYCHECK 一行都没有。
            swt = getattr(m, "start_wt_pos", None)
            if swt is None:
                swt = getattr(m, "start_WT_pos", None)
            if swt is None:
                return
            # ⚠ 类型也要兼容：upstream 的 .msg 是 `time start_WT_pos`，而**装好的**消息是
            #   `double start_wt_pos`（`get_fields_and_field_types()` 实测）。按 Time 取 `.sec`
            #   会抛 AttributeError，被 except 吞掉 → 多项式恒为 None（两次校验飞行的 POLYCHECK
            #   一行都没有，原因就在这里）。
            if hasattr(swt, "sec"):
                st = float(swt.sec) + 1e-9 * float(swt.nanosec)
            else:
                st = float(swt)
            state["poly"] = (st, np.cumsum(dur), cx, cy, cz)
        except Exception:                               # noqa: BLE001
            state["poly"] = None

    def eval_poly(now_ros, scale):
        """在 tau = (now - start) * scale 处求值；超出这条轨迹的时间范围返回 None。"""
        p = state.get("poly")
        if p is None:
            return None
        st, cum, cx, cy, cz = p
        tau = (now_ros - st) * float(scale)
        if tau < 0.0 or tau > cum[-1]:
            return None
        i = int(np.searchsorted(cum, tau, side="right"))
        i = min(i, len(cum) - 1)
        t = tau - (cum[i - 1] if i > 0 else 0.0)
        pw = np.array([t ** k for k in range(cx.shape[1])])
        return np.array([float(cx[i] @ pw), float(cy[i] @ pw), float(cz[i] @ pw)])

    def eval_poly_tau(tau):
        """在显式的轨迹时间 tau 处求值（时间缩放的锁相用）；越界返回 None。"""
        p = state.get("poly")
        if p is None:
            return None
        st, cum, cx, cy, cz = p
        if tau < 0.0 or tau > cum[-1]:
            return None
        i = int(np.searchsorted(cum, tau, side="right"))
        i = min(i, len(cum) - 1)
        t = tau - (cum[i - 1] if i > 0 else 0.0)
        pw = np.array([t ** k for k in range(cx.shape[1])])
        return np.array([float(cx[i] @ pw), float(cy[i] @ pw), float(cz[i] @ pw)])

    def lock_tau(p_cmd, span=2.0, step=0.02):
        """在多项式上找与当前 100 Hz 指令最匹配的轨迹时间 tau（锁相）。

        为什么需要它：`--poly-dt` 那个**绝对**偏移并不稳（实测 POLYCHECK 残差 0.28/0.60/0.30 m，
        只有少数拍是 1–3 cm），所以不能用一个常数去对齐。改为每拍在多项式上做一次一维搜索，
        得到"这条参考现在处在轨迹的哪个时刻"，再让缩放后的参考从这个锁点按 k 倍速前进。
        """
        p = state.get("poly")
        if p is None:
            return None, None
        st, cum, cx, cy, cz = p
        tau0 = _ros_now() - st
        best, best_tau = 1e9, None
        taus = np.arange(max(0.0, tau0 - span), min(cum[-1], tau0 + span) + 1e-9, step)
        for tt in taus:
            q = eval_poly_tau(float(tt))
            if q is None:
                continue
            e = float(np.linalg.norm(np.asarray(q)[:2] - np.asarray(p_cmd, float)[:2]))
            if e < best:
                best, best_tau = e, float(tt)
        return best_tau, best

    def on_odom(m):
        p = np.array([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z])
        now = time.monotonic()
        dt = now - state["prev_wall"]
        if state["prev_p"] is not None and dt > 1e-3:
            raw = (p - state["prev_p"]) / dt
            state["v_l"] = raw if state["v_l"] is None else 0.8 * state["v_l"] + 0.2 * raw
        state["prev_p"], state["prev_wall"] = p, now
        state["p_l"], state["n"] = p, state["n"] + 1
        # ★ 步进器按 50 Hz 走，不按 odom 的 200 Hz 走 ★
        #   pacer.step 是整个航点状态机（到达判定、驻留、超时、相位切换）。它原来在**每一条**
        #   odom 上跑，而 /lio/robo/odom 实测 198 Hz —— 4 倍的活干在同一件事上。
        #   实测我的薄层占 128% CPU（一个 100 Hz 发布者不该超过一个核的零头），
        #   而这台机器上每一分 CPU 都要让给雷达仿真。50 Hz 对"到达/驻留"判定绰绰有余。
        if now - state["last_pace_wall"] >= 0.02:
            state["last_pace_wall"] = now
            pacer.step(here_w(),
                       None if state["v_l"] is None else float(np.linalg.norm(state["v_l"][:2])),
                       now)

    def on_odom_vel(m):
        v = np.array([m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z])
        if float(np.linalg.norm(v)) > 1e-6:
            state["v_l"] = v

    def on_ref(m):
        """SUPER 的参考：位置/速度/加速度是 L，**yaw 也是 L 系的角**（yaw_mode=2，沿速度方向）。

        ★ 第 88 轮审查 F2：yaw 不能直接搬 ★
          `/plan/cmd` 的 yaw 字段按 `cmd_stream.py:83-91` 的契约是**飞控的罗盘航向**
          （0=北, +90=东），而 SUPER 给的是 **L 系的 ENU 角**。以前这里写"已经是 FCU 的航向语义，
          直接搬"，于是整条链把 L 系的角当成罗盘航向发了出去。实测（kn3-01-01，5015 个采样）：
          `yaw_cmd` 等于真实航向的**相反数**（中位 7.3°、rms 28.7°），飞控照做了 5.9° 以内 ——
          也就是大半条航线上机头是歪的。换算只允许在 `common.frames.l_to_wire_yaw` 里做一次，
          在**发布那一拍**做（见 on_timer），这里保持 L 系原值。
        """
        state["ref"] = (np.array([m.position.x, m.position.y, m.position.z]),
                        np.array([m.velocity.x, m.velocity.y, m.velocity.z]),
                        np.array([m.acceleration.x, m.acceleration.y, m.acceleration.z]),
                        float(getattr(m, "yaw", 0.0)), float(getattr(m, "yaw_dot", 0.0)))
        state["ref_wall"], state["n_ref"] = time.monotonic(), state["n_ref"] + 1
        # ★ 多项式校验（--poly-check）★ 把 SUPER 自己的多项式在当前时刻求值，与这条 100 Hz
        #   参考对比。两者对得上，就说明"按倍速求值"是可行的（时间缩放的唯一前提）。
        if state.get("poly") is not None and int(getattr(args, "poly_check", 0)):
            now_ros = _ros_now()
            p_poly = eval_poly(now_ros, 1.0)
            if p_poly is None:
                state["poly_none_n"] = state.get("poly_none_n", 0) + 1
                if state["poly_none_n"] in (10, 200, 1000):
                    _st, _cum = state["poly"][0], state["poly"][1]
                    print("POLYMSG eval None: now=%.3f start=%.3f total=%.3f tau=%.1f "
                          "(现在 - 起点 必须在 [0, total] 内)" % (now_ros, _st, _cum[-1], now_ros - _st),
                          flush=True)
            elif p_poly is not None:
                d = float(np.linalg.norm(np.asarray(p_poly)[:2]
                                         - np.array([m.position.x, m.position.y])))
                state["poly_err"] = d if state.get("poly_err") is None else 0.9 * state["poly_err"] + 0.1 * d
                state["poly_n"] = state.get("poly_n", 0) + 1
                if state["poly_n"] % 100 == 0:
                    print("POLYCHECK n=%d  |poly(%s) - cmd| = %.4f m (filtered %.4f)"
                          % (state["poly_n"], "now", d, state["poly_err"]), flush=True)
                    # ★ 自校准：扫一个时间偏移 δ，报"最优 δ 与残差" ★
                    #   实测第一版残差 0.28–0.60 m 且偶发 3.9 m —— 前者对应 ~75 ms 的时钟差
                    #   （4 m/s × 0.075 s），后者是取到了旧轨迹。δ 一旦稳定，就能用它做时间缩放。
                    best_e, best_dt = 1e9, 0.0
                    for dt_s in np.arange(-0.30, 0.3001, 0.01):
                        q = eval_poly(now_ros + float(dt_s), 1.0)
                        if q is None:
                            continue
                        e = float(np.linalg.norm(np.asarray(q)[:2]
                                                 - np.array([m.position.x, m.position.y])))
                        if e < best_e:
                            best_e, best_dt = e, float(dt_s)
                    state["poly_dt"] = best_dt
                    print("POLYFIT  best |poly - cmd| = %.4f m at dt = %+.2f s"
                          % (best_e, best_dt), flush=True)

    def seg_len_w(i):
        if i <= 0:
            return float(np.linalg.norm(corners_w[0][:2]))
        return float(np.linalg.norm(corners_w[i][:2] - corners_w[i - 1][:2]))

    pacer = cmd_stream.TourPacer(corners_w, args.hold_s, args.arrive_m, args.settle_speed,
                                 args.settle_s, args.leg_timeout_s, time.monotonic(),
                                 log=lambda s: print(s, flush=True), seg_len_w=seg_len_w,
                                 odom_label=args.odom, leg_speed=1.0,
                                 entry_hold_s=args.first_hold_s, holds=holds,
                                 # >0 turns on the "passed the waypoint" arrival test, which is what
                                 # lets the tour advance when the plan CARRIES SPEED through a
                                 # corner instead of stopping on it (see cmd_stream.py:step)
                                 pass_by_m=args.pass_by_m,
                                 pass_by_long_m=getattr(args, "pass_by_long_m", 0.0))

    node.create_subscription(Odometry, args.odom, on_odom, best_effort)
    node.create_subscription(Odometry, args.odom_vel_topic, on_odom_vel, best_effort)
    # ★★ P0：必须用 BEST_EFFORT 订 SUPER 的出口 ★★
    #   SUPER 的 `/planning/pos_cmd` 发布者是 `QoS(1).best_effort().keep_last(1)`
    #   （`super_planner/include/ros_interface/ros2/fsm_ros2.hpp:313-320`）。
    #   rclpy 的默认订阅是 RELIABLE → **永远匹配不上 → on_ref 一次都不触发**。
    #   我之前用默认 QoS，于是 state["ref"] 恒为 None，全程发 hold_sample（15 m 外的角点），
    #   飞机按"盲飞向角点"飞完了方形 —— swap-02..08 的那些"SUPER 规划出了 15 m 的边"全都不是 SUPER。
    qos_ref = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(MarsMsg, args.super_cmd_topic, on_ref, qos_ref)
    if PolyTrajMsg is not None:
        node.create_subscription(PolyTrajMsg, args.poly_topic, on_poly, qos_ref)

    def publish_goal_if_needed(force=False):
        """pacer 换航点 → 一次 W→L 的 goal 给 SUPER（显式用 calib，不拟合）。

        ★ `force`：SUPER 还没出过参考时每秒重发 ★
          `fsm_node` 是 Popen 之后才 spin 的，它建 goal 订阅要 5–10 s，而我在第一拍就发了首个 goal
          → **必丢**。丢了以后 SUPER 停在 WAIT_GOAL，永远不会自己醒（只有新 goal 能唤醒它）。
          所以 `n_ref == 0` 期间必须每秒重发同一个 goal；一旦收到参考就恢复"每航点只发一次"。
        """
        # ★ PRE-GOAL（--pre-goal-m）★：离当前角点还剩 pre_goal_m 米时，就把**下一个**角点发给 SUPER。
        #   为什么这样做：SUPER 每条轨迹终点速度恒为 0，所以只要"目标 = 当前角点"，飞机就必须在那里停。
        #   在**还剩 2 m** 的时候把目标换成下一个角点，当前角点就变成这条轨迹的内部点（不停车），
        #   而 A* 只有 2 m 的余量、切不出大弯 —— 与 §41 的 lead=1 不同（那是离角点 15 m 就换目标，
        #   A* 直接切掉 8-9 m，W1b 只有 7/13）。
        #   到达判据不变：pacer 仍然量到 pacer.i_leg 那个角点（pass-by 盒子）。
        pre = float(getattr(args, "pre_goal_m", 0.0) or 0.0)
        sent_i = pacer.i_leg
        # ⚠ 停机坪那一条腿（i_leg=0，竖直起飞）**不能用 pre-goal**：实测 missionA-pg-01（tag 撞名，
        #   旧记录已覆盖），距停机坪 0 m < 2 m 立刻把目标换成角点 1 → 竖直腿被跳过 → 飞机没爬升，
        #   truth 最高 0.389 m、setpoints.csv 为空。
        if pre > 0.0 and pacer.i_leg >= 1 and pacer.i_leg < len(corners_l) - 1:
            p_w_now = here_w()
            if p_w_now is not None:
                d_now = float(np.linalg.norm(np.asarray(p_w_now, float)[:2]
                                             - corners_w[pacer.i_leg][:2]))
                if d_now < pre:
                    sent_i = pacer.i_leg + 1
        if sent_i == state["goal_i"] and not force:
            return
        if sent_i == state["goal_i"] and force and (now_goal() - state["last_goal_t"]) < 1.0:
            return
        state["goal_i"] = sent_i
        state["last_goal_t"] = now_goal()
        g = PoseStamped()
        g.header.stamp = node.get_clock().now().to_msg()
        g.header.frame_id = cmd_stream.FRAME_L
        # ★ GOAL LEAD ★（--goal-lead k）：发给 SUPER 的是**下一个**角点，当前角点成为轨迹的内部点，
        #   于是轨迹终点不在这个角点上，SUPER 就不会在这里把参考速度降到 0。
        #   到达判据仍然是 pacer 量到 pacer.i_leg 那一个角点（targets_w 不变），所以"到过这个航点"
        #   这件事没有被放宽 —— 变的只是规划器把哪一点当成终点。
        gi_t = int(min(max(sent_i, pacer.i_leg + max(0, int(getattr(args, "goal_lead", 0)))),
                       len(corners_l) - 1))
        pl = corners_l[gi_t].copy()
        # ★ 目标点推到角点之后 ★
        #   SUPER 的每条轨迹都把**终点速度设为 0**（super_planner.cpp:725 的 pos_fina_state.setZero()），
        #   所以"把角点当目标"必然要求它在角点上停下 —— 实测 missionA-ph-01：离角点 2 m 内的路程占
        #   40%、速度只有 1.77 m/s；把加速度上限从 4.5 提到 9.0 只把峰值从 4.43 抬到 6.44，均值仍只到
        #   2.45。真正的解法是让轨迹的终点**不在角点上**：沿下一段腿的方向前推 lookahead 米，
        #   于是减速发生在角点之后，飞机带着速度穿过航点盒。
        #   到达判定仍然量到**角点**（pacer.targets_w 不变），所以"到过航点"这件事没有被放宽。
        la = float(getattr(args, "goal_lookahead_m", 0.0))
        if la > 0.0 and gi_t == pacer.i_leg and 0 < pacer.i_leg < len(corners_l) - 1:
            # ★ 用**去向**（本角点 → 下一个角点），不是来向 ★
            #   用 incoming 方向会把目标推到角点之外、顺着刚飞过来的那条腿延长出去 ——
            #   实测 missionA-pk-01：第一个目标变成 (-11.7, -11.7)，直接飞出场地撞上围栏
            #   （ABORT: geofence: truth at [-10.50 -7.80]，只到第 2 个航点）。
            d = corners_l[pacer.i_leg + 1] - corners_l[pacer.i_leg]
            n = float(np.linalg.norm(d[:2]))
            if n > 1e-6:
                pl[0] += la * d[0] / n
                pl[1] += la * d[1] / n
        # ★ PVA GOAL（--goal-pva 1）★
        #   目标仍然是这个角点（所以 W1b 不会被放宽），但把**离开角点的速度**一起告诉 SUPER：
        #   方向 = 出弯方向（本角点 → 下一个角点），大小 = --corner-speed。SUPER 的轨迹终点因此
        #   不是静止点，转角就不再是"停点"。转角速度由几何决定（scripts/corner_room.py）：
        #   本世界两个紧角 R=0.95 m → v≈2.0 m/s；宽角 3.5-4.3 m/s。最后一个航点仍然给 0（要停）。
        vout = np.zeros(3)
        # ⚠ 停机坪那一腿（index 0，竖直起飞）**必须 v=0**：实测 missionA-pv-01，给它一个 2 m/s 的
        #   水平出弯速度后，MINCO 每拍都报 "Omg or thr or Pos violation"（4881 次），
        #   一条轨迹都优化不出来，飞机 265 s 原地悬停（路径 4.5 m）。最后的航点同理要给 0（要停）。
        if int(getattr(args, "goal_pva", 0)) and 0 < gi_t < len(corners_l) - 1:
            # ★ 方向由 --corner-mode 决定 ★
            #   outgoing（默认）: 终点速度沿**出弯**方向 —— 实测不可行（90° 转向塞不进最后一段走廊，
            #     missionA-pv2-01：95 次 "Omg or thr or Pos violation"，飞机在角点磨蹭，1.70 m/s）。
            #   bisector: 取来向与去向的**角平分线**（90° 角 ⇒ 45°），终点转向只要 45°，走廊装得下；
            #     飞机以 45° 斜穿角点，再被下一条腿的规划拉正。
            #   incoming: 沿来向（直着冲过角点），留作对照。
            u_out = corners_l[gi_t + 1] - corners_l[gi_t]
            u_in = corners_l[gi_t] - corners_l[gi_t - 1]
            u_out = u_out / max(np.linalg.norm(u_out[:2]), 1e-9)
            u_in = u_in / max(np.linalg.norm(u_in[:2]), 1e-9)
            mode = str(getattr(args, "corner_mode", "outgoing"))
            if mode == "bisector":
                u = u_in + u_out
            elif mode == "incoming":
                u = u_in
            else:
                u = u_out
            nv = float(np.linalg.norm(u[:2]))
            # 每个角的安全速度可以不同（几何定）：--corner-speed-list "2.0,2.4,2.0,2.4" 按圈循环
            csl = str(getattr(args, "corner_speed_list", "") or "")
            if csl:
                vals = [float(x) for x in csl.split(",") if x.strip()]
                vc = vals[(gi_t - 1) % len(vals)] if vals else float(args.corner_speed)
            else:
                vc = float(args.corner_speed)
            if nv > 1e-6:
                vout = np.array([vc * u[0] / nv, vc * u[1] / nv, 0.0])
        if int(getattr(args, "goal_pva", 0)):
            m = MarsMsg()
            m.header.stamp = node.get_clock().now().to_msg()
            m.header.frame_id = cmd_stream.FRAME_L
            m.position.x, m.position.y, m.position.z = float(pl[0]), float(pl[1]), float(pl[2])
            m.velocity.x, m.velocity.y, m.velocity.z = float(vout[0]), float(vout[1]), float(vout[2])
            m.acceleration.x = m.acceleration.y = m.acceleration.z = 0.0
            m.jerk.x = m.jerk.y = m.jerk.z = 0.0
            m.yaw, m.yaw_dot = 0.0, 0.0
            m.trajectory_id = int(pacer.i_leg + 1)
            m.trajectory_flag = 1
            goal_pva_pub.publish(m)
            print("GOAL %d/%d → SUPER [PVA]: W(%s) = L(%.2f, %.2f, %.2f)  v_out (%.2f, %.2f)%s"
                  % (pacer.i_leg + 1, len(corners_w), np.round(corners_w[gi_t], 2),
                     pl[0], pl[1], pl[2], vout[0], vout[1],
                     "  [resend]" if force else ""), flush=True)
            return
        g.pose.position.x, g.pose.position.y, g.pose.position.z = float(pl[0]), float(pl[1]), float(pl[2])
        g.pose.orientation = Quaternion(w=1.0)
        goal_pub.publish(g)
        print("GOAL %d/%d%s → SUPER: W(%s) = L(%.2f, %.2f, %.2f)%s"
              % (pacer.i_leg + 1, len(corners_w),
                 (" +lead %d" % (gi_t - pacer.i_leg)) if gi_t != pacer.i_leg else "",
                 np.round(corners_w[gi_t], 2),
                 pl[0], pl[1], pl[2], "  [resend]" if force else ""), flush=True)

    def hold_sample(force_vertical=False):
        """SUPER 还没出参考 / 断流时的兜底。

        ★ 三种情形要分开（这是第二轮审查抓到的启动死锁）★
          (a) **还没收到过任何参考**（`n_ref == 0`）：`fsm_node` 是 Popen 之后才 spin 的，
              它建 goal 订阅要 5–10 s，而我第一拍就把首个 goal 发了 → **必丢**。
              这一段必须发**竖直腿目标**（停机坪正上方、z=2.0）：盲爬到 2 m 是安全的，
              而且 pacer 的 `_arm` 需要 z>0.5 或位移>0.5 m —— 发"当前位置"会让它永不 arm、
              永不换 leg、永不再发 goal，整套死锁。
              （旧版发 15 m 外的角点只是"盲爬"才偶然绕过这个 race，代价是盲飞。）
          (b) 有过参考但断流：**保持最后被指令点**（前工程 `ego_source.py:305-320` 的语义）。
          (c) 完全没有位姿：发竖直腿目标。

        `force_vertical` 用于**第一段（竖直腿）**：那一段故意由我们自己飞单调爬升，
        不用 SUPER 的参考。理由见 on_timer 里的注释。
        """
        if force_vertical or state["n_ref"] == 0:
            p = corners_l[0].copy()          # 竖直腿目标（(0,0,vertical_first_m) 或首角点）
        elif state["last_pub"] is not None:
            p = state["last_pub"].copy()
        elif state["p_l"] is not None:
            p = state["p_l"].copy()
        else:
            p = corners_l[0].copy()
        return cmd_stream.CmdSample(p, np.zeros(3), np.zeros(3),
                                    state["hold_yaw"], 0.0,
                                    pacer.i_leg + 1,
                                    cmd_stream.TRAJECTORY_STATUS_COMPLETED if pacer.done
                                    else cmd_stream.TRAJECTORY_STATUS_READY)

    def on_timer():
        # 还没收到过任何参考 → 每秒重发 goal（否则首个 goal 在 fsm_node 建好订阅前就丢了，永不唤醒）
        publish_goal_if_needed(force=(state["n_ref"] == 0))
        # ★ P0：SUPER 死了要大声说，不能静默继续发 hold ★
        #   但收尾阶段（stop.flag）是先杀子进程再退出的，那时看到 rc=-15 是正常的，不要误报。
        if child is not None and child.poll() is not None and not stop["flag"]:
            print("FATAL: fsm_node 已退出 (rc=%s) —— SUPER 不在环里了，停止发指令流"
                  % child.returncode, flush=True)
            stop["rc"] = 3
            stop["flag"] = True
            return
        now = time.monotonic()
        ref = state["ref"]
        # ★★ 参考断流太久 ⇒ 立刻失败退出，不要一路 hold 到撞围栏（第 88 轮补 8）★★
        #   MEASURED missionA-kw9-01-02：SUPER 从第 6 个航点（约 t=60 s）起不再发布新参考，
        #   `[status]` 的计数冻在 4312、age 一路涨到 **61 s**，而本层按 `--stale-s 1.0` 只是
        #   "保持上一点"，于是一条**冻住的目标**把飞机带了 50 多秒，最后是围栏/高度门在 100 m 外
        #   才中止。同一时刻 LIO 与真值只差 0.02 m —— 也就是说**跑飞的是参考，不是估计器**。
        #   `child.poll()` 只能发现进程"退出"，发现不了"活着但不发"，所以必须按**数据年龄**判。
        #   退出码 4 会让 `/plan/cmd` 停止，supervisor 的"规划静默 3 s"门随即按名字中止。
        if (child is not None and stop.get("rc", 0) == 0 and state["n_ref"] > 0 and not pacer.done
                and (now - state["ref_wall"]) > float(getattr(args, "stale_fail_s", 3.0))):
            print("FATAL: SUPER 参考断流 %.1f s（>%.1f s 阈值）—— 停止发指令流，让 supervisor 按名字中止"
                  % (now - state["ref_wall"], float(getattr(args, "stale_fail_s", 3.0))), flush=True)
            stop["rc"] = 4
            stop["flag"] = True
            return
        # ★ 第一段（竖直腿）由我们自己飞，不用 SUPER 的参考 ★
        #   MEASURED missionA-ny-02: the plan box stayed on leg 1/13 for the WHOLE 57 s sortie
        #   (`cmd x` in -0.12..+0.03, `cmd y` in -0.03..+0.95, i.e. it never left the pad) while
        #   `cmd_z` wandered 1.02..1.99 with NO trend.  That wander is SUPER's: its FSM only emits
        #   rest2rest trajectories and replans at replan_rate, and PlanFromRest zeroes the initial
        #   velocity every time, so for a goal only 1.8 m away the reference keeps decelerating to
        #   zero and re-accelerating -- a bob of about +-0.5 m.  The aircraft followed it, bounced
        #   off the floor twice (truth_U 1.01 -> 0.10 -> 1.43), and the pacer's 0.45 m height gate
        #   never latched, so the tour never started.
        #   The first leg is BY DESIGN a blind vertical climb ("盲爬到 2 m 是安全的"), so it is
        #   flown here as a monotone ramp to the vertical-leg target, shaped by the rate limiter,
        #   and SUPER takes over from leg 2 onward.
        on_vertical_leg = (pacer.i_leg == 0)
        if on_vertical_leg or ref is None or (now - state["ref_wall"]) > args.stale_s:
            s = hold_sample(force_vertical=on_vertical_leg)
        else:
            # ★★ yaw 在这一拍换算成飞控的罗盘航向（第 88 轮审查 F2）★★
            #   换算只允许发生在 common.frames.l_to_wire_yaw 里（一处），且必须用**同一份 calib**
            #   把位置和航向都转过去；否则机头与航迹不是一个坐标系，实测偏差中位 7.3°、rms 28.7°。
            yaw_ned = float(frames.l_to_wire_yaw(ref[3], calib))
            if not state.get("yaw_logged"):
                state["yaw_logged"] = True
                print("YAW: SUPER yaw %.4f rad (L) -> wire heading %.4f rad (%.1f deg), "
                      "calib yaw %.3f deg" % (float(ref[3]), yaw_ned, math.degrees(yaw_ned),
                                              calib.yaw_deg()), flush=True)
            s = cmd_stream.CmdSample(ref[0], ref[1], ref[2], yaw_ned, ref[4],
                                     pacer.i_leg + 1,
                                     cmd_stream.TRAJECTORY_STATUS_COMPLETED if pacer.done
                                     else cmd_stream.TRAJECTORY_STATUS_READY)
            state["hold_yaw"] = yaw_ned               # latch：断流时保持这个航向，不要发 0.0
        # ★ P0：两个参考源之间的钳位 ★
        #   SUPER 掉一拍再回来时，指令会在"最后指令点"与"前方 0.5 m"之间跳十几米。
        #   ego 臂为此有 EGO_REF_CLAMP_M=1.2（`run_mission.sh:373`），这里照做：单拍位移上限。
        #
        # ★★ 但单拍上限不是限速 ★★  MEASURED missionA-ms-03：垂直腿悬停得极好
        #   （truth/cmd 差 p50 0.027 m），然后参考从 (0,0) 一次跳到 (-3.06,-3.03)，
        #   飞机为追它压坡度、掉高度，31 s 时 U 从 1.94 → 0.94 → 32 s 触地 0.10 m，
        #   之后 SITL 模型彻底发散（真值到 273 m）。
        #   100 Hz 定时器下 1.2 m/拍 = **120 m/s 的目标滑移速率**，等于没限。
        #   所以真正的限幅必须是"速率 × dt"：6 m/s 的滑移上限把 4.3 m 抹到 0.7 s，
        #   压力落到姿态环能跟的范围内；而 4 m/s 的真实巡航参考只走 0.04 m/拍，
        #   远在 0.06 m 的限幅之内，不会被削。
        # ★ 未限幅的规划点，留给调速器用 ★
        #   下面的位置限幅会把 `s.p` 夹到 `last_pub` 周围 `REF_MAX × dt`（4.5 m/s × 10 ms = 0.045 m）
        #   的球里。第一版沿航迹调速器读的是**夹过的** `s.p`，于是它看到的"沿航迹滞后"永远 ≤0.045 m、
        #   算出的速度封在 kp×0.045 = 0.9 m/s —— 实测 missionA-rs-01：参考 61 % 的时间在 0.5–1.5 m/s、
        #   整趟 262 s、只判到 11/13（W6 反而漂亮：零延迟 0.188 m，因为指令慢到飞机完全跟得上）。
        #   调速器要的是**规划真正的落点**，所以在这里先存一份。
        # ★ 时间缩放（--time-scale k>1）★
        #   校验（第 69 轮 pgx-01）：SUPER 自己的多项式在 `now + 0.30 s` 处与它自己的 100 Hz 参考
        #   相差 **1.2–3.5 cm**（`POLYFIT` 日志），所以"按倍速求值同一条多项式"就是**同一条路径、
        #   时间压缩 k 倍**：路径不变 → 净空与 W1b 不变，只有时间参数化变快。
        #   动机：实测参考路径的时间最优是 **3.53 m/s**，而规划只给 2.89（§46.2）。
        #
        #   第 69 轮首飞（`pgy-01`，k=1.15）不可用：路径 708 m、接触、`|v_poly|` 首拍 7.86 m/s。
        #   三个工程原因与修法（都在这一版里）：
        #     ① 速度用 0.02 s 前向差分 → 多项式**分段边界**处一阶导不连续 → 尖峰。
        #        修：**中心差分**（±0.10 s）+ |v| 钳到规划上限。
        #     ② `state["poly"]` 可能与**正在执行的轨迹不同步**（换目标瞬间）。
        #        修：先按 k=1 求值，与 SUPER 这条 100 Hz 参考比；差 >0.10 m 就这一拍**回退到参考**。
        #     ③ k>1 让参考速度整体 ×k，会超过规划自己的上限。
        #        修：`--time-scale-vmax` 钳位（默认 = 薄层限幅）。
        if args.time_scale > 1.0 and state.get("poly") is not None:
            tau_lock, lock_err = lock_tau(np.asarray(s.p, float))
            state["ts_sync_err"] = lock_err if lock_err is not None else 1e9
            if tau_lock is not None and lock_err is not None and lock_err <= float(args.time_scale_gate):
                # 锁点：如果还没有自己的轨迹时间、或与锁点漂得太远，就重新锁定
                tau_cur = state.get("ts_tau")
                if tau_cur is None:
                    tau_cur = tau_lock
                    state["ts_t" + "_prev"] = now
                else:
                    dtk = max(0.0, now - state.get("ts_t_prev", now))
                    tau_cur = tau_cur + float(args.time_scale) * dtk
                state["ts_t_prev"] = now
                # 漂移保护：缩放后的参考不能领先/落后实际轨迹太多（否则就是另一条路径了）
                if abs(tau_cur - tau_lock) > float(args.time_scale_drift):
                    tau_cur = tau_lock                      # 重新锁定
                    state["ts_relock"] = state.get("ts_relock", 0) + 1
                state["ts_tau"] = tau_cur
                q0 = eval_poly_tau(tau_cur)
                qa = eval_poly_tau(max(0.0, tau_cur - 0.10))
                qb = eval_poly_tau(tau_cur + 0.10)
                if q0 is not None and qa is not None and qb is not None:
                    _p = np.asarray(q0, dtype=float)
                    _v = (np.asarray(qb, dtype=float) - np.asarray(qa, dtype=float)) * (
                        float(args.time_scale) / 0.20)
                    _nv = float(np.linalg.norm(_v[:2]))
                    _vlim = (float(args.time_scale_vmax) if args.time_scale_vmax > 0
                             else max(1.0, float(args.ref_max_speed_mps)))
                    if _nv > _vlim and _nv > 1e-9:
                        _v = _v * (_vlim / _nv)
                    s.p = np.array([_p[0], _p[1], float(s.p[2])])
                    s.v = np.array([_v[0], _v[1], 0.0])
                    state["ts_n"] = state.get("ts_n", 0) + 1
                    if state["ts_n"] % 500 == 0:
                        print("TIMESCALE k=%.2f |v_ts|=%.2f m/s lock=%.3f m lead=%+.2f s "
                              "(n=%d skip=%d relock=%d)"
                              % (args.time_scale, float(np.linalg.norm(_v[:2])), lock_err,
                                 tau_cur - tau_lock, state["ts_n"],
                                 state.get("ts_skip", 0), state.get("ts_relock", 0)), flush=True)
                else:
                    state["ts_skip"] = state.get("ts_skip", 0) + 1
            else:
                state["ts_skip"] = state.get("ts_skip", 0) + 1
                if state["ts_skip"] in (5, 200, 2000):
                    print("TIMESKIP lock_err=%s (gate %.2f m) tau0=%.3f — 锁相失败，这一拍用参考"
                          % ("None" if lock_err is None else "%.3f m" % lock_err,
                             float(args.time_scale_gate),
                             _ros_now() - state["poly"][0]), flush=True)
                state["ts_tau"] = None
        p_super_raw = np.asarray(s.p, dtype=float).copy()
        if args.ref_z_hold > 0 and not state.get("z_latch"):
            hw = here_w()
            if hw is not None and float(hw[2]) >= float(args.ref_z_latch_m):
                state["z_latch"] = True
                print("ZHOLD latched: estimated z %.2f m >= %.2f m -> reference z held at %.2f +- %.2f m"
                      % (float(hw[2]), float(args.ref_z_latch_m), float(args.ref_z_hold),
                         float(args.ref_z_tol)), flush=True)
        if state["last_pub"] is not None and (args.ref_max_speed_mps > 0 or args.ref_clamp_m > 0):
            prev_t = state.get("last_pub_t")
            dt = (now - prev_t) if prev_t is not None else 1.0 / max(1.0, args.cmd_rate)
            # ★★ 净空调速（--clr-gov-margin）★★
            #   这条链的**最深横向偏差随速度线性增长**（46 趟拟合，§37.2）：deep(v) = 0.154·v − 0.014。
            #   所以"别跑得比余量快"的正确关系式是 v ≤ (c − margin) / 0.154，其中 c 是参考点在
            #   **前进方向上看一小段**的最小柱面距离（不是刹车距离——净空是横向的，不是纵向的）。
            #   它只降速、不改路径，所以 W1b 与参考净空本身都不受影响。
            #   动机：missionA-pgl-01 在 5.6/5.4 下接触（跟踪向内 0.262 m，参考只剩 +0.204 m），
            #   同一套参数的 pgf-01 却是 +0.275 —— 净空需要"跟着余量走"的调速，而不是固定上限。
            v_cap = float(args.ref_max_speed_mps)
            # ★ 发射段限速（--entry-max-speed-mps）★
            #   起飞后的第一条腿（pad -> c1 的对角线）是从**悬停**直接接到 4-5 m/s 的参考：飞机滞后
            #   0.3-0.5 s，于是开头 10-20 s 的 |cmd−truth| p90 是 0.71-0.92 m，而后面整趟只有
            #   0.55-0.60（实测 v3h-01，scripts/residual_segment.py）。W6 按"四等分"判据就是被这一段
            #   顶掉的（第一段 0.81，其余 0.56/0.51/0.58）。发射段本身没有速度指标，慢一点不损失什么。
            if float(getattr(args, "entry_max_speed_mps", 0.0)) > 0.0 and pacer.i_leg <= 1:
                v_cap = min(v_cap, float(args.entry_max_speed_mps))
            if pillars is not None and args.clr_gov_margin > 0:
                u_dir = state.get("last_dir")
                if u_dir is None:
                    u_dir = p_super_raw[:2] - np.asarray(state["last_pub"], float)[:2]
                    nn = float(np.linalg.norm(u_dir))
                    u_dir = (u_dir / nn) if nn > 1e-6 else None
                c_now = clr_min(s.p[:2], u_dir, args.clr_gov_look_m)
                if c_now is not None:
                    v_safe = max(0.0, (c_now - args.clr_gov_margin) / 0.154)
                    # 平滑：不平滑的 cap 会"卡一下、放一下"，把峰值顶高（实测 pgn-01：p95 4.30
                    # vs 无调速的 4.08），W6 反而超了 0.04。100 Hz 下 α=0.05 ≈ 0.2 s 时间常数。
                    prev_cap = state.get("clr_v_cap", v_safe)
                    v_safe = 0.95 * prev_cap + 0.05 * v_safe
                    state["clr_v_cap"] = v_safe
                    if v_safe < v_cap:
                        v_cap = v_safe
                        state["clr_gov_hits"] = state.get("clr_gov_hits", 0) + 1
                        state["clr_gov_c"] = c_now
                    if now - state.get("clr_gov_t", 0.0) > 3.0:
                        state["clr_gov_t"] = now
                        print("CLRGOV c=%.3f m -> v_safe %.2f m/s (cap now %.2f, hits %d)"
                              % (c_now, v_safe, v_cap, state.get("clr_gov_hits", 0)), flush=True)
            cap = v_cap * max(dt, 1e-3)
            if args.ref_clamp_m > 0:
                cap = min(cap, args.ref_clamp_m)
            d = s.p - state["last_pub"]
            n = float(np.linalg.norm(d))
            # ★★ 参考永不后退 ★★
            #   `PlanFromRest` 从**实际位姿**起规划（super_planner.cpp:409-410），而实际位姿落后指令点
            #   v*延迟（4 m/s × 0.2 s ≈ 0.8 m）。于是每次"轨迹走完 → 重新规划"，新轨迹的起点就在
            #   指令点**后面**约 0.8 m，参考整体往回跳一次；我的限幅器再把这一跳摊成 5 m/s 的倒退。
            #   这不是规划，这是重规划的毛刺：指令点倒退时飞机要么被拽回去要么彻底跟不上。
            #   实测 missionA-pp-01：W6（指令 vs 真值，去掉时间基后 p90）0.68/0.71/0.77/0.79 m，
            #   超出 0.60 的判据；按"到角点距离"和"参考速度"分桶都是均匀的（p50 0.31、p90 0.67）——
            #   说明它不是拐角问题也不是速度问题，而是这些 1 m 级的**倒跳**把 p90 抬起来的。
            #   所以：若新参考落在当前前进方向的后方，就保持不动，等新规划追上来。
            retract = 0.0
            if args.ref_no_retract and state.get("last_dir") is not None and n > 1e-6:
                retract = float(d @ state["last_dir"])      # <0 means the target is behind
                if retract < -0.05:
                    s.p = state["last_pub"].copy()
                    s.v = np.zeros(3)
                    s.a = np.zeros(3)
                    state["n_retract"] = state.get("n_retract", 0) + 1
                    d = np.zeros(3)
                    n = 0.0
            clamped = n > cap
            if clamped:
                s.p = state["last_pub"] + d / n * cap
            # ★ 竖直通道单独、更慢 ★
            #   垂直腿是一次 0.2 m → 2.0 m 的阶跃。水平追阶跃可以靠压坡度（有 g 的余量），
            #   竖直不能：油门顶上去就会冲过头。实测 mz-02/03 在 6 m/s 的三维限幅下仍然
            #   U 冲到 3.13 / 3.35 m（指令 2.0），把 supervisor 的 3.0 m 爬升门顶穿。
            #   1.5 m/s 把 1.8 m 摊到 1.2 s，落在 WP_SPD_UP 2.5 m/s 之内。
            if args.ref_max_speed_z_mps > 0:
                zcap = args.ref_max_speed_z_mps * max(dt, 1e-3)
                dz = float(s.p[2] - state["last_pub"][2])
                if abs(dz) > zcap:
                    s.p[2] = state["last_pub"][2] + float(np.sign(dz)) * zcap
                    clamped = True
            # ★★ 定高窗口（--ref-z-hold）★★
            #   平方航线是定高飞行，任务高度恒为 2.0 m；SUPER 的三维优化器在窄走廊里却会让 z 上下摆
            #   （实测 v2d-01：参考 z 0.50~3.50 m，真值 p95 偏高 +0.82 m，W3 差 0.02 m 不过，W6 也被
            #   这一维抬高）。这不是"改规划"，而是**任务本来就只有平面**：锁存后把参考 z 夹回任务高度。
            #   锁存条件是飞机已经爬到位（估计 z ≥ --ref-z-latch-m），所以垂直起飞段完全不受影响。
            if args.ref_z_hold > 0 and state.get("z_latch"):
                # `--ref-z-hold` 是**任务高度（W）**，而 s.p[2] 是 **L**（t_WL.z = +0.11557）。
                # 不减掉这个平移，窗口就整体高 11.6 cm，把飞机命令到 2.07-2.17 m（实测 cmd_z 上限
                # 正好是窗口上沿 2.16557）。第 88 轮审查 F3。
                zt = float(args.ref_z_hold) - float(calib.t_WL[2] if hasattr(calib, "t_WL") else 0.0)
                lo, hi = zt - float(args.ref_z_tol), zt + float(args.ref_z_tol)
                if float(s.p[2]) < lo or float(s.p[2]) > hi:
                    s.p[2] = min(max(float(s.p[2]), lo), hi)
                    clamped = True
                    state["z_hold_hits"] = state.get("z_hold_hits", 0) + 1
            # ★★ 被限幅的那一拍，速度前馈要不要改成"限幅后的真实滑移速率"？默认**不改** ★★
            #   理由是本轮实测出来的一个硬矛盾：
            #     · 限幅只动位置、不动速度时，指令自相矛盾（位置以 6 m/s 前进、速度前馈写 0），
            #       这确实是 missionA-pm-01 里看到的"落后 → 重新规划 → 再跳"的材料；
            #     · 但改成 (p_new − p_prev)/dt 之后，**参考速度会超过规划器自己的上限**：
            #       pv-01 用的是 max_vel 5.0 的配置，`setpoints.csv` 的参考峰值却是 6.00 m/s ——
            #       那一拍的速度前馈正是 REF_MAX_MPS=6.0 的滑移速率。规划从没要过 6 m/s，是我加的。
            #   而 W6 的残差是"滞后 × 速度"，参考峰值 4.96 的三次飞行残差 0.52~0.59（过线），
            #   峰值 6.00 的三次是 0.72~0.82（不过）。所以这个改写只该作为实验开关，
            #   默认保持 SUPER 自己的速度：`--ref-ff 1` 才改写。
            if clamped and args.ref_ff:
                v_eff = (s.p - state["last_pub"]) / max(dt, 1e-3)
                s.v = v_eff
                s.a = np.zeros(3)
        # 记住**真的走出去的**方向：`--ref-no-retract` 用它判断"新参考是不是在后方"。
        # 只在实际前进了 >1 mm 时更新，所以悬停/冻结的那几拍不会把方向设成 0。
        prev_pub = state["last_pub"]
        # ★★ 参考调速器：把参考变成一条自洽、连续的轨迹（`--ref-gov-kp > 0` 才启用）★★
        #   位置限幅（上面的 cap）解决的是"参考别跳"，但它有个副作用：位置在以 5 m/s 滑动的那些拍里，
        #   速度前馈仍然是 SUPER 那条轨迹自己的值 —— 两者方向和大小都不一致。飞控在
        #   `BRIDGE_FIELDS=pos` 时忽略速度，这个矛盾是隐形的；一旦开了 `posvel`，飞控会把这个
        #   自相矛盾的速度喂进它的加速度受限参考模型（shape_pos_vel_accel_xy）。实测 qa-01：
        #   真值冲到 7.12 m/s（参考峰值只有 5.00）、高度到 3.58 m、路径 211 m —— 被推起来了。
        #   调速器改成：状态 p_ref，每拍
        #       v = clamp(v_super + kp*(p_super − p_ref), vmax)
        #       p_ref += v * dt
        #   位置和速度由同一个 v 产生，所以处处自洽、处处连续，速度天然有界。
        #   注意它**每拍都要跑**：第一版把它写进了 `if nadv > 1e-3`（只在有位移的那拍更新），
        #   那样静止的拍就不会收敛。
        if args.ref_gov_kp > 0.0:
            if state.get("p_ref") is None:
                state["p_ref"] = s.p.copy()
            dtg = (now - state["last_pub_t"]) if state.get("last_pub_t") is not None \
                else 1.0 / max(1.0, args.cmd_rate)
            dtg = min(max(dtg, 1e-3), 0.2)          # a stalled tick must not become a big step
            # ★ 调速器只管**水平**；竖直通道保持"只喂位置" ★
            #   实测 missionA-qf-01（紧跟踪 + 前推目标）：水平很好，但竖直冲到 +1.6 m
            #   （W3 的 excursion 判据不过）。竖直和水平在这里不是同一类问题：
            #   水平误差可以靠压坡度补，竖直只能靠油门，而油门一旦带上前馈就会冲过头
            #   （同一个教训在 `--ref-max-speed-z-mps` 的注释里已经写过一次）。
            #   所以：x/y 走调速器（自洽、连续、有加速度上限）；z 仍走原来的
            #   限幅参考位置 + **零竖直前馈**，等于"竖直只喂位置"。
            # ★★ 超前补偿（--ref-lead-s）★★ 把目标点沿 SUPER 自己的速度方向外推 T 秒：
            #   p_target = p_super + v_super*T*fade，fade 在规划转向快时降到 0（拐角不超前）。
            #   只改**输入**，p_ref 仍由发布速度积分，所以 (p, v) 的自洽性不受影响。
            p_tgt = np.asarray(p_super_raw, dtype=float).copy()
            _lead = float(getattr(args, "ref_lead_s", 0.0) or 0.0)
            if _lead > 0.0:
                _v = np.asarray(s.v, dtype=float)[:2]
                _sp = float(np.linalg.norm(_v))
                if _sp > 0.25:
                    _u = _v / _sp
                    _up = state.get("u_prev")
                    state["u_prev"] = _u
                    _dth = 0.0
                    if _up is not None and dtg > 1e-6:
                        _c = float(_u[0] * _up[1] - _u[1] * _up[0])
                        _d = float(_u[0] * _up[0] + _u[1] * _up[1])
                        _dth = abs(math.atan2(_c, _d)) / dtg
                    _turn = float(getattr(args, "ref_lead_turn_rad_s", 1.5) or 1.5)
                    _fade = max(0.0, min(1.0, 1.0 - _dth / max(_turn, 1e-6)))
                    _dist = min(_lead * _sp, 0.60) * _fade
                    p_tgt[:2] = p_tgt[:2] + _u * _dist
                    if state.get("lead_dbg") is None or (now - state["lead_dbg"]) > 5.0:
                        state["lead_dbg"] = now
                        print("   lead: |v| %.2f m/s, turn %.2f rad/s, fade %.2f -> %.2f m ahead"
                              % (_sp, _dth, _fade, _dist), flush=True)
                else:
                    state["u_prev"] = None
            vg = np.asarray(s.v, dtype=float) + args.ref_gov_kp * (p_tgt - state["p_ref"])
            # SUPER's own UNCLAMPED point (the position limiter above already pulled `s.p` in)
            p_super = p_tgt.copy()
            # ★★ 只做沿航迹调速（--ref-gov-along 1）★★
            #   一阶滤波式的"朝规划点追"在拐弯处必然横着切进去（滞后 v/kp 是**纯横向**的），
            #   而柱子就在拐弯处 —— 实测 kp=6 时指令净空 p1 只剩 0.084 m，规划却给了 0.325 m。
            #   把 kp 抬到 20 能把它压到 0.22 m，但指令过于忠实、飞机跟不上（rp-01：W6 残差
            #   0.84-1.11，时间基偏移从 -0.85 s 缩到 -0.15 s —— 那 0.85 s 本来就是滤波器滞后）。
            #   沿航迹模式：只允许 p_ref 沿 SUPER 自己的前进方向 u 前进，
            #       v = clamp(kp * (Δ·u), 0, vmax) * u
            #   于是**横向偏差恒为 0**（不切角），而沿航迹滞后 = v/kp 可以很小。
            if args.ref_gov_along >= 2:
                # ★★ 弧长同步（--ref-gov-along 2）★★ 见本节顶部注释：追**弧长**，不追投影。
                uxy = np.asarray(s.v, dtype=float)[:2]
                sp = float(np.linalg.norm(uxy))
                vmax_x = args.ref_max_speed_mps if args.ref_max_speed_mps > 0 else 99.0
                ks = float(getattr(args, "ref_gov_ks", 4.0) or 4.0)
                st = state
                if st.get("s_plan") is None:
                    st["s_plan"] = 0.0
                    st["s_ref"] = 0.0
                st["s_plan"] += sp * dtg
                err_s = float(st["s_plan"] - st["s_ref"])
                # ★ 参考永远不许比规划自己快太多 ★
                #   ke2-01-01 的教训：这一支原来只用 --ref-max-speed-mps（臂里是 6.0 的安全上限）钳，
                #   而 SUPER 的点在每个角点都会慢下来（实测 p1 = 0.34 m/s）⇒ 参考被命令到 6 m/s 去
                #   "追缺口"，飞机跟着 ~5 m/s 飞出场地（cmd 到了 y = 68.9 m）。现在上限是
                #   |v_super| + REF_CATCHUP_MPS：允许追赶，但不许超过自己正在追的那条轨迹。
                v_cap = min(vmax_x, sp + float(getattr(args, "ref_catchup_mps", 0.5) or 0.5))
                if sp > 0.2:
                    uxy = uxy / sp
                    # plan speed + arc-length correction, never negative
                    v_ref_s = max(0.0, min(v_cap, sp + ks * err_s))
                    vg = np.array([uxy[0] * v_ref_s, uxy[1] * v_ref_s, 0.0])
                else:
                    # the plan itself is (almost) stopped: close the remaining gap, but gently
                    dxy = p_tgt[:2] - st["p_ref"][:2]
                    vg2 = args.ref_gov_kp * 0.5 * dxy
                    nv = float(np.linalg.norm(vg2))
                    if nv > v_cap:
                        vg2 *= v_cap / max(nv, 1e-9)
                    vg = np.array([vg2[0], vg2[1], 0.0])
                # NOTE: `s_ref` is advanced AFTER the acceleration limiter below, with the speed
                # that is actually published -- counting a speed the limiter later cuts would make
                # the arc error drift by exactly the cut.
                if st.get("arc_dbg") is None or (now - st["arc_dbg"]) > 10.0:
                    st["arc_dbg"] = now
                    print("   arc-sync: plan %.2f m/s, s_err %+.2f m, v_ref %.2f m/s"
                          % (sp, err_s, float(np.linalg.norm(vg[:2]))), flush=True)
            elif args.ref_gov_along:
                uxy = np.asarray(s.v, dtype=float)[:2]
                un = float(np.linalg.norm(uxy))
                vmax_x = args.ref_max_speed_mps if args.ref_max_speed_mps > 0 else 99.0
                dxy = p_tgt[:2] - state["p_ref"][:2]
                if un > 0.2:
                    uxy = uxy / un
                    # ★★ 参考速度不得超过"规划自己 + 追赶余量"（第 88 轮审查 F1）★★
                    #   以前这里只有 --ref-max-speed-mps（臂里 6.0）这个**安全上限**，于是 kp=12 时
                    #   0.5 m 的沿航迹缺口就把它顶到 6 m/s —— 是规划器 max_vel 1.5 的 4 倍。
                    #   实测（kq1-02-02）：发布的位置流 p95 3.34 / max 5.52 m/s，而规划只认证 1.5；
                    #   桥又把速度字段钳到 1.50 ⇒ 位置与速度自相矛盾，且参考在 5.5 与 0 之间方波跳变，
                    #   姿态环根本跟不上 —— 这就是"冲过头再回来"的来源。
                    v_cap = min(vmax_x, float(un) + float(getattr(args, "ref_catchup_mps", 0.5) or 0.5))
                    spd = max(0.0, min(v_cap, args.ref_gov_kp * float(dxy @ uxy)))
                    # ★ 不许停住（第 88 轮补）★ 投影律在拐弯处必然掉到 0：实测 44 次/趟、19 次贴角点，
                    #   每次都跟着一次"飞机冲到 2-3 m/s 再追"的爆发。规划还在动时给一个下限，
                    #   参考就只在**规划自己停住**时停（SPEC S1 要求）。
                    _floor = float(getattr(args, "ref_along_floor", 0.0) or 0.0) * float(un)
                    if _floor > spd:
                        spd = min(v_cap, _floor)
                    vg = np.array([uxy[0] * spd, uxy[1] * spd, 0.0])
                else:
                    # the plan is (almost) stopped: close the remaining gap directly, still rate-capped
                    vg2 = args.ref_gov_kp * 0.5 * dxy
                    nv = float(np.linalg.norm(vg2))
                    if nv > vmax_x:
                        vg2 *= vmax_x / nv
                    vg = np.array([vg2[0], vg2[1], 0.0])
            # horizontal and vertical caps are different on purpose (a vertical step cannot be
            # recovered by banking -- see the --ref-max-speed-z-mps comment)
            vh = np.array([vg[0], vg[1], 0.0])
            nh = float(np.linalg.norm(vh))
            vmax = args.ref_max_speed_mps if args.ref_max_speed_mps > 0 else 99.0
            if nh > vmax:
                vh *= vmax / nh
            vg = np.array([vh[0], vh[1], 0.0])
            # ★ 前馈的加速度上限（"钳位壳"）★
            #   ArduPilot 的 posvel 通道把这个速度喂进一个加速度受限参考模型。速度本身有界还不够：
            #   它的一阶导（加速度）若是阶跃，`PSC_NE_JERK 100` 允许 100 m/s^3，5 m/s 可以在 50 ms
            #   内被要求出来，参考模型就被踢飞 —— pz-01/qb-01/qb-02 三次冲出围栏都是这个形态
            #   （qa-01 则是冲过指令：真值 7.12 m/s 对参考峰值 5.00）。限住 dv/dt 才是那层壳。
            if args.ref_gov_acc > 0.0 and state.get("v_ref") is not None:
                dv = vg - state["v_ref"]
                ndv = float(np.linalg.norm(dv))
                # ★ 非对称（第 88 轮补 11）★ 只对"减速"设更严的上限：
                #   实测 plan 在两条腿中间也会把速度掉到 0.2 m/s（11 次/140 s，离角点 3.7-4.9 m），
                #   参考跟着一个 tick 掉下去，飞机（1.0-1.3 m/s）就冲过去再被拉回来。
                #   减速限到 ref_gov_dec（默认 2.0），加速仍用 ref_gov_acc（8.0）——
                #   对称地把两者都压到 2.5 试过，反而更差（参考长期慢于飞机）。
                _sp_new = float(np.linalg.norm(vg))
                _sp_old = float(np.linalg.norm(state["v_ref"]))
                _lim = args.ref_gov_acc
                if _sp_new < _sp_old and float(getattr(args, "ref_gov_dec", 0.0) or 0.0) > 0.0:
                    _lim = float(args.ref_gov_dec)
                cap_dv = _lim * dtg
                if ndv > cap_dv and ndv > 1e-9:
                    vg = state["v_ref"] + dv / ndv * cap_dv
            _v_prev = state.get("v_ref")
            state["v_ref"] = vg
            # arc-length mode: count what the reference actually travelled (after every clamp/limiter)
            if args.ref_gov_along >= 2 and state.get("s_ref") is not None:
                state["s_ref"] += float(np.linalg.norm(vg[:2])) * dtg
            # 水平由调速器积分，竖直直接取限幅后的参考位置（已含 `--ref-max-speed-z-mps`）
            state["p_ref"] = np.array([state["p_ref"][0] + vg[0] * dtg,
                                       state["p_ref"][1] + vg[1] * dtg,
                                       float(s.p[2])])
            # ★ 不许切角：把 p_ref 拉回到规划点 radius 之内 ★
            #   一阶滤波器在拐弯处内切，而柱子就在拐弯处。实测 missionA-qi-01：接触那一刻**指令**
            #   自己的净空还有 +0.681 m，是飞机偏了 0.798 m 进去；但同一趟里指令净空 p1 只有 0.084 m、
            #   8.3 % 的样本已经进了规划自己的 0.325 m 余量（验收档 pw-01 只有 0.19 %）——
            #   也就是说调速器先把指令推进了柱子旁边，飞机再偏一点就撞。
            #   限住 |p_ref − p_super| 就把切角量直接限住了。
            if args.ref_gov_maxlag > 0.0:
                dxy = state["p_ref"][:2] - p_super[:2]
                nxy = float(np.linalg.norm(dxy))
                if nxy > args.ref_gov_maxlag:
                    state["p_ref"][:2] = p_super[:2] + dxy / nxy * args.ref_gov_maxlag
            s.p = state["p_ref"].copy()
            # `--ref-ff-gain` scales what the FCU is told to move at; the POSITION is unchanged, so a
            # gain < 1 asks the position loop to do more of the work (which is the way to tell whether
            # the overshoot comes from the feed-forward compounding with the position error).
            s.v = vg.copy() * float(args.ref_ff_gain)
            # ★ 加速度前馈（第 88 轮补）★ 以前这里是 `s.a = np.zeros(3)`，实测线上 |a| 恒等于 0。
            #   现在发 v 的差分：由调速器自己的限速保证 |a| ≤ --ref-gov-acc，再按规划器上界钳一次。
            _a = np.zeros(3)
            if _v_prev is not None and dtg > 1e-6:
                _a = (vg - _v_prev) / dtg
                _amax = float(args.ref_ff_acc_max) if float(args.ref_ff_acc_max) > 0 else None
                if _amax is not None:
                    _n = float(np.linalg.norm(_a))
                    if _n > _amax:
                        _a = _a * (_amax / _n)
            s.a = _a * float(args.ref_ff_gain)
        adv = s.p - prev_pub if prev_pub is not None else np.zeros(3)
        nadv = float(np.linalg.norm(adv))
        if nadv > 1e-3:
            state["last_dir"] = adv / nadv
        # ★★ CUSP WATCH（第 89 轮）：参考路径在腿中**掉头**的那一下 ★★
        #   MEASURED `kx1-02-01` t=106.9 s（scripts/dip_zoom.py）：发布的参考从 1.32 m/s 平滑减到
        #   **0.000 m/s 并连续两拍位置完全冻结**，然后倒着走 8 cm、横着走 7 cm，再重新前进 1.2 m/s。
        #   `dip_context.py` 在四趟上量到 7–9 次/趟，全部在离角点 **4–5 m** 处、换轨迹后 4–6 s，
        #   所以它不是"角点交棒"也不是"超时闸丢计划"（patch 10 之后 `Replan over time` 已经是 0）。
        #   缺的那块证据是"这一刻 SUPER 的状态机在做什么"：`fsm_node` 的日志与薄层写**同一个文件**，
        #   行序就是真实顺序，所以这一行就是能把两边钉在一起的钉子——下一趟直接看它前后各 5 行。
        if prev_pub is not None and nadv > 2e-3:          # >0.2 m/s at 100 Hz
            d_prev = state.get("cusp_dprev")
            if d_prev is not None:
                nprev = float(np.linalg.norm(d_prev))
                if nprev > 2e-3:
                    cosang = float(np.dot(adv[:2], d_prev)) / (nadv * nprev)
                    if cosang < -0.2:                     # a >101 deg fold, not wire jitter
                        state["cusp_n"] = state.get("cusp_n", 0) + 1
                        # ★ t_wall = time.monotonic() 与 `setpoints.csv`/`wire.csv` 的 `t_wall` 是同一个时钟 ★
                        #   （同一个进程所在同一台机器），所以这一列可以**直接**和记录对表，
                        #   不需要像 §79.4 那样去猜 epoch↔sim 的映射。
                        print("CUSP #%d t_wall=%.3f: the reference folded %.0f deg in one tick "
                              "(step %.3f m after %.3f m; |v| %.2f m/s) at L(%.2f, %.2f) "
                              "leg %d/%d done=%s origin=%s"
                              % (state["cusp_n"], now,
                                 math.degrees(math.acos(max(-1.0, min(1.0, cosang)))),
                                 nadv, nprev, float(np.linalg.norm(s.v[:2])),
                                 float(s.p[0]), float(s.p[1]),
                                 pacer.i_leg + 1, len(corners_w), pacer.done,
                                 "hold" if (on_vertical_leg or ref is None
                                            or (now - state["ref_wall"]) > args.stale_s) else "super"),
                              flush=True)
            state["cusp_dprev"] = adv[:2].copy()
        state["last_pub"] = s.p.copy()
        state["last_pub_t"] = now
        # ★ 参考低通（第 20 轮，--ref-lp-tau；默认 0 = 关）★
        #   依据 docs/INTEGRATION.md §99.3–§102：交付档的 W6 零时移 p90 是 0.34 m，由约 0.08 m 记录
        #   时基 + 沿航迹钟摆（p5 −0.19…−0.36）+ 横向 0.19 m 组成；九趟飞控参数手臂（§100–§102）
        #   一个都没压下去。离线实验（_scratch/ref_smooth_ceiling.py）把**命令流**低通到
        #   τ = 0.05–0.10 s 时，真值到命令的 p90 会降到 ~0.26 m —— 这个开关就是把那个低通放到
        #   发布路径上，让飞控也收到更平滑的参考。
        #   实现放在 src/whtf/ros/ref_lp.py（纯 numpy，可单测：scripts/test_ref_lp.py 断言
        #   τ=0 严格直通、直流增益 1、无过冲、不超前、dt≤0 不动状态）。
        _tau = float(getattr(args, "ref_lp_tau", 0.0) or 0.0)
        if _tau > 0.0:
            st_lp = state.get("lp")
            if st_lp is None:
                st_lp = ref_lp.new_state(s.p, s.v, s.a)
                state["lp"] = st_lp
                state["lp_t"] = now
            _dt = now - float(state.get("lp_t", now))
            state["lp_t"] = now
            p_pub, v_pub, a_pub, st_lp = ref_lp.update(st_lp, s.p, s.v, s.a, _tau, _dt)
            state["lp"] = st_lp
        else:
            p_pub, v_pub, a_pub = ref_lp.pass_through(s.p, s.v, s.a)
        m = CmdMsg()
        m.header.stamp = node.get_clock().now().to_msg()
        m.header.frame_id = s.frame_id
        m.position.x, m.position.y, m.position.z = (float(p_pub[0]), float(p_pub[1]), float(p_pub[2]))
        m.velocity.x, m.velocity.y, m.velocity.z = (float(v_pub[0]), float(v_pub[1]), float(v_pub[2]))
        m.acceleration.x, m.acceleration.y, m.acceleration.z = (float(a_pub[0]), float(a_pub[1]),
                                                                float(a_pub[2]))
        m.yaw, m.yaw_dot = float(s.yaw), float(s.yaw_dot)
        m.kx = [float(v) for v in cmd_stream.KX]
        m.kv = [float(v) for v in cmd_stream.KV]
        m.trajectory_id = int(s.trajectory_id)
        m.trajectory_flag = int(s.trajectory_flag)
        cmd_pub.publish(m)

    timer = node.create_timer(1.0 / max(1.0, args.cmd_rate), on_timer)

    def on_status():
        print("[status] odom %d (%.0f Hz-ish), SUPER ref %d (age %.2f s), leg %d/%d, %s, done=%s"
              % (state["n"], 0.0, state["n_ref"],
                 (time.monotonic() - state["ref_wall"]) if state["ref_wall"] else -1.0,
                 pacer.i_leg + 1, len(corners_w),
                 "dwelling" if pacer.dwelling else "flying", pacer.done), flush=True)

    node.create_timer(args.log_period_s, on_status)

    # ---- fsm_node（SUPER 本体）----
    child = None
    if not args.no_super:
        # ★ P0：先确认配置真的存在 ★
        #   `Config(ROOT_DIR+"config/"+name)` 遇到缺文件会抛 YAML::BadFile 直接 abort，
        #   而进程死了没人看 → 飞机在**零规划器**下飞完 3 圈，日志里只有一行 "starting: ..."。
        cfgp = os.path.join(args.super_config_dir, args.super_config)
        if not os.path.isfile(cfgp):
            have = sorted(f for f in os.listdir(args.super_config_dir) if f.endswith(".yaml")) \
                if os.path.isdir(args.super_config_dir) else []
            print("CANNOT START: SUPER 配置不存在 %s\n  可用：%s" % (cfgp, ", ".join(have)),
                  flush=True)
            return 2
        cmd = ["ros2", "run", "super_planner", "fsm_node", "--ros-args",
               "-p", "config_name:=%s" % args.super_config]
        print("starting: %s  (config=%s)" % (" ".join(cmd), cfgp), flush=True)
        # start_new_session：ros2 run 是 Popen 包装、不 exec（ros2run/api/__init__.py:64），
        # 直接 terminate 只杀掉 wrapper，fsm_node 会变孤儿继续发 /planning/pos_cmd。
        child = subprocess.Popen(cmd, start_new_session=True)

    def _bye():
        # killpg：子进程是 start_new_session=True 起的独立进程组，terminate 只杀 wrapper。
        if child is not None and child.poll() is None:
            try:
                import os as _os
                _os.killpg(_os.getpgid(child.pid), signal.SIGTERM)
            except Exception:                            # noqa: BLE001
                try:
                    child.terminate()
                except Exception:                        # noqa: BLE001
                    pass

    # ★ 信号处理里只置标志，收尾放在 spin 循环之后 ★
    #   在 handler 里 destroy_node()/shutdown() 会和 executor 的 waitset 抢生命周期，
    #   实测抛 `InvalidHandle: cannot use Destroyable because destruction was requested`。
    stop = {"flag": False, "rc": 0}

    def _sig(_s, _f):
        stop["flag"] = True

    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGINT, _sig)

    # 单线程执行器：所有回调都是非阻塞的短活（存一个位姿 / 存一个参考），多线程只带来
    # 自旋和 GIL 争用的开销。这台机器上 CPU 要留给雷达仿真，所以按最省的方式跑。
    from rclpy.executors import SingleThreadedExecutor
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    try:
        while rclpy.ok() and not stop["flag"]:
            # 5 ms：100 Hz 的出口 timer 需要被及时 spin，0.1 s 会把实测频率压到 ~66 Hz（实测）
            ex.spin_once(timeout_sec=0.005)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        _bye()
        try:
            node.destroy_node()
        except Exception:                                # noqa: BLE001
            pass
        try:
            rclpy.shutdown()
        except Exception:                                # noqa: BLE001
            pass
    # 退出码由上面的 FATAL 分支决定：3 = fsm_node 死了，4 = 参考断流超过 --stale-fail-s
    return int(stop.get("rc", 0))


if __name__ == "__main__":
    sys.exit(main())
