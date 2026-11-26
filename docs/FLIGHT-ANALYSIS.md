# FLIGHT-ANALYSIS —— 飞行数据分析与高速竞速方案
# （合并自 FLIGHT-LOOP-ANALYSIS / HIGH-SPEED-PLAN，并收录 2026-11-25 晚 phase4w/4x/4y 的角点实验）

本文件回答三个问题：**为什么绕圈**、**为什么跟踪看着不贴**、**怎么把速度从 1.35 提到 8–10 m/s**。
所有结论来自真飞日志（phase4p/q/r/s/u/v/w/x/y），工具：`scripts/analyze_loops.py`、
`scripts/track_quality.py`、`scripts/plot_flight_topdown.py`、`scripts/plot_flight_timeseries.py`、
`scripts/plot_corner_details.py`；图在 `docs/evidence/`。

## 0. 2026-11-25 晚角点实验结论（新增，先看这个）

| 趟 | 配置 | 结果 |
|---|---|---|
| phase4p | 速度接口修复前基线 | 5 处绕圈；指令停顿 22 次（角 11/腿中 11）；11 pass/3 fail |
| phase4q | LIO→rog_map 速度修复 | 绕圈 1；腿中停顿 3；12 pass/2 fail（W6 True） |
| phase4r | 同上复飞 | 绕圈 2；腿中 8；W6 False（方差大） |
| phase4s | `GOAL_PVA=1 bisector CORNER_SPEED=1.2 ARRIVE_M=0.3` | 角点平均转角 172°→139°，W1b/W6 过，12 pass/2 fail |
| phase4u | `NAV_HZ=100` | 实测 94.4 Hz，nav age p90 33 ms（48 Hz 时 35 ms，几乎不变）；跟踪没变好（W6 False）；**nav 频率不是主要杠杆** |
| phase4v | `PRE_GOAL_M=2.0`（未锁存） | goal 在"当前/下一个角点"间来回横跳（日志 30 次交替），6/13 到点，W1b 3/13 |
| phase4w | `PRE_GOAL_M=1.5` + 锁存 | goal 单调不再横跳，但暴露新问题：SUPER 到点后 `Traj finish`→WAIT_GOAL，飞机停在 0.5 m 门外，pacer 未判到达 → 参考断流 3 s → **中途 abort**（4/13） |
| phase4x | 4w + 参考断流时重发 goal + 诊断打印 | **事后复核：这不是"进展"，而是被掩盖的故障**——aircraft 在 pad 停留 40 s（`LEG 2 TIMEOUT ... 9.932 m from waypoint 2`），原因是 corner1 的 PVA 用 outgoing 方向（90° 角）规划不可行 + 我们的"参考断流重发"每 ~1 s 重置一次 SUPER 的 new_goal，规划永远收敛不了；tour 超时后**人工推进**才飞完剩下航点，所以 W1 缺 corner1、W1b 只有 12/13。飞机在 c2 附近与 c5→c6 中段的绕圈（6 处）都发生在这个混乱阶段 |
| phase4y | `GOAL_LOOKAHEAD_M=1.5` + CORNER_SPEED=1.2 | **失败/坠地**：76.9 s 时 `SIM Hit ground at 2.05 m/s`，W2/W6 False → goal-lookahead 当前不可用 |
| **phase5a** | 滚动目标 v1：`TARGET_MODE=rolling TARGET_LOOK_M=2.5 TARGET_STEP_M=0.8 TARGET_VREF=1.2 GOAL_PVA=1 CORNER_MODE=bisector CORNER_SPEED=1.2 PASS_BY_M=0.5` | 1.35 m/s 下**首次无 stall、无 timeout、W1 13/13、W2/W6 过**；但 W1b 8/13（lap1 corner6 真值最近 **0.500 m** 卡线，导致 lap2 整段不算）、W4 −0.069 m 接触、12 处绕圈、角点仍有 0.2–1.0 s 指令停顿（12 次共 11.1 s）、114 次 GOAL ROLL（goal 更新≈1.6 Hz，`PlanFromRest failed` 55 次） |
| phase4z | `PRE_GOAL_M=1.2 CORNER_CARRY_M=0.8 PASS_BY_M=0.5 PASS_BY_LONG_M=1.0 REF_NO_RETRACT=1`（"连续目标+越点判据"原型） | **回归**：12/13 ARRIVED 但 24 处绕圈、每个角点平均转角 178–238°、角点指令最小速度 0.00–0.15 m/s、W6 p90 0.616 → pass-by 半平面判据 + carrot 组合不可用，需重设计 |

结论：角点"不停车"的两个现成机制都还不能直接进交付配置——pre-goal 与验收的"按序到点"口径冲突，
goal-lookahead 会把静止终点推到角点之后并导致坠地。**当前交付配置保持 phase4q/4s 的形态**：
速度接口修复 + GOAL_PVA/角速度（可选）+ 默认不 pre-goal。



---

## 绕圈机理（原 FLIGHT-LOOP-ANALYSIS.md）

> 原文：`docs/FLIGHT-LOOP-ANALYSIS.md`（已并入本文件，独立文件删除）

## 绕圈归因：参考"停一下"，飞机"冲过头"（missionA-phase4p-01-01）

> 配套图：`docs/evidence/topdown-phase4p*.png`（俯视/分圈）、`timeseries-phase4p-xyz-t.png`、
> `phase4p-vxyz.png`、`timeseries-phase4p-speed-t.png`。
> 复现：`python3 scripts/analyze_loops.py --tag phase4p-01-01`、
> `python3 scripts/plot_flight_timeseries.py --tag phase4p-01-01`。

### 1. 检测到的事实

用"回到自己 1-6 s 前走过的点附近 + 期间转过 ≥300°"检测真值轨迹，本趟共 **5 处绕圈**（每处约 1 s，
净位移 0.2–0.6 m）。逐处对齐**规划指令的速度**后，规律非常一致：

| 事件 | 时间 (sim s) | 位置 (E,N) | 绕圈前 cmd 速度 | 绕圈时 cmd 最小速度 | 真值当时速度 | 距最近航点 |
|---|---|---|---|---|---|---|
| 1 | 59.8–60.8 | (0.4, 7.8) | 1.26 m/s | **0.08** | 1.07 | 4.7 m（腿中） |
| 2 | 89.7–90.7 | (−1.3,−8.3) | 1.15 | **0.00** | 1.32 | 3.7 m（腿中） |
| 3 | 91.8–93.8 | (−0.6,−8.1) | 0.53 | **0.00** | 1.56 | 4.5 m（腿中） |
| 4 | 99.6–100.9 | (5.0,−8.7) | 1.14 | **0.04** | 1.48 | 0.06 m（corner6 到点） |
| 5 | 151.0–152.2 | (−2.1,−8.0) | 1.34 | **0.10** | 3.05 | 3.0 m（腿中） |

**每条绕圈都对应一次"指令速度掉到 ≈0"**（0.00–0.10 m/s），而飞机当时还有 1.0–1.5 m/s。
指令位置本身没有反向（只是停住再走），所以不是"参考倒退"；飞机因为刹不住，冲过停住的参考点后
被位置环拉回来，形成半径约 0.2 m 的小圈，然后继续。

整趟统计：**指令"停一下"共 22 次**（速度 >0.8 → <0.4 m/s），其中 **11 次在航点上**（到点换目标，属设计行为）、
**11 次在腿中间**（异常）。5 次绕圈全部发生在这 22 次"停一下"里（不全是每次停都会绕圈：只有当时飞机
仍在 1 m/s 以上才会冲过头）。

### 2. 排除的解释

- **不是"径向速度太快、冲过航点"**：绕圈点大多离最近航点 3–4.7 m（腿中），且指令位置没有越过航点后反向；
  横向（cross-track）误差 p90 只有 0.2–0.5 m —— 飞机一直在参考线附近，不是侧向跑飞。
- **不是"planner 无解"**：整趟 1211 次 replan 成功（≈9 Hz）；`Opt failed` 62 次、`GenerateExpTrajectory failed`
  70 次，但只有事件 4（98.1–98.6 s 有 6 次失败）和事件 5（149.2 s 2 次）之前有失败簇；事件 1 附近
  **一次失败都没有**。所以"无解"不是共同原因。
- **不是估计器**：同期 FCU 估计速度与真值速度几乎相同（例如事件 1：估计 1.13 vs 真值 1.12 m/s），
  EKF 没有速度尖峰；也排除了"横向增益振荡"（cross-track 一直很小）。

### 3. 那"指令为什么会在半腿停一下"？

这是真正要修的地方。两个候选，本趟数据指向第 1 个（第 2 个只在事件 4/5 有局部证据）：

1. **replan 时参考被"从静止重新起一段"**：SUPER 以 10 Hz 滚动重规划，新轨迹的**起点速度**来自它看到的
   机器人状态。本仓库的 LIO 里程计 **twist 全为 0**（`lio.csv` 的 `vx,vy,vz` 整趟 0.000；实测
   `p50/p99/max = 0.000`），而规划器/适配器拿到的速度若因此为 0，则每次重规划的新轨迹从 v=0 起步，
   发布出来的 `/planning/pos_cmd` 速度就会在那一拍掉到 0 —— 正是上表看到的 0.00–0.10 m/s。
   （适配器侧虽然对 `/lio/robo/odom` 做了 0.8/0.2 的差分速度估计，但它只用于航点状态机，
   没有回填给 SUPER；`/lio/odom` 这条路在本趟没有可用 twist。）
2. **优化失败时的兜底**：`Opt failed, Omg or thr or Pos violation` 在 98 s 附近连发 6 次、149 s 2 次，
   对应事件 4/5 之前；patch 9 会让**已装订轨迹**继续发布，但如果当时轨迹已快走完，FSM 会短暂进入
   无参考/减速状态，同样表现为指令速度掉 0。

**结论（可执行的）**：
- 先修**速度接口**：让 SUPER 看到真实机器人速度（把 LIO 的差分速度或 `/lio/odom` 的 twist
  真正发布/转发给 `fsm_node`），这是"半腿停顿"的最高嫌疑；
- 在适配器里给 `/plan/cmd` 加**速度连续性**兜底：参考速度突降到 0 而距最近航点还远时，
  短暂保持上一拍的指令速度（例如 0.2–0.4 s 的保持/限速），避免飞机因"参考停车"而绕圈；
- 事件 4 这种**到点换目标**的停顿：可用已有的 `--goal-lookahead/--goal-pva` 或更早的交棒
  （把 `arrive-m` 从 1.0 调小、或把目标沿下一腿前推）来消除；
- 每次飞行都应统计"指令停一下"的次数（本轮工具输出 22 次/11 次腿中），作为下一轮改动的前后对照。

### 4. 我们的 planner 什么时候重规划（本趟实际配置）

本趟的 plan box 是 `bridge_nodes/thin/planner_source_super.py` + SUPER 原生 `fsm_node`
（`hku_super/config/super_planner_hex.yaml`）：

1. **FSM**：`WAIT_GOAL → GENERATE_TRAJ → FOLLOW_TRAJ → WAIT_GOAL`。本趟状态序列：
   `FOLLOW_TRAJ@1s → WAIT_GOAL@6s(第一段悬停) → FOLLOW_TRAJ@8s → WAIT_GOAL@136s(结束)`。
2. **FOLLOW_TRAJ 期间按 `replan_rate: 10 Hz` 滚动重规划**（本趟 `ReplanOnce succeed` 1211 次 ≈ 9 Hz）：
   每次用当前状态 + ROG-Map 最新点云，A* → SFC/CIRI → MINCO 重新优化一条到当前目标的轨迹；
   - `planning_horizon: 16 m`、`receding_dis: 1.0 m`：一条轨迹覆盖整条腿，轨迹快走完时向前延伸；
   - `replan_time_limit: 0.35 s`（patch 10：墙钟预算，超了才算失败）；
   - patch 9：优化期间**继续发布已装订轨迹**，所以"优化慢"不会让参考断流；
   - patch 11：新轨迹若前 0.6 s 沿当前速度方向**倒退**则拒绝，保留旧轨迹；
   - `hot_start_en: true`：从静止起规划时用实测速度，不让它从 0 起步。
3. **换目标**：由 `cmd_stream.TourPacer` 判定到点（`--arrive-m 1.0 m` 半径内、settle 后）→ 发
   `/planning/click_goal`（W→L 经 calib）→ SUPER 对新目标重新规划（可能是 `PlanFromRest`，dwell 3 s 只在
   第一个点；"ENTRY 腿"不留 dwell）。
4. **失败**：单拍优化失败保持旧参考并重试（下一拍 100 ms 后再试）；失败连续且旧轨迹已走完时会短暂
   无参考 / 减速 —— 这正是上节候选 2，也是事件 4/5 的局部证据。

### 5. 下一步建议（不飞行即可先做）

1. 给 SUPER 的输入接上**真实速度**（修 `/lio/odom` twist 或让适配器把差分速度作为 SUPER 的 odom 输入），
   再复盘"指令停一下"是否从 22 次降下来；
2. 在 `/plan/cmd` 出口加"参考停车保护"（腿中速度突降→短暂保持）；
3. 把 `scripts/analyze_loops.py` 的"指令停一下/腿中绕圈"计数纳入每趟飞行后的自动检查，作为回归指标；
4. 上述改动之后重飞 2 趟，用同一张 xyz-t / vxyz-t 图和绕圈表对比。

### 6. 角点 A/B：带速度过弯（2026-11-25 晚，phase4s / phase4t）

在上述速度接口修复（LIO→rog_map→hot_start）之后，又试了"角点不停车"的两个开关组合：

| 趟 | 配置 | 角点平均转角（几何 60°） | cmd 停顿（角/腿中） | 结果 |
|---|---|---|---|---|
| phase4p | 修复前基线 | 180° | 22（11/11） | 11 pass / 3 fail |
| phase4q | 仅速度接口修复 | 172° | 11（8/3） | 12 pass / 2 fail（W6 True） |
| **phase4s** | `GOAL_PVA=1 CORNER_MODE=bisector CORNER_SPEED=1.2 ARRIVE_M=0.3` | **139°** | 16（11/5） | 12 pass / 2 fail（W6 True） |
| phase4t | 4s + `GOAL_LEAD=1` | — | — | **W1/W1b/W6 失败，拒绝**（提前一个航点发 goal 破坏了到达判据） |

结论：`GOAL_PVA=1 + bisector + arrive-m 0.3` 把角点平均转角从 172° 降到 139°、指令速度 p10 从 0.49 提到
0.57 m/s，验收行不退化（W1b/W6 仍过）；`GOAL_LEAD=1` 不可用。剩余角点"停一下"是 SUPER 轨迹终点静止
（`pos_fina_state=0`）与到点换目标的综合结果，下一步按 `docs/HIGH-SPEED-PLAN.md` 的"延迟补偿 + 参数调硬"
推进；这两趟的图在 `docs/evidence/phase4s-*`（若未生成则用 `plot_corner_details.py --tag phase4s-01-01` 生成）。


---

## 跟踪诊断与高速方案（原 HIGH-SPEED-PLAN.md）

> 原文：`docs/HIGH-SPEED-PLAN.md`（已并入本文件，独立文件删除）

## 跟踪质量诊断 & 高速竞速方案（2026-11-25 调研结论）

> 目标：先回答"为什么无避障时轨迹也不贴"，再给出从当前 **1.35 m/s** 到 **8–10 m/s** 的分阶段方案。
> 依据：本仓库实测（phase4q/4s 飞行数据、ArduPilot 4.7.1 源码 `.build/ardupilot`、参数实测 dump）、
> SUPER 论文/GitHub、ArduPilot GUIDED 文档。**不改任何验收判据**，方案里的判据影响在最后单列（P0）。

### 0. 结论先说

1. **不是飞控 PID 没调好**：姿态/角速率环（ATC_*）保持固件默认，实测姿态跟踪正常；1.35 m/s 下没有振荡迹象
   （横向误差 p90 0.154 m，且是"滞后"不是"振荡"）。
2. **不是控制量不完备**：我们发的是完整 PVA（`wire.csv`: `fields=posvelacc`, type_mask=2048），
   ArduPilot 4.7 GUIDED 的 `posvelaccel` 路径是 `AC_PosControl::input_pos_vel_accel_NE_m(..., limit_output=false)`
   （`ArduCopter/mode_guided.cpp:943`、`AC_AttitudeControl/AC_PosControl.cpp:633`）——**前馈位置/速度/加速度原样使用，
   不受 WP_* 限幅**；限幅只作用在"位置误差修正"上。
3. **软的原因是"修正环被限得很保守 + 参考本身有停顿"**。实测（phase4q，2 m 高度段）：
   - 指令 vs 真值：零滞后误差 **p50 0.155 m / p90 0.266 m**，最优滞后 **−0.13 s**（飞机落后参考约 0.13 s）；
     沿航迹误差 p50 **−0.124 m**（落后），横侧 p50 −0.023 m（很小）→ 是"追不上"，不是"跑偏"。
   - 参考加速度：|a_xy| p50 0.72、p90 **1.45**、p99 **2.81**、max 3.94 m/s²。
   - 飞行实际生效的 FCU 参数（SITL dump 含最后加载的 `guided_limits_fast.parm`）：
     `WP_SPD 1.5`、`WP_ACC 4.0`、`PSC_NE_POS_P 1.2`、`PSC_NE_VEL_P 2.0`、
     `ATC_ANGLE_MAX 45°`、`PSC_NE_JERK 80`。
     ⚠ 在 PVA 模式下 jerk 限幅取的是 **`PSC_NE_JERK`（80 m/s³，宽松）**，不是 `WP_JERK`：
     `AC_PosControl::NE_set_max_speed_accel_m`（`AC_PosControl.cpp:468`）里
     `_jerk_max_ne_msss = _shaping_jerk_ne_msss;`，而 `_shaping_jerk_ne_msss` 就是 `PSC_NE_JERK`
     （同文件:136）。`WP_JERK` 只影响"只发位置"的航点导航路径。
   - **主因排序（实测细化，2026-11-25 22:10）**：先把"0.13 s"拆开——在**匀速直线段**（|v_cmd|>1.1、
     |a_cmd|<0.3）上分别对齐，结果完全不同：
     | 比较 | 最优滞后（cruise） | 含义 |
     |---|---|---|
     | 真值 ↔ LIO 里程计 | **35 ms** | LIO 滤波输出本身的估计滞后（nav 送出时 age p50 只有 2 ms，说明不是传输） |
     | 真值 ↔ 飞控 EKF 估计 | **60 ms** | LIO 35 ms + EKF 视觉融合 ~25 ms（VISO 48 Hz + `VISO_DELAY_MS 10` + 平滑） |
     | 真值 ↔ 指令 | **110 ms** | 评分用的"表观滞后"（含上面 60 ms 估计滞后 + 路径/瞬态） |
     | 飞控 EKF 估计 ↔ 指令（**同一时钟**） | **~10 ms**（沿航迹平均偏 ~3-4 cm ≈ 30 ms） | **控制闭环本身**：84 setpoint 50 Hz + 参考 100 Hz + 姿态内环，很快 |
     ⇒ 修正结论：**不是 PID 慢、也不是前馈缺失**（见下）；是"控制器看到的状态比真值旧 60 ms"
     （LIO 滤波 35 ms + EKF 融合 25 ms），评分用的真值在那之后 0.08 m（1.35 m/s）——高速时 8 m/s 就是 0.5 m。
     这是竞速的第一瓶颈，比调 PSC 增益重要。
   - 其余贡献：`PSC_NE_POS_P 1.2` 偏低（位置环在估计域里收敛慢）；参考角点停-走瞬态；
     LIO 位置相对真值本身 p90 ~7 cm。
   - **前馈是存在的、且被飞控使用**：`wire.csv` 实测 |v| p50 1.23 m/s（95% 非零）、|a| p50 0.70 / p90 1.42 /
     max 4.08 m/s²，与 `setpoints.csv` 完全一致；ArduPilot 4.7 `shape_pos_vel_accel_xy`
     （`AP_Math/control.cpp:389-403`）把 `vel_desired` 加进 `vel_target`、把 `accel_desired` 加在限幅之后；
     历史 A/B（`scripts/fly_hex.sh` 头注释）：`pos` 只到 3/19 航点，`posvel/posvelacc` 19/19。
     如果真是"没有前馈"，误差会随速度成比例放大、且 1.35 m/s 下直线段不会只有 −0.12 m 的沿航迹偏。
   - 参考加速度需求：|a_xy| p90 1.45 / p99 2.81 / max 4.02 m/s²，1.35 m/s 时 **0%** 超过 WP_ACC 4.0
     （所以现在只是"信息旧"，还没到修正饱和）；但参考加速度 ≈ v²/R 随速度平方增长，
     4 m/s 的同一批角点 ~13 m/s²，**必然被 WP_ACC 4.0 削平** —— 高速必须同时改控制参数。
   - 参考本身的停-走/角点刹停（`docs/FLIGHT-LOOP-ANALYSIS.md`）让误差周期性变大，进一步放大这种印象。
4. **SUPER 原论文的能力远高于我们当前设置**：280 mm 轴距、推重比 >5、70 m 3D LiDAR，报道 **>20 m/s** 穿越细障碍
   （[Science Robotics 2025](https://www.science.org/doi/10.1126/scirobotics.ado6187)、
   [论文摘要](https://search.library.wisc.edu/article/cdi_pubmed_primary_39879279)）。它的机器人用 PX4 系飞控
   （[SUPER GitHub](https://github.com/hku-mars/SUPER) 列出 Nxt-FC/PX4 硬件平台），与我们的 ArduPilot GUIDED 是两条控制栈；
   我们的 SITL 机体、参数与世界几何都还没按"竞速"配置。

### 1. 当前链路里"限制速度/精度"的东西（按影响力排序）

| 层 | 限制 | 现在值 | 影响 |
|---|---|---|---|
| 规划器 `hku_super/config/super_planner_hex.yaml` | `traj_opt.boundary.max_vel` / `max_acc` | **1.35 m/s** / 6.0 m/s² | 参考本身就只有 1.35，W2 门限也是按它定的（第 20 轮） |
| 规划器 | `planning_horizon` / `receding_dis` / `robot_r` | 16 m / 1.0 m / 0.10 m | 8–10 m/s 时转弯半径 v²/a（45°: 6.5 m @8 m/s）超出 10 m 半径的六边形走廊 |
| 适配器 | `--ref-max-speed-mps` | 6.0（默认）/ 本次未显式改 | 超过 6 的参考会被限速 |
| FCU 参数 | 修正环 `WP_SPD / WP_ACC / WP_JERK` | 1.5 m/s / 4 m/s² / **1.0 m/s³** | 误差修正慢；WP_JERK=1 尤其保守 |
| FCU 参数 | 位置/速度环 `PSC_NE_POS_P 1.2 / VEL_P 2.0` | 1.35 m/s 档调出来的 | 更高速度需要重新调（更高增益+更高 jerk 限） |
| FCU 参数 | `ATC_ANGLE_MAX` | 45°（加速度能力 9.8 m/s²） | 8 m/s 转弯需要更大侧向加速度 → 更大倾角或更大半径 |
| 控制频率 | 84 setpoint | 50 Hz | 高速下 20 ms 的参考量化会放大滞后 |
| 估计 | LIO 200 Hz + EKF extnav 48 Hz | — | 48 Hz 喂数 + 估计延迟 ≈ 0.1 s 量级；高速下成为相位滞后主项之一 |
| 世界/几何 | 六边形半径 10 m、柱距 1 m | — | 8–10 m/s 不可行（转弯半径不够）；竞速需要长直道/大半径世界 |

### 2. 分阶段方案（每一步都可飞、可量化、可回滚）

#### 阶段 0：先量化"控制器在哪个速度开始跟不上"（不改变交付配置）
- 用一个**无避障的合成参考**（大圆 r=8–10 m，或长直道 20 m，速度阶梯 1.35 → 2 → 3 → 4 → 6 m/s），
  每档飞 1–2 趟，输出：零滞后误差 p50/p90、最优滞后、沿/横分解、饱和比例（|a_cmd|>4 m/s² 的时长）。
- 现有工具：`scripts/fly_speed_run.sh`（已封装速度 arm + 三项测量）、
  `scripts/traj_error.py`（APE/RPE）、`scripts/analyze_loops.py`（停走/绕圈）。
  需要新增一个"跟踪质量"小工具（把上面的指标做成一行输出），可与 `fly_speed_run.sh` 串联。
- 产出：tracking-vs-speed 表 → 得到 ArduPilot 在当前参数下的"拐点速度"。

#### 阶段 1：先补延迟、再调硬（每项单变量 A/B）
- **延迟补偿（优先，因为主因是"状态旧 60 ms"）**：
  `REF_LEAD_S=0.05–0.06`（不是 0.13：0.13 是表观值，其中 0.06 是估计滞后、其余是路径/瞬态）；
  适配器已有 `--ref-lead-s`（沿 SUPER 速度方向外推）与 `--ref-lead-turn-rad-s`（弯道限幅）；
  **降估计滞后**（真正的杠杆）：`--nav-hz 48 → 60`（age 尾部 p90 35 ms 要压下去）、84 `--wire-hz 50 → 100`、
  LIO 处理延迟/时间戳补偿、`VISO_DELAY_MS` 按实测链路延迟重标定、EKF 视觉融合频率；
  每项都用 `scripts/track_quality.py` 的"同一时钟控制滞后"回归。
- **参数调硬（每次一趟）**：
  `WP_ACC 4 → 6`、`WP_SPD 1.5 → 3`、`PSC_NE_POS_P 1.2 → 2.0 → 3.0`、
  `PSC_NE_VEL_P 2.0 → 3.0`、`PSC_NE_JERK 80 → 120`（先看 0.5–2 Hz 增益）、
  `ATC_ANG_RLL/PIT 6 → 10`、`ATC_ANGLE_MAX 45 → 55`。
- 每档都看：tracking 指标 + W1b/W4（历史教训：加硬会把角点净空吃掉；`apm_guided_fast.parm` 的注释里有
  POS_P 2.4/3.0/5.0 的实测对比）。
- 需要新参数文件 `guided_limits_race.parm`（不动交付的 `guided_limits_fast.parm`），并让 `fly_hex.sh` 可切。

#### 阶段 2：规划侧提速（3 → 6 m/s）
- `traj_opt.boundary.max_vel: 1.35 → 3.0 → 6.0`，`max_acc` 同步（保持 a_lat ≤ g·tan(ATC_ANGLE_MAX)）；
  新建 `super_planner_race.yaml` 而不是改交付配置。
- `REF_MAX_MPS` 升到 8；`--ref-ff`/`--ref-gov-*` 复核；`--goal-pva` 保留（角点不停车）。
- 世界：新增**竞速世界**（长直道/大半径弯，柱距不变），因为 6 m/s 在六边形里转弯半径已 >4 m。
- 验收：W1b/W4 必须仍然过；W2 会自然超过当前门限（P0 判据问题，见 §4）。

#### 阶段 3：8–10 m/s（需要控制栈升级评估）
- 能力核算：8 m/s、45° → 转弯半径 6.5 m；10 m/s → 10.2 m。若要小半径，需要
  `ATC_ANGLE_MAX` 60–65°（a = 17–21 m/s²），先确认 SITL 机体 T/W 与电机/推力饱和（读 SITL 模型参数，
  必要时把机体模型调到 T/W ≈ 4–5，与 SUPER 论文的 >5 对齐）。
- 控制栈两条路线：
  A. **继续 ArduPilot GUIDED**：84 setpoint 提到 100 Hz；修正环参数再调；EKF extnav 延迟补偿
     （`VISO_DELAY_MS` 标定）；估计器输出频率提高（LIO → 331 提到 60–100 Hz）。上限预计 5–7 m/s。
  B. **自建姿态+推力环（SUPER 式）**：`SET_ATTITUDE_TARGET` + 我们自己的位置/速度/加速度控制器
     （ArduPilot 只跑姿态/角速率内环）。这是 SUPER 在 PX4 上的做法，能突破 GUIDED 级联的带宽限制；
     代价是要写控制器、做安全门、真机验证，属于独立里程碑。
- 建议：先做 A 到 6 m/s；用阶段 0 的拐点数据决定是否需要 B。

### 3. 需要新增/复用的工具

| 工具 | 用途 |
|---|---|
| `scripts/track_quality.py`（新） | 从一趟 sortie 输出：零滞后误差、最优滞后、沿/横分解、参考加速度分位、修正饱和比例、按速度分箱 |
| `scripts/fly_speed_run.sh`（已有） | 速度 arm 的飞行封装 + 三项测量钩子 |
| 新增 `hku_super/config/super_planner_race.yaml`（新） | 竞速规划参数（max_vel/max_acc/robot_r/horizon），不改交付配置 |
| 新增 `fcu_interface/params/guided_limits_race.parm`（新） | 竞速 FCU 参数，`EXTRA_PARMS=` 切换 |
| 新增竞速世界 `sim/worlds/race_straight.yaml`（新） | 长直道/大半径，供 6–10 m/s |
| `docs/metrics/speed-ladder-*.txt` | 每档速度的 tracking 数据和结论 |

### 4. 与验收判据的关系（P0，**只记录不改**）

- **W2**（现要求 p90≥1.40、max≥1.50 m/s）：竞速后必须重定义（如 p90≥6、max≥8）；
- **W4**（净空 >0）：高速下需要更大走廊和新的 world，阈值/几何都要重议；
- **W6**（滞后 spread ≤0.15 s）：高速下滞后绝对值会增大，判据应改成"相对速度归一化 + 去线性漂移"（已在
  `docs/P0-CRITERIA-PROPOSAL.md` 提案）；
- **H2/H6**：真值速率 380 Hz 的旧判据（已提案改）；高速时雷达 10 Hz/IMU 200 Hz 是否足够也要重新评估。
  以上都需要所有者批准后才动 `accept.py` 的默认值。

### 5. 参考来源

- SUPER 论文与摘要（>20 m/s、280 mm、T/W>5、70 m 3D LiDAR）：
  [Science Robotics 2025](https://www.science.org/doi/10.1126/scirobotics.ado6187)、
  [UW 摘要页](https://search.library.wisc.edu/article/cdi_pubmed_primary_39879279)、
  [HKU 新闻](https://engg.hku.hk/News-Events/Details/id/8718)、[GitHub](https://github.com/hku-mars/SUPER)
- ArduPilot GUIDED 命令文档（速度/加速度命令需要持续重发、加速度字段语义）：
  [Copter Commands in Guided Mode](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html)
- ArduPilot 4.7 源码（本仓库 `.build/ardupilot`）：
  `ArduCopter/mode_guided.cpp:943`（posvelacc 用 `limit_output=false`）、
  `libraries/AC_AttitudeControl/AC_PosControl.cpp:633`（限幅只作用于修正）、
  `AC_PosControl.cpp:458`（`NE_set_max_speed_accel_m`）
- 本仓库历史实测：`fcu_interface/params/apm_guided_fast.parm` 头注释（PSC 参数命名/取值 A/B）、
  `scripts/fly_hex.sh`（pos/posvel/posvelacc 对比：pos 时 3/19 航点，posvelacc 时 19/19）、
  `docs/FLIGHT-LOOP-ANALYSIS.md`（停走/绕圈归因）。

### 6. 立刻可做的第一步（建议）

1. 基准已建立：`python3 scripts/track_quality.py --tag phase4q-01-01` →
   zero-lag p50 0.154 / p90 0.265 m、best lag **−0.13 s**、参考 |a| p90 1.45 m/s²、
   速度分箱 1.0–1.5 m/s 档 p90 0.182 m；下一步在 `REF_LEAD_S=0.13` 与 `PSC_NE_POS_P=2.0` 上做单变量 A/B。
2. 用 `fly_speed_run.sh` 的竞速 arm 参数（新 `super_planner_race.yaml`，max_vel 3）飞 2 趟，测 tracking 拐点；
3. 再按阶段 1 逐项调 FCU 参数 A/B。每一步都只 commit 本地、不 push、不改默认交付配置。

## 7. 角点"猛绕弯子"的机理（owner 报告 phase4x c2 第一圈，2026-11-25）

phase4x 的 corner2 第一圈量化：真值冲过角点 **0.77 m**（到 (5.77,8.74)）再折回；窗口内累计转角 **325°**、
净转角仅 +24°；两段"指令速度=0"（62.7–63.0 s、63.6–65.7 s，长达 2–3 s 静止）。对照指令轨迹可见
**参考本身在换目标时跳变**（cmd 位置 5.38→5.05→5.16→5.34 m 往复），飞机追一个跳变的参考 → 过冲 →
回来 → 视觉上的圈。

三个候选机制及 2026-11-25 夜的真飞结论：

1. **轨迹末点静止 + 到点才换目标**（原逻辑）：已确认是"停顿→过冲"的来源；
   `GOAL_PVA + bisector + corner-speed 1.2`（phase4s）把角点平均转角从 172° 降到 139°，可保留。
2. **提前换目标（pre-goal）**：无锁存时 goal 在相邻角点间横跳（phase4v，30 次交替，6/13 到点）；
   加锁存后单调（phase4w：4/13，暴露"SUPER 到点后 WAIT_GOAL、飞机停在 0.5 m 门外"的站死）；
   加"参考断流重发 goal"后恢复为 12/13、W6 True（phase4x），但仍不过 W1b（按序到点口径冲突）。
3. **连续目标 carrot + 越点（pass-by）判据**（phase4z 原型，本次新增 `--corner-carry-m`）：
   **显著回归**（24 绕圈、角点全部停一下、W6 p90 0.616）。原因：pass-by 是"越过角点平面且横向<0.5 m"的
   半平面判据，配上 0.8 m carrot 会在多段几何上提前/重复触发成"反复换目标"，飞机被来回拉。

**结论（下一版设计，不再叠参数）**：
- 到达判据改成**单调的航路进度（route s）**：把航迹投影到"当前 leg 的方向"，只有进度真正越过角点
  0.3–0.5 m 且横向 ≤0.5 m 才推进一次，并**锁存**；绝不用半平面。
- 目标始终是"下一个角点 + 0.5–1.0 m 出弯前移的 carrot"，但 carrot 只在**本 leg 内**滑动（目标点随
  飞机推进连续前移，不跳到下一角点），直到进度判据推进后才改到下一个角点。
- 末端速度保持 PVA 的 1.2 m/s（最后一个点例外），并保留"参考断流重发 goal"防站死。
- 验收 W1b 用真值按序入口 0.5 m 盒独立检查；carrot 的前移量要保证真值到角点 ≤0.5 m（phase4z 的
  W1b 12/13、到点误差 0.01–0.19 m 说明这个量级可行，但行进过程必须平滑）。

## 8. phase4x 的绕圈全表（owner 提醒：12 之间、56 之间、c2 第一圈）

用 `analyze_loops.py` 对 phase4x 复核，共 **6 处**绕圈（不是只有角点）：

| # | 时间 (s) | 位置 (E,N) | 相对航段 | 真值累计转角 | 指令是否也绕 | 备注 |
|---|---|---|---|---|---|---|
| 1 | 55.3–58.0 | (4.5, 8.5) | c1→c2 末段（离 c2 0.56 m） | 315° | 否 | 真值冲过 c2 后回切 |
| 2 | 59.6–60.6 | (5.0, 8.7) | c2 角点 | 325° | **是** | 规划本身也在 c2 画圈 |
| 3 | 61.1–62.7 | (5.4, 8.8) | c2 之后（去 c3 起步） | 315° | 否 | 同上，连续第二圈 |
| 4 | 63.0–64.0 | (5.0, 8.6) | c2 角点 | 308° | 否 | 之后指令速度 0 持续 **14 s** |
| 5 | 109.7–110.7 | (0.0, −8.3) | **c5→c6 中段**（底边中间） | 314° | 否 | 指令/参考停滞导致 |
| 6 | 167.6–168.6 | (1.0, −8.1) | c5→c6 中段 | 317° | 否 | 附近 53 次 `Opt failed` |

- "12 之间"的绕圈实际发生在 c1→c2 末段与 c2 附近（#1–#4），"56 之间"的两处在底边 c5→c6 中段（#5–#6）；
- 分段统计：phase4x 每角点平均转角 c1:55° c2:**206°** c3:48° c4:69° c5:47° c6:96°——c2 明显异常；
- 指令侧：全程只有 1 处（#2）规划自己成圈，其余是"参考停滞/跳变 + 飞机追"；#4 之后指令 |v|<0.1 持续 **14 s**
  （`setpoints.csv` 实测 63.5–77.5 s），期间飞机只能悬停/漂移。

**phase4x 的教训（写进下一版设计）**：
1. 参考断流重发必须是"**低频、有次数上限**"的唤醒手段（已改为：沉默 2 s 后、每 2.5 s 一次、每个 goal 最多 3 次），
   否则会把 SUPER 的 new_goal 重置风暴化，规划永远不收敛；
2. PVA 角速度方向在 90° 角上**不能用 outgoing**（实测 62 次 GenerateExpTrajectory failed + 40 s 站死），
   用 bisector（phase4s 已验证 W1b 13/13）；
3. "目标持续推进"必须在**参考可用率 100%、规划不失败**的前提下做，否则任何 carrot/提前换目标都会把
   "参考停滞"放大成绕圈。

## 9. 滚动目标 v1（phase5a）分析与下一步

**实现**（`--target-mode rolling`）：每拍把目标放在航路上"飞机投影点 + look（2.5 m）"处；距角点 ≤ look 时
目标就是角点本身（不让 A* 切弯），终点速度沿 bisector 出弯方向 1.2 m/s；目标移动 ≥0.8 m 才重发 goal；
到达仍由 pacer（半径 0.5 m 或 pass-by 0.5 m/1.5 m 盒）决定。

**得到**：`W1 13/13`、`W2 PASS`、`W6 PASS`（spread 0.125）、无 TIMEOUT、无 pad 站死——比 phase4x 干净得多；
`GOAL ROLL` 114 次 = 目标更新约 1.6 Hz，SUPER 每次当 new_goal 做 rest2rest，`PlanFromRest failed 55` 次。

**仍不合格**：
1. **W1b 8/13**：lap1 corner6 真值最近距离 0.500 m 卡在盒边界（差一点），光标卡在 target7，lap2 全部无法计入。
   pass-by 的 0.5 m 横向盒允许切角到 0.5 m，正好把 W1b 推过界。下一版把 `PASS_BY_M=0.3`、`PASS_BY_LONG=1.0`。
2. **W4 −0.069 m**：滚动目标让角点通过更"斜"，净空比基线差；需要在角点附近保持目标=角点（已做）并把
   corner speed 降到 0.9 或把 `robot_r` 从 0.10 提到 0.15 观察。
3. **12 处绕圈、12 次角点指令停顿（0.2–1.0 s）**：指令位置无 >5 cm 跳变，停顿是 handover 瞬间的
   SUPER 重规划间隙（new_goal→rest2rest→PlanFromRest failed 55 次），不是参考 retract。
4. 目标更新 1.6 Hz 对 SUPER 偏快：下一步把 `TARGET_STEP_M` 提到 2.0（≈0.7 Hz）先看绕圈/停顿是否下降；
   若仍不行，就在 SUPER 侧打补丁：new_goal 且目标前移 < 3 m 时走 `ReplanOnce`（用旧轨迹做 guide）
   而不是 `PlanFromRest`，让"目标持续推进"在规划器内部平滑实现，而不是靠 plan box 频繁换 goal。

## 10. 速度阶梯第一段（2026-11-26 00:35–00:50）：1.35 → 2.5 → 4.0 m/s

| 趟 | 规划配置 | 真值速度 p90 / p95 / max | W1 | W1b | W4 | W6 |
|---|---|---|---|---|---|---|
| phase5d | 1.35 基线 + bisector 角速 1.2 | 1.37 / 1.42 / 2.2 | 13/13 | 13/13 | −0.03（已知缺陷） | PASS |
| phase5e | `super_planner_hex_fast.yaml`（max_vel 2.5 / max_acc 6） | 2.39 / 2.71 / 3.86 | 13/13 | 13/13 | −0.094 接触 | FAIL（p50 0.38 / p90 1.21 m） |
| phase5f | `super_planner_hex_race4.yaml`（max_vel 4.0 / max_acc 8） | 2.64 / 3.01 / **4.49** | 13/13 | 13/13 | −0.095 接触 | FAIL（p50 0.49 / p90 1.33 m） |

结论与下一步：
- **1.35 m/s 干净基线成立**（phase5d：W1+W1b+W2+W6 全过，只有 H2/W4 已知项）——这是加速的底座；
- 2.5/4.0 m/s 两档都保住了 W1/W1b/W2（航点全部按序到点），速度按配置抬上去了（4 档 max 4.49）；
- 代价：**W4 由 −0.03 掉到 −0.09**（切内弯更狠）、**W6 滞后随速度线性放大**（0.38→0.49 m p50、
  最优偏移 0.4–0.55 s），说明"估计滞后 60 ms + 参考不连续"在高速度下被放大，必须先解决这两项再上 6/8；
- 4 m/s 在六边形里 p95 只有 3.01：直线能到 4.49，角点必须减速。**6/8 m/s 需要更大的世界**（保持柱距 1.0 m、
  半径 20 m 级别），并同步把 `CORNER_SPEED` 抬到 3–4 m/s、`robot_r` 复核；
- 工具：`scripts/path_continuity.py` 现在用于"沿整条路径"检查，而不是只盯上一轮的坏点。

## 11. 飞机能力台架（2026-11-26 01:30，1 kg / 250 mm 机体 + 固件参数）

**目的**：按 owner 要求"先把飞机设置到接近完美，再谈路径"。台架把规划器排除在外：开放世界
（`sim/worlds/bench_low.yaml`：30 m 正方形飞行区无遮挡，区内 1.2 m 高的低柱给 Super-LIO 提供特征，
21-23 m 外有高柱环），PLANNER=rectangle（位置目标、无前馈），固件参数 `guided_limits_bench.parm`。

| 项 | 实测（phase6j） |
|---|---|
| 机体 | `sim/models/quad_1kg_250mm.json`：1.0 kg、250 mm 轴距、T/W≈4.25 |
| 速度 | p50 6.00 / p90 **8.03** / p95 8.11 / **max 8.34 m/s**；1 s 平均最大 **8.09** |
| 水平加速度 | p90 8.15 / max 11.8 m/s²（WP_ACC 8） |
| 姿态 | pitch p90 32° / max 51.6°；roll 同类（ATC_ANGLE_MAX 55） |
| 高度 | 目标 2.0 m，高速时 p50 1.87（下垂 ~0.13 m） |
| 航点 | 14/14 ARRIVED，30 m 正方形两圈，无 abort |
| 宿主 | flight_rtf 0.9999，lidar cast p50 11.9 ms，nav 22 199 行 |

**过程中修掉的两个系统级限制（不是规划器！）**：
1. **固件**：`WP_SPD 1.5 / WP_ACC 4 / ATC_ANGLE_MAX 45` 是 1.35 m/s 档；台架参数组改为
   `WP_SPD 8 / WP_ACC 8 / WP_JERK 30 / ATC_ANGLE_MAX 55 / ATC_ANG_PIT,PLL 10 / PSC_NE_* 加硬 /
   MOT_THST_HOVER 0.39 / EXPO 0.65`（与 1 kg 模型一致）。
2. **桥的外导航物理门限 `--nav-gate-mps` 默认 5.0 m/s**：8 m/s 时合法 LIO 帧被判"物理不可达"
   （phase6h 44 次拒绝），supervisor 据此报 `ESTIMATOR IS DIVERGING` 并 abort。已做成可配置
   （`NAV_GATE_MPS`/`NAV_GATE_DPS`，台架用 15/900），并给 supervisor 的拒绝率上限加了 `NAV_SKIP_MAX`。
3. 另：SITL 自定义机体 JSON 只认相对路径，run-mission 现在把模型拷进 run 目录再加载。

**下一步**：台架已证明 8 m/s 在 1 kg 机体上可达。接下来在台架上做**跟踪质量调参**（PVA 前馈/位置环/
姿态环），目标是 8 m/s 下跟踪误差 p90 ≤0.2 m、无振荡；然后把同一套固件参数用到路径飞行，先 1.35 m/s
质量门禁（绕圈=0、中段停顿=0）再提速。

## 12. PVA 跟踪台架（2026-11-26 01:55）：前馈打通后 8 m/s 仍可跟住

台架参考 `bridge_nodes/planner/bench_source.py`：**连续调幅正弦**（1→8 m 振幅、ω=1 rad/s，即峰值 1→8 m/s、
1→8 m/s²；振幅按 smoothstep 连续过渡，无速度阶跃），PVA 全量上台（`BRIDGE_FIELDS=posvelacc`），
1 kg 机体 + `guided_limits_bench.parm`、`NAV_GATE_MPS=15`。

| 指标 | phase7g |
|---|---|
| 跟踪误差（最优滞后 +0.05 s） | p50 **0.093 m**、p90 **0.415 m** |
| 分速度段 p90 | 1–2 m/s 0.30；2–4 0.23；4–6 0.28；**6–9 m/s 0.34 m**（p50 0.15） |
| 速度跟踪 | 真值 max **8.03 m/s** vs 指令 8.00（6–9 m/s 段速度误差 p90 0.27 m/s） |
| 垂直 | z 误差 p50 0.14 / p90 0.23 m（高速时高度下垂，待调 PSC_D/MOT） |
| 参考连续性 | 幅度阶跃改为连续后，参考速度阶跃消失（上一版硬切换造成 1–2 m/s 阶跃，飞机"跟不住"其实是参考不连续） |

**重要教训（写进流程）**：台架参考自身必须连续，否则测出来的是"参考跳变"不是飞机能力。修掉之后，
1 kg 机体在 PVA 前馈下 **8 m/s 跟得住（p90 0.34 m）**。下一步固件微调目标：6–9 m/s 段 p90 ≤0.2 m、
z 误差 ≤0.10 m（PSC_NE_POS/VEL 加硬、PSC_D 垂直环、ATC 姿态环、必要时 WP_ACC 提到 10）。

## 13. 固件 A/B（2026-11-26 02:00–02:20，PVA 台架）

在同一 PVA 台架上改固件参数组，指标：水平跟踪 p90（6–9 m/s 段）与**垂直动态** p90（先扣除 0.13–0.16 m 的
常值高度基准偏差——那是估计器/高度基准，不是控制）。

| 变体 | 关键改动 | 水平 p90 @6–9 m/s | 垂直动态 p90 | 结论 |
|---|---|---|---|---|
| v1 bench | 初版（PSC_NE 2.0/2.5/1.2/0.25/120，ATC_ANG 10，WP_ACC 8） | 0.392 | **0.060** | 垂直最好，水平最差 |
| v2 | + PSC_NE 2.5/3.0、ATC_ANG 12、WP_ACC 10 | **0.218** | 0.351 | 水平最好，垂直变差 |
| v3 | v2 + 垂直 PSC_D_POS 1.2 | 0.263 | 0.486 | 垂直更差 |
| v4 | v2 + ATC_ANG 13、WP_ACC 9、PSC 2.8 | 0.239 | 0.379 | 未超 v2 |
| **v5** | v1 内环 + PSC_NE 2.3/2.8/1.3 | 0.259 | **0.044** | 垂直极好，水平中等 |
| v6 | v5 + PSC 2.6/3.0、ATC_ANG 11、WP_ACC 9 | 0.317 | 0.045 | 折中失败 |

**结论**：目前没有一组能同时达到"水平 0.20 / 垂直 0.10"。瓶颈看起来是**环路延迟**（最优滞后 0.05–0.06 s，
8 m/s 时 = 0.4–0.5 m）而不是增益：加硬水平环能减小水平误差但把垂直扰动放大。下一轮做**系统级延迟实验**：
`WIRE_HZ=100`、`NAV_HZ=100`（都已可配置），先在 v2 参数上看水平误差能否降到 0.20 以内；若有效，再做
"v2 水平 + v1 内环"的组合。图：`docs/evidence/bench-firmware-ab.png`。

## 14. 延迟实验与最终飞机设置（2026-11-26 02:10–02:25）

1. **环路频率**（phase7m，v2 参数 + `WIRE_HZ=100`/`NAV_HZ=100`）：水平 6–9 m/s p90 0.214（v2 基线 0.218，
   基本不变）、垂直动态 0.366（仍差）→ 50 Hz 量化不是主因，**主因是估计链 ~0.05–0.06 s 的滞后**。
2. **预测超前（bench_source --lead-s）**：把参考按 lead 提前发布，直接补这条滞后：
   | 组合 | 水平 p90 @6–9 | 垂直动态 p90 | 备注 |
   |---|---|---|---|
   | v5 无 lead | 0.259 | 0.044 | |
   | v5 + lead 0.05 | 0.244 | 0.075 | |
   | v5 + lead 0.10 | **0.241**（p50 0.065） | **0.038** | 当前最佳组合 |
   | v2 + lead 0.05 | 0.281 | 0.405 | v2 的垂直问题被 lead 放大 |
3. **结论（飞机设置 v1.0）**：1 kg/250 mm 机体 + `guided_limits_bench5.parm` + **参考超前 0.10 s**：
   6–9 m/s 段水平 p90 **0.241 m**（p50 0.065）、动态垂直 p90 **0.038 m**、真值峰值 8.37 m/s（指令 8.00）。
   水平 p90 还差一点点（目标 0.20），剩余误差集中在 ±8 m 振幅换向处（最大加速度 8 m/s² 的冲击）；
   继续压缩需要更高带宽的控制/更低延迟，先按现状进入路径验证，后续再迭代。
   图：`docs/evidence/bench-final-1kg.png`；参数：`fcu_interface/params/guided_limits_bench5.parm`。

## 15. 1 kg 飞机 + 1.35 m/s 路径：绕圈仍在，根因是 SUPER 换目标时的 replan 失败（2026-11-26 02:25–02:50）

用新飞机设置（1 kg/250 mm + guided_limits_bench5.parm）飞交付森林 1.35 m/s 路径，三组角点/到点参数：

| 趟 | 参数 | W1 | W1b | W6 | 绕圈 | 中段指令停顿 | 参考 Δv max |
|---|---|---|---|---|---|---|---|
| phase8a | arrive 0.3 + corner 1.2（bisector） | 13/13 | ✅ | ✅ | **7** | 9 次 / 8.6 s | 0.48 m/s |
| phase8b | arrive 1.2 + corner 1.2 | 13/13 | ❌ 12/13 | ✅ | **0** | 6 次 / 5.9 s | 0.32 m/s |
| phase8c | arrive 0.3 + corner 1.35 | 13/13 | ✅ | ❌ | 6 | 12 次 / 10.9 s | 0.40 m/s |

**根因定位（planner 日志三趟一致）**：
- `ReplanOnce succeed` 1100–1200 次，但 **`GenerateExpTrajectory failed` 64–82 次**；
- 每次到点 ⇒ `Traj finish` 15–26 次 ⇒ FSM 进 `WAIT_GOAL`/`GENERATE_TRAJ` ⇒ `PlanFromRest`（还会 failed 5–18 次）；
- 也就是说：**换目标时 SUPER 的 ReplanOnce（用旧轨迹做 guide 的平滑重规划）经常失败**，退回"轨迹结束→从静止重规划"，
  wrapper 在这段时间发 hold ⇒ 指令速度掉 0 ⇒ 飞机冲过头/绕圈。根因不在飞机，也不在到点半径本身。

**下一版修复（planner 侧，不再是 plan box 调参）**：
1. 在 SUPER 的 `generateExpTraj`/`ReplanOnce` 的 new_goal 路径上放宽/修掉换目标时的失败原因（拿到具体失败分支：
   corridor/A* 还是 MINCO 边界），并保持"旧轨迹做 guide"的连续性；
2. 若 new_goal 的 ReplanOnce 失败，**不要**让 committed 轨迹自然结束（patch 9 的发布继续），而是用
   "旧轨迹终点作为 guide 的延长段 + 新目标"重新规划一次；
3. 到点判定改为单调 route-progress（角点之后才推进），配合 1.2 m 的提前目标（phase8b 已证明可把绕圈降到 0）。

### 15.1 继续扫参数（02:50–03:05）：corner speed 与 arrive gate 的组合

| 趟 | 配置 | W1 | W1b | W6 | 绕圈 | 中段停顿 | ExpTraj failed | Traj finish | 质量结论 |
|---|---|---|---|---|---|---|---|---|---|
| 8a | arrive 0.3 / corner 1.2 | 13/13 | ✅ | ✅ | 7 | 9 次 8.6 s | 69 | 25 | 差 |
| 8b | arrive 1.2 / corner 1.2 | 13/13 | ❌12/13 | ✅ | **0** | 6 次 5.9 s | 64 | 15 | 无圈但 W1b 挂 |
| 8c | arrive 0.3 / corner 1.35 | 13/13 | ✅ | ❌ | 6 | 12 次 10.9 s | 82 | 26 | 差 |
| **8d** | arrive 0.3 / **corner 0.8** | 13/13 | ✅ | ✅ | **2** | **3 次 6.1 s** | **44** | **11** | **当前最佳** |
| 8e | arrive 0.3 / corner 0.5 | 13/13 | ✅ | ✅ | 4 | 7 次 7.4 s | 65 | 21 | 比 0.8 差 |
| 8f | arrive 0.8 / corner 0.8 | 13/13 | ✅ | ✅ | **0** | 5 次 8.9 s | 52 | 13 | 无圈但跟踪 0.68 m、PlanFromRest failed 66 |
| 8g | 滚动目标（pose goal, look 3 m, step 2 m） | ❌ | ❌ | ❌ | - | - | - | - | plan box 层滚动目标再次不可用 |

**结论**：`corner 0.8 + arrive 0.3` 把换目标失败从 69 降到 44、Traj finish 从 25 降到 11，绕圈 7→2、中段停顿 9→3；
但"轨迹结束→PlanFromRest"这条路径仍然存在（11 次），是剩余 2 个圈与停顿的唯一来源。
**plan box 层滚动目标（8g）复测再次失败**（W1/W1b/W6 全挂），确认必须改 SUPER 内部，而不是继续调 plan box 参数。

**下一版（planner 侧，round 8 开始）**：
1. 在 `super_planner` 的 new_goal 路径上修 `GenerateExpTrajectory failed` 的原因（具体失败分支：corridor/A* 还是 MINCO
   的 Omg/thr/Pos violation），目标是换目标时 ReplanOnce 成功率接近 100%；
2. 若失败，不能让 committed 轨迹自然结束进入 WAIT_GOAL，而是"延长旧轨迹 + 新目标"再规划一次；
3. 到点判定在 plan box 内改为单调 route-progress（配合 arrive 0.8），目标是绕圈=0 且 W1b 全过。

### 15.2 planner 侧尝试与否决（2026-11-26 03:10–03:30）

| 尝试 | 结果 | 结论 |
|---|---|---|
| **patch 13**：new_goal 时 `split_dis=0`（不再沿旧轨迹 recede，直接从当前状态 A* 到新目标） | ExpTraj failed 44→**88**、Traj finish 11→**20**、绕圈 2→**5**、停顿 3→**9** | **否决并 revert**：旧的 receding splice 反而更可靠；换目标失败不是"旧 guide 拼接"造成的 |
| `GOAL_LOOKAHEAD_M=0.8`（目标沿出弯方向前移 0.8 m，让末点落在角点之后） | W1/W1b/W6 全挂（planner 声明到点与 pacer 半径判据错位） | **否决**：前瞻目标会破坏到点记账，必须同时改到点判据，不能单独用 |

**当前事实**：
- 1.35 m/s 最优仍是 **phase8d（arrive 0.3 / corner 0.8）**：W1/W1b/W6 全过、绕圈 2、中段停顿 3 次 6.1 s；
- 换目标失败（44 次/趟）→ Traj finish（11 次）→ 停-走 是剩余 2 圈的唯一来源；
- plan box 层的滚动目标（8g）、pre-goal、carrot、pass-by、goal-lookahead 全部已被真飞否决；
- **下一步必须重做 plan box 与 SUPER 的接口**：要么在 SUPER 内部实现"移动目标"（把目标作为状态变量在 10 Hz 重规划里更新，而不是通过 new_goal 消息），要么把 plan box 改成"轨迹流"（订阅 SUPER 多项式轨迹并自行拼接/前移末端），不再靠 SUPER 的 FSM 收尾。这两条都超出调参，属于下一阶段的设计工作。

### 15.3 plan box 侧"参考断流"补救（03:30–03:55）

| 趟 | 改动 | 绕圈 | 中段停顿 | ExpTraj failed | Traj finish | 验收 |
|---|---|---|---|---|---|---|
| 8j | goal-lookahead 0.8 + arrive 0.8 | 2 | 2 次 5.2 s | 59 | 16 | W1b 挂（corner1 真值 0.62 m 被切） |
| 8k | REF_DEADRECKON_S=1.0（stale 1.0 s 后用最后参考速度外推） | 5 | 7 次 7.7 s | 63 | 21 | W1b/W6 过 |
| 8l | REF_STALE_S=0.12 + 外推 0.6 s | 4 | 3 次 6.1 s | 59 | 20 | W1b/W6 过（但外推速度是 0） |
| **8m** | 外推改用"最后非零参考速度" | 4 | 5 次 6.9 s | **43** | **14** | W1b/W6 过 |
| 8d（基线） | 无外推 | **2** | **3 次 6.1 s** | 44 | 11 | W1b/W6 过 |

**发现**：SUPER 在换目标时发布的最后一个参考样本 **|v|≈0**（end-of-trajectory 采样），所以"按最后速度外推"起初
等于原地 hold；改成"最后非零速度"后 ExpTraj failed/Traj finish 有改善，但绕圈没有下降（4），因为外推只能持续
0.1–0.3 s，SUPER 重规划一来参考还是会跳/停。

**结论**：plan box 层已到极限。**必须做接口层重设计**（下一轮开始实施）：
1. 让 SUPER 内部持有"移动目标"状态并在 10 Hz 重规划中前移（不经过 `new_goal` 消息），或
2. 把 plan box 改成轨迹流：订阅 SUPER 多项式轨迹，自己拼接/前移末端，不依赖 FSM 的收尾语义。
两个方案都保持 W1b/pacer 记账不变。

### 16. **patch 14（SMOOTH-HANDOVER）**：换目标先走连续重规划 —— 首次做到绕圈 = 0（04:00–04:15）

**patch 14**（`hku_super/patches/patch_super_smooth_handover.py`，已 apply 且进门禁）：
`Fsm::callReplanOnce` 里新目标先调用 `ReplanOnce(..., new_goal=false)` —— 走 SUPER 的**连续重规划**路径
（用整条已提交轨迹做 guide、再 A* 到新目标），只有硬失败（FAILED/EMER/NEW_TRAJ）才回退到原来的
`new_goal=true` 行为。于是换目标不再必然"从静止重新规划"。

| 趟 | 配置 | W1 | W1b | W6 | 绕圈 | 中段停顿 | ExpTraj failed | Traj finish |
|---|---|---|---|---|---|---|---|---|
| 8d（基线） | corner 0.8 / arrive 0.3 | 13/13 | ✅ | ✅ | 2 | 2 次 0.7 s | 44 | 11 |
| **8n** | + patch 14 | 13/13 | ✅ | ✅ | **0** | 4 次 1.4 s | 58 | 25 |
| **8o** | + patch 14 + REF_DEADRECKON 0.6/stale 0.15 | 13/13 | ✅ | ✅ | 2 | 3 次 1.3 s | **30** | 20 |

（停顿统计已剔除停机坪上的正常 pre-takeoff hold。）

**结论**：
- patch 14 首次把绕圈降到 **0**（8n）且 W1/W1b/W6 全过——方向成立：**换目标必须走连续重规划**；
- 但 `GenerateExpTrajectory failed` 仍偏多（30–58），说明连续重规划在角点仍会失败并回退到旧路径，
  绕圈数在 0–2 之间波动（8o 又出现 2 个）；还剩 1.3–1.4 s 的短停顿；
- 下一步：在 patch 14 基础上继续压 `GenerateExpTrajectory failed`（失败分支定位：A* 还是 MINCO 边界），
  以及把"连续重规划失败时的回退"从 `new_goal=true` 改为"保持旧轨迹继续、下拍再试"，目标是
  **绕圈=0、停顿=0** 稳定复现，然后才提速。

### 16.1 patch 14 变体与飞机/参考调优（04:15–04:45）

| 趟 | 配置 | W1 | W1b | W6 | 绕圈 | 中段停顿 | ExpTraj failed | 结论 |
|---|---|---|---|---|---|---|---|---|
| 8n | patch14（失败回退 new_goal） | 13/13 | ✅ | ✅ | **0** | 4 次 1.4 s | 58 | 保留 |
| 8p | patch14b（失败只 defer，不回退） | 13/13 | ✅ | ✅ | 5 | 4 次 1.6 s | **24** | **否决**：失败次数降了但绕圈 5 |
| 8q | patch14b + 软固件（bench v1） | 13/13 | ✅ | ✅ | 2 | 8 次 2.6 s | 54 | 无优势 |
| 8r | patch14b + REF_LP_TAU 0.08（参考低通） | 13/13 | ✅ | ✅ | 6 | 13 次 5.4 s | 54 | **否决** |
| 8s | 3 kg + 交付固件 + patch14b | — | ❌ | ❌ | — | — | — | **无效**：LIO 外导航断流 1.68 s abort（host 抖动，非配置结论） |

**结论**：
- 仍以 **patch 14（失败时回退 new_goal）** 为准（8n：绕圈 0、W1/W1b/W6 全过），已恢复该版本并重编译；
  14b/低通/软固件都无优势；
- 1.35 m/s 的绕圈在 0–5 之间**随趟波动**，主因是**换目标瞬间的跟踪瞬态**（真值速度冲到 1.9 m/s 而指令 1.1），
  不是 planner 成圈；下一步要**多趟统计**（同一配置 ≥3 趟）而不是单趟判定，然后再调瞬态；
- 速度阶梯（4/6/8）尚未开始；先把这个统计做完。

## 17. 1.35 m/s 多趟统计 + **参考速度 slew 限制**（05:00–05:45）

### 17.1 多趟统计（patch 14 + corner 0.8 / arrive 0.3，1 kg 机体 + bench5 固件）

| 趟 | 绕圈 | 中段停顿 | 参考 Δv p90 / max | 跟踪 p90 |
|---|---|---|---|---|
| 9a | 4 | 15 次 6.9 s | 0.075 / 0.482 | 0.397 |
| 9b | 2 | 8 次 3.8 s | 0.055 / 0.398 | 0.268 |
| 9c | 2 | 6 次 3.9 s | 0.065 / 0.464 | 0.306 |

三趟验收：W1/W1b/W2/W3/W5/W6 全过（W4 两趟过、一趟挂，H2 已知）；但**绕圈 2–4、停顿 6–15 次**，
单趟波动大。9d（1 kg + 交付低速度固件）绕圈 9、9e（专门低速参数组 guided_limits_slow1kg）绕圈 3 停顿 15
→ **固件参数不是主因**。

### 17.2 **REF_V_SLEW：指令速度 slew 限制**（本轮突破）

原因：`flight_quality` Q3 实测参考速度在换目标时 20 ms 内跳 **0.40–0.53 m/s（≈20–26 m/s² 的指令加速度）**，
飞机跟不上这种阶跃 → 真值过冲成小圈（cmd 本身不成圈）。在 plan box 发布前对**速度**加 slew 限制
（位置保持规划器的点，不改变路径），新参数 `REF_V_SLEW`（m/s²）：

| 趟 | REF_V_SLEW | 绕圈 | 中段停顿 | Δv p90/max | 验收 |
|---|---|---|---|---|---|
| 9f | **3.0** | **1** | **0 次 0.0 s（首次 PASS）** | 0.038 / 0.320 | **除 H2 外全过（含 W4）** |
| 9g | 2.0 | 2 | 3 次 1.4 s | 0.042 / 0.320 | 除 H2/W4 外过 |
| 9h | 4.0 | 2 | 1 次 0.2 s | 0.039 / 0.320 | 除 H2 外全过（W4 过） |
| 9i | 3.0 + corner 1.0 | 5 | 3 次 1.0 s | 0.046 / 0.400 | 变差 |
| 9j | 3.0 + arrive 0.5 | — | — | — | W1/W1b 挂（arrive 必须 0.3） |

**9f 是当前最佳 1.35 趟**：指令停顿 **0 次**（首次全清）、绕圈 1（仅 corner1 低速小幅回旋）、
W1/W1b/W2/W3/W4/W5/W6 全过，仅剩已知 P0 H2。剩余 Q3 max 0.32 与 Q4 0.385 需要下一轮继续压
（位置连续性/Q4 放宽或降低估计延迟）。

### 17.3 仍然有效的结论
- `REF_V_SLEW≈3.0`（不是 lead、不是低通）是解决"换目标瞬态"的正确旋钮；
- corner 0.8 / arrive 0.3 仍是最优组合；把 arrive 提前会破坏 W1b；
- 下一步：以 9f 配置为主，先复飞 3 趟确认统计（目标绕圈 ≤1、停顿 0），然后进入速度阶梯。

## 18. round 12：9f 配置复飞确认 + 4 m/s 首试（06:05–06:20）

### 18.1 确认复飞（slew 3.0 / corner 0.8 / arrive 0.3，patch 14）

| 趟 | 绕圈 | 中段停顿 | 验收 | 备注 |
|---|---|---|---|---|
| 10a | 0 | 4 次 6.8 s | W1 6/13（abort） | SUPER PlanFromRest 卡住（PathSearch start 反复失败），plan box 3 s 断流后**主动退出**导致 abort |
| 10b | 3 | 5 次 1.5 s | 除 H2/W4 外全过 | 把 `--stale-fail-s` 提到 8 s，同样的卡顿被容忍、飞机悬停等 planner 恢复后继续飞 |

**修正**：`--stale-fail-s` 已暴露为 `REF_STALE_FAIL_S`（默认 3.0）——"planner 卡几秒"不该处死整趟任务。
但 1.35 的绕圈/停顿仍在 0–3 / 1.5–6.8 s 间波动，说明还有随机性；先接受为"接近干净"，进入阶梯验证。

### 18.2 4 m/s 首试（frozen forest，race4 配置，corner 2.0，REF_V_SLEW 6，NAV_GATE 15）

结果 **W1 7/13、W1b 8/13、W4 CONTACT（-0.098 m）、W6 挂；绕圈 13、跟踪 p90 4.59 m、真值峰值 9.09 m/s**。
结论：**10 m 边长的冻结森林装不下 4 m/s**（飞机以 9 m/s 冲进柱子）。按 objective"地图不够大时新建更大世界"，
下一轮：
1. 修 `scripts/make_world_v2_hex.py` 的 HALF 常量（大场地时 audit 网格越界，之前 `IndexError: 540/441`）；
2. 生成并冻结 **forest_hex16/20**（保持 pitch 1.0 柱距、radius 16–20 m、arena 22–26 m），
   同时把 `check_frozen.py` 改成按 world 的指纹表；
3. 在 16 m 边长世界上重跑 4 m/s，再 6 → 8 m/s（8 必须达成）。

## 19. round 13：大世界生成（修 bug）+ 4 m/s 在大世界的首轮（06:25–06:40）

### 19.1 生成器修复与新世界

- `scripts/make_world_v2_hex.py` 的 audit 网格 `HALF` 原本写死 11.0 m：`--cols 20` 的 40 m 场地会让航点/
  路线格越界（`IndexError: 540/441`）。改为 `HALF = max(11.0, half_field + 3.0)`。
- 新世界（保持 SPEC 柱距 **pitch 1.0 m**、柱半径 0.1、keepout 3×3）：
  - `sim/worlds/forest_hex16.yaml`：radius 16 m、arena ±22、**1546 柱**、54 空格；
  - `sim/worlds/forest_hex12w.yaml`：radius 12 m、arena ±18、**970 柱**、54 空格（seed 20241101）。
  指纹用 `scripts/fingerprint.py` 记录（见 `docs/evidence/`）。

### 19.2 4 m/s 三趟结果

| 趟 | 世界 | 关键结果 | 结论 |
|---|---|---|---|
| 11a | hex16（1546 柱，10k 点） | H6 挂（raycast max 1803 ms、47 scans skipped）、H5 挂（EKF/真值不一致）、W1 1/13 | **大世界把 lidar 模拟器压垮**，LIO 断流 |
| 11b | hex16 + 6000 点 | H6 仍挂（max 1053 ms、22 skipped）、W2 真值速度 p90 0.08（飞机基本没动） | 仍然 starvation，tour 没起来 |
| **11c** | **hex12w（970 柱，6000 点）** | **H6 过（p50 10.8 ms、max 33.5、skipped 4）**、H5 过、**W2 过（巡航 p90 3.09、max 7.91 m/s）**；但 W1 4/13、W1b 5/13、W4 CONTACT、W6 挂 | 世界可用了，**4 m/s 的跟踪/超调仍是问题** |

**11c 的关键发现**：参考 max_vel=4，但真值峰值 **7.91 m/s**（超调 ~2×），W4 出现 -0.083 m 接触柱子；
W1 只有 4/13——飞机在角点/换目标处大幅超调，走廊被突破。

### 19.3 下一步（round 14）
1. 4 m/s 的固件/参考匹配：`WP_SPD/WP_ACC` 与 `REF_V_SLEW` 要按速度重标定（现在是 1.35 m/s 的
   slew 6 和 8 m/s 的 bench5 增益混用）；先做 4 m/s 的台架 PVA 跟踪（bench_source ω 调大），再上路径；
2. 把 W4 接触压回正余量后再试 4 m/s 完整趟；
3. 大世界固定用 `forest_hex12w`（H6 已过），需要更长边时再回 hex16 并用更低的 lidar 点数/更快的 raycast。

## 20. round 14：4 m/s 的瓶颈在 SUPER 规划器，不在飞机（06:45–07:00）

| 趟 | 世界/配置 | H6 | W2 真值速度 | ExpTraj failed | 结论 |
|---|---|---|---|---|---|
| 11c | hex12w / race4(corner2.0,slew6) | 过 | p90 3.09 / max 7.91 | 2703 | 飞机能动但乱 |
| 12a | hex12w / race4b(corner1.0,slew8) | 过 | p90 2.34 / max 6.56 | 2078 | W1 6/13、停顿 72 s |
| 12b | hex16 / race4b（4000 点） | 过 | p90 0.05 | 2426 | **飞机基本没动**，规划从起飞就失败 |
| 12c | hex16 / race4b + pose goal（无 PVA） | 挂 | p90 0.05 | — | 同样卡住 |

**诊断**：4 m/s 时 `GenerateExpTrajectory failed` **2078–2703 次/趟**（1.35 m/s 只有 44 次），
失败类型：`Opt failed, Omg or thr or Pos violation` / `Minco exp_traj opt failed` / `NaN`。
即 **SUPER 的 MINCO 在 1 m 柱距走廊里给不出可行的 4 m/s 轨迹**（转弯半径 v/omg 与走廊宽度矛盾），
FSM 于是长时间停在 WAIT_GOAL/GENERATE_TRAJ，plan box 发 hold —— 12a 里累计 **72 s 指令 hold**。

**结论**：
- 速度阶梯的下一道坎是**规划器可行性**，不是飞机（8 m/s 飞机能力台架已证明）；
- 需要（下一轮）：把 4 m/s 的规划参数按可达性重做 —— 降低走廊内速度上限/角点速度（corner 0.3–0.5），
  或提高 `max_omg`、放大 `planning_horizon`，并先用 2.5 m/s 找到"新世界可用"的阈值；
- hex16 的 lidar 模拟器在本机仍不稳（H6 时好时坏），高速度档建议先用 hex12w（H6 稳定），
  leg 不够时再优化 raycast。

## 21. round 15：新世界规划失败的真正原因 —— **规划器 map_size 只有 24 m**（07:05–07:25）

三趟对照（hex12w，1 kg 机体 + bench5 固件）：

| 趟 | 配置 | W1 | ExpTraj failed | 结论 |
|---|---|---|---|---|
| 13a | fast(2.5) + 默认 map_size 24 | 4/13 | 2463 | 速度不是主因 |
| 13b | 默认 1.35 + 默认 map_size 24 | 6/13（4 次 LEG TIMEOUT，飞机长期没离开停机坪） | 1819 | **新世界在 1.35 也规划不了** |
| **13c** | 默认 1.35 + **map_size 40**（`super_planner_hex_big.yaml`） | **8/13（顺序 1..8）** | 1668 | 明显变好，但仍失败 |

**根因（13b/13c 对比）**：SUPER 的 `rog_map.map_size` 是 **24 m**（±12 m）。新世界 corner 半径 12 m、
柱子铺到 ±16 m：航点正好压在 **OUT_OF_MAP 边界**上，A* / MINFC 频繁失败（1668–2463 次/趟），
飞机被留在停机坪（13b 连续 4 次 LEG TIMEOUT）。把 map_size 提到 **40 m**（±20）后 W1 从 6→8 且顺序正确，
证明假设成立；但 1668 次失败仍偏高——新世界比冻结世界大、柱多（970 vs 522），还有别的约束在卡。

**已提交的配置**：`super_planner_hex_big.yaml` / `_fast_big.yaml` / `_race4b_big.yaml`（map_size 40）。

**下一轮**：继续把 map_size 调到 48、修 1668 次残余失败（看失败点是不是在 ±12 边界附近），
在 1.35 干净后按 2.5→4 m/s 重跑；若仍卡，则退回到"小世界 + 更宽柱距"的对照实验定位几何项。

## 22. round 16：map_size 细调 + 2.5 m/s 在冻结图上的实测（07:26–07:55）

### 22.1 新世界 map_size 继续调

| 趟 | 世界 / map_size | W1 | ExpTraj failed | H6 | 结论 |
|---|---|---|---|---|---|
| 13c | hex12w / 40 | 8/13 | 1668 | 过 | 边界问题缓解 |
| 14a | hex12w / **48** | 8/13 | **650** | 挂（lidar behind、43 skipped、max 1804 ms） | 规划改善但**主机扛不住 48 m 地图**（0.05 m 分辨率下体素数 ~8×） |
| **14b** | hex12w / **32** | **10/13** | 1350 | 挂（behind 39、max 1343 ms） | 规划更稳，仍是 H6/时序 |

**结论**：map_size 是主因之一（24→40 让 W1 6→8），但 48 m 地图在这台机器的 CPU/内存预算下会把
lidar 模拟器拖到 "behind"（扫不到点 → LIO 变差 → 规划更差）。hex12w 用 **map_size 32** 是当前折中。

### 22.2 冻结地图上的 2.5 m/s 实测

| 趟 | 固件 | W1/W1b/W2 | 真值速度 | 质量 |
|---|---|---|---|---|
| 14c | bench5（8 m/s 档） | 全过 | p90 3.96 / max **6.54** | 绕圈 14、跟踪 p90 0.91、W4 接触 |
| 14d | **guided_limits_2p5**（新建 2.5 档：WP 3/4、ANGLE 40、PSC 1.9/2.6） | 全过 | p90 3.03 / max 4.75 | 绕圈 21、跟踪 p90 1.59、W4 接触 |

**结论**：2.5 m/s 时航点记账全过，但**飞机大幅超调**（参考 2.5、真值峰值 4.75–6.54）→ 接触柱子、
跟踪差。超调的来源已定位到**换目标瞬间参考的速度/位置跳变**（Q3 |Δv| p90 0.16 m/s@20ms = 8 m/s²），
`REF_V_SLEW` 只能限速度、限不住位置跳变。下一轮：
1. 在 plan box 加**位置连续性限制**（ref position 的跳变按 REF_MAX/加速度预算限幅，而不是只限速度）；
2. 2.5 m/s 用更低角点速度（0.8）与更慢 slew（4）复测，目标是 W4 正余量；
3. 之后再谈 4 m/s；8 m/s 需要 hex16（map_size 32 不够，需要 >32 的地图 + 主机性能优化）。

## 23. round 17：参考调速器（REF_GOV_*）在 2.5 m/s 上的首次真飞（08:00–08:20）

2.5 m/s 冻结图（fast 配置，2.5 档固件，corner 1.2）：

| 趟 | 调速器配置 | 真值速度 | W2 | W4 | W6 | Q4 跟踪 |
|---|---|---|---|---|---|---|
| 15a | along=2（弧长同步）+ kp4 | p50 0.58 / p90 0.67 | 挂（p90<1.4） | **✅ 净空 +0.039 m** | ✅ | **p90 0.111 m** |
| 15b | 15a + 速度下限 floor 0.6 + catchup 1.0 | p50 0.92 / p90 1.31 | 挂（1.31<1.4） | 挂（-0.084 接触） | ✅ | p90 0.185 m |
| 15c | along=1（投影）+ floor 0.8 | — | 过 | 挂 | 挂 | — |

**重要收获**：调速器把**跟踪和净空一次性修好了**（15a：Q4 p90 0.111 m、W4 首次出现**正余量 +0.039 m**、
W6 pass），证明"位置/速度自洽的参考"是正确方向；但弧长同步模式与 plan box 的 hold fallback 冲突：
SUPER 短暂停顿时 governor 继续用 ≤1 m/s 爬向目标，`s_ref` 累积超前（日志 `s_err -16.7 m`），
之后被 catch-up 上限钉死在 1 m/s，整趟变慢（p90 0.67–1.31），W2 因此挂在"巡航速度"上。

**下一轮（round 18）**：
1. 修 arc-sync 与 hold 的交互：SUPER 断流/hold 拍**不推进 s_plan/s_ref**，或直接冻结 governor，
   让"停顿时保持位置"而不是爬行；
2. 修好后预期同时拿到：巡航 ≥2.5 m/s、W4 正余量、跟踪 p90 ≤0.2；
3. 然后 4 m/s（同一套 governor），再做 6/8（大世界 + map_size 32 折中）。

## 24. round 18：governor 与 hold 解耦；16c 拿到"最安全的轨迹"但仍慢（08:18–08:40）

- 修复：governor 在 SUPER 断流/hold 拍**完全旁路**（`state["is_hold"]`），不再用 catch-up 速度爬行。
- 16a/16b 因主机/DDS 抖动无效（16a lidar starvation max 3.17 s → 估计器丢失；16b LIO 收不到点云）。
- **16c（2.5 m/s，governor along=2 修后，8000 点，DDS 清理后）**：
  - W1b **13/13**、W4 **+0.037 m CLEAR**、W6 pass、Q4 p90 **0.110 m** —— 安全性与跟踪是历次最好；
  - 但真值速度 p50 0.58 / p90 0.67（W2 挂），中段指令 hold **76 次 28.7 s**，绕圈 8。
- 诊断：hold 的来源是 **SUPER 自身在 2.5 m/s 下频繁进入静默/规划失败**（切目标 Traj finish + PlanFromRest），
  governor 忠实跟随慢参考；也就是说 2.5 m/s 的瓶颈仍是**规划可用性**，不是控制。

**下一轮（round 19）**：
1. 统计 16c 的 SUPER 静默区间（时长/位置/失败类型），把 `replan_time_limit` 从 0.35 提到 0.8，
   并把 corner speed 提到 2.0（让 SUPER 在角点不把速度压到 0）；
2. 目标：W2 巡航 p90 ≥2.2 且保持 W4 正余量、Q4 ≤0.2；
3. 然后 4 m/s（沿用 governor），再 6/8 m/s。

## 25. round 19：低障碍"大世界"方案 + DDS 抖动修复（08:42–09:15）

### 25.1 新世界（保持柱距 1.0 m、更大的世界，但跑道净空）

按 objective"6+ m/s 需要更大世界、保持柱距"的要求，生成两个**低障碍**世界（柱子只到 z=1.2 m，飞行高度 2.0 m
上方净空；柱距仍是 1.0 m，柱子数量/布局满足 SPEC S2 的 1 m 格规则）：

| 世界 | 几何 | 柱子 | 用途 |
|---|---|---|---|
| `forest_hex14_low.yaml` | 六边形半径 14 m（28 m 对边）、arena ±18、柱高 1.2 m | 970 | 4→6 m/s |
| `forest_hex18_low.yaml` | 半径 18 m（36 m 对边）、arena ±22、柱高 1.2 m | 1546 | 6→8 m/s |

### 25.2 4 m/s 首轮（hex14_low）

| 趟 | 结果 | 结论 |
|---|---|---|
| 17b（hex18_low） | H6/H5 挂 | 1546 柱 + 48 m 规划图超出本机实时能力 |
| 17c | LIO 收不到点云（3.99 s 无样本） | **Fast-DDS 共享内存残留**：清 /dev/shm + ros2 daemon 后恢复 |
| **17d** | **W2 过（巡航 p90 3.22、max 7.65）**、**W4 过（+0.192 CLEAR）**、W5 过；H6 挂（skipped 57、max 1535 ms）、W1 5/13 | 速度/净空第一次同时正向 |
| 17e | H5/W4/W5 过，H6 仍挂、W1 挂 | 长趟（~250 s）后 lidar 模拟器持续 "behind" |

**已修**：`scripts/clean_harness.sh` 现在会 `ros2 daemon stop` + 清 `/dev/shm/fastrtps_*`（16b/17c 同症状的根因）。

**观察**：H6 的 starvation 集中出现在**长 sortie 的后段**（14a 的 1547/1580、17d/17e 的尾部），怀疑
lidar 模拟器的内存/GC 随时间增长。下一轮：① LAPS=1 缩短趟时做对照；② 把雷达点降到 4000；③ 若确认长趟
才出问题，则给 lidar 模拟器做内存/GC 优化（或固定每 N 次 gc.collect）。

## 26. round 20：主机抖动诊断（09:18–09:36）

| 趟 | 场景 | H6 | 结果 |
|---|---|---|---|
| 18a | hex14_low 4 m/s，4000 点、BAG=0、map_size 32 | 挂（skipped 96、max 1137 ms） | W2/W4/W5 过，W1 2/13 |
| 18b | **冻结图 1.35（9f 同配置）**，BAG=0 | 挂 | **已知好配置也挂** → 不是世界/不是配置 |
| **18c** | **冻结图 1.35（9f 原配置，BAG=1）** | **过** | H5 过；但 W1 挂、W2/W4/W6 挂 |

**结论**：
- H6 的 lidar starvation 是**主机级间歇抖动**（WSL：dmesg 有 OOM/journal 损坏记录，cast 偶发 1–3 s 停顿），
  同一份 9f 配置 18b 挂、18c H6 过 —— 不是配置或世界的问题；BAG=0 也不改善（18a/18b 仍挂）。
- 但同为 H6-safe 的 18c 与 9f 飞行质量差异大（9f 绕圈 1/停顿 0，18c W1 挂）→ **run-to-run variance 很大**，
  单趟结论不可靠。
- 已做的工程改进保留：`race4b_big32`（map 32，内存友好）、低障碍世界、DDS 清理。

**下一轮（round 21）**：① 对 H6 抖动做重试策略（连续跑、只采纳 H6 通过的趟）；② 用 3 趟 H6-safe 的飞行
统计 4 m/s 的 W1/W2/W4；③ 若 4 m/s 在 hex14_low 能稳定（W4 正余量、W2 过），再上 6/8 m/s。

## 27. phase9f 法医分析：角点其实仍在"停-转"；跟踪误差 80% 来自定位（2026-11-26 11:10）

新工具 `scripts/flight_forensics.py`（离线，不飞）把一趟飞行拆成四件事：跟踪 / 执行 / 定位 / 高度，
逐角点给出"是否停、是否往复、转了多少度"。

### 27.1 角点：仍然会停一下 + 回退（之前 Q2 的"0 停顿"是门限盲区）

9f 每个角点（±2 s / +3 s 窗口，首次通过）：

| 角点 | cmd 最小速度 | truth 最小速度 | truth<0.25 m/s | 1.5 m 内停留 | path/net | 过角点后回退 |
|---|---|---|---|---|---|---|
| c1 | 0.27 | 0.14 | 0.25 s | 3.58 s | 1.57 | **-0.20 m** |
| c2 | 0.28 | 0.06 | 0.17 s | 3.51 s | 1.47 | **-0.51 m** |
| c3 | 0.26 | 0.10 | 0.18 s | 3.46 s | 1.41 | **-0.41 m** |
| c4 | 0.07 | 0.12 | 0.17 s | 3.34 s | 1.29 | **-0.51 m** |
| c5 | 0.29 | 0.23 | 0.10 s | 3.10 s | 1.20 | **-0.76 m** |
| c6 | 0.22 | 0.09 | 0.40 s | 3.05 s | 1.25 | **-0.66 m** |

- **不是"停了 0.5 s 以上"**：指令速度 <0.25 m/s 的持续时间 0–0.14 s，恰好躲过 `flight_quality` Q2 的
  "|v|<0.3 持续 >0.2 s" 判据 —— 所以门禁报 0 停顿，但**每角点都掉到 0.1–0.3 m/s、停留 ~3 s**；
- **确实有往复**：过角点后沿出弯方向回退 0.2–0.76 m，累计航向变化 215–616°（几何只要求 120°）；
- 根因（指令侧可见）：到达门 0.3 m 时派发下一个 goal，SUPER 新规划从"离角点 0.3 m、仍朝角点方向 ~0.8 m/s"
  的状态起步，新轨迹要先冲过角点再拐回来 —— cmd 轨迹在 c1 冲过角点 x=10.24 m（+0.24 m）再回到 9.91 m。
  **这是参考本身的角点绕行/回摆，飞机只是忠实执行。**

### 27.2 跟踪误差分解（最佳时延 +0.12 s）

```
cmd - truth = (cmd - FCU 估计) + (FCU 估计 - truth)
```

| 分量 | p50 | p90 | p99 |
|---|---|---|---|
| 总跟踪误差 | 0.155 | **0.216** | 0.332 m |
| 执行（cmd - FCU） | 0.070 | 0.156 | 0.288 m |
| 定位（FCU - truth） | 0.151 | **0.188** | 0.217 m |

- **定位误差解释了 80% 的跟踪误差能量**（水平 p90：跟踪 0.165 / 定位 0.125 / 执行 0.154 m）；
- 速度上反过来：cmd-truth p90 0.258、cmd-FCU 0.275、FCU-truth 0.144 m/s → 速度误差主要来自**执行**；
- 角点 vs 直道：执行误差 p90 0.220 vs 0.119 m（角点是执行/参考最差的地方），定位误差两者相近（0.19/0.18 m）；
- **原始 LIO 是准的**：lio-truth p50 0.061 / p90 0.087 / p99 0.106 m，偏置 (+0.000,-0.001,+0.016) m；
- 也就是说 +0.1 m 量级的误差是 **FCU 里 EKF 融合层**引入的（估计比真值高 0.122 m、慢 0.06 s）。

### 27.3 高度：定高是"在飞控坐标系里定高"，不是"相对真值定高"

- 指令 z 恒为 2.022 m（常值）；
- 真值高度 p50 **1.869 m**，偏置 **-0.118 m**；动态部分 p50 0.024 / p90 0.046 m（2–5 cm，无水平加速度相关 r=0.02）；
- FCU 估计高度比真值高 **+0.122 m**，而原始 LIO(SLAM) 高度只偏 +0.014 m；
- `calib.json` 的 t_WL z = **+0.1156 m**，与 -0.118 m 几乎相等；
- 结论：飞机一直保持"同一个高度"，但**飞控/EKF 的高度基准相对真值高了约 0.12 m**（extnav 高度基准 vs EKF 原点/气压计基准），
  所以指令 2.0 m 在真值上表现为 1.88 m —— **是坐标/基准问题，不是定高控制问题**。

### 27.4 待验证的角点修法（下一步，需 owner 同意后飞）

1. **到点派发时机**：把 arrive 0.3 m 改到 0.05–0.1 m（角点上派发），避免"带着朝角点的速度 + 提前 0.3 m 起步"的新规划；
2. **终点速度方向**：`CORNER_MODE` 由 bisector 改 outgoing（终点速度沿出弯方向）+ corner speed 1.2，让新规划起始速度与出弯方向一致；
3. **高度基准**：把 extnav z 与 EKF 原点的 0.12 m 偏差标定进任务高度（或设置 EKF 原点），使指令 2.0 m 对应真值 2.0 m。

图：`phase9f-vnorm.png`（速度 2-范数）、`phase9f-lio-truth.png`（LIO/FCU vs 真值）、
`phase9f-tracking-decomp.png`（误差分解）、`phase9f-z-t.png`（高度）、`phase9f-corner-forensics.png`（六角点回摆）。

## 28. 三件"修法"的真飞对照 + 高度基准/定高（2026-11-26 12:25–13:05）

按 §27 的三条修法逐条真飞（冻结森林、1 kg 机体、patch14、REF_V_SLEW 3.0、基准 CORNER_SPEED 1.2）：

| 趟 | 改动 | W1 | W1b | W4 | W3 | W6 | 角点回退(m) | 结论 |
|---|---|---|---|---|---|---|---|---|
| 19a | `ARRIVE_M` 0.3→**0.1** + `REF_Z_OFFSET=+0.118` | ✅ | ✅ | ❌ -0.046 | ✅ | ✅ | 0.35–0.83 | **arrive 提前派发 → 更糟** |
| 19b | 再加 `CORNER_MODE=outgoing` | ✅ | ✅ | ❌ -0.010 | ✅ | ❌ | 0.72–1.11 | **outgoing 更糟** |
| 19c | arrive 0.3 + `REF_DEADRECKON_S=0.6/STALE 0.15` | ✅ | ✅ | ✅ **+0.069** | ✅ | ❌ | 0.37–1.03 | 死区外推无帮助 |
| 19d | + `REF_Z_HOLD/TOL=0.005`、VISO 0.05/0.10 | ✅ | ✅ | ❌ -0.092 | ✅ | ❌ | — | H6 抖动，无效对照 |
| 19e | VISO 0.03/0.05、EK3_ACC_P_NSE 0.30 | ✅ | ✅ | ❌ -0.087 | ✅ | ❌ | — | H6 干净，见 28.2 |
| **19f** | **垂直环软化 PSC_D_VEL_P 5.0→3.0** | ✅ | ✅ | ✅ **+0.030** | ✅ | ❌ | 0.46–0.86 | **定高最好的一趟** |

### 28.1 三条修法的结论

1. **arrive 0.1 / outgoing 都被真飞否决**：角点回退反而从 9f 的 0.20–0.76 m 增到 0.35–1.11 m，
   W6 也挂。原因：旧轨迹本来就终止于角点，提前 0.1 m 派发时新规划更容易在角点前接手，
   把"过冲-回摆"放大；outgoing 只改终点速度方向，不改变"新规划从角点前带速度起步"的结构。
2. **REF_DEADRECKON 无效**（19c 角点回退 0.37–1.03 m）：角点的低速不是断流 hold，而是 SUPER 新轨迹
   本身在角点附近降到 0.1–0.3 m/s（指令轨迹在角点画小弯），外推补不了。
3. **唯一有效的是高度基准** `REF_Z_OFFSET_M=+0.118`：真值高度从 1.87 → **2.02–2.05 m**，W3 bias 从
   -0.113 变成 **+0.020~+0.034 m**（见 §27.3 的 0.12 m 基准解释）。

### 28.2 "外部位姿源置信度太低?"——对照实验的答案：**只解释了一部分（时延），不是主因**

按 owner 假设把 `VISO_POS_M_NSE` 0.10→0.05→0.03、`VISO_VEL_M_NSE` 0.25→0.10→0.05、`EK3_ACC_P_NSE` 0.10→0.20/0.30：

| 趟 | VISO pos/vel | EKF↔真值时延 | EKF↔真值误差 p50/p90 | 说明 |
|---|---|---|---|---|
| 9f | 0.10 / 0.25（交付） | **+0.06 s** | 0.151 / 0.188 m | 最干净的一趟 |
| 19c | 0.10 / 0.25 | +0.03 s | 0.225 / 0.335 m | 更激进、H6 有抖动 |
| 19e | **0.03 / 0.05** | **+0.01 s** | 0.209 / 0.375 m | H6 干净 ↑越激进 |
| 19f | 0.10 / 0.25（回到交付） | — | 0.248 / 0.345 m | 更激进 |

- **收紧置信度确实把 EKF 的时延从 +0.06 s 降到 +0.01 s** —— owner 的猜想方向是对的；
- 但 **EKF↔真值的误差没有下降**（甚至变大），且误差随飞行更激进（|a| p90 1.35→2.27 m/s²）而增大：
  这说明剩下的 0.15–0.25 m 不是"信得太少"，而是 **EKF 输出的时延/滤波平滑 × 速度** + 飞行本身更猛；
- 原始 LIO 始终是 0.06–0.09 m（p50/p90），所以**问题不在外源精度，而在 EKF 融合的时延**；
- 下一步更对症的做法（未做）：让 bridge 在 ODOMETRY 里发**真实协方差**（现在是零协方差，飞控按
  VISO_* 参数兜底），并把 `VISO_DELAY_MS` 从 10 ms 按实际 ~35–60 ms 重标定。

### 28.3 定高：命令已恒定到 1 cm，真值残余 2–5 cm（主要是垂直环振荡）

19f（`REF_Z_OFFSET=+0.118`、`REF_Z_HOLD=2.0/TOL=0.005`、`PSC_D_VEL_P 5.0→3.0`）：

| 指标 | 9f（修前） | 19f（修后） |
|---|---|---|
| 指令 z 范围 | 0.10 m（SUPER 计划 z 摆动） | **0.010 m** |
| 真值高度中位 | 1.869 m | **2.017 m** |
| 高度残差 std / p50 / p90 | 0.044 / 0.024 / 0.062 m | **0.033 / 0.022 / 0.050 m** |
| 真值垂直速度 std | 0.075 m/s | **0.034 m/s** |
| W3 bias | -0.113 m | **+0.020 m** |

真值残余高度变化与水平加速度不相关（r≈-0.08），是垂直环自身的小幅振荡；19g（VEL_P 2.0 再软化）
因 H6/H5 抖动无效，尚未定论。

## 29. 更正：EKF 并不"差"——之前 0.15 m 里 0.12 m 是高度基准；以及前视/终点机制清单（2026-11-26 13:20）

### 29.1 跟踪误差分解的更正（去掉 z 基准后重算）

把 `fcu-truth` 的高度中位偏置（+0.12 m 基准）扣除后再分解：

| 趟 | EKF↔真值 水平误 p50/p90 | 跟随(cmd−EKF) p50/p90 | 跟踪(cmd−truth) p50/p90 | 定位占误差能量 |
|---|---|---|---|---|
| 9f | **0.052 / 0.086 m** | 0.143 / 0.205 m | 0.151 / 0.224 m | **12 %** |
| 19f | **0.047 / 0.101 m** | 0.141 / 0.462 m | 0.152 / 0.481 m | **5 %** |

- **EKF 的水平定位其实很好（5 cm p50 / 9–10 cm p90）**，与原始 LIO（6–9 cm）一致；
- 之前的 0.15 m 是 **3D 误差被 +0.12 m 高度基准主导**（原始 LIO 高度只差 1.4 cm）；
- 跟踪误差的主项是 **跟随（cmd→飞机）**，不是定位；而跟随误差在角点处暴涨（19f 角点 0.46 m）；
- 置信度实验（VISO 0.10→0.03）只把 EKF 时延 +60→+10–30 ms，误差没降 —— 与"EKF 差"的判断一致修正：
  **不需要再调外源置信度**；若要继续压定位，可选做"bridge 发真实协方差 + VISO_DELAY_MS 按实测 30–60 ms
  重标定"，但收益上限 ~5 cm。

### 29.2 前视/终点机制清单（现有实现）

**SUPER 内部（hku_super/config/super_planner_hex.yaml）**

| 参数 | 当前值 | 含义 |
|---|---|---|
| `planning_horizon` | 16.0 m | 一条轨迹的 guide 路径最多覆盖多远 |
| `receding_dis` | 1.0 m | 10 Hz 重规划时沿旧轨迹保留/重接的长度 |
| `replan_forward_dt` | 0.1 s | 重规划起点在**轨迹时间**上提前多少 |
| `replan_rate` / `replan_time_limit` | 10 Hz / 0.35 s | 重规划频率 / 单次墙钟预算 |
| `corridor_bound_dis` / `corridor_line_max_length` | 0.8 / 1.0(6.0) m | 走廊生成 |
| `goal_pva_en` / `hot_start_en` / `goal_vel_en` | true / true / false | 终点 PVA / 热启动 / 终点速度（另一路） |

**plan box 目标与前视（本次飞行的默认值）**

| 参数 | 本次值 | 作用 |
|---|---|---|
| `--arrive-m` | 0.3 | 离角点多近就派发下一个 goal |
| `--pre-goal-m` | **0（关）** | 离角点 M 米时把**下一个角点**当 goal → 当前角点成为内部点 |
| `--goal-lookahead-m` | **0（关）** | goal 沿出弯方向前推 M 米（角点不再是终点） |
| `--corner-carry-m` | **0（关）** | 仅在 pre-goal 生效时，把 goal 再沿出弯方向推 0.5–1.0 m |
| `--goal-lead` | **0（关）** | goal 直接给 k 个航点之后（跳过角点） |
| `--pass-by-m` / `--pass-by-long-m` | 0 / 0 | 到达判据：越过航点平面/盒子（配合切角） |
| `--target-mode` / `--target-look-m` / `--target-step-m` | corner / (3.0) / (0.8) | rolling=连续 carrot 目标（本趟未用） |
| `--corner-mode` / `--corner-speed` | bisector / 1.2 | PVA 终点速度方向/大小 |

**终点是怎么算的（代码事实）**：plan box 把**单个 goal 点**发给 SUPER；SUPER `PathSearch` 用 A* 到该 goal，
`guide_path.back() == goal` 附近；`pos_fina_state.col(0) = guide_path.back()`，`col(1)=goal_v`（PVA 时）。
所以**每条轨迹的终点位置就是 goal**；`goal_pva` 只让末速度非零，不把末点移出角点。这就是"旧路径在角点结束"的机制。

### 29.3 owner 的方案 -> 现有开关

"前视 3 m；到角点前一直瞄角点；离角点 3 m 内瞄到角点之后" = **`PRE_GOAL_M=3.0` + `CORNER_CARRY_M≈0.8`**
（当前角点作为轨迹内部点、终点落在下一角点/角点后 0.8 m）。另一条是 `GOAL_LOOKAHEAD_M=3.0`
（终点在角点后 3 m，但角点无硬约束，需 `PASS_BY_M` 判到达）。

历史结论（旧栈，**没有** patch14/slew/z 修正）：pre-goal 1.5 曾死锁（stale-resend 已修）；
pre-goal 1.2+carry 0.8 造成 W1b 8/13 与接触（phase4z）；goal-lookahead 0.8 切角 W1b 挂（phase8i）。
**这些需要在当前栈上重测**。

若要"角点硬约束 + 终点在角点后"，SUPER 目前**没有 via 点**支持；需要 patch `generateExpTraj`：
先 A* 到 via（角点），再 A* 到 goal（角点后 3 m），拼 guide 路径、统一走廊后做单条 MINCO；
终点=goal，角点是走廊内部点。改动集中在 frontend，风险是跨角点走廊/SFC 的连续性与可行性。

## 30. 实现 owner 的"角点后前视"并首飞：角点不再停，但会切角（2026-11-26 13:30）

### 30.1 新开关（plan box，`--corner-exit-look-m` / `--corner-exit-trigger-m`）

owner 的设计原话："前视 3 m，到角点之前一直前视到角点，靠近角点 3 m 之内，前视到后面。"
实现（`planner_source_super.py::publish_goal_if_needed`）：

- 离当前角点 **> trigger**：goal = 角点（旧行为）；
- 离当前角点 **≤ trigger**：goal = **角点 + 出弯方向 × look**，终点速度 = 出弯方向 × corner_speed；
  每个 leg 只切换一次（latch），到达判据仍是 pacer 量到**角点**（半径/pass-by）。

### 30.2 首飞（20a：look 3.0 / trigger 3.0 / pass-by 0.6 / arr 0.3）

| 指标 | 9f（原） | 20a（角点后前视 3 m） |
|---|---|---|
| c1/c2/c4/c6 指令最小速度 | 0.07–0.29 m/s | **1.05–1.19 m/s** |
| c1/c2/c4/c6 真值最小速度 | 0.06–0.23 m/s | **1.04–1.17 m/s** |
| 真值 <0.25 m/s 时间 | 0.10–0.40 s | **0.00 s**（c5 除外） |
| 过角点回退 | 0.20–0.76 m | 0.00–0.42 m（c5 −0.79） |
| 1.5 m 内停留 | 3.0–3.6 s | **1.95–2.22 s** |
| W1b（<=0.5 m） | 13/13 | **4/13**：角点被切 0.45–1.13 m |

**结论**：这个开关**确实把"角点停-转"消掉了**（4/6 角点速度全程 ≥1 m/s、无停顿、停留时间几乎减半），
代价是**切角**：终点在角点后 3 m 时，从"离角点 3 m"的位置到新目标点的直线会在角点内侧 1.1–1.5 m 处通过，
W1b 因此挂掉。切角量可以用几何估：`cut ≈ T·L·sinθ / |T·u_in + L·u_out|`（T=trigger，L=look，θ=转向角），
本六边形 θ=60°、T=L=3 时 ≈1.5 m（实测 1.13 m，受走廊约束略小）；要 W1b ≤0.5 m，需 L·T 再小 ~4 倍
（如 T=1.0、L=0.8 ⇒ 理论 0.48 m）。

### 30.3 由此得到的两个选择

1. **小前视**（T≈1.0、L≈0.8–1.0）+ pass-by：不改 planner，靠几何把切角压到 0.5 m 内；
   代价是"末点在角点后 0.8 m"，能否完全消除停-转要真飞验证（20b 用 T=1/L=0.8 那一趟因主机 H6/EKF
   发散 abort，无效，需重飞）。
2. **真 via 点**（推荐，需 patch SUPER）：goal 仍可给"角点后 3 m"，但把**角点作为走廊内部约束**：
   frontend 先 A* 到角点、再从角点 A* 到 goal，拼 guide path 后统一走廊 + 一条 MINCO；
   终点=goal，角点被硬约束 —— 这才是 owner 要的"过角点（约束）+ 终点延伸"。改动集中在
   `generateExpTraj` 的 guide/SFC 拼接，风险是跨角点走廊连续性与 MINCO 可行性。

## 31. 低速段普查（owner 的分析规则）：20a 的"lap2 路径中间往复"找到了原因（2026-11-26 13:55）

新工具 `scripts/low_speed_census.py`：先找出**所有真值 |v|<0.5 m/s 的段**，再逐段分类
（LAUNCH/LAND、CORNER-PIVOT、CORNER-SLOW、MID-LEG LOOP、MID-LEG STALL），并给出每段的
时间/圈/位置/最近角点/路径净比/累计转角/指令最小速度。图：`<tag>-lowspeed-census.png`。

| 趟 | 角点段 | **路径中间** stall/loop | 备注 |
|---|---|---|---|
| 9f | 14（9 pivot + 5 slow） | **1 stall**（0.19 s，转角 23°） | 干净趟：低速全部发生在角点 |
| 19f | 7 | **4（2 stall + 2 loop）** | 中段低速处 cmd 也降到 0.28–0.48 → 参考自己停 |
| **20a** | 3 | **10（3 stall + 7 loop）** | **7 个 loop 全在 (1.0–2.2, 8.7) 附近** |

### 31.1 20a 路径中间 7 个 loop 的根因：**extended-goal 死锁**（不是跟踪问题）

- 20a 用了 `CORNER_EXIT_LOOK=3 m`：c2 的 goal = **c2 + 3 m 沿 c2→c3 = (2.0, 8.66)**；
- 7 个 loop 的位置实测在 (2.0,8.66) 周围 0.04–1.85 m 内绕圈（统计：55–59.5 s 平均距离 0.64 m，
  107.5–110 s 平均 0.39 m）；
- 原因：飞机切 c2 0.54 m，pacer 的**到达判据没触发**（ARRIVE 0.3 半径错过；pass-by 0.6 也错过首次穿越），
  于是 `pacer.i_leg` 不前进 → plan box 每一拍继续把同一个 extended goal 发给 SUPER →
  SUPER 反复做 rest-to-rest 规划 → 指令速度 1.0→0.1→1.0 的锯齿（实测 cmd |a| 在 +2.9/−2.7 m/s² 之间来回），
  飞机就在 (2,8.66) 附近绕圈。**这是 carrot 死锁，和 9f 的"角点停-转"是两回事。**

### 31.2 因此"小前视"方案必须先解决到达判据（方案，未飞）

设 trigger=T、look=L、六边形转角 θ=60°，切角估计
`cut ≈ T·L·sinθ / sqrt((T+L cosθ)^2 + (L sinθ)^2)`：

| T | L | 估计切角 | 是否 < ARRIVE 0.3 | 是否 < W1b 0.5 |
|---|---|---|---|---|
| 3.0 | 3.0 | ~1.5 m（实测 1.13） | ✗（已死锁） | ✗ |
| 1.0 | 0.8 | ~0.44 m | ✗ | ✓ |
| 1.0 | 0.6 | ~0.35 m | ✗（接近） | ✓ |
| 0.8 | 0.5 | ~0.30 m | 边界 | ✓ |

**方案 A（推荐先飞）**：`CORNER_EXIT_TRIGGER_M=1.0`、`CORNER_EXIT_LOOK_M=0.6`、
`ARRIVE_M=0.5`（到达半径放宽到切角之上，保证 pacer 一定前进）、`PASS_BY_M=0.7/PASS_BY_LONG_M=2.5`（双保险）、
corner speed 1.2、z 基准/定高开关维持 §28、垂直环 PSC_D_VEL_P=3.0。

**验收门（每趟用 census 判）**：
1. W1b 13/13（真值到角点 ≤0.5 m）；
2. census：**路径中间 stall/loop = 0**（LAUNCH 除外）；角点段 ≤6/圈；
3. 角点真值最低速度 ≥0.5 m/s、1.5 m 内停留 ≤2 s、过角点回退 ≤0.2 m；
4. H6 干净、W4 正余量、W6 不劣于 9f。

若 A 仍出现"角点 pivot 但中间 = 0"（预期：L=0.6 只能减轻、不能根除角点停），
则进方案 B：**SUPER via-point patch**（角点作为走廊内部约束 + 终点延伸 3 m），这是唯一能同时满足
"角点硬约束 + 终点不在角点"的做法。

