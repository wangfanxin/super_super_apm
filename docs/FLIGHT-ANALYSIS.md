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
| phase4x | 4w + 参考断流时重发 goal + 诊断打印 | **12/13 ARRIVED，环飞完成**；W6 True；但 W1b 仍 False（到点顺序/判定口径与 pre-goal 冲突），W1 12/13 |
| phase4y | `GOAL_LOOKAHEAD_M=1.5` + CORNER_SPEED=1.2 | **失败/坠地**：76.9 s 时 `SIM Hit ground at 2.05 m/s`，W2/W6 False → goal-lookahead 当前不可用 |

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
