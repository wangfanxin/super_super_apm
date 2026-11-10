# 数据链路与状态机（以 `missionA-pe-01` 实测为准）

本文件只写**这一轮实际读过、测过**的东西。每条转移条件的出处都标了文件与行号；
凡是推断都会写「推断」。配套图：`pipeline-states.png`（已归档；重画 `scripts/plot_pipeline_states.py`）（时序）、
`docs/evidence/v3q-01-vxy-z-t.png`（v2 柱林 11/0 PASS 那趟的航点交接；v1 的 `sc-01-vxy-z-t.png` 已归档到
`_evidence_archive_round79.tar.gz`，`python3 scripts/plot_vz_t.py sc-01` 可重画）。

---

## 0. 一句话总览

```
航点盒（12 角点）→ SUPER fsm_node（原生 A*/CIRI/GCOPTER）→ 100 Hz 参考
   → 薄层（限速 + 三态兜底 + 航点步进）→ /plan/cmd
   → 桥（L→W→NED 一次换算）→ MAVLink 84 类 → ArduPilot GUIDED
                                                    ↑
      Super-LIO ← 仿真雷达/IMU ← 真值流 ← SITL ←────┘  （外测 331 类喂回飞控）
```

**四个进程各自有自己的状态机**，它们靠三根「握手线」串起来：
`/plan/trigger`（放行）、`trajectory_flag`（飞行/结束）、以及**参考是否在流**（薄层的兜底判据）。

---

## 1. 数据链路：谁产出什么、多快、在哪个系

| # | 进程 | 输入 | 输出（话题） | 实测速率 | 帧 |
|---|---|---|---|---|---|
| 1 | SITL `arducopter` | `--model x` 物理 | 真值 UDP tap（1200 Hz，`TAP` 行） | **1200.5 Hz** | NED |
| 2 | `truth_rx` | tap | `/sim/truth/odom`、`/clock`、`/sim/rtf`；UDP fanout 200 Hz；`tap.csv` | 1200 Hz 记录 | W |
| 3 | `lidar_imu`（仿真雷达） | fanout | `/livox/lidar`（20000 点/帧）、`/livox/imu` | **9.98 Hz / 199.7 Hz** | `mid360_link` |
| 4 | `super_lio_node` | 点云 + IMU | `/lio/robo/odom`、`/lio/odom`、`/lio/cloud_world` | **200.1 Hz** | L（`world`） |
| 5 | `fsm_node`（SUPER） | `/lio/robo/odom`、`/lio/cloud_world`、`/planning/click_goal` | `/planning/pos_cmd`、`/planning_cmd/poly_traj` | 10 Hz 重规划；参考 100 Hz | L |
| 6 | **薄层** `planner_source_super.py` | `/planning/pos_cmd`（QoS **BEST_EFFORT**）、`/lio/robo/odom` | **`/plan/cmd`** | 100 Hz 声明 / 50 Hz 上线 | L→**W**（用 calib） |
| 7 | 桥 `bridge_node` | `/plan/cmd`、`/lio/robo/odom`、`/plan/trigger` | 类 84 位置目标 + 类 331 外测，`/fcu/odom` | 50 Hz / ~100 Hz | **NED** |
| 8 | `supervisor` | 自己的 MAVLink 端点 + `/sim/truth/odom` | 解锁/起飞/模式/LAND（类 76）+ `/plan/trigger` | 20 Hz 泵 | — |
| 9 | 记录器 | 各路 | `tap.csv` `lio.csv` `setpoints.csv` `wire.csv` `traj.csv` `phase.csv` | — | — |

**三个"一次换算"的纪律**（前工程已经定死，我沿用）：
1. **L→W 只在桥里做一次**（`frames.l_to_wire`，含 `R_WL` 与 `t_WL`），薄层和桥必须用**同一份**
   `_stage/calib.json`，否则每个目标偏 54 mm（实测过的坑）；
2. **W→NED 的轴换序也在桥里**（`x_ned, y_ned, z_ned = p[1], p[0], -p[2]`）；
3. 姿态四元数必须 body-FLU→ENU 换成 body-FRD→NED，否则飞控以为机头朝东（历史事故）。

**实测到的录制时间基偏移**：`setpoints.csv` 的 `t_sim_us` 比 `tap.csv` 的 `t_us` **早约 0.85 s**
（`pc-02` −0.84、`pb-03` −0.86、四分位互差 ≤0.04 s，所以是常量而非滞后）。
验收里 W6 就是先证明这个偏移是常量、再要求残差；**任何跨表对齐都要先扣掉它**。

---

## 2. 状态机 A：SUPER 的 FSM（`fsm.cpp`，230 行，全文读过）

五个状态，**两个回调**驱动：主回调 `callMainFsmOnce()`，重规划回调 `callReplanOnce()`（`fsm/replan_rate` Hz）。

```
                 ┌──────────────── 首个 goal 到达（started_=true）──────────────┐
                 ↓                                                            │
   ┌────────┐  ┌───────────┐  new_goal   ┌────────────────┐ PlanFromRest      │
   │  INIT  │→ │ WAIT_GOAL │───────────→ │ GENERATE_TRAJ  │ SUCCESS/FINISH    │
   └────────┘  └───────────┘ ←────────── └────────────────┘ ──────────────┐   │
        (没 goal 就 return)   closeToGoal(0.1)   ↑  │  goalValid()==false  ↓   │
                              或 goal 无效      │  └──→ WAIT_GOAL   ┌──────────────┐
                                                │                   │ FOLLOW_TRAJ  │
                             ReplanOnce→NEW_TRAJ│                   │ 只发参考，   │
                                                └───────────────────│ 自己不退出   │
                             ReplanOnce→EMER                        └──────────────┘
                                    ↓                                      │
                              ┌───────────┐  下一拍                    重规划回调
                              │ EMER_STOP │──────────→ WAIT_GOAL      （下面 §2.2）
                              └───────────┘
```

### 2.1 主回调的逐条转移（`fsm.cpp:112-173`）

| 当前 | 条件 | 下一状态 | 副作用 |
|---|---|---|---|
| `INIT` | `!started_` | `INIT`（**return**） | 没收到过 goal 就永远不动 ← **这就是薄层必须每秒重发首个 goal 的原因** |
| `INIT` | 收到过 goal | `WAIT_GOAL` | 无条件 |
| `WAIT_GOAL` | `!gi_.new_goal` | `WAIT_GOAL`（return） | — |
| `WAIT_GOAL` | `gi_.new_goal` | `GENERATE_TRAJ` | 清可视化路径 |
| `GENERATE_TRAJ` | `closeToGoal(0.1)` | `WAIT_GOAL` | `new_goal=false`、`finish_plan=true` |
| `GENERATE_TRAJ` | `!goalValid()` | `WAIT_GOAL` | 打 `Goal is invalid, skip this goal` |
| `GENERATE_TRAJ` | `PlanFromRest` = `SUCCESS` | `FOLLOW_TRAJ` | `new_goal=false`、`plan_from_rest_=true`、`publishPolyTraj()` |
| `GENERATE_TRAJ` | `PlanFromRest` = `FINISH` | `FOLLOW_TRAJ` | 同上 + `finish_plan=true` |
| `GENERATE_TRAJ` | `PlanFromRest` = `FAILED` | **留在原地** | 打 `PlanFromRest failed, try replan`，下一拍再试 |
| `FOLLOW_TRAJ` | — | **不动** | 只 `publishCurPoseToPath()` |
| `EMER_STOP` | 下一拍 | `WAIT_GOAL` | — |

### 2.2 重规划回调（`fsm.cpp:45-86`）——**新 goal 在这里被消费**

前置门槛（任一成立就 return）：`stop` / 状态不是 `FOLLOW_TRAJ` / `finish_plan` / `plan_from_rest_`
（后者是一次性跳过，让刚算出的轨迹先跑一拍）。然后 `ReplanOnce(goal, new_goal)`：

| `ReplanOnce` 返回 | 动作 |
|---|---|
| `EMER` | → `EMER_STOP` |
| **`NEW_TRAJ`** | → `GENERATE_TRAJ`（**新航点就是这样触发一次全新的 `PlanFromRest`**） |
| `SUCCESS` / `FINISH` | `new_goal=false` + `publishPolyTraj()`（留在 `FOLLOW_TRAJ`） |
| `FAILED` | 什么都不做（静默） |

**这就是「航点无缝切换」的真实机制，也是最容易看错的地方**：
飞行中新航点**不是**由主 FSM 接的（`FOLLOW_TRAJ` 分支压根不看 `new_goal`），
而是由**重规划回调**接住 `new_goal` → 返回 `NEW_TRAJ` → 走一次 `GENERATE_TRAJ` → 新的 `PlanFromRest`。

**`PlanFromRest` 的含义**（`super_planner.cpp`）：它**每次都 `last_exp_traj_info_.setEmpty()` 并把
初始状态清零**（`pos_init_state.setZero(); col(0)=local_start_p_`）。所以**每条腿都是 rest→rest**：
起点速度为 0、终点速度为 0。这直接解释了 `pe-01` 的速度曲线（见 §5）。

### 2.3 goal 入口（`fsm.cpp:184-223`）

```
新 goal 到达（/planning/click_goal）
  → click_height > -5 ? 覆盖 z : 保留（我们把 click_height 设成 -10，所以保留）
  → getNearestInfCellNot(OCCUPIED, goal, ..., 3.0)   // 把落在障碍里的目标吸附到最近自由格
      失败 → "Goal is deeply occupied, skip this goal"（丢弃）
  → 与当前位置距离 < 0.1 m → 直接丢弃
  → 写 gi_.goal_p / goal_yaw，started_ = true，new_goal = true
```

---

## 3. 状态机 B：薄层的航点步进器（`cmd_stream.py` 的 `Pacer`）

它决定「什么时候算到一个航点、什么时候发下一个 goal」，是**任务层的状态机**。

```
 未武装 ──_arm() 三条之一──→ 武装 ──d≤gate 且 v≤settle──→ 到达(settled)
                              │                              │
                              │                              └→ 驻留(hold_for)
                              │                                    │
                              └──leg_timeout_s 超时──→ 跳点 ────────┘
                                                          ↓
                                                     _advance(): i_leg+1
                                                          │ 最后一条腿？
                                                          └→ done=True / TOUR COMPLETE
```

### 3.1 `_arm()`（`cmd_stream.py:_arm`）——三条之一即可

1. `p_w` 不是 None **且** 该腿有水平延伸 **且** 到目标水平距离 ≤ `arm_near_m`；
2. `p_w[2] > arm_above_m`（**0.50 m**）；
3. 相对 `p_first` 位移 > `arm_move_m`（**0.50 m**）。

> 这条门是为了不让「停在原地」被当成起飞成功；同时也解释了为什么竖直腿的目标必须是
> **停机坪正上方**：发「当前位置」会让它永不武装、永不换腿、永不再发 goal（死锁，实测过）。

### 3.2 到达判据（`cmd_stream.py:384-425`）

```
垂直腿：d = |p_w.z - target.z|          （高度误差）
水平腿：d = |p_w.xy - target.xy|        （水平误差）
gate  = arrive_m + (vertical_arrive_slack_m if 垂直腿 else 0)
```

| 条件 | 动作 |
|---|---|
| `d ≤ gate` **且** `v_h < settle_speed_mps` | `_arrive(capped=False)` → 立即算到达 |
| `d ≤ gate`（速度没降下来） | 连续满足 `settle_s` 秒后 `_arrive(capped=True)` |
| `d > gate` | `t_inside=None`；若 `now − t_armed > leg_timeout_s` → **TIMEOUT 并跳点** |

`pe-01` 里这些数取的是 `--arrive-m 0.25`、竖直腿另有 slack；`--hold-s -1` 表示用自动驻留。
**竖直腿判高度、水平腿判水平**这个例外不是优化，是被两次飞错逼出来的（`RECT31`/`RECT32` 的教训，
代码注释里留着完整记录）。

#### 3.2.1 到达判据的两个可调项，以及它们为什么决定速度（第 45 轮实测）

到达判据不只是"准不准"，它还决定**下一条腿什么时候被发出去**，因此决定整条链是"走走停停"
还是"连续飞"。两个旋钮：

| 旋钮 | 默认（验收档） | 作用 |
|---|---|---|
| `--arrive-m` | `2.50`（验收飞行的实际值，横幅里写着；代码默认 1.5） | 盒子半径。**越大越早交接**：交接时飞机还在动，规划不必先刹到 0 |
| `--settle-speed` / `--settle-s` | `1.5 m/s` / `0.5 s` | 到达还要求速度降下来。设成 `4.9 / 0.0` 就等于"进盒即到达" |
| `--pass-by-m` | `0.0` | >0 时改用"**已越过航点所在平面**且横向偏差 ≤ 该值"（切角比盒子宽也能判到）；0 = 老行为 |

实测差别（详见 `docs/INTEGRATION.md` §22）：把后两项打开、并让 SUPER 的 `PlanFromRest` 从实测速度
起步（`hot_start_en`），均速从 2.17 提到 2.49 m/s，但**4 次里 2 次撞柱**；停稳再交接 4 次全部清空。
所以验收档保留 `1.5 / 0.5`，动中交接那一档留在代码里但默认关闭。

### 3.3 `_advance()` 与 `done`

```
_advance():
   if i_leg >= n_leg-1:  done=True; log("TOUR COMPLETE: waypoint N held ...")
   else:                 i_leg += 1; armed=False; t_armed/t_hold/t_inside/p_start_w 清空
```
`done=True` 之后**指令流不停**，而是改发 `trajectory_flag = COMPLETED` 并保持悬停 ——
「不停说话，只是不再要下一个」。

---

## 4. 状态机 C：supervisor 的相位（`supervisor.py` + `phase.csv`，实测）

`pe-01` 的 `phase.csv` 逐行（`t_sim_us` 是仿真时间，同时记了 cmd/lio/truth 三路位置）：

```
preflight → takeoff → hold → leg1 … leg13 → land → done
   0.0        24.4      28.3   31.4      138.0   145.2
```

| 相位 | 进入条件（实测/代码） | 它做的事 |
|---|---|---|
| `preflight` | 端点有心跳 | 等 LIO、测停放标定、设 EKF 原点、进 GUIDED、解锁 |
| `takeoff` | 已解锁 + `landed_state=1` | 发 `MAV_CMD_NAV_TAKEOFF`，`packet.z` = 0.2 m（**逐字**，不加基准） |
| `hold` | 真值上升 > 阈值（**墙钟** 20 s 内） | 发 `/plan/trigger` 放行，等 3 s（A4 窗口） |
| `legN` | 计划盒子声明到达第 N 个航点 | 只记录，不干预 |
| `land` | `done` 或任一 abort | 切 LAND |
| `done` | 落地 + 上锁 | 收尾 |

**abort 是一等公民**，每条都带名字（这一轮实测触发过的）：
`geofence: truth at [...]` / `THE TAKE-OFF ALTITUDE DID NOT MATCH THE REQUEST` /
`never left the ground` / `the FCU left the mission mode (LAND)` / `no LIO odometry` /
`THE PLAN BOX IS NOT TALKING`。**验收看的正是「一条 abort 都没有」**。

---

## 5. 状态机 D/E：飞控与桥（为什么这条链能闭合）

**桥的 relay**：`/plan/trigger` 一到达 → 类 84 才开始上线路；飞控离开 GUIDED → 立刻关掉。
所以「起飞前一个 setpoint 都不上线」是设计，不是 bug。

**飞控 GUIDED 子模式**：`MAV_CMD_NAV_TAKEOFF` → `SubMode::TakeOff` + `auto_takeoff`。
`_AutoTakeoff::run()`（`takeoff.cpp:115+`）：

```
land_complete 时：只爬油门（TKOFF_SLEW_TIME 2.0 s），直到
                   油门 ≥ min(TKOFF_THR_MAX 0.9, 0.9) 或 竖直加速度 ≥ 50% 上限
                   或 竖直速度 ≥ 10% 上限  →  清 land_complete
之后：           pos_d_m = -(complete_alt_m + terrain_u_m)  → 竖直位置目标
完成判据：       到目标剩余距离 < 停止距离 且 爬升率 < 10% 上限
```
一旦 relay 打开、计划的 setpoints 上线，**类 84 的 z 就接管**，`auto_takeoff` 的目标不再是唯一的约束。

---

## 6. 一次航点切换的完整时序（`pe-01` 的位置表可对照）

```
 t=32.0  真值 (0.00, -0.03, 1.78)   竖直腿爬升中      [薄层发竖直腿目标]
 t=33.6  pacer: LEG2 武装 → 判高度 d=|z-2.0|≤gate → ARRIVED → 驻留 → _advance
 t=33.6  薄层发 GOAL 2/13 = W(-7.5,-7.5,2.0) → SUPER setGoalPosiAndYaw → new_goal=true
 t=33.6  SUPER 主回调仍在 FOLLOW_TRAJ（它不看 new_goal）
 t=33.7  SUPER 重规划回调接住 new_goal → ReplanOnce → NEW_TRAJ → GENERATE_TRAJ
         → PlanFromRest(起点=当前位姿, 初速=0) → SUCCESS → FOLLOW_TRAJ + 发新参考
 t=36.0  真值 (-3.33, -3.07, 2.45)  ← 斜插向角点 1
 t=40.4  真值 (-7.36, -7.24, 1.85)  ← 到角点 1；pacer 判到达（水平门）→ _advance → GOAL 3
 ... 每 ~35 s 一圈 ...
t=145.2  最后一条腿结束 → done=True → trajectory_flag=COMPLETED → LAND
```

**「无缝」在哪里**：切换那一刻薄层只做两件事 ——
(a) 参考源永远不断流（无参考时发竖直腿目标、断流时保持最后指令点、无位姿时发竖直腿目标），
(b) 对参考做**速率限幅**（`--ref-max-speed-mps` 6.0、竖直单独 1.2），
所以新轨迹的起点与旧指令之间的落差不以阶跃形式上线路。

**「不无缝」在哪里（必须承认）**：**每条腿都是 rest→rest**。
`PlanFromRest` 把初速度清零，所以每个角点都**减速到 0 再加速**。实测速度曲线是这个样子：

```
pe-01  |v_xy| : p50 1.07 m/s，p95 3.86，max 4.20，>4.0 m/s 累计 3.97 s
        周期 ≈ 5 s 的 0 ↔ 4.2 往复 —— 停走式，不是平滑巡航
高度        : p50 1.90 m（指令 2.0）、p95 2.56，但瞬时在 0.5 ↔ 2.9 之间摆
```

这不是控制环的错，是**规划语义**决定的：要真正平滑巡航，得让 SUPER 在切换时接受上一条轨迹的
末速度（把 `PlanFromRest` 换成「带上初速的重规划」，涉及 2–3 个文件），这不属于本轮范围。
验收 W2/W3 是按「达到过 4 m/s」「中位与 p95 在带内」判的，这两条它过了；但**"平滑巡航"没有做到**，
这一点在图上能直接看出来，不该被通过的字样盖住。

---

## 7. 避障是怎么被证明的

* 世界审计（生成器算的，不是许愿）：**4/4 条腿被直线阻断**；杆正好落在四条腿**正中**；
* 实测最小柱面距离 **+0.222 m**，4109 个采样里 **0 个**在机体内；
* 位置表能看到绕行幅度：上边腿绕到 **N=+10.0**（腿在 +7.5，即外扩 2.5 m），左边腿到 **E=−8.14**；
* 绕行不是"撞了再修"，因为 `pc-*`/`pe-01` 的 W4 都是 CLEAR。

## 8. 注意：那条"常数时间基"其实是参考滤波器的滞后（第 48 轮更正）

第 45 轮起，`accept.py` 的 W6 一直靠"四个四分位各自拟合一次时间偏移、互差 ≤0.15 s"来证明
`setpoints.csv` 与 `tap.csv` 之间的 −0.84…−0.88 s 是**录制时间基**而不是控制滞后。那个结论在它自己的
意义上是对的（偏移确实是常数），但**成因**在第 48 轮被更正了：

`missionA-rp-01` 把参考调速器的增益从 kp=6 提到 kp=20（沿航迹滞后 v/kp 从 0.75 m 压到 0.22 m）之后，
W6 拟合出的偏移**从 −0.85 s 缩到 −0.15 s**（四个四分位 −0.12 / −0.12 / −0.22 / −0.16，互差 0.10 s）。

所以那 0.85 s 里，约 0.7 s 是"参考被限幅/滤波之后以固定方式落后于规划"造成的**控制**滞后
（4 m/s × 0.7 s ≈ 2.8 m，沿航迹、不损失净空），只有剩下的约 0.15 s 是真正的录制时间基。

读数注意：**不要把 W6 的偏移当成纯录制量**。同一档配置换了参考处理方式，它会跟着变；
判断"是录制还是滞后"要看偏移在**同一配置内**是否跨四分位恒定，而不是跨配置比较。

---

## 9. 每一行判据读的是哪个文件（第 57 轮：两处"自证"漏洞）

验收的十一行不是同一等级的，区别在于**证据来自哪**。第 57 轮发现其中一行读的是**被考核者自己的
日志**，而它在一种配置下给过错误的通过，于是补了一行独立复核。这张表是现在的对应关系：

| 行 | 判据 | 读的文件 | 证据性质 |
|---|---|---|---|
| H1 | 仿真跟得上实时（rtf 0.98–1.02） | `tap.csv` | 独立 |
| H2 | 真值采样完整（1200 Hz，无空洞） | `tap.csv` | 独立 |
| H3 | LIO 交付频率、z 跨度 | `lio.log` + `tap.csv` | 独立 |
| H4 | SUPER 参考在动（`max SUPER ref count`） | 规划盒日志 | **自报**（只作诊断） |
| W0 | 路线时间窗、最大高度 | `tap.csv` | 独立 |
| W0b | 路线内高度带占比 ≥90 % | `tap.csv` | 独立 |
| W1 | 到达 13/13 | 规划盒日志（`ARRIVED at waypoint`） | **自报**（只作诊断） |
| **W1b** | **真值到 13 个目标的最近距离全部 ≤1.5 m** | `tap.csv` + 世界几何 | **独立（闸门看它）** |
| W2 | 速度 >4 m/s 的累计时长 ≥1.0 s | `tap.csv` | 独立 |
| W3 | 高度偏差 / 瞬时超调 | `tap.csv` + `setpoints.csv` | 独立 |
| W4 | 逐柱最小间隙 > 0 | `tap.csv` + `worlds/*.yaml` | 独立 |
| W5 | 相对直线折线的偏离（信息行） | `tap.csv` | 独立 |
| W6 | 规划 vs 真值残差 p90 ≤0.60（按四分位拟合时间基） | `setpoints.csv` + `tap.csv` | 独立 |

两处漏洞都出在"拿自述当证据"：

1. **`W1` 会误判"到过"。** `--pass-by-m` 的到达判据是"越过航点所在平面 + 横向偏差 ≤1.5 m"。
   当目标被推到角点**之后**、轨迹又**抄近路**时，它会从角点旁 3–5 m 处满足这个条件。
   实测 7 次误判（`sk-01` 自报 13/13、真值 4/13）。补的 `W1b` 只读真值。
2. **"有效飞行"的过滤也用过 `W1`。** 前沿图（`speed_frontier.py`）与主机混淆检验
   （`w6_vs_host.py`）原来都用自报的 13/13 当有效性门槛，于是抄近路的飞行被当成数据点。
   改成 `W1b` 后有效条数 48 → 46（**剔除 4 条，又补回 2 条被 pacer 少报的** —— 自报的误差是双向的）。

读数注意：**"自报"不等于"错"，但它不能当闸门。** `W1`/`H4` 现在是诊断列；任何结论如果只能靠它们
支撑，就必须先用 `scripts/verify_waypoints.py` 或 `scripts/w1_audit.py` 复核一遍。

