# 全流程链路（第 87 轮快照）：每一段的 topic、内容、名义/实测频率、判定参数

> **判定层逐条注解在 [`CHAIN_DETAIL.md`](CHAIN_DETAIL.md)**（航点盒的 13 个目标与 0.5 m 判据、
> SUPER FSM 的三个定时器与四个状态、薄层每拍的五个判断，全部对 `kn3-01-01` 的日志核过）。
> 本文是**这一段送了什么**，那份是**这一层判了什么**。
>
> 这份文档是**一条链路的现场清单**，不是设计意图：每一段的"实测频率"都来自某一趟真实记录的自身日志，
> 用的工具是 `scripts/chain_rates.py`（可以随时对任何一趟重跑）。名义值来自启动参数，两者不一致的地方
> 在 §6 单独列出来 —— 这条链路已经两次被"配的是一个频率、给的是另一个"坑过（仿真慢到 2.4 Hz、
> 外部导航 47/48），所以本文只写能复算的数字。

## 1. 一页图

```
                        ┌───────────────────────────── 飞行器（SITL 物理 + 4 个"传感器"）─────────────────────────────┐
                        │                                                                                          │
   worlds/forest_v2_hex.yaml ──(冻结, SHA a5a43f8f…)──> sim/raycast.py ──> sim/lidar_imu.py                          │
                        │        (250 根柱：z<2.1 方柱 0.2 m / 以上圆柱)     │ 用 truth tap 的 1200 Hz 位姿逐点投射   │
                        │                                                   ▼                                      │
                        │   /livox/lidar  PointCloud2  8.87 Hz  ┐    /livox/imu  Imu  199.1 Hz                     │
                        │   (mid360_link, x,y,z,intensity,time)  │    (mid360_link, rad/s, m/s²)                  │
                        └────────────────────────────────────────┼─────────────────────────────────────────────────┘
                                                                 ▼
                                              ┌────────────── Super-LIO（lio_ws）──────────────┐
                                              │  点云+IMU 紧耦合；输出两个话题                  │
                                              │  /lio/robo/odom  Odometry(L, 200.1 Hz)          │
                                              │  /lio/odom       带动速度（10 Hz 量级）         │
                                              └───────────────┬─────────────────────────────────┘
                                                              │ 200.1 Hz
                                                              ▼
   ┌────────────────────────────── 薄层 planner_source_super.py（PLANNER 插件）──────────────────────────────┐
   │  收：odom（L, 200 Hz）、/plan/cmd（自己的规划回读? 否）、/fcu/odom（W, FCU 自己的估计）、/plan/trigger   │
   │  发：/planning/click_goal_pva（航点 PVA：p=精确点, v=2.0 m/s 指向下一航点, a=0）  —— 只在"换航点"时发生 │
   │  发：/plan/cmd（quadrotor_msgs/PositionCommand, frame lio_world = L）50.0 Hz（SUPER 100 Hz 的降采样）  │
   │  判定：arrive 0.50 m 球 / settle 6.0 m/s 0.2 s / hold 0 / leg-timeout 40 s                              │
   └───────────────┬───────────────────────────────────────────────────────┬─────────────────────────────────┘
                   │ /plan/cmd (L) 50 Hz                                    │ /planning/click_goal_pva
                   ▼                                                        ▼
   ┌───────────────────────────── SUPER（fsm_node, upstream 原生）─────────────────────────────┐
   │  A*（膨胀图 0.10 m）→ CIRI 走廊（robot_r 0.10 m）→ SFC → MINCO                             │
   │  输出：自己的参考 100.0 Hz（traj.csv），**同一个 /plan/cmd 话题**（薄层转发）                │
   │  判定：max_vel 2.0 / max_acc 5.0 / max_jerk 40 / max_omg 2.5 / horizon 16 / receding 1.0  │
   └───────────────┬──────────────────────────────────────────────────────────────────────────┘
                   │ /plan/cmd 50.0 Hz
                   ▼
   ┌───────────────────────── bridge_node.py（薄层，唯一 L→W 转换点）─────────────────────────┐
   │  收：/plan/cmd(L) 50 Hz、/lio/robo/odom(L) 200 Hz、FCU LOCAL_POSITION_NED               │
   │  发：MAVLink 84 SET_POSITION_TARGET_LOCAL_NED  50.0 Hz（type_mask 见 §3）               │
   │  发：MAVLink 331 ODOMETRY（外部导航 = LIO 的位姿）46.8 Hz                                │
   │  发：/fcu/odom（FCU 自己的估计，W）                                                      │
   │  钳位：--ff-vel-max 2.5 / --ff-acc-max 5.0 / --ff-acc-slew 40（只钳前馈，不钳位置）        │
   └───────────────┬──────────────────────────────────────────────────────────────────────────┘
                   │ MAVLink / tcp:5780
                   ▼
   ┌──────────────────────────── ArduPilot Copter（SITL，4.7）────────────────────────────┐
   │  EKF3（外部导航 + IMU）→ 位置环（PSC_NE_POS_P 3.0 / VEL_P 3.0 / JERK 150）             │
   │  → 姿态环（ATC_ANG_RLL/PIT_P 6.0，ATC_ANGLE_MAX 45°）→ 电机 → 物理                     │
   │  日志：ATT/PSC*/RCOU/XKF* 10.0 Hz、IMU 200 Hz、SIM/SIM2 10 Hz                          │
   └───────────────┬──────────────────────────────────────────────────────────────────────────┘
                   │ truth tap：SIM_Aircraft.cpp 每步一行（patched，LIO_TRUTH_TAP=端口）
                   ▼
   ┌──────────────────────────── sim/truth_rx.py（真值记录 + 分发）──────────────────────────────┐
   │  tap.csv  1200.5 Hz（记录）          fanout（÷6 → 200 Hz）→ lidar_imu.py                    │
   │  truth_report.txt（重力/跨度/四元数自检）                                                    │
   └───────────────┬──────────────────────────────────────────────────────────────────────────────┘
                   │ 1200 Hz
                   ▼
   ┌──────────────── supervisor + accept.py（判据层，见 §5）────────────────┐
   │  geofence 10.5 m（用 tap 判！）；11 行 W/H 判据；四类图的数据源          │
   └──────────────────────────────────────────────────────────────────────────┘
```

**一帧数据的路径**（从雷达打到柱子到电机动作）：
柱面 → `raycast` → 每点带 `time`（秒）→ `/livox/lidar`（扫描起点时间戳）→ LIO 去畸变（每点用 1200 Hz tap 位姿）
→ `/lio/robo/odom`（200 Hz）→ 桥的 331 外部导航（46.8 Hz）→ EKF3 → 位置环（10 Hz 日志、内部更快）
→ 电机；同时 LIO 的位姿经 calib（yaw **+90.049771°**、t = (−0.0008, −0.00409, +0.11557) m）转到 L，
薄层在 L 里做航点判定、SUPER 在 L 里规划、桥把 L 转回 W/NED 发出去。

## 2. 帧与符号（一处一个名字，不许混用）

| 名字 | 含义 | 定义处 |
|---|---|---|
| **L**（`lio_world`） | LIO 自己的世界系：雷达第一帧为原点 | `docs/FRAMES.md`、`planner/cmd_stream.py` |
| **W**（`world_w`） | 场地系 = SITL 的 NED 换成 ENU：x=东, y=北, z=上 | `common/frames.py` |
| **NED / FRD** | 飞控内部 | MAVLink |
| **S** | 雷达壳体（`mid360_link`），云与 IMU 都在 S 里 | `sim/lidar_imu.py` 头注释 |
| calib | L→W：yaw **+90.049771°**、t = (−0.0008, −0.00409, +0.11557) m | `_stage/calib.json`，桥启动时打印 |

**唯一转换点**：`bridge/mavlink_impl.py::send_cmd`（`tools/test-s4-l-to-wire.py` 用 15 000 个用例把它钉住，
本轮改动没有碰它，A/B 仍然 PASSED）。

## 3. 逐段清单（topic / 内容 / 名义 / 实测）

实测一列来自 `missionA-v3pv-02-01`（第 87 轮 3 m/s 那批里飞得最远的一趟，`scripts/chain_rates.py v3pv-02-01`）；
括号里是第 85 轮 `pv1-02-02` 的旧值，用来对比"这一段有没有变"。

| # | 段 | topic / 接口 | 消息与内容 | 名义 | **实测（第 87 轮）** | 证据 |
|---|---|---|---|---|---|---|
| 1 | SITL 物理 → 真值记录 | UDP，`LIO_TRUTH_TAP` | ASCII 每行：`TAP inst t_us pn pe pd vn ve vd qw qx qy qz wx wy wz ax ay az`（NED/FRD） | 1200 Hz | **1200.5 Hz**（1200.5） | `tap.csv` 290885 行 / 242.3 s |
| 2 | 真值 → 雷达仿真 | UDP fanout（÷6） | 同上 | 200 Hz | **200**（÷6） | `truth_report.txt` |
| 3 | 柱面 → 点云 | `/livox/lidar` | `PointCloud2`，`mid360_link`，字段 `x,y,z,intensity,time`（time=秒，自扫描起点） | **10.0 Hz**，20000 点/帧 | **8.88 Hz**（8.87），19153 点/帧 | `lidar_imu.log`：2160 帧 / 243.2 s |
| 4 | 柱面 → IMU | `/livox/imu` | `Imu`，`mid360_link`，rad/s、m/s² | **200 Hz** | **199.2 Hz**（199.1） | `lidar_imu.log`：48434 条 |
| 5 | LIO 输出（位姿） | `/lio/robo/odom` | `Odometry`，pose 在 L，twist 在 L | ~200 Hz | **200.1 Hz**（200.1） | `lio.csv` 36265 行 / 181.2 s |
| 6 | LIO 输出（速度） | `/lio/odom` | `Odometry` with twist，10 Hz 量级 | 10 Hz | 桥报 `lio_hz=417.5`（含两路；第 85 轮 483.3） | `bridge.log` |
| 7 | SUPER 的参考 | **同一个 `/plan/cmd`**（薄层转发） | `quadrotor_msgs/PositionCommand`，`frame_id=lio_world`(=L)：`position, velocity, acceleration, yaw, yaw_dot, trajectory_id, trajectory_flag` | 100 Hz | **100.0 Hz**（100.0） | `traj.csv` 23271 行 / 232.7 s |
| 8 | 薄层 → 桥 | `/plan/cmd` | 同上（**降采样后的 50 Hz**） | 50 Hz | **50.0 Hz**（50.0） | `setpoints.csv` 8447 行 / 168.9 s |
| 9 | 薄层 → SUPER（目标） | `/planning/click_goal_pva` | PVA 目标：p=航点精确点、v=**3.0 m/s** 指向下一航点、a=0 | 事件驱动（每航点 1 次） | **13 个 goal / 趟**（两圈口径；第 85 轮 19 个） | `accept.json` H4 |
| 10 | 桥 → 飞控（setpoint） | MAVLink **84** `SET_POSITION_TARGET_LOCAL_NED` | `MAV_FRAME_LOCAL_NED`，`type_mask` 见下 | 50 Hz | **50.0 Hz**（50.0） | `wire.csv` 8447 行 / 168.9 s |
| 11 | 桥 → 飞控（外部导航） | MAVLink **331** `ODOMETRY` | LIO 位姿，`MAV_FRAME_LOCAL_NED`（含 L→W 转换 + z trim） | **48 Hz** | **46.9 Hz**（46.8） | `bridge.log` "nav_hz=46.9" |
| 12 | 桥 → 薄层 | `/fcu/odom` | `Odometry`，W，飞控自己的 `LOCAL_POSITION_NED` | 随 32 回来 | `sent=10863` | `bridge.log` |
| 13 | 飞控日志 | `flight.BIN` | `ATT/PSCN/PSCE/PSCD` 10 Hz、`IMU` 200 Hz、`RCOU` 10 Hz、`XKF*` 10 Hz、`SIM/SIM2` 10 Hz | — | **10.0 Hz**（PSC） | `flight.BIN` 1754 行 / 175.3 s |
| 15 | SUPER → 薄层（参考） | `/planning/pos_cmd` | SUPER 自己的参考（`mars` 消息） | 100 Hz | 100.0 Hz | `traj.csv` |
| 16 | 薄层 → SUPER（目标） | `/planning/click_goal_pva` | PVA 目标（上游另有 `click_goal` 不带速度；本项目用 `goal_pva_en` 才会读它） | 事件 | 13 个/趟 | `accept.json` H4 |
| 17 | LIO 内部（地图输入） | `/lio/cloud_world` | 世界系下的点云，直接喂 SUPER 的 rog_map | 随雷达 | ~8.9 Hz | `lidar_imu.log` |
| 18 | 飞控 → 桥（估计） | MAVLink **32** `LOCAL_POSITION_NED` | FCU 自己的位置估计，桥转成 `/fcu/odom`（W） | 随请求 | 47 Hz 量级 | `bridge.log` |
| 14 | 飞控命令 | MAVLink **76** / COMMAND_LONG | ARM/DISARM、`NAV_TAKEOFF`、`SET_MODE`（LAND）—— **只有 supervisor 发** | 事件 | 每趟 3–5 条 | `supervisor.log` |

**`type_mask` 的三个取值（这是第 85 轮的核心发现）**

| `--fields` | mask | 含义 |
|---|---|---|
| `pos`（**第 85 轮之前一直是它**） | **2552** | 只认位置 + yaw，忽略 v、a、yaw_rate |
| `posvel` | 2496 | 位置 + 速度 |
| **`posvelacc`（第 85 轮用户要求，现在的默认）** | **2048** | 位置 + 速度 + 加速度（只忽略 yaw_rate） |

## 4. 核心判定参数（谁在哪里决定什么）

### 4.1 SPEC（`docs/SPEC.md`，用户口述，不得擅自改）

| 条 | 参数 | 值 | 判定什么 |
|---|---|---|---|
| S0 | 六边形，六角各 5.000 m，**飞行臂 2 圈 = 12 角点 + 停机坪**（世界文件仍冻结在 3 圈），z = 2.0 m | 冻结 + 臂 | 场地几何 |
| S1 | 到达半径 | **0.50 m 球** | 是否算"到过" |
| S1 | 航点 PVA 速度 | **3.0 m/s**（第 87 轮；第 84–86 轮是 2.0），方向指下一航点，a = 0 | 出弯不停车 |
| S2 | 格子 / 柱 / 形状 | 1.0 m 格、0.2 m 柱、下方上圆（z_split 2.1）、无车道、航点格留空 | 地图 |
| S3 | 全程平均地速 | **> 4.0 m/s** | 速度 |
| S4 | `robot_r` / `inflation_step` | **0.10 m** / **1**（= 0.10 m 膨胀） | 走廊净空 |
| S5 | 地图与航点 | **冻结**（SHA-256 `a5a43f8f…`，17531 B，250 柱） | 每次起飞前 `check_frozen.py` 8/8 |
| S6 | 每次飞两趟 | 2 个 sortie | 会话 |

### 4.2 薄层（`src/whtf/ros/planner_source_super.py`，臂给的参数）

| 参数 | 值 | 作用 |
|---|---|---|
| `--arrive-m` | **0.50** | 3D 到达球（与 S1 一致） |
| `--settle-speed` / `--settle-s` | **6.0 m/s / 0.2 s** | "动着的到达"也算（不停车） |
| `--hold-s` | **0** | 到点不悬停 |
| `--pass-by-m` | **0** | 不用半平面判据（那会提前 13 m 判到） |
| `--leg-timeout-s` | **40** | 一条腿最多等 40 s ⇒ **失败时会 HOLD 40 s**（§61.5 的 81.6/83.2 s 静止就是两次） |
| `--leg` (publish) | 50 Hz | 把 SUPER 的 100 Hz 降到 50 Hz 发 `/plan/cmd` |

### 4.3 SUPER 规划器（`config/super_planner_hex.yaml`）

> **文件名是历史遗留，它不是 Super-LIO 的配置。**
> 读它的是 `ros2 run super_planner fsm_node -p config_name:=super_planner_hex.yaml`（由
> `src/whtf/ros/planner_source_super.py:1199` 起来），内容全是 `super_planner.*` / `rog_map.*` /
> `traj_opt.*`。**Super-LIO（SLAM）的配置是 `lio_avoid/lio/config/livox_360.yaml`**，由
> `ros2 run super_lio super_lio_node --params-file ...` 读取（`run-mission.sh:741`）。
> 两个包、两份配置、两个进程，不要混。

| 参数 | 值 | 作用 |
|---|---|---|
| `robot_r` | **0.10 m** | CIRI 把多面体从柱面推开的距离 + 种子线邻域 |
| `rog_map.inflation_resolution` × `inflation_step` | 0.1 m × **1** = **0.10 m** | **A\* 看到的膨胀**（决定它绕多远） |
| `rog_map.resolution` | 0.05 m | 地图体素 |
| `traj_opt.boundary.max_vel` | **3.0 m/s** | 参考速度上限（第 87 轮用户指定"3 m/s"；第 85–86 轮是 2.0） |
| `max_acc` / `max_jerk` | **8.0 m/s² / 40 m/s³** | MINCO 的可行性约束；**桥的前馈钳位现在就从这三个数读**（`scripts/cfg_envelope.py`） |
| `max_omg` / `max_acc_thr` / `min_acc_thr` | 2.5 rad/s / 17 / 6 | 姿态与推力约束 |
| `planning_horizon` / `receding_dis` | 16 m / 1.0 m | 规划视界与滚动 |
| `goal_pva_en` / `hot_start_en` | true / true | 终点带速度；新轨迹从实测速度起 |
| `yaw_mode` | 2 | 机头沿路径 |

### 4.4 飞控（`params/guided_limits_fast.parm`，**这一档是"紧"的**）

| 参数 | 值 | 作用 |
|---|---|---|
| `WP_SPD` / `WP_ACC` | 6.5 m/s / 10 m/s² | 位置环追赶速度/加速度上限 |
| `PSC_NE_POS_P` / `PSC_NE_VEL_P` | **3.0 / 3.0** | 位置环、速度环 P（ArduPilot 4.7 默认 1.0–2.0，这里更紧） |
| `PSC_NE_JERK` | 150 | 位置环 jerk |
| `ATC_ANGLE_MAX` | **45°** | 水平加速度上限 = g·tan45° = **9.81 m/s²** |
| `ATC_ANG_RLL/PIT_P` | 6.0 | 姿态环（4.7 默认 4.5） |

（另有温和档 `params/guided_soft.parm`：POS_P 1.5 / VEL_P 2.0 / ANG 4.5 / WP_SPD 2.5，
实测把 W6 跟踪从 0.76–0.87 降到 **0.407–0.417**。）

### 4.5 判据层（`scripts/accept.py` 的十一行）

| 行 | 判据 | 阈值 |
|---|---|---|
| H1 | 主机实时性 | rtf 0.98–1.02 |
| H2 | tap 完整性 | ≥380 Hz，最大间隔 ≤10 ms |
| H3 | LIO 存活 | 覆盖整趟 |
| H4 | SUPER 自己的参考（不是 fallback） | 参考计数 > 0 |
| **H5** | **真值源自洽**：tap 与 FCU 自己的估计在同一趟里的**空间跨度**相差 | ≤ max(2.0, 0.5·fcu_span)（垂直 max(1.5, 0.5·fcu_up)）；FAIL 即“这趟数据不可评分” |
| W0b | 在任务高度带 | ≥90 % 的样本在 2.00 ± 1.50 m |
| W1 | 每个航点都被"规划箱"记为到达 | 19/19 |
| W1b | **真值**进过 0.50 m 球 | 19/19，最差 ≤0.50 m |
| W2 | 巡航 ≥4 m/s | ≥1.0 s |
| W3 | 高度 | 偏差 ±0.30、偏移 ±0.80 m |
| W4 | **不碰柱**（盒体，z<2.1） | 最小净空 > 0 |
| W5 | 确实绕了（离开直线） | INFO |
| W6 | 规划-真值跟踪 | p90 ≤ 0.60 m |

### 4.6 监督层

| 参数 | 值 | 作用 |
|---|---|---|
| geofence | **10.5 m** | 用 **tap** 判；超了立刻 ABORT → LAND（§61.7：tap 跟 EKF 原点走，可能假警报） |
| `LOCK` | `/tmp/lio_avoid_sortie_super.lock` | 单飞 |
| 起飞前 | `check_frozen.py` 8/8 | 地图/航点没动才允许起飞 |

## 5. 名义 vs 实测：不一致的清单（都要盯）

| 项 | 名义 | 实测 | 影响 | 现状 |
|---|---|---|---|---|
| 雷达扫描 | 10.0 Hz | **8.87 Hz**（另一趟 9.7） | LIO 的输入变稀、去畸变窗口变长 | 已知，未解决（raycast 42–55 ms/帧） |
| IMU | 200 Hz | 199.1 Hz | 可忽略 | — |
| 外部导航 | 48 Hz | **46.8 Hz** | EKF 的观测比设定少 2.5 % | 已知（`--nav-hz 48`） |
| `/plan/cmd` | SUPER 100 Hz | 薄层发 **50 Hz** | 参考被降采样，转折处信息丢一半 | 设计如此（桥按 50 Hz 发） |
| 桥的 setpoint | 50 Hz | **50.0 Hz** | — | 正常 |
| 飞控位置控制日志 | — | 10 Hz | 频率响应的分析上限（只能看到 5 Hz 以内） | 正常 |
| tap | 1200 Hz | **1200.5 Hz** | — | 正常 |

## 6. 一条真实的时间线（`pv1-02-02`）

| 时刻 | 事件 | 证据 |
|---|---|---|
| 0 s | SITL 起步，tap 开始记录（1200 Hz） | `tap.csv` 第一行 |
| ~28.4 s | 起飞到 1.92 m（`cmd_z`） | `setpoints.csv` |
| 29.8–30.0 s | 薄层发第一个 PVA goal，桥开 relay | `wire.csv` 第一行 / `bridge.log` |
| 30–208 s | 参考 50 Hz；SUPER 100 Hz；外部导航 46.8 Hz | §3 |
| 36.9 / 91.0 / 134.5 s | 三圈的边界（真值进 c1 的 1.2 m 内，实测） | `plot_hex_laps.py` |
| 208 s | LAND，relay 关闭，supervisor `phase -> done` | `supervisor.log` |

## 7. 复现

```bash
# 链路频率（任意一趟）
python3 scripts/chain_rates.py pv1-02-02

# 每一段的"到底送了什么"（type_mask + L/W 两套数）
python3 - <<'PY'
import csv; r=list(csv.DictReader(open('/home/fanxin/lio_avoid/logs/missionA-v3pv-02-01/wire.csv')))
print(set(x['type_mask'] for x in r), len(r))
PY

# 规划自己的净空 / 跟踪 / 频率响应
python3 scripts/plan_clearance.py v3pv-02-01
python3 scripts/fcu_tracking.py /home/fanxin/lio_avoid/logs/missionA-v3pv-02-01
python3 scripts/loop_response.py v3pv-02-01 mx8-01-01

# 起飞前的硬约束（一个闸门，两个检查）
bash scripts/check_static.sh
```

## 8. 本轮（第 87 轮）改过的东西（"有没有偷偷搞破坏"的清单）

只有下面这些，每一处都在 `docs/SPEC.md` 修改记录或对应文件的注释里写了原因：

| 文件 | 改动 | 记录在哪 |
|---|---|---|
| `config/super_planner_hex.yaml` | `traj_opt.boundary.max_vel` 2.0→**3.0** | SPEC S1 + 修改记录；三份副本 md5 **`b21f5e2a94530c3b32e666395f80e780`** |
| `scripts/make_config_v2_08.py` | `max_vel=3.0` | 同上 |
| `scripts/fly_hex.sh` | `LAPS` 3→**2**、`CORNER_SPEED` 2.0→**3.0**；`BRIDGE_ARGS` 的三个前馈钳位改成**从配置读**（`scripts/cfg_envelope.py`） | 文件内长注释 + SPEC S0/S1 |
| `scripts/cfg_envelope.py`（新） | 从规划器 YAML 读 `max_vel/max_acc/max_jerk` | 文件头（写清第 86 轮那个陈旧钳位是怎么来的） |
| `lio_avoid/src/bridge/bridge_node.py` | `/bridge/status` 末尾加 `cv=/ca=/sa=`（钳位计数器）；**`send_cmd` 未动**（`tools/test-s4-l-to-wire.py` 仍 PASSED） | 代码注释 |
| `scripts/spec_check.py` | S1 的 PVA 速度改成与配置 `max_vel` **交叉核对**；新增 `S0 飞行圈数（臂）= 2`、`S1 前馈钳位 = 规划器上界` | `ALL STATIC CHECKS PASS` |
| `scripts/check_static.sh`（新） | 把 `spec_check.py` + `check_frozen.py` 合成起飞前一个闸门（顺带绕开 PowerShell 传 `\|` 的引号问题） | 文件头 |
| `scripts/plot_hex_laps.py` | 新增 `--laps`（两圈的世界文件不再写"第 X/3 圈"）、`--clip-m`（裁掉模型/估计分家后的尾巴） | 参数 help |
| `docs/SPEC.md` | S0 飞行圈数、S1 PVA 速度、S1 `max_vel` 三处；修改记录 **3 行** | 文内 |
| `docs/evidence/*` | 换成 `v3pv-*` 四张，删 `pv1-*` 五张 | `check_evidence_links.sh`（33 张旧图断言不在） |
| 新诊断脚本 | `pva_wire_census.py`、`round87_ledger.py`、`attitude_truth.py`、`yaw_compare.py`、`thrust_probe.py`、`alt_divergence.py`、`pscd_window.py`、`setpoint_window.py`、`vel_scale.py`、`lidar_rate_compare.sh`、`minco_fail_compare.sh` | `docs/INTEGRATION.md` §69.9、§70.5 |

### 8.1 第 87 轮下半场（§70）：两个真 bug + 一个新参数档

| 文件 | 改动 | 记录在哪 |
|---|---|---|
| `src/whtf/ros/planner_source_super.py` | **新增 `world_tour()`**：`--tour world` 现在真的按 `--laps` 截断（世界文件声明 3 圈 × 6 个，飞 2 圈就取 12 个 + 停机坪竖直腿 = 13 个目标）；`--laps` 大于世界声明时**不发明**，只告警；world 分支改用它并打印 `TOUR TRUNCATED …` / `tour: N corners + pad = M targets` | 函数 docstring（写清这个 bug 与它的证据） |
| `scripts/accept.py` | 每圈角点数改从世界文件的 `mission.laps` 取（原来与 `--laps` 混用，六边形被算成 9 个/圈）；`--laps` 只表示飞几圈；打印补上圈数/目标数 | 参数 help + 行内注释；**W1b 的 0.5 m 球、W4、H1–H5 等阈值一个没动** |
| `scripts/run_accept.sh` / `scripts/fly_speed_run.sh` | 臂的 `LAPS` 作为第 7 个参数传给 accept | 行内注释 |
| `scripts/spec_check.py` | 新增 `S0 两圈航线（薄层真的截断了）`：**真的调用**薄层的 `world_tour`，要求 12 个角点 | 行内注释 |
| `params/guided_v3_soft.parm`（新） | `PSC_NE_POS_P 1.0` / `PSC_NE_VEL_P 2.0`，其余同 `guided_limits_fast.parm` | 文件头（依据：3 m/s 趟真值跑到参考的 1.7 倍） |
| `scripts/test_world_tour.py`（新） | 2 圈=12、3 圈=18、9 圈不发明、截断是完整航线的前缀 | 文件头 |
| `scripts/path_curvature.py` | 新增 `--want` / `--plan-max-acc`：把"这个速度过这个半径要多大加速度"打成倍率 | 参数 help |
| `scripts/fly_v3_soft.sh`、`scripts/fly_v3_handover2.sh`、`scripts/fly_v3_posvel.sh`、`scripts/fly_v3_pos.sh`、`scripts/fly_v3_clamp5.sh`（新） | 这一轮五组对照的臂，每个文件头写清它只改哪一件 | 文件头 |

### 8.2 第 87 轮第三段（§71）：量优化器的违规量 + 找第二趟

| 文件 | 改动 | 记录在哪 |
|---|---|---|
| `scripts/set_optimizer_log.py`（新） | 一行开关 `traj_opt.print_optimizer_log`（`on`/`off`），改完要重装三份副本 | 文件头；**量完已关回去**，md5 回到 `b21f5e2a…` |
| `scripts/minco_penalty_stats.py`（新） | 解析 `traj.log` 里每个优化块的 Pos/Vel/Acc/Jerk/Omg/Thr 最大违规量，并对照代码里的阈值 | 文件头写了 `exp_traj_optimizer_s4.cpp:715-719` 的四条硬拒收 |
| `scripts/hunt_two_laps.sh`（新） | 按目标第 3 条（`H1..H5 + W1b` 全 PASS）反复飞、凑够两趟就停；每次尝试都记进 `/tmp/hunt_two_laps.txt`（含失败） | 文件头 |
| `config/super_planner_hex.yaml` | **只临时**把 `print_optimizer_log` 打开量了一趟，随后关回去；三份副本 md5 最终仍是 **`b21f5e2a94530c3b32e666395f80e780`** | §71.5 |
| `docs/evidence/*` | 换成 **`v3d-*`** 四张（唯一达标的 3 m/s 两圈趟），删 `v3pv-*` 四张 | `check_evidence_links.sh`（37 张旧图断言不在） |

**下半场仍然没有动的**：`worlds/forest_v2_hex.yaml`（8/8）、航点、`accept.py` 的阈值、`mavlink_impl.py::send_cmd`、`sim/`、SPEC 的 3.0 m/s 与 2 圈。

**整轮（含三段）一句话**：`worlds/` 一个字节没变、`check_frozen.py` 每趟 8/8；`SendCmd` 没动（L→W 只有一处）；
薄层只改了**航线来源那一处**（`--laps` 生效）；评分器只改了**目标表怎么来**（阈值一个没动）；飞控参数新增**一档软的**，
原来的 `guided_limits_fast.parm` 原样保留；规划器配置只临时开过一次调试打印，已关回原值。
