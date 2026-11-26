# PIPELINE — 从 SITL 到验收的完整链路

本文是**当前**链路的唯一描述（旧的 CHAIN*/DATA_PIPELINE/EXTERNAL-NAV/MISSION/SPEC/PHASE* 文档已合并进来，
过期轮次的记录合并到 `docs/FLIGHT-ANALYSIS.md` 附录）。所有路径相对仓库根。

## 1. 组件与进程

| 组件 | 位置 | 职责 |
|---|---|---|
| SITL | `.build/ardupilot/build/sitl/bin/arducopter` | ArduCopter 4.7.1，GUIDED 模式；`--model x:<json>` 可换机体 |
| 真值客户端 | `sim/src/sim_state_recv.py` | 自己的 MAVLink `SIM_STATE(108)` 客户端 → `/sim/state` 200 Hz + tap |
| 雷达/IMU 仿真 | `sim/src/lidar_imu.py` + `sim/raycast.py` | MID-360 扫描模式、**逐点位姿**（真值插值）；`/livox/lidar` 10 Hz、`/livox/imu` 200 Hz |
| Super-LIO | `slam/super_lio` | LiDAR-inertial 里程计 → `/lio/robo/odom`（200 Hz，L 系） |
| SUPER planner | `hku_super/super_planner` | 前端 A*+走廊 + MINCO 多项式轨迹；`fsm_node`，10 Hz 重规划 |
| plan box | `bridge_nodes/thin/planner_source_super.py` | 航点状态机（pacer）、目标派发、参考整形、起停 `fsm_node`、发布 `/plan/cmd` |
| FCU 客户端 | `bridge_nodes/mavlink/fcu_client.py`（merged：bridge + supervisor） | ODOMETRY #331、消息 84 PVA、supervisor 守卫、日志 |
| 验收/出图 | `scripts/accept.py`、`scripts/plot_*`、`scripts/flight_*` | 15 行验收 + 全套图 + 法医/census |

启动顺序由 `scripts/run-mission.sh` 负责：清理 → SITL → truth tap → lidar_imu → Super-LIO → plan box
（含 fsm_node）→ fcu_client → 等 LIO → 起飞 → `/plan/trigger` 放行参考 → 监控 → LAND → 落盘。

## 2. 坐标系与标定

- **W**：世界系（x=东、y=北、z=高度），真值 tap、命令行 `cmd_*`、`fcu_*` 都是 W。
- **L**：LIO 世界系（Super-LIO 自己的世界）；`/plan/cmd`、SUPER 目标、`lio_*` 都是 L。
- **wire**：MAVLink NED；1 个 `yaw` 换算只在 `common/frames.l_to_wire_yaw` 做一次。
- 标定：`calib/calib.json` 的 `R_WL`、`t_WL`（`p_W = R_WL p_L + t_WL`），`t_WL[2] ≈ 0.1156 m`。
- **高度基准**：EKF 高度基准比真值高约 **0.122 m**（原始 LIO 高度只差 1.4 cm），
  所以 plan box 用 `REF_Z_OFFSET_M=0.118` 补偿；这是 9f+ 配置的一部分（旧 9f 没补偿，真值只有 1.87 m）。
- 时间：sim 时钟 `t_sim_us` 为唯一参考；tap 与各日志都带 `t_sim_us`，`truth_timebase.json` 记录映射。

## 3. 控制链路（plan box → FCU）

```
plan box 100 Hz → /plan/cmd（quadrotor_msgs/PositionCommand，L，PVA） 
   → bridge：L→W（同一份 calib）→ MAVLink #84（50 Hz，SET_POSITION_TARGET_LOCAL_NED PVA）
   → ArduPilot GUIDED：pos+vel+acc 直接进 AC_PosControl（PVA 模式，不经过 WP_* 整形）
                 修正项受 WP_ACC/WP_JERK 限；PVA 加速度上限由 PSC_NE_JERK 控制
```

- plan box 的参考整形（9f/9f+ 相关）：
  - `--ref-v-slew`（`REF_V_SLEW`）：对**指令速度**做 slew 限幅（位置/路径不变）——1.35 干净的關鍵；
  - `--ref-z-offset-m`（`REF_Z_OFFSET_M`）：补偿 EKF 高度基准；
  - `--ref-z-hold/--ref-z-tol/--ref-z-latch-m`：把指令 z 锁成常数（`2.0 ± 0.005`）；
  - `--stale-s` / `--stale-fail-s`：SUPER 断流多久转 hold / 多久退出；
  - `--clr-gov-margin`（净空调速，实验开关）、`--ref-gov-*`（弧长/投影调速，实验开关）默认关。
- 桥的硬门限：
  - `--nav-gate-mps/--nav-gate-dps`：外导航"物理可达"校验（默认 5 m/s；速度阶梯时需同步提高，
    否则 8 m/s 的合法帧会被拒 → supervisor 误报 `ESTIMATOR IS DIVERGING`）；
  - `--nav-hz`（默认 48）、`--nav-max-age-s`、`--nav-min-quality`；
  - `--wire-hz`（默认 50）、`--setpoint-log/--wire-log/--nav-log`。
- supervisor 守卫：geofence、高度上下限、规划静默 3 s、nav 频率 floor（默认 30 Hz）、
  nav 拒绝率（`--nav-skip-max-per-s`，默认 10/s）、EKF 发散（桥外导航拒绝）、真值/FCU 位置一致性、
  LIO 断流（`--nav-floor-hz` 与 `nav_age`）。

## 4. 估计链路

```
lidar_imu（10 Hz 点云 + 200 Hz IMU）
   → Super-LIO → /lio/robo/odom（200 Hz，L）
   → bridge：ODOMETRY #331（48 Hz；零协方差，飞控按 VISO_* 参数兜底：POS 0.10 m / VEL 0.25 m/s / YAW 0.10 rad）
   → EKF3：EK3_SRC1/2/3 = 6（ExternalNav 全集），VISO_TYPE=1，VISO_DELAY_MS=10
```

实测（phase9f，扣除 0.12 m 高度基准后）：

- 原始 LIO vs 真值：p50 **0.061 m** / p90 0.087 m / p99 0.106 m，偏置 (0.000, -0.001, +0.014)；
- FCU EKF 水平误差：p50 **0.052 m** / p90 **0.086 m**；高度动态残余 std 2–3 cm；
- EKF 相对真值时延 ~45–65 ms（最佳时延修正只解释约 10% 误差）；
- 结论：**定位不是跟踪误差的主因**（去基准后占 5–12%），主因是参考跟随（尤其角点）。

## 5. 规划链路（SUPER）

- 入口：plan box 发**单个 goal 点**（PoseStamped 或 MarsMsg PVA 到 `<goal-topic>_pva`）；
- SUPER：A*（`PathSearch`）→ 走廊（`SearchPolytopeOnPath`）→ 引导路径 + 时间戳 → MINCO 优化
  → 提交多项式轨迹，10 Hz 重规划（`ReplanOnce`）。
- 终点定义：`pos_fina_state.col(0) = guide_path.back()`（≈ goal）；`goal_pva_en` 时
  `col(1)=goal_v`、`col(2)=goal_a`（终点速度/加速度）——**终点位置就是 goal**，
  所以"角点作为 goal"必然让轨迹在角点结束。
- 关键配置（`hku_super/config/super_planner_hex.yaml`）：
  `max_vel 1.35 / max_acc 6 / max_jerk 15 / max_omg 2.5 / planning_horizon 16 m / receding_dis 1.0 m /
  replan_rate 10 Hz / replan_forward_dt 0.1 s / replan_time_limit 0.35 s /
  corridor_bound_dis 0.8 / corridor_line_max_length 1.0(6.0) / map_size 24 / inflation_step 2 /
  goal_pva_en true / hot_start_en true / goal_vel_en false`。
- **已应用的补丁**（`hku_super/patches/`，`scripts/check_static.sh` 会检查关键几个）：
  | 补丁 | 内容 |
  |---|---|
  | patch 9 | 重规划时保持发布已提交轨迹（不静默） |
  | patch 10 | `super_planner/replan_time_limit` 墙钟预算（0.35 s） |
  | patch 11 | 拒绝"第一步就反向"的规划 |
  | patch 14 `patch_super_smooth_handover.py` | 新目标先走**连续重规划**（旧轨迹做 guide），失败才回退 `new_goal=true` → 显著减少角点停-走 |
  | patch 15 `patch_super_pva_tail.py` | PVA goal 后追加 `pva_tail_len` 米直线延长段（默认 0=关），让轨迹终点不在角点；**实验开关，尚未定论** |
- plan box 的目标/到达参数（当前默认 / 9f+ 值）：
  `--target-mode corner`、`--arrive-m`（默认 1.0 / 9f+ 0.3）、`--pre-goal-m 0`、`--goal-lookahead-m 0`、
  `--corner-carry-m 0`、`--goal-lead 0`、`--pass-by-m 0`、`--corner-mode bisector`、`--corner-speed 0.8`、
  `--goal-pva 1`。
  （`CORNER_EXIT_LOOK/TRIGGER` 与 `REF_DEADRECKON` 已在清理中删除——它们被真飞否决。）

## 6. 世界与任务

- 冻结地图：`sim/worlds/forest_hex12.yaml`（`check_frozen.py` 校验 SHA/柱数/角点/边长/场地墙）：
  六边形半径 10 m、相邻角点 10 m、3 圈、522 柱、柱距 1.0 m、柱 0.2 m 见方（z<2.1 m）、waypoint 3×3 空。
- 任务：从停机坪起飞到 2.0 m，依次经过 6 角点 × 3 圈（pad 也是第 1 个目标），最后回到 pad 上方。
- 大世界（速度阶梯备用）：`forest_hex12w.yaml`（970 柱）、`forest_hex16.yaml`（1546 柱）、
  `forest_hex14_low.yaml` / `forest_hex18_low.yaml`（柱高 1.2 m、飞行高度上方净空）。
- 这些世界**不是** SPEC 冻结图；用于速度/能力诊断，验收仍以冻结图为准。

## 6b. 泛化测试：同一规则、不同 seed 的地图套件（owner 2026-11-26）

冻结 SPEC 地图仍是 `sim/worlds/forest_hex12.yaml`（`check_frozen.py` 永远校验它，不参与 seed 变化）。
为了验证**泛化性**，用同一个生成器、同一几何规则、只换 seed 生成诊断地图：

```bash
python3 scripts/make_world_v2_hex.py --seed 20240522 --template sim/worlds/square15_forest.yaml \
    --arena-m 12.5 --out sim/worlds/forest_hex12_s22.yaml     # s23/s24 同理
WORLD_NAME=forest_hex12_s22.yaml <其余飞行环境> bash scripts/fly_hex2.sh gen22a 1
```

- 每张图仍为 522 柱、54 个 waypoint 空单元、六边形半径 10 m、柱距 1.0 m、飞行高度 2.0 m；
- `fly_hex2.sh` 会打印 "DIAGNOSTIC WORLD" 警告；`check_frozen.py` 仍只检查冻结图；
- 验收（`run_accept.sh`/`quality_rows.py`）使用 `WORLD_NAME` 指向的那张图；
- 评估协议：**同一套配置至少 3 个 seed 通过 15 行；调参只看最差 seed**，避免对一张图过拟合。

## 7. 数据与日志（每趟 run 目录）

```
.runs/missionA-<tag>-01-01/
  tap.csv        真值 200 Hz（pn/pe/pd；x=pe, y=pn, z=-pd）
  setpoints.csv  cmd_*（W）、fcu_*（飞控估计 W）、lio_*（LIO L）、vel/acc
  nav.csv        LIO 外导航样本（L）+ fcu_est（W）+ 拒绝原因/age
  lio.csv        Super-LIO 原始 odom 200 Hz（L）
  scans.csv      每帧雷达 cast 时间/点数/跳过原因（H6 依据）
  phase.csv      supervisor 阶段时间线
  accept.json    15 行验收结果
  traj.log       plan box + fsm_node 日志（GOAL/ARRIVED/LEG/planning 失败）
  fcu_client.log supervisor/bridge 日志（abort 原因）
  lidar_imu.log / lio.log / sitl.log / flight.BIN
  derived/       bag → CSV 派生（offline 校验用）
```

约定：
- `cmd_*`/`fcu_*` 是 W；`lio_*` 是 L（原始估计）；`accept.py` 不混用；
- 所有分析用 `t_sim_us`（sim 时钟）；`truth_timebase.json` 记录 tap 与墙钟映射；
- H6（雷达尽力而为）判据：cast p50 ≤100 ms、skipped ≤20；超限的趟**不可评分**（先 retry）。

## 8. 验收与质量门

- `scripts/accept.py`（15 行）：H1–H6（宿主/估计/链路健康）、W0–W6（任务质量，含 W1b 单调到点、W4 柱面净空、
  W2 巡航速度、W3 高度、W6 跟踪时延）。细则见 `docs/ACCEPTANCE.md`。
- `scripts/flight_quality.py`：Q1 绕圈、Q2 中段指令停顿、Q3 参考连续性、Q4 跟踪；提速前必须过。
- `scripts/flight_forensics.py`：跟踪 = 执行(cmd−EKF) + 定位(EKF−truth)，并给角点停/回摆、高度基准、LIO 误差。
- `scripts/low_speed_census.py`（owner 规则）：**先列出所有真值 |v|<0.5 m/s 的段，再逐段分类**：
  LAUNCH/LAND、CORNER-PIVOT/SLOW、MID-LEG LOOP/STALL。以后所有飞行分析以它为第一入口。
- `bash tests/run_all.sh`：全量门禁（语法/静态/模式/证据链接/时间基/nav gate/launch matrix/APE-RPE/
  criteria-v2/FCU bench/path policy/tool smoke）。

## 9. 运行约定与已知坑

1. **只飞 SITL，不飞真机**；不 push。
2. 飞行前 `bash scripts/clean_harness.sh`：清端口进程、LIO、fsm_node（SIGTERM 后可能不退出，需 `-9`）、
   `ros2 daemon`、`/dev/shm/fastrtps_*`（DDS 残留会让 LIO 收不到点云）。
3. SITL 自定义机体 JSON 只接受相对路径：`run-mission.sh` 会把模型 COPY 到 run 目录再加载。
4. 起飞是"设定点驱动"：起落架离地后 plan box 才发 `/plan/trigger`；竖直段由 plan box 自己拉高度。
5. 地图/航点冻结：`scripts/check_frozen.py` 不过就不起飞；换图必须 owner 批准并更新指纹。
6. 主机雷达仿真是尽力而为：H6 starvation 时整趟不可评分；同一配置时好时坏，必须先清理再飞。
7. `fsm_node` 的配置在 `hku_super/super_planner/config/`（`scripts/install_config_hex.sh` 负责同步）；
   修改 `hku_super/config/*.yaml` 后要拷贝到那里。
