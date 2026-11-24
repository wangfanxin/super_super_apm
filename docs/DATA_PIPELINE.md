# 数据管线（DATA PIPELINE）—— 一次飞行的数据流、产物与评分链

> 这份文档回答一个问题：**这个仓库里，数据从哪来、经过谁、变成什么、最后怎么被评分**。
> 它与 `docs/CHAIN.md`（第 87 轮的逐段快照）、`docs/PIPELINE.md`（更早的状态机笔记）的区别是：
> 那两份是历史记录，**这份是当前实现**，并且把"运行时的数据流"和"离线产物的数据流"画在一起。
> 数字来自三处：`scripts/run-mission.sh`（启动器）、`scripts/run_until_healthy.sh`（飞行臂）、
> `apm_thin/ros/planner_source_super.py`（薄层），以及最近一次实测记录 `smoke3-01-01` 与
> `kx6-03-01-01`（后者是 14 行全过的那趟）。

---

## 0. 两张图

### 0.1 运行时（一次飞行，进程与端口）

```
  sim/worlds/forest_hex12.yaml          hku_super/config/super_planner_hex.yaml        params/*.parm
        │ 场景几何                        │ 规划器参数                          │ 飞控参数
        ▼                                 ▼                                     ▼
 ┌────────────────────────┐      ┌────────────────────────┐            ┌──────────────────────┐
 │ lidar_imu.py (仿真)     │      │ SUPER fsm_node          │            │ ArduPilot SITL        │
 │ 10 Hz / 12000 点 /      │      │  hku_super      │            │  build/sitl/arducopter│
 │ IMU 200 Hz              │      │  10 Hz 重规划            │            │  serial0 → tcp:5780   │
 └───────┬────────────────┘      └───────▲─────────┬──────┘            │  serial1 → tcp:5781   │
         │ /livox/lidar (点云)             │ /planning/pos_cmd        │  serial2/3 → 5782/83  │
         │ /livox/imu                      │ (SUPER 自己的多项式)       └───────▲───────┬──────┘
         ▼                                 │                                  │       │
 ┌────────────────────────┐      ┌─────────┴──────────────┐                  │       │
 │ Super-LIO（.build/lio_ws）│     │ 薄层 planner_source_super│                  │       │
 │ 10 Hz 点云 → 200 Hz odom │────▶│  100 Hz / L→W 标定／调速器│                  │       │
 │ /lio/robo/odom           │      │  /plan/cmd (PositionCommand,100 Hz)
 │ /lio/cloud_world         │      └─────────┬──────────────┘                  │       │
 └───────┬────────────────┘                │ 位置+速度+加速度                  │       │
         │                                 ▼                                   │       │
         │                        ┌────────────────────────┐                   │       │
         │                        │ bridge_node.py          │── MAVLink(84/331)─┘       │
         │                        │  钳位=规划包络 → setpoints│                          │
         │                        └────────────────────────┘                           │
         │                                                                             │
         │        ┌────────────────────────┐    ┌──────────────────┐                   │
         └───────▶│ truth_rx.py（真值 tap）  │    │ supervisor.py     │── arm/takeoff/──┘
                  │ UDP:14560 → 14561 扇出   │    │ 守卫/降落/中止     │   mode/LAND
                  └──────────┬─────────────┘    └──────────────────┘
                             │ tap.csv (≈1.2 kHz)
                             ▼
                        .runs/missionA-<tag>/
```

### 0.2 离线（记录 → 评分 → 证据）

```
 .runs/missionA-<tag>/                  scripts/accept.py                 docs/evidence/
 ┌──────────────────────┐              ┌────────────────────────┐        ┌────────────────────┐
 │ tap.csv      真值     │──┐           │ H1..H6 前置闸门          │        │ lap1/lap2-paths.png│
 │ setpoints.csv 发布参考 │──┤           │ W0..W6 任务行            │───────▶│ vxyz-t.png         │
 │ wire.csv      上线值   │──┼──────────▶│ 15 行 + accept.json      │        │ lio-err-t.png      │
 │ lio.csv      估计器     │──┤           │ (行来源/输入哈希)        │        └────────────────────┘
 │ phase.csv    阶段       │──┤           └───────────┬────────────┘                  ▲
 │ traj.log     规划器日志 │──┘                       │                              │
 │ lidar_imu.log / 其它   │                            ▼                              │
 └──────────────────────┘                    RESULT / GATE 行              scripts/render_evidence.sh
```

---

## 1. 坐标系与时钟（一处一个名字）

| 名字 | 是什么 | 谁定义 |
|---|---|---|
| **W** | 任务系（ENU，场地坐标；角点半径 10.000 m） | `sim/worlds/forest_hex12.yaml` |
| **L** | LIO 世界系（SUPER 看到的点云/地图所在系） | Super-LIO 初始化 |
| **fcu/compass** | 发给飞控的偏航（罗盘航向） | `apm_thin/common/frames.py` 的 `l_to_wire_yaw()` |
| **t_wall** | `time.monotonic()`（薄层、bridge 同一个进程时钟） | 记录里所有 `t_wall` 列 |
| **t_sim_us** | SITL 仿真时间（微秒） | `tap.csv` / `setpoints.csv` 的 `t_sim_us` |

**规则**：跨设备（薄层 ↔ 记录）只能用 `t_wall`；跨仿真（SITL ↔ 滤波）只能用 `t_sim_us`。
`t_sim_us` 在记录里量化到 5 ms，**不能拿它当频率分母**（见 `docs/INTEGRATION.md` 的记录纪律）。
L→W 的换算只有一条实现：`calib/calib.json`（yaw +90.049771°，t = [−0.0008, −0.00409, +0.11557] m）。

## 2. 端口与话题

| 通道 | 地址 / 名字 | 方向 | 内容 |
|---|---|---|---|
| SITL truth tap | UDP **14560**（`TAP`） | SITL → 记录/扇出 | 1.2 kHz 真值（`tap.csv`） |
| 扇出 | UDP **14561**（`FAN`） | truth_rx → lidar_imu/记录 | 同上，多消费者 |
| MAVLink A | tcp **5780**（`PORT0`） | bridge ↔ FCU | 指令 84（setpoints）+ 外部导航 331 |
| MAVLink B | tcp **5781**（`PORT1`） | supervisor ↔ FCU | arm / takeoff / mode / LAND |
| MAVLink C/D | tcp **5782 / 5783** | 诊断 | 备用 |
| `/livox/lidar`, `/livox/imu` | ROS 2 | lidar_imu → Super-LIO | 10 Hz / 200 Hz |
| `/lio/robo/odom` | ROS 2 | Super-LIO → 薄层/bridge | 200 Hz 位姿+速度 |
| `/lio/cloud_world` | ROS 2 | Super-LIO → 记录/ego | 10 Hz 世界系点云 |
| `/planning/pos_cmd` | ROS 2 | fsm_node → 薄层 | SUPER 的多项式轨迹上的点 |
| `/plan/cmd` | ROS 2 | 薄层 → bridge/记录 | `quadrotor_msgs/PositionCommand`，100 Hz（位置+速度+加速度） |

## 3. 一次飞行启动了哪些进程（及它们写什么）

| 进程 | 入口 | 频率/端口 | 产物 |
|---|---|---|---|
| ArduPilot SITL | `.build/ardupilot/build/sitl/bin/arducopter` | 实时（rtf≈1.0） | `sitl.log`、`phase.csv`（经 supervisor） |
| 真值 tap | `sim/src/truth_rx.py` | UDP 14560→14561 | `tap.csv` |
| 雷达/IMU 仿真 | `sim/src/lidar_imu.py` | 10 Hz / 200 Hz | `lidar_imu.log`、`cloud.csv`（可选） |
| 估计器 | `ros2 run super_lio super_lio_node` | 200 Hz odom | `lio.csv`、`lio.log` |
| SUPER 规划器 | `hku_super/.../fsm_node`（`config_name` 由薄层传） | 10 Hz 重规划 | `traj.log`（规划器自身日志） |
| 薄层 | `apm_thin/ros/planner_source_super.py` | 100 Hz | `traj.csv`（参考轨迹记录） |
| 桥 | `ardupilot/bridge/bridge_node.py` | 50 Hz 上线 | `setpoints.csv`、`wire.csv`、`bridge.log` |
| 监督器 | `ardupilot/supervisor/supervisor.py` | 事件驱动 | `supervisor.log`、`phase.csv` |
| 指标 | `scripts/metrics.py` | 结束后一次 | `metrics.txt` |
| 评分 | `scripts/accept.py` | 结束后一次 | `accept.json` |

## 4. 一次飞行的产物（`.runs/missionA-<tag>/`）

| 文件 | 产生者 | 内容 | 被谁读 |
|---|---|---|---|
| `tap.csv` | truth_rx.py | 1.2 kHz 真值（ENU 位置/速度/姿态/加速度） | `accept.py`（W0/W1b/W2/W3/W4/W6、H2/H5）、几乎全部分析工具 |
| `setpoints.csv` | bridge | 50 Hz 发布参考（cmd_*）+ FCU 回读 | `accept.py` W6、`reversal_table.py`、`dip_context.py` |
| `wire.csv` | bridge | 上线后的钳位值（|v|≤max_vel、|a|≤max_acc） | `pva_wire_census.py`、诊断 |
| `lio.csv` | record-odom.py | 200 Hz 估计器位姿 | `accept.py` H3、`lio_*` 工具 |
| `phase.csv` | supervisor | 阶段时间线（takeoff…land） | `accept.py` H1 飞行窗 |
| `traj.log` | 薄层 | 启动横幅、目标序列、CUSP/守卫事件 | `accept.py` H4、分析工具 |
| `traj.csv` | record-traj.py | 100 Hz 参考轨迹（position_cmd） | 轨迹/曲率分析 |
| `lidar_imu.log` | lidar_imu.py | 扫描开销、点数、图案来源 | `lidar_stall.py`（H6）、`accept.py` H6 |
| `supervisor.log` / `bridge.log` | 两节点 | 中止原因、nav gate、钳位 | 诊断（abort 归因） |
| `metrics.txt` | metrics.py | 旧指标（A1/A2/B1…） | 仅历史对照，**验收不用**（见 `docs/INTEGRATION.md` §7） |
| `accept.json` | accept.py | 15 行结果 + 输入文件 sha256 + 臂参数 | 归档/复核 |
| `flight.BIN`, `eeprom.bin`, `cloud.csv` | SITL / 记录 | 飞控日志、EEPROM、点云 | 诊断（体积大，可删） |

**保留策略**：`.runs/` 不入库。文档引用到的趟次保留裁剪版（tap/setpoints/wire/lio/traj.log/phase 等，
≈40 MB/趟），其余可按需删除；仓库里只保留 `docs/evidence/` 的四张图与 `accept.json` 的摘要。

## 5. 参数从哪来（谁决定行为）

| 参数 | 文件 | 谁读 | 备注 |
|---|---|---|---|
| 场地/柱林/角点/任务高度 | `sim/worlds/forest_hex12.yaml` | lidar_imu.py、薄层、`check_frozen.py` | SHA-256 冻结，改它要先改 SPEC |
| 规划器（max_vel 1.35、robot_r、inflation…） | `hku_super/config/super_planner_hex.yaml` | fsm_node（`config_name:=`） | 三份副本由 `scripts/install_config_hex.sh` 同步 |
| 飞控（外部导航信任、guided 限幅） | `params/ekf_trust_lio_flight.parm`、`params/guided_limits_fast.parm` | SITL `--defaults` | `fly_speed_run.sh` 打印生效值 |
| L→W 标定 | `calib/calib.json` | 薄层、bridge | 实测值，不入 .git？—— **入库**（唯一真值） |
| 参考调速/前馈 | 环境变量（`REF_*`，默认在 `fly_hex.sh`） | 薄层 | `traj.log` 首行打印 `ref shaping:` 横幅 |

## 6. 评分链（`scripts/accept.py`）

输入：sortie 目录 + world 文件 + 判据参数（`--alt`、`--waypoint-box`、`--body`、`--ground-speed`、
`--cruise-p90`、`--known-defect`）。输出：屏幕 15 行 + `accept.json`。

* **H1–H6 前置闸门**：主机实时（飞行窗 rtf）、tap 完整性、LiDAR 扫描预算（H6）、LIO 存活（H3）、
  SUPER 参考存在（H4）、真值与 FCU 量程一致（H5）。任一不过 → 数字不可评级，`GATE: FAIL`。
* **W0–W6 任务行**：路线窗、航点/按序接近、巡航（p90≥1.40 且触到上限）、高度、**柱面净空**、
  避障、跟踪（零时移 p50/p90 + 每四分之一时基偏移）。
* `--known-defect ROW`：用户已接受的已知缺陷（当前 `W4`），数字照打、JSON 里 `ok:false`，
  门限行写明"accepting N KNOWN DEFECT(S)"。

## 7. 数据分类与生命周期（什么进 git）

| 类别 | 位置 | 进 git？ | 说明 |
|---|---|---|---|
| 源码/配置/世界/参数/标定/文档/图 | 仓库内 | **是** | 全部可复现的输入 |
| 第三方源码（SUPER/Super-LIO/livox） | `third_party/` | **是**（SUPER 已打补丁） | 见 `third_party/pins.env` |
| ArduPilot SITL | `.build/ardupilot` | 否 | `setup/fetch_ardupilot.sh` 固定 commit |
| MID-360 射线图案 | `sim/pattern/*.csv` | 否（供应商数据） | `sim/pattern/README.md` / `scripts/check_pattern.sh` |
| 编译产物 | `.build/` | 否 | `setup/build.sh` |
| 飞行记录 | `.runs/` | 否（保留裁剪版于本机） | `setup/run_hex.sh` 写入 |
| 机器本地路径 | `setup/runtime.env` | 否 | 由 `setup/env.sh` 读取 |

## 8. 失败模式 → 对应仪器（本仓库特有）

| 现象 | 先看哪个文件 | 工具 |
|---|---|---|
| rtf 掉到 0.98 以下（主机忙） | `phase.csv` 飞行窗 | `scripts/check_flight_rtf.py`、accept H1 |
| 雷达扫描超预算（点云断档 → LIO 跳变） | `lidar_imu.log` | `scripts/lidar_stall.py`（accept H6） |
| 估计器跳变/发散 | `setpoints.csv` 的 lio_* 列、`lio.csv` | `bridge` nav gate 计数、`supervisor` 守卫 |
| 参考折返/腿中掉速 | `setpoints.csv`、`traj.log`（CUSP 行） | `scripts/reversal_table.py`、`dip_context.py` |
| 规划器拒规划（走廊不可行/超时） | `traj.log` | `_scratch/faults_census.sh`、`scripts/planner_faults.sh` |

## 9. 目录 → 角色（五个模块 + 跨模块目录）

```
apm_thin/    模块1 自研薄层 + 共享库：ros/planner_source_super.py、ros/ref_lp.py、common/、planner/
hku_super/   模块2 hku-mars SUPER：super_planner/、rog_map/、mission_planner/、mars_uav_sim/、
             config/（交付配置与速度变体）、patches/（我们的补丁 10/11/12）
slam/        模块3 Super-LIO + livox_ros_driver2 + config/livox_360.yaml
ardupilot/   模块4 SITL 驱动侧：bridge/、supervisor/、params/、msgs/quadrotor_msgs
sim/         模块5 仿真环境：src/（lidar_imu/raycast/truth_rx）、pattern/（射线图案）、
             worlds/（冻结世界）、rviz/、frozen/
scripts/     跨模块编排：飞行臂 fly_*、评分器 accept.py、记录器、run-mission.sh、~150 个分析工具
setup/       入口脚本（env.sh 是唯一路径权威；build/fetch/run_hex）
calib/       L→W 标定（calib.json）
docs/        规约（SPEC）、本管线、记录（INTEGRATION）、证据（evidence）、历史（harness-history/）
tests/       无飞行自检（run_all.sh + CI）
PINS.env     第三方 commit 与补丁说明
```

模块之间只通过两种东西耦合：**话题/端口**（运行时，见 §2）和 **`setup/env.sh` 的路径变量**
（文件，构建时）。所以换掉任一模块（例如把仿真换成实机、把 SUPER 换成别的规划器）不需要动其它模块。

## 10. 与"别人的项目"的对照（为什么这样分）

* **ROS 2 工作区惯例**（SUPER/Super-LIO/livox 各自是独立包）→ 本仓库不把它们揉进自己的源码树，
  而是放 `third_party/` 并用 `setup/build.sh` 软链进 `.build/ws_*`；这与 ArduPilot 的 `--defaults`
  外置参数、PX4 的 `ROMFS/init.d` 外置启动脚本是同一类做法：**上游源码只读，改动以补丁脚本存在**。
* **研究仓库惯例**（如 FAST-LIO/EGO-Planner 的 docs/ + scripts/ + config/ 三分）→ 我们保留
  `docs/`（规约与记录分开）、`scripts/`（可复现实验入口）、`config/`（实验变量）；
  额外用 `third_party/pins.env` 记录 commit（EGO/FAST-LIO 常用 submodule，我们用 vendored + pins，
  避免 clone 时必须 `--recursive`）。
* **可复现性惯例**（论文仓库常见 `reproduce.sh`）→ 我们的对应物是 `setup/run_hex.sh` +
  `tests/run_all.sh`：前者一条命令飞+评分，后者一条命令做无飞行自检（静态闸门、图案、单测、链接检查）。
