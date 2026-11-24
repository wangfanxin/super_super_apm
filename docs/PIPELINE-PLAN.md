# 数据管线完善方案（PIPELINE PLAN）

> 目标读者：本仓库所有者。本文是**方案**，不是实现记录；每条改动都给出验证方式与回滚点。
> 背景：2026-11-24 的逐文件代码审查（4 个模块）发现，当前"飞得起来、能出数"的链路里有几处
> **自建替代品**（改固件取真值、每 topic 手写 CSV、两个自建 MAVLink 客户端、自建 lidar 仿真），
> 它们把仿真与实机、录制与评分、配置与二进制绑在一起，导致：
> ① 一趟飞行的数字不能从产物独立复算；② 仿真学到的时序/时基结论无法迁移到实机；
> ③ 配置里写的约束与实际二进制不一致（与本文正交，属另一条线，见对话中的 2026-11-24 逐文件代码审查记录）。
>
> **判定类问题（P0，判据/评分器）明确不在本文范围**，按所有者意见延后。

---

## 0. 一页结论

| 项 | 现状 | 目标 |
|---|---|---|
| 真值 | 改 ArduPilot 固件插 1.2 kHz UDP tap + 自建 fanout | **SIM_STATE**（MAVROS `sim_state` 插件，官方已对 ArduCopter SITL 验证） |
| 录制 | 每个 topic 一个 `record-*.py` → CSV（不可回放、会丢尾、无 gap 计数） | **rosbag2 全量录制**（含 tf、/clock、/sim/state），CSV 由 bag 派生 |
| 时钟 | 三套戳（sim µs / host monotonic / Unix wall）混在 CSV | bag 接收时间 + 源戳齐全；**时钟映射显式记录**（`/clock`、t_wall↔t_sim 对） |
| 控制接口 | 两个自建 MAVLink 客户端（bridge 84/331 + supervisor 76） | **单一标准接口**：MAVROS（`setpoint_raw/local`、`vision_pose`、`state`）或 ArduPilot DDS |
| LIO→FCU | 冻结位姿无限转发、无质量/协方差/reset | 年龄门 + quality/covariance + reset_counter；停发即记名 |
| 仿真器 | 自建 UDP tap + 自建 raycast；图案只加载一次、12k 点=60ms 突发 | 标准 Gazebo（`ardupilot_gz` + 传感器插件）**为主**，轻量自建 sim 保留为 CI/快速回归 |
| 可复现 | sortie 目录缺 world/calib/parm/banner，accept.json 只哈希 9 个输入 | 每趟 **manifest**（git/二进制/配置 hash、argv、世界、标定、parm、时钟图）＋ bag 可回放复算 |
| 自包含 | 7.1 GB 运行根在仓库外（ArduPilot、upstream/SUPER 副本、编译产物、记录） | `.build/`、`.runs/` 收进仓库（gitignore），无 `runtime.env`、无外部运行根 |

**推荐节奏**：Phase 0（自包含，1–2 天）→ Phase 1（真值+时钟，2–3 天）→ Phase 2（bag 录制+派生，2–3 天）
→ 到此为止可获得 **90% 的收益**；Phase 3（控制接口标准化）与 Phase 4（Gazebo）按需推进。

---

## 1. 管线目标（验收性质）

一条**合格**的数据管线应满足下面 6 条，后续每个 Phase 都对着它们验收：

1. **单一真值源**：仿真真值来自不受我们修改的 ArduPilot（`SIM_STATE`），实机真值来自 FCU 自己的日志；
   同一份分析代码能读两者。
2. **单一时间语义**：每个数据都带源戳；任何跨设备比较先过**显式时钟映射**；不存在"猜"的时基。
3. **可回放**：一趟飞行录成 bag；重放 bag 能重跑分析、必要时重跑节点，结果可复算。
4. **可复现**：manifest 记录 git hash、二进制 hash、配置 hash、世界/标定/parm、argv 与阈值；
   只给产物目录就能独立复算全部验收数字。
5. **失败可见**：任何静默降级（LIO 停发、命令被拒、录制丢包、模式被改）必须在产物里留**命名**记录并可被闸门检查。
6. **真机同构**：仿真路径与实机路径的差异只剩"真值从哪来"和"传感器从哪来"，其余 topic/节点/参数一致。

---

## 2. 目标架构

```
                 ┌──────────────────────────── 仿真侧（可替换） ────────────────────────────┐
 worlds/ ───────▶│ ArduPilot SITL（不修改，--defaults 外置参数）  +  Gazebo / 轻量 sim       │
                 │   MAVLink / DDS                              SIM_STATE(108) 真值         │
                 └───────────────┬──────────────────────────────────────┬────────────────┘
                                 │ setpoint_raw/local + vision_pose      │ /sim/state, /clock
                                 ▼                                       ▼
  ┌────────────── 机载侧（真机同一套） ──────────────────────────────────────────────────────┐
  │  slam/  Super-LIO ──/lio/robo/odom(200Hz)──▶  bridge_nodes/thin ──/plan/cmd──▶ bridge  │
  │            └──/lio/cloud_world(10Hz)──▶ hku_super fsm_node ──/planning/pos_cmd──┘        │
  │                      ▲                      │                                            │
  │                      │ goals: PoseStamped/PVA（W→L 一次转换）                          │
  │  bridge_nodes/supervisor（守卫/命名中止/一个 MAVLink 或 MAVROS 连接）                    │
  └──────────────────────────────┬──────────────────────────────────────────────────────────┘
                                 │ 所有 topic（含 tf、/clock、/sim/state、status）
                                 ▼
                    ┌──────── rosbag2 全量录制 ────────┐
                    │  bag  ──▶ 派生 CSV（分析/评分）   │──▶ scripts/accept.py（读派生层）
                    │  manifest.json（git/bin/config） │──▶ docs/evidence（图）
                    └──────────────────────────────────┘
```

### 2.1 组件职责（目标态）

| 层 | 组件 | 输入 | 输出 | 备注 |
|---|---|---|---|---|
| 仿真 | ArduPilot SITL（**不修改**） | `--defaults` 外置 parm | MAVLink / DDS；`SIM_STATE` | 真值不再靠固件补丁 |
| 仿真 | Gazebo（`ardupilot_gz`）或轻量 sim | world、pattern | `/livox/lidar`、`/livox/imu` | 二选一，接口一致 |
| 估计 | `slam/` Super-LIO | 点云/IMU | `/lio/robo/odom`、`/lio/cloud_world`、`/lio/status`（新增：速率、扫描消费数、reset 计数） | 状态必须可被判活 |
| 规划 | `hku_super/` fsm_node | odom、世界系点云、goal | `/planning/pos_cmd`、`poly_traj` | 上游代码尽量不改 |
| 规划适配 | `bridge_nodes/thin/` | pos_cmd、goal、world | `/plan/cmd`（**L 系**，yaw=罗盘） | 保留调速器/fallback，但 fallback 必须指向当前腿 |
| 桥 | `bridge_nodes/mavlink/`（或 MAVROS 客户端） | `/plan/cmd` | setpoint_raw/local；vision/odom 给 EKF | **一个连接**；年龄门、质量、`/bridge/status` 命名降级 |
| 守卫 | `bridge_nodes/supervisor/` | FCU 状态、truth/FCU 对比 | arm/takeoff/land；命名 abort | 不再需要 tap 也能工作（仿真用 SIM_STATE，实机用 FCU） |
| 录制 | `rosbag2` 节点 | 全部 topic | bag | 唯一录制底座 |
| 派生 | `scripts/bag_to_csv.py`（新增） | bag | CSV（与旧格式兼容） | 分析脚本不改也能读 |
| 评分 | `scripts/accept.py` | 派生 CSV + manifest | 15 行 + JSON | 判据本身延后修 |
| 证据 | `scripts/render_evidence.sh` | 派生 CSV | docs/evidence | 由 bag 可重画 |

### 2.2 关键接口约定（写死）

| 接口 | 帧/单位 | 契约 |
|---|---|---|
| `/plan/cmd`（`quadrotor_msgs/PositionCommand`） | L 系（z up），yaw = **FCU 罗盘航向**（0=N，+90=E） | 位置/速度/加速度均为 L；yaw 只在**生产者**转换一次；yaw_dot 明确为 0 |
| `vision_pose` / `ODOMETRY` | 位置 NED，姿态/速度按接口要求 | 单一转换函数；质量/协方差/reset 来自 `/lio/status` |
| `/lio/status`（新增） | 无帧 | `{stamp, odom_age, cloud_age, cloud_hz, scans_consumed, reset_counter, quality}` |
| `/bridge/status` | 无帧 | `{sent, refused, nav_age, mode, clamps_hit, wire_alive, named_degradations[]}` |
| manifest.json | 无帧 | `{git, binary_sha, config_sha, world_sha, calib_sha, parms[], argv, accept_args, clock_map, build_env}` |

---

## 3. 现状 → 目标：保留 / 替换 / 删除

| 项 | 处理 | 理由 |
|---|---|---|
| 规划器接口（`PositionCommand`、世界系点云） | **保留** | 与 EGO/px4ctrl/SUPER 生态一致，是本仓库最对的地方 |
| 薄层调速器/限速器（policy） | **保留**，但拆成独立节点 `bridge_nodes/thin/governor` | 策略与传输解耦；`ref_lp` 修子步后再启用 |
| 两个自建 MAVLink 客户端 | **替换**为单连接（MAVROS 或 DDS） | 标准化、有 SIM_STATE/vision 插件、可被生态工具观察 |
| ArduPilot 固件 tap 补丁 | **从飞行路径移除**（保留在旧分支供对比一年） | 真机不存在；SIM_STATE 已官方验证 |
| 每 topic 手写 CSV 录制器 | **替换**为 rosbag2 + 派生工具 | 回放/可复现/不丢尾 |
| 自建 lidar 仿真 | **降级为轻量 sim**（CI/快速回归）；发布路径用 Gazebo | 传感器模型标准化；图案窗口 bug 一并修 |
| 自建 `truth_rx` fanout | **删除** | SIM_STATE 直接是 ROS 话题 |
| `setup/runtime.env` + WSL 运行根 | **删除** → `.build/`、`.runs/` 进仓库 | 自包含 |
| 旧评分器判据 | **暂不动**（按所有者意见延后） | 与管线正交 |

---

## 4. 分阶段实施

每个 Phase 都有：做什么 / 交付物 / 验证 / 回滚 / 工时 / 风险。**任一 Phase 完成后系统都可飞**。

### Phase 0 — 自包含与可追溯（1–2 天，需编译）

**做什么**
1. `.build/`、`.runs/` 移进仓库（gitignore 已覆盖）；删除 `setup/runtime.env` 与 `/home/fanxin/super_super_apm` 运行根、`/home/fanxin/lio_avoid` 兼容壳。
2. 从 `hku_super/` 重编 `ws_super`（fsm_node 首次真正来自仓库源码）；从 `slam/` 重编 `lio_ws`；ArduPilot 拉进 `.build/ardupilot`（先保留 tap 补丁）。
3. 启动横幅打印：git hash、fsm_node 二进制 sha256、配置 sha256、world/calib sha256、argv。
4. 每趟写 `manifest.json`（先只做可字段，不含评测阈值）。
5. `PINS.env` 补全（列出实际 baked 的行为改动与对应脚本）。

**交付物**：仓库内 `.build`/`.runs`；`manifest.json`；更新后的 `setup/build.sh` 文档。
**验证**：`tests/run_all.sh` PASS；`env -u DSH_ROOT bash setup/run_hex.sh <tag> 1 1` 飞一趟，manifest 完整；记录 30 分钟编译时间基线。
**回滚**：`git revert`；运行根在删除前先 `tar` 到 `_keep/`。
**风险**：`/mnt/d` 上编译慢、记录写入落在 Windows 盘可能影响 rtf → 验证时对比 H1；必要时只把 `.runs` 留 WSL（由 manifest 记录位置）。

### Phase 1 — 真值与时钟（2–3 天）

**做什么**
1. 安装并接 MAVROS（或 `ardupilot_ros` DDS，二选一，见决策 D1）的 `sim_state` 插件；订阅 `/sim/state/*`。
2. 新增 `scripts/compare_truth.py`：同一趟里旧 tap.csv 与 SIM_STATE 的位置/速度/时间差（目标：位置差 < 1 cm、时间差 < 2 ms 的映射可拟合）。
3. 三条腿并行跑 3 趟（tap / SIM_STATE / 双录），出对比报告；确认 SIM_STATE 的更新时间与频率（SITL 每步）后，**把 tap 从飞行路径移除**。
4. 时钟：录制 `/clock`；每个录制行都带 `(recv_stamp, src_stamp)`；`bag→CSV` 时同时写两列（替代当前 t_wall/t_sim_us 的歧义）。
5. 删除 `sim/src/truth_rx.py` 与 fanout。

**交付物**：`/sim/state` 订阅与录制；`compare_truth` 报告（3 趟）；更新 `sim/pattern/README`、`docs/DATA_PIPELINE.md` 的真值一节。
**验证**：H5/H1 在改动后仍 PASS；`compare_truth` 的位置差 < 1 cm、时钟映射残差 < 2 ms；相同 sortie 的 W6 在两种真值下差 < 0.01 m。
**回滚**：tap 补丁与 fanout 代码保留在 `sim/legacy/`，`LIO_TRUTH_TAP` 环境变量切回即回滚。
**风险**：SIM_STATE 在 AC 4.5 的字段/频率与文档有差异 → 用 `compare_truth` 先量再做。

### Phase 2 — 录制与派生（2–3 天）

**做什么**
1. `scripts/record_bag.sh`：rosbag2 录 `/lio/*`、`/plan/cmd`、`/planning/*`、`/mavros/*`、`/sim/state/*`、`/bridge/status`、`/tf`、`/clock`、`/rosout`；QoS 显式（BEST_EFFORT 的 topic 用 `--qos-profile-overrides-file`）。
2. `scripts/bag_to_csv.py`：从 bag 生成与旧格式**同列名**的 CSV（tap/setpoints/wire/lio/traj），再跑 `accept.py`；先对比同一趟新旧 CSV 的 accept 数字**完全一致**，作为无缝迁移的证明。
3. 保留手写录制器一个版本（feature flag）作为兜底；两趟后删除。
4. `bag` 每小时约几百 MB：`--max-bag-size` 分卷，`--compression-mode file zstd`。

**交付物**：bag 录制/派生脚本；同趟新旧对比报告；`accept.py` 无需改动即可读派生 CSV。
**验证**：同一 sortie：`accept.py` 从 CSV（旧）与从 bag→CSV（新）逐行一致；`ros2 bag play` 回放后重跑派生结果一致；断电/中断不产生"看似正常"的 CSV（派生脚本校验 bag 完整性）。
**回滚**：派生脚本只读 bag，旧录制器保留一个 release。
**风险**：ROS 2 的消息时间与现有 CSV 语义差异 → 全部留在派生层解决，不动分析/评分代码。

### Phase 3 — 控制接口标准化（3–5 天）

**做什么**
1. `bridge_nodes/mavlink/` 改为 MAVROS 客户端（订阅 `/plan/cmd`，发 `/mavros/setpoint_raw/local`；`vision_pose`/`vision_speed` 发估计器）；或改 ArduPilot DDS（AP_DDS）。**单一连接**。
2. LIO→FCU：`/lio/status` 年龄门（>200 ms 停发 + 命名降级）；quality/covariance/reset_counter 映射；`VISO_*` 参数与 status 一致。
3. `nav gate`、`wire liveness`（relay==1、`sent` 递增、模式离开 GUIDED 即命名中止）、完成超时。
4. yaw 转换只保留**一处**（生产者），bridge 对 `frame_id` 校验；ego 臂补上转换。
5. supervisor 与 bridge 共享同一连接或两个 MAVROS 命名空间；truth 相关守卫在仿真用 SIM_STATE、实机用 FCU 日志。

**交付物**：新 bridge（feature flag `BRIDGE_IMPL=mavros|legacy`）；3 趟 A/B（legacy vs mavros）：8 行核心验收 + W6 对比，要求差异在跑动方差内。
**验证**：`LIO` 停发实验（`pkill` LIO）→ 200 ms 内 extnav 停发、supervisor 命名 abort、产物可查；模式切 RTL → 命名 abort；断电/断链 → 可见失败。
**回滚**：环境变量切回 legacy 实现；MAVROS 与自建实现并存一个 release。
**风险**：MAVROS 引入依赖与延迟（DAEMON）；`setpoint_raw` 的 yaw 语义要与现有转换对齐 → 用同一趟回放做单元级对比。

### Phase 4 — 仿真标准化（3–5 天，可选）

**做什么**
1. Gazebo（`ardupilot_gz` + livox/MID360 模型）跑同一 world；确认 SUPER 的 ROG-Map 仍要世界系点云（保持）。
2. 轻量 sim 保留为 `sim-lite`：修 **图案窗口只在启动时加载一次**、**12000 点=60 ms 突发**两个问题；新增每帧 CSV（cast_ms/原因/点数）。
3. 世界与 rviz 作为唯一「环境」来源，Gazebo/SITL/轻量 sim 读同一份 YAML。

**验证**：同一 world 在 Gazebo/轻量 sim 下各飞一趟，H1/W2/W6 差异可解释；H6 改为读每帧 CSV。
**回滚**：`SIM_BACKEND=gazebo|lite` 环境变量。
**风险**：Gazebo 传感器模型与自建 raycast 不同 → 需要一次"传感器等价"对比（同一 world 的柱面距离分布）。

### Phase 5 — 评估与产物（1–2 天，判据类改动仍在延后）

**做什么**
1. accept.py 加 manifest 输入（不做判据变更）；缺失必需输入 → FAIL（这条属于判据，等批准）。
2. 标准轨迹指标（evo APE/RPE）作为**附加**行（不改 15 行）。
3. 保留裁裁剪后的 sortie（bag + 派生 CSV + manifest）与 `docs/evidence` 的对应图。
4. CI 加：`bag → derive → accept` 的回放一致性检查（用仓库里一个 30 s 样例 bag）。

**交付物**：可从产物独立复算的验收数字；CI 回放测试。

---

## 5. 关键决策点（需要你拍板）

| # | 决策 | 选项 | 推荐 |
|---|---|---|---|
| D1 | 控制接口 | (a) MAVROS 2；(b) ArduPilot DDS（`ardupilot_ros`/AP_DDS）；(c) 自建单连接 | **(a)** 生态最成熟 + `sim_state` 插件 + 与 EGO/px4ctrl 同构；DDS 作为后续选项 |
| D2 | 仿真器 | (a) 轻量自建为主；(b) Gazebo 为主 + 轻量做 CI；(c) 只留 Gazebo | **(b)**，Gazebo 负责"传感器模型可信"，轻量 sim 负责"快" |
| D3 | 真值 | (a) SIM_STATE；(b) 保留 tap | **(a)**，tap 只保留在旧分支 |
| D4 | 时钟 | (a) ROS sim time（`use_sim_time`）；(b) wall + bag 记录 + 显式映射 | **(b)**：改动小、实机同构；映射写进 manifest |
| D5 | 自包含 | (a) `.build`/`.runs` 进仓库；(b) 外部运行根 + manifest | **(a)**（你的要求）；若 rtf 受影响，`.runs` 单独由 manifest 声明位置 |

---

## 6. 明确不做 / 延后

* **验收判据改造（P0）**：按你的意见延后（W2/W6/H2/H3/H6/W1b/W4 的语义与阈值）。
* **SUPER 内部死键/约束失效**：单独一条线（`yaw_mode`/`backup_traj_en`/`hot_start`/边界约束），要改上游或永久文档化，与管线解耦。
* **角点停 vs SPEC S1**：需要你决策，不阻塞管线。
* **ego 臂是否保留**：Phase 3 后只有一条控制实现；ego 臂降级为历史。

---

## 7. 管线的验收脚本（新增，Phase 0/1 起逐步启用）

```bash
# 一趟飞完自动执行
bash tests/pipeline_check.sh <tag>
#  1. manifest 完整（git/bin/config/world/calib/argv）
#  2. bag 存在且可读；派生 CSV 与 bag 一致
#  3. /sim/state 与 FCU 日志的帧/时钟映射残差 < 阈值
#  4. extnav 停发实验（仿真内注入）→ 命名降级出现
#  5. 回放 bag → 重跑派生 → accept.json 的哈希一致
```

---

## 8. 风险清单

| 风险 | 影响 | 缓解 |
|---|---|---|
| MAVROS 引入额外延迟/依赖 | 跟踪指标短期波动 | A/B 3 趟；legacy 保留一个 release |
| `/mnt/d` 上编译/记录慢 | rtf 掉出 0.98–1.02 → 不可评级 | Phase 0 先测；必要时 .runs 留 ext4 由 manifest 声明 |
| SIM_STATE 字段/频率与文档不符 | 真值频率不足 | Phase 1 先做 `compare_truth`，不通过不切换 |
| bag 体积（每小时） | 磁盘 | zstd + 分卷 + 只保留有分析的趟次 |
| 两套 sim 并存 | 维护成本 | 轻量 sim 只做 CI，功能冻结 |

---

## 9. 立即可开工的三件事（都不涉及判据）

1. **Phase 0**：自包含迁移（1–2 天，含编译），这是后面所有验证的前提。
2. **Phase 1 的只读部分**：先写 `compare_truth.py` 并只跑现有记录（不需要改飞行链）。
3. **Phase 2 的只读部分**：写 `bag_to_csv.py` 与一条样例 bag 的派生对比（不动现有录制器）。

> 三条都可以并行且可回滚；任何一条完成都会让"数字可复算"推进一大步。
