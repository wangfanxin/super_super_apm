# ACCEPTANCE —— 验收、交付与真机迁移（合并自 DELIVERY-2026-11-25 / P0-CRITERIA-PROPOSAL /
# REAL-MACHINE-CHECKLIST / PATHS / LEGACY_PATHS）

本文件是验收与交付的**唯一**入口：15 行判据、v2 提案、交付清单、"规范化"7 条、真机迁移清单、路径政策、
历史分析工具假设。改动判据（P0）必须所有者批准；本文件只记录现状与提案。

## 0. 一页看懂

- 判据：`scripts/accept.py` 共 15 行（H0–H6 / W0–W6 / W0b）；默认 **v1**，`--criteria-v2` 为 opt-in 提案。
- 已知缺陷：`W4`（±5 cm 接触，显式记账 `--known-defect W4`）、`H2`（tap-era 380 Hz，P0 待批）；
  `W6`（单季滞后）为真实瞬态，v2 下仍不过。
- 基线航班：`phase4oF`（H6 p50 12.9 ms/skip 9、W6 spread 0.079、W4 −0.027）与 `phase4oH`（13.9/8、0.084、+0.003）。
- 验收命令：`python3 scripts/accept.py --run <sortie> --world sim/worlds/forest_hex12.yaml --alt 2.0 [--criteria-v2] [--json out.json]`
- 交付入口：`scripts/fly_hex2.sh`；门禁：`bash tests/run_all.sh`（GATE: PASS 后方可 commit）；
  证据：`docs/evidence/`（图 + INDEX.txt）。


---

## 交付清单（DELIVERY-2026-11-25.md）

> 原文：`docs/DELIVERY-2026-11-25.md`（已并入本文件，独立文件删除）

## 交付清单：数据管线「规范化完成」（2026-11-25）

> 本文件是这条数据管线**交付状态**的单一入口：哪些做完了、证据在哪、怎么跑、怎么回滚、还剩什么。
> 状态：**Phase 0–5 完成；门禁连续两次 PASS；P0 判据改造已实现为 opt-in、默认未切换（等所有者批准）。**

### 1. 运行方式（三条命令）

```bash
## 1) 一次性构建（自包含：SITL + LIO + SUPER 规划器 + 本仓库节点）
bash setup/build.sh

## 2) 一趟飞行（默认：merged 单客户端、10000 点 + legacy 调度、SIM_STATE 自建真值、bag 录制）
BAG=1 BAG_SPLIT=1 SORTIES=1 bash scripts/fly_hex2.sh <tag> 1

## 3) 门禁（不需要飞行；约 2–3 分钟）
bash tests/run_all.sh          # 期望最后一行 GATE: PASS
```

单趟产物（`<LIO_LOGS>/missionA-<tag>-01-01/`）：`manifest.json`（含 git/bin/config/world/calib/argv/
`truth_timebase`）、`bag_a/`+`bag_b/`（核心 4 话题）、`derived/lio.csv`+`derived/traj.csv.cmd.csv`、
`tap.csv`（真值，带 `t_us,t_sim_us`）、`scans.csv`（每帧雷达）、`phase.csv`、`nav.csv`、`setpoints.csv`、
`wire.csv`、`accept.json`（15 行 + `extras`）、`fcu_client.log`、`lio.log`、`lidar_imu.log`。
回放一趟：`bash scripts/replay_bag_rviz.sh <sortie> [--rate 2]`（无屏幕加 `--no-rviz`）。

### 2. 阶段状态与证据

| Phase | 交付 | 证据 |
|---|---|---|
| 0 仓库自包含 | `.build/.runs` 在仓库内；从 `hku_super/`+`slam/` 重编；无外部运行根/`runtime.env` | `docs/PHASE0-VERIFY.md`，飞行 `phase0l` |
| 1 真值 | 自建 MAVLink 客户端订阅 `SIM_STATE(108)`；`compare_truth` 与旧 tap 对比；**D4 显式时钟映射**（`tap.csv t_sim_us`、`truth_timebase.json`、manifest 字段） | `docs/PHASE1-TRUTH.md`、`tests/check_timebase.py` |
| 2 录制 | rosbag2（tmpfs 先写 + split writers）→ 派生 CSV；派生层逐样本等于实时层 | `docs/PHASE2-RECORDING.md`、`phase2k`、`tests/check_bag_pipeline.py` |
| 3 FCU 接口 | 单客户端（merged：一进程、两连接、两 node/线程）；extnav 显式年龄/质量/reset 门；bisect 形态删除，legacy 仅回滚 | `docs/PHASE3-MAVLINK.md`、`phase4g/4h`、`tests/check_launch_matrix.sh`、`tests/check_nav_gates.py`、`tests/check_fcu_client.sh` |
| 4 轻量 sim | 默认 10000 点 + legacy 调度；每帧 `scans.csv` 供 H6；rviz 回放入口 | `docs/PHASE4-SIM.md`、`phase4oF/4oH`、`tests/check_rviz_replay.sh`（按需） |
| 5 评估/产物 | manifest 完整性、APE/RPE 附加行、bag→derive→live CI、P0 提案 | `docs/PHASE5-EXTRAS.md`、`scripts/traj_error.py`、`docs/P0-CRITERIA-PROPOSAL.md` |

### 3. 默认配置（交付值）

| 项 | 值 | 说明 |
|---|---|---|
| FCU 客户端 | `FCU_CLIENT=merged` | 回滚：`FCU_CLIENT=legacy`（双进程） |
| 真值 | `TRUTH_SRC=simstate` | tap 补丁与旧 `truth_rx.py` 在 `sim/legacy/` |
| 雷达 | `LIDAR_POINTS=10000`，`LIDAR_SCHEDULER=legacy` | 两个都是 **sim-only**；整窗均匀调度用 `LIDAR_SCHEDULER=new` |
| 录制 | `BAG=1`（点云默认不录，`BAG_CLOUDS=1` 可选） | 核心约 45–50 MB/趟 |
| 验收 | 15 行，v1 判据（默认） | `accept.py --criteria-v2` 为 opt-in 提案 |
| 时间 | 飞行路径只用真实 `time.monotonic()` | 飞控侧 `JitterCorrection` 负责任意时基映射 |

### 4. 回滚开关（都经过实测）

| 场景 | 开关 |
|---|---|
| 合并客户端回归 | `FCU_CLIENT=legacy`（历史双进程） |
| 雷达调度回归 | `LIDAR_SCHEDULER=legacy`（默认） / `SIM_LIDAR_LEGACY=1` |
| 真值链路 | `TRUTH_SRC=tap` + `LEGACY_TAP=1`（补丁 SITL `arducopter.tap-patched` 仍在 `.build`） |
| 录制底座 | 不传 `BAG` 时回退到实时 CSV 记录器（过渡期两条都保留） |
| 判据 | v1 为默认；`--known-defect W4` 接受已知缺陷 |

### 5. 已知遗留（明确不是"规范化"欠账，需要决策/硬件）

1. **P0 判据**：`docs/P0-CRITERIA-PROPOSAL.md` + `docs/metrics/p0-criteria-ab.txt`（104 趟 A/B）。
   批准后把 `--criteria-v2` 切默认即可；不批准则维持 v1。
2. **W4 柱面接触**：已知交付缺陷（−0.02…−0.10 m），根因是规划器安全边距，需单独立项（patch 12 实测有害）。
3. **W6 单季度滞后**：时间基准修复后仍偶发（`4oE/4oG/4p`），是真实跟踪瞬态，等 P0 判据改造后归因。
4. **真机迁移**：见 `docs/REAL-MACHINE-CHECKLIST.md`（无 GPS/罗盘/气压计配置、ODOMETRY 接口、extnav 门限、
   上电→首飞顺序）。真机没有 SIM_STATE 真值，15 行验收不能整套照搬。
5. **可选研究**：`LIDAR_SCHEDULER=new` 是否值得切默认（现有 A/B 与 W6 无稳定关联）。

### 6. 门禁覆盖（`tests/run_all.sh`）

语法 → 静态门禁（世界冻结/参数/补丁已应用）→ 图案窗口（整窗覆盖/逐扫描前进）→ 单元测试 →
**bag 派生 + 派生层=实时层** → **timebase（D4）** → **nav gates（extnav 显式门）** → **launch matrix
（一套 FCU 实现）** → **APE/RPE + manifest 完整性** → **criteria-v2 opt-in** → FCU bench（SITL+MAVLink 真实链路）
→ 路径政策 → 证据链接 → 工具 smoke。
按需（不在默认 gate）：`tests/check_rviz_replay.sh`（headless 回放冒烟，有 bag 时）。

### 7. 本次"规范化完成"的判定标准（全部满足）

- [x] 一份克隆可构建、可飞、可评分，不依赖作者机器路径（`tests/no_abs_paths.sh` PASS）
- [x] 真值、时钟、录制、FCU 接口各有**唯一**实现，且都有回滚开关
- [x] 每趟飞行产物可独立复算（manifest + bag + 派生层 + 明文日志）
- [x] 验收 15 行 + 附加块（APE/RPE、manifest 完整性）可重复产出
- [x] 门禁连续两次 PASS；新增测试都阻断在 `run_all.sh`
- [x] 每个阶段有证据文档 + PROGRESS 流水 + 已知遗留清单（P0/W4/W6/真机）


---

## P0 判据提案（P0-CRITERIA-PROPOSAL.md）

> 原文：`docs/P0-CRITERIA-PROPOSAL.md`（已并入本文件，独立文件删除）

## P0 判据改造方案（提案，**未实施** —— 等所有者批准）

> 依据 `docs/PIPELINE-PLAN.md` §6："验收判据改造（P0）按所有者意见延后"。本文只做**数据 + 方案**，
> `scripts/accept.py` 的 15 行判定与阈值**一行未改**。批准后建议以 `--criteria-v2` 开关实现，旧判据保留为默认，
> 逐项切换、每项都用 `docs/` 里的历史 sortie 重算对比（仓库已有 80 趟 `accept.json` 可回放）。

### 数据来源

`/home/fanxin/sp_runs/missionA-*-01-01/accept.json`，共 **80 趟**（2026-10 至今，含 tap 时代与 SIM_STATE 时代）。关键分布：

| 行 | 现状 | 80 趟实测 |
|---|---|---|
| H2 真值速率 | 要求面值 **380 Hz**（tap 时代 1200 Hz 打点） | SIM_STATE 之后所有趟 `rate median` = **200.0 Hz**（配置的发布率）；tap 时代 333.3/569.8/1200.5 都有 → 阈值测的是**录制方式**，不是飞行质量 |
| H6 扫描预算 | cast MAX ≤100 ms，skipped ≤20 | 健康趟 p50 10–16 ms、MAX ≤60 ms、skipped 5–13；坏趟 p50 可到 1515 ms、MAX 3326 ms、skipped 621（主机停滞期） |
| W1b 航点 | 每个航点 ≤0.50 m | 所有通过的趟 worst 0.06–0.16 m；边缘 0.5 的趟都是坏趟 |
| W4 柱面间距 | **> 0 m**（CLEAR） | 80 趟 min = **−0.096 m**，中位数 **−0.0525 m** ⇒ 按"必须 >0"几乎每趟都会 FAIL（已知交付缺陷） |
| W6 时间一致性 | 四季度 offset spread ≤**0.15 s**，且 lag 补偿后 p90 ≤0.60 m | 见下：0.019–0.147 的趟大量存在，但也有 0.15–0.5 的"单季度异常"；时间基准修复前还有 0.6/1.2（拟合窗边缘）的假失败 |

**W6 的关键观察**（时间基准修复后，`tap.csv t_sim_us` 上线起）：通过趟（4oA/4oF/4oH/4oB/4g/4h/4kA）spread 0.049–0.133；
失败趟（4oC/4n/4kB/4oG/4p）spread 0.20–0.47，形态统一为**某一个季度** offset 明显偏离（−0.18…−0.54 s），
其余季度正常；同趟 p90-at-offset 也从 ~0.25 涨到 0.5–1.1。也就是说：**它不是时钟问题，而是那一段真的滞后了**
（飞机在某个季度落后参考 ~0.2–0.5 s），但 W1b/W2/W5 都通过 —— 是"参考跟踪的瞬时滞后"，不是走错路。

### 提议（逐行，全部等批准）

#### P0-1 W6（最需要改）
- **语义**：把"零滞后"拆成两件事并分开报告：
  1. **稳态**：四季度 lag 的**中位数**（稳健）必须 ≤ **0.35 s**（实测通过趟 −0.21…+0.13 s）；
  2. **一致性**：去掉线性时钟漂移（offset 对季度中点时间做一次线性拟合）后的**残差标准差** ≤ **0.12 s**
     （实测通过趟残差 ≤0.06 s；失败趟 0.15–0.35 s）；
  3. 保留现有的 **lag 补偿后 p90 ≤0.60 m**（每季度）。
- **理由**：现有 spread 把"线性时钟比误差"（rtf 允许 0.98–1.02，130 s 可漂 ±0.24 s）和"单季度真实滞后"混在一起，
  而时间基准修复后前者应被显式去掉；同时**单独报告**"单季度异常"（任意季度 lag 偏离中位数 >0.25 s 记为 WARN 行，
  不直接 FAIL）——这样既不掩盖真实滞后，也不会让一个瞬时抖动否掉整趟。
- **阈值来源**：80 趟 + 时间基准修复后的 12 趟干净/失败对照（见上表）。

#### P0-2 H2（真值速率）
- **现状问题**：380 Hz 是 tap 时代（1200 Hz 内插）的产物；SIM_STATE 时代按设计发布 200 Hz，永远 FAIL。
- **提议语义**：`真值速率 == 配置的 --publish-hz（±5%）` **且** `最大间隙 ≤ 2 × 1/rate` **且** `tap.csv 行数覆盖飞行窗`。
  实测所有 SIM_STATE 趟 rate=200.0、max gap 0.005 s，稳定通过。

#### P0-3 W4（柱面间距）
- **现状问题**：CLEAR > 0 的判据下 80 趟里几乎没有通过的（min −0.096、中位 −0.0525），这是**已知交付缺陷**，
  不是每趟新事故。
- **提议（需所有者选择）**：
  a. **容差版**：允许最近间距 ≥ **−0.05 m**（机身盒模型宽 0.2 m 的一半）且"进入柱体内部的样本比例 ≤ 0.5 %"；
  b. **不改**，而是把 W4 明确标为 `known defect`（accept.py 已有 `--known-defect W4` 机制，GATE 会**点名接受**）；
  c. 先修规划器安全边距（patch 12 SFC margin 实测有害，需另立项目）。
  实测支持：负值集中在 −0.02…−0.09，且仅 0.3–2% 样本在柱体内。
- **建议**：先用 (b) 让验收干净（owner 已批准 W4 为已知缺陷），(a) 作为下一阶段目标；**不改数字口径**。

#### P0-4 H6（扫描预算）
- 健康趟 p50 12–16 ms、MAX ≤60 ms、skipped 5–13；现有 100 ms/20 次仍能区分坏趟（p50 1515 ms / skipped 621）。
- **提议**：阈值不动；把 skipped 上限从绝对 20 改为 **max(20, 1% × 扫描数)**（130 s ≈ 1300 帧 → 13 次 ≈ 1%），
  避免长航时被固定 20 卡住；同时报告 **cast p99**（现有只报 p50/MAX）。

#### P0-5 H3 / W1b / W2（低优先）
- H3（LIO 200 Hz / span）：建议改为"速率 == 配置 `--nav-hz`(48) 的期望 200 Hz ±10% 且无 >0.1 s 间隙"（与 W6 稳态一致化）；
- W1b（≤0.50 m）：实测通过趟 ≤0.16，提议收紧到 **0.30 m**（更能反映真实精度）；
- W2：round 20 已把 4.0 改到实测巡航（1.40/1.50），**不再改**。

### 实施方式（批准后）

1. `scripts/accept.py` 增加 `--criteria-v2`，**默认关闭**；v1 行为逐字节保持不变；
2. 每个改动的行在 JSON 里加 `"v2": {...}` 数据块（新旧数字并排），便于回放对比；
3. 用仓库里 80 趟 `accept.json`/原始 CSV 做一次全量回放，输出"新旧判定差异表"进 `docs/metrics/`；
4. 所有者签字后才把默认切到 v2，并在 `docs/SPEC.md` 更新语义说明。

### 为什么现在不做

P0 属于**判据语义/阈值**，会改变"什么算通过"；它必须由所有者决定（尤其 W4 是否允许接触、W6 是否拆成稳态+一致性）。
本文件只是把数据摆好，供审批。


### A/B 回放证据（2026-11-25，104 趟存量 sortie，**阈值未改**）

`scripts/criteria_ab.py --runs-dir /home/fanxin/sp_runs --out docs/metrics/p0-criteria-ab.txt`
把每趟的现行判据（v1）与提案（v2）并排重算，结果：

| 行 | v1 → v2 翻转 | v2 不可判 | 说明 |
|---|---|---|---|
| H2 | **4 趟 F→T**（phase4oF/4oG/4oH/4p） | 100（tap 时代 sortie 无 `t_sim_us`） | 这正是时间基准修复后 200 Hz SIM_STATE 的趟；v1 的 380 Hz 是 tap 时代产物，永远 F |
| W4 | **20 趟 F→T**（clear −0.005…−0.047 m，体内样本 0.02–0.42%） | 24 | v2 的"≥−0.05 m 且体内 ≤0.5%"恰好覆盖全部已交付的 ±5 cm 缺陷，且不放过任何更深的接触 |
| W6 | **0 趟翻转**（可判 28 趟） | 76（老 sortie 无显式仿真钟） | v2 与 v1 在全部可判趟上一致：**单季度真实滞后（如 phase4p）v2 仍然 FAIL**，不会把它洗白；v2 只是把"线性时钟漂移"显式去掉 |
| H6 | 0 趟翻转（可判 14 趟） | 90 | skip 上限 max(20, 1%) 在现役航程上等价 |

结论：v2 只在该翻的地方翻（H2 语义、W4 容差），在 W6/H6 上与 v1 等价；没有"用新判据掩盖问题"的趟。
evidence 文件：`docs/metrics/p0-criteria-ab.txt`（逐趟明细在文件后半部分）。

### 实现状态（2026-11-25）

- **已实现 `accept.py --criteria-v2`（默认关闭）**：H2/W4/W6/H6 四项按本文提案实现，v1 行为逐字节不变
  （`tests/check_criteria_v2.py` 钉住：默认 v1 与该 sortie 存档 accept.json 一致，v2 只允许这四行翻转）；
- JSON 会写 `"criteria": "v1-default" | "v2-proposed"`，print 第一行也会标注；
- **默认仍是 v1**；切默认需要所有者明确批准（这是 P0 判据类改动）。

### 批准后要做的

1. 用 `docs/metrics/p0-criteria-ab.txt` 过一遍四行差异；
2. 所有者签字后把 `accept.py` 默认改为 v2，同时更新 `docs/SPEC.md` 的判据语义与 `--known-defect W4` 的用法；
3. 跑一次真实飞行，确认 GATE 行与预期一致（H2/W4 变绿、W6 仍反映真实滞后）。


---

## 真机迁移清单（REAL-MACHINE-CHECKLIST.md）

> 原文：`docs/REAL-MACHINE-CHECKLIST.md`（已并入本文件，独立文件删除）

## 真机迁移检查表（从这台仿真栈搬到真实 MID-360 + ArduPilot 飞机）

> 本文是"仿真验收通过之后、第一次真机飞行之前"的检查清单。仿真侧的证据见 `docs/PIPELINE-PLAN.md` /
> `docs/PHASE3-MAVLINK.md` / `docs/EXTERNAL-NAV-ODOMETRY.md`。**没有一项需要改代码**：仿真里跑的东西
> 在真机上替换的是"数据来源"，接口与判据不变。

### 1. 仿真与真机分别负责什么（先分清"哪些是仿真-only"）

| 环节 | 仿真（本仓库） | 真机 |
|---|---|---|
| 飞控/物理 | ArduPilot SITL（同一套 Copter 代码与参数） | 真实飞控（同版本固件最好） |
| 真值 | 自建 MAVLink 客户端订阅 `SIM_STATE(108)`（`sim/src/sim_state_recv.py`） | **不存在**（真机没有真值；评测只能用外部测量/RTK/动捕，或只做在线健康检查） |
| 激光雷达 | `sim/src/lidar_imu.py` 用世界+厂商图案做 raycast（**`LIDAR_POINTS` 等只属于这里**） | Livox MID-360 驱动直接发 `/livox/lidar` + `/livox/imu` |
| 世界/环境 | `sim/worlds/*.yaml`（冻结的六边形森林等） | 真实场地（安全网/护笼/隔离） |
| 可视化 | rviz + `sim/rviz/pipeline.rviz` | 同 rviz（bag 回放也能看真机记录） |
| 验收判据 | 15 行 accept（依赖 tap 真值） | 只能复用其中**不依赖真值**的行（H1/H2 时间、H3 喂数、H4 参考、W1/W1b 航点、W4 需要世界） |

**结论**：`LIDAR_POINTS`、`SIM_LIDAR_*`、`TRUTH_SRC=simstate`、`tap.csv`、`scans.csv`、`worlds/*.yaml` 都是
仿真-only；飞控路径（`fcu_client`/`bridge`/`supervisor`/LIO）与真机**完全同一套代码**。

### 2. 真机侧必须替换/确认的接口

1. **LIO 输入**：真实驱动替换 `lidar_imu.py`，话题必须仍是 `/livox/lidar`（`PointCloud2`，每点带 time）
   与 `/livox/imu`（`Imu`，200 Hz）；外参 `slam/config/livox_360.yaml` 的 `lio.extrinsic.lidar_imu`
   必须用真机实测值（仿真里是 0）。
2. **LIO 输出**：`/lio/robo/odom`（`lio_world`，pose+twist）与 `/lio/odom`（速度）不变；LIO 的
   `calib/calib.json`（`R_WL`/`t_WL`，L→W 的 yaw+平移）必须在真机上重新标定（仿真里 yaw=+90.05°）。
3. **FCU 链路**：`fcu_client --mav` 从 `tcp:127.0.0.1:5780` 改为真实串口/网络（如 `/dev/ttyUSB0,921600`）；
   `--mav-supervisor` 改 serial1（或同一条链路的第二连接，若飞控只有一个口则需要合并为单连接——
   这是唯一需要提前验证的部署问题）。
4. **参数**：`fcu_interface/params/apm_lio_only.parm` 的五源块（`EK3_SRC1_* = 6`、`VISO_TYPE 1`、
   `GPS1_TYPE 0`、`COMPASS_USE 0`、`AHRS_EKF_TYPE 3`）原样写入真机飞控；真机若装罗盘/气压计但不想用，
   必须显式关掉（否则 prearm 会报）。
5. **时间**：飞行路径只用真实 `time.monotonic()`（仿真与真机一致）；不需要飞控时钟（`JitterCorrection` 会
   映射任意时基，要求单调、不来自未来、落后 <500 ms）。
6. **录制**：`BAG=1` 的 bag 底座在真机上同样适用（`record_bag.sh`/`finalize_bag.sh` 不看仿真）；真值
   `tap.csv` 没有，但不影响 bag 与派生层；`accept.py` 会因为没有 tap 而给 H2/W2/W3/W4/W5/W6 判 n/a/F，
   这是预期的（真机验收只用不依赖真值的行 + 外部测量）。

### 3. 上电→首飞的检查顺序（真机）

1. **参数与传感器**：`param show EK3_SRC1_*`、`VISO_TYPE`、`GPS1_TYPE`、`COMPASS_*`；IMU/罗盘校准；
   电机转向/桨叶方向；安全开关/失控保护（`FS_*`）按机体设好。
2. **只接地面**：不装桨。启动 LIO + `fcu_client`（`--nav-max-age-s 0.3 --nav-min-quality 50` 建议开启），看：
   - 飞控 STATUSTEXT 出现 `EKF3 IMU0 is using external nav data`；
   - `LOCAL_POSITION_NED` 跟随真实搬动（水平/高度方向正确、无 90° 旋转错误）；
   - 预解锁通过（无 `VisOdom: not healthy`、无 EKF prearm 报错）。
3. **原点/点**：`SET_GPS_GLOBAL_ORIGIN(611)` 用起飞点经纬高（`fcu_client` 预检已自动发）；
   `LIO` 世界系原点与机头方向在启动时对齐（真机 yaw 观测要合理）。
4. **低空悬停/GUIDED**：2 m 以下、有护笼/系绳；检查位置保持、`/fcu/odom` 与外部测量一致、无 EKF 创新门
   报警；观察 `nav_age`（应 <0.1 s）与 `relay` 状态。
5. **小航线**：先 1 个航点、再 3 个；用 `BAG=1` 录全程，回放 `replay_bag_rviz.sh` 检查轨迹/点云。
6. **逐步放量**：速度、航点、留空时间；每步检查 `W1b` 类指标（航点到达）与 EKF 健康。
7. **失败预案**：EKF failsafe（LAND）、`ESTIMATOR FEED IS STALE`（>1.5 s 会 abort→LAND）、
   LIO 漂移 >2 m 持续 2 s（abort）；真机务必在开阔场地验证这些保护动作。

### 4. 与仿真 A/B 的对照（首次真机后要回答的问题）

| 问题 | 仿真基线（供对照） |
|---|---|
| LIO 相对真值 APE | `phase4p`：rmse 0.027 m、RPE(1s) 0.015 m（`scripts/traj_error.py`） |
| 跟踪滞后 | W6：稳态 lag 中位数 ~−0.15 s，lag 补偿后 p90 ~0.25 m |
| 巡航速度 | p90 1.4–1.6 m/s（`max_vel 1.5`）、高度 2.0 m |
| 扫描/IMU 预算 | H6：cast p50 12–14 ms、skipped 5–10；IMU 200 Hz |
| 喂数速率 | 331：48 Hz（实测 46.8–47），extnav 健康阈值 300 ms |
| 航点精度 | W1b worst 0.09–0.16 m（0.5 m 门限） |
| 柱面间距 | W4 已知缺陷 −0.02…−0.10 m（P0 提案容差 −0.05 m） |

### 5. 真机唯一"没有对应物"的东西

- **真值/验收 15 行**：真机没有 SIM_STATE；建议真机评测用 RTK/动捕（外部真值）或只做在线健康
  （EKF flags、nav_age、relay、bag 回放）+ 场地标记点的到达精度。
- **世界 YAML 与几何判据**（W4/W5）：真机需要实测场地地图（点云→地图），否则这两行无意义。
- **`tap.csv` / `scans.csv` / `compare_truth`**：仿真专属记录，真机录制只保留 bag 与派生层。


---

## 路径政策（PATHS.md）

> 原文：`docs/PATHS.md`（已并入本文件，独立文件删除）

## 路径约定（仓库内 / 运行根 / 历史路径）

### 1. 三层布局

| 层 | 位置 | 进 git？ | 谁决定 |
|---|---|---|---|
| **仓库**（源码/配置/世界/参数/标定/文档） | 本目录（clone 到哪都行） | 是 | `setup/env.sh` 从脚本位置推出 `DSH_ROOT` |
| **运行根**（编译产物、飞行记录） | `.build/`、`.runs/`（默认在仓库内；本机在 WSL 的 `/home/fanxin/super_super_apm`） | 否 | `setup/runtime.env`（不入库）或环境变量 |
| **模块** | `bridge_nodes/`（薄层+MAVLink 桥+监督器+共享库）、`hku_super/`（SUPER）、`slam/`（Super-LIO+livox）、`fcu_interface/`（飞控参数+消息）、`sim/`（仿真/世界/rviz/冻结） | 是 | 本文件 §1.1 |
| **ArduPilot** | `.build/ardupilot`（`setup/fetch_ardupilot.sh`） | 否 | `PINS.env` |

`setup/env.sh` 是**唯一**的路径权威：`DSH_ROOT` / `BRIDGE_ROOT` / `SUPER_PLANNER_ROOT` / `SLAM_ROOT` / `SIM_ROOT` / `ARDUPILOT_DIR` /
`WS_SUPER` / `LIO_WS` / `ARDUPILOT_ROOT` / `LIO_LOGS` / `CALIB_FILE` / `MID360_PATTERN`。
每个入口脚本都会自己向上找 `setup/env.sh` 并 source 它（不需要先 export 任何东西）。

### 2. 历史路径（读旧记录时对照）

`docs/INTEGRATION.md` 与 `docs/evidence/` 是**历史记录**，逐字保留，里面出现的下列路径是迁移前的布局：

| 记录里的路径 | 现在的等价物 |
|---|---|
| `/home/fanxin/lio_avoid/logs/missionA-<tag>` | `<LIO_LOGS>/missionA-<tag>`（本机 `/home/fanxin/super_super_apm/.runs/`） |
| `/home/fanxin/lio_avoid/src/...` | `sim/src`（仿真）、`ardupilot/{bridge,supervisor}`（驱动侧）或 `.build/ardupilot`（SITL） |
| `/home/fanxin/lio_avoid/lio_ws` | `.build/lio_ws`（本机 `/home/fanxin/super_super_apm/.build/lio_ws`） |
| `/home/fanxin/super_super_apm/upstream/SUPER` | `hku_super`（本机的构建仍用 WSL 里那份上游副本） |
| `/mnt/d/distill_self/lio_avoid` | 拆分后的 `sim/` + `bridge_nodes/` + `fcu_interface/` |
| `/mnt/d/distill_self/_stage/calib.json` | `calib/calib.json` |
| `/mnt/d/distill_self/release` | 已并入本仓库（`release` 是发布前的暂存名） |

旧的 `lio_avoid` / `lio_nav_apm` / `gazebo_apm` / `_stage` 大头已在 2026-11-24 清理；
`/home/fanxin/lio_avoid` 现在只是一个**符号链接兼容壳**（4 个链接），让历史分析工具的默认路径仍能解析。

### 3. 规则

1. 新脚本一律用 `setup/env.sh` 的变量或 `scripts/_paths.py`，**不写作者的绝对路径**。
2. 飞行/评分链（`setup/`、`scripts/fly_*`、`scripts/run_*`、`scripts/accept.py`、`scripts/check_*`、
   `bridge_nodes/{thin,mavlink,supervisor}`、`sim/src`、`tests/`）必须路径无关，CI 会检查。
3. 历史分析工具（`scripts/` 下的一次性诊断）如果仍含旧路径，登记在下表，不再扩大。

### 4. 仍含历史路径的文件（有意保留）

见 `tests/no_abs_paths.sh` 的输出：它把"必须路径无关"的文件当失败，把历史工具当提示。


---

## 历史分析工具的路径假设（LEGACY_PATHS.md）

> 原文：`docs/LEGACY_PATHS.md`（已并入本文件，独立文件删除）

## 仍假设作者绝对路径的文件（编译与飞行链**不**在其中）

本仓库的**编译 / 飞行 / 评分**链是位置无关的：`setup/env.sh` 是唯一知道路径的地方，
`scripts/` 与 `harness/` 里的入口脚本会自己向上找到它（`setup/env.sh` 的 bootstrap 由打包脚本插入）。
从任意目录 clone 都能跑，`docs/` 里的每个数字都出自这条链。

**例外是分析工具**。`scripts/` 下约 150 个工具里，仍有一部分把作者的绝对路径写成默认值，例如

    /mnt/d/distill_self/…            （作者的工作区）
    /home/fanxin/lio_avoid/logs/…    （作者的记录目录）

它们不影响编译与飞行，而且**都接受显式参数**（`--run`、`--world`、`--tag`），所以在任何机器上只要
把路径传进去就能用，例如：

    python3 scripts/accept.py --run /data/runs/missionA-kx6-03-01-01 \
                              --world worlds/forest_hex12.yaml --alt 2.0 --body box

为什么没有全部改掉：打包时对 223 个 Python 文件做过一次机械替换，结果 23 个文件被插到了
`from __future__` 之前而报语法错误、108 个文件里的 f-string / 隐式拼接仍然残留旧路径 —— 那种"看起来
改完了其实一半没改"的状态比诚实标注更糟。所以范围被明确限定为"编译 + 飞行 + 评分"这条链，
其余在本文档里列清楚。

判断某个工具属于哪一类：如果它在 `setup/run_hex.sh`、`scripts/fly_hex*.sh`、
`scripts/run_until_healthy.sh`、`scripts/run_accept.sh` 的调用链上，它已经位置无关；否则按上面的
方式显式传参。


### FCU 接口：一套实现 + 一个显式回滚开关（2026-11-25 第 20 轮）

Phase 3 的 bisect 模式（`bridge_only` / `bridge_thread` / `supervisor_only` / `--legacy-shape` /
`--dual-link`）已完成使命并被**删除**（`bridge_nodes/mavlink/fcu_client.py` 与 `scripts/run-mission.sh`
里都不再存在）。现在只有两个值：

| `FCU_CLIENT=` | 形态 | 用途 |
|---|---|---|
| `merged`（默认） | 一个进程 `fcu_client.py`：bridge 半边 serial0（84/331，自己的线程+node+executor），supervisor 半边 serial1（class-76/守卫，自己的线程+node+executor） | **唯一验收路径** |
| `legacy` | 历史双进程：`bridge_node.py`（serial0）+ `supervisor.py`（serial1） | **回滚开关**：merged 出回归时用它 bisect/复飞；不再是验收路径 |

`tests/check_launch_matrix.sh`（入门禁）静态钉住这件事：启动器只接受这两个值、merged 分支必须带两个
endpoint、legacy 分支必须起两个进程、merged 客户端不得再出现已删除的 bisect 参数、`--bench` 必须保留。
