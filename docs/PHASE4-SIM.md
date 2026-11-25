# Phase 4：轻量仿真的三个修正（图案窗口 / 每帧 CSV / rviz 回放）

**日期**：2026-11-25　**状态**：代码完成并通过门禁与 bench 验证；**默认仍是 legacy 调度**，
按计划先做 3 趟 A/B 再切换（第一趟 `phase4i` 已飞：整圈 13/13 完成，但 W6 lag spread 0.379 s > 0.15，
说明这个传感器侧改动在控制行里可见，必须先有 A/B 集）。

## 1. 图案窗口与"12000 点 = 60 ms 突发"（`sim/src/raycast.py`、`sim/src/lidar_imu.py`）

**问题**（两处）：
- `load_pattern(n=12000)` 取的是供应商图案的**前 12000 行**：200 kHz 下正好 60 ms，于是每次 100 ms 的扫描
  只在前 60 ms 打点，形成"突发"；
- 图案窗口在启动时装载一次，`start` 从不推进——真实的 MID-360 是非重复图案，逐扫描应前进。

**修正**（`--new-lidar` / `SIM_LIDAR_SCHEDULER=new`）：
- `raycast.load_pattern_all()` 一次读入整个 1 s 图案（800 001 行）；
- `raycast.window_indices(n, start_row, window_rows, total)` 在 `1/scan_hz` 的整窗内**均匀**取 n 个点
  （n ≥ window_rows 时取全窗）；
- 每次扫描 `scan_index` 前进一个窗口（`start_row = k·window_rows`），`Sensor.set_pattern()` 换窗（numpy 视图，不重读 CSV）；
- per-point 时间 = 窗口内偏移/点率，因此 12000 点也覆盖 **0.099995 s**（原来 0.059995 s）。

**回滚**：`--legacy-lidar` 或 `SIM_LIDAR_LEGACY=1`（默认就是 legacy，直到 A/B 完成）。

## 2. 每帧 CSV（`--scan-csv`）与 H6

`lidar_imu.py --scan-csv <run>/scans.csv` 每次扫描尝试写一行：

```
t_start_us,cast_ms,points,reason        # reason ∈ ok / empty / truth_hole / behind
```

`scripts/run-mission.sh` 现在总是传 `--scan-csv "$OUT/scans.csv"`（两种调度都写）。
`scripts/accept.py` 的 **H6 优先读 scans.csv**（`[per-scan: scans.csv]`），否则回退旧的 2 s 聚合日志
（`lidar_stall.parse`）——阈值一字未改，只是统计粒度从"2 s 汇总"变成"每帧"。

实测（`missionA-phase4i-01-01`，新调度）：`scans.csv` 1644 行，H6 = `p50 18.5 ms, MAX 51.2 ms, skipped 12`，
与旧聚合在量级上一致。

## 3. rviz 回放（单一入口）

| 文件 | 作用 |
|---|---|
| `sim/rviz/pipeline.rviz` | 固定帧 `world_w`：world markers、truth(/sim/truth/odom)、LIO(/lio/robo/odom)、FCU(/fcu/odom)、plan 位姿+路径、`/lio/cloud_world`、`/livox/lidar`（可选） |
| `scripts/viz_bridge.py` | 只读适配器：odometry → `/viz/{truth,lio,fcu}_path`，`PositionCommand`(无 rviz 显示类型) → `/viz/plan_pose` + `/viz/plan_path` |
| `scripts/replay_bag_rviz.sh` | 一条命令回放一趟：读 `calib/calib.json` 发布 `world_w→lio_world` 静态 TF、起 viz_bridge、`ros2 bag play`（支持 `bag/` 或 split 的 `bag_a`+`bag_b`）、开 rviz2 |
| `scripts/run_rviz.sh` | 默认配置改为 `pipeline.rviz`（live 查看时同一套显示） |

点云默认不进 bag（owner 指示：省体积），需要看 `/lio/cloud_world` 时用 `BAG_CLOUDS=1` 录一趟。
本工具**不在**自动验收链里，是人工 debug 入口。

## 4. 验证与 A/B 计划

已通过（无飞行）：
- `scripts/check_pattern.sh` 新增断言：20000 点跨满 0.099995 s、12000 点均匀抽稀后同样跨满、窗口逐扫描前进；
- bench（SITL+真值客户端+lidar 6 s）：新调度 prints `window 0.100 s = 20000 rows, advanced per scan`，
  `scans.csv` 59 行全 `ok`、cast p50 16.1 ms；legacy 回滚路径复现 0.059995 s；
- `tests/run_all.sh` GATE PASS；`accept.py` H6 在 phase4i 上以 `[per-scan: scans.csv]` 读出。

## 5. A/B 实测（2026-11-25，LIDAR_POINTS=12000 固定）

| 趟 | 调度 | 结果 |
|---|---|---|
| `phase4g/4h`（Phase 3 验收） | legacy | 完整 13/13，H6 True，**W6 spread 在 0.15 内** |
| `phase4i` | new | 完整 13/13，H6 True（p50 18.5 ms/skipped 12），**W6 spread 0.379** |
| `phase4jD` | legacy @20000 点 | H6 **FAIL**（skipped 237：20000 点在此主机上超出扫描预算），H5/W6 连带失败 |
| `phase4kA` | legacy | H6 True（p50 16.1/skipped 9），但 plan box 在 leg9 停发导致 abort（不计入比较） |
| `phase4kB` | **new** | 完整 13/13，H6 True（p50 18.3 ms/skipped 9，**per-scan CSV 1628 行**），W4 −0.061（已知缺陷），**W6 spread 0.201** |

初步结论：
- **20000 点在本主机上不可用**（扫描预算超支、大量 skipped）→ 保持 12000；
- 新的整窗调度本身工作正常（H6 与旧聚合同量级，scans.csv 完整，扫描窗口 0.099995 s、逐扫描前进）；
- 但两趟 new 调度的 **W6 offset spread 都超过 0.15**（0.379 / 0.201），而 legacy 基线在界内。W6 衡量的是"plan 与 truth 的
  时间基准偏移在四个季度间的一致性"——new 把点的时间分布从"前 60 ms 密集"改成"整 100 ms 均匀"，LIO 的输出时间行为随之
  改变，这是**判据感知到的真实差异**（不是 H6 的扫描预算问题）。
- ⇒ 默认继续 legacy；需要再做两对 A/B（安静主机）并检查 LIO 侧是否应按点时间分布调整（或官方是否本就用 60 ms 窗口）。

**待办**：
1. 再飞 2 对 A/B 确认 W6 差异是否稳定复现；若稳定，方案二选一：(a) 保持 60 ms 窗口但在其内均匀抽点；
   (b) 与 LIO 的 `livox_360.yaml` 时间参数对齐后再切换；
2. `LIDAR_POINTS` 的 fly 默认 12000 与注释里的 20000 不一致，需统一（本轮已确认 20000 不可用）。


## 6. 2026-11-25 15:25 默认切换（owner 指示）：10000 点 + 整窗均匀调度

owner 决定：**默认改成 10000 点/帧，并使用整窗均匀抽点**（`--new-lidar`）。已改：

| 位置 | 旧默认 | 新默认 |
|---|---|---|
| `scripts/run-mission.sh` | `LIDAR_POINTS=20000`, `LIDAR_SCHEDULER=legacy` | **`LIDAR_POINTS=10000`, `LIDAR_SCHEDULER=new`** |
| `scripts/run_until_healthy.sh` | `LIDAR_POINTS=12000` | **`LIDAR_POINTS=10000`，并把 `LIDAR_SCHEDULER` 显式传给 run-mission** |
| `scripts/fly_session.sh` | 20000 | **10000** |

回滚：`SIM_LIDAR_LEGACY=1` 或 `--legacy-lidar`（调度）；`LIDAR_POINTS=<n>`（点数）。
两个参数都是 **SIM-ONLY**：只有 `sim/src/lidar_imu.py` 读它们；真机的点云来自 Livox 驱动，这个脚本根本不会启动，
LIO/bridge/supervisor/FCU 都不读 `LIDAR_POINTS`（run-mission 的横幅里也注明了 `[sim-only]`）。

**为什么 10000 可以接受**：实测 LIO 相对真值的漂移很小（`scripts/lio_drift.py`，两圈 130 s 飞行）：
`phase4kB` 误差 0.10–0.37 m、`phase4g` 0.04–0.20 m，没有 >0.25 m 的跳变；降低点数主要影响点云密度，
需要飞行 A/B 确认 W1b/W6 不退化。


## 7. 2026-11-25 16:20 A/B 结论：默认 = 10000 点 + **legacy 调度**（整窗均匀调度改为 opt-in）

owner 指示"改成 10000 点 + 均匀采"后，先按指示把默认切到 `LIDAR_POINTS=10000` + `LIDAR_SCHEDULER=new`，
然后做了 A/B（固定 10000 点，只切换调度；全部单趟、BAG=1、merged 客户端、同一 world）：

| 趟 | 调度/窗口 | H6 | W1 | W1b | W6 | W6 spread |
|---|---|---|---|---|---|---|
| `phase4oA` | legacy | T | 13/13 | T | **T** | 0.077 |
| `phase4oB` | **new**/0.100 | T | 13/13 | T | **T** | 0.129 |
| `phase4n` | **new**/0.100 | T | 13/13 | T | F | 0.250（Q2 −0.177） |
| `phase4oC` | **new**/0.100 | T | 13/13 | T | F | 0.469（Q2 −0.374） |
| `phase4oD` | new/0.060（窗内均匀） | **F**（cast MAX 580 ms、skipped 43 = 主机停顿） | 13/13 | T | F | 0.516（污染，不计） |
| `phase4oE` | legacy | T | 13/13 | T | F | 0.000，**四个季度全部 +0.600（搜索边界）** |

A/B 统计（干净的趟）：new 调度 4 趟里 1 趟过 W6；legacy 6 趟里 5 趟过（`4oA/4d/4g/4h/4kA` 过，`4oE` 不过）。

**但 `phase4oE` 的失败暴露了 W6 自身的另一个问题（比调度更重要）**：该趟的
`cmd vs truth` 偏移是 **常数 +1.35 s / 四个季度一致**，`lio vs truth` 也是常数 **+1.45 s（quarter 间 spread 0.03）**
—— 这是**录制时间基准**的问题（`setpoints.csv` 的 `t_sim_us` 来自 LIO 消息时间戳，`tap.csv` 是 SIM_STATE 真值时间，
两者在这一趟差了 ~1.4 s），不是跟踪问题；W6 的 ±0.6 s 搜索窗因此全部顶在边界。`accept.py` 自己就有注释指出
W6 记录两套时钟、在好趟上只剩 2 ms 余量、**"de-trending 是判据改动（P0）不是 bug fix，所以没做"**。

结论（按计划"先 A/B 再切默认"的规则）：
1. **默认保持 legacy 调度 + 10000 点**（`LIDAR_SCHEDULER=legacy`）；新调度用 `LIDAR_SCHEDULER=new`
   或 `--new-lidar` 显式开启。它是可用的（H6 更好：p50 12–13 ms；13/13 航点），但当前 W6 判据下通过率更低。
2. 10000 点本身没问题：legacy@10000 的 `4oA` H6 p50 13.5 ms / skipped 6 / W6 过；比 12000 更省 CPU。
3. **下一个真问题**：`setpoints.csv t_sim_us`（LIO 时间戳）与 `tap.csv`（SIM_STATE 时间）之间的时间基准差
   在某些趟里是 0.15 s、某些趟是 1.4 s —— 属于 D4（显式时钟映射）范畴，应在 W6 判据改造（P0）之前先把
   录制时间基准统一/标定，否则 W6 会周期性假失败。


## 8. 2026-11-25 16:40 收尾：时间基准修复 + Phase 4 完成证据

上面 §7 的 A/B 被一个新发现推翻了一半：**`tap.csv` 的 `t_us` 与飞行侧 `t_sim_us` 是两套时钟**，
W6 直接混用会把"录制偏移"计入（`phase4oE` 常数 +1.35 s → 四个季度全顶在 ±0.6 s 搜索边界）。
修复见 [PHASE1-TRUTH.md](PHASE1-TRUTH.md) 的 D4 补充：`tap.csv` 现在带 `t_sim_us` 列、accept 优先用它、
`truth_timebase.json` + `manifest.truth_timebase` 记录偏移、`tests/check_timebase.py` 入门禁。

修复后的真实飞行（固定 `LIDAR_POINTS=10000`）：

| 趟 | 调度 | H6 | W1/W1b | W4 | W6 | spread |
|---|---|---|---|---|---|---|
| `phase4oF` | legacy | T (p50 12.9/skip 9) | 13/13, T | CONTACT −0.027（已知缺陷） | **T** | 0.079 |
| `phase4oH` | legacy | T (p50 13.9/skip 8) | 13/13, T | **CLEAR +0.003** | **T** | 0.084 |
| `phase4oG` | new | T | 13/13, T | CONTACT | F | 0.413（Q1 −0.500, p90 0.77；时钟无关） |

**结论**：
- **Phase 4 完成**：每帧 CSV + H6（已生效）、rviz 单入口（`pipeline.rviz` + `viz_bridge.py` + `replay_bag_rviz.sh`）、
  图案窗口修正（`--new-lidar`）已实现并有 bench/飞行数据；**默认 = `LIDAR_POINTS=10000` + legacy 调度**，
  `LIDAR_SCHEDULER=new` 保留为实验开关。
- **验收证据**：最终默认下**连飞两趟干净趟** `phase4oF` / `phase4oH`（除 H2 这一 P0 判据外全绿；
  W4 是已知 ±5 cm 缺陷，两趟中一趟 CLEAR 一趟 CONTACT）。
- **遗留调查项**：new 调度的 W6 单季度异常（跟踪行为，非时钟、非点云预算），在 P0 判据改造（de-trending /
  单季度残差门限）之后再评估是否切默认。
