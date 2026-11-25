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

**待办（下一轮）**：
1. 按计划做 3 趟 A/B（legacy vs new）并在安静主机上对比 H6/W6/W2/W1b，决定是否把 new 设为默认；
2. 若 W6 lag 变差（首趟 spread 0.379），先查是否需要在 LIO 侧按新点时间分布调整（或把点数默认提到 20000）；
3. `LIDAR_POINTS` 当前默认 20000，但 fly 脚本可能仍传 12000 —— A/B 时固定为同一值。
