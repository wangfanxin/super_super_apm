# Phase 2 验证记录：rosbag2 作为录制底座，CSV 层从 bag 派生

**日期**：2026-11-25
**结论**：目标态可用 —— 一趟真实飞行（`missionA-phase2k-01-01`，两圈、正常 LAND+disarm）的录制**全量**落在
rosbag2 里（`/lio/robo/odom` 200.0 Hz、`/sim/truth/odom` 与 `/plan/cmd` 100.0 Hz、`/fcu/odom` 49.6 Hz 即其发布率），
`scripts/bag_to_csv.py` 从 bag 派生的 `lio.csv` **包含现采记录的每一行（逐字节相同）**、`traj.csv.cmd.csv` 14456 行
全部逐字节相同。同趟新旧 accept 对比 **14/15 行完全一致**；唯一差异是 H3 的"录制窗口"（派生层比旧记录器**多 42 个
样本**，因为旧记录器丢了它们）；把派生层裁到旧记录器的同一时间窗后 **15/15 行完全一致**。

## 组成

| 部件 | 文件 | 作用 |
|---|---|---|
| bag 记录器 | `scripts/record_bag.sh` | 单 writer 或 `BAG_SPLIT=1` 双 writer；QoS 覆盖表 `scripts/bag_qos.yaml` |
| 收尾/派生 | `scripts/finalize_bag.sh` | 把 tmpfs 里的 bag 拷回 sortie 目录，派生 `derived/{lio.csv,traj.csv.cmd.csv}`，写 `bag_compare.txt` |
| 派生工具 | `scripts/bag_to_csv.py` | `--bag` 可重复（合并 split bag），`--derive-all`；列与旧记录器逐字节一致 |
| 无飞行门禁 | `tests/check_bag_pipeline.py`（已接入 `tests/run_all.sh`） | 合成 Odometry/PositionCommand → 录 bag → 派生 → 校验行内容 |
| 启动器接入 | `scripts/run-mission.sh` | `BAG=1` [±`BAG_SPLIT=1`]；bag 在 LiDAR/LIO 发布者**之前**启动；`LIGHT=1` 跳过 cloud/traj CSV |

## 关键测量（都改变了实现）

1. **磁盘写入会冻结整个 WSL VM**：bag 写在 ext4 虚拟盘上时，LiDAR `raycast` 从 ~15 ms 跳到 **1861–2249 ms**，
   LIO 失锁、sortie abort（phase2g/2h/2i）。改为先写 **tmpfs（/dev/shm）**、飞行结束后拷回仓库后，
   同一套链路的飞行恢复正常（phase2j/2k 两趟完整通过）。
2. **单 writer 在飞行负载下抓不全**：`/lio/robo/odom` 只抓到 ~35–128 Hz（应为 200）。隔离基准显示单 writer
   写磁盘与 tmpfs 都能到 **438 Hz**，所以不是 rosbag2/磁盘能力问题，而是整栈运行时 writer 被抢。
   `BAG_SPLIT=1` 把 topic 按速率分给两个 writer（A: 200 Hz odom；B: 三个 100 Hz topic），并分核
   （`BAG_CORES=15`、`BAG_CORES2=19`）后达到全量。
3. **bag 记录器要在发布者之前启动**：rosbag2 会订阅之后出现的 topic；先启动 bag（和旧的 `record-odom` 并列），
   派生层与现采层覆盖同一窗口。`record_bag.sh` 在调用方已 source ROS 时跳过重复 source（省 2–4 s 启动延迟）。

## 同趟新旧 accept 对比（`phase2k-01-01`）

```
live vs bag-derived        identical 14/15   differ=['H3']
live vs clipped-derived    identical 15/15   differ=[]
H3 live : 30268 samples over 151.3 s (200.0 Hz)
H3 der  : 30310 samples over 151.5 s (200.0 Hz)   <- 派生层是超集：live 的 30268 行全部逐字节存在
H3 clip : 30268 samples over 151.3 s (200.0 Hz)   <- 把派生层裁到 live 的时间窗
```

`traj.csv.cmd.csv`：live 14456 行 = derived 14456 行，**全部逐字节相同**。

## 复现

```bash
# 一趟带 bag 的飞行（tmpfs、双 writer）
BAG=1 BAG_SPLIT=1 bash scripts/fly_hex2.sh <tag> 1
# 该 sortie 目录里会有：bag_a/ bag_b/ derived/ bag_compare.txt
# 同趟新旧对比
LIO_LOGS=/tmp/copy bash scripts/run_accept.sh <sortie-tag> forest_hex12.yaml 2.0 10.0 box 0.5 2
# 无飞行门禁（合成数据）
python3 tests/check_bag_pipeline.py
```

## 仍未完成/延后

* H2 的 380 Hz 速率门槛是 tap 时代判据（真值现在 200 Hz）——P0 批次重定义。
* W4 的 −0.05 m 接触是交付已知缺陷（与管线无关）。
* 目标态还需要"无 CSV 记录器"运行（全部从 bag 派生）；本轮为同趟对比保留了 `record-odom`，下一步把它也去掉，
  并把 `derived/` 直接落成 `lio.csv`/`traj.csv.cmd.csv`（`finalize_bag.sh` 已支持：文件不存在时用派生结果补位）。

## Phase 2 目标态飞行（`missionA-phase2m-01-01`）

**没有任何 per-topic CSV 记录器**：`record-odom`/`record-traj`/`record-cloud` 全部不启动；飞行结束后
`finalize_bag.sh` 把 tmpfs 里的两个 bag 拷回 sortie 目录、派生 `lio.csv`（28871 行）+ `traj.csv.cmd.csv`，
`metrics.py`/`accept.py` 直接读派生层。

accept 实测（只失败 **H2 的 tap 时代 380 Hz 速率门槛**，其余全绿）：

```
H1 rtf 1.0000        H3 28871 samples over 144.4 s (200.0 Hz)     H4 13 goals
H5 tap 10.21 m vs FCU 10.24 m                                     H6 cast p50 18.3 ms / skipped 5
W0b 100%  W1 13/13  W1b 13/13 (worst 0.08 m)  W2 p90 1.42  W3 -0.080 m
W4 +0.026 m CLEAR   W5 ok   W6 p50 0.178 / spread ok
```

修过的收尾顺序问题：`finalize_bag.sh` 必须在 rosbag2 记录器**收到 SIGINT 并写完 metadata.yaml 之后**运行
（第一版把它放在"停止"之前：`phase2l` 的 `derived/` 为空、`bag_compare` 报 no bag、H3 读不到 lio.csv）；
并且只允许执行一次（用 `$OUT/.bag_finalized` 文件标记，EXIT trap 会再次进入 stop_stack）。

**Phase 2 完成判据对照**：全量录制 ✓（bag 与发布率一致）；bag→CSV 派生 ✓（与现采逐字节一致）；
同趟新旧 accept 逐行一致 ✓（14/15 + 窗口对齐后 15/15，见上）；目标态（无 CSV 记录器）真实飞行 ✓（本趟）。

## 数据量策略（owner 指示 2026-11-25：减少录包，点云不要）

**点云不进 bag**（`/livox/lidar`、`/lio/cloud_world`）。实测：`/livox/lidar` 约 **197 kB/条**、`/lio/cloud_world`
约 **307 kB/条**、各 10 Hz ⇒ 每 150 s sortie 约 **0.7–1.0 GB**，是其余全部 topic 的 ~16 倍。CSV 派生层与
`accept.py` 的分析都不需要点云；需要重放时用确定性仿真（pattern + world）重建即可。
真正需要原始点云做调查时显式 `BAG_CLOUDS=1`（横幅会警告体积）。

**默认 topic 集合（现在 45–50 MB/趟）**

| 集合 | 内容 | 体积（phase2m，144 s） |
|---|---|---|
| `core`（默认） | `/lio/robo/odom`、`/sim/truth/odom`、`/plan/cmd`、`/fcu/odom` | A 24 MB + B 23 MB |
| `full`（`BAG_TOPICS=full`） | core + `/lio/odom`、`/livox/imu`、`/planning/pos_cmd`、`/planning_cmd/poly_traj`、`/sim/rtf`、`/rosout`（仍不含点云） | ~60 MB/趟 |
| `BAG_CLOUDS=1`（显式） | 上面 + 两路点云 | **~0.7–1.0 GB/趟** |

`/clock` 也不录：D4 的时间基就是真值客户端自己的 stamps，`/clock` 是可派生量。

