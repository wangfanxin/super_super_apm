# 数据管线总汇报（2026-11-25，暂停点）

> 依据 `docs/PIPELINE-PLAN.md` 推进；约束：**D1 不用 mavros / D2 不用 Gazebo / D3 真值 = 自建客户端订阅 MAVLink
> SIM_STATE(msg 108) / D4 wall 时钟 + bag 记录 + 显式映射 / D5 全部进仓库**；验收判据类改动（P0）按所有者要求延后。
> 本报告记录"暂停时的真实状态"，不粉饰未完成项。

## 1. 各阶段状态

| Phase | 状态 | 关键证据 |
|---|---|---|
| **0 仓库自包含** | ✅ 完成 | `.build/.runs` 进仓库(in-repo symlink→ext4)；从 `hku_super/`+`slam/` 重编全部；删除外部运行根与 `runtime.env`；真实飞行 `missionA-phase0l-01-01` 验证（15/15 行） `docs/PHASE0-VERIFY.md` |
| **1 真值换成自建客户端** | ✅ 完成 | SITL 换 unpatched 构建，tap 补丁+旧 `truth_rx.py` 归档 `sim/legacy/`；`sim/src/sim_state_recv.py` 订阅 msg 108 自建客户端；`scripts/compare_truth.py` 与旧 tap 对比（显式时钟映射 −0.748 s 后 W6 差 0.003/0.011 m）；验收飞行 `phase1y` 除 H2(P0)/W4(已知) 外全绿 `docs/PHASE1-TRUTH.md` |
| **2 bag 作录制底座** | ✅ 完成 | `BAG=1` + split writers + **tmpfs 先写**；`finalize_bag.sh` 拷回并派生 CSV；`missionA-phase2k` 同趟对比：派生层**包含现采每一行（逐字节相同）**，同趟新旧 accept 14/15 行一致、窗口对齐后 15/15；目标态无 CSV 记录器的飞行 `phase2m` 除 H2 外全绿 `docs/PHASE2-RECORDING.md`。按 owner 指示点云不进 bag（默认 45–50 MB/趟） |
| **3 bridge+supervisor 合并** | ✅ 功能目标达成 | `bridge_nodes/mavlink/fcu_client.py` 一个程序、两个连接、两个 node、两个 executor/线程（84/331 在 serial0 自己的线程；class-76/守卫在 serial1）；`FCU_CLIENT=merged` 已是**默认**；连飞 `phase4g/phase4h`：H1/H3/H4/H5/H6 + W0b/W1(13/13)/W1b/W2/W3/W5/W6 全过，与 legacy 基线行级一致；仅 H2(P0) 与 W4(已知 ±5 cm) 不同 `docs/PHASE3-MAVLINK.md` |
| **4 轻量 sim 修正** | 🟡 代码完成，默认待 A/B | 图案整窗均匀抽点 + 逐扫描前进（`--new-lidar`，默认仍 legacy）；`--scan-csv` 每帧 CSV 且 **H6 改读它**；rviz 单一回放入口 `sim/rviz/pipeline.rviz` + `scripts/viz_bridge.py` + `scripts/replay_bag_rviz.sh`；A/B 首轮：new 调度完整 13/13、H6 正常，但 **W6 spread 0.201/0.379 > 0.15**；20000 点在本机不可用（skipped 237）⇒ 保持 12000 点与 legacy 默认 `docs/PHASE4-SIM.md` |

## 2. 管线现状（默认路径）

```
SITL(unpatched) ⇄ fcu_client(merged: bridge 线程 serial0 84/331 + supervisor serial1 class-76/守卫)
     │ SIM_STATE(msg108, 自建客户端, 实时 monotonic 时间戳)
     ▼
sim_state_recv → /sim/state(200Hz) → lidar_imu(世界+图案) → /livox/lidar+imu(10/200Hz)
     ▼
Super-LIO → /lio/robo/odom(200Hz) → [fcu_client 331→EKF] + [planner]
     ▼
planner → /plan/cmd → fcu_client 84 → FCU
录制：rosbag2(split, tmpfs) → finalize → derived CSV → metrics/accept
```

* 时间戳规则（owner 2026-11-25 指令）：飞行路径**只用真实 `time.monotonic()`**，不用仿真时间、不用 FCU 时钟；
  ArduPilot 侧由 `JitterCorrection` 自行把任意 offboard 时基映射到本地。
* 录制体量：默认 core = `/lio/robo/odom /sim/truth/odom /plan/cmd /fcu/odom`，约 45–50 MB/趟；点云需 `BAG_CLOUDS=1`。

## 3. 最近一趟的验收数字（`missionA-phase4g/4h`，merged 默认路径）

| 行 | phase4g | phase4h | legacy 基线 phase2m |
|---|---|---|---|
| H1 rtf / 飞行窗 | 1.0000 / 130.41 s | 1.0000 / 128.41 s | 1.0000 / 130.52 s |
| H3 速率 | 200.0 Hz | 200.0 Hz | 200.0 Hz |
| H5 水平/垂直 span | 10.23 vs 10.26 | 10.21 vs 10.17 | 10.21 vs 10.24 |
| H6 cast p50 / skipped | 20.2 ms / 13 | 18.8 ms / 11 | 18.3 ms / 5 |
| W0b / W1 / W1b | 100% / 13/13 / 0.12 | 100% / 13/13 / 0.09 | 100% / 13/13 / 0.08 |
| W2 p50 / W3 bias | 1.27 / −0.088 | 1.27 / −0.085 | 1.25 / −0.080 |
| W4 最近间距 | +0.018 CLEAR | −0.047 CONTACT | +0.026 CLEAR |
| W6 p50 / p90 | 0.172 / 0.248 | 0.177 / 0.244 | 0.178 / 0.246 |
| H2（P0 判据） | FALSE | FALSE | FALSE |

## 4. 工程上真正修掉的问题（可复述给后来人）

1. **bag 写盘造成整机停顿**：ext4 虚拟盘上写 bag 会让 LiDAR raycast 从 15 ms 跳到 1.9–2.2 s、LIO 失锁；
   改为先写 tmpfs（`/dev/shm`），飞行后 `finalize_bag.sh` 拷回并派生 CSV。
2. **单 writer 抓不全**：飞行负载下单 rosbag2 writer 只有 ~35 Hz/200 Hz topic；split writers + 分核 + 先于发布者启动后满速率。
3. **真值链路**：自建 MAVLink 客户端（msg 108）；`--stamp-offset` 映射、实时时钟戳、`nav.csv` 每帧记录 331 内容。
4. **merge 客户端的两个杀手**：
   * `rclpy.spin_once` 用进程全局 executor，两个线程同时用会 `ValueError: generator already executing`；
   * supervisor 与 bridge 共享同一个 node 时，EKF 的 `XKF4.SH` 高度创新涨到 1–3 m、extnav 被拒、位置环发散。
     最终形态=每半身自己的 node + `SingleThreadedExecutor` + 连接 + 线程。
5. **supervisor 的等待绕过 pump**（`poll()+sleep`）会饿死 #331，#331 静默 >300 ms 时 FCU 报 `Arm: VisOdom: not healthy`；
   改为全程走 `pump`，并给 `_command`/`calibrate_accel_simple`/`set_mode` 加 `on_wait` 钩子。
   `inspect.stack()` 首次调用还要 import `inspect/linecache`（实测 0.48 s），正好卡在 arm 前——改用 `sys._getframe`。
6. **LiDAR 仿真两个 bug**：图案窗口只在启动装载一次且不前进；12000 点=前 60 ms 突发。已实现整窗均匀抽点+逐扫描前进，
   但仍待 A/B 再切默认。

## 5. 已知缺陷 / 风险（未关闭）

| 项 | 说明 |
|---|---|
| **H2** | 真值 200 Hz，判据仍要求 380 Hz（tap 时代）；属 P0 判据类，等所有者批准后重定义 |
| **W4** | 真值→柱面最近距离在 ±5 cm 量级抖动（+0.018 … −0.096），多趟出现 CONTACT；已知交付缺陷，未修 |
| **主机停顿** | 偶发（~1–3 s）会让 lidar/LIO 停止发布，supervisor 正确以 `ESTIMATOR FEED IS STALE` 中止；非代码缺陷但影响成图率 |
| **Phase 4 new 调度** | 两趟 W6 spread 超界（0.379/0.201 vs 0.15）；需再两对 A/B 并核对 LIO 的时间参数（`livox_360.yaml`） |
| **extnav 门限** | 目前是 age 记录 + nav-gate 拒绝计数，尚未做成显式可配置的年龄/质量/reset 门限（Phase 3 遗留小项） |
| **legacy 路径** | 双进程 bridge+supervisor 仍保留（`FCU_CLIENT=legacy`），未删除——按计划需连飞达标后再清理 |

## 6. 回滚点

* `FCU_CLIENT=legacy`：回到历史双进程 FCU 接口；`bridge_only/bridge_thread/supervisor_only` 为诊断模式。
* `SIM_LIDAR_LEGACY=1` / `LIDAR_SCHEDULER=legacy`：回到旧图案调度（当前默认）。
* tap 补丁与旧 `truth_rx.py`：`sim/legacy/`（含 README 与打/回补丁命令）。
* 录制：`BAG=1` 用 bag 派生；不传 `BAG` 仍走实时 CSV 记录器（过渡期两条都保留）。

## 7. 下一步建议（恢复推进时）

1. **Phase 4 A/B 收尾**（优先）：安静主机上再跑 2 对 legacy vs new（固定 12000 点），确认 W6 差异是否稳定；
   若稳定，方案二选一：(a) 保持 60 ms 窗口但在窗内均匀抽点；(b) 先在 LIO 侧对齐时间参数再切默认。
2. 把 extnav 年龄/质量门限做成显式参数并写进 docs（Phase 3 清单剩余项）。
3. legacy 双进程路径清理（连飞 2 趟通过后）。
4. 可选 Phase 5：accept 的 manifest 完整性检查、evo APE/RPE 附加行、bag→derive→accept 回放 CI。
5. 验收判据（P0）批次：H2/W2/W6/H3/H6/W1b/W4 的语义与阈值，需要所有者批准后统一改。
