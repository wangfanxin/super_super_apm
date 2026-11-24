# Phase 0 验证记录（仓库自包含）

**日期**：2026-11-25
**结论**：用仓库内源码重建的工作区（`hku_super/` → fsm_node、`slam/` → super_lio）跑了一趟**完整真实飞行**：
两圈六边形、13/13 航点按要求到达、结束时正常 LAND+disarm。所有"数据管线经得起检查"的硬行（H2/H3/H4/H5/H6、
W1/W1b/W2/W3/W5/W6）都是实测通过；只有 H1（窗口把着陆后的等待算进去，0.9781 对 0.98 门槛）和 W4
（已知缺陷：-0.047 m 接触）两行不绿，且都不是自包含改造引入的。

| 项 | 值 |
|---|---|
| 验证 sortie | `/home/fanxin/sp_runs/missionA-phase0l-01-01` |
| 仓库 HEAD（manifest 记录） | `c722bac` + 本轮修改（脏文件数见 manifest） |
| fsm_node | `.build/ws_super/install/super_planner/lib/super_planner/fsm_node`，sha256 `82593a54…` |
| Super-LIO | `slam/` 源码构建，`liblio.so` `b49aa6a1…`；`super_lio_node` 52,496 B |
| ArduPilot | `.build/ardupilot` → `/home/fanxin/sp_build/ardupilot`，rev `dbe7921` |
| world | `sim/worlds/forest_hex12.yaml`，sha256 `d30918ef…`（冻结未改） |
| calib | `calib/calib.json`，sha256 `5509ccfe…` |
| 记录目录 | `/home/fanxin/sp_runs`（仓库 `.runs` → 符号链接） |

## 实飞判据（accept.json，两圈六边形）

```
H1   flight window: sim 136.68 s / wall 139.74 s -> rtf 0.9781   (门槛 0.98-1.02，差 0.2%)
H2   rate median 1200.5 Hz, max gap 0.0008 s, 0 gaps > 10 ms                          PASS
H3   LIO 31301 samples over 156.4 s (200.1 Hz)                                        PASS
H4   SUPER ref count 12997, 13 goals issued                                           PASS
H5   tap span 10.67 m vs FCU 10.61 m (tolerance 5.31 m)                               PASS
H6   raycast p50 16.4 ms, MAX 74.7 ms, scans skipped 5                                PASS
W0b  100.0 % of route inside 2.00 +- 1.50 m    truth U min 1.00 p50 1.95 max 2.04     PASS
W1   13 of 13 waypoints reached                                                       PASS
W1b  13 of 13 approached within 0.50 m, worst 0.15 m                                  PASS
W2   |v_xy| p50 1.23 p90 1.50 p95 1.59 max 3.07 m/s (cruise p90 >= 1.40)              PASS
W3   bias p50 -0.048 m (bound +-0.30), excursion p95 -0.016 m (bound +-0.80)          PASS
W5   route p50 0.244 p90 1.248 p99 8.645 max 8.653 m                                  PASS
W6   ZERO lag p50 0.213 p90 0.334 max 1.000 m, quarter spread 0.115 s                 PASS
W4   min truth-to-pillar-surface distance -0.047 m (CONTACT, 0.39 % in body)          FAIL -- 已知缺陷
```

## 两行不绿为什么不算自包含失败

1. **H1**：`rtf` 窗口取 take-off..**done**。本趟 done 比 land 晚 3–6 s（landed 之后 supervisor 还要等
   disarm 状态确认），窗口里夹了一小段仿真已停止的时间，把 0.9781 压低；同一趟的 tap 窗口实测比值为 1.0。
   这是**判据/窗口定义问题**（P0 类改动，所有者要求暂缓），不是构建/路径问题。
2. **W4**：`-0.047 m` 的接触和本轮改造无关（同一条臂在旧运行根上也出现），属交付已知缺陷，验收时按
   `--known-defect` 记录。

## D5 的自包含边界（必须知道的事实）

"全部进仓库"在本机的边界是：**源码、配置、文档、启动器全在仓库**；**编译产物/运行目录物理上在 ext4**，
由仓库内的符号链接指过去：

```
.build/ws_super   -> /home/fanxin/sp_build/ws_super     (从 hku_super/ 构建)
.build/lio_ws     -> /home/fanxin/sp_build/lio_ws       (从 slam/ 构建)
.build/ardupilot  -> /home/fanxin/sp_build/ardupilot    (SITL 二进制；源码在 fcu_interface/)
.runs             -> /home/fanxin/sp_runs                (每趟记录)
```

原因（实测）：`/mnt/d` 是 drvfs/9p，ROS 产物放上面时 fsm_node/liblio 缺页导致节点启动 ~30 s、SITL 在飞行中
rtf 掉到 0.18–0.67；CMake 在 9p 上配置还会卡进 D 状态。放 ext4 后节点启动恢复正常，验证飞行才可能通过。
这些符号链接已加入 `.gitignore`（`/.build/`、`.runs` 等），克隆后由 `setup/build.sh` 重建。

## 启动器在本轮加的调度隔离（验证飞行能通过的前提）

SITL / 传感器 / LIO / 规划器 / MAVLink 节点各自独占核，避免互相饿死（本机 20 vCPU = 10 物理核 × 2）：

```
SITL        SIM_CORES     = 0-3
传感器      SENSOR_CORES  = 4-7      (lidar_imu)
Super-LIO   LIO_CORES     = 8-11
规划器      PLANNER_CORES = 12-15    (thin layer + fsm_node；fsm 继承父进程亲和性)
I/O 节点    AUX_CORES     = 16-19    (bridge、supervisor、记录器、SIM_STATE)
```

**测量依据**：不加隔离时，飞行中 lidar 仿真掉到 cloud 6.7 Hz / IMU 134 Hz（名义 10/200），LIO 在 leg 8–12
发散；最小栈（SITL+truth_rx+传感器+LIO，无规划器/桥/记录器）实测 cloud 9.95 Hz / IMU 198.5 Hz / cast 10.6 ms。
把规划器和 I/O 节点分到独立核后，验证趟 H6 恢复（cast p50 16.4 ms、skipped 5）。

## 外部运行根的删除

验证通过后，旧运行根 `/home/fanxin/super_super_apm`（7.5 GB，含旧 lio_ws/ws_super/ardupilot/.runs）
与 `setup/runtime.env` 一并删除；仓库启动器的所有默认路径都指向 `.build/*` 与 `.runs`，`tests/run_all.sh`
的 self-containment 检查会确认没有文件引用旧根。
