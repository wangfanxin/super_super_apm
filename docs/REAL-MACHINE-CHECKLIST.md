# 真机迁移检查表（从这台仿真栈搬到真实 MID-360 + ArduPilot 飞机）

> 本文是"仿真验收通过之后、第一次真机飞行之前"的检查清单。仿真侧的证据见 `docs/PIPELINE-PLAN.md` /
> `docs/PHASE3-MAVLINK.md` / `docs/EXTERNAL-NAV-ODOMETRY.md`。**没有一项需要改代码**：仿真里跑的东西
> 在真机上替换的是"数据来源"，接口与判据不变。

## 1. 仿真与真机分别负责什么（先分清"哪些是仿真-only"）

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

## 2. 真机侧必须替换/确认的接口

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

## 3. 上电→首飞的检查顺序（真机）

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

## 4. 与仿真 A/B 的对照（首次真机后要回答的问题）

| 问题 | 仿真基线（供对照） |
|---|---|
| LIO 相对真值 APE | `phase4p`：rmse 0.027 m、RPE(1s) 0.015 m（`scripts/traj_error.py`） |
| 跟踪滞后 | W6：稳态 lag 中位数 ~−0.15 s，lag 补偿后 p90 ~0.25 m |
| 巡航速度 | p90 1.4–1.6 m/s（`max_vel 1.5`）、高度 2.0 m |
| 扫描/IMU 预算 | H6：cast p50 12–14 ms、skipped 5–10；IMU 200 Hz |
| 喂数速率 | 331：48 Hz（实测 46.8–47），extnav 健康阈值 300 ms |
| 航点精度 | W1b worst 0.09–0.16 m（0.5 m 门限） |
| 柱面间距 | W4 已知缺陷 −0.02…−0.10 m（P0 提案容差 −0.05 m） |

## 5. 真机唯一"没有对应物"的东西

- **真值/验收 15 行**：真机没有 SIM_STATE；建议真机评测用 RTK/动捕（外部真值）或只做在线健康
  （EKF flags、nav_age、relay、bag 回放）+ 场地标记点的到达精度。
- **世界 YAML 与几何判据**（W4/W5）：真机需要实测场地地图（点云→地图），否则这两行无意义。
- **`tap.csv` / `scans.csv` / `compare_truth`**：仿真专属记录，真机录制只保留 bag 与派生层。
