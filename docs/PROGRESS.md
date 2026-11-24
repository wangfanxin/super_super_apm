# 阶段总结（每小时一条）

规则（所有者 2026-11-25 指示）：执行数据管线目标期间，**每过一小时写一条阶段总结**，汇报进度与方向；
每条包含：时间、阶段、已完成、进行中、阻塞/风险、下一步。条目追加在下面（新的在上）。

**02:35 更新（第 2h 末）**：验证飞行仍然失败，且**旧运行根对照组同样失败**（ctrl1/ctrl2 失败、ctrl3 通过），
说明不是自包含改造引入。逐次观测到主机侧负载越来越重：LiDAR cast p50 从傍晚 ~11 ms → 19 ms → 33.6 ms，
MAX 148–260 ms，丢扫描 24–31 个，Super-LIO 在 leg 8–12 被 EKF 判为发散而中止。SITL 单独跑（不加载栈）仍是满速，
所以是"栈 + 主机调度"的临界。Windows 电源方案是 Balanced、插电 100%。**下一步**：先让主机回到平静/高性能状态
再复飞（或降低仿真负载：LIDAR_POINTS 12000→8000 做一次对照，记录对 H6/W 行的影响）；同时排查
SIM_STATE 在飞行中收不到样本的问题（独立测试 200 Hz 正常，飞行中 0 样本，记录器已确认连上并收到 heartbeat）。

---

## 2026-11-25 02:10（第 2 条，Phase 0 结构完成，验证飞行受阻）

**阶段**：Phase 0 的结构部分完成；Phase 1a 工具完成；**验证飞行还没通过**，原因是飞行中段的估计器发散（旧运行根同样复现），不是自包含改造引入的。

**已完成（本轮）**
- 仓库改为"源码全在仓库、构建/记录在 ext4、仓库内用符号链接指过去"：
  `.build/{ws_super,lio_ws,ardupilot}` → `/home/fanxin/sp_build/...`，`.runs` → `/home/fanxin/sp_runs`。
  原因（实测）：ROS 产物放在 `/mnt/d` 上，fsm_node/lib 走 9p 缺页，super_lio 启动 ~30 s、SITL rtf 掉到 0.18–0.67；
  放到 ext4 后节点启动恢复正常。`setup/build.sh` 增加 livox 必需的 `-DDISTRO_ROS=humble -DROS_EDITION=ROS2`。
- 两个工作区都**从仓库源码重建成功**：fsm_node 21,744,912 B（hku_super）、super_lio_node 52,496 B（slam）。
- `scripts/write_manifest.py`：每趟 `manifest.json` 记录 git HEAD/脏文件数、fsm_node sha256、配置三份哈希、
  world/calib/parm 哈希、ArduPilot rev、解析后的路径、argv。已在实际 sortie 产物中验证。
- `sim/src/sim_state_recv.py` + `scripts/compare_truth.py`（Phase 1a）：自建客户端订阅 MAVLink SIM_STATE；
  compare 工具可拟合 `t_sim=a·t_wall+b`（a 即 rtf）与 lat/lon→NED 仿射，`--selftest` PASS。
- 启动器修复：banner 不再对复制的 ArduPilot 跑 `git diff --stat`（drvfs 上 >60 s）；
  失效锁按"pid 是否仍是 run-mission"判断；`scripts/archive/harness/bin-triage.py` 路径修正；
  辅助节点固定到 12–19 核（SITL 0–3、传感器 4–7、LIO 8–11），避免 supervisor 的 1 ms 循环抢 SITL 核。

**实测事实（重要）**
- 未打补丁的 SITL 可提供 SIM_STATE，但**没有仿真时间戳、没有局部 NED 位置**（只有 lat/lon/alt + 速度/姿态/加速度）；
  请求 200 Hz 实得 ~225 Hz。⇒ 真值切 SIM_STATE 必须同时把时钟基准改为 wall + 显式映射（与 D4 一致）。
- 用旧运行根（完全 ext4、与 19:56 通过的同一套）做对照：**3 趟里 1 趟全过（15/15）、2 趟在 leg8–9 因 EKF variance/
  estimator lost 中止** ⇒ 中段发散是**既有的间歇性飞行稳定性问题**（表现为 LiDAR cast 偶发 190–260 ms、丢扫描，
  LIO 随后偏离），不是仓库自包含改造造成的；自包含改造后的失败与之一致。

**02:20 更新**：SIM_STATE 记录器已打通——ArduPilot 拒绝 1000 Hz 请求，**200 Hz 被接受**（实测 799 样本/4 s =
199.7 Hz，字段完整、lat/lon/alt/加速度合理）；启动器已改为默认 200 Hz，`--home=` 形式修正。待一次双录飞行后跑
`compare_truth.py` 得到 `t_sim=a·t_wall+b` 与一致性判据。

**下一步**
1. 把"中段发散"先定性/定量：统计旧运行根连续多趟的失败率，抓一次失败前的 lidar cast/scan skip 时间线；
2. 若确认为主机调度导致的 cast 尖峰，考虑把扫描预算留足（降 LIDAR_POINTS 或把 cast 放到独立核/降频）并记录；
3. 拿到一趟仓库内构建的干净 15/15 后，删旧运行根/`runtime.env`/`lio_avoid` 壳，提交 Phase 0 完成；
4. 然后做 Phase 1 双录对比（`SIM_STATE=1` 已在飞臂默认打开，但 `sim_state.csv` 还没成功落盘——
   已定位 `--home` 参数以 `-` 开头被 argparse 当选项，需要 `--home=...` 形式，下轮修）。

**提交**：本轮改动（manifest、SIM_STATE 接收器、compare_truth、构建/启动修复、文档）已 commit+push。

---

## 2026-11-25 02:45（第 3 条，Phase 2 录制底座完成；飞行稳定性仍是唯一阻塞）

**已完成（本轮）**
- **Phase 2 录制底座**：`scripts/record_bag.sh`（rosbag2，13 个 topic，QoS 覆盖 YAML 解决 BEST_EFFORT 收不到的问题）
  + `scripts/bag_to_csv.py`（从 bag 派生与旧记录器**逐列一致**的 lio.csv / traj.csv.cmd.csv）
  + 启动器 `BAG=1` 开关注入与优雅停止（SIGINT + 10 s 等待）。
- **合成端到端测试 PASS**（`_scratch/test_bag_pipeline.py`）：发布 242 条 Odometry + 247 条 PositionCommand →
  录制 → 派生，两个 CSV 表头与旧记录器完全一致、数值关系（y=2x、qw=1、vx=1、yaw_dot=0.1）逐行核对通过。
- 实测坑：rosbag2 的 zstd 压缩包用 `rosbag2_py.SequentialReader` 打不开（"file is not a database"，CLI 能读）；
  录制器因此默认**不压缩**，待 reader 路径验证后再加开关。

**飞行稳定性（仍未解决，已排除的假设）**
- SITL 本身**不慢**：失败趟的 tap 实测 `span/wall = 1.000`；H1 的 "rtf 0.39–0.64" 是 **accept 的窗口混入着陆后死时间**
  （FCU 在 LAND 后 45 s 不 disarm，supervisor 等到超时才写 done，窗口里 sim 时间冻结）——属判据/窗口问题（P0，延后）。
- 真实失败链：LiDAR 仿真吞吐下降（cloud 6.7 Hz、IMU 134 Hz 而名义 10/200）→ LIO 在 leg 8–12 发散 → EKF failsafe/abort；
  旧运行根对照组同样复现（3 趟 1 过 2 败），因此**不是自包含改造引入**。
- 已加"辅助节点固定 12–19 核"（SITL 0–3 / 传感器 4–7 / LIO 8–11），未根本解决。

**下一步**
1. 传感器吞吐：查 `lidar_imu` 为何只有 6.7 Hz / 134 Hz（cast 本身 19 ms、flat 路径独立实测 8–13 ms，说明是**调度/内存带宽**
   下的退化）；试单核隔离（`SENSOR_CORES=4`）+ 提高 `chrt` 优先级 + 关闭非必要 recorder 做 A/B。
2. SIM_STATE 飞行中 0 样本：独立 SITL + 参数复现为 128.7 Hz 正常，说明是**飞行启动时序/连接**问题；下一趟加 ACK 与
   连接日志后定位（或改为在 LIO 起后延迟 10 s 再启动 recorder）。
3. 拿到一趟 15/15 后完成 Phase 0 收尾（删旧运行根/runtime.env/壳），再做 Phase 1 双录 `compare_truth`。

---

## 2026-11-25 03:10（第 4 条，**Phase 0 完成**；调度隔离找到）

**Phase 0 完成（实飞证据）**
- 仓库内源码构建的工作区飞出**完整两圈六边形**：`/home/fanxin/sp_runs/missionA-phase0l-01-01`。
- accept 实测：H2/H3/H4/H5/H6 + W1（13/13）/W1b（13/13，worst 0.15 m）/W2/W3/W5/W6 全过；
  H1 rtf 0.9781（窗口把着陆后等待算进去的判据假象，P0 延后）；W4 -0.047 m 接触（已知缺陷）。
- manifest 记录：repo HEAD `c722bac`、fsm_node sha256 `82593a54…`、`liblio.so` `b49aa6a1…`、
  world/calib/parm 哈希、ArduPilot `dbe7921`；路径全部解析到 `.build/*`、`.runs`。
- **删除外部运行根**：`/home/fanxin/super_super_apm`（7.5 GB）与 `/home/fanxin/lio_avoid` 壳已删；86 趟历史记录
  归档到 `/home/fanxin/sp_archive/oldroot_runs`（同盘 mv，未丢）；`setup/runtime.env` 已不存在。
  删除后 `tests/run_all.sh` 仍 PASS，`env -u DSH_ROOT` 下所有路径解析正常。
- 文档：[docs/PHASE0-VERIFY.md](PHASE0-VERIFY.md)（证据 + D5 符号链接边界 + 调度依据）；PIPELINE-PLAN Phase 0 标记完成。

**调度隔离（本轮真正的修复）**
- 症状：飞行中 LiDAR 仿真掉到 cloud 6.7 Hz / IMU 134 Hz（名义 10/200），LIO 在 leg 8–12 发散。
- 最小栈实测（SITL+truth_rx+传感器+LIO，无规划器/桥/记录器）：cloud 9.95 Hz / IMU 198.5 Hz / cast 10.6 ms → 主机与
  仿真代码本身没问题。
- 分阶段加负载：fsm 空跑、bridge+supervisor、记录器分别都不掉速；**规划器真正规划时**才掉速。
  把规划器与 I/O 节点拆到各自核（`PLANNER_CORES=12-15`、`AUX_CORES=16-19`，SITL 0-3、传感器 4-7、LIO 8-11）后，
  验证趟 H6 恢复（cast p50 16.4 ms、skipped 5）。
- 另发现：`SIM_STATE=1`（200 Hz 记录器）那几趟更容易失败；Phase 1 需要把它放到独立核/降速率验证（见下一步）。

**下一步（Phase 1）**
1. 用 `SIM_STATE_HZ=100` 并把它挪到单独核跑一趟双录，确认 `sim_state.csv` 有数据且飞行不受影响；
2. 跑 `compare_truth.py --tap … --sim-state …`，得到 `t_sim=a·t_wall+b` 与位置/速度/姿态一致性判据；
3. 达标后按计划把 tap 从飞行路径移除（Phase 1b），再进 Phase 2 的同趟 bag 逐行一致验证。

---

## 2026-11-25 00:25（第 1 条，Phase 0 进行中，约 1h20m）

**阶段**：Phase 0（仓库自包含）——3/4 完成；Phase 1 代码已就绪。

**已完成**
- D1–D5 决策并入 `docs/PIPELINE-PLAN.md`（提交 `b931e59`，已推送）。
- `setup/runtime.env` 已挪走（备份 `/tmp/runtime.env.bak`），路径解析改到仓库内 `.build`。
- ArduPilot SITL（1.5 GB）已拷入 `<repo>/.build/ardupilot`；80 趟保留记录（3.3 GB）已拷入 `<repo>/.runs`。
- **fsm_node 首次从仓库源码 `hku_super/` 编出**：21,744,912 B（依赖包 mars_quadrotor_msgs/rog_map/mission_planner 7m17s，super_planner 10m59s）。
- Phase 1a 代码：`sim/src/sim_state_recv.py`（自建客户端订阅 SIM_STATE）、`scripts/compare_truth.py`（时钟映射/测地拟合/一致性，`--selftest` PASS）、启动器接线（`SIM_STATE=1` 双录）、`scripts/write_manifest.py`（每趟 manifest）。
- 文档：`docs/PHASE1-TRUTH.md`（实测：未打补丁 SITL 200 Hz 请求→224.8 Hz；SIM_STATE 无仿真时间戳、无局部 NED 位置）+ 文档索引。

**进行中**
- `lio_ws` 编译：`livox_ros_driver2` 已编过（需要 `-DDISTRO_ROS=humble -DROS_EDITION=ROS2`，已写回 `setup/build.sh`），`super_lio` 配置/编译中。

**阻塞与风险（已定位）**
- `/mnt/d`（drvfs/9p）上 CMake/big-tree 扫描极慢（曾 9 分钟 configure 仅 32 s CPU，进程 D 态）。对策：构建目录放 ext4 `/tmp`，install 目录留在仓库内（产物自包含、构建可复现）。
- 这带来一个待验证项：**运行时从 `/mnt/d` 读取 ROS install 树**可能拖慢节点启动；飞一趟量 rtf 后给结论。
- `runtime.env` 已停用、旧运行根仍保留（7.1 GB，回滚点）；在仓库内构建飞过之前不删。

**下一步**
1. 等 `super_lio` 编完 → 跑 `tests/run_all.sh` + `install_config_hex.sh`（三份 md5 一致）。
2. 用仓库内构建飞一趟（`SIM_STATE=1` 双录），检查 rtf 与 manifest。
3. 通过后删旧运行根/`runtime.env`/`lio_avoid` 壳，提交 Phase 0；随后跑 `compare_truth.py` 开始 Phase 1 判据。
