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

## 2026-11-25 03:45（第 5 条，Phase 1 真值链路打通 + 时钟/一致度实测）

**打通的部分**
- `sim/src/sim_state_recv.py` 在真实 sortie 中记录 SIM_STATE 成功：自适应退避后稳定 **~328 Hz**（5.1 万行/趟）。
  ArduPilot 对超速请求回 `MAV_RESULT_DENIED`（"too fast. Increase SCHED_LOOP_RATE"），100 Hz 有时被拒、
  120/200 基本被拒；记录器现在遇 DENIED 自动 ×0.8 退避（下限 50 Hz），所以请求 1000 Hz 也能落到可接受速率。
- `scripts/compare_truth.py` 升级：
  * 时钟映射改为**位置残差最小化**（旧 FFT 事件拟合一趟偏 2%：1.0193 vs 真值 0.9998）；
  * `--window FROM:TO` 支持只在正常飞行段评估；
  * 采样时刻重建（按段线性拟合 arrival->index），减少 TCP 成批到达的时间抖动；
  * `--selftest` PASS（恢复 a=0.999500/b=3.250、位置残差 0、速度 1e-4 m/s）。

**实测结果（phase1f-01，窗口 0–30 s）**
```
clock   t_sim = 0.99960*t_wall + 0.996   (FFT 交叉对照 a=1.003)
地理    pos rms 11.3 mm (p99 52.9 mm)
速度    rms 0.7 mm/s     姿态 rms 0.012 deg
速率    899.7 Hz (p99 gap 6.9 ms)
```
=> 低速段 SIM_STATE 可替代 tap（mm 级）。**但飞行中段（20–50 s）位置残差 0.16–0.29 m**，源自 TCP 成批到达
（到达时间≠采样时刻）与一次 ~1 s 采样空洞（t=18.45，1035 ms），高速段还不能满足 20 mm 判据。
另一发现：SIM_STATE 的垂直速度在机动段与 tap 差 ~0.1–0.2 m/s（水平分量 18 mm/s 内），需要在换真值前定标。

**飞行稳定性仍是 Flaky**：phase1b/1c/1e/1f 都在 40–150 s 内因 EKF estimator lost / altitude guard 中止；
phase1a/1d 传感器 H6 也偶发（cast 尖峰）。每趟都能拿到 pre-divergence 段数据，但完整 2 圈通过仍需要更稳的主机。

**下一步**
1. 处理 1 s 采样空洞与时间抖动：给 SIM_STATE 批次重建时间戳（按 interval 3 ms 重建 + 空洞分段），
   并在记录器里记录每个样本的到达顺序/间隔统计；
2. 定标 vd 通道（对照 tap vd 与 SIM_STATE alt 差分）；
3. 复飞拿到完整 2 圈 + 双录，跑 compare_truth 全窗口；达标后按计划移除 tap（Phase 1b），
   再做 Phase 2 同趟 bag 逐行一致。

---

## 2026-11-25 04:00（第 6 条，Phase 2 bag 底座已能在真实 sortie 落盘；飞行稳定性仍是最外层阻塞）

**Phase 2（真实 sortie 验证的进展）**
- 用 `BAG=1` 飞了一趟 `missionA-phase2a-01-01`：bag 成功落盘（13 topics、5390 条消息、74 MB sqlite3、99 s，
  `/rosout` 也录到），`scripts/bag_to_csv.py --derive-all` 能从这个**真实 bag** 反演出 `lio.csv` /
  `traj.csv.cmd.csv`（表头与旧记录器一致）。
- 但这趟飞行本身崩了：`raycast p50 106 ms`（正常 10 ms）、cloud 3 Hz、LIO 只出 5.8 Hz、EKF/高度守卫中止。
  bag 记录器与 4 个 CSV 记录器 + bridge + supervisor 共 8 个进程挤在 AUX 的 4 个核上，写作 74 MB 的同时把
  仿真饿死。**结论：bag 作为"录制底座"可行，但必须给它独立的核/或改用 CSV+bags 二选一的负载策略**；
  下轮在 BAG=1 时把 CSV 记录器停掉（派生层可以从 bag 生成），这本来也是 Phase 2 的目标态。

**当前阻塞（诚实记录）**
- 主机/调度仍不稳：同一套臂在 03:00–04:00 之间 8 趟只有 1 趟完整飞完（phase0l）；其余在 40–150 s 因
  estimator lost / altitude guard 中止，且 LiDAR cast p50 在 10 ms 与 100–150 ms 之间跳变。
- 因此 Phase 1 的"全窗口 mm 级一致"与 Phase 2 的"同趟 bag↔CSV 逐行一致"都还缺一趟**完整 2 圈**的飞行。

**下一步（第 25 轮）**
1. BAG=1 时停掉 CSV 记录器（bag 成为唯一底座），bag 记录器独占一核；再复飞；
2. 若飞行仍 flaky：把 `SIM_STATE` 记录器与 bag 同理降载，并做一次"只飞必要节点"的验收模式
   （bridge+supervisor+planner+sensor+LIO，不跑任何记录器）先证明飞行本身能稳定 2 圈；
3. 之后再做 Phase 1 全窗口 compare 与 Phase 2 bag↔CSV 一致性。

---

## 2026-11-25 04:35（第 7 条，找到飞行不稳的主因：主机空闲低功耗态；bag 抓取率仍是 Phase 2 的卡点）

**主因（实飞证据）**
- 同一套臂、同一世界：空闲主机上飞 → H6 失败（LiDAR cast p50 100+ ms、丢扫描）、EKF/高度守卫中止；
  **先让 20 个 vCPU 忙 45 s 再飞**（`scripts/warmup.sh`，已接进 `fly_hex2.sh`，`WARMUP=0` 可关）→
  `missionA-warm1-01-01`：H1 rtf 0.9998、H6 cast p50 15.0 ms / MAX 23.5 ms / skipped 0、W1 13/13、W1b 13/13、
  H2/H3/H4/H5/W2/W3/W5 全过（只有 W4 已知缺陷、W6 max 1.543 m 略超）。
- 结论：傍晚那些"好飞行"都紧跟在长时间编译之后（主机高 p-state）；空闲时 SITL/sensor 线程唤醒延迟把 10 Hz
  扫描的 100 ms 预算打穿。**这解释了整轮反复失败的根因。**

**Phase 2 bag 抓取率（新卡点，已有数据）**
- `phase2f`（预热 + 只有 bag 在录、SIM_STATE=0）：飞行中 `/lio/robo/odom` 现采 200 Hz，而 bag 只抓到
  **~40 Hz**（7240/178 s）；`phase2b/2c/2d` 同样 ~16–45 Hz。隔离环境（无飞行负载）bag 能满速抓 185 Hz，
  所以不是脚本/QoS 配置错误，而是**整栈运行时 bag 记录器被饿/丢包**。
- 已得到的正面结论：bag→CSV 派生**忠实**——`phase2b` 中 bag 派生的 3163 条 lio.csv 行与现采记录器**逐字节相同**；
  12/15 条 accept 行与现采一致（H3 因 bag 抓取少而不同，W1b/W4 因我手动跑 accept 时没带 WBOX/BODY/LAPS 参数）。

**下一步（下轮优先）**
1. 定位 bag ~40 Hz 的丢包：给 bag 记录器单独写计数/诊断（发布侧计数对比），试 `nice`/独占核/换 RMW 或
   `--storage-config-file` 提高 sqlite 批量；必要时改用"只在起飞后录制"或分 topic 两个 bag。
2. bag 抓取恢复后：预热 + `BAG=1 SIM_STATE=0` 复飞 → accept(bag 派生) vs accept(现采) 逐行对比（带正确 WBOX/BODY/LAPS）；
3. Phase 1 的飞行中段时戳抖动仍待解决（SIM_STATE 无时间戳 + TCP 成批到达）；预热后应能拿到更干净的整趟数据。

---

## 2026-11-25 04:55（第 8 条，预热后拿到干净整趟；Phase 1 位置通道定位到 SITL 平滑/移动原点）

**干净整趟（关键进展）**
- `LIGHT=1`（跳过 cloud/traj CSV 记录器）+ 预热 + SIM_STATE=1：`missionA-phase1h-01-01`
  **H1–H6 全过**（H1 rtf 0.9998、H6 cast p50 15 ms / skipped 0）、W1 13/13、W1b 13/13、W2/W3/W5/W6 过；
  只有 W4 已知缺陷（-0.05 m 接触）。=> 预热 + 减负是把飞行从"不可复现"拉回来的正确做法。
- 启动器新增 `LIGHT=1`：BAG=1 或 LIGHT=1 时跳过 record-cloud / record-traj（accept 不需要，减少订阅者负载）。

**Phase 1 双录对比（同一趟）**
- window 0–20 s（地面）：rate 197 Hz、gaps 0、clock/位置/速度/姿态 **全 PASS**（pos rms 0.000）。
- window 30–50 s（巡航）：rate 211 Hz、gaps 0、clock PASS、vel 8.6 mm/s、att 0.42°，但 **pos rms 0.346 m FAIL**。
- 源码定位：`SIM_Aircraft::fill_fdm` 在 `use_smoothing` 时把经纬高/速度/姿态换成 `smoothing.*`（一阶平滑，
  τ=0.1 s），且 `smoothing.location = origin + smoothing.position` 而 SITL 的 origin **1 Hz 移动**；tap 用
  `get_position_relhome()` 规避了这一点。位置滞后假设（τ 扫描）被否（最优 τ=0）。
  => **速度/姿态/速率可用于真值；位置通道还不能替代 tap**，Phase 1b 前要解决（积分重建 / 混合源 / 继续深挖）。

**下一步**
1. 用 SIM_STATE 速度积分重建位置并评估 150 s 内漂移（候选 a）；
2. 查 `use_smoothing` 在腾空后是否会复位（若不复位，unpatched SITL 的 SIM_STATE 位置天生是平滑派生量）；
3. 预热 + LIGHT + BAG=1 复飞，做 Phase 2 的 accept(bag 派生) vs accept(现采) 逐行对比（带 WBOX=0.5/BODY=box/LAPS=2）；
4. bag 抓取率低（~40 Hz vs 200 Hz）的问题仍未解决——优先给它单独诊断/换策略。

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
