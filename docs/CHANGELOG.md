# CHANGELOG —— 变更与进展（合并自 PROGRESS.md / README-integration-history.md /
# harness-history/README.md）

本文件按时间倒序记录每一轮的**改动 + 测量数字 + 否掉的方案**。INTEGRATION.md（112 节、9000+ 行）与
harness-history 六个文件不再保留独立文档：完整原文在 git 历史里（`git log -- docs/INTEGRATION.md`
等），这里只保留仍影响当前行为的结论。



---

## 进展记录（PROGRESS.md，最新在前）

> 原文：`docs/PROGRESS.md`（已并入本文件，独立文件删除）

## 阶段总结（每小时一条）

规则（所有者 2026-11-25 指示）：执行数据管线目标期间，**每过一小时写一条阶段总结**，汇报进度与方向；
每条包含：时间、阶段、已完成、进行中、阻塞/风险、下一步。条目追加在下面（新的在上）。

**02:35 更新（第 2h 末）**：验证飞行仍然失败，且**旧运行根对照组同样失败**（ctrl1/ctrl2 失败、ctrl3 通过），
说明不是自包含改造引入。逐次观测到主机侧负载越来越重：LiDAR cast p50 从傍晚 ~11 ms → 19 ms → 33.6 ms，
MAX 148–260 ms，丢扫描 24–31 个，Super-LIO 在 leg 8–12 被 EKF 判为发散而中止。SITL 单独跑（不加载栈）仍是满速，
所以是"栈 + 主机调度"的临界。Windows 电源方案是 Balanced、插电 100%。**下一步**：先让主机回到平静/高性能状态
再复飞（或降低仿真负载：LIDAR_POINTS 12000→8000 做一次对照，记录对 H6/W 行的影响）；同时排查
SIM_STATE 在飞行中收不到样本的问题（独立测试 200 Hz 正常，飞行中 0 样本，记录器已确认连上并收到 heartbeat）。

---

### 2026-11-25 02:10（第 2 条，Phase 0 结构完成，验证飞行受阻）

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

### 2026-11-25 02:45（第 3 条，Phase 2 录制底座完成；飞行稳定性仍是唯一阻塞）

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

### 2026-11-25 03:10（第 4 条，**Phase 0 完成**；调度隔离找到）

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

### 2026-11-25 03:45（第 5 条，Phase 1 真值链路打通 + 时钟/一致度实测）

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

### 2026-11-25 04:00（第 6 条，Phase 2 bag 底座已能在真实 sortie 落盘；飞行稳定性仍是最外层阻塞）

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

### 2026-11-25 04:35（第 7 条，找到飞行不稳的主因：主机空闲低功耗态；bag 抓取率仍是 Phase 2 的卡点）

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

### 2026-11-25 04:55（第 8 条，预热后拿到干净整趟；Phase 1 位置通道定位到 SITL 平滑/移动原点）

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

### 2026-11-25 05:35（第 9 条，Phase 1b 实现：tap 移出飞行路径，首飞 H1/H5 PASS）

**已完成（代码）**
- `sim/src/sim_state_recv.py --ros` 成为真值节点：/sim/state（Odometry NED/FRD，位置=pad 锚点+速度积分）、
  /sim/state/imu（同 stamp 配对）、/sim/truth/odom（world_w）、/clock、/sim/rtf；并把**发布网格**写成 tap 格式
  CSV（accept 无需改）。pad 锚点 (0,0,-0.0999) 依据：速度积分起点用该值对 tap 位置残差 5.7 mm（n/e 2 mm、d 9.5 mm）。
- `sim/src/lidar_imu.py --truth-topic`：ROS 真值消费。两个关键修正：订阅用**专用 spin 线程**（否则 400 回调/s
  只处理 ~100/s，IMU 15–20 Hz、75/80 扫描被跳）；odom/imu 配对**先查再 pop**（先 pop 吃掉 odom，配对永远失败）。
- `scripts/run-mission.sh`：`TRUTH_SRC=simstate` 分支（客户端替换 truth_rx，lidar 用真值话题，`LEGACY_TAP=1` 时
  并行录旧 tap 供对比）；`LIGHT=1` 跳过 cloud/traj CSV 记录器。
- `scripts/check_truth_ros.sh`（新）：无飞行管线校验 SITL→客户端→/sim/state→lidar→Super-LIO。

**实测**
- 无飞行校验 PASS：cloud 9.88 Hz、IMU 195.3 Hz、cast 15.8 ms、skipped 0；真值 CSV 恰好 200.0 Hz、max gap 5 ms、0 空洞。
- `missionA-phase1l-01-01`（预热 + LIGHT + TRUTH_SRC=simstate + LEGACY_TAP）：**完整飞行、正常 LAND+disarm**；
  H1 0.9998、H3/H4、**H5**、W0b、W1 13/13、W1b 13/13、W2/W3/W5/W6 全过；仅 H2（333 Hz < tap 时代的 380 下限）、
  H6（skipped 32 > 20）、W4（已知缺陷）不绿。
- 同趟两种真值打分对比：H1/H3/H4/H5/W1/W1b/W2/W3/W5 一致；W0 起点差 0.35 s；**W6 差 0.37 m**（计划要求 <0.01 m）。
  根因：客户端时间轴用"块平均周期"重建，长趟有相对漂移。

**下一步**
1. 时间轴改成"到达时刻的滚动线性回归（alpha+beta·i）"，消除相对漂移；复飞后重做 W6 双真值对比（目标 <0.01 m）。
2. H2 速率/空洞门槛随真值重定义（P0 类，延后但必须记录）；H6 skipped 的残余来源（DDS 丢包/arm 空洞）继续压。
3. 之后删除固件 tap 补丁与 `sim/src/truth_rx.py`/fanout（保留 `sim/legacy` 回滚），再进 Phase 2 收尾。

---

### 2026-11-25 06:00（第 10 条，Phase 1b 打磨：时间基对齐/流就绪门/重连；剩 LIO 停顿）

**本轮修复（全部已提交前验证）**
- 时间基：launcher 把 `client_start - sitl_start` 传给客户端 `--stamp-offset-us`；真值 t=0 = 仿真 t=0。
  实测 phase1p 的 W6 偏移 +0.600 s、四段 spread **0.005 s**（此前块平均周期 0.118 s、两真值差 0.37 m）→ **漂移解决**。
- 采样时刻：滚动最小二乘（alpha+beta·i）替代块平均；CSV 200.0 Hz、max gap 5 ms、0 空洞（H2 的空洞项已过）。
- 流就绪门：客户端等 autopilot 心跳（system≥1）；`FIRST SIM_STATE` 后 launcher 才起 LiDAR；8 s 无样本自动重连。
  phase1o 的教训：早起 LiDAR 进带洞缓冲区 -> 271 次跳扫、LIO 无里程计、abort。
- 客户端把发布网格写成 tap 格式 CSV（accept 免改），H2 的 max gap 从 1.04 s → 0.005 s。

**剩余**
- phase1p 飞行中段一次 **LIO 1.70 s 停顿** → "ESTIMATOR FEED IS STALE"（1.50 s 限）→ LAND 不完整、W1 6/13。
  H6 skipped 21（门槛 20）。都指向 SIM_STATE→ROS 偶发丢包/停顿：下一轮做单条组合消息（回调减半）+ DDS 资源上限
  + 客户端 hold 策略，再复飞拿一趟完整两圈。
- H2 rate 200 Hz vs 380 门槛属 tap 时代判据（P0 延后，需随真值重定义）。

---

### 2026-11-25 06:40（第 11 条，Phase 1b 收敛：单消息真值 + sim-t0 锚定 + 桥独占核 -> 干净整趟）

**本轮修复**
- 时间基：用"SITL 打印 tap banner 的 wall 时刻"当仿真 t=0（20 ms 轮询），客户端时间基与其对齐；此前用进程启动
  时刻，真值比 tap 早 0.600 s（W6 实测恒定 +0.600）。
- 单消息真值：`/sim/state` 用一条 Odometry（specific force 放 pose.covariance[0:3]），回调 400/s→200/s；
  LiDAR skipped 从 68–154 降到 20–23。
- 桥独占核：`ESTIMATOR FEED IS STALE` 的根因不是 LIO 丢数据（lio.csv 200.0 Hz、0 空洞），而是 bridge 的 LIO
  回调被 supervisor 的 1 ms 循环同物理核饿死。拆核：planner 12–13、**bridge 14**、AUX 16–17、truth 18–19。

**`missionA-phase1w-01-01`（TRUTH_SRC=simstate+LEGACY_TAP+LIGHT）**
- 飞完两圈、正常 LAND+disarm；H1 0.9998、H3/H4、**H5 10.79 vs 10.79 m**、W0b、W1 13/13、W1b、W2/W3/W5 全过。
- 余：H2（200 Hz vs tap 时代 380 门槛，P0 延后）、H6 skipped 23（门槛 20）、W4（已知缺陷）、
  W6（p50 0.238、spread 0.220 > 0.15，时间基仍有 ±0.1 s 慢摆）。

**下一步**
1. W6 慢摆：时间基从"到达时刻回归"改为"对齐系统时间（TIMESYNC/SYSTEM_TIME）+ 斜率拟合"，或每 1 s 重新锚定；
2. H6 skipped 23 -> 压到 20 以下（DDS 深队列/QoS 或 lidar 扫描调度）；
3. 然后删固件 tap 补丁/truth_rx/fanout，Phase 1 收尾，Phase 2 同趟 bag↔CSV 一致。

---

### 2026-11-25 07:00（第 12 条，Phase 1b 完成：unpatched SITL + tap 归档，两真值逐行一致）

**已完成**
- 重编 **UNPATCHED SITL**（patch -R + waf configure/copter，2m11s），旧 patched 二进制备份为
  `arducopter.tap-patched`；tap 补丁与 `truth_rx.py` 归档到 `sim/legacy/`（README + 回滚命令）；默认
  `TRUTH_SRC=simstate`，不读 tap 时 SITL 以 `LIO_TRUTH_TAP=-1` 启动；sim t=0 锚点改用两版都有的
  `Smoothing reset at ...` 行。
- **两真值同趟对比（phase1w，显式时钟映射 −0.748 s）**：H1/H5/W1/W1b/W2/W3/W0 完全一致；
  **W6 p50 差 0.003 m、p90 差 0.011 m、max 差 0.006 m** ⇒ 满足"W6 两真值差 <0.01 m"（p90 略超，记录在案）。
- `setup/build.sh`/`docs/DATA_PIPELINE.md` 更新 tap/真值说明与 patched/unpatched 构建步骤。

**未完成**
- unpatched SITL 首趟（phase1x）被主机 2 s 卡顿（raycast 2017 ms）打断 → LIO 失锁、模型飞走；与补丁无关，
  需在安静主机复飞确认。
- H2 速率门槛（tap 时代 380 Hz）待 P0 重定义；H6 skipped 20-23 略高于 20（留 Phase 4 sim 调度改造）。

**下一步**
1. 安静主机上复飞 unpatched SITL + SIM_STATE 真值，拿到完整两圈并归档 accept；
2. Phase 2 收尾：bag 抓取率问题（整栈运行时 ~40 Hz）+ 同趟 bag 派生 CSV 的 accept 逐行一致；
3. 之后 Phase 3（bridge+supervisor 合并）与 Phase 4（sim 图案窗口/每帧 CSV/rviz）。

---

### 2026-11-25 07:10（第 13 条，Phase 1 目标态验收飞行通过）

**`missionA-phase1y-01-01`（UNPATCHED SITL + 自建客户端 SIM_STATE 真值，无 tap）**
- 完整两圈、正常 LAND+disarm（13.85 s 布防 → 146.62 s 落地）：**H1/H3/H4/H5/H6 + W0b/W1(13/13)/W1b/W2/W3/W5/W6 全过**；
  H5 水平跨度 10.22 vs 10.22 m；H6 cast p50 18.1 ms、skipped 8；W6 spread 0.147 s。
- 仅 H2（rate 200 Hz vs tap 时代 380 门槛，P0 延后）与 W4（-0.05 m 已知缺陷）不绿。
- 全程无 tap：SITL `LIO_TRUTH_TAP=-1`、无 tap_legacy.csv、`sim/legacy/truth_rx.py` 未运行；时间基偏移 +0.1918 s。
- 结论：Phase 1 完成（判据类 H2 重定义留给 P0 批次）。下一步 Phase 2：bag 抓取率 + 同趟 bag 派生 CSV 的 accept 逐行一致。

---

### 2026-11-25 07:25（第 14 条，Phase 2：split-bag 写入与多 bag 派生；飞行仍受主机 2 s 卡顿影响）

**Phase 2 进展（代码）**
- `record_bag.sh` 支持 `BAG_SPLIT=1`：**两个 recorder 各写一半 topic**（A 组 /lio/robo/odom 200 Hz；
  B 组 /sim/truth/odom + /plan/cmd + /fcu/odom 各 100 Hz；/clock 故意不进 bag，D4 时间基来自真值客户端 stamps）。
  原因是实测单 writer 在飞行负载下只跑到 ~35 Hz/200 Hz topic。
- 两个 recorder 分核：`BAG_CORES=15`（bridge 的 sibling）、`BAG_CORES2=19`（truth client 的 sibling）。
- `bag_to_csv.py --bag` 改为可重复（`action="append"`），`--derive-all` 可跨多个 bag 派生同一套 CSV。

**实测（phase2h/2i，BAG_SPLIT=1）**
- 抓取率从单 bag 的 ~35 Hz/200 Hz topic 提升到：bag_a 128 Hz（/lio/robo/odom）、bag_b 66 Hz（100 Hz topics）。
  **仍未全量**；两趟都在飞行中段出现 LiDAR 进程 1.9–2.2 s 卡顿（raycast 2249 ms）→ LIO 失锁 → abort，
  属于主机侧偶发停顿（与 tap/真值改造无关，之前 bench 也复现过 1861–2014 ms）。
- 单 recorder 能力基准（隔离、无飞行负载）：写入 /dev/shm 与磁盘都稳定 **438 Hz**，所以瓶颈不是磁盘/rosbag2 本身，
  而是整栈运行时的调度/停顿。

**下一步**
1. 主机停顿：给 LiDAR 进程更高优先级/独立核（当前 4-7 两物理核，但 cast 仍会突然变 2 s）；或先做 Phase 4 的
   sim 慢扫描诊断（cast 2 s 时到底阻塞在哪）；
2. bag 抓取率：减少 bag recorder 的对手（BAG 模式下停 record-odom，从 bag 派生 lio.csv；run-mission 在飞行结束后
   自动派生），或把 bag 写 /dev/shm 再拷回；
3. 全量抓取后用 bag 派生 CSV 与现采 CSV 做同趟 accept 逐行对比（Phase 2 验收）。

---

### 2026-11-25 07:45（第 15 条，Phase 2 完成：bag 全量录制 + 派生层与现采逐字节一致）

**根因修复**
- bag 写 ext4 虚拟盘会触发 **VM 级 2 s 停顿**（LiDAR raycast 15 ms → 1861–2249 ms，LIO 失锁，sortie abort；
  phase2g/2h/2i 复现）。改为先写 **tmpfs（/dev/shm）**、飞行后由 `scripts/finalize_bag.sh` 拷回仓库 → phase2j/2k 两趟完整通过。
- 单 writer 在飞行负载下抓不全（35–128 Hz vs 应 200 Hz）；隔离基准显示单 writer 能到 438 Hz，问题是运行时被抢。
  `BAG_SPLIT=1` 双 writer（A: 200 Hz odom；B: 三个 100 Hz topic）+ 分核（15/19）+ bag **先于发布者启动**
  （rosbag2 会订阅后出现的 topic）+ `record_bag.sh` 跳过重复 source（省 2–4 s）→ 全量。

**实测（missionA-phase2k-01-01，完整两圈、正常 LAND+disarm）**
- bag：`/lio/robo/odom` 30310 行 @200.0 Hz、`/sim/truth/odom` 100.0 Hz、`/plan/cmd` 100.0 Hz、`/fcu/odom` 49.6 Hz（发布率）。
- `bag_compare.txt`：live lio.csv 30268 行**全部**在派生 30310 行中逐字节存在；traj cmd 14456 = 14456 全相同。
- 同趟新旧 accept：**14/15 行完全一致**；唯一 H3 差异是派生层比旧记录器多 42 个样本（旧记录器丢的），
  把派生层裁到同一窗口后 **15/15 完全一致**。
- accept（phase2k）：H1/H3/H4/H5/H6 + W0b/W1(13/13)/W1b/W2/W3/W5/W6 全过，仅 H2（P0 门槛）与 W4（已知缺陷）。
- 新门禁 `tests/check_bag_pipeline.py`（合成 → bag → 派生 → 校验）已接入 `tests/run_all.sh`，GATE PASS。

**下一步（Phase 3/4）**
1. Phase 2 目标态收尾：去掉唯一保留的 record-odom，让 `derived/` 直接成为 lio.csv/traj.csv.cmd.csv（finalize 已支持）；
2. Phase 3：bridge+supervisor 合并成单个自建 MAVLink 客户端（extnav 年龄门/质量/reset、wire liveness、yaw 单点转换）；
3. Phase 4：轻量 sim 的图案窗口/1200→60 ms 突发、每帧 CSV、rviz 回放。

---

### 2026-11-25 08:00（第 16 条，Phase 2 目标态飞行通过：没有 CSV 记录器，accept 只差 H2）

**`missionA-phase2m-01-01`：完全没有 per-topic CSV 记录器**（record-odom/traj/cloud 都不启动），
飞行后 `finalize_bag.sh` 从 tmpfs 的两个 bag 派生 `lio.csv`（28871 行）+ `traj.csv.cmd.csv`，metrics/accept 直接读派生层：

- H1 1.0000、H3 200.0 Hz / 144.4 s、H4 13 goals、H5 10.21 vs 10.24 m、H6 cast p50 18.3 ms / skipped 5、
  W0b 100%、W1 13/13、W1b 13/13（worst 0.08）、W2/W3/W5/W6 全过、**W4 +0.026 m CLEAR**；
  **只有 H2（200 Hz vs tap 时代 380 Hz 门槛）不绿** —— P0 延后重定义。
- 修了两个收尾问题：finalize 必须在记录器收到 SIGINT 并写完 metadata.yaml 之后运行（phase2l 的 derived 为空、
  H3 读不到 lio.csv）；且只能执行一次（EXIT trap 会再次进入 stop_stack）→ 用 `$OUT/.bag_finalized` 文件标记。

**Phase 2 完成**：全量录制 ✓、bag→CSV 与现采逐字节一致 ✓、同趟新旧 accept 逐行一致（14/15，窗口对齐 15/15）✓、
无 CSV 记录器的目标态真实飞行 ✓。docs/PHASE2-RECORDING.md 记录了全部证据与复现命令。

**下一步：Phase 3** —— bridge+supervisor 合并为单个自建 MAVLink 客户端（extnav 年龄门/质量/reset、
wire liveness、yaw 单点转换）；然后 Phase 4（sim 图案窗口/每帧 CSV/rviz 回放）。

---

### 2026-11-25 08:05（第 17 条，Phase 3 设计稿）

把 Phase 3 的合并设计写成 `docs/PHASE3-MAVLINK.md`（现状缺陷、目标架构、写死的接口约定、迁移与 A/B 验证计划、风险）：
- 目标 `bridge_nodes/mavlink/fcu_client.py`：一个 pymavlink 客户端、一条连接、**唯一** L→W→FCU 转换点；
  84 50 Hz / 331 48 Hz / COMMAND_LONG 事件驱动；状态机与守卫从 supervisor 搬入；`phase.csv`/`wire.csv`/`setpoints.csv`
  列名不变（accept 不动）。
- 写死接口：extnav age（0.5 s 降级 / 1.5 s abort）、quality（连续 3 帧异常停 331）、reset（>2 m 跳变默认只 abort，
  先 log-only 飞一趟看发生率）、wire liveness（心跳 >2 s / ACK 超时 3 次 → 停 setpoint+LAND）、yaw 单点转换、起飞门语义保持。
- 迁移：`FCU_CLIENT=merged|legacy`（默认 legacy 直到验证）；无飞行测试 `tests/check_fcu_client.py`；同臂 A/B 飞行
  对比 accept 与 84 目标时间序列；merged 连飞 2 趟通过后删旧实现。
- 风险表：单进程崩溃面、reset 误 abort、守卫循环与发送互相拖延（单线程 event loop + deadline 调度）。

---

### 2026-11-25 08:15（第 18 条，按 owner 指示减少录包：点云不再进 bag）

- **点云不录**：`/livox/lidar`（~197 kB/条 ×10 Hz）与 `/lio/cloud_world`（~307 kB/条 ×10 Hz）约 **0.7–1.0 GB/趟**，
  是其余 topic 的 ~16 倍；派生层与 accept 都不需要它们。默认关闭，`BAG_CLOUDS=1` 才显式打开（横幅警告体积）。
- 默认 `core` 集合缩到 4 个 topic（`/lio/robo/odom`、`/sim/truth/odom`、`/plan/cmd`、`/fcu/odom`），
  `/clock` 也去掉（D4 的时间基是真值客户端 stamps）；`full` = 全部非点云 topic（含 IMU/时间因子/规划参考/rosout）。
- 实测体积：`phase2m`（144 s）core 两个 bag 合计 47 MB；带点云的同长度飞行约 0.7–1.0 GB。
- 文档：`docs/PHASE2-RECORDING.md` 增加"数据量策略"节；PIPELINE-PLAN Phase 2 状态补 owner 指示。

---

### 2026-11-25 08:25（第 19 条，Phase 3 第一步：单连接合并客户端实现 + 门禁 PASS + 首飞暴露控制异常）

**已实现**
- `bridge_nodes/mavlink/fcu_client.py`：一个进程、一条 MAVLink 连接、一个 rclpy node。
  `BridgeProcess`/`Supervisor` 支持 `bridge=`/`node=` 依赖注入；supervisor 的 pump 每轮调 `BridgeProcess.step()`，
  84/331/class-76 由同一线程串行写；两半的 argparse 各自抽成 `add_*_arguments()`，合并 parser 用
  `conflict_handler="resolve"`。启动器 `FCU_CLIENT=merged` 可切换（默认 legacy）。
- 新门禁 `tests/check_fcu_client.sh`（SITL+合成输入，无飞行、不 arm）已接入 `tests/run_all.sh`：
  实测 `#331 46.9 Hz`、`#84 500/11 s`、`setpoints.csv 500 行`、`wire.csv 500 行`、extnav age gate 触发 -> PASS。
  全量 GATE: PASS。

**首飞（missionA-phase3a-01-01，FCU_CLIENT=merged）**
- 链路/状态机通：arm → 起飞 → 悬停 → THE GO → relay 使能 → leg1/leg2；84=1054 条、331=1774 条、phase.csv 正常；
  H1/H3/H4/H5/H6 全过。
- 控制异常：GO 后 z 冲到 3.17 m（legacy 同 arm 同时刻 1.85 m），随后跌到 1.6 m、SITL `SIM Hit ground`、
  EKF/truth 差 2.0 m → abort。同 arm 的 `wire.csv` 早期行与 legacy **完全一致**，所以不是目标值问题；
  待查 331 时间戳/相位、84 使能竞态、extnav 与 legacy 的逐行差异。默认仍 legacy，修好后 A/B 复飞。

**下一步**
1. 对比 merged 与 legacy 同一 arm 的 331 序列（t_usec、速度、发送相位）与 84 首批样本时刻，定位 z 过冲；
2. 修好后 merged 复飞，A/B 对比 accept 与 84 目标时间序列；连飞 2 趟后把默认切 merged。

---

### 2026-11-25 08:50（第 20 条，Phase 3 第二步：修 legacy 回归 + 331 黑匣子，merged 仍待修）

- **修回归**：Phase 3 重构让 legacy `run_loop()` 的状态发布丢了 `String` 导入 → bridge 第一个状态 tick
  NameError 死掉、FCU 拒绝解锁（phase3c 11.4 s abort）。修复后 legacy `phase3d` 完整 13 段、正常 LAND+disarm，
  FCU 与 truth 偏差 p99 0.30 m / max 0.45 m；H1/H3/H4/H6 + W0b/W1 13/13/W1b/W2/W3/W5/W6 全过。
  H5 的 FAIL 是**判据窗口伪影**（落地后 tap 又录 ~95 s，中位数漂移把 span 从 10 撑到 19.7），属 P0 类，记录不改。
- **331 黑匣子**：`--nav-log`（两种模式自动 `--nav-log $OUT/nav.csv`），逐条记录发出去的 ODOMETRY (#331)
  的 t_us/age/位置/速度/姿态/是否被 nav gate 拒绝；同时把 LIO 位置差分速度改为**消息时间戳 dt**（墙钟兜底）
  + `--nav-vmax` 15 m/s 限幅（合并模式回调成串时墙钟 dt 会算出几十 m/s 的假速度）。
- **merged 模式仍实验性**（默认 legacy）：phase3a/3b 能起飞/GO/leg1/leg2，之后 EKF 估计向上跑飞
  （2.6→70 m，~0.9 m/s²），模型 `SIM Hit ground`/`EKF variance`。nav.csv 显示 merged 起飞前有 **2.5 s 的 331 断流**
  且飞行中多次 0.1–0.2 s 小断流（legacy 无），跑飞期间 331 内容本身合理。下一步查断流根因与 EKF
  dead-reckoning 关系。

**下一步**：merged 断流根因（pump 阻塞分支）→ 复飞 A/B → 默认切 merged；然后 Phase 4。

---

### 2026-11-25 09:45（第 21 条，Phase 3 第三步：extnav 时间戳改用 FCU boot 时钟；单连接已被排除）

- **从 FCU 的 BIN 读出关键测量**：`VISP.RTimeUS` vs 本地时间 = EKF 看到的 extnav 年龄。legacy（phase2m）
  0.631 s，merged（phase3e）**1.611 s**。我们的 ODOMETRY 用仿真时间戳，而 EKF 用 boot 时钟比，1.6 s 的旧数据让
  位置环滞后发散。
- **修复**：请求 `SYSTEM_TIME`（2 Hz），把每条 331 的 time_usec 改写成 "FCU boot-now − 样本年龄"，并加严格单调
  下限。phase3f 的 BIN 实测：**VISP 年龄 p50 0.007 s / max 0.058 s**，`Ign=0`。
- 但 merged 仍然失控（phase3f/3g），且 phase3g 的 BIN 显示失控前 3 s 有一条 **mode 9 LAND, Rsn=2 (GCS_COMMAND)**，
  来源待查；`phase3h` 用 **--dual-link**（一个进程、一个 node、两个连接，即历史架构）同样失败 ⇒ **单连接共享
  被排除**，问题在"两个半身共处同一进程/pump"。
- 本轮改动：`--nav-log` 增加 FCU 侧列（ekf_ok/flags/armed/mode/nav_hz）；`--dual-link` 诊断/回退模式；
  LIO 差分速度改用消息时间戳 dt + `--nav-vmax` 限幅；bench 校验时间戳落在 FCU boot 时钟里。
- **下一步 bisect**：merged 进程 + legacy supervisor 进程，判定是 bridge 半身还是 supervisor 半身在合并进程里
  改变了行为；同时抓 serial0 上 class-76 的来源，解释 phase3g 的意外 LAND。

---

### 2026-11-25 10:40（第 22 条，★ owner 指令：飞行路径只用真实时钟 ★ + merged 喂数存活修复）

- **owner 指令**："不能按照 FCU 时间，要上真机，必须用真实 time.time/mono 给的时间，即使仿真也要尽量实时"。
  → 撤销"用 SYSTEM_TIME 把 #331 时间戳改写成 FCU boot 时钟"的做法；`#331` 现在用
  `time.monotonic()*1e6 − 样本年龄`，严格单调（停更时 +1 ms）。删除 SYSTEM_TIME 请求/解析。
- **修正上一轮的误读**：`VISP.RTimeUS` 的差不是"EKF 看到的陈旧度"，ArduPilot 用 `JitterCorrection`
  （AP_RTC）学习任意 offboard 时间基与 FCU 到达时间的最小滞后偏移（只要求不来自未来、滞后 <500 ms）。
  所以 0.63 s/1.61 s 只是时钟基差；真实时钟戳才是正确接口。
- **merged 喂数存活**：supervisor 有两处 `poll()+sleep` 等待绕过 pump，在 merged 里停掉 #331 与 ROS 回调
  （实测量化：`phase3n` nav.csv 出现 2.0 s 空洞、age 2.0 s，FCU 报 "Arm: VisOdom: not healthy"）：
  `check_own_link` 与 `check_position_stream` 都改为 `self.pump(period)`；并给 `MavlinkBridge` 加 `on_wait`
  钩子，让 `_command`/`calibrate_accel_simple`/`set_mode` 的 ACK 等待也驱动 `bp.step()`。
  每条 class-76 命令带调用者日志、FCU armed/mode 变化也打印。
- `phase3o`：2.0 s 空洞消失；残余 0.48 s 空洞在 accel 校准后、arm 前，仍超 FCU 的 300 ms extnav 健康阈。
  下一步把该间隙也归零，再飞完整 merged 趟。
- 验证：`tests/run_all.sh` GATE: PASS（bench 断言 `t_feed` 为真实单调时钟且严格递增）。

---

### 2026-11-25 11:20（第 23 条，真实时钟戳 legacy 验证通过；merged 已能起飞但仍不稳定）

- **owner 指令落实后的验证**：legacy 飞 `phase3r`，`#331` 用真实 `time.monotonic()` 戳——起飞/爬升/巡航正常
  （truth 1.87 vs FCU 1.95–2.08），W2/W3/W5/W6/W0b 过；77.6 s 因主机卡顿导致 lidar 被拖垮（skipped 89），
  supervisor 以 "ESTIMATOR FEED IS STALE (1.62 s)" 正确中止。⇒ 真实时钟戳对已验收链路无害。
- **merged 关键修复**：`inspect.stack()` 首次调用 import `inspect`+`linecache` 实测 0.48 s，正好卡在 arm 命令前，
  饿死 #331 >300 ms → FCU 报 "Arm: VisOdom: not healthy"。改用 `sys._getframe`（零 import）。
- `phase3p`（merged）：**解锁成功、起飞、leg1/2/3**；#331 无 >0.15 s 空洞；`#84` 序列与 legacy 相同；
  但爬升速率不同（merged 冲到 3.3 m，legacy 同目标 2.0 m 处 1.9 m），随后落地、EKF 滞后 ~0.6 m → abort。
- 已排除：喂数空洞/时基、发送抖动（p99 23 vs 22.5 ms）、84 目标值、单连接（`--dual-link` phase3q 同样不稳定）。
- 待查：CTUN/PSCN 层面的推力/姿态差异；bisect（merged 只跑 supervisor 半边 + legacy bridge）定位哪一半改变行为。

---

### 2026-11-25 11:30（第 24 条，门禁卫生：bag 测试的 rosbag2 残留进程）

- `tests/run_all.sh` 一度**间歇性 FAIL**：`check_bag_pipeline.py` 每次运行都留下一个 `ros2 bag record` 子进程
  （它被 `start_new_session` 的 shell 包着，SIGINT 打给 shell 不一定会传到 rosbag2），四次之后门禁被干扰。
- 修复：测试现在 (a) 启动时按 **argv[0..3] 签名**（`python3 …/ros2 bag record` + 本测试的 bag 路径）清理残留，
  (b) 结束时用 `os.killpg(spawn_pid, SIGKILL)` 杀整个进程组再补一次扫描。
  **注意**：不能用"命令行子串匹配"（gate 自己的 shell 命令行里就包含测试源码文本，子串匹配会杀掉 gate 本身，
  实测 exit 9）。
- 门禁连续两次 PASS；跑完后 `argv` 级检查无残留 recorder。

---

### 2026-11-25 12:00（第 25 条，Phase 3 bisect 工具就绪；supervisor-only 首飞被主机卡顿污染）

- 新增 bisect 模式：`FCU_CLIENT=bridge_only`（bridge 半边在合并进程里跑自己的 loop，supervisor 仍 legacy）与
  `FCU_CLIENT=supervisor_only`（supervisor 半边在合并进程，84/331 由 legacy bridge 进程提供，自己走 serial1）。
  run-mission 已接线；门禁 PASS。
- `supervisor_only` 首飞（phase3t）：连接/状态机/起降走通（正常 LAND+disarm），但主机卡顿（lidar skipped 86、H6 FAIL、
  SIM Hit ground 1.64 m/s）污染，不能作 bisect 结论；需安静主机重飞。
- 已积累对照事实：merged 与 legacy 的 84/331 内容与发送抖动几乎一致；merged 失败形态是**起飞后 FCU 姿态估计发散**
  （ATT roll/pitch 从 ~10° 涨到 40–50° 再到 137°/425°）+ 爬升 1.8 m/s（legacy 1.0 m/s）→ 模型失控。
- 主线怀疑：合并进程里单 node 承载两半订阅 + `spin_once` 每次只处理一个 callback，可能改变 `/lio`、`/plan/cmd`
  的 callback 顺序/延迟，从而改变 331 "发送时刻 vs 数据时刻"的关系。下一轮用干净 bisect 飞行定位。

---

### 2026-11-25 12:25（第 26 条，★ Phase 3 定位+修复：merged 的喂数必须由独立线程驱动 ★）

- **bisect 定位**：`FCU_CLIENT=bridge_only`（bridge 半边在合并进程但跑自己的 loop，supervisor 仍 legacy）
  飞了完整 **13/13 航点**（W0b 100%、W1b worst 0.10 m，H1/H3/H4/H5/H6 全过）；而 pump 驱动的 merged 会姿态发散。
  `supervisor_only` 趟被主机卡顿污染无法判读。⇒ 问题不是 bridge 代码/进程，而是**它被 supervisor 的 pump 驱动**。
- **修复**：merged 模式给 bridge 半边**独立线程**（1 kHz `bp.step()`），supervisor pump 只做 poll/spin/门限/class-76；
  `MavlinkBridge` 把 pymavlink 的每个 `*_send` 包上 `_tx_lock`（一个连接两个写线程），读仍在 supervisor 线程。
- `phase3x`（merged + 线程）：起飞正常、爬升到 2.38 m（目标 2.0）、无姿态发散，H1/H3/H4/H5/H6/W0b/W4/W5 全过；
  29 s 因 "ESTIMATOR FEED IS STALE (1.61 s)" 正确中止——`super_lio` 进程活着但停止发布（主机侧问题，
  legacy `phase3r` 77 s 也遇到同样 stale 中止），与改造无关。
- 遗留：`bridge_only` 那趟 W6 p50 0.93 m lag 需在稳定主机复查；下一步安静主机复飞 merged（期望 13/13）并做 A/B。

---

### 2026-11-25 13:15（第 27 条，★ Phase 3 bisect 收敛：bridge 半边独立线程+独立进程完全正常，问题在 supervisor 同进程 ★）

- 新增 `FCU_CLIENT=bridge_thread`（bridge 半边在 fcu_client 进程、独立线程内 poll+spin+tick；supervisor 仍 legacy 进程）。
- 飞行 `phase4d`：**H1/H3/H4/H5/H6 + W0b/W1/W1b/W2/W3/W4/W5/W6 全过（仅 H2=P0 门槛）** —— 与已验收基线等价。
- 对照：`bridge_only`（bridge 本进程 run_loop + legacy supervisor）=13/13 航点；`merged`（supervisor 同进程，含 dual-link）
  = 起飞后 4–5 s 失推力坠地/姿态发散。
  ⇒ **问题确定在"supervisor 与 bridge 同处一个进程"**；连接共享不是原因（dual-link 同进程仍复现）。
  最可能机制：两个线程共用一个 rclpy node + 一个连接（supervisor pump 在 node 上 spin_once，bridge 线程在同一 node/连接上发），
  而 `bridge_thread` 是"poll→spin→tick"串行。
- 下一轮最小实验：merged 模式下给 supervisor 单独的 rclpy node（bridge 线程 spin 自己的 node 并 poll 连接），
  通过则 Phase 3 形态定为"单进程 + 单连接 + 两个 node + 两个线程"。

---

### 2026-11-25 13:35（第 28 条，★ Phase 3 功能目标达成：merged 成为默认飞行路径并连飞两趟 ★）

- **最终根因**：supervisor 与 bridge 共享同一个 rclpy node / 进程全局 executor 时，起飞后 4–5 s 失推力/发散
  （FCU 的 XKF4.SH 高度创新 1–3 m → 拒绝 extnav）；拆成"每半身自己的 node + 自己的 SingleThreadedExecutor +
  自己的连接 + 自己的线程"后完全正常。`rclpy.spin_once` 是进程全局 executor，两线程同用会
  "ValueError: generator already executing"（本次一并修掉）。
- **形态**：一个程序（`bridge_nodes/mavlink/fcu_client.py`）、两个连接（serial0 = 84/331；serial1 = class-76/守卫）、
  两个 node（`fcu_client` / `fcu_client_sup`）、两个 executor/线程。`MavlinkBridge._tx_lock` 保留。
- **验收**：`FCU_CLIENT=merged` 设为**默认**；连飞 `phase4g`、`phase4h` 两趟：
  H1/H3/H4/H5/H6 + W0b/W1(13/13)/W1b/W2/W3/W5/W6 全过，与 legacy 基线（phase2m）行级一致；
  仅 H2（tap 时代 380 Hz，P0 延后）与 W4（+0.018 CLEAR / −0.047 CONTACT，已知 ±5 cm 缺陷）不同。
- **回滚**：`FCU_CLIENT=legacy` 回历史双进程；bridge_only/bridge_thread/supervisor_only 保留为诊断模式。
- 遗留：extnav 年龄/质量显式门限、legacy 路径清理（连飞达标后）。

---

### 2026-11-25 14:55（第 29 条，Phase 4 代码完成：图案窗口/每帧 CSV/rviz 回放入口；默认仍 legacy 待 A/B）

- **图案窗口修正**：`raycast.load_pattern_all()` 读整个 1 s 图案；`window_indices()` 在 `1/scan_hz` 整窗内均匀取点；
  每次扫描窗口前移（`Sensor.set_pattern`，numpy 视图）。bench 实测：12000 点也覆盖 **0.099995 s**（原 0.059995 s），
  窗口 `0.100 s = 20000 rows, advanced per scan`；`--legacy-lidar`/`SIM_LIDAR_LEGACY=1` 可回滚。
- **每帧 CSV**：`lidar_imu --scan-csv` 每扫描尝试一行（`t_start_us,cast_ms,points,reason`）；run-mission 总是写
  `$OUT/scans.csv`；**accept H6 优先读 scans.csv**（阈值不变，粒度变细），无文件时回退旧聚合。
- **rviz 回放**：`sim/rviz/pipeline.rviz`（world/truth/LIO/FCU/plan/cloud）、`scripts/viz_bridge.py`（odometry→Path、
  PositionCommand→Pose/Path）、`scripts/replay_bag_rviz.sh`（calib→静态 TF、起适配器、bag play、rviz2）；
  `scripts/run_rviz.sh` 默认配置同步。点云不进 bag，需要时 `BAG_CLOUDS=1`。
- `scripts/check_pattern.sh` 增加断言（整窗覆盖 + 逐扫描前进）；`tests/run_all.sh` GATE PASS。
- **新调度首飞 `phase4i`**：整圈 13/13、正常 LAND；H6 以 `[per-scan: scans.csv]` 读出（p50 18.5 ms / skipped 12）；
  但 **W6 lag spread 0.379 s > 0.15**（H5 False 是落地后 truth 尾巴的老窗口伪影）。按计划默认仍是 legacy，
  下一轮做 3 趟 A/B（legacy vs new）再决定切换。

---

### 2026-11-25 14:25（第 30 条，Phase 4 A/B 首轮：new 调度可用但 W6 变宽；默认保持 legacy；按 owner 要求暂停总汇报）

- 修 `LIDAR_SCHED_ARG` 未定义 bug（定义位置在 sensor 启动之后，set -u 直接崩）。
- A/B（固定 12000 点）：
  * `phase4jD` legacy@20000：H6 FAIL（skipped 237），确认 **20000 点在本主机不可用**；
  * `phase4kA` legacy@12000：H6 True（p50 16.1/skipped 9），但 plan box 停发导致 abort，不作比较；
  * `phase4kB` new@12000：完整 13/13、H6 True（p50 18.3/skipped 9，per-scan CSV 1628 行）、W4 −0.061 已知缺陷，
    **W6 spread 0.201 (>0.15)**；此前 `phase4i` new 的 W6 spread 0.379。legacy 基线（phase4g/4h）W6 在界内。
  ⇒ new 调度工作正常，但"整 100 ms 均匀点"改变了 LIO 输出时间行为，W6 判据感知到差异。**默认保持 legacy**，
  待再两对 A/B + LIO 时间参数核对后再决定切换。
- 按 owner 要求：**暂停飞行与推进**，输出总体汇报（docs/PHASE-SUMMARY-2026-11-25.md）。

---

### 2026-11-25 15:25（第 31 条，owner 指示：默认 10000 点 + 整窗均匀调度；新增 ODOMETRY 外部导航手册）

- 按 owner 决定把飞行默认改为 `LIDAR_POINTS=10000` + `LIDAR_SCHEDULER=new`（整窗均匀抽点、图案窗口逐扫描前进）；
  回滚 `SIM_LIDAR_LEGACY=1` / `LIDAR_POINTS=<n>`。两个参数都是 **sim-only**（仅 `sim/src/lidar_imu.py` 读取；
  真机点云来自 Livox 驱动，下游 LIO/bridge/supervisor/FCU 均不读该参数）。
- 修 run_until_healthy 未把 LIDAR_SCHEDULER 显式传入 run-mission 的问题（之前靠环境继承，易漏）。
- `scripts/lio_drift.py` 实测 LIO 相对真值漂移：phase4kB 0.10–0.37 m、phase4g 0.04–0.20 m，无 >0.25 m 跳变
  —— 支持"降点数到 10000"的决策，但 W1b/W6 仍需飞行 A/B 确认。
- 新增 `docs/EXTERNAL-NAV-ODOMETRY.md`：ODOMETRY(#331) 喂 EKF3 的完整实战手册（字段契约/帧/原点/参数/时间戳/
  上线检查清单/常见故障表/pymavlink 示例）。
- 下一步：用新默认飞 1–2 趟验证 H6/W6/W1b；若 W6 变宽，回滚调度或点数并记录。

---

### 2026-11-25 16:25（第 32 条，Phase 4 收尾：默认 10000 点 + legacy 调度；发现 W6 的时间基准问题）

- 按 owner 指示先切了 10000 点 + 整窗均匀调度，随后做 A/B（固定 10000 点，只切调度）：new 调度 4 趟干净趟里
  1 趟过 W6（0.129），3 趟失败（0.250/0.469，另一趟主机停顿污染）；legacy 6 趟 5 趟过（0.077 等）。
- **发现更根本的问题**：`phase4oE`（legacy）W6 失败不是跟踪问题——`cmd vs truth` 是**常数 +1.35 s**（四季度一致），
  `lio vs truth` 是**常数 +1.45 s**，即 `setpoints.csv t_sim_us`（LIO 时间戳）与 `tap.csv`（SIM_STATE 时间）
  这一趟差了 ~1.4 s；W6 的 ±0.6 s 搜索窗全部顶边。W6 代码注释本就说明它是两套时钟的记录、好趟只剩 2 ms 余量，
  de-trending 属 P0 判据改动。→ 列入下一步：D4 时间基准统一/标定（在 P0 之前）。
- 结论：Phase 4 完成（per-scan CSV/H6、rviz 回放、图案窗口修正已实现且可 opt-in），**默认 = 10000 点 + legacy 调度**；
  `LIDAR_SCHEDULER=new` 保留为实验开关。证据 `phase4oA/4oB/4n/4oC/4oE`，docs/PHASE4-SIM.md §7。

---

### 2026-11-25 16:45（第 33 条，D4 时间基准修复 + Phase 4 完成：默认 10000 点 + legacy，连飞两趟干净趟）

- **根因修复**：W6 假失败的根源是 `tap.csv t_us`（录制器单调钟）与飞行侧 `t_sim_us`（仿真钟）混用，
  差值 = `--stamp-offset-us`（正常 0.15 s，phase4oE 1.35 s，直接顶穿 W6 的 ±0.6 s 拟合窗）。
  修：tap.csv 增加 `t_sim_us` 列（两列都写）、accept.tap_world 优先用 t_sim_us（老 sortie 兼容）、
  run-mission 写 `truth_timebase.json` 并注入 `manifest.truth_timebase`、新增 `tests/check_timebase.py` 入门禁。
- 修复后 W6 绝对偏移变为真实跟踪滞后 ~−0.15 s；连飞最终默认（10000 点 + legacy）：
  `phase4oF` W6 T(0.079)、W4 CONTACT −0.027；`phase4oH` W6 T(0.084)、**W4 CLEAR**、H1/H3/H4/H5/H6+W0b/W1/W1b/W2/W3/W5 全过
  （仅 H2 P0 判据不过）⇒ **Phase 4 完成（连飞两趟干净趟）**。
- new 调度在时钟修复后仍 W6 F（`phase4oG` 0.413，Q1 −0.500/p90 0.77，与时钟无关）⇒ 默认保持 legacy，
  `LIDAR_SCHEDULER=new` 作为实验开关；遗留调查项记入 PHASE4-SIM §8。
- 门禁：`tests/run_all.sh` GATE: PASS（含新 timebase 测试）。

---

### 2026-11-25 16:50（第 34 条，Phase 3 遗留收尾：extnav 年龄/质量/reset 显式门 + 单元测试）

- 新增 `bridge_nodes/mavlink/nav_gates.py`（无 ROS/MAVLink 依赖的决策表）：`refusal_reason(physical_ok, age, quality,
  min_quality, max_age)` → `0=sent / 1=physical / 2=stale / 3=quality`，顺序物理>陈旧>质量；`describe()` 给状态行用。
- bridge 新增参数：`--nav-quality`(默认100)、`--nav-min-quality`(0=关)、`--nav-max-age-s`(0=关)、`--nav-reset-gap-s`(0=关，
  开启后两次已发送样本间隔超限 → reset_counter++，EKF 接受跳变)；`send_external_nav*()` 的 quality/reset_counter 变成参数。
- 记录：`nav.csv` 的 `refused` 变为原因码，新增 `quality,reset_counter` 列；状态行加 `nav_age_ref/nav_q_ref/nav_resets`。
- **默认与交付行为逐位等价**（quality 100、三门全关），不会改变飞行；飞行开法 `BRIDGE_ARGS="--nav-max-age-s 0.3 ..."`。
- 测试：新增 `tests/check_nav_gates.py` 入门禁；`tests/check_fcu_client.sh` 主 bench 保持 47.1 Hz/relay 500/age-gate PASS，
  并断言生产默认下 quality=100/reset=0/refused=0 且两列存在。`tests/run_all.sh` GATE: PASS。

---

### 2026-11-25 17:05（第 35 条，Phase 3 清理固化：bisect 形态删除、一套 FCU 实现 + legacy 回滚开关）

- `bridge_nodes/mavlink/fcu_client.py`：删除 `--bridge-only/--bridge-thread/--supervisor-only/--legacy-shape/
  --dual-link` 与对应分支；只保留 `merged`（默认：一进程、两连接、两 node、两 executor/线程）与 `--bench`。
- `scripts/run-mission.sh`：FCU 分派收缩为 `merged`（默认）| `legacy`（回滚，两进程），未知值 exit 2；删除
  bridge_thread/bridge_only/supervisor_only 分支及其对 supervisor 启动的影响。
- 新增 `tests/check_launch_matrix.sh` 入门禁：静态断言启动矩阵、merged 两 endpoint、legacy 两进程、无残留
  bisect 参数、`--bench` 存在、客户端可编译。
- **清理后飞行验证** `phase4p`：H1/H3/H4/H5/H6 + W0b/W1(13/13)/W1b/W2/W3/W5 全过（H6 p50 12.9 ms/skip 10，
  truth_timebase.json 正常）；W4 CONTACT −0.016（已知缺陷）、W6 FAIL（spread 0.427，Q4 −0.544 单季度异常）。
- **W6 归因修正**：时间基准修复后 W6 失败在 legacy/new 两种调度都出现且都是"单季度偏移大"（4oE legacy、
  4oG new、4p legacy），说明它与调度/点云预算无稳定关联，是 P0 判据本身的脆弱性（accept.py 注释已写明
  两套时钟、好趟仅 2 ms 余量）。→ 下一步不再用飞行追调度差异，改为 P0 判据改造（de-trending/残差门限）。
- 门禁 `tests/run_all.sh` GATE: PASS。

---

### 2026-11-25 17:15（第 36 条，Phase 5 附加块 + rviz 回放 headless 冒烟）

- **APE/RPE 附加行**：新增 `scripts/traj_error.py`（numpy 实现 Umeyama 对齐 + Δ=1 s 相对误差，无 evo 依赖）：
  真值用 `tap.csv t_sim_us`（显式仿真钟，老文件回退 t_us），同时评 `lio.csv`(L→W) 与 `setpoints.csv fcu_*`；
  输出 rmse/p50/p95/max + 对齐 yaw/平移。实测 `phase4p`：lio APE rmse 0.027 m / RPE 0.015 m，fcu APE 0.052 m，
  z 平移 +0.10 / −0.12 与已知 z 偏置方向一致。
- **MANIFEST 完整性（record-only）**：`accept.py` 新增 `extras.manifest`（present/missing/legacy_timebase），
  RESULT 前单独打印；缺字段只报告不判 FAIL（P0 规则）。
- **accept extras**：`--no-extras` 可关；15 行判据完全未改；`accept.json` 增加 `extras` 块。
- **rviz 回放 headless 冒烟**：新增 `tests/check_rviz_replay.sh`（无 bag 时 SKIP，入门禁）：验证 bag 播放、
  `/viz/*` 话题速率（truth 100–120 Hz、plan ~35 Hz）、`/tf_static` 的 `world_w→lio_world`；
  顺带修两个 bug（静态 TF printf 参数错位、`--no-rviz` 空 TF 行 set -u 崩溃），TF 检查对 latched 话题重试。
- 测试：`tests/check_traj_error.py` 入门禁（已知刚体变换 APE<5mm、能看见 drift、--no-align 暴露标定偏移、
  manifest 完整/残缺/缺失三种情况）；`tests/run_all.sh` GATE: PASS。
- 文档：新增 `docs/PHASE5-EXTRAS.md`；PIPELINE-PLAN Phase 5 标记 [部分完成]；PHASE4-SIM §9；README 索引。

---

### 2026-11-25 17:35（第 37 条，Phase 5-3：bag→derive→live 一致性 CI 入门禁）

- `tests/check_bag_pipeline.py` 扩展：同一次运行里同时用 bag（split writers）与飞行实时 CSV 记录器
  （`record-odom.py`、`record-traj.py`）记录同一对已知话题（30 Hz，避免两个订阅者丢不同包），派生 bag 后
  逐样本对比"派生层 vs 实时层"：派生每行必须能在实时层找到同 t_us 且数值一致（<1e-6），要求 ≥95% 匹配、
  0 个值不一致。实测 lio/traj.cmd 两条均 `matched 100.0% of derived, value mismatches 0`。
- 这就是 Phase 2 真实飞行结论（派生层逐字节包含现采层）的可重复 CI 版本，不需要飞行。
- 文档：PHASE5-EXTRAS.md §4 更新；门禁 GATE: PASS。

---

### 2026-11-25 17:50（第 38 条，P0 判据改造提案文档（未实施））

- 新增 `docs/P0-CRITERIA-PROPOSAL.md`：用仓库内 **80 趟** accept.json 的实测分布写 P0 提案：
  - W6：拆"稳态 lag 中位数 ≤0.35 s" + "去线性时钟漂移后的残差 std ≤0.12 s" + 保留 p90 ≤0.60 m；
    单季度偏离中位数 >0.25 s 记为 WARN 而非 FAIL。数据：时间基准修复后通过趟 spread 0.049–0.133；失败趟 0.20–0.47
    且形态统一为单季度滞后（−0.18…−0.54 s），W1b/W2/W5 仍通过 → 是跟踪瞬时滞后，不是时钟。
  - H2：380 Hz 是 tap 时代产物，SIM_STATE 时代配置 200 Hz → 提议改"速率==配置 ±5% + 最大间隙 ≤2×周期"。
  - W4：80 趟 min −0.096 / 中位 −0.0525，"CLEAR>0" 不可行 → 建议先用既有 `--known-defect W4` 接受，
    容差版（≥−0.05 m 且体内样本≤0.5%）作为下一阶段目标。
  - H6：阈值不动，skip 上限建议 max(20, 1%×扫描数) 并增加 p99；H3/W1b 微调建议；W2 不再改（round 20 已改）。
- **未实施**：15 行阈值一字未改；批准后以 `--criteria-v2` 开关 + 历史 sortie 回放差异表来切换。
- 门禁 GATE: PASS。

---

### 2026-11-25 18:00（第 39 条，第 21 轮收尾：Phase 5 完成，管线五个 Phase 全部关闭）

- Phase 5 交付：manifest 完整性（record-only）、APE/RPE 附加行（traj_error.py + accept.extras）、rviz 回放
  headless 冒烟（check_rviz_replay.sh）、bag→derive→live 一致性 CI（check_bag_pipeline 扩展，派生 98.8–100% 匹配
  且 0 值不一致）、P0 判据改造提案（P0-CRITERIA-PROPOSAL.md，未实施）。
- 门禁新增 4 个测试（timebase / nav-gates / launch-matrix / traj-error / rviz-replay / bag-replay 扩展），
  `tests/run_all.sh` GATE: PASS；HEAD `d3ab11d` 及后续已推送。
- 至此 PIPELINE-PLAN 的 Phase 0–5 全部关闭；剩余事项只有 **P0 判据改造（等所有者批准）** 与
  new 调度 W6 单季度异常的后续调查（等 P0 改造后再评估）。

---

### 2026-11-25 20:00（第 40 条，P0 A/B 证据 + accept --criteria-v2（默认关）+ 真机迁移检查表）

- `scripts/criteria_ab.py`（新）：对 **104 趟**存量 sortie 重算"现行 v1 vs 提案 v2"，产出
  `docs/metrics/p0-criteria-ab.txt`。结果：H2 4 趟 F→T（SIM_STATE/200 Hz 的趟）、W4 20 趟 F→T
  （clear −0.005…−0.047、体内 0.02–0.42%）、**W6/H6 0 趟翻转**（W6 单季度真实滞后在 v2 下仍 FAIL，没有被洗白）。
- `accept.py --criteria-v2`（默认关）已实现 H2/W4/W6/H6 的提案语义；v1 行为逐字节不变（默认 JSON/print 标注
  `criteria: v1-default`）。`tests/check_criteria_v2.py` 入门禁：重放该 sortie 存档 argv，断言默认 v1 与存档
  14 行一致、v2 只允许这四行翻转。
- 新增 `docs/REAL-MACHINE-CHECKLIST.md`：仿真-only 清单、真机需替换的接口（LIO 输入/输出、FCU 链路、参数、
  标定、录制/真值缺失）、上电→首飞检查顺序、与仿真基线的 A/B 对照问题。
- 文档：P0 提案补 A/B 证据与实现状态；PHASE5-EXTRAS §5；README 索引；门禁新增 criteria-v2 测试。

---

### 2026-11-25 20:15（第 41 条，★ 数据管线「规范化完成」交付 ★）

- 新增 `docs/DELIVERY-2026-11-25.md`（交付单一入口）：三条运行命令、每趟产物清单、Phase 0–5 状态与证据、
  默认配置、回滚开关、已知遗留（P0/W4/W6/真机）、门禁覆盖、"规范化完成"的 7 条判定标准（全部满足）。
- 本轮同时完成：`scripts/criteria_ab.py`（104 趟 v1/v2 A/B，`docs/metrics/p0-criteria-ab.txt`）、
  `accept.py --criteria-v2`（默认关，`tests/check_criteria_v2.py` 钉住 v1 不变）、
  `docs/REAL-MACHINE-CHECKLIST.md`（真机迁移检查表）。
- 门禁：`tests/run_all.sh` 连续两次 **GATE: PASS**；工作树干净（本地 commit，按所有者要求未 push）。
- 状态：Phase 0–5 全部完成；剩余只有**所有者决策类**（P0 判据是否切默认）与**立项类**（W4 规划器安全边距、
  W6 单季度滞后归因、真机迁移）。

---

### 2026-11-25 22:05（第 43 条，跟踪质量诊断 + 高速竞速方案；角点过弯 A/B）

- **跟踪诊断**（新工具 `scripts/track_quality.py`）：phase4q 零滞后误差 p50 0.154 / p90 0.265 m，最优滞后 **−0.13 s**；
  沿航迹 p50 −0.124 m、横侧 p50 −0.023 m → 是"延迟/追不上"而非跑偏。参考 |a_xy| p90 1.45 / p99 2.81 / max 4.02 m/s²。
  结论：不是 PID 没调好、也不是控制量缺失（`wire.csv` 是全 PVA；ArduPilot 4.7 `posvelacc` 用
  `input_pos_vel_accel_NE_m(..., limit_output=false)`，前馈不削、只削修正）；主因是 ~0.13 s 总延迟
  + `PSC_NE_POS_P 1.2` 偏低 + 参考加速度随速度平方增长。
- **修正认知**：PVA 模式 jerk 限幅是 `PSC_NE_JERK`（80 m/s³），不是 `WP_JERK`（后者只影响位置-only 导航）。
- **高速方案** `docs/HIGH-SPEED-PLAN.md`：阶段 0 tracking-vs-speed 阶梯（工具已就绪）；阶段 1 先补延迟
  （`REF_LEAD_S`、nav/wire 频率）再调硬（WP_ACC/WP_SPD/PSC_NE_* /ATC）；阶段 2 规划侧 3→6 m/s（新 race 配置/世界）；
  阶段 3 8–10 m/s 需控制栈升级评估（据 SUPER 论文 >20 m/s、T/W>5；我们 GUIDED 级联预计 5–7 m/s 上限）。
- **角点过弯 A/B**：`GOAL_PVA=1 bisector CORNER_SPEED=1.2 ARRIVE_M=0.3`（phase4s）角点平均转角 172°→**139°**、
  指令速度 p10 0.49→0.57、W1b/W6 仍过；`GOAL_LEAD=1`（phase4t）W1/W1b/W6 失败 → 拒绝。
- 文档：`docs/HIGH-SPEED-PLAN.md`、`docs/FLIGHT-LOOP-ANALYSIS.md` §6；图：`docs/evidence/phase4s-corner*-zoom.png`。

### 2026-11-25 20:55（第 42 条，绕圈归因：参考"停车"→飞机冲过头；新增俯视分圈/时序图/分析脚本）

- 新增 `scripts/plot_flight_topdown.py`（整趟/`--per-lap` 分圈俯视）、`scripts/plot_flight_timeseries.py`
  （xyz-t / vxyz-t / speed-t，真值 vs 指令，标出分圈与绕圈事件）、`scripts/analyze_loops.py`
  （绕圈检测 + 与 SUPER 规划日志对齐）。
- **phase4p 分析结论**：5 处绕圈（每处 ~1 s、净位移 0.2-0.6 m）**全部发生在"指令速度掉到 0.00–0.10 m/s"
  的时刻**（真值仍 1.0-1.5 m/s）：飞机刹不住、冲过停住的参考点后被位置环拉回 → 小圈。
  整趟"指令停一下"22 次（11 次在航点属设计、11 次在腿中异常）。不是径向超速（绕圈点多在腿中、离航点
  3-4.7 m，cross-track p90 0.2-0.5 m）、不是 planner 无解（1211 次 replan 成功；事件 1 附近零 Opt failed）、
  不是估计器（FCU 估计速度与真值几乎相同）。
- 头号嫌疑：LIO 里程计 twist 整趟为 0（lio.csv vx,vy=0），若 SUPER 以"速度=0"重规划，新轨迹从静止起步
  → 每拍参考速度掉 0；次因：局部 Opt failed 簇（事件 4/5 之前）。修复建议见文档 §5。
- 文档：docs/FLIGHT-LOOP-ANALYSIS.md；图：docs/evidence/topdown-phase4p*.png、
  timeseries-phase4p-{xyz-vxyz-speed}-t.png；README 索引、PROGRESS 本条。

---

### 2026-11-25 00:25（第 1 条，Phase 0 进行中，约 1h20m）

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


---

## 历史集成记录（README-integration-history.md）

> 原文：`docs/README-integration-history.md`（已并入本文件，独立文件删除）

## super_super_apm —— 航点 → SUPER → bridge → ArduPilot

### 0. 当前状态（第 88 轮：**新图 `forest_hex12.yaml` 上飞完两圈**；真凶是雷达仿真的光栅化，不是规划）

> ### ⚠ 两条硬约束（用户第 81 轮）：`docs/SPEC.md` 是唯一有效的规矩；**地图与航点已冻结**
>
> `scripts/check_frozen.py` 把新世界文件的 SHA-256（`d30918ef…`）、**522 根柱**、六个角点（半径 10.000 m）
> 写进代码，`scripts/fly_hex2.sh` **每次起飞前跑它**（13 行），任何一条不过就拒绝起飞；本轮没有改动
> 世界文件（13/13 PASS）。世界文件一旦本轮冻结就不再改。

| 项 | 结果 |
|---|---|
| **★★★ 交付：新图两圈飞完；最好一趟 W6 p90 已到 0.43 m** | **`kn3-01-01`**（acc 6 / jerk 15 / corner 0）：0 abort、H1–H5 **全 PASS**、W1/W1b **13/13（最差 0.06 m）**、**W6 p50 0.230 / p90 0.433**、第 2 圈真值净空 **+0.081 m（无接触）**、rtf 1.0005；W6 只差“时基四段跨度 0.16 > 0.15 s”这一条。复算 `bash scripts/render_evidence.sh kn3-01-01` |
| **★★★ 质量指标：全面优于旧冻结图** | 规划**停住 69.1 % → 11.2 %**、重规划跳变 **> 0.3 m 27 次 → 0 次**、横向误差 **p50 0.283 → 0.106 m / p90 0.818 → 0.277 m**、沿航迹 p50 **+0.135 → −0.02 m**、角点自由半径 **0.245 → 1.026 m**；最好一趟（角点 0）W6 p50 **0.219** / p90 **0.492** |
| **★★★ Super-LIO 的 SIGFPE：定位并修好** | 三个 core 的 backtrace 完全一致：`pcl::transformPointCloud` ← `SuperLIO::Output()` ← `stateProcess()` 定时器；故障指令 `div %r9`、**rax = r9 = 0** ⇒ 空点云进了 PCL 的整数 `0/0`。`Output()` 在定时器线程、点云在回调线程，无锁。节点里加两行守卫（空云直接 return），重编 `liblio.so`（旧库备份 `liblio.so.bak-round88`）；**加守卫后 4 趟 0 次 SIGFPE**（此前约每 2–4 次尝试一次）。验证：`bash scripts/lio_guard_verify.sh` |
| **★★★ 真凶：雷达仿真的 trunk 光栅化** | `_cast_trunks_chunk` 的 85.7 ms 不在算术而在**522 次 Python 循环 × 每次 ~15 个 numpy 调用**。新写 `_cast_trunks_flat` 把窗口摊平成一张 (柱, 射线) 对表再一次向量化：**67.3 → 10.7 ms（6.3×）**，并与逐柱循环**逐位一致**（`scripts/check_raycast_flat.py`：0 处不一致）、对 `brute_force` 3.6e-5 m |
| **★★★ 它修好了什么** | cloud **7.00 → 9.97 Hz**、每帧 **3 909 → 20 000 点**、**丢弃扫描 372 → 0**、cast 21–25 → **11–17 ms**。因果链：**7 Hz 还丢帧的雷达云 → LIO 吃不饱 → 飞控外部位姿时断时续 → 位置环发散**，第 74.7 节的"飞机跑在规划前面 3 m"就是这条链的末端 |
| **第二把钥匙：参考必须自洽** | `--ref-gov-maxlag 0.15 → 0`、`kp 6 → 12`：旧夹取把 `p_ref` 硬拉向规划点，而那段位移**不在发布的速度里**（实测速度字段 ≤1.07 m/s 时位置流跑到 2.4 m/s）。归零后 `p_ref` 的导数**恒等于**发布速度（滞后 `δ=v_super/kp`）；竖直同理 `REF_Z_TOL 0.20 → 0.05`（`v_z=0` 才不是谎话） |
| **直道与角点统一 1.5 m/s** | `max_vel` = `CORNER_SPEED` = **1.5**。120° 角点 `R=v²/max_acc`：2.8 m/s ⇒ R=0.98 m，叠加 ~0.4 s 位置环滞后 ≈1.1 m，正好用光"角点 10 m 到围栏 12 m"的 2 m 余量 —— 本轮四次撞线**全在角点**（`hx9-01-01` x=+12.02、`hy0-01-01` x=−12.01） |
| **位置环 PIO（`hz1-01-01`）** | 参考全程 ≤1.50 m/s，真值跑到 **4.1–5.2 m/s** 并掉高触地。机理：`PSC_NE_POS_P 3.0 × 1.5 m` = +4.5 m/s 的追赶需求，`WP_SPD 6.5` 不拦。按用户"pid 参数不够就调固件"，只动振荡的那一层：`WP_SPD 6.5→2.0`、`WP_ACC 10→4`、`JERK 150→80`、`POS_P 3.0→1.2`、`VEL_P 3.0→2.0`，**姿态环一个字没动** |
| **试过并否定：参考超前补偿** | `--ref-lead-s`（目标点沿速度提前 0.35 s、拐角按转向率淡出）：W6 p50 **0.375 → 0.552 / 0.598**，更差。原因：本机残差是**横向跟不上绕柱蛇形**，不是沿航迹落后。默认 0，开关与代码保留 |
| **估计器（不是瓶颈）** | `docs/evidence/lio-err-t.png`：水平 p50 **0.036** / p90 0.057 / max 0.084 m，竖直 p50 **0.012** / p90 0.024 m。比规划净空小一个量级 |
| 仍然不达标的三行（**如实**） | **W2** 4.0 m/s 是第 81 轮旧规格、与本轮 1.5 冲突（实测 p50 1.35 / max 3.45，**阈值一个字没改**）；**W4** 最小柱面净空 −0.096 m，根因是**规划自己**两圈的净空只有 **+0.013 / +0.067 m**（`inflation_step=1` ⇒ 0.10 m 膨胀，小于 MINCO 拐角切角量），比跟踪残差 p90 0.39 m 小一个量级；**W6** p50 0.375 m（其中横向 ~0.15 m） |
| 环境类失败（非本链缺陷） | ① **Super-LIO 偶发 `SIGFPE`**：历史 886 趟里 **46 趟（5.2 %）**，靠重试；② **SITL 模型飞走**（模型与飞控估计几秒内差 70–95 m，LIO 本身仍与真值差 4 cm）；③ **RTF 门 0.98**：好几趟落在 0.978–0.979 被判"不可评分"，**飞的时候不要跑分析/画图** |
| 图（本轮三类 + 地图图） | `docs/evidence/`：`lap1-paths.png`、`lap2-paths.png`（每圈俯视图，标题写该圈规划/真值最小净空与 \|规划−真值\| p50/p90）、`vxyz-t.png`、`lio-err-t.png`、`world-hex12.png`；旧图族全部删除，`scripts/check_evidence_links.sh` 断言它们不在，也断言没有第二种命名方案 |
| 文档 | `docs/INTEGRATION.md` **§75**（真凶、两把钥匙、PIO、超前补偿的否定、十一行、质量指标、下一步三候选）；`docs/SPEC.md` S1 重写 + 修改行；`docs/evidence/README.md` |
| 用户已拍板（第 88 轮补） | ① `rog_map.inflation_step` **1 → 2**（规划净空 0.1 → 0.2 m，针对 W4 接触）；② `max_acc` **8.0 → 9.0**（角点速度上限 2.86 → **3.04 m/s**，3.0 m/s 回到界内）。两处都已落进配置 + SPEC + `spec_check`，静态门 PASS；**尚未用新参数重飞**（下一轮的第一件事） |
| 复算一条链 | `bash scripts/check_static.sh` → `bash scripts/check_evidence_links.sh` → `python3 scripts/spec_check.py` → `python3 scripts/check_raycast_flat.py` → `python3 scripts/tracking_diag.py ke1-01-01` → `bash scripts/render_evidence.sh ke1-01-01` → `python3 scripts/plot_hex_laps.py --tag ke1-01-01 --laps 2 --world worlds/forest_hex12.yaml --prefix lap` |

#### 0.1 第 85–86 轮（历史：`posvel` 把航点拉到 19/19；W4 的地板是规划自己扎进柱子）

| 项 | 结果 |
|---|---|
| 用户第 85 轮的两句话 | "robot_r 降到 0.10 m"、"**但是现在飞的一团糟，这是为什么？求解不出来吗？跟踪非常差**" |
| `robot_r 0.10` | 已改：`docs/SPEC.md` S4 + `config/` 三份副本 md5 一致 `e7827164…` + `spec_check` PASS。七条腿的走廊余量 **b − robot_r = 0.145 … 0.424 m 全正**（`set_robot_r.sh 0.10`） |
| **"跟踪差"的答案（一条）** | `run-mission.sh` 对非 ego planner 默认 **`BRIDGE_FIELDS=pos`**，于是 bridge 用 `type_mask 2552` 把 SUPER 的**速度与加速度全屏蔽**，飞机只收到 50 Hz 的**位置阶跃**。FCU 自己的 `PSCN/PSCE` 日志（`scripts/fcu_tracking.py`）显示误差是**横向**的：\|DP−P\| p50 **0.610 m** 而纵向 p50 只有 **+0.063 m** —— 这就是"位置环没有速度前馈"的签名 |
| 为什么横向误差这么大（物理上限） | 新工具 `scripts/path_curvature.py`：冻结地图把 SUPER 的路径逼到 **R p10 = 0.39–0.48 m**；45° 倾角只能给 9.81 m/s² ⇒ 这种弯的速度上限 **1.96–2.17 m/s**，而参考跑到 **4.1 m/s**（2 m/s、R 0.39 m 时需 **10.3 m/s²**）。第 79 轮的宽车道世界 R p10 = **1.79 m**（上限 4.19 m/s），同一套控制器只差 **0.090 m** ⇒ 差别不在控制器，在**弯太急** |
| 参考还会"停" | `scripts/plan_anatomy.py`：`d85-01-02` 全程只有 **13 条轨迹**，其中**四条长度正好 40.18 s 的 HOLD**（= 薄层 `--leg-timeout-s`）⇒ 两段 **81.6 s / 83.2 s 静止**，期间真值漂 6.2 m。MINCO 失败 3759 次是它的上游原因 |
| **PVA 到底有没有上过线**（用户第 85 轮追问） | **没有。** `wire.csv` 的 `type_mask` 记着每一轮：`v4a`/`d84`/`d85`/`r10` 全是 **2552（pos，只有位置）**，`pv1` 是 **2496（posvel）**，`pva1` 才是 **2048（posvelacc）**。SUPER 的 v/a 一直在 `setpoints.csv` 里（那是桥的日志），只是没上线——两份日志不摆在一起看不出来。现在按用户要求**全 PVA 上线**，并给前馈加了钳位（`--ff-vel-max 2.5 / --ff-acc-max 5.0 / --ff-acc-slew 40`，= SUPER 自己的 max_vel/max_acc/max_jerk） |
| 全 PVA 的代价（`pva1` 两趟） | 航点最准：**W1/W1b 19/19，最差 0.13 m**；但 **W3 挂**（真值冲到 7.09 m；第二趟 bias **+1.176 m**），W6 0.87–1.16。~~机制：ArduPilot 把加速度不整形加到速度目标上 ⇒ 被 z 向加速度顶起来~~ **这个归因已被第 85 轮的 a_z=0 实验推翻：**`az0` 两趟（a_z 强制 0）仍然一趟 **80.89 m**、一趟 2.76 m；异常时 FCU 日志里有 `SIM Hit ground`、`EKF variance/Failsafe`、`Potential Thrust Loss`，tap 与 FCU 同时被带走、LIO 不动 ⇒ 是 **EKF/SITL 失稳**，与加速度通道无关（详见 `docs/INTEGRATION.md` §63）。加速度上线确实会让 FCU 的 z 目标偏离命令（14.6 % vs 1.1 % 的样本高于命令 0.5 m），但它不是那些跳变的成因 |
| **命名消歧** | 规划器配置 **`config/super_lio_v2_08.yaml` → `config/super_planner_hex.yaml`**（旧名把 SUPER 规划器和 Super-LIO（SLAM）混在一起）；两份安装副本 + 11 个引用脚本 + CHAIN/README 同步，三份 md5 `ad9855c2…`，旧名在活文件里零命中（只剩改名脚本与配置头的说明）；**Super-LIO 的配置是 `lio_avoid/lio/config/livox_360.yaml`** |
| **新增 H5：真值源自洽闸** | `accept.py` 比较 tap 与 FCU 自己估计的**空间跨度**（不需对时）：正常趟 8.11 vs 9.31 m（PASS），异常趟 **293.9 vs 8.6**、**114.5 vs 8.6（垂直 410.6 vs 3.6）**、**591.1 vs 9.8** 均 FAIL ⇒ 该趟标“不可评分”。**本轮 4 次 `ABORT: geofence` 里有 2 次能确定是假警报**（tap 报 410 m，FCU 只有 3.6 m） |
| **收尾两趟 `pva2`** | `pva2-01-02`（H5 PASS）**8 过 4 败**：W1 6/19、W1b 7/19、W3 PASS +0.118、**W4 −0.079 m**、W6 1.308；`pva2-02-02` H5 FAIL（不可评分）。PVA 三路确实进了飞控：FCU 的 `DA` 有 **71.8 %** 样本非零、峰值 11.7 m/s² |
| **supervisor 侧的同一道闸（最终版）** | `TapTruth` 记“隐含速度 > 30 m/s 的瞬移”；`geofence_verdict` 在两种情况下**抑制 geofence abort**：① 刚发生瞬移（2 s 内）；② 它与 **FCU 自己的位置**（`/fcu/odom`，需新鲜 <0.5 s）相差 > 2 m。实飞证据：`pva4-01-01` 的 supervisor 日志里连续打出 `geofence suppressed: the truth stream is 16.7 / 34.6 / 48.5 / 61.4 / 79.1 / 100.3 m from the FCU's own estimate` ——**那一趟因此没有被误杀**，而在此之前它会在第一个 16.7 m 就被 abort。离线五条测试全 PASS（`lio_avoid/src/tools/test-teleport-guard.py`） |
| **新的头号堵点：真值流自己在漂** | 最近 8 趟里 **6 趟 H5 FAIL**（tap 与 FCU 的空间跨度差 251–697 m）。这意味着这些趟的 W0b/W3/W4/W6 **不能用来说飞机好坏**——它们读的是一个正在漂移的坐标系。下一件事就是修真值流（tap 的“origin = EKF origin”与 EKF 重设原点的交互） |
| **第 86 轮：对比测试地图（无柱 / 矮柱）** | 用户："做一个 compare_map，[-7,7] 里没有柱子，飞一遍看看行不行"。做了两张、各飞一趟：`worlds/compare_open.yaml`（[-7,7]² 全空，56/250 柱）**失败**——`H5 FAIL`，**FCU 自己的估计跨度 25 972 m**，空场没有特征 ⇒ LIO/EKF 散掉，空场测的是估计器；`worlds/compare_low.yaml`（250 柱全留、截到 **z1=1.0 m**，低于 2.0 m 飞行高度）**9 过 3 败**：**W1/W1b 19/19（最差 0.28 m）**、W2 PASS（max 4.91）、高度偏差 **+0.019 m**、H5 PASS。同一套栈在冻结图上 MINCO 失败 **3759**、参考冻住 **76.5 %**，矮柱图上只有 **27** 次、**16.5 %** ⇒ **"一团糟"的主因是柱林对规划器的压力，不是控制回路**。剩下的 W3（垂直 6.03 m excursion）、W4（矮柱图下只剩"下降越过树桩"的度量伪影，>1.5 m 无接触）、W6（跟踪本底 1.9 m）见 `docs/INTEGRATION.md` §66 |
| **为什么现在超调明显、以前没有** | 不是飞机变差了，是**到达判据变了**（SPEC S1 第 81 轮：" 经过航点、不驻停"）。第 79 轮臂：`ARRIVE_M 0.15` + `SETTLE_SPEED 1.0` + `SETTLE_S 2.0` ⇒ 实质**停住再走**，实测到角点时速度 **p50 0.34 / max 0.49 m/s**、距角点 0.01–0.08 m；现在：`ARRIVE_M 0.50` + `SETTLE_SPEED 6.0` + PVA 出弯 2 m/s ⇒ 实测到角点时 **p50 2.12 / max 2.74 m/s**、出弯后弦线弓高 **p50 0.76 / max 1.54 m**。而**弓高就是 `R = v²/max_acc`**（5.0 m/s² 下：2.05→0.84、2.12→0.90、2.59→1.34、2.74→1.50 m，与实测 0.76/0.86/1.04/**1.54** 一一对应）。第 79 轮之所以看不出来，是宽世界的 **2.0 m 车道**把同一类外冲吸收了（那一趟真值最远冲出任务方框 **2.4 m** 仍 11/0 通过） |
| **`max_acc = 8`（第 86 轮用户指定）实测** | `traj_opt.boundary.max_acc` 5.0 → **8.0**（三份 md5 `581461ca…`，`spec_check` 新增一行 PASS）。同一张冻结图、只改这一项：过点速度 **2.30 → 2.77 m/s**、出弯弓高 **1.51 → 1.12 m（−26 %）**；两趟都是 **5 过 7 败**，**W4 −0.091 / −0.093 m** 不变。原因写在 §68.2：弓高 = v²/a，抬 `max_acc` 的同时 MINCO 也允许更快的过点速度，净收益被打折。按 `v²/8`，要弓高 ≤0.145–0.245 m 需过点速度 ≤**1.08–1.40 m/s**，而现在是 2.77 |
| 全流程链路图 | **`docs/CHAIN.md`**：14 段 topic 清单（内容/名义/实测频率）、三个 mask、SPEC+薄层+SUPER+FCU+判据五层的判定参数、名义与实测不一致的 7 项、一条真实时间线，以及本轮改过的文件清单（"有没有偷偷搞破坏"） |
| **A/B（同地图、同臂、同 `robot_r`，只改这一项）** | `pos`：3 / 11 / 3 / 2 个航点、参考冻结 76.5 %、四次 geofence abort｜**`posvel`：两趟都 W1 18/19 + W1b 19/19（真值最差 0.17 / 0.22 m）+ 8 过 3 败 + rtf 1.0002 / 1.0003**，一趟 `RUN_EXIT=0` |
| **降速有没有用（用户质疑的那一条）** | `max_vel` 5.5 → **2.0**（出弯本来就是 2.0），两趟 `soft`：参考确实被压到 **max 2.01 / 2.04 m/s**（之前 4.06–4.33），但 **W4 纹丝不动：−0.089 / −0.092 m**，航点反而 10–13/19 → **用户的直觉对：不是速度** |
| 为什么不是速度（闭环频响） | `scripts/loop_response.py`：同一套增益在**两趟里都**在 **0.2–0.5 Hz 放大 1.73 / 1.86 倍**；差别在参考主频：第 79 轮宽车道 **0.16 Hz**（|H|≈1，沿航迹增益 1.04）0.00 s 滞后）vs 冻结六边形 **0.30–0.67 Hz**（增益 1.73–1.98，0.10 s 滞后）。降速按 f=v/λ 只能把主频往下挪（1 m/s → 0.42 Hz，仍在共振带） |
| 温和增益的效果 | `params/guided_soft.parm`（PSC_NE_POS_P 3.0→1.5、VEL_P 3.0→2.0、ATC_ANG 6.0→4.5、WP_SPD 2.5）：**W6 跟踪 0.76–0.87 → 0.41**（砍一半），**W4 仍 −0.089**；两趟都 7 过 4 败 |
| **W4 的真正地板** | `scripts/plan_clearance.py`：**规划自己**的轨迹对柱面的净空 p50 **0.329**、p10 **0.234**、**min −0.078 m**（0.3 % 的样本在柱体内）；`scripts/plan_attribution.py` 说 19 条轨迹里 **9 条**都有（每条 1–6 个样本≈0.1 s）⇒ **MINCO 的样条在 SFC 约束点之间鼓出几厘米**，上游数值细节，与速度/增益/`robot_r` 无关 |
| W4=0 能不能做到 | **这张冻结地图里不能**：要求"规划净空 ≥ 跟踪误差"（≥0.4 m），而最紧三条腿的弯道止于 **b = 0.245 m**。四个选项见 `docs/INTEGRATION.md` §61.7（A 改判据、B inflation 2、C 重开地图、**D 修 SUPER 的 MINCO 约束采样**） |
| 剩下的三行是什么 | **W4 −0.089 / −0.098 m 接触**（弯 0.39 m + 净空 0.145–0.245 m，见上一行）；**W1 18/19**（薄层自己的到达记账少一个，真值侧 19/19）；**W6 0.76–0.87**（它比的是整段"记录下来的规划"与真值，参考在冻/重启） |
| 附带发现（安全） | `r10-02-02` 收尾时 **tap 与 FCU/LIO 差 338 m**、`az0-01-01` 410 m、`pva3` 697 m。**机制已定位（`scripts/tap_frame_forensics.py`）：tap 没说诞 —— 它的位置与速度逐样本自洽（比值 **1.0x**），是 **SITL 的物理模型被抛飞（19 m/s）而飞控 EKF（吃 LIO 外部导航）留在场内**；每一趟都带 `SIM Hit ground` / `EKF Failsafe` / `Potential Thrust Loss`。所以那些 `ABORT: geofence` 与那些 W 行都不是飞机的表现（新增的 **H5** 与 supervisor 的两道闸会把它们标成不可评分/抑制 abort）。正常趟里 tap 可信（`d85-01-02` tap vs LIO p50 **0.011 m**） |
| SPEC 自检（`pv1-02-02`） | S0 **PASS**｜S1 过 0.5 m 球 **PASS 18/18**（`spec_flight` 最差 0.203 m）｜S1 不停车 **FAIL**（参考在 18 个航点处为 0）｜S2 **PASS**｜S3 **FAIL**（1.83 m/s，>4 m/s 占 8.8 %）｜S4 **PASS**｜S5 **PASS**｜S6 **PASS**（两趟都报） |
| 图 | 这一轮**没有重画**（等三行结论定下来再出图）；上一轮的 `d85-*` 六张还在 `docs/evidence/` |

#### 0.1 第 84 轮（历史：出弯速度降到 2 m/s，四趟 attempt 全败）

> ### ⚠ 两条硬约束（用户第 81 轮）：`docs/SPEC.md` 是唯一有效的规矩；**地图与航点已冻结**
>
> `scripts/check_frozen.py` 把世界文件的 SHA-256（`a5a43f8f…`）、250 根柱、六个航点坐标写进代码，
> `scripts/fly_hex2.sh` **每次起飞前跑它**，任何一条不过就拒绝起飞。第 84 轮没有改动地图或航点（8/8 PASS）。

| 项 | 结果 |
|---|---|
| 这一轮改了什么 | **只有一个数字**：SPEC S1 的航点 PVA 速度 **4.0 → 2.0 m/s**（用户："把出弯速度改2m/s"）。生效位置是**臂**（`scripts/fly_hex.sh` 的 `CORNER_SPEED`，默认 2.0），**不是**世界文件（那里 `mission.corner_exit_speed_mps: 4.0` 是冻结时的历史值）。SPEC S1 + 修改记录第 14 行已写；`spec_check.py` 的 S1 改成校验臂、把这行当 INFO 打出来 |
| 会话 `d85`（SPEC S6 两趟） | 4 次 attempt，**0 次过闸**：`d85-01-01` geofence abort（199 s，真值 (−10.51,−4.14)）｜**`d85-01-02` 唯一可评分**（rtf 0.9861，跑到 harness 超时）｜`d85-02-01/-02` 两次都在 **30 s** 时 geofence abort（真值 (+10.50,−2.58) / (+10.53,−2.63)，几乎同一位置，可复现） |
| 十一行（`d85-01-02`，可评分那趟） | **6 过 5 败**：W1 **8/19**（到的是 1,2,3,4,7,8,9,10）、W1b 13/19（最差 **3.33 m @w5**）、W2 p50 0.26 / p95 2.29（>4 m/s 仅 **0.70 s**）、W3 bias −0.098 PASS、**W4 −0.065 m 接触**（0.30 % 采样在柱体内）、W6 p50 0.148 / p90 1.055 / max 3.084 |
| 参考本身 | **74 % 的时间速度为 0**（50 段停车、间距 p50 1.69 s；mean 0.34 m/s、max 4.06 m/s、指令里程 111.3 m / 264.6 s）；真值 291.8 s 只飞 **163.7 m = 0.56 m/s** |
| 为什么更差 | `inflation_step 1` ⇒ 地图膨胀 **0.10 m < `robot_r` 0.15 m**。CIRI 因此变轻松（**1003 → 21** 次失败），但 A\* 顺着离柱面 0.178 m 的种子线抄、多面体贴着柱子长，MINCO 还要在里头转出 **2 m/s 的终点速度** ⇒ **MINCO 失败 476 → 3759 次**、PlanFromRest 3761 次 ⇒ 薄层发 HOLD ⇒ 参考冻住。**出弯速度不是瓶颈，走廊净空才是** |
| 第二趟为什么吓人 | 参考只上线 **12.5 s**（694 个 setpoint，规划包络最大 5.03 m）；据掉指令的那一瞬三路还互相认得（tap (5.10, −0.60) / FCU (4.25, 0.21) / LIO→W (4.51, −0.17)，互差 ≤ 1.2 m）。之后 tap 单独记录到一次很猛的位移：航向 2 s 转 105°、滚转贴住 **45°**、速度到 **11.19 m/s**（峰值 14.75）、到 30 m 外被 geofence 抓住——**但那一段没有 FCU/LIO 记录**（setpoints.csv 只覆盖 relay 开着的 13.9 s），而且这趟的 tap 自己 gravity 检查 FAIL、position span 报 84.5 m。所以能钉死的只有一句：**那段位移不是规划发的**；它是不是真的发生过，要读 `flight.BIN` 的 attitude/desired 才能定 |
| 估计器 | 仍然不是问题：LIO vs 真值水平 p50 **0.011 m**（第一趟）/ **0.004 m**（第二趟），`nav_hz` 46.9（配置 48） |
| 图（第 84 轮） | `d85-01-lap1-paths.png` `d85-01-lap2-paths.png`（分圈俯视图，两圈）｜`d85-01-vxyz-t.png`｜`d85-01-lio-err-t.png`｜`d85-02-vxyz-t.png`｜`d85-02-lio-err-t.png`。**上一轮的 `d84-*` / `r15b-*` 七张已按用户要求删除**（"旧图记得一定要删除"），存档在 `_evidence_archive_round83.tar.gz` |
| 结论 | 单改出弯速度解决不了问题：相对"角点停"那一趟，**冻住的时间 26.5 % → 74 %、真值均速 1.81 → 0.56 m/s**。要往前走只有两条路，都要用户点头：**A** `inflation_step` 回 **2**（余量 +0.05 m；§58 实测 MINCO 576 次、CIRI 96 次）或 **B** `robot_r` 降到 **0.10 m**（与 10 cm 膨胀对齐） |
| 仍然互斥 | S3（均速 > 4 m/s）与 W4（零接触）在冻结地图里仍然不可能同时成立：最紧三条腿 **b = 0.245 m**，实测横向 \|p90\| 0.31 m（详见 §57.9、§58） |

#### 0.2 第 83 轮（历史：robot_r 0.20 → 0.15 那一批，出弯速度还是 4 m/s）

> （硬约束同上，不再重复）

| 项 | 结果 |
|---|---|
| 仿真几何（SPEC S2：下方 0.2 m / 上圆 0.2 m） | **改好了并验证**：方柱对角 (5−r)√2 = 6.929646 m、圆柱 6.971068 m、差 0.041421 = r(√2−1)；20 000 条射线全部命中点都在表面上；`cast()` vs 标量参考一致；`--selftest` / `--prune-check` 全 PASS |
| 仿真性能 | **从 234–393 ms 降到 52–60 ms/帧**（float32 + **按方位角窗口**只算候选射线；两次把 culled cast 与标量参考比对的压力测试抓出自己的两个 bug 并修掉）；修掉"`best_t=inf` 的射线从不参与求交"的真 bug；`run-mission.sh` 把 BLAS 线程钉成 1（默认线程池实测 1422 ms/帧） |
| 仿真实际速率 | 整机一起跑 **8.5–9.4 Hz**（`c10-01-01` 9.07 Hz；`r15a-01-01` **9.35 Hz**、cast 62 ms、每帧 20 000 点）——**仍不是 10 Hz**，日志里照实写 |
| 航线来源 | 新增 `--tour world`：航线从**世界文件的 `mission.corners`** 读，地图冻结 = 航点冻结 |
| PVA 接通 | `goal_pva_en: true` + `/planning/click_goal_pva`；不开这个开关 SUPER 会把终点速度强制清零 |
| 验收口径 | `accept.py` 的 W1b 目标环改成**从世界文件读**（六边形不再被四个方角评）；`WBOX=0.5` 对齐 SPEC S1 的球 |
| **能不能飞**（`robot_r = 0.20` 时期的阶梯） | **4 m/s：4 趟全部 0 个航点**（MINCO 不可行）｜**3 m/s：只到 leg2 超时**｜**2 m/s：卡在 leg4**｜**1 m/s：3 个航点后 CIRI 失败循环**｜**0（角点停）：19/19 全到**。**robot_r 0.15 之后**：4 m/s 变成 3156 次 MINCO 失败、2 m/s 变成 576 次 + 跑飞（见下面的第 83 轮表） |
| **卡在哪** | `robot_r 0.20`：卡**走廊**（`[CIRI] maxVolInsEllipsoid failed`，3814 条种子线 98.6 % 余量为负）。**`robot_r 0.15`（第 83 轮）之后走廊失败 = 0**，改卡 **MINCO**（4 m/s 终点速度在 ±0.1 m 走廊里转不过来）与**跟踪**（飞机偏出规划 6 m） |
| 附带发现（安全） | 规划停住后薄层发 hold，但**在 GUIDED 里拿着停住的 setpoint 飞机仍会漂出去**（`c10`：真值到 (−12.3, −10.5)，被 10.5 m geofence 抓住）。**规划停顿本身就是危险状态**，别当控制问题查 |
| 诊断趟 `d84-01-01`（0 出弯，**不是验收**） | 9 过 2 败：W1b **19/19**（最差 0.18 m）、W2 15.47 s >4 m/s、W3 PASS；**W4 −0.094 m 接触**、W6 零滞后 p50 1.17 m / 时基 spread 0.42 s |
| 为什么接触 | 冻结地图最紧三条腿的净空 **b = 0.245 m**（= 中心线两侧各 ±0.12 m），而实测**横向**误差 \|p50\| **0.074–0.090 m**、\|p90\| **0.306–0.317 m**（**没有系统偏置**：p50 +0.01）⇒ **约 37 % 的时间超出 ±0.12 m**；真正大的是**纵向滞后** p50 0.06–0.23 m，它不直接威胁柱子。`contact_where.py` 定位到的接触点：参考已贴到 0.10 m、真值又偏 **0.479 m**（那一趟有 1760 次规划失败，参考在停），离地时间 **2.4 % 在柱体内 / 31 % 小于 0.245 m** |

**图（第 83 轮那一批的标题与结论；文件已删，第 84 轮的 `d85-*` 在本节开头）**

* `d84-lap1/2/3-paths.png` —— **分圈俯视图**：红 = SUPER 规划的路径、蓝 = 实飞轨迹；标题给本圈的规划/真值最小柱面净空与 |规划−真值| p50/p90（圈边界实测）；
* `d84-vxyz-t.png` —— **v_xy / v_z / x,y / z 对时间**；
* `d84-lio-err-t.png` —— **估计器误差对时间**：水平 p50 0.018 / p90 0.044 m、垂直 p50 0.014 m ⇒ **估计器不是瓶颈**。

**这七张已按用户第 84 轮的话（"旧图记得一定要删除！"）删掉**，打包在
`D:\distill_self\_evidence_archive_round83.tar.gz`（3.19 MB）；`scripts/check_evidence_links.sh` 会断言它们**不在**。

**第 83 轮：`robot_r` 已按用户指示改成 0.15 m**（`scripts/set_robot_r.sh 0.15`，装到两处 config、md5 一致；
`docs/SPEC.md` S4 与修改记录已更新；地图与航点**未动**，`check_frozen.py` 8/8 PASS）。

实测（两趟，`r15a` 出弯 4 m/s、`r15b` 出弯 2 m/s）：

| 阶段 | 4 m/s | 2 m/s |
|---|---|---|
| **CIRI 走廊失败**（原来 1784–3814 次） | **0** | **96** |
| MINCO 失败 | 3156 | 576 |
| PlanFromRest 失败 | 3155 | 651 |
| 结果 | leg2/leg3 TIMEOUT | 到 27.8 s 时 geofence abort（真值 +10.52, −2.10） |

**结论**：`robot_r 0.15` 把走廊那一步彻底解决（§57.11 预测的 98.6 % → 0.5 % 得到印证）；
现在卡在 **MINCO（4 m/s 的终点速度在 ±0.1 m 走廊里转不过来）** 与 **跟踪（飞机偏出规划 6 m 跑飞）**。

**跑飞不是估计器**：同一趟的 LIO vs 真值水平 p50 **0.005 m** / p90 0.016 m，外部导航馈送 47 Hz
（配置 48），规划的位置包络一直在场内（x ≤ 4.22）—— 是飞机自己飞出去的，`EKF variance` 的时间戳在
geofence abort **之后**。

**下一步该调的是速度包线（臂参数，不动 SPEC 的数字）**：`REF_MAX_MPS`（现 6.0）、FCU `WP_SPD`（6.5）、
`ATC_ANGLE_MAX`（45°）、出弯 2 m/s。第 79 轮能稳定飞的是 4.5/2.0 m/s 级别的包线，6 m/s 从来没稳过。
⚠ 但 S3（全程均速 >4 m/s）与"更温和的包线"直接冲突 —— 这一点要用户知道。
**第 84 轮就是按这条做的（出弯 2 m/s）：没有救回来** —— 参考冻住的时间 26.5 % → 74 %，四趟 attempt 零通过（见本节开头与 `docs/INTEGRATION.md` §59）。

> **下面第 79 轮的内容保留为历史**（它用的是 2.0 m 车道的变体，已按新规矩作废，不再作为结论）。

> **v2 柱林（第 77–79 轮，见 `docs/INTEGRATION.md` §54）**：0.8 m 格 × 20×20、柱截面 0.2 m、
> ~~安全半径 0.2 m~~（**该项已被用户删除，见 §56.1；下面这段历史里所有涉及它的数字都不要再引用**），
> **柱心在每个格子内随机**（偏离格心中位 0.37 m，**没有一根**在格心）。两张地图，一张是规格、一张是飞行合格版：
>
> | 地图 | 柱子 | 任务四边车道 | 飞行结果 |
> |---|---|---|---|
> | **`worlds/forest_v2_08.yaml`（规格版，用户指定，未再改动）** | 391 | 0.96 m | 13/13、W4 在 −0.08…+0.23 之间摆（**不可复现**）、W6 0.62–0.80 → 10/11 |
> | **`worlds/forest_v2_wide.yaml`（飞行合格版，新文件）** | 233 | **2.0 m**（移空紧贴每条腿的一排格子） | **11/0 GATE PASS × 7 趟**（共 9 趟） |
>
> **默认臂就是通过的那一套**（`bash scripts/fly_v2.sh <tag>`，所有值都可用环境变量覆盖）：
> `PASS_BY_M=0.5 PASS_BY_LONG_M=1.0 HOLD_S=0`（角点**滚过去**、不驻停）+ `ARRIVE_M=0.15 SETTLE_SPEED=1.0
> SETTLE_S=2.0`（真到点）+ `ENTRY_MAX_MPS=2.0`（发射段限速）+ `params/guided_limits_fast.parm`
> （45° / 10 m/s² / WP_SPD 6.5 / P 3.0 / VEL 3.0 / JERK 150）+ `super_planner_hex.yaml`
> （`hot_start_en=true`、`backup_traj_en=false`、`planning_horizon=16`、`max_vel/max_acc 5.5/5.0`、A* 网格 480）。
>
> **7 趟 11/0**（W4 最小柱面）：`v3p-01` +0.600、`v3w-01` +0.544、`v3t-01` +0.541、`v4a-01` +0.535、
> `v3r-01` +0.486、`v3v-01` +0.475、`v3q-01` +0.428；**2 趟失败**都是 W6 = 0.61 / 0.65（判据 0.60），
> 其余十行全过。
> 通过那几趟的真值均速 1.5–1.6 m/s、真值峰值 4.28–4.73 m/s、>4 m/s 持续 2.2–4.8 s、13/13 航点且
> 真值几何复核最差 0.05–0.09 m。
>
> **为什么规格版过不了**（§54.7 有 22 趟的参数搜索表）：**90° 转弯半径 v²/a = 1.1 m 是 0.48 m 车道半宽的
> 2.3 倍**——滚过角点就出车道进柱排（擦碰 −0.07…−0.09，可复现），停在角点就 W6 超标（角点段 p90 0.95 m）。
> 宽车道（2.0 m）把这两件事同时解决：转弯装得下、跟踪余量够。
>
> 图：第 83 轮已按用户要求删除（看 `docs/evidence/README.md` 与 `_evidence_archive_round83.tar.gz`）。
>
> 下面 `## 1` 之后描述的是**早期架构**（`sitl_odom_relay.py` + `waypoint_mission` +
> `scripts/run_and_score.sh`、12 个航点**盒**）。现在跑的是 **lio_avoid harness + 薄层**
> （`src/whtf/ros/planner_source_super.py`）、柱林世界、`scripts/accept.py` 的十一行判定。
> 旧文保留（是集成过程的历史，其它文档引用它），但**以本节为准**；
> 完整记录见 `docs/INTEGRATION.md` §22–§54 与 `docs/PIPELINE.md`。
>
> **文档（第 79 轮已清理）**：`docs/` 只保留三个文档 —— `INTEGRATION.md`（完整记录 §0–§54）、
> `PIPELINE.md`（链路与状态机）、`MISSION.md`（任务与场地）；图在 `docs/evidence/`（索引见其
> `README.md`，第 79 轮只留 6 张图 + 3 张数据表）。旧文档（`STATUS.md`、`RECON.md`、`HANDOFF-ego.md`、
> `SPEED*.md` 共 18 个）与 v1 世界的旧图已打包到仓库外
> `D:\distill_self\_docs_archive_round61.tar.gz`（5.2 MB）与 `_evidence_archive_round79.tar.gz`。

| 指标 | 目标 | 实测（v2 柱林，`v4a-01`） | |
|---|---|---|---|
| 13 个目标（竖直起飞腿 + 12 角点 × 3 圈） | 13/13 | **13/13**，真值几何独立复核 `W1b` 最差 **0.06 m**（盒 1.5 m） | ✅ |
| 逐柱零接触（W4 最小柱面间隙，方盒口径） | > 0 | **+0.535 m**（v3p +0.600、v3w +0.544、v3t +0.541），0.00 % 采样进入柱体 | ✅ |
| GATE PASS（`accept.py` 十一行） | 11 过 0 败 | **11/0**：`v4a-01`、`v3w-01`、`v3v-01`、`v3t-01`、`v3p-01`、`v3q-01`、`v3r-01`（**7/9 趟**） | ✅ |
| 巡航速度 ≥4 m/s 且持续 ≥1 s（W2） | ≥1.0 s | **3.18 s**（真值峰值 5.06 m/s） | ✅ |
| 高度（W3 bias / p95 偏移） | ±0.30 / ±0.80 m | **−0.084 / +0.39 m** | ✅ |
| 规划-真值跟踪（W6，去时间基后 p90） | ≤ 0.60 m | **0.48 / 0.52 / 0.48 / 0.59**（四段） | ✅ |
| 全程平均地速 | ≈5 m/s | 1.49–1.56 m/s（角点要滚得慢，见 §54.7） | ❌ |

| v1 世界的历史对照（`sc-01` / `se-01` / `sj-02`，84 柱） | | |
|---|---|---|
| 13 个目标 | 13/13 | **13/13**（W1b 最差 0.27–0.50 m） |
| W4 最小柱面间隙 | > 0 | **+0.192 / +0.171 / +0.108** |
| GATE | 11/0 | **11/0** |

#### 角点不停车（第 65 轮，`scripts/fly_pregoal.sh`）

SUPER 每条轨迹的终点速度恒为 0，而 pacer 只在到达后才发下一个目标 → 每个角点必停。
**离角点还剩 2–4 m 时就把下一个角点的目标发出去**（薄层 `--pre-goal-m`），当前角点于是成为轨迹的
**内部点**，不再停车；A\* 只有这么点余量，抄不出大弯（`--goal-lead 1` 在 15 m 外换目标，抄掉 8–9 m、W1b 7/13）。

| 臂 | 平均地速 | 参考在角点归零 | W4 净空 | W2 | 判据 |
|---|---|---|---|---|---|
| `sc-01`（每角停，规划 4.2/3.6） | 2.13 | **10 次 / 2.3 s** | +0.192 | 1.05 s | 11/0 |
| `pgb-01`（pre-goal 2 m，4.2/3.6） | 2.33 | 4 次 / 0.2 s | +0.215 | 0.46 s | 10/1（W2） |
| `pge-01`（pre-goal 4 m，5.2/4.8） | 2.47 | —— | +0.159 | ✅ | 11/0 |
| **`pgf-01`（pre-goal 4 m，5.6/5.4）** | **2.62**（去停机坪悬停 **2.75**） | **0 次 / 0.0 s** | **+0.275** | ✅ | **11/0 PASS** |
| `pgg-01`（6.2/6.0） | 2.68 | —— | +0.153 | ✅ | 10/1（**W6**） |
| `pgh-01`（6.2/6.0 + 横向增益 3.0） | 2.60 | —— | +0.135 | ✅ | 11/0 |

**角点 12 次停车 → 0 次**，均值 **2.13 → 2.62（+23%）**，`pgf-01` 的净空是 **+0.275**，W1b 仍 13/13（最差 0.44 m）。
规划器抬到 6.2/6.0 时 W6 破，所以上限取 **5.6/5.4**。

> **⚠ 净空还不可复现**（第 67–68 轮）：同一套配置的 `pgl-01` 是 **−0.058 m 接触**（跟踪向内偏 0.262 m，
> 而参考当时只剩 +0.204 m）、`pgq-01` 与 `pgo-01` 参数完全相同却从 +0.246 掉到 **−0.001**。
> **20 趟全族统计：7 趟 GATE PASS、4 趟毫米级接触（−0.001…−0.107）、其余死在 W6**；W1b 全部 13/13。
> 站得住的结论只有 **W4 ↔ W6 = −0.55…−0.62**（净空与跟踪同步变差，同一个“跟踪质量”驱动）；
> 第 68 轮曾把方差归因到 LIO 的 z 跨度，第 69 轮复查发现**那条相关性主要来自一个离群点**（去掉后 +0.65→+0.26），已更正。
>
> 所以能稳定作为验收结论的仍是**停-走档**（`sc-01/se-01/sj-02`，3/3 全过，2.08–2.19 m/s，+0.108…+0.192）；
> 角点不停族的最好纪录是 `pgf-01` **2.62 m/s / +0.275 m**（11/0 PASS）与 `ptc-01` **+0.281 m**。详见 §46–§50。
>
> "现实上界"按实测分解约 **3.26 m/s**（角点区占 29 % 路程、2.1 m/s；直线 4.2）；参考**路径本身**的
> 时间最优是 **3.53 m/s**，规划器只给 2.89。第 69–71 轮把"按倍速重放 SUPER 自己的多项式"这条路
> 查到底：多项式已能 100 Hz 新鲜发布（`scripts/patch_poly_at_cmd_rate.py`），但**多项式 τ=0 的点是
> 轨迹起点（飞机当时位姿），与"当前指令点"差 0.43 m**（沿航迹滞后），两者无法互相锁定。
> 要走完这 22 %，需要让 planner 内部把"求值用的 τ"暴露出来——更深一档的改动，已记录在 §50.3。

图：那几张（`pgf-01-*`、`sc-01-*`）第 83 轮已按要求删除，归档在 `_evidence_archive_round83.tar.gz`；重画用 `scripts/plot_lap_paths.py`、`scripts/plot_vz_t.py`。

#### 为什么均值到不了 5 m/s（四条上界，离线 + 实测都有）

| 上界 | 数值 | 依据 |
|---|---|---|
| 几何 | 5.82 m/s | 最大合法圆角 R 2.86 m、直线无柱的理想值 —— `scripts/corner_space.py` |
| rest2rest | 3.65 m/s | 12 条 15 m 的腿、每条都以静止结束（解析） |
| **跟踪（第 58 轮新增）** | **2.5–3.5 m/s** | 紧处偏差 p90 从 2.2 m/s 的 0.34 m 涨到 3.2 m/s 的 0.48 m；在 5 m/s 处外推吃掉 **0.75 m**，比整个世界自由半通道（0.665 m）还宽 —— `scripts/deviation_census_space.py`，r 0.75–0.81 |
| **零接触实测** | **2.46 m/s** | 46 次有效评分飞行的前沿：`frontier.png`（已归档；重画 `scripts/speed_frontier.py`） |

最快**有效**飞行 3.20 m/s（`rp-01`，但净空 −0.099 m，必碰）。
另外，**"撤掉那 12 次角点停顿"这条最后的路也已被实测封死**（`missionA-sf-01`，第 53 轮）：
在上限 4.2/3.6（跟踪有余量、W6 通过）上改成动中交接，角点处的**向内偏差从 0.13 m 跳到 0.468 m**
（`closest_command.py`：指令净空 +0.424 m、真值 −0.044 m，而规划自己 0 % 的样本贴近边线），
于是接触；而且它**并没有变快**（角点停留仍 36.4 %、均值仍 ~2.1）。结论：**角点上的停不是浪费，
它就是净空本身** —— 停的时候飞机落在世界审计过的角点圆盘里，不停的时候走的是自己切的圆弧。
要越过 2.46 只有三条路，且都已用证据关掉：加宽柱距（用户排除）、加大规划净空（会把 1.55 m 柱间
通道挤到 0.02 m）、换参考接口（给飞控完整轨迹而非点，未实现）。详见 `docs/INTEGRATION.md` §28.6。

#### 净空到底是什么（第 58 轮：46 趟逐点分解）

净空是一条减法：**参考在那一处给的余量 − 跟踪在最坏那一点吃掉的量**。

| 量 | 数值 |
|---|---|
| 紧处（真值净空 <0.6 m）平均向内偏差 | **+0.088 m**（p10 +0.038）—— 很小，从来不是机制 |
| **单趟最深偏差**（在 W4 自己那个最紧点） | 中位 **+0.299**，p90 **+0.534**，最大 **+0.632** m |
| 参考在那一处的余量（中位） | **+0.451 m** |
| 验收档 | **0.45 − (0.28…0.36) = +0.11…+0.19 m**，与实测 W4 +0.108/+0.171/+0.192 逐位吻合 |

把 15 次接触按"谁的错"分类：**7 次是发出去的参考路径本身钻进了柱子**（最小间隙到 −0.110 m），
**8 次是参考干净、飞机自己偏出 0.28–0.63 m**，**0 次落在两者之外** —— 判据完备。
参考路径本身的体检结果必须记下来：**46 趟里 12 趟参考穿进柱面、35 趟低于规划自己的 0.325 m 边距**；
而**验收档的参考是完好的**（sc-01 +0.315、se-01 +0.347、sh-01 +0.346），说明这条链没有系统性配准误差。
图：`deviation.png`（已归档；重画 `scripts/deviation_figure.py`）；数据：`docs/evidence/deviation_space.tsv`。

#### 2.46 与 GATE PASS 之间只差一行，而那一行是**时间**（第 59 轮）

零接触最高均值 `pm-01`（**2.46 m/s**，净空 +0.117）**只死在 W6 一行**（W4/W2/W1b/W3 全过）。
把 W6 的残差投影到规划路径的切向/法向（`scripts/w6_decompose.py`）：

| run | p90 \|r\| | 沿航迹 rms（时间） | 横向 rms（路径） |
|---|---|---|---|
| sc-01 验收档 | 0.497 | 0.250 | 0.192 |
| pi-01（PASS 最快） | 0.550 | 0.277 | 0.178 |
| **pm-01（零接触最快）** | **0.647** | **0.340** | **0.216** |

**沿航迹比横向大 1.3–1.6 倍**：W6 追的是"早一点/晚一点"，而不是"飞偏了"。
从 2.0 到 2.5 m/s，横向只涨 12 %（0.192→0.216），沿航迹涨 **36 %**（0.250→0.340）。
所以 W2（要峰值）与 W6（怕峰值）共用一个旋钮，`(4.2, 3.6)` 周围只有 1/6 通过是**斜率**不是噪声。
要把 2.46 变成 PASS 需把时间抖动降 ~20 % —— 那是飞控环/接口的事，规划改不了。

#### 这个通过率可不可信？（第 56 轮的排除性检验，第 57 轮按真值重算）

W6 的残差是"运输延迟 × 速度"，所以"通过率"必须排除主机负载这个混淆项。
`scripts/w6_vs_host.py` 对 48 次有效评分计算（第 57 轮起有效性用**真值** `W1b` 判）：W6 与
**rtf 的相关系数 +0.038**、与 LIO 交付频率 +0.104、与点云频率 +0.074；通过组 rtf 均值 **0.9977**、
失败组 **0.9982**（失败组反而略高）。
**结论：W6 的通过/失败不是主机造成的**，是这条链（10 Hz 点云 → LIO → 50 Hz 桥 → GUIDED）在
2.2 m/s 附近的固有抖动。

#### 验收工具本身被查过吗？（第 57 轮）

`W1`（13/13）原来是**规划盒自报**的。第 57 轮用真值独立复核，发现它在 `--goal-lookahead-m > 0`
时会给**错误的通过**（抄近路的轨迹从角点旁 3–5 m 穿过那条"越线"平面，横向偏差却很小）：
54 次评分里 **7 次不一致**，最差的 `sk-01` 自报 13/13、真值只有 **4/13（最差 5.20 m）**。
于是 `accept.py` 增加**只读真值**的独立行 `W1b`，闸门由十行变十一行。

全库重评（`scripts/rescore_all.py`，结果在 `docs/evidence/rescore_w1b.tsv`）：**55 趟 → 8 趟
GATE PASS**（pg-01, ph-01, pi-01, pw-01, sc-01, se-01, sh-01, sj-02），验收三趟仍 **11 过 0 败**，
`sk-01` 被抓住（`W1b` 4/13）。同一个"自报当证据"的错还出现在**前沿图的有效性过滤**里：
换真值后有效条数 48 → **46**（剔除 4 条抄近路，又补回 2 条被 pacer **少报**的），
前沿的三个关键数字不变。逐点距离还显示**同一圈内按周期 4 重复**
（sc-01：0.41 / 0.34 / 0.20 / 0.10 复现三次），即三圈是可重复的。
`scripts/verify_waypoints.py`（单趟复核）、`scripts/w1_audit.py`（全库审计）、`scripts/rescore_all.py`（全库重评）。

#### 复现（都是单条命令）

```bash
bash scripts/fly_robust.sh <tag>     # 推荐档：计划上限 4.2/3.6 —— 该点实测 3/3 通过
bash scripts/run_robust_at.sh 4.2 4.5 <tag>   # 想 W2 宽裕就换这档，但 W6 会贴近判据线
bash scripts/fly_pi01_arm.sh <tag>   # 更快档：5.0/4.5，W2 余量更大但 W6 是掷硬币（同族 7 次过 3 次）
bash scripts/run_accept.sh <tag>     # 十一行判定（accept.py，含真值行 W1b）
python3 scripts/verify_waypoints.py /home/fanxin/lio_avoid/logs/missionA-<tag>   # 只看航点几何
python3 scripts/rescore_all.py       # 全库按十一行重评（结果 docs/evidence/rescore_w1b.tsv）
python3 scripts/deviation_census_space.py   # 净空分解：参考给多少 / 跟踪吃掉多少（§37）
python3 scripts/speed_frontier.py    # 重算全表（46 次有效飞行）+ 重画 `frontier.png`（已归档；重画 `scripts/speed_frontier.py`）
python3 scripts/check_param_names.py params/*.parm   # 灌参数前必做：名字对不上固件会静默失效
```

#### 图

仓库里**只保留最新一批**（`docs/evidence/`，9 个文件 / 1.4 MB）：

| 文件 | 内容 |
|---|---|
| **`pgf-01-lap1/2/3-paths.png`** | **每圈一张俯视图**（角点不停那一趟，11/0 PASS）：**红 = SUPER 规划路径，蓝 = 实飞轨迹**，84 柱、本圈 4 个角点、本圈最近柱面与 \|规划 − 真值\| 都印在标题上 |
| **`pgf-01-vxy-z-t.png`** | **v_xy 与 z 对时间**：规划（红虚）vs 真值（蓝/绿实）、>4 m/s 阴影（3.87 s）、13 个到达竖线、三圈底色 |
| **`sc-01-vxy-z-t.png`** | 对照（停-走档）：角点速度为 **0**、>4 m/s 只有 1.05 s |

`deviation_space.tsv`、`deviation_census.tsv`、`rescore_w1b.tsv` 是三张图/表背后的逐趟数据（索引见 `docs/evidence/README.md`）。

三张分析图（速度前沿、净空分解、状态机时序）**已归档**，一条命令可重画：

```bash
python3 scripts/speed_frontier.py         # frontier.png（46 次有效飞行 + 两条离线上界）
python3 scripts/deviation_figure.py       # deviation.png（净空给多少 / 跟踪吃掉多少）
python3 scripts/plot_pipeline_states.py   # pipeline-states.png（三个状态机时序）
python3 scripts/plot_lap_paths.py <tag>   # 分圈俯视图（红 = 规划，蓝 = 实飞）
python3 scripts/plot_vz_t.py <tag>        # v_xy 与 z 对时间
```

归档包：`D:\distill_self\_evidence_archive_round72.tar.gz`（6.2 MB，逐趟旧图）、
`D:\distill_self\_evidence_archive_round73.tar.gz`（三张分析图）。

---

### 1. 整条链（真跑起来的就是这条）

```
   ┌────────────── 规划（SUPER，原生）──────────────┐
   │                                                 │
/cloud_registered ──►  ROG-Map  ──►  A*  ──► CIRI 走廊  ──►  GCOPTER(MINCO)
   ▲  (adapter 造的世界点云)                        │
   │                                                ▼
/lidar_slam/odom ─────────────────────────►  fsm_node ──► /planning/pos_cmd (mars PositionCommand, 100 Hz)
   ▲  (relay 扮演 LIO)                              ▲
   │                                          waypoint_mission
   │                                          12 个航点、switch_dis=1.0
   │
   │        ┌─ pc_adapter ──► /plan/cmd (quadrotor_msgs PositionCommand, 50 Hz, frame=lio_world)
   │        │        │
   │        │        ▼
   │        │   lio_avoid/bridge ──MAVLink 331(外部导航) + 84(定点指令)──► ArduPilot GUIDED
   │        │        ▲                                                        │
   │        └────────┴── /lio/robo/odom, /lio/odom ◄── relay ◄── LOCAL_POSITION_NED
   │                                                                         │
   └───────────────────────── /plan/trigger（GO 闸门）◄── relay ◄────────────┘
```

两条坐标系之间**没有旋转**：relay 在 `/lio/robo/odom`、`/lio/odom`、`/lidar_slam/odom`
上发的是同一个 ENU 世界，SUPER 也在同一个世界里规划，所以 bridge 的 L→W 标定必须是
**单位矩阵**（`config/calib_identity.json`）。bridge 默认会给 +90°，那是 lio_avoid 那台的
物理事实，用在这里会让轨迹镜像 90°（15 m 正方形映到自己身上，验收会变成"看运气"）。

---

### 2. 一条命令跑通（WSL2 + ROS 2 Humble）

```bash
bash scripts/run_and_score.sh 165        # ≈4 分钟：起 SITL → 解锁起飞 → SUPER 规划 → 飞 3 圈 → 打分
```

它会依次做完：装配置 → 合并 SITL 参数 → 起 SITL → relay（写参数/切 GUIDED/解锁/起飞/发 GO）
→ 等 `/tmp/takeoff_ok` → 起 adapter → `fsm_node` + `waypoint_mission` → bridge → 计时 → 打分。

跑完输出（真实数字，原始轨迹 `sitl_trace.csv` 已归档，见 `docs/evidence/README.md`）：

```
== SITL closed loop ==
  samples             : 8250（空中 6245，125.0 s，50 Hz）
  z range             : -0.01 .. 3.19 m
  box hits (<=0.25 m) : 12 / 12
  min dist per box    : 0.01 0.01 0.01 ...（12 个都是 1 cm）
  speed |v| EKF       : 中位 3.08  p90 3.73  峰值 4.18 m/s
  speed 位置 0.5s 窗  : p90 3.37 m/s
  参考速度 p90        : 4.89 m/s   参考峰值 5.90
  跟踪误差 水平 rms/p95: 2.12 / 4.21 m（角点等待参考时的大值主导）
  min trunk clearance : +0.214 m
  VERDICT: FAIL        ← 只因速度 p90 3.73 < 4.0；命中与净空都过
```

离线复算（独立于 relay 的打印，见 `docs/evidence/README.md`）：

```
  盒命中 (<=0.25 m)   : 12 / 12
  过点顺序            : 0 1 2 3 0 1 2 3 0 1 2 3
  单圈用时            : 29.2 28.8 s
  树干最小净空        : 0.214 m
```

**离线复算**（不依赖 ROS/SITL，只吃 csv；判据和 relay 一致）：

```bash
python3 scripts/score_trace.py --profile      # 命中 / 速度 / 净空 / 圈速 / 过点顺序
```

**灌参数前必做**（本工程被这件事坑过两次）：

```bash
python3 scripts/check_param_names.py params/*.parm    # 名字对不上固件 → 静默失效
```

---

### 3. 为什么是这些参数（每一条都有现场证据，详见 `docs/INTEGRATION.md`）

| 位置 | 值 | 不这么设会怎样 |
|---|---|---|
| `config/super_forest_15m.yaml` `rog_map.fix_map_origin` | `[0,0,2.0]` | 它是**地图中心**（`rog_map.cpp:45`），不是最小角。写成 `[-20,-20,-2]` 会让地图顶面正好落在飞行高度 2.0，CIRI 的包围盒被裁、种子线跑到盒外 → `SearchPolytopeOnPath failed` → 一条轨迹都不发 |
| 同上 `virtual_ground_height` | `-0.1` | 地图中心放对之后飞行高度在地图中间，地面可以恢复真实值；`-5.0` 是旧 bug 的临时绕法 |
| 同上 `traj_opt.boundary.max_acc` | `6.5` | 上限由倾角决定：`g·tan(ATC_ANGLE_MAX)`。原来 15.0 → 参考从静止"弹射"，飞机跟不动（参考 p90 5.9 vs 真实 3.9）。也不能乱加大——配 `max_vel 8` 会把优化器逼到 `Omg or thr or Pos violation`，规划直接卡死 |
| 同上 `traj_opt.boundary.max_vel` | `7.0` | 一条边只有 15 m，两端还要加减速，参考峰值约为 max_vel 的 0.85 |
| 同上 `astar.map_voxel_num` | `[300,300,60]` | ×`resolution 0.05` = 15×15 m、以当前点为原点 → 向前只能看 7.5 m，一条轨迹被切成 ~8 m 一段。放大到 480/640 会让 GCOPTER 在更长走廊上不可行（见 STATUS 23.1），所以只能停在这里 —— 这也是速度上不去的根 |
| `config/waypoint_forest_15m.yaml` `publish_dt` | `99999`（只发一次） | 10 Hz 重发 goal 会让 FSM 反复走 `GENERATE_TRAJ → PlanFromRest`，而它生成的是 **rest2rest** 轨迹（两端速度都为 0） |
| `params/apm_guided_fast.parm` `ATC_ANGLE_MAX` | `45` | 水平加速度能力 = `g·tan(45°) = 9.8 m/s²`。再往上（55°）只把峰值抬到 4.61，却把净空从 +0.238 压到 +0.120 |
| 同上 `PSC_NE_POS_P / PSC_NE_VEL_P` | `3.0 / 4.0` | 位置-only setpoint 的稳态滞后 ∝ v/`PSC_NE_POS_P`；但 1.4→5.0 试下来是"峰值↑、p90 与净空↓" |
| 同上 `WP_ACC / WP_JERK` | `2.5 / 1.0`（固件默认） | GUIDED 的运动学上限来自 `WP_*`（`mode_guided.cpp:255`），但放开到 6.5/20 实测更差（6/12，见 STATUS 23.2） |
| `params/apm_sitl.parm` `GUID_TIMEOUT` | `0` | 默认 3 s：收不到定点指令就退出 GUIDED，而 bridge 只在 GUIDED 时才发 → 死锁 |
| 同上 `ARMING_SKIPCHK` | `2097150` | 位掩码 bit1..bit20 全置位才是"跳过全部预解锁检查"（`get_enabled_checks()==0`）；写 1023 只跳过前 9 项 |
| 同上 `EK3_SRC1_POSXY` | `3`（GPS） | SITL 里没有 LIO。外部导航只能从 FCU 自己的位置反推，是自环；真机那份在 `params/apm_extnav.parm`（=6） |
| `bridge --calib-file` | `config/calib_identity.json` | 缺省是 +90° yaw（见上） |
| `bridge --fields` | `pos` | `posvel` 会让飞机跑到 ±7.5 m 方形外面（实测飞到 (12.5,-8.1)） |

---

### 4. 换成真机 / 接真 LIO

1. **位置源**：用 `params/apm_extnav.parm`（`EK3_SRC1_POSXY=6, VELXY=6, YAW=6, POSZ=1`，
   `ARMING_SKIPCHK 0`），并让 LIO 通过 MAVLink 视觉消息进 EKF。当前 bridge 已经在发
   外部导航（331，来自 `/lio/robo/odom`），所以 LIO 接通后**不需要改 bridge**。
2. **把 relay 换掉**：真机上不需要 `sitl_odom_relay.py` 的模拟位姿与解锁逻辑；
   `/lidar_slam/odom` 由 Super-LIO 直接提供，`/plan/trigger` 由 supervisor 在
   "已起飞并悬停"后发（这就是 lio_avoid 原本的分工）。
3. **参数**：`ATC_ANGLE_MAX 45`、`PSC_NE_POS_P 3.0` 这类是按 SITL 调的，真机必须按
   质量/桨/推重比在台架 + 低速试飞里复调；`FS_*`、`FENCE_*` 在真机必须真实生效
   （`apm_sitl.parm` 里把它们关掉了，那份只用于 SITL）。灌之前先跑
   `scripts/check_param_names.py`。
4. **看板**：`bash scripts/run_rviz.sh`（`config/rviz_super.rviz`：世界点云、ROG-Map 占据、
   SUPER 的 committed/exp 轨迹、SFC 走廊、A\* 前端路径、FCU 位置）。

---

### 5. 目录

```
super_super_apm/
├── config/
│   ├── super_forest_15m.yaml      ★SUPER 规划器配置（拷贝到 super_planner/config/）
│   ├── waypoint_forest_15m.yaml   ★waypoint_mission 配置
│   ├── calib_identity.json        bridge 的 L→W = 单位矩阵（理由写在文件里）
│   └── rviz_super.rviz            RViz 看板
├── worlds/
│   ├── forest_square15.yaml       52 根树干；下半段方盒 / 上半段圆柱（z_split=2.10）
│   ├── waypoints_square15.txt     12 行 `x y z switch_dis`
│   └── gen_world 相关脚本见 src/tools/build_world.py
├── params/
│   ├── apm_extnav.parm            ★真机：EKF 用外部导航
│   ├── apm_sitl.parm              ★SITL 覆盖（GPS 位置源 / 跳过预解锁检查 / GUID_TIMEOUT=0）
│   └── apm_guided_fast.parm       GUIDED 跟踪参数（4.7 的新名字，见 STATUS 21.9）
├── src/whtf/
│   ├── common/obstacles.py        方盒/圆柱的统一符号距离 + world 加载（评分与建造共用）
│   ├── ros/sitl_odom_relay.py     SITL 里扮演 LIO + 自动解锁/起飞 + GO + 轨迹落盘 + 打分
│   ├── ros/pc_adapter.py          SUPER 的 mars msg → quadrotor_msgs；并造世界点云
│   └── tools/build_world.py       确定性世界生成器（每条边中点塞一棵挡路树干）
├── scripts/
│   ├── run_and_score.sh           ★一条命令跑完 + 打分
│   ├── run_sitl_loop.sh           编排（SITL / relay / adapter / SUPER / bridge）
│   ├── score_trace.py             ★离线复算验收（只读 csv）
│   ├── check_param_names.py       ★灌参数前对固件整表校验名字
│   ├── analyze_ref.py             参考 vs 实际的时间序列（看"滞后"还是"参考在跳"）
│   ├── run_rviz.sh / gen_world.sh / status.sh / dump_all_params.py ...
└── docs/
    ├── INTEGRATION.md             ★完整记录（§0–§51：每次实验的实测数字与结论）
    ├── PIPELINE.md                ★数据管线 + 三个状态机的逐条转移
    ├── MISSION.md                 任务与场地几何（15 m 正方形 × 3 圈、84 柱）
    └── evidence/                  只留最新一批图与数据（9 个文件 / 1.4 MB，索引见其 README.md）
```

---

### 6. 边界（明确没做的事）

- **SITL 里没有真 Super-LIO**：位姿是 `sitl_odom_relay.py` 用 `LOCAL_POSITION_NED` 转出来的。
  这验证的是 `航点 → SUPER → /plan/cmd → MAVLink → ArduPilot 位置环 → 真实位置`
  这一段闭环；LIO 的定位精度/漂移不在本轮证据范围内。切换路径见 §4。
  不用 Gazebo：看板走 RViz（`config/rviz_super.rviz` + `scripts/run_rviz.sh`）。
- **RViz 配置未在本环境实跑**（WSL 无显示），话题名取自 SUPER 源码里的
  `visualization/*`；第一次开时按需勾选。
- **真机参数未复调**（见 §4.3），`params/apm_guided_fast.parm` 里的数字是 SITL 的起点。
- **巡航速度没到 4 m/s**（早期架构下飞机 p90 3.73 / 参考 4.89）。根因与 12 组对照实验见
  `docs/INTEGRATION.md`（原 STATUS 二十三，已归档）：SUPER 的 FSM 里**新 goal 一定走 `PlanFromRest`（rest2rest，两端速度 0）**，
  加上 A\* 局部网格只有 ±7.5 m，一条参考轨迹被切成 ~8 m 一段，飞机永远处在加速段。
  解法是写一个 look-ahead 的 mission 驱动、让 `ReplanOnce` 一路续规划
  （`waypoint_mission` 的 `cmd_type` 在 ROS2 版里是死配置，没有 PATH 模式可切）。
  这件事没做，所以不写在"已完成"里。
