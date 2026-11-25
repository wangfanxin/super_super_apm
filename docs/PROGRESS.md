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

## 2026-11-25 05:35（第 9 条，Phase 1b 实现：tap 移出飞行路径，首飞 H1/H5 PASS）

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

## 2026-11-25 06:00（第 10 条，Phase 1b 打磨：时间基对齐/流就绪门/重连；剩 LIO 停顿）

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

## 2026-11-25 06:40（第 11 条，Phase 1b 收敛：单消息真值 + sim-t0 锚定 + 桥独占核 -> 干净整趟）

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

## 2026-11-25 07:00（第 12 条，Phase 1b 完成：unpatched SITL + tap 归档，两真值逐行一致）

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

## 2026-11-25 07:10（第 13 条，Phase 1 目标态验收飞行通过）

**`missionA-phase1y-01-01`（UNPATCHED SITL + 自建客户端 SIM_STATE 真值，无 tap）**
- 完整两圈、正常 LAND+disarm（13.85 s 布防 → 146.62 s 落地）：**H1/H3/H4/H5/H6 + W0b/W1(13/13)/W1b/W2/W3/W5/W6 全过**；
  H5 水平跨度 10.22 vs 10.22 m；H6 cast p50 18.1 ms、skipped 8；W6 spread 0.147 s。
- 仅 H2（rate 200 Hz vs tap 时代 380 门槛，P0 延后）与 W4（-0.05 m 已知缺陷）不绿。
- 全程无 tap：SITL `LIO_TRUTH_TAP=-1`、无 tap_legacy.csv、`sim/legacy/truth_rx.py` 未运行；时间基偏移 +0.1918 s。
- 结论：Phase 1 完成（判据类 H2 重定义留给 P0 批次）。下一步 Phase 2：bag 抓取率 + 同趟 bag 派生 CSV 的 accept 逐行一致。

---

## 2026-11-25 07:25（第 14 条，Phase 2：split-bag 写入与多 bag 派生；飞行仍受主机 2 s 卡顿影响）

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

## 2026-11-25 07:45（第 15 条，Phase 2 完成：bag 全量录制 + 派生层与现采逐字节一致）

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

## 2026-11-25 08:00（第 16 条，Phase 2 目标态飞行通过：没有 CSV 记录器，accept 只差 H2）

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

## 2026-11-25 08:05（第 17 条，Phase 3 设计稿）

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

## 2026-11-25 08:15（第 18 条，按 owner 指示减少录包：点云不再进 bag）

- **点云不录**：`/livox/lidar`（~197 kB/条 ×10 Hz）与 `/lio/cloud_world`（~307 kB/条 ×10 Hz）约 **0.7–1.0 GB/趟**，
  是其余 topic 的 ~16 倍；派生层与 accept 都不需要它们。默认关闭，`BAG_CLOUDS=1` 才显式打开（横幅警告体积）。
- 默认 `core` 集合缩到 4 个 topic（`/lio/robo/odom`、`/sim/truth/odom`、`/plan/cmd`、`/fcu/odom`），
  `/clock` 也去掉（D4 的时间基是真值客户端 stamps）；`full` = 全部非点云 topic（含 IMU/时间因子/规划参考/rosout）。
- 实测体积：`phase2m`（144 s）core 两个 bag 合计 47 MB；带点云的同长度飞行约 0.7–1.0 GB。
- 文档：`docs/PHASE2-RECORDING.md` 增加"数据量策略"节；PIPELINE-PLAN Phase 2 状态补 owner 指示。

---

## 2026-11-25 08:25（第 19 条，Phase 3 第一步：单连接合并客户端实现 + 门禁 PASS + 首飞暴露控制异常）

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

## 2026-11-25 08:50（第 20 条，Phase 3 第二步：修 legacy 回归 + 331 黑匣子，merged 仍待修）

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

## 2026-11-25 09:45（第 21 条，Phase 3 第三步：extnav 时间戳改用 FCU boot 时钟；单连接已被排除）

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

## 2026-11-25 10:40（第 22 条，★ owner 指令：飞行路径只用真实时钟 ★ + merged 喂数存活修复）

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

## 2026-11-25 11:20（第 23 条，真实时钟戳 legacy 验证通过；merged 已能起飞但仍不稳定）

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

## 2026-11-25 11:30（第 24 条，门禁卫生：bag 测试的 rosbag2 残留进程）

- `tests/run_all.sh` 一度**间歇性 FAIL**：`check_bag_pipeline.py` 每次运行都留下一个 `ros2 bag record` 子进程
  （它被 `start_new_session` 的 shell 包着，SIGINT 打给 shell 不一定会传到 rosbag2），四次之后门禁被干扰。
- 修复：测试现在 (a) 启动时按 **argv[0..3] 签名**（`python3 …/ros2 bag record` + 本测试的 bag 路径）清理残留，
  (b) 结束时用 `os.killpg(spawn_pid, SIGKILL)` 杀整个进程组再补一次扫描。
  **注意**：不能用"命令行子串匹配"（gate 自己的 shell 命令行里就包含测试源码文本，子串匹配会杀掉 gate 本身，
  实测 exit 9）。
- 门禁连续两次 PASS；跑完后 `argv` 级检查无残留 recorder。

---

## 2026-11-25 12:00（第 25 条，Phase 3 bisect 工具就绪；supervisor-only 首飞被主机卡顿污染）

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

## 2026-11-25 12:25（第 26 条，★ Phase 3 定位+修复：merged 的喂数必须由独立线程驱动 ★）

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

## 2026-11-25 13:15（第 27 条，★ Phase 3 bisect 收敛：bridge 半边独立线程+独立进程完全正常，问题在 supervisor 同进程 ★）

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

## 2026-11-25 13:35（第 28 条，★ Phase 3 功能目标达成：merged 成为默认飞行路径并连飞两趟 ★）

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

## 2026-11-25 14:55（第 29 条，Phase 4 代码完成：图案窗口/每帧 CSV/rviz 回放入口；默认仍 legacy 待 A/B）

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

## 2026-11-25 14:25（第 30 条，Phase 4 A/B 首轮：new 调度可用但 W6 变宽；默认保持 legacy；按 owner 要求暂停总汇报）

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

## 2026-11-25 15:25（第 31 条，owner 指示：默认 10000 点 + 整窗均匀调度；新增 ODOMETRY 外部导航手册）

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

## 2026-11-25 16:25（第 32 条，Phase 4 收尾：默认 10000 点 + legacy 调度；发现 W6 的时间基准问题）

- 按 owner 指示先切了 10000 点 + 整窗均匀调度，随后做 A/B（固定 10000 点，只切调度）：new 调度 4 趟干净趟里
  1 趟过 W6（0.129），3 趟失败（0.250/0.469，另一趟主机停顿污染）；legacy 6 趟 5 趟过（0.077 等）。
- **发现更根本的问题**：`phase4oE`（legacy）W6 失败不是跟踪问题——`cmd vs truth` 是**常数 +1.35 s**（四季度一致），
  `lio vs truth` 是**常数 +1.45 s**，即 `setpoints.csv t_sim_us`（LIO 时间戳）与 `tap.csv`（SIM_STATE 时间）
  这一趟差了 ~1.4 s；W6 的 ±0.6 s 搜索窗全部顶边。W6 代码注释本就说明它是两套时钟的记录、好趟只剩 2 ms 余量，
  de-trending 属 P0 判据改动。→ 列入下一步：D4 时间基准统一/标定（在 P0 之前）。
- 结论：Phase 4 完成（per-scan CSV/H6、rviz 回放、图案窗口修正已实现且可 opt-in），**默认 = 10000 点 + legacy 调度**；
  `LIDAR_SCHEDULER=new` 保留为实验开关。证据 `phase4oA/4oB/4n/4oC/4oE`，docs/PHASE4-SIM.md §7。

---

## 2026-11-25 16:45（第 33 条，D4 时间基准修复 + Phase 4 完成：默认 10000 点 + legacy，连飞两趟干净趟）

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

## 2026-11-25 16:50（第 34 条，Phase 3 遗留收尾：extnav 年龄/质量/reset 显式门 + 单元测试）

- 新增 `bridge_nodes/mavlink/nav_gates.py`（无 ROS/MAVLink 依赖的决策表）：`refusal_reason(physical_ok, age, quality,
  min_quality, max_age)` → `0=sent / 1=physical / 2=stale / 3=quality`，顺序物理>陈旧>质量；`describe()` 给状态行用。
- bridge 新增参数：`--nav-quality`(默认100)、`--nav-min-quality`(0=关)、`--nav-max-age-s`(0=关)、`--nav-reset-gap-s`(0=关，
  开启后两次已发送样本间隔超限 → reset_counter++，EKF 接受跳变)；`send_external_nav*()` 的 quality/reset_counter 变成参数。
- 记录：`nav.csv` 的 `refused` 变为原因码，新增 `quality,reset_counter` 列；状态行加 `nav_age_ref/nav_q_ref/nav_resets`。
- **默认与交付行为逐位等价**（quality 100、三门全关），不会改变飞行；飞行开法 `BRIDGE_ARGS="--nav-max-age-s 0.3 ..."`。
- 测试：新增 `tests/check_nav_gates.py` 入门禁；`tests/check_fcu_client.sh` 主 bench 保持 47.1 Hz/relay 500/age-gate PASS，
  并断言生产默认下 quality=100/reset=0/refused=0 且两列存在。`tests/run_all.sh` GATE: PASS。

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
