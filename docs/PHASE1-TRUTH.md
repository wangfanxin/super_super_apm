# Phase 1 记录：不修改固件能否拿到真值（2026-11-24 实测）

> 结论先行：**能，但只能拿到"状态"，拿不到"仿真时钟"。** SIM_STATE（MAVLink 108）由未打补丁的
> SITL 直接发出；它没有 `time_usec`、也没有局部 NED 位置（只有 lat/lon/alt 与 NED 速度/加速度/角速度）。
> 因此 Phase 1 的迁移必须同时做两件事：**真值换 SIM_STATE**、**时钟语义换成 wall + 显式映射**（与 D4 一致）。

## 1. 实测（`_scratch/probe_sim_state.py`，repo 内 SITL，`--speedup 1`）

| 请求速率 | 实际收到 | 说明 |
|---|---|---|
| 200 Hz | **224.8 Hz** | 流调度器把请求速率"取整"到模型循环的整数分之一（模型循环约 1 kHz） |
| 100 Hz | 112.3 Hz | 同上 |

收到的字段（完整）：`q1..q4, roll, pitch, yaw, xacc, yacc, zacc, xgyro, ygyro, zgyro, lat, lon, alt,
vn, ve, vd, lat_int, lon_int, std_dev_horz, std_dev_vert`。

**没有**：`time_usec`（仿真时间）、`x/y/z`（局部 NED 位置）。ArduPilot 的 `SIM::sim_state_send()`
（`.build/ardupilot/libraries/SITL/SITL.cpp:1606`）只填这些字段；固件侧的 `AP_HAL_SITL` 时钟是
`CLOCK_MONOTONIC`（wall），不是模型时间。旧 tap 之所以有 `t_us`/`pn/pe/pd`，是因为当年的补丁把
`state.time_now_us` 与 `get_position_relhome()` 塞进了自定义 UDP 流。

## 2. 对管线的影响（设计结论）

1. **真值来源**：SIM_STATE 足够做"状态真值"（速度/姿态/加速度/角速度与旧 tap 同字段、同 NED 约定），
   且**不需要改 ArduPilot**。
2. **时间基准**：SIM_STATE 只有接收时刻 ⇒ 飞行链的权威时钟改为 **wall**（D4 决策），bag/CSV 同时记
   `t_wall_us`（monotonic）与 `t_unix_us`；仿真侧的调度也改为 wall（`lidar_imu` 的扫描节拍不再依赖 tap）。
3. **位置**：用 lat/lon/alt 反算局部 NED，映射由数据拟合（见下），不写死测地公式常数。
   注意量化：`lat_int` 1e-7° ≈ **1.1 cm**（对 W4 的 −0.04 m 判据敏感，属 P0 判据线，暂不动）。
4. **rtf（真实时间因子）**：旧 H1 用 tap 的 sim/wall 比。改 wall 后由 `compare_truth.py` 的重叠期拟合
   给出（`t_sim = a·t_wall + b`，`a` 即 rtf）；双录期内可直接对比旧 H1。单靠 SIM_STATE 时，可用
   "请求最大速率时的到达速率 / 标定速率"近似（待 Phase 2 记录验证）。

## 3. 已实现的两个工具（Phase 1a，已在仓库内）

| 工具 | 作用 |
|---|---|
| `sim/src/sim_state_recv.py` | 自建 pymavlink 客户端：`SET_MESSAGE_INTERVAL` 请求 SIM_STATE，记录 `sim_state.csv`（原始 lat/lon/alt + 速度/姿态/加速度/角速度 + 接收 wall 时钟 + 临时 NED 估计），SIGTERM 时刷盘 |
| `scripts/compare_truth.py` | 从同一趟的 `tap.csv` 与 `sim_state.csv` 实测：(a) 时钟映射 `t_sim=a·t_wall+b`（事件信号 FFT 互相关，`a` 即 rtf）；(b) lat/lon→NED 仿射拟合与残差；(c) 速度/姿态/加速度/角速度一致性；(d) SIM_STATE 的速率与 gap。输出 `truth_compare.json`，并给出 PASS/FAIL 判定 |

`compare_truth.py --selftest`：用仓库里的一趟 `tap.csv` 反向合成"已知 `a=1/1.0005`、`b=3.25`"的
SIM_STATE 流，工具拟合出 `a=0.9996`、`b=3.239`，测地残差 5.0 mm、速度残差 6 mm/s ⇒ **自检 PASS**。

## 4. 迁移判据（双录几趟，全部通过才能把 tap 移出飞行路径）

```
bash scripts/compare_truth.py --tap  .runs/missionA-<tag>/tap.csv \
                              --sim-state .runs/missionA-<tag>/sim_state.csv
```

| 判据 | 阈值 | 为什么 |
|---|---|---|
| SIM_STATE 速率 | ≥ 90 Hz | 低于此无法支撑 W2/W6 类统计（1.2 kHz 的 tap 是历史契约，见 P0） |
| gap>20 ms 的个数 | 0 | 丢样会让速度/姿态对比出现假差异 |
| 时钟拟合 | `0.97 ≤ a ≤ 1.03` 且相关 > 0.9 | `a` 就是 rtf；相关低说明两段不是同一趟或信号太静 |
| 速度 RMS | ≤ 0.02 m/s | 同一物理量、同一 NED 约定 |
| 位置 RMS（拟合后） | ≤ 0.02 m | lat/lon 量化 + 测地映射残差 |
| 姿态 RMS | ≤ 0.5° | 四元数约定一致性 |

全部通过后，飞臂的 `--truth` 从 tap 切到 `sim_state.csv`（`SIM_STATE=1` 已在 `scripts/fly_hex.sh` 里默认打开，
双录现在就会发生）；tap 补丁保留在旧分支，作为对比与回滚。

## 5. 尚未解决 / 留给后续 Phase

* **H1 的"飞行窗 rtf"**：旧实现读 tap 的 sim 时间；改 wall 后需要新定义（拟合 `a` 或 wall 覆盖度），
  属判据线（P0 延后）。
* **H2 的 1.2 kHz tap 完整性**：SIM_STATE 达不到 1.2 kHz；H2 的判据需要重新定义（P0）。
* **W2 的 max 尖峰**：旧 tap 1.2 kHz 才能捕捉的瞬态，225 Hz 会平滑；判据线（P0）。
* **仿真时钟**：`lidar_imu` 目前仍用 tap 的 sim 时间调度；Phase 1b 改为 wall 调度（同一次迁移里做，
  避免出现"传感器 sim 时间、真值 wall 时间"的混用）。

## 实测补充（2026-11-25，自建客户端已打通）

`sim/src/sim_state_recv.py` 现在能在真实 sortie 里记录 SIM_STATE，并且 `scripts/compare_truth.py` 能从两路数据
自己解出时钟映射与地理映射。以下是**本机实测**（sortie `missionA-phase1f-01-01`、`missionA-phase1a-01-01`）：

**速率上限（ArduPilot 侧）**
- `SET_MESSAGE_INTERVAL` 请求超过调度器能力时 ArduPilot 回 `MAV_RESULT_DENIED`：
  "Requested rate for message ID 108 too fast. Increase SCHED_LOOP_RATE"。飞行参数集下 **100 Hz 有时被拒**、
  120/200 Hz 基本被拒、1000 Hz 请求经自适应退避后落在 **~328 Hz（interval 3 ms）** 并被接受。
- 所以记录器实现了**自适应退避**（DENIED 时 ×0.8 重试，下限 50 Hz）；请求 1000 Hz 实测稳定记录 ~328 Hz，
  `sim_state.csv` 5 万行/趟。**换真值的 90 Hz 门槛满足**，代价只是"请求值"不是"实际值"。

**时钟映射（位置残差最小化拟合，替代旧的 FFT/事件相关）**
- `t_sim = 0.99960 * t_wall + 0.996`（本趟），交叉对照 FFT 给 `a=1.003`——两种方法一致到 3e-3。
- 旧的 FFT 拟合在另一趟给出 `a=1.0193`（偏 2%），因为事件信号在巡航段太弱；**位置拟合是主拟合**。
- b≈1 s 是记录器的启动偏移（SIM_STATE 记录晚于 tap 的 t=0），这就是 D4 要求的"显式映射"。

**一致度（窗口 0–30 s，起飞前后、未进入发散段）**
```
地理拟合  pos rms 11.3 mm (p99 52.9 mm)
速度      rms 0.7 mm/s
姿态      rms 0.012 deg
速率      899.7 Hz（p99 gap 6.9 ms）
```
**飞行中段（20–50 s）仍有 0.16–0.29 m 位置残差**：TCP 成批到达使"到达时间≠采样时刻"；按段线性重建采样时刻
后残差仍大，说明该段存在非线性的到达延迟（或记录器一次 ~1 s 停顿：t=18.45 s 处有 1035 ms 的采样空洞）。
**结论：静态/低速段 SIM_STATE 完全可替代 tap（mm 级）；高速段需要先解决时间戳抖动，才能满足 20 mm 的迁移判据。**

**已知待办**
- 1 s 采样空洞（每趟约一次，发生在起飞前后）：查 SITL 侧还是记录器侧；
- 高速段残差：候选方案是给 SIM_STATE 配独立时间戳（TIMESYNC/系统时间对齐），或在记录器里按批次重建。

## 干净整趟（2026-11-25 / `missionA-phase1h-01-01`，预热 + LIGHT 模式 + SIM_STATE=1）

先机 20 vCPU 预热 45 s（`scripts/warmup.sh`）后飞出的完整两圈：H1 rtf 0.9998、H6 cast p50 15 ms、W1 13/13 全过，
只剩 W4（已知缺陷）。同一趟的双录对比：

```
window 0..20 s (停在地面)   rate 197.2 Hz, gaps 0, clock PASS, pos rms 0.000 m, vel 0.000 m/s, att 0.000 deg -> PASS
window 30..50 s (巡航)      rate 211.4 Hz, gaps 0, clock PASS, vel rms 0.0086 m/s, att rms 0.42 deg,
                            pos rms 0.346 m (p99 0.677) -> geodetic FAIL
```

**定位**：SIM_STATE 的经纬高在飞行段与 tap 的 NED 位置有 ~0.2–0.35 m 的横向差，而速度/姿态通道都对得很好
（8.6 mm/s、0.42°）。查 ArduPilot 源码（`libraries/SITL/SIM_Aircraft.cpp`）：

* `fill_fdm()` 在 `use_smoothing` 为真时，把 fdm 的 `latitude/longitude/altitude/velocity/accel/gyro` 换成
  **`smoothing.*`（一阶平滑滤波器，time_constant 0.1 s）** 的值；平滑滤波器自己的 `smoothing.location =
  origin + smoothing.position`，而 SITL 的 `origin` 会 **1 Hz 移动**（`SIM_Aircraft.cpp:205`，tap 那边专门用
  `get_position_relhome()` 规避了它）。
* 也就是说：**SIM_STATE 的经纬高不是模型的原始真值，而是"平滑态 + 移动原点"的派生量**；速度/姿态在平滑里也
  被同样处理，但它们的误差要小一个量级。用"位置滞后"模型做过拟合（τ 网格 ±0.5 s），最优 τ=0，排除纯滞后。

**结论（Phase 1 的诚实状态）**
- 速率（197–211 Hz 实测，> 90 Hz 门槛）、时钟映射（`t_sim = 0.9999·t_wall + 1.018`）、速度、姿态：达标；
- **位置通道还不能替代 tap**：0.2–0.35 m 的系统性差异需要在 Phase 1b 之前解决。三条候选：
  (a) 用 SIM_STATE 的 `vn/ve/vd` 积分重建位置（速度通道 8.6 mm/s，150 s 内漂移可评估）；
  (b) 继续用 tap 的位置（真值源混合）直到找到 unpatched SITL 里更可靠的几何量；
  (c) 深挖 `fill_fdm` 的平滑/原点路径（需要确认 `use_smoothing` 在腾空后是否复位——本轮代码里只见置真，未见清假）。

## 定案（2026-11-25 05:00）：速度通道定时钟 + 速度积分重建位置 => 判据全过

上面的"位置通道不能替代 tap"结论在**正确建模**下可以绕过：SIM_STATE 的 **速度/姿态/角速度通道是可信的**
（对 tap 残差 ~4 mm/s、0.15–0.43°），不可信的只是**经纬高**（SITL `fill_fdm` 的平滑态 + 1 Hz 移动原点，
慢变 0.2–0.35 m）。因此 `scripts/compare_truth.py` 改为：

1. **时钟映射只用速度通道拟合**：`t_sim = a·t_wall + b`，最小化 vn/ve/vd 残差。
   实测干净整趟 `t_sim = 0.99984·t_wall + 1.006`（残差 0.053 m/s 全趟；窗内 0.004 m/s）。
   b≈1.0 s 正好是"记录器启动晚于 SITL 启动"的偏移；若把 SITL 启动的 wall 时刻传给记录器，映射就是
   `t_sim = t_wall − t_sitl_start`（a=1），与实测一致。
2. **位置用 SIM_STATE 速度积分重建**（梯形法，采样时刻用按段重建的均匀时刻），再做 NED→tap-NED 仿射拟合。
   `missionA-phase1h-01-01` 实测：

```
window 30..50 s : pos rms 2.5 mm (p99 5.3 mm), vel 4.1 mm/s, att 0.149°, rate 211.4 Hz, gaps 0 -> PASS 全部
window 60..100 s: pos rms 1.8 mm (p99 5.0 mm), vel 3.4 mm/s, att 0.428°, rate 914.5 Hz, gaps 0 -> PASS 全部
full run        : pos rms 9.4 mm (p99 51 mm), vel 2.8 mm/s, att 0.592°, gaps 1(1035 ms) -> attitude/gaps 不达标
```

诊断输出仍同时打印"直接经纬高"的位置残差（0.36 m），用来跟踪 SITL 平滑态路径的变化。

**唯一的 1 s 采样空洞**在每趟的 **arm 时刻**（记录器启动后 ~18.5–18.8 s，恰好是 supervisor 布防时），
取飞后窗口评估时它不在窗口内；换真值时它发生在起飞前，不影响飞行判据，但需要记录在文档里。

**结论**：SIM_STATE 作为真值源在**飞行窗口**满足 Phase 1 判据（位置 <1 cm、速度 <2 cm/s、姿态 <0.5°、
速率 >90 Hz、显式时钟映射），可以进入"从飞行路径移除 tap"的实现（`/sim/state` 话题 + lidar 消费它）。

## Phase 1b：把 tap 从飞行路径移除（2026-11-25 实现 + 首飞）

`TRUTH_SRC=simstate` 已接进 `scripts/run-mission.sh`：

* `sim/src/sim_state_recv.py --ros` 现在是**真值节点**：订阅 SIM_STATE（msg 108）后发布
  `/sim/state`（nav_msgs/Odometry，NED/FRD，位置 = pad 锚点 + 速度积分）、`/sim/state/imu`
  （sensor_msgs/Imu，与 odom 同 stamp，按 stamp 配对）、`/sim/truth/odom`（world_w，供 supervisor/
  记录器，w 系转换走 `bridge_nodes/common/frames.py` 一处）、`/clock`、`/sim/rtf`；并把它发布网格上的
  同一串样本写成 **tap 格式 CSV**（`tap.csv`，表头/列与 truth_rx 一致），所以 `scripts/accept.py`
  不用改。
* `sim/src/lidar_imu.py --truth-topic /sim/state`：改用 ROS 真值而不是 UDP tap。两个关键修正：
  订阅必须在**专用 spin 线程**里跑（主循环一次 spin_once 只处理一个回调，200 Hz 真值 = 400 回调/s，
  实测 IMU 掉到 15–20 Hz、75/80 扫描被跳过）；按 header stamp 配对 odom/imu 时**先查再 pop**
  （先 pop 会把 odom 吃掉，配对永远失败）。
* 调度：客户端用**重建的均匀采样时刻**（滚动块平均周期），而不是 TCP 到达时刻——到达是成批的，
  直接用会让 LiDAR 的 20 ms 窗口检查报洞、CSV 出现 1.04 s 空洞、重建时间轴 1.6× 偏快（phase1k 实测
  202 Hz/323 Hz、627 次跳扫）。
* 机头仍用带 tap 补丁的固件（tap 仍在发、但飞行路径不再读它）；删除补丁/truth_rx/fanout 是下一步。

**无飞行管线校验**：`bash scripts/check_truth_ros.sh`（SITL → 客户端 → /sim/state → lidar → Super-LIO）。
实测 PASS：cloud 9.88 Hz、IMU 195.3 Hz、cast 15.8 ms、skipped 0；CSV 恰好 200.0 Hz、max gap 5.0 ms、
>10 ms 空洞 0。

**首飞（`missionA-phase1l-01-01`，预热 + LIGHT + TRUTH_SRC=simstate + LEGACY_TAP=1）**
飞行完整：13.7 s 布防、152.5 s 正常 LAND + disarm。同一趟用两种真值打分（accept 相同参数）：

| 行 | 旧 tap 真值 | SIM_STATE 真值 |
|---|---|---|
| H1 rtf | 0.9998 | 0.9998 |
| H5 水平跨度 | 10.65 m（FCU 10.60） | 10.66 m（FCU 10.60） |
| W1 / W1b | 13/13；worst 0.19 m | 13/13；worst 0.19 m |
| W2 p90 | 1.51 | 1.49 |
| W3 偏差 p50 | -0.089 m | -0.088 m |
| W6 ZERO lag p50 / spread | 0.596 m / 0.120 s | **0.229 m / 0.117 s** |
| H2 | 1200.5 Hz，0 gap | **333 Hz，max gap 0.64 s，101 gap>10ms** |
| H6 skipped | 3 | 3（同一 lidar 日志） |

* **H1/H5 在更换真值后仍 PASS** —— 这是计划给 Phase 1b 的验收条件。
* H2 的 380 Hz 速率下限/空洞门槛是**为 1.2 kHz tap 定的**（SIM_STATE 上限 ~330 Hz，被
  `SCHED_LOOP_RATE` 卡住）；这属于验收判据类改动（P0，延后），但"真值切 SIM_STATE"后必须重新定义。
* **W6 两种真值差 0.37 m**（计划要求 <0.01 m）：根因是客户端时间轴用"块平均周期"重建，飞行 130 s 上
  仍有相对漂移（W0 起点差 0.35 s 也指示这一点）。修法是改成"到达时刻的滚动线性回归（alpha+beta·i）"
  当时间轴，漂移相对 wall 恒定为 0；这是下一步的第一件。
* 相对旧 tap 真值，SIM_STATE 真值还让 **W6 从 0.596 m 降到 0.229 m**（旧 tap 的 1.2 kHz 自身时序
  有 ~0.6 m 的等效滞后，值得下一轮用同一趟数据核实是谁的偏差）。

## Phase 1b 打磨（2026-11-25 06:00）：时间基对齐、流就绪门、断线重连

* **时间基**：`run-mission.sh` 在 SITL 启动前记 wall 时刻，客户端启动时把 `client_start - sitl_start` 作为
  `--stamp-offset-us`，于是真值 CSV/话题的 t=0 就是仿真 t=0（实测 +1.17 s）。同一趟 `missionA-phase1p` 的
  W6 偏移为 **+0.600 s，四段 spread 仅 0.005 s**（此前用块平均周期时 spread 0.118 s、W6 两真值差 0.37 m）；
  **漂移问题已解决**，剩下的 0.600 s 是飞行中段的真实跟踪滞后（见下）。
* **采样时刻**：改为"到达时刻对样本序号做滚动最小二乘（alpha+beta·i）"，beta=流周期、alpha=wall 锚点，
  长趟无相对漂移；CSV 恰好 200.0 Hz、max gap 5 ms、>10 ms 空洞 0。
* **流就绪门**：客户端等到 autopilot 心跳（system id ≥1）才开始；`FIRST SIM_STATE` 标记出现后 launcher 才启动
  LiDAR（phase1o 的教训：客户端 t≈2 s 连上、调度器拒绝到 ~10 s，LiDAR 起进一个带洞的缓冲区，271 次跳扫、
  LIO 没有里程计、abort）。若 8 s 无样本自动重连（最多 3 次）。
* **剩余问题**：`phase1p` 飞行中段一次 **LIO 1.70 s 无输出** 触发 "ESTIMATOR FEED IS STALE"（限 1.50 s）→ LAND 不完整；
  H6 skipped 21（门槛 20）——都指向 SIM_STATE → ROS 的**偶发丢包/停顿**。候选修法：单条组合消息（回调减半）、
  提高 DDS 资源上限、客户端停顿时的 hold 时长/重发。H2 的 rate 200 vs 380 是 tap 时代门槛，等 P0 一并重定义。

## Phase 1b 收敛（2026-11-25 06:40）：单消息真值 + sim-t0 锚定 + 桥独占核 → 完整干净飞行

三个仍是"真值切 SIM_STATE 之后才暴露"的问题，本轮都定位并修掉：

1. **时间基**：patched SITL 在 `SIM_Aircraft::init` 打印 tap banner 的时刻就是仿真 t≈0；`run-mission.sh` 用
   20 ms 轮询把它 wall-clock 下来（`SITL_SIMT0_US`），客户端 `--stamp-offset-us = client_start − SITL_sim_t0`。
   此前用"进程启动时刻"，真值比 tap 早 0.600 s，W6 读出恒定 +0.600 s（phase1q）。
2. **回调减半**：真值改成**每样本一条消息**——`nav_msgs/Odometry`，specific force 放 `pose.covariance[0:3]`
   （nav_msgs/Odometry 没有加速度字段；此前 `/sim/state` + `/sim/state/imu` 两条 200 Hz = 400 回调/s，
   LiDAR 的扫描调度会落后，实测 skipped 44–154）。单条后 H6 skipped 21→20、且不再 68/154。
3. **桥被饿死**：`ESTIMATOR FEED IS STALE`（1.5 s 限）不是 LIO 丢数据（lio.csv 实测 200.0 Hz、0 个 >50 ms 空洞），
   而是 bridge 的 LIO 回调被 supervisor 的 1 ms 循环同物理核抢死。拆核：planner 12–13、bridge **独占 14**、
   AUX(含 supervisor) 16–17、truth client 18–19、SITL 0–3、传感器 4–7、LIO 8–11。

**`missionA-phase1w-01-01`（TRUTH_SRC=simstate + LEGACY_TAP + LIGHT）**：飞完两圈、正常 LAND+disarm；
H1 0.9998、H3/H4、**H5（tap 10.79 m vs FCU 10.79 m）**、W0b、W1 13/13、W1b、W2/W3/W5 全过；
余下 H2（rate 200 vs tap 时代 380 门槛，P0 延后重定义）、H6（skipped 23，门槛 20）、W4（已知缺陷）、
W6（p50 0.238 / spread 0.220 s > 0.15，时间基仍有 ±0.1 s 慢摆，下一轮把"到达时刻回归"换成"对齐系统时间+斜率拟合"消掉）。

