# Phase 3 设计：把 bridge + supervisor 合并成一个自建 MAVLink 客户端

**状态**：设计稿（2026-11-25），尚未实现。Phase 1（真值）与 Phase 2（录制）已完成，本阶段动的是
**FCU 接口层**——也是全链路最后两个"各写一半"的进程。

## 1. 现状（两个进程、两条连接、两套转换）

| 进程 | 连接 | 职责 | 产物 |
|---|---|---|---|
| `bridge_nodes/mavlink/bridge_node.py` | tcp:5780 | 订阅 `/plan/cmd`(L) + `/lio/robo/odom`；L→W→FCU-NED 转换；SET_POSITION_TARGET_LOCAL_NED(84) 50 Hz；ODOMETRY(331) 48 Hz；relay 开关（起飞门）；clamp ff v/a | `setpoints.csv`、`wire.csv`、`bridge.log` |
| `bridge_nodes/supervisor/supervisor.py` | tcp:5781 | 自己的 `MavlinkBridge`；状态机（arm→takeoff→airborne→leg guards→land/abort）；发 COMMAND_LONG（arm/takeoff/land）；读 FCU 位姿判守卫 | `phase.csv`、`supervisor.log` |

由此产生的**具体缺陷**（本轮之前的只读代码审查已定位，未修）：

1. **两套 frame 转换**：bridge 用 `calib.json` 的 L→W（yaw/t），supervisor 各自从 `/sim/truth/odom`/`fcu/odom`
   再判 W。yaw 的换算点分散，任何一处改动都不会被另一处看见。
2. **wire liveness 缺失**：没有"FCU 心跳多久没来/命令是否被执行"的统一看门狗；只有 supervisor 的 plan-silence
   与若干数值守卫。传输层断流时 bridge 仍会继续把旧位姿喂给 EKF（Phase 0 曾观测到 extnav 冻结 1.5 s）。
3. **extnav 侧只有"发送年龄"**：bridge 在发送侧做 age gate，但没有对 **LIO 质量**（std_dev/跳变/reset）与
   **EKF 是否接受**（EKF origin/lane switch）做闭环检查；也没有 331 的 reset 处理（LIO 重启后位姿跳变）。
4. **abort 语义分散**：supervisor 发 LAND/改模式，bridge 只负责停 setpoint；一个进程死掉时另一方的行为未定义
   （Phase 0 见过"stop setpoints 后 supervisor 仍在等"的状态）。
5. **两条 MAVLink 连接**：56 字节/50 Hz 的 setpoint 和 331 的 extnav 走两个 tcp；带宽不是问题，但**控制回路的
   一致性**（同一时刻的 FCU 状态、同一时刻的 liveness）被拆散。

## 2. 目标：一个进程、一条连接、一处转换

新文件 `bridge_nodes/mavlink/fcu_client.py`（自建 pymavlink 客户端，**不用 mavros**，D1）：

```
/plan/cmd (PositionCommand, L) ─┐
/lio/robo/odom (Odometry, L)  ──┤
                                ├─► [Frame once]  L→W (calib) → FCU NED/FRD (仅此处换算)
/sim/truth/odom (W, 测试真值) ──┘        │
                                         ├─► SET_POSITION_TARGET_LOCAL_NED (84)  50 Hz
                                         ├─► ODOMETRY (331)                      48 Hz
                                         ├─► COMMAND_LONG: arm/takeoff/land/mode 事件驱动
                                         └─► 状态机 + 守卫（原 supervisor）
                                                  │
                                                  ├─► phase.csv / wire.csv / setpoints.csv
                                                  └─► /fcu/status (liveness/age/quality)
```

**必须写死的接口约定**

| 项 | 约定 |
|---|---|
| extnav age gate | 发送前 `age = now - lio_stamp`；> 0.5 s 停发 331 并置 degraded；> 1.5 s 触发 abort（LAND） |
| extnav quality | 用 LIO 的位姿协方差/`/lio/health`（无则用相邻样本跳变 > 0.5 m 或 > 5 m/s 判定）；连续 3 帧异常 → 331 停发 |
| extnav reset | 检测到 LIO 位置跳变 > 2 m（重启/重定位）时：重发 SET_GPS_GLOBAL_ORIGIN？不——先停 331，要求 EKF reset（`EKF3` 的 extnav reset 通过重启 extnav 流或 LAND），并在 `fcu.log` 记录；是否允许自动 reset 由 owner 拍板（默认只 abort） |
| wire liveness | 心跳 > 2 s 或 COMMAND_ACK 超时 > 3 次 → 记录 + 停 setpoint + LAND（若已 armed）；RELAY 关闭时不再接受新 plan |
| yaw 单点转换 | 只在此进程内：L yaw → W yaw（calib yaw）→ FCU yaw 的 +/-pi 归一化；supervisor 的 W 界面不变 |
| take-off 门 | 保持现行语义（SETPOINT z=0.2 起飞、AIRBORNE 后放行 plan setpoints；96 命令不参与） |
| 落盘 | `phase.csv`（现有列）、`wire.csv`（84 实际值）、`setpoints.csv`（cmd/fcu/lio 对照）保持列名不变，accept 不变 |

## 3. 迁移与验证

1. **实现侧**：`fcu_client.py` 内部拆成 `transport`（pymavlink 连接/重连）、`frames`（唯一转换点）、
   `relay`（84/331 发送与 clamp）、`supervisor`（状态机/守卫）、`log`（CSV 三件套）五块，便于单测。
2. **开关**：`FCU_CLIENT=merged|legacy`（默认 `legacy` 直到验证）；`legacy` 仍启动 bridge+supervisor 两个进程。
3. **无飞行验证**：`tests/check_fcu_client.py`（SITL + 合成 plan + LIO odom 回放 → 断言 84/331 的发出序列、
   yaw 转换、age gate、abort 序列；不需要起飞）。
4. **飞行验证**（同一套臂 A/B，各 1–2 趟）：
   - `FCU_CLIENT=legacy`（现状）一趟 → 基线 accept；
   - `FCU_CLIENT=merged` 一趟 → **H1/H3/H4/H5 不降级、W 行与基线同量级（|Δ| 在测量抖动内）**；
   - 对比 `wire.csv`/`setpoints.csv` 行数与 84 的目标值时间序列（同 arm 下应仅差数 ms 的采样）。
5. **删除条件**：merged 连飞 2 趟通过后，删除 bridge/supervisor 旧文件与 `MavlinkBridge` 重复实现，
   把回滚路径记入 `docs/INTEGRATION.md`（git revert + `FCU_CLIENT=legacy`）。

## 4. 风险

| 风险 | 缓解 |
|---|---|
| 合并后一个进程承担 arm/land + setpoint，崩溃即失去两条路径 | systemd 不管；但 `run_until_healthy` 已有"桥/守卫死则 abort"；merged 进程加 `--watchdog` 自愈（退出前发 LAND） |
| extnav reset 策略过激导致误 abort | 阶段 3 先**只记录不动作**（log-only）飞 1 趟，拿到真实发生率再决定 |
| supervisor 的 1 ms 守卫循环与发送在同一进程，可能互相拖延 | 发送线程 + 守卫线程分离（pymavlink 非线程安全 → 单线程 event loop + 定时器，84/331 用 deadline 调度） |
| 判据类行为（P0） | 不改任何 accept 门槛；行语义保持 |


## 实现进度（2026-11-25，第 1 步：单连接骨架完成，实验性启用）

已实现 `bridge_nodes/mavlink/fcu_client.py`：**一个进程、一条 MAVLink 连接、一个 rclpy node**。

* `bridge_node.BridgeProcess` 与 `supervisor.Supervisor` 都支持**依赖注入**（`bridge=`/`node=`）；
  supervisor 的 pump 循环每轮调用 `BridgeProcess.step()` —— 同一个线程串行化对 socket 的每一次写，
  84（50 Hz）/331（48 Hz）/class-76 共享一条链路。
* 两半的 argparse 各自提取成 `add_bridge_arguments()` / `add_supervisor_arguments()`，
  `fcu_client.build_parser()` 用 `conflict_handler="resolve"` 合并（同义 flag 只留一个值）。
* 启动器支持 `FCU_CLIENT=merged`（默认仍是 `legacy` 双进程，直到验证完成）。
* **无飞行门禁 `tests/check_fcu_client.sh`（已接入 `tests/run_all.sh`）**：SITL + 合成输入 →
  `fcu_client --bench`，实测 `#331 46.9 Hz`、`#84 500 sends / 11 s`、`setpoints.csv 500 行`、
  `wire.csv 500 行`、extnav age gate 触发 → **PASS**。

### 首飞（`missionA-phase3a-01-01`，`FCU_CLIENT=merged`）

链路与状态机走通：arm → 起飞 → 悬停 → THE GO → relay 使能 → leg1/leg2，`#84` 1054 条、`#331` 1774 条、
`phase.csv` 正常；H1/H3/H4/H5/H6 全过。但**控制异常**：THE GO 后 z 冲到 3.17 m（legacy 同 arm 同一时刻 1.85 m），
随后掉到 1.6 m、SITL 报 `SIM Hit ground`，EKF 与 truth 差 2.0 m → abort（`phase -> abort/land`）。
`wire.csv` 与 legacy 同 arm 的早期行**完全一致**（同一目标 z=2.000），所以不是目标值错，需要查：

1. `#331` 的时间戳/速度（`bridge.send_nav` 在合并后是否被 `step()` 的调用时机改变了相位）；
2. 起飞段 84 的第一个样本是否早于 legacy（relay 使能时刻、takeoff→GUIDED 切换的竞态）；
3. 合并后 EKF 收到的 extnav 与 legacy 的逐行对比（可用同一 arm 的 legacy 飞行 `wire.csv` + `#331` 序列）。

在查清之前 `FCU_CLIENT` 保持 `legacy` 默认；下一次修好后同一 arm A/B 复飞。


## 第 2 步（2026-11-25 08:30–08:50）：修好 legacy 回归、给 331 加"黑匣子"，merged 仍待修

### 修掉的回归（我的 Phase 3 重构造成的）
`BridgeProcess.run_loop()` 的状态发布在重构时丢了 `String` 的导入，**legacy bridge 在第一个 200 ms 状态 tick
就 NameError 死掉** → FCU "Arm: VisOdom: not healthy" 拒绝解锁（`missionA-phase3c-01-01` 11.4 s abort）。
修复后 legacy 复飞 `missionA-phase3d-01-01`：完整 13 段、正常 LAND+disarm，FCU 估计与 truth 偏差
**p99 0.30 m / max 0.45 m**（全程），H1/H3/H4/H6 + W0b/W1(13/13)/W1b/W2/W3/W5/W6 全过。
（H5 这一趟 FAIL 是**判据窗口伪影**：落地后 tap 又录了 ~95 s，中位数从 (0,0) 漂到 (5.2,−8.1)，
"H5 span" 因此 19.73 vs 10.13；FCU 与 truth 的逐点对比只有 0.3 m。属于 P0 判据类，暂不修改。）

### 831 号黑匣子：`--nav-log`（nav.csv）
每条真正发出去的 ODOMETRY #331 记录 `t_wall,t_us,age,px,py,pz,vx,vy,vz,q*,refused`（两种模式都记录，
run-mission 自动带 `--nav-log $OUT/nav.csv`）。同时把 LIO 位置差分速度改为**用消息时间戳做 dt**（墙钟只在没有
时间戳时兜底）并加 `--nav-vmax`（默认 15 m/s）限幅：合并模式下同一个 rclpy node 的 LIO 回调可能成串到达，
墙钟 dt 会把 5 mm/5 ms 的步长算成几十 m/s，而那个数字直接进 EKF。

### merged 模式的现状（仍是实验性，默认 legacy）
两趟（phase3a/3b）都能 arm/起飞/GO/leg1/leg2，但之后 **EKF 自己的估计向上跑飞**（leg 内从 2.6 m 线性涨到
70 m，速度 0.9 m/s²），真值模型被自己的位置环压到地面（`SIM Hit ground`、`EKF variance`/`EKF Failsafe`）。
`nav.csv` 显示 merged 在起飞前有一次 **2.5 s 的 331 断流**（legacy 同一段无断流），飞行中还有 0.1–0.2 s 的
多次小断流；而 331 的位置/姿态/限幅速度在跑飞期间本身是**合理**的（|v| ≤3.3 m/s、四元数归一）。
下一步查：这些断流是否让 EKF 进入 dead-reckoning（加速度计零偏 → 估计以 ~0.9 m/s² 发散），
以及断流根因（merged 的 supervisor pump 在 preflight/arm 期间的分支阻塞）。


## 第 3 步（2026-11-25 09:00–09:40）：extnav 时间戳改成 FCU 时钟；单连接/双连接都排除

### 用 FCU 自己的 BIN 读出了真实年龄（关键测量）
`flight.BIN` 的 `VISP/VISV` 记录含 `RTimeUS`（消息里的 time_usec）与本地时间，二者之差就是 **EKF 看到的 extnav
数据年龄**：

| 飞行 | 模式 | VISP 年龄 p50 | 结果 |
|---|---|---|---|
| `missionA-phase2m`（legacy） | 两连接 | **0.631 s** | 正常 13 段飞行 |
| `missionA-phase3e`（merged） | 单连接 | **1.611 s** | 起飞后 20 s 内失控 |

也就是说：我们的 ODOMETRY `time_usec` 用的是**仿真时间**，而 EKF 拿它和 **FCU 自己的 boot 时钟**比，两个时间基
的差在两种模式下不同（legacy 0.63 s / merged 1.61 s）——1.6 s 的"旧数据"会让位置环滞后并最终发散。

**修复**：向 FCU 请求 `SYSTEM_TIME`（2 Hz），用 `time_boot_ms` 估计 FCU 的 boot 时钟，把每条 331 的
`time_usec` 改写成 **"FCU boot-now − 该样本的年龄"**，并加严格单调下限（回退到旧时间基时也不会倒退）。
实测（`phase3f` 的 BIN）：**VISP 年龄 p50 0.007 s / max 0.058 s**，`Ign=0`（全部被接收）。

### 但 merged 仍然失败（所以时间戳不是唯一原因）
- `phase3f`（单连接 + FCU 时钟）：VISP 年龄 7 ms，仍然在 GO 后 2 分钟内失控（模型爬到 4.7 m、FCU 估计停在
  1.7–2.3 m，然后 EKF/位置环把模型压到地面）。
- `phase3g`：BIN 的 `MODE` 记录显示 boot 50.77 有一次 **mode 9 = LAND，Rsn=2 = GCS_COMMAND**，比
  "SIM Hit ground" 早 3 s —— 说明失控前 FCU 收到过一条我们没料到的 LAND；`fcu_client.log` 里 supervisor
  的 abort 在那之后。这条 LAND 的来源仍待查（唯一会发 class-76 的是 supervisor，而它的日志没有对应行）。
- `phase3h`（**--dual-link**：同一进程、同一个 node，但 class-76 走 serial1、84/331 走 serial0 —— 即历史两连接
  架构合并进一个进程）：同样失败。**⇒ 单连接共享被排除**；问题在"两个半身共处同一进程/同一 pump"这一侧。

### 代码改动（本轮）
- `mavlink_impl`：请求并解析 `SYSTEM_TIME`，`fcu_boot_now_us()`；`bridge` 用 `--nav-log` 记录每条的
  `t_feed_us`（实际时间戳）与 FCU 侧 `ekf_ok/ekf_flags/armed/mode/nav_hz`（BIN/VISP 的遥测对照）。
- `bridge_node`：`--feed-clock`（默认 fcu）时间戳改写 + 单调下限；LIO 差分速度用**消息时间戳**的 dt（墙钟兜底）
  + `--nav-vmax` 限幅；`nav.csv` 增加 FCU 侧列。
- `fcu_client`：`--dual-link`（诊断/回退：一个进程、一个 node、两个连接）。
- `tests/check_fcu_client.sh`：bench 也验证"时间戳落在 FCU boot 时钟里"（470/470 条）。

### 下一步（bisect 计划）
1. **merged 进程 + legacy supervisor 进程**（bridge 半身在 merged 进程里，class-76 由原来的独立 supervisor 发）：
   若这样能飞，问题在 supervisor-in-merged-process（pump 结构/回调顺序）；若不能，问题在 bridge-in-merged-process。
2. 查 phase3g 里那条 `Rsn=2` 的 LAND 是谁发的（抓 serial0 上所有 class-76 的源：给 `MavlinkBridge` 加
   `command_long` 日志，或对比 supervisor 日志时间）。


### 第 3 步补充：merged 的失控现象与 legacy 的逐项对照（数据事实）
- merged 的 **extnav 位置是"真的"**：`phase3g` 的 BIN 里 `VISP.PZ` 在起飞后 12 s 内从 −0.11 涨到 −4.60 m，
  与模型自己的 `SIM_STATE`（`alt` 584.1→589.0）一致；legacy 同阶段 PZ 停在 −1.85…−1.95（目标 2 m）。
  也就是说 merged 里飞机**真的被飞到了 4.6 m**，而 84 的 setpoint 一直是 `p_l_z=1.88 / z_ned=−2.00`
  （`wire.csv` 678 行，`type_mask`、fields 与 legacy 完全相同），plan 也一直只发竖直腿（`derived/traj z=1.88`）。
- EKF 侧：merged 的 `XKF4.SH`（高度创新）在起飞后 ~16 s 达到 **3.77 m**（legacy 全程 ≤0.28 m），
  `SP` 到 2.04 m —— EKF 从一开始就没有跟上竖直 extnav；`VISP` 的 `Ign` 全为 0（说明不是 quality 门），
  因此是 **创新门/状态发散**，不是数据没到。
- `phase3g` 的 `MODE` 记录：boot 50.77（≈起飞后 20.6 s）出现 **mode 9 LAND，Rsn=2 = GCS_COMMAND**，
  比 `SIM Hit ground` 早 3 s；`fcu_client.log` 里 supervisor 的 abort 在此之后。**这条 LAND 来源未定**。
  （`phase3h` 的 `--dual-link` 也复现同样的失控 ⇒ 与单连接无关。）

下一轮优先级：
1. 抓 class-76 源：给 `MavlinkBridge.command_long_send` 类调用点加统一日志（谁、何时、哪条命令），
   解释那条 `Rsn=2` 的 LAND；
2. bisect：merged 进程只跑 supervisor 半身 + 独立 legacy bridge 进程（以及反过来），判定哪一半在
   "合并进程"里改变了 EKF 看到的输入；
3. 注意 legacy 在本轮之前从未用 `--feed-clock fcu` 飞过：需要一趟 legacy 复飞来确认 extnav 时间戳改写成
   boot 时钟对已验收链路无害（BIN 的 VISP 年龄应为 ~ms 级、13 段飞行正常）。


## 第 4 步（2026-11-25 10:00–10:40）：★ 只许用真实时钟（owner 指令）★ + 修正上一节的误读

### owner 指令
> "你不能按照 FCU 时间，我最后要上真机的！！！必须用真实 time.time/mono 给的时间，即使是仿真也要尽量实时"

据此**撤销**了上一节"用 SYSTEM_TIME 把 #331 时间戳改写成 FCU boot 时钟"的做法：飞行路径里的时间戳一律用
**本机真实时间**（`time.monotonic()`，µs），不再依赖 FCU 时钟，也不依赖仿真时间。

### 同时修正上一节的误读（重要）
上一节把 `flight.BIN` 的 `VISP.RTimeUS` 差的绝对值当成"EKF 看到的 extnav 年龄"，**这是错的**：
ArduPilot 收到 ODOMETRY 后走 `GCS_MAVLINK::handle_odometry` → `correct_offboard_timestamp_usec_to_ms()` →
`JitterCorrection`（`libraries/AP_RTC/JitterCorrection.{h,cpp}`）：它**学习我们的时间基与 FCU 到达时间之间的
最小滞后偏移**，把任意 base 的 offboard 时间戳映射到 FCU 本地时间，只要求"不来自未来、滞后不超过 500 ms"。
因此 `VISP` 里显示的 0.63 s / 1.61 s 只是**两个时钟基的差**，不是数据陈旧；legacy 0.63 s 恰好说明那一版的
喂数本来就慢半秒但仍在 500 ms 门限内。正确做法就是 owner 说的：用真实时钟，让 FCU 自己对齐。

### 本轮代码（真实时钟 + merged 的喂数存活）
- `bridge_node`：`#331` 的 `time_usec = int(time.monotonic()*1e6) − 该样本的年龄`，并加严格单调下限
  （样本停更时每次 +1 ms，绝不倒退）；删除 SYSTEM_TIME 请求/解析与 `fcu_boot_now_us`。
- `tests/check_fcu_client.sh`：断言 `t_feed` 是真实单调时钟且严格递增。
- **merged 模式的喂数存活**（这是"VisOdom: not healthy"的真正原因）：supervisor 里有两处 `poll()+sleep`
  等待**绕过了 pump**，在 merged 里就停掉 #331 和 ROS 回调：
  * `check_own_link` 的心跳等待；
  * `check_position_stream` 的 2.0 s 吞吐测量（实测 `phase3n`：nav.csv 出现 2.0 s 空洞，age 涨到 2.0 s，
    随后 FCU 拒绝解锁 "Arm: VisOdom: not healthy"）。
  两处都改为 `self.pump(period)`，并给 `MavlinkBridge` 加了 `on_wait` 钩子：`_command` / `calibrate_accel_simple`
  / `set_mode` 的 ACK 等待里也驱动 `bp.step()`（Phase 3 的合并客户端里，pump 是 #331 的唯一驱动者）。
- 每一条 class-76 命令现在都带调用者日志（`SENDING command N ... (from arm <- preflight)`），FCU 的
  armed/mode 变化也会打印——上一轮 `phase3g` 里那条"意外的 LAND（Rsn=2 GCS_COMMAND）"就靠这个抓。

### 现状
`phase3o`：2.0 s 的喂数空洞已消失；剩余 0.48 s 的空洞出现在 `calibrate_accel_simple` 之后、`arm` 命令发出前，
仍足以让 FCU 判 "VisOdom: not healthy"（阈值 300 ms，`AP_VISUALODOM_TIMEOUT_MS`）。下一步是把这段间隙也
归零（`arm` 调用前的 `pump(0.5)` 实测有空洞，需要查它为何不驱动 #331），然后再飞一次完整的 merged 趟。
