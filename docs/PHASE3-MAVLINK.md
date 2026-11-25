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
