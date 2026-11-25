# 用 MAVLink `ODOMETRY`（#331）给 ArduPilot EKF3 喂外部导航 —— 实战手册

> 适用：自建视觉/激光惯性里程计（LIO、VIO）→ MAVLink → ArduPilot（Copter 4.5+ / EKF3）。
> 本文来自本仓库在 SITL 上的实现与飞行验证（`bridge_nodes/mavlink/mavlink_impl.py::send_external_nav`，
> 参数见 `fcu_interface/params/apm_lio_only.parm`），真机同样适用。
> **核心结论**：EKF 不会被替代；ODOMETRY 是喂给 EKF 的**观测**（external nav），EKF 仍然用 IMU 传播、做创新门限、
> 失效保护。`ODOMETRY` 比 `VISION_POSITION_ESTIMATE` 字段更全（带速度、reset_counter、quality、帧对）。
> **没有 GPS 也能用**（GPS 只是另一个观测源）：需要 EKF 原点 + extnav 航向 + 持续 331，见 §10。

---

## 1. 为什么选 #331 而不是别的"位姿消息"

| 消息 | ID | 带什么 | 适合 |
|---|---|---|---|
| **ODOMETRY** | **331** | 位置 + 速度 + 姿态 + 6×6 位姿协方差 + `reset_counter` + `quality` + 帧对 | **首选**（ArduPilot 的 `AP_VisualOdom` MAV 后端完整支持） |
| VISION_POSITION_ESTIMATE | 102 | 位置 + 姿态 + 协方差（无速度、无 reset_counter、无 quality） | 只有位姿可发时的退化选择 |
| VISION_SPEED_ESTIMATE | 103 | 速度（NED） | 只补速度，不能单独定位 |
| ATT_POS_MOCAP | 138 | 位置 + 姿态（动捕） | 动捕场景 |
| LOCAL_POSITION_NED（上行） | 32 | 是**飞控自己的输出**，不是输入 | 用来对比/监控，不要发回来 |

ArduPilot 会把 331 同时喂两条路：位置/姿态 → `writeExtNavData`，速度 → `writeExtNavVelData`，都进 EKF3。

## 2. 飞控必须设置的参数（缺一个就"数据被静默丢弃"）

```
AHRS_EKF_TYPE 3        # 用 EKF3
EK3_ENABLE    1
VISO_TYPE     1        # ★ 外部导航的 MAVLink 后端。不设它 AP_VisualOdom 根本不启用（测过：静默）
EK3_SRC1_POSXY 6       # 6 = ExternalNav
EK3_SRC1_VELXY 6
EK3_SRC1_POSZ  6       # 高度也吃外部导航（否则默认吃气压计）
EK3_SRC1_VELZ  6
EK3_SRC1_YAW   6
EK3_SRC_OPTIONS 0
GPS1_TYPE     0        # 纯外部导航机上通常禁用 GPS；有 GPS 时可留作辅助源
```

可选但常调：`VISO_POS_M_NSE`（位置观测噪声地板）、`VISO_YAW_NSE`、`VISO_QUAL_MIN`（默认 0）、
`VISO_DELAY_MS`（已知固定延迟时补偿）、`EK3_POSNE_M_NSE` / `EK3_VELNE_M_NSE` / `EK3_YAW_M_NSE`。

> ★ 踩过的坑：`EK3_SRC1_POSXY 6   # 6 = ExternalNav` 这种**行尾注释会让整行解析失败**，参数静默停在旧值。
> 参数文件里注释必须单独占行。

## 3. 消息字段契约（照抄即可）

| 字段 | 必须值 / 含义 |
|---|---|
| `time_usec` | 你的估计器产生该状态的时间。**用真实单调时钟**（`time.monotonic()` 纳秒/微秒）。ArduPilot 会用 `JitterCorrection` 把它映射到飞控本地时基，见 §5 |
| `frame_id` | **`MAV_FRAME_LOCAL_FRD`（20）** —— 否则 ArduPilot `return` 掉整条消息，**静默无日志** |
| `child_frame_id` | **`MAV_FRAME_BODY_FRD`（12）** |
| `x, y, z` | 位置（米），本地帧：x 北/前、y 东/右、z 下，**相对 EKF 原点**（原点由 `SET_GPS_GLOBAL_ORIGIN` 或 GPS 设定） |
| `q` | 姿态四元数 `[w, x, y, z]`，**机体 FRD → 本地 NED**（即"当前姿态在 NED 中"） |
| `vx, vy, vz` | 速度（m/s），**机体系 FRD**（ArduPilot 会用 `q` 旋到 NED） |
| `rollspeed, pitchspeed, yawspeed` | 角速度；ArduPilot 的 331 处理**不使用**这三个字段（可填 0） |
| `pose_covariance[21]` | 行优先 6×6。ArduPilot 只用对角元素：`posErr = sqrt(c[0]+c[6]+c[11])`，`angErr = sqrt(c[15]+c[18]+c[20])`；若 `c[0]` 是 NaN → 视为 0（最信任）。位姿噪声会被 `VISO_POS_M_NSE`/`VISO_YAW_NSE` 夹住下限 |
| `velocity_covariance[21]` | ArduPilot 的 331 处理**不使用**（可填 0） |
| `reset_counter` | 估计器"重启/重初始化"计数。**每次你的估计器重置（重新初始化、重置原点）就 +1**，EKF 会据此接受位置跳变而不是当成坏创新 |
| `estimator_type` | `MAV_ESTIMATOR_TYPE_VISION`（2，视觉/LIO 通用） |
| `quality` | 0–100，`-1` 表示失败。只有 `quality >= VISO_QUAL_MIN`（默认 0）才被 EKF 消费，否则日志显示 `VISP Ign=1`（收到但不融合） |

**姿态/坐标关系（必须自洽）**：位置在 NED、速度在机体 FRD、四元数把机体 FRD 转到 NED。
如果 LIO 输出在 ENU/FLU，发送前做一次固定变换（本仓库 `bridge_nodes/common/frames.py` 里 `wire_to_world` / `world_to_wire`），
并特别注意 LIO 世界系的 yaw 与 LIO 输出四元数的关系（本仓库 `calib/calib.json` 的 `R_WL` 标定）。

## 4. 最小可运行示例（pymavlink）

```python
import time
from pymavlink import mavutil

mav = mavutil.mavlink_connection("tcp:127.0.0.1:5780" if sim else "/dev/ttyUSB0,57600")
mav.wait_heartbeat()

def send_odometry(t_us, pos_ned, quat_wxyz, vel_body_frd, reset_counter, quality=100):
    # 位置：NED（米，相对 EKF 原点）；速度：机体 FRD（m/s）；姿态：机体 FRD -> NED
    cov = [0.0] * 21
    mav.mav.odometry_send(
        int(t_us),                                   # time_usec（真实单调时钟）
        mavutil.mavlink.MAV_FRAME_LOCAL_FRD,         # 20
        mavutil.mavlink.MAV_FRAME_BODY_FRD,          # 12
        pos_ned[0], pos_ned[1], pos_ned[2],
        *quat_wxyz,                                  # [w, x, y, z]
        vel_body_frd[0], vel_body_frd[1], vel_body_frd[2],
        0.0, 0.0, 0.0,                               # 角速度（飞控不用）
        cov[0:21],                                   # pose_covariance（填 0 或 NaN 都行）
        [0.0] * 21,                                  # velocity_covariance（飞控不用）
        reset_counter,                               # 估计器重启计数
        mavutil.mavlink.MAV_ESTIMATOR_TYPE_VISION,   # 2
        quality,                                     # 0..100，-1 = 失败
    )

# 48 Hz 是经验值；10 Hz 也够 EKF，但 300 ms 内没有新数据就会被判不健康
t0 = time.monotonic()
while True:
    now = time.monotonic()
    send_odometry(int(now * 1e6), pos_ned, quat_wxyz, vel_frd, reset_counter=0)
    time.sleep(1.0 / 48.0)
```

真实项目里不要像我上面那样在循环里 `sleep`；用一个固定频率驱动器（本仓库是 1 kHz 线程里按 `--nav-hz 48` 节流），
**保持发送节奏稳定**，不要在命令等待、甚至标定过程中让这条流断（本仓库的真实故障：
`_command()` 等待 ACK 时没把喂数插进去 → 超过 300 ms → `Arm: VisOdom: not healthy`）。

## 5. ArduPilot 内部怎么处理（决定你踩不踩坑）

```
GCS_MAVLINK::handle_odometry(msg)                    libraries/GCS_MAVLink/GCS_Common.cpp
  ├─ 只接受 frame_id == MAV_FRAME_LOCAL_FRD(20) && child_frame_id == MAV_FRAME_BODY_FRD(12)
  │    否则 return —— 消息静默丢弃（不会 STATUSTEXT，也不报错）
  ├─ posErr/angErr ← pose_covariance 对角；c[0] 为 NaN 时视为 0
  ├─ timestamp_ms ← correct_offboard_timestamp_usec_to_ms()
  │    JitterCorrection：要求时间戳【单调、不能来自未来、落后 <500 ms】；
  │    任意"时基"都行（不需要和飞控时钟同步），但抖动大时会被校正/丢弃
  ├─ AP_VisualOdom::handle_pose_estimate(...)   ← 位置/姿态；quality < VISO_QUAL_MIN 不消费
  └─ handle_vision_speed_estimate(FRD→NED)      ← 速度
         └─ 写入 EKF3：writeExtNavData / writeExtNavVelData
EKF3：IMU 高频传播 + 外部导航观测融合 + 创新门限 + 超时/失效保护
```

关键阈值（默认值，4.5+）：
- **健康超时 300 ms**（`AP_VISUALODOM_TIMEOUT_MS`）：超过就 `VisOdom: not healthy`，预解锁直接拒绝 arm；
- EKF 对 331 位置/速度的**创新门限**与 `EK3_POSNE_M_NSE`/`EK3_VELNE_M_NSE` 相关，位置跳变会被拒；真跳变必须用 `reset_counter` 声明；
- 姿态 yaw 观测受 `VISO_YAW_NSE` 影响；
- `quality = -1` 或 < `VISO_QUAL_MIN` → 数据记录但不融合（BIN 里 VISP.Ign=1）。

## 6. EKF 原点与"本地帧"

- EKF3 需要一个**原点**（EKF origin）。纯外部导航机上：`COMMAND_LONG` `MAV_CMD_DO_SET_GLOBAL_ORIGIN`（611）
  或 `SET_GPS_GLOBAL_ORIGIN`，把原点设为你 LIO 世界系的原点（本仓库：起飞点、`home`）。
- 外部导航位置要**相对这个原点**，单位米，NED。
- EKF 的 local frame 是 NED（x 北、y 东、z 下）；`MAV_FRAME_LOCAL_FRD` 的"FRD"在 ArduPilot 的 331 处理里按本地
  NED 顺序解读（x=北/前、y=东/右、z=下）——只要你的原点与航向定义一致就不会有隐藏旋转。
- 若你的里程计世界系原点/航向和 EKF 原点不一致：先在地面对齐（发送前把 LIO 位姿变换到"EKF 原点 + NED"），
  不要指望飞控替你猜。

## 7. 上线前检查清单（真机）

1. **参数**：§2 全设；`VISO_TYPE 1` 必查（`param show VISO_*`）。
2. **帧**：`frame_id=20, child=12`，否则无数据（建议在发送侧加断言）。
3. **时间戳**：单调、真实时钟、不要用仿真时间或飞控时间；与飞控的时基可以不同（JitterCorrection 会映射）。
4. **速率**：≥10 Hz，推荐 20–50 Hz；我们有 48 Hz。中间断流 >300 ms 会被判不健康。
5. **原点**：起飞前设 EKF 原点（611 或 GPS origin）。
6. **地面验证**（不飞）：
   - 上电 → 发 331 → 看飞控 STATUSTEXT 出现 `EKF3 IMU0 is using external nav data`；
   - `LOCAL_POSITION_NED`（#32）应该跟着你的输入走（推着飞机走或转动，数值要合理）；
   - `EKF_STATUS_REPORT` flags 正常；
   - 预解锁能过（`VisOdom` 健康）。
7. **飞行验证**：GUIDED、小范围、低空；看 BIN 日志 `XKF4`（位置/高度创新 `SH`/`SP`）与 `VISP`（`RTimeUS`、`Ign`）。
8. **异常处理**：估计器重启 → `reset_counter++`；估计器失效 → `quality=-1`（EKF 会 dead-reckon 并最终触发 EKF failsafe → 降落保护）。
9. **不要**为了让 EKF 接受坏数据而关创新门限/失效保护；那是最后一道保险。

## 8. 常见故障对照表（本仓库真实遇到过的）

| 现象 | 根因 | 处理 |
|---|---|---|
| 预解锁报 `Arm: VisOdom: not healthy` | 331 间隔 >300 ms（常见：命令等待/标定/主循环卡住时喂数中断） | 喂数放在独立高频线程；命令等待期间也要继续喂 |
| 日志有 `VISP` 但 `Ign=1` | `quality < VISO_QUAL_MIN` 或 `quality=-1` | 提高 quality / 检查估计器状态 |
| 飞控位置完全不动 | `frame_id/child_frame_id` 不对（静默丢弃）或 `VISO_TYPE≠1` | 修帧对与参数 |
| `VISP.RTimeUS` 看起来"年龄 1.6 s" | 那是 **lag/延迟** 字段，不是"距上次更新"；JitterCorrection 映射后大 lag 会被丢 | 检查时间戳是否单调、是否来自未来、是否落后 >500 ms |
| EKF 刚开始正常、几秒后发散 | 喂数发送节奏抖动大/与其它重活共享线程；或位姿/速度坐标系不自洽 | 独立线程固定频率；用真值/日志核对位置/速度/姿态三者一致性 |
| EKF variance / EKF failsafe → 自动降落 | 外部导航跳变/漂移，被创新门限拒绝后失效保护 | 检查重置计数与估计器质量；不要手改 failsafe |

## 9. 与本仓库实现的对应

| 本文概念 | 本仓库实现 |
|---|---|
| 发送 ODOMETRY | `bridge_nodes/mavlink/mavlink_impl.py::send_external_nav()`（48 Hz、真实单调时间戳、nav gate、z trim、严格单调） |
| 帧对/四元数/速度 | `send_external_nav_l()` + `bridge_nodes/common/frames.py` |
| 参数 | `fcu_interface/params/apm_lio_only.parm`（`EK3_SRC1_* = 6`、`VISO_TYPE 1`、`GPS1_TYPE 0`） |
| 原点 | `bridge_nodes/mavlink/mavlink_impl.py::send_ekf_origin()`（611） |
| 记录/复核 | `nav.csv`（每帧实际发送内容 + FCU 侧 `ekf_flags`）、`/fcu/odom`、`H5` 行、`scripts/lio_drift.py` |
| 无飞控 bench | `tests/check_fcu_client.sh --bench`（SITL + 合成输入，验证速率/年龄门/时间戳） |

## 10. 没有 GPS 还能用 ODOMETRY 吗？——能，而且本仓库就是这么飞的

**能。ODOMETRY 与 GPS 无关**：GPS 只是 EKF 的另一个观测源；没有 GPS 时，EKF3 仍然用 IMU 传播 + 外部导航观测，
需要的是下面三件事：

1. **一个原点（EKF origin）**：`MAV_CMD_SET_GPS_GLOBAL_ORIGIN`（611）或 `SET_GPS_GLOBAL_ORIGIN`，给一组
   经纬高（通常就是起飞点）。没有原点，EKF 没有本地系，外部导航位置无参考；
2. **航向源**：`EK3_SRC1_YAW 6`（ExternalNav）。没有罗盘/没有 GPS 速度时，EKF 的 yaw 只能来自外部导航；
3. **持续的 331 数据流**（≤300 ms 间隔，见 §5）。

本仓库的飞行配置就是**"无 GPS、无罗盘、无气压计"**（`fcu_interface/params/apm_lio_only.parm`，
文件头注释原文："Mission A: the EKF takes EVERY source from the LIO. No GPS, no compass, no barometer."，
且该文件来自真机参数表）：

```
AHRS_EKF_TYPE 3
EK3_ENABLE    1
VISO_TYPE     1
EK3_SRC1_POSXY 6
EK3_SRC1_VELXY 6
EK3_SRC1_POSZ  6      # 高度也不用气压计
EK3_SRC1_VELZ  6
EK3_SRC1_YAW   6
EK3_SRC2/3_*   6      # 三个 lane 全部外部导航
GPS1_TYPE     0       # 无 GPS
SIM_GPS1_TYPE 0       # 仅 SITL 需要
COMPASS_USE   0       # 无罗盘（USE2/USE3 同样 0）
```

### 没有 GPS 时仍然正常的事
- **解锁/预解锁**：GPS、罗盘检查随 `GPS1_TYPE 0` / `COMPASS_USE 0` 自动不适用；本仓库所有飞行都无 GPS 解锁；
- **GUIDED / LOITER / LAND / RTL / AUTO 航点**：全部走本地位置估计，有 origin 就能用（RTL 回 origin 对应的 home）；
- **地面站地图**：设置 origin 后，ArduPilot 用"本地位置 + origin"反算 global 位置，地图上能看到飞机；
- **EKF 失效保护**：仍然生效（外部导航健康变坏 → EKF failsafe → 自动降落保护）。

### 没有 GPS 时**不能指望**的事
- **绝对位置校正**：没有 GPS 的长期修正，长时间飞行只靠 LIO，漂移会累积——这取决于你的 LIO 质量
  （本仓库实测 130 s、两圈、约 120 m 航程：LIO 相对真值漂移 0.1–0.4 m，见 `scripts/lio_drift.py`）；
  真机长航时建议 loop closure / 已知地标再定位 / 人工点；
- **无罗盘时的初始航向**：EKF 用外部导航的 yaw。上电时如果 LIO 世界系的朝向与真机机头不一致，会一直错；
  所以 LIO 世界系要在初始化时明确"机头方向"，且第一帧的 yaw 观测要合理；
- **高度基准**：没有气压计/GPS 高度，z 全部来自 331；origin 的 alt 决定 home 高度，z 有偏会直接变成高度误差
  （本仓库用 z-trim 在起飞点对齐）；
- **GPS 相关失效保护**：没有 GPS 就不会触发 GPS failsafe（这不是 bug，是自动禁用）；
- **GPS 混合场景**：不要设 `GPS1_TYPE 0`，而是让 GPS 作 SRC2/SRC3、extnav 作主源（`EK3_SRC_OPTIONS` 控制），
  这样 GPS 丢失时自动降级到 extnav。

### 新固件参数名注意
较新的 ArduPilot（含本仓库 SITL）里 `ARMING_CHECK` 已改名为 **`ARMING_SKIPCHK`**（位掩码，语义相反），
`GPS_TYPE` 已改为 `GPS1_TYPE`；写错名字会**静默丢弃**（PARAM_ERROR）。本仓库 `ARMING_SKIPCHK 0` 意味着
"不跳过任何检查"，在没有 GPS/罗盘时依然能解锁——因为那些检查不适用，而不是被跳过。

### 无 GPS 起飞前的验证顺序（真机）
1. 设 origin（611，经纬高=起飞点）→ 确认 STATUSTEXT 出现 EKF origin 设置成功；
2. 发 331 → 地面推/转飞机，看 `LOCAL_POSITION_NED`（#32）跟随、`/fcu/odom` 与你的输入一致；
3. 预解锁通过（`VisOdom` 健康、无 EKF prearm 报错）；
4. 地面站地图上位置合理（由 origin 反算）；
5. GUIDED 低空小范围试飞，盯 BIN 的 `XKF4`/`VISP`（见 §7）。
