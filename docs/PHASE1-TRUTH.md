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

