# 跟踪质量诊断 & 高速竞速方案（2026-11-25 调研结论）

> 目标：先回答"为什么无避障时轨迹也不贴"，再给出从当前 **1.35 m/s** 到 **8–10 m/s** 的分阶段方案。
> 依据：本仓库实测（phase4q/4s 飞行数据、ArduPilot 4.7.1 源码 `.build/ardupilot`、参数实测 dump）、
> SUPER 论文/GitHub、ArduPilot GUIDED 文档。**不改任何验收判据**，方案里的判据影响在最后单列（P0）。

## 0. 结论先说

1. **不是飞控 PID 没调好**：姿态/角速率环（ATC_*）保持固件默认，实测姿态跟踪正常；1.35 m/s 下没有振荡迹象
   （横向误差 p90 0.154 m，且是"滞后"不是"振荡"）。
2. **不是控制量不完备**：我们发的是完整 PVA（`wire.csv`: `fields=posvelacc`, type_mask=2048），
   ArduPilot 4.7 GUIDED 的 `posvelaccel` 路径是 `AC_PosControl::input_pos_vel_accel_NE_m(..., limit_output=false)`
   （`ArduCopter/mode_guided.cpp:943`、`AC_AttitudeControl/AC_PosControl.cpp:633`）——**前馈位置/速度/加速度原样使用，
   不受 WP_* 限幅**；限幅只作用在"位置误差修正"上。
3. **软的原因是"修正环被限得很保守 + 参考本身有停顿"**。实测（phase4q，2 m 高度段）：
   - 指令 vs 真值：零滞后误差 **p50 0.155 m / p90 0.266 m**，最优滞后 **−0.13 s**（飞机落后参考约 0.13 s）；
     沿航迹误差 p50 **−0.124 m**（落后），横侧 p50 −0.023 m（很小）→ 是"追不上"，不是"跑偏"。
   - 参考加速度：|a_xy| p50 0.72、p90 **1.45**、p99 **2.81**、max 3.94 m/s²。
   - 飞行实际生效的 FCU 参数（SITL dump 含最后加载的 `guided_limits_fast.parm`）：
     `WP_SPD 1.5`、`WP_ACC 4.0`、`PSC_NE_POS_P 1.2`、`PSC_NE_VEL_P 2.0`、
     `ATC_ANGLE_MAX 45°`、`PSC_NE_JERK 80`。
     ⚠ 在 PVA 模式下 jerk 限幅取的是 **`PSC_NE_JERK`（80 m/s³，宽松）**，不是 `WP_JERK`：
     `AC_PosControl::NE_set_max_speed_accel_m`（`AC_PosControl.cpp:468`）里
     `_jerk_max_ne_msss = _shaping_jerk_ne_msss;`，而 `_shaping_jerk_ne_msss` 就是 `PSC_NE_JERK`
     （同文件:136）。`WP_JERK` 只影响"只发位置"的航点导航路径。
   - **主因排序（实测 + 源码）**：
     1. **总延迟 ≈ 0.13 s**（最优滞后 −0.13 s；1.35 m/s 时 = 0.18 m 沿航迹误差，正好对上 p50 0.155 m）。
        组成：LIO 200 Hz + extnav 送飞控 ~48 Hz（`--nav-hz 48`）+ 84 setpoint 50 Hz + 姿态内环；
     2. **位置环增益偏低**（`PSC_NE_POS_P 1.2`，4.7 默认 3.0 附近；前几轮 A/B 里 2.4/3.0 更紧但会牺牲角点净空）；
     3. **参考本身加速度需求已接近修正能力**：参考 |a_xy| p90 1.45 / p99 2.81 / max 4.02 m/s²，
        当前 **0%** 超过 WP_ACC 4.0（所以 1.35 m/s 下修正限幅还没饱和）；
        但参考加速度 ≈ v²/R 随速度平方增长：到 4 m/s 同一批角点就是 ~13 m/s²，**必然被 WP_ACC 4.0 削平**
        —— 这就是"现在看起来只是软、高速会直接跟不上"的机理；
     4. 参考的停-走/角点刹停（`docs/FLIGHT-LOOP-ANALYSIS.md`）周期性制造大误差。
     4.7 的 `limit_output=false` 只保证**前馈不被削**，修正仍受 `WP_SPD/WP_ACC/ATC_ANGLE_MAX` 约束。
   - 参考本身的停-走/角点刹停（`docs/FLIGHT-LOOP-ANALYSIS.md`）让误差周期性变大，进一步放大这种印象。
4. **SUPER 原论文的能力远高于我们当前设置**：280 mm 轴距、推重比 >5、70 m 3D LiDAR，报道 **>20 m/s** 穿越细障碍
   （[Science Robotics 2025](https://www.science.org/doi/10.1126/scirobotics.ado6187)、
   [论文摘要](https://search.library.wisc.edu/article/cdi_pubmed_primary_39879279)）。它的机器人用 PX4 系飞控
   （[SUPER GitHub](https://github.com/hku-mars/SUPER) 列出 Nxt-FC/PX4 硬件平台），与我们的 ArduPilot GUIDED 是两条控制栈；
   我们的 SITL 机体、参数与世界几何都还没按"竞速"配置。

## 1. 当前链路里"限制速度/精度"的东西（按影响力排序）

| 层 | 限制 | 现在值 | 影响 |
|---|---|---|---|
| 规划器 `hku_super/config/super_planner_hex.yaml` | `traj_opt.boundary.max_vel` / `max_acc` | **1.35 m/s** / 6.0 m/s² | 参考本身就只有 1.35，W2 门限也是按它定的（第 20 轮） |
| 规划器 | `planning_horizon` / `receding_dis` / `robot_r` | 16 m / 1.0 m / 0.10 m | 8–10 m/s 时转弯半径 v²/a（45°: 6.5 m @8 m/s）超出 10 m 半径的六边形走廊 |
| 适配器 | `--ref-max-speed-mps` | 6.0（默认）/ 本次未显式改 | 超过 6 的参考会被限速 |
| FCU 参数 | 修正环 `WP_SPD / WP_ACC / WP_JERK` | 1.5 m/s / 4 m/s² / **1.0 m/s³** | 误差修正慢；WP_JERK=1 尤其保守 |
| FCU 参数 | 位置/速度环 `PSC_NE_POS_P 1.2 / VEL_P 2.0` | 1.35 m/s 档调出来的 | 更高速度需要重新调（更高增益+更高 jerk 限） |
| FCU 参数 | `ATC_ANGLE_MAX` | 45°（加速度能力 9.8 m/s²） | 8 m/s 转弯需要更大侧向加速度 → 更大倾角或更大半径 |
| 控制频率 | 84 setpoint | 50 Hz | 高速下 20 ms 的参考量化会放大滞后 |
| 估计 | LIO 200 Hz + EKF extnav 48 Hz | — | 48 Hz 喂数 + 估计延迟 ≈ 0.1 s 量级；高速下成为相位滞后主项之一 |
| 世界/几何 | 六边形半径 10 m、柱距 1 m | — | 8–10 m/s 不可行（转弯半径不够）；竞速需要长直道/大半径世界 |

## 2. 分阶段方案（每一步都可飞、可量化、可回滚）

### 阶段 0：先量化"控制器在哪个速度开始跟不上"（不改变交付配置）
- 用一个**无避障的合成参考**（大圆 r=8–10 m，或长直道 20 m，速度阶梯 1.35 → 2 → 3 → 4 → 6 m/s），
  每档飞 1–2 趟，输出：零滞后误差 p50/p90、最优滞后、沿/横分解、饱和比例（|a_cmd|>4 m/s² 的时长）。
- 现有工具：`scripts/fly_speed_run.sh`（已封装速度 arm + 三项测量）、
  `scripts/traj_error.py`（APE/RPE）、`scripts/analyze_loops.py`（停走/绕圈）。
  需要新增一个"跟踪质量"小工具（把上面的指标做成一行输出），可与 `fly_speed_run.sh` 串联。
- 产出：tracking-vs-speed 表 → 得到 ArduPilot 在当前参数下的"拐点速度"。

### 阶段 1：先补延迟、再调硬（每项单变量 A/B）
- **延迟补偿（优先，因为主因）**：
  `REF_LEAD_S=0.13`（适配器已有 `--ref-lead-s`：把目标沿 SUPER 速度方向外推 T 秒，注释里就是给
  "参考滞后"用的）、`--ref-lead-turn-rad-s` 配合弯道；
  提高环路频率：`--nav-hz 48 → 60`（ODOMETRY #331，桥有 35–60 Hz 的健康区间判断）、
  84 setpoint `--wire-hz 50 → 100`；LIO 输出频率/`VISO_DELAY_MS` 标定。
- **参数调硬（每次一趟）**：
  `WP_ACC 4 → 6`、`WP_SPD 1.5 → 3`、`PSC_NE_POS_P 1.2 → 2.0 → 3.0`、
  `PSC_NE_VEL_P 2.0 → 3.0`、`PSC_NE_JERK 80 → 120`（先看 0.5–2 Hz 增益）、
  `ATC_ANG_RLL/PIT 6 → 10`、`ATC_ANGLE_MAX 45 → 55`。
- 每档都看：tracking 指标 + W1b/W4（历史教训：加硬会把角点净空吃掉；`apm_guided_fast.parm` 的注释里有
  POS_P 2.4/3.0/5.0 的实测对比）。
- 需要新参数文件 `guided_limits_race.parm`（不动交付的 `guided_limits_fast.parm`），并让 `fly_hex.sh` 可切。

### 阶段 2：规划侧提速（3 → 6 m/s）
- `traj_opt.boundary.max_vel: 1.35 → 3.0 → 6.0`，`max_acc` 同步（保持 a_lat ≤ g·tan(ATC_ANGLE_MAX)）；
  新建 `super_planner_race.yaml` 而不是改交付配置。
- `REF_MAX_MPS` 升到 8；`--ref-ff`/`--ref-gov-*` 复核；`--goal-pva` 保留（角点不停车）。
- 世界：新增**竞速世界**（长直道/大半径弯，柱距不变），因为 6 m/s 在六边形里转弯半径已 >4 m。
- 验收：W1b/W4 必须仍然过；W2 会自然超过当前门限（P0 判据问题，见 §4）。

### 阶段 3：8–10 m/s（需要控制栈升级评估）
- 能力核算：8 m/s、45° → 转弯半径 6.5 m；10 m/s → 10.2 m。若要小半径，需要
  `ATC_ANGLE_MAX` 60–65°（a = 17–21 m/s²），先确认 SITL 机体 T/W 与电机/推力饱和（读 SITL 模型参数，
  必要时把机体模型调到 T/W ≈ 4–5，与 SUPER 论文的 >5 对齐）。
- 控制栈两条路线：
  A. **继续 ArduPilot GUIDED**：84 setpoint 提到 100 Hz；修正环参数再调；EKF extnav 延迟补偿
     （`VISO_DELAY_MS` 标定）；估计器输出频率提高（LIO → 331 提到 60–100 Hz）。上限预计 5–7 m/s。
  B. **自建姿态+推力环（SUPER 式）**：`SET_ATTITUDE_TARGET` + 我们自己的位置/速度/加速度控制器
     （ArduPilot 只跑姿态/角速率内环）。这是 SUPER 在 PX4 上的做法，能突破 GUIDED 级联的带宽限制；
     代价是要写控制器、做安全门、真机验证，属于独立里程碑。
- 建议：先做 A 到 6 m/s；用阶段 0 的拐点数据决定是否需要 B。

## 3. 需要新增/复用的工具

| 工具 | 用途 |
|---|---|
| `scripts/track_quality.py`（新） | 从一趟 sortie 输出：零滞后误差、最优滞后、沿/横分解、参考加速度分位、修正饱和比例、按速度分箱 |
| `scripts/fly_speed_run.sh`（已有） | 速度 arm 的飞行封装 + 三项测量钩子 |
| 新增 `hku_super/config/super_planner_race.yaml`（新） | 竞速规划参数（max_vel/max_acc/robot_r/horizon），不改交付配置 |
| 新增 `fcu_interface/params/guided_limits_race.parm`（新） | 竞速 FCU 参数，`EXTRA_PARMS=` 切换 |
| 新增竞速世界 `sim/worlds/race_straight.yaml`（新） | 长直道/大半径，供 6–10 m/s |
| `docs/metrics/speed-ladder-*.txt` | 每档速度的 tracking 数据和结论 |

## 4. 与验收判据的关系（P0，**只记录不改**）

- **W2**（现要求 p90≥1.40、max≥1.50 m/s）：竞速后必须重定义（如 p90≥6、max≥8）；
- **W4**（净空 >0）：高速下需要更大走廊和新的 world，阈值/几何都要重议；
- **W6**（滞后 spread ≤0.15 s）：高速下滞后绝对值会增大，判据应改成"相对速度归一化 + 去线性漂移"（已在
  `docs/P0-CRITERIA-PROPOSAL.md` 提案）；
- **H2/H6**：真值速率 380 Hz 的旧判据（已提案改）；高速时雷达 10 Hz/IMU 200 Hz 是否足够也要重新评估。
  以上都需要所有者批准后才动 `accept.py` 的默认值。

## 5. 参考来源

- SUPER 论文与摘要（>20 m/s、280 mm、T/W>5、70 m 3D LiDAR）：
  [Science Robotics 2025](https://www.science.org/doi/10.1126/scirobotics.ado6187)、
  [UW 摘要页](https://search.library.wisc.edu/article/cdi_pubmed_primary_39879279)、
  [HKU 新闻](https://engg.hku.hk/News-Events/Details/id/8718)、[GitHub](https://github.com/hku-mars/SUPER)
- ArduPilot GUIDED 命令文档（速度/加速度命令需要持续重发、加速度字段语义）：
  [Copter Commands in Guided Mode](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html)
- ArduPilot 4.7 源码（本仓库 `.build/ardupilot`）：
  `ArduCopter/mode_guided.cpp:943`（posvelacc 用 `limit_output=false`）、
  `libraries/AC_AttitudeControl/AC_PosControl.cpp:633`（限幅只作用于修正）、
  `AC_PosControl.cpp:458`（`NE_set_max_speed_accel_m`）
- 本仓库历史实测：`fcu_interface/params/apm_guided_fast.parm` 头注释（PSC 参数命名/取值 A/B）、
  `scripts/fly_hex.sh`（pos/posvel/posvelacc 对比：pos 时 3/19 航点，posvelacc 时 19/19）、
  `docs/FLIGHT-LOOP-ANALYSIS.md`（停走/绕圈归因）。

## 6. 立刻可做的第一步（建议）

1. 基准已建立：`python3 scripts/track_quality.py --tag phase4q-01-01` →
   zero-lag p50 0.154 / p90 0.265 m、best lag **−0.13 s**、参考 |a| p90 1.45 m/s²、
   速度分箱 1.0–1.5 m/s 档 p90 0.182 m；下一步在 `REF_LEAD_S=0.13` 与 `PSC_NE_POS_P=2.0` 上做单变量 A/B。
2. 用 `fly_speed_run.sh` 的竞速 arm 参数（新 `super_planner_race.yaml`，max_vel 3）飞 2 趟，测 tracking 拐点；
3. 再按阶段 1 逐项调 FCU 参数 A/B。每一步都只 commit 本地、不 push、不改默认交付配置。
