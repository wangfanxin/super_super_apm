# ACCEPTANCE — 验收标准与判定政策

一趟飞行是否"可评分、通过"只由 `scripts/accept.py` 的 15 行决定（默认 **v1**；`--criteria-v2` 是
可选的提案语义）。本文件是所有行的定义、阈值、数据来源与判定政策。

用法：

```bash
python3 scripts/accept.py --run .runs/missionA-<tag>-01-01 \
    --world sim/worlds/forest_hex12.yaml --alt 2.0
```

## 数据来源（accept.py 只读记录文件）

| 文件 | 内容 |
|---|---|
| `tap.csv` | 真值（自有 MAVLink SIM_STATE 客户端），W 系：x=pe, y=pn, z=-pd；速度 ve/vn/vd |
| `setpoints.csv` | plan box 发出的 cmd（W）+ 飞控估计 fcu_*（W）+ LIO 原始位姿 lio_*（L） |
| `nav.csv` | 外导航样本（L）+ FCU 估计（W）+ 拒绝原因/age |
| `lio.csv` | Super-LIO 原始里程计（L，200 Hz） |
| `scans.csv` | 雷达每帧 cast 时间/点数/跳过原因（H6） |
| `phase.csv` | supervisor 阶段时间线（H1 的 rtfs 窗口） |
| `traj.log` / `fcu_client.log` | plan box/FSM 与 supervisor/bridge 的日志（H2/H4/W1/W4 细节与 abort 原因） |
| `accept.json` | 本脚本输出（全部行 + 来源） |

## 15 行（v1，默认）

### H 行：链路/宿主健康（不健康则 W 行不可评）

| 行 | 判据 | 数据 |
|---|---|---|
| **H1** 宿主实时性 | 起飞..落地窗口的 flight_rtf ∈ [0.98, 1.02] | `phase.csv` |
| **H2** tap 完整性 | **2026-11-26 修订**：按**配置的 200 Hz**（±5%）+ max gap ≤ 10 ms + ≥1000 样本判定（旧 380 Hz 是 tap 时代数字，对当前链路无信息量）；H2 不再永远 F | `tap.csv` |
| **H3** LIO 存活 | LIO odometry 覆盖整个飞行窗口、频率正常 | `lio.csv` |
| **H4** 参考存活 | SUPER 自己的参考（非 fallback hold）出现过 ≥1 次 | `setpoints.csv` + `traj.log` |
| **H5** 真值/飞控一致性 | tap 与 FCU 估计的跨度差在容差内（水平 5 m、垂直 2 m 量级），否则"两套数据描述的不是同一趟飞行" | `tap.csv` + `setpoints.csv` |
| **H6** 雷达预算 | cast p50 ≤ 100 ms、skipped ≤ 20；超限的趟**不可评分**（先重飞） | `scans.csv` |

### W 行：任务质量

| 行 | 判据 |
|---|---|
| **W0 / W0b** | 真值确实起飞（>0.5×alt）且在任务高度带内（2.0 ± 1.50 m，≥90% 路线样本） |
| **W1** | 计划器自报的航点到达数 = 计划航点数（来自 `traj.log`，**是计划器自己的话**） |
| **W1b** | 真值**按顺序**进入每个航点 0.50 m 盒（单调、无跳号；launch pad 单独计） |
| **W2** | 巡航速度：路线内 \|v_xy\| **p90 ≥ 1.40 m/s** 且 **max ≥ 1.50 m/s**（交付档 1.35 m/s 的合理承诺；SPEC 速度的修订记录见 `docs/PIPELINE.md`） |
| **W3** | 高度保持：bias（真值−指令）中位在 ±0.30 m、95 分位偏移 ≤0.80 m（注意：EKF 高度基准 0.12 m 需由 `REF_Z_OFFSET` 补偿，否则 bias 恒 −0.11） |
| **W4** | 柱面净空：真值到每个柱面的最小距离 ≥ 0（<0 即接触）；需要 `--world`。**注意**：v1 默认 `--body circle` 且把机体当**点**；交付飞行用 `--body box`（躯干方盒）。它**不含** planner 的 robot_r=0.10 机身半径——按机身半径判定时看 `*-clearance.png` 的 body margin（见 FLIGHT-ANALYSIS §35.3） |
| **W5** | 绕行：飞过的路径相对"角点折线"的偏移 p90 ≥ 0.50 m（证明它在绕柱而不是直线飞） |
| **W6** | 跟踪：cmd vs truth 的零时延误差 p50/p90 及四等分时间基偏移在界内（v1 的界按交付档标定；v2 是可选语义） |

`GATE:` 行的规则：H 行全部通过（或明确豁免）且 W 行全部通过才 PASS；H2 是**已知 P0**（见下）。

## 已知 P0

- **H2 修订（owner 2026-11-26："H2 没意义"）**：旧 380 Hz 是 tap 时代数字（当时真值 1200 Hz 插值/400 Hz）；
  当前真值链是 SIM_STATE 108 的**配置 200 Hz**流（D4 时间基）。H2 现在判"是否按配置速率、无真缺口到达"：
  `200 Hz ±5%`、`max gap ≤ 10 ms`、`≥1000 样本`。23c/27b/28b/28c 全部 PASS（200.0 Hz、max gap 5 ms、35k 样本）。
  无 `t_sim_us`/`truth_timebase.json` 的历史 tap 退回旧语义并标注 legacy。

## 新增质量行（Q1–Q4，owner 2026-11-26："一些新判断放进去"）

`scripts/quality_rows.py`（`run_accept.sh` 每趟自动跑，`--json` 写 `quality.json`）。这些行描述"飞得好不好"，
不在原 15 行里：

| 行 | 判据 | 为什么 |
|---|---|---|
| **Q1 中段低速/绕圈** | 离所有角点 >1.5 m 的**路径中间**，真值 \|v\|<0.5 m/s 持续 ≥0.15 s 的段 = **0 段** | owner 的普查规则；路径中间 stall/loop 是"不干净"的直接证据 |
| **Q2 机身半径净空** | 真值 body-edge margin = `surface_dist(躯干 box) − robot_r(0.10)` 的最小值 ≥ **−0.02 m** | W4 是"点对 box"；planner 自己按 0.10 m 机身规划 |
| **Q3 中段跟踪** | 离角点 >1.5 m 的 cmd−truth 水平误差 **p90 ≤ 0.25 m** | 速度阶梯的前置条件（agile PSC 实测 0.17–0.20） |
| **Q4 角点动力学** | 每角点每次通过：1.5 m 内停留 ≤3.5 s **且** 真值最低速度 ≥0.40 m/s | 直接量化"角点停-转"这个交付缺陷 |

**当前状态（4 趟 H6-clean：23c/27b/28b/28c）**：Q3 全部 PASS；Q1/Q2/Q4 仍 FAIL
（Q1 3–7 段；Q2 −0.013…−0.091 m；Q4 角点真值最低速 0.01–0.32 m/s）。
即：**15 行表全过 ≠ 飞干净**；Q1/Q2/Q4 是下一阶段验收重点。

## v2 语义（可选提案）

`--criteria-v2` 会改变 H2/H6/W2/W4/W6 的评分语义（更贴近当前链路的实测；例如 H2 不再要求 380 Hz、
W4 加入"路线在任务高度带的采样"等）。默认关闭，**所有正式结论用 v1**；v2 只用于分析对照。
语义细节在 `accept.py` 的 `--criteria-v2` 分支中，并由 `tests/check_criteria_v2.py` 覆盖。

## 证据政策（与 owner 约定一致）

1. **判定只看 `accept.py`**；图（topdown/xyz-t/速度/角点）用于解释，不参与判定。
2. **H6 不合格的趟不可评分**：雷达 starvation 会连锁污染 LIO/EKF/规划，数字没有意义，先 cleanup 重飞。
3. **质量门（提速前必须）**：`scripts/flight_quality.py` —— Q1 绕圈=0、Q2 中段指令停顿=0、
   Q3 参考连续性、Q4 跟踪；此外每次飞行后跑 `flight_forensics.py`（误差分解/角点/高度）和
   `low_speed_census.py`（先列所有 <0.5 m/s 段再分类）。**W4 必须为正**，否则不算"飞干净"。
4. **单趟不过即不过**：不因为"总体趋势好"放过一趟；统计结论需要 ≥3 趟 H6-clean 的飞行。
5. **地图/航点冻结**：`scripts/check_frozen.py` 不通过就不起飞；换图需 owner 批准并更新指纹
   （见 `docs/README.md` 与 `docs/PIPELINE.md`）。
6. **只飞 SITL，不飞真机；不 push。**

## 当前基线（2026-11-28 校正）

- 当前最佳（新二进制，item2/4 修复后）：`phase180b-01-01` —— **15 行全过**、W4 **CLEAR +0.061**（全速率）、
  W6 p90 0.209、Q3 0.145、Q1 **0**、P1 PASS（max 0.355，0 个 >0.5 m）、S2 2 环/3.7 s、S1 1.17 s；
  Q2 **−0.039（仍 FAIL，门槛 −0.02）**、Q4 c3 vmin 0.32（仍 FAIL）。
- 旧二进制对照：`phase177b/177c` —— 15 行 14/15（W4 CONTACT −0.000 / CLEAR +0.006）、W6 0.231/0.241、
  Q1 1/2；S2 老旧口径不可直接比。
- 证据：`docs/evidence/final/sheet-BEST-180b.png` + `phase180b-01-01_*`（详见 `docs/evidence/INDEX.txt`）。
- 完美定义见 `docs/FLIGHT-ANALYSIS.md` §124/§127：有效性 + 15 行 + W4>0 + Q2≥−0.02 + Q1=0 + Q4 全过 +
  R2R（chase=0 / mid-leg=0 / full-leg≤2/趟），连续 2 趟同配置。

## 泛化评估（owner 2026-11-26）

冻结 SPEC 图的验收仍是 15 行 + Q1–Q4 的**硬门**；此外，交付前必须在**至少 4 张同规则不同 seed 的地图**上
跑同一配置（见 `docs/PIPELINE.md §6b`），并报告最差那张的结果（调参不得只看单图）。

首次套件结果（agile PSC，2026-11-26）：

| 地图 | 15 行 | W4 点净空 | Q1 中段低速 | Q2 机身净空 | Q3 中段跟踪 | Q4 角点 |
|---|---|---|---|---|---|---|
| 冻结 `forest_hex12`（3 趟 27b/28b/28c） | 15/15（3/3） | +0.087/+0.013/+0.060 | 2–7 段 FAIL | −0.013…−0.087 FAIL | PASS | FAIL |
| seed22 `forest_hex12_s22` | **14/15（W4 接触）** | **−0.051** | 3 段 FAIL | −0.155 FAIL | PASS | FAIL |
| seed23 `forest_hex12_s23` | 15/15 | +0.056 | 3 段 FAIL | −0.045 FAIL | PASS | FAIL |
| seed24 `forest_hex12_s24` | 15/15 | +0.056 | 2 段 FAIL | −0.073 FAIL | PASS | FAIL |

结论：**15 行表在 4 张图里 5/6 趟全过**（seed22 一次 W4 接触），泛化性初步可以；但 Q1/Q2/Q4 在所有图上都 FAIL，
说明"停-转/窄缝/机身净空"是**跨地图的系统性问题**，不是冻结图的偶然。下一步：加 seed 到 6 张、用最差 seed 做验收门。
