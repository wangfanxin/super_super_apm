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
| **H2** tap 完整性 | rate median ≥ 380 Hz（v1，历史 tap 语义）、max gap ≤ 10 ms、静止段 |a| 合理；**已知 P0：当前 200 Hz SIM_STATE 永远判 F**（见下文） | `tap.csv` |
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
| **W4** | 柱面净空：真值到每个柱面的最小距离 ≥ 0（<0 即接触）；需要 `--world` |
| **W5** | 绕行：飞过的路径相对"角点折线"的偏移 p90 ≥ 0.50 m（证明它在绕柱而不是直线飞） |
| **W6** | 跟踪：cmd vs truth 的零时延误差 p50/p90 及四等分时间基偏移在界内（v1 的界按交付档标定；v2 是可选语义） |

`GATE:` 行的规则：H 行全部通过（或明确豁免）且 W 行全部通过才 PASS；H2 是**已知 P0**（见下）。

## 已知 P0

- **H2**：当前真值链走 SIM_STATE 108 的 200 Hz 流（时间基 D4 要求），而 H2 的 v1 语义仍要求 ≥380 Hz
  （tap 时代产物）。**任何趟都不会过 H2**，这是记录项，不是回归。验收结论写作"除 H2 外全过"。

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

## 当前基线（2026-11-26）

- 历史最佳实飞：`phase9f-01-01` —— 除 H2 外 15/15 行全过；W4 **+0.027 m CLEAR**、W6 p90 0.262、W2 p90 1.43。
- 新基准（9f+，待飞确认）：9f 的几何/角点设置 + `REF_Z_OFFSET/REF_Z_HOLD` + 垂直环软化；
  预期把 W3 bias 从 −0.113 修到 ~+0.02，同时保留 W4 CLEAR。
- 基线航班与证据：`docs/evidence/phase9f-*`（详见 `docs/evidence/INDEX.txt`）。
