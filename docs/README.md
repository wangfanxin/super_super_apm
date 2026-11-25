# super_super_apm —— 自主飞行管线（SUPER 规划 + ArduPilot SITL/真机 + 自建 MAVLink 链路）

本仓库是"**可复现、可验收**"的自主飞行栈：SUPER(MINCO) 规划器 → 本仓库的 plan box（航点状态机 + 参考整形）
→ 自建 MAVLink 客户端 → ArduPilot GUIDED；真值来自自己的 MAVLink SIM_STATE(108) 客户端；所有飞行都落盘
（bag + 每趟 CSV + 图），`accept.py` 用 15 条判据打分。

约束：**D1 不用 mavros；D2 不用 Gazebo；D3 真值 = 自己的 SIM_STATE 客户端；D4 墙钟 + bag + 显式时钟映射；
D5 一切在仓库内**（含构建脚本、SITL 参数、世界、验收与图）。

## 文档（只有 5 篇）

| 文档 | 内容 |
|---|---|
| **[README.md](README.md)** | 本页：入口、快速开始、当前状态、证据图索引 |
| **[PIPELINE.md](PIPELINE.md)** | 架构/数据链/坐标系/时钟/阶段 0–5 的规定-实现-证据（3400 行，全部技术细节） |
| **[ACCEPTANCE.md](ACCEPTANCE.md)** | 验收 15 行、v2 提案、交付清单、真机迁移清单、路径政策、已知缺陷 |
| **[FLIGHT-ANALYSIS.md](FLIGHT-ANALYSIS.md)** | 绕圈机理、跟踪/滞后分解、角点 A/B（phase4p–4y）、高速竞速方案（1.35→8–10 m/s） |
| **[CHANGELOG.md](CHANGELOG.md)** | 按轮次的变更与测量记录（含历史集成记录摘要） |

> 历史长文（INTEGRATION.md 112 节、harness-history、旧 SPEC/MISSION/CHAIN 等）已并入上述 5 篇或留在
> git 历史中（`git log -- docs/INTEGRATION.md` 等），不再作为独立文档维护。

## 快速开始

```bash
# 0) 环境（路径权威）
source setup/env.sh

# 1) 构建（两阶段：先消息包/rog_map，再 super_planner；LIO 单独一个 workspace）
bash setup/build.sh

# 2) 门禁（约 2–3 分钟；GATE: PASS 才允许 commit）
bash tests/run_all.sh

# 3) 飞一趟（SITL + LIO + SUPER + 世界；bag 可选）
BAG=1 BAG_SPLIT=1 bash scripts/fly_hex2.sh <tag> 1

# 4) 验收（15 行；--criteria-v2 为 P0 提案，默认 v1）
python3 scripts/accept.py --run <sortie> --world sim/worlds/forest_hex12.yaml --alt 2.0 --json out.json

# 5) 分析/出图（每趟飞行后全套）
python3 scripts/analyze_loops.py      --tag <tag>
python3 scripts/track_quality.py      --tag <tag>
python3 scripts/plot_flight_topdown.py --tag <tag> --per-lap
python3 scripts/plot_flight_timeseries.py --tag <tag>
python3 scripts/plot_corner_details.py --tag <tag>
```

## 当前状态（2026-11-25 夜）

- 验收基线（交付配置）：`phase4oF` / `phase4oH`（见 ACCEPTANCE）；15 行里已知只有 **H2（P0 待批）** 与
  **W4（±5 cm 接触，显式记缺陷）** 不过。
- **速度接口修复**（11-25）：Super-LIO 的 robot twist 补齐 → rog_map 读入 → SUPER hot_start 生效；
  腿中"指令停顿"11→3、飞机绕圈 5→1（phase4p→phase4q）。交付配置已含此修复。
- **跟踪主因**（FLIGHT-ANALYSIS §2）：不是 PID/前馈缺失；控制器看到的状态比真值旧 ~60 ms
  （LIO 滤波 35 ms + EKF 融合 25 ms），控制环本身 ~10 ms 量级；1.35 m/s 时对应 8 cm，8 m/s 时 0.5 m。
- **角点"不停车"**：`GOAL_PVA + bisector + corner speed 1.2` 可用（phase4s：角点转角 172°→139°，验收不退化）；
  `PRE_GOAL` 与验收"按序到点"口径冲突（phase4x 12/13 到点但 W1b 仍 F）；`GOAL_LOOKAHEAD` 当前不可用（phase4y 坠地）。
- 下一步：先确认/固化交付配置，再按 FLIGHT-ANALYSIS 的速度阶梯（2→3→4 m/s）与估计延迟降低推进竞速。

## 证据图索引（`docs/evidence/`，全部可重出）

- **每趟全套**（phase4x/4y 为最新）：`<tag>-topdown.png`、`<tag>-topdown-lap{1,2,...}.png`、
  `<tag>-xyz-t.png`、`<tag>-vxyz.png`/`<tag>-velocityfix-vxyz.png`、`<tag>-speed-t.png`、`<tag>-corner{1..6}-zoom.png`
- **速度接口修复前/后**：`fix-velocity-compare.png`（指令停顿/绕圈统计）、`phase4q-velocityfix-*.png`
- **角点对比**：`corner-ab-turn.png`（phase4p/q/s 每角点转角与最小指令速度）、`phase4p/4q/4s/4x-corner*-zoom.png`
- **绕圈机理**：`loop-mechanism.png`（指令速度阶跃 + 过冲回切）
- **滞后分解**：`latency-decomposition.png`（真值-指令 0.11 s 中估计 0.06 s、控制 ~0.01 s）
- **早期证据**：`world-hex12.png`、`lap1-paths.png`、`lap2-paths.png`、`vxyz-t.png`、`lio-err-t.png`
- 逐图说明与重画命令见 `docs/evidence/INDEX.txt`。


---

## 附：证据图逐张说明（原 evidence/README.md）

> 原文已合并到 README；独立文件改名为 `evidence/INDEX.txt` 保留给校验脚本。

## 证据目录（第 88 轮：新地图 `forest_hex12.yaml` 的两圈飞行）

本轮三件事：**清理工作空间** → **换地图**（±12 m、522 柱、边长 10.000 m 六边形）→ **在新图上飞完两圈**。
旧的飞行图（`pv1-*`、`d84-*`、`d85-*`、`r15b-*`、`v4a-01-*`、`v3pv-*`、`v3d-*`、`v3h132-*`、`compare/*`）
在换图时**已全部删除**，`scripts/check_evidence_links.sh` 断言它们不在，也断言本目录里没有别的命名方案。

现在这个目录里恰好六件东西：

| 文件 | 是什么 | 重画命令 |
|---|---|---|
| `lap1-paths.png` | **第 1 圈俯视图**：红 = SUPER 规划路径，蓝 = 真值轨迹，深色小方块 = 柱，空心方块 = 航点。标题给出该圈的规划/真值最小柱面净空与 \|规划−真值\| p50/p90 | `python3 scripts/plot_hex_laps.py --tag kx6-03-01-01 --laps 2 --world worlds/forest_hex12.yaml --out docs/evidence --prefix lap` |
| `lap2-paths.png` | 同上，第 2 圈 | 同上（一次出两张） |
| `vxyz-t.png` | `v_xy` / `v_z` / `x` `y` `z` 对时间（规划 vs 真值） | `python3 scripts/plot_hex_vxyz_t.py --tag kx6-03-01-01 --world worlds/forest_hex12.yaml` |
| `lio-err-t.png` | 估计器误差对时间 + 累计分布（水平/竖直分开） | `python3 scripts/plot_hex_lio_err_t.py --tag kx6-03-01-01` |
| `world-hex12.png` | **新地图的评审图**：场地、522 根柱、角点 3×3 空格、10 m 六边形航线、A\* 路线与每腿最窄点 | `python3 scripts/plot_world_hex.py --world worlds/forest_hex12.yaml --out docs/evidence/world-hex12.png` |
| `README.md` | 本文件 | — |

数据来源：**`kv1-02-01`**（`/home/fanxin/lio_avoid/logs/missionA-kv1-02-01`，**审查修复后的第一趟**：
rtf **1.0005**、0 abort、**rc=0**、13/13、W1b **按圈按序 13/13**（最差 0.11 m）、**W6 p50 0.225 / p90 0.448**、
W4 最小净空 **−0.067 m**、W2 max 3.31 m/s）。重出全部图：`bash scripts/render_evidence.sh kv1-02-01`
`python3 scripts/prep_run.py --run <sortie> --out logs/<tag>` 生成。

> 上一批图来自 `kn3-01-01`（修 `yaw` 之前那趟）。换图是因为**图必须来自当前配置的趟**，
> 不是因为 kn3 的数字更好看：kn3 的规划净空是 +0.114/+0.112 m，而这一趟是 **+0.067/+0.021 m** ——
> 规划本身离柱子更近了，这是本轮唯一一处**变差**的量，见下面的说明。

### 这两圈的关键数字（可复算）

| | 第 1 圈 | 第 2 圈 |
|---|---|---|
| 规划最小柱面净空 | **+0.067 m** | **+0.021 m** |
| 真值最小柱面净空 | **−0.045 m（接触）** | **−0.067 m（接触）** |
| \|规划−真值\| p50 / p90 | **0.22 / 0.51 m** | **0.23 / 0.39 m** |

当前配置：`max_vel 1.5 / max_acc 6 / max_jerk 15 / penna_jerk 常开 / CORNER_SPEED 0 / inflation_step 2 / geofence 13`，
飞控 `WP_SPD 1.5 / WP_ACC 4.0 / POS_P 1.2 / VEL_P 2.0`，外部导航噪声 `VISO 0.10/0.25/0.10`。

估计器（`lio-err-t.png`）：水平 p50 **0.027** / p90 0.057 / max 0.103 m，竖直 p50 **0.012** / p90 0.025 m
（时钟拟合 +0.02 s）—— 估计器仍然比规划净空小一个量级，所以**接触不是估计器造成的**。

**这一轮唯一变差的量：规划自己的净空**（+0.112 → +0.021 m）。同一张图、同一套规划参数，变化的是
参考（`WP_SPD 1.5`、`VISO`、逐轴 slew）与外环的落点，于是重规划在角点附近切得更靠柱。
要真正消掉接触，得抬 `rog_map.inflation_step`（第 85 轮用户定为 2 ⇒ 0.10 m 膨胀）或减小
`CORNER_SPEED` 之外再压角点速度——两者都属于 S4/S1 的口述值，**需要用户拍板**。

### 新地图 vs 旧冻结图（"规划差"的量化对照）

| | 旧冻结图 `forest_v2_hex.yaml` | **新图 `forest_hex12.yaml`（本轮实测）** |
|---|---|---|
| 场地 / 柱数 | ±8 m / 250 根（只留航点格） | **±12 m / 522 根**（航点周围 3×3 格留空） |
| 六边形边长 = 角点到 (0,0) | 5.000 m | **10.000 m** |
| 角点自由半径（到最近柱面） | 0.245 m | **1.026 m** |
| 过弯速度上限（45° 倾角 9.81 m/s²） | 1.55 m/s | **3.17 m/s** |
| 过弯速度上限（规划器 `max_acc 8`） | 1.40 m/s | **2.86 m/s** |
| **规划停住比例** | 69.1 % | **11.2 %** |
| **重规划跳变 > 0.3 m** | 27 次（9 次 > 1 m） | **0 次** |
| **横向误差 p50 / p90** | 0.283 / 0.818 m | **0.106 / 0.277 m** |
| 沿航迹 p50 | +0.135 m | **−0.027 m** |

复算：`python3 scripts/tracking_diag.py kh1-02-01`、`python3 scripts/corner_clearance.py worlds/forest_hex12.yaml`。

### 三类图的口径（不变）

用户第 83 轮定的是：只保留 ① **分圈俯视图（路径-轨迹）**、② **vxyz-t**、③ **LIO 误差-t**，其他图都删。
本轮就是这三类 + 地图评审图 + 本 README，没有第四类。

### 旧数据

`deviation_*.tsv` / `rescore_w1b.tsv`（v1 世界 46/55 趟）与所有旧图一起随本轮清理移出仓库，
在 `/mnt/d/distill_self/_archive_super/` 的归档里（每份归档都有 `CLEANUP-manifest.md` 列出删了什么、留了什么）。

### 第 88 轮补 6：四张图换成 `kw1-01-01`（当前交付配置）

交付配置 = `replan_forward_dt 0.1`（原 0.2）+ `POS_P 1.2` + `ARRIVE_M 0.5` + `WP_SPD 1.5`。
出图用 **`kw1-01-01`**（`bash scripts/render_evidence.sh kw1-01-01`）：rtf 0.9801、13/13、
**十二过一败**（只败 W2：max 3.32 m/s 对 4.0 的界，`max_vel 1.5` 下算术上不可能）、
W1b **13/13 按圈按序**（最差 0.10 m）、**W6 p50 0.211 / p90 0.395**、**W4 两圈都无接触**。

| | 第 1 圈 | 第 2 圈 |
|---|---|---|
| 规划最小柱面净空 | **+0.065 m** | **+0.134 m** |
| 真值最小柱面净空 | **+0.027 m（无接触）** | **+0.089 m（无接触）** |
| \|规划−真值\| p50 / p90 | **0.20 / 0.51 m** | **0.20 / 0.55 m** |

**这一段的上文数字（`kv1-02-01` 的 −0.067 m 接触、`W6 0.43`）已被这一批取代**；
为什么换成这一趟、以及"往复"这件事的机理（激励 = 规划腿中减速，放大器 = 0.3–0.8 Hz 的 4 倍增益），
见 `docs/INTEGRATION.md` §77。**注意：同配置的第二趟（`kw1-02-01`，rtf 0.975）没有这么好**
（后退 7.12 m），所以这一批数字是"最好的一趟"，不是"稳定复现"。

### 第 89 轮：四张图换成 `kx1-02-01`（patch 10 之后，13/13 且验收 11 过 2 败）

出图用 **`kx1-02-01`**（`bash scripts/render_evidence.sh kx1-02-01`，**rtf 0.9765 是整窗数，
飞行窗 0.9998**，见 §79）。这一批与上一批的差别只有一个补丁（`replan_time_limit`，§78）：

| | 第 1 圈 | 第 2 圈 |
|---|---|---|
| 规划最小柱面净空 | 0.064 m | 0.134 m |
| 真值最小柱面净空 | **−0.009 m（9 mm 擦到）** | +0.020 m |
| \|规划−真值\| p50 / p90 | 0.19 / 0.43 m | 0.21 / 0.44 m |

* `vxyz-t.png`：规划 `v_xy` p50 1.38 / max 1.50；真值 p50 1.27 / max **3.18** m/s
  （patch 之前同一张图上是"规划 0.5 m/s 级、真值冲到 3 m/s 级"）。
* `lio-err-t.png`：LIO 对真值 **水平 p50 0.031 / p90 0.067 / max 0.102 m，垂直 p50 0.012 / p90 0.017 m**
  —— 估计器这一轮无可指摘，"往复"不是它造成的。
* 验收 **11 过 2 败**（W2 是 SPEC 与 `max_vel 1.5` 的算术冲突；W4 是 9 mm 的擦碰）。


### 第 20 轮定稿：四张图换成 `kx6-03-01-01`（交付 `max_vel` 1.35、14 通过 0 失败）

出图用 **`kx6-03-01-01`**（`bash scripts/render_evidence.sh kx6-03-01-01`）—— 交付速度切到 **1.35 m/s**
之后的一趟，也是本项目**第一次零豁免 14 通过 0 失败（GATE: PASS）**：H1 rtf 0.9998、H6 cast MAX 17.0 ms、
13/13、W1b 13/13（最差 0.18 m）、**W2 p90 1.43**（新判据 ≥ 1.40）、**W4 +0.001 m（无接触）**、
**W6 p50 0.187 / p90 0.321**。

| | 第 1 圈 | 第 2 圈 |
|---|---|---|
| 规划最小柱面净空 | **+0.153 m** | **+0.049 m** |
| 真值最小柱面净空 | **+0.028 m（无接触）** | **+0.016 m（无接触）** |
| \|规划−真值\| p50 / p90 | **0.17 / 0.31 m** | **0.18 / 0.57 m** |

* `vxyz-t.png`：规划 `v_xy` p50 **1.25** / max **1.35**（新上限）；真值 p50 1.19 / max 2.67 m/s ——
  真值不再大幅超调（1.5 档时真值 p50 1.36–1.40、max 3.0–3.4）。
* `lio-err-t.png`：LIO 对真值 **水平 p50 0.039 / p90 0.089 / max 0.130 m，竖直 p50 0.014 / p90 0.040 m**
  （时钟拟合 +0.03 s）—— 估计器仍比净空小一个量级，擦碰不是它造成的。
* 为什么切到 1.35：18 趟六点速度前沿（`docs/INTEGRATION.md` §106/§107/§110/§112）给出
  `W6 p90 ≈ 0.25·v − 0.03`、`W2 p90 ≈ 1.20·v − 0.19`，1.35 是唯一同时满足"W2 过 1.40"与
  "W4 自己转正"的点。六趟合并（变体 3 + 交付态 3）：W2 1.41–1.46 全过、W6 均值 **0.306**（1.5 档 0.352）、
  **W4 3/6 趟无接触**、3/6 趟零豁免全过。
* 以上批次（`kx1-02-01`、`kw1-01-01`、`kv1-02-01`）的数字**已被本批取代**，只留作历史对照。

### 俯视图（2026-11-25）
- `topdown-phase4p.png`：flight `missionA-phase4p-01-01` 的整趟俯视图（含分圈着色）
- `topdown-phase4p-lap1.png` / `topdown-phase4p-lap2.png`：按圈分开的俯视图（分圈边界 = 真值经过 corner 1 附近 1.2 m 内，实测规则）
- `topdown-phase4p-launch.png`：起飞后到第一个航点段
- 生成命令：`python3 scripts/plot_flight_topdown.py --tag phase4p-01-01 [--per-lap]`

---

## 文档政策

- `docs/` 下**只保留 5 篇 Markdown 文档**（README/PIPELINE/ACCEPTANCE/FLIGHT-ANALYSIS/CHANGELOG）。
- `docs/evidence/` 只放证据图与数据（`INDEX.txt` 为图索引）；`docs/metrics/` 的数据并入 `docs/evidence/`。
- 历史文档一律并入上面 5 篇或留在 git 历史，不再新增独立文档；新增内容先并入现有文档。
