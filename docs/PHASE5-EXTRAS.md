# Phase 5（可选）：评估与产物增强 —— 已做部分

**原则不变**：15 行验收判据**一行未改**；本页所有内容都是 **record-only 附加块**（写进 `accept.json` 的
`extras` 并在 RESULT 之前单独打印），任何一项缺失都**不会**让 GATE 变 FAIL —— 因为"缺输入即 FAIL"属于 P0
判据变更，仍等所有者批准。

## 1. manifest 完整性（record-only）

`scripts/accept.py` 现在检查每趟 `manifest.json` 是否记录了 Phase-0 要求的字段：

```
tag / repo.head / world.sha256 / calib.sha256 /
planner.fsm_node_sha256 / planner.config_sha256 / argv /
ardupilot.rev / truth_timebase.offset_us
```

- 全部存在 → `manifest   : present, all required fields recorded`
- 有缺失 → `manifest   : present, MISSING required fields: <列表>`（只报告）
- 文件不存在 → `manifest   : ABSENT (reproducibility hole -- record-only, not a gate row)`
- 老 sortie（2026-11-25 之前）没有 `truth_timebase` → 额外标注 `legacy_timebase`。

数据也在 `accept.json` → `extras.manifest = {present, path, missing[], legacy_timebase}`。

## 2. 轨迹指标 APE/RPE（附加行）

`scripts/traj_error.py`（无 evo 依赖，numpy 实现；语义与 evo 的 APE/RPE 相同）：

```bash
python3 scripts/traj_error.py --run .runs/missionA-<tag> [--json out.json] [--no-align]
```

- 真值：`tap.csv` 的 `t_sim_us`（显式仿真时钟；老文件回退 `t_us`，与 accept 一致）；
- 两条被评轨迹：`lio`（`lio.csv`，L→W：E=−y_l, N=x_l, U=z_l）与 `fcu`（`setpoints.csv` 的 `fcu_*`，已在 W）；
- 窗口：`phase.csv` 的 takeoff..land；
- **APE**：在飞行窗口上做一次 **SE(3) Umeyama 对齐（无尺度）** 后逐点位置误差（rmse/p50/p95/max），
  同时打印对齐得到的 yaw 与平移 —— 把标定误差与估计器误差分开看；
- **RPE(Δ=1 s)**：同一对齐下的 1 s 相对运动误差（rmse/p95），去掉全局摆放的影响；
- `--no-align` 给原始误差（会把 `calib/calib.json` 的摆放差也算进去，用于诊断）。

`accept.py` 在 RESULT 之前打印、并写入 `extras.trajectory`。实测（`missionA-phase4p-01-01`）：

```
lio  APE  rmse 0.027 m  p50 0.022  p95 0.045  max 0.070   (n=31963)   align yaw -0.32 deg, t=[-0.018 -0.001 +0.101]
lio  RPE 1s rmse 0.015 m  p95 0.029
fcu  APE  rmse 0.052 m  p50 0.043  p95 0.089  max 0.154   (n=6624)    align yaw -0.53 deg, t=[-0.018 +0.005 -0.122]
fcu  RPE 1s rmse 0.049 m  p95 0.092
```

（LIO 的 z 平移 +0.10 m 与 FCU 的 −0.12 m 正是已知的 z 偏置方向，与 `scripts/altitude_split.py` 的结论一致。）

## 3. rviz 回放 headless 冒烟（Phase 4 验证收尾）

`tests/check_rviz_replay.sh [sortie]`（**按需执行，不在默认 gate**——整轮 gate 中 ROS 图探测偶发失败，
而回放本身健康，实测 2026-11-25；无 bag 时 SKIP）：用 **`--no-rviz`** 跑
`scripts/replay_bag_rviz.sh`，断言 RViz 真正需要的东西都在：

1. 录制的 4 个话题在播放（`/sim/truth/odom /lio/robo/odom /plan/cmd /fcu/odom`）；
2. `viz_bridge` 发布的 `/viz/truth_path /viz/lio_path /viz/plan_path /viz/plan_pose` 存在且有数据
   （truth ≥20 Hz、plan ≥10 Hz；实测 truth ~100–120 Hz、plan ~35 Hz）；
3. `/tf_static` 有 `world_w`（父）→ `lio_world`（子）变换（从 `calib/calib.json` 读 R_WL/t_WL）。

顺带修掉两个真实 bug：静态 TF 的 printf 参数个数错位（TF 从未发布成功）、`--no-rviz` 下空 TF 行触发
`set -u` 崩溃；TF 检查改为对 latched 话题重试 3 次（QoS 时序）。

## 4. bag→derive→live 一致性 CI（2026-11-25 第 21 轮，已做）

`tests/check_bag_pipeline.py`（入门禁）现在把三件事放进一次 gate 运行：

1. 发已知 `/lio/robo/odom` 与 `/plan/cmd`（30 Hz，避免两个订阅者丢不同的包）；
2. **同时**用 bag 记录（`scripts/record_bag.sh`，split writers）和飞行实际使用的实时 CSV 记录器
   （`scripts/record-odom.py` → `lio.csv`；`scripts/record-traj.py --csv traj.csv` → `traj.csv.cmd.csv`）；
3. 用 `bag_to_csv.py` 派生 bag，再逐样本对比 **派生层 vs 实时层**（**报告为 info，不是 gate 判据**：
   两个独立订阅者在负载下会丢不同子集，实测重叠 47–100% 但**匹配上的样本值全部一致**；gate 判据是
   "派生不改变数值"——与发布出去的已知值对上；"派生层逐样本等于实时层"的权威证据是真实飞行
   `missionA-phase2k`（现采每一行逐字节存在于派生层）：
   - 派生层的每一行必须能在实时层里找到**同一时间戳**、且所有数值字段一致（<1e-6）；
   - 要求 ≥95% 的派生行匹配、0 个数值不一致。
   实测：信息行形如 `info same/differs (subscriber loss): ... matched N%, value mismatches 0`；
   数值不一致 = 0 是硬要求（派生逻辑回归时会立刻红）。

这正是 Phase 2 在真实飞行上验证过的"派生层 ⊇ 现采层"的**可重复版本**，不需要飞行；真实飞行的证据仍是
`missionA-phase2k`（逐字节包含）与 Phase 2 文档。

## 5. P0 判据改造方案（已写；v2 已实现为 opt-in，默认仍是 v1）

`docs/P0-CRITERIA-PROPOSAL.md`：用 **80 趟 accept.json** 的实测分布提出 W6（稳态 lag ≤0.35 s + 去线性漂移后残差
≤0.12 s + 保留 p90 ≤0.60 m；单季度异常记为 WARN）、H2（速率 == 配置 publish-hz ±5%）、W4（CLEAR>0 不可行：
min −0.096 / 中位 −0.052；建议先按已有 `--known-defect W4` 接受，容差版作为下一阶段目标）、H6（skip 上限改
max(20, 1%×扫描数)）、H3/W1b 微调。**15 行默认阈值一字未改**。`accept.py --criteria-v2` 已实现（默认关，`tests/check_criteria_v2.py` 钉住
"默认 = v1 存档一致、v2 只允许 H2/W4/W6/H6 翻转"），并用 `scripts/criteria_ab.py` 对 **104 趟**存量 sortie
做了 v1/v2 回放（`docs/metrics/p0-criteria-ab.txt`）：H2 4 趟、W4 20 趟按预期转绿，W6/H6 **0 趟翻转**
（W6 的单季度真实滞后在 v2 下仍然 FAIL）。切默认仍需所有者批准。
