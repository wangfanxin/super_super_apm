# super_super_apm — 密集森林自主巡线（SUPER + Super-LIO + ArduPilot SITL）

> **Abstract.** A 10 m-radius hexagonal tour through a 522-pillar forest (1.0 m pillar pitch, 0.2 m
> square pillars), planned by [hku-mars/SUPER], tracked by Super-LIO as the only state source, flown by
> ArduPilot SITL and graded offline from the recordings by a fixed 15-row acceptance table. The value of
> this repo is the **measurement chain**: every number in `docs/` can be regenerated from a recording,
> and every configuration change was adopted only after a measured A/B — rejected ideas are recorded with
> their numbers too.

**入口文档：[`docs/README.md`](docs/README.md)**（当前状态、最佳配置、架构、快速开始、证据索引）。

## 当前状态（2026-11-26）

| 项 | 状态 |
|---|---|
| 历史最佳实飞 | **phase9f-01-01**（1.35 m/s）：除已知 P0（H2）外验收全过；W1/W1b 13/13、**W4 +0.027 m CLEAR**、W6 p90 0.262、中段指令停顿 0 |
| 新基准（9f+，待飞确认） | 9f 的几何/角点设置 + `REF_Z_OFFSET=0.118` / `REF_Z_HOLD`（指令 z 恒定 1 cm） + 垂直环软化 `PSC_D_VEL_P=3.0` |
| 机体 | `sim/models/quad_1kg_250mm.json`（1.0 kg / 250 mm）；能力台架 8.34 m/s、PVA 8 m/s 跟踪 p90 0.24 m |
| 地图 | 冻结 SPEC `sim/worlds/forest_hex12.yaml`（半径 10 m 六边形、10 m 边、3 圈、522 柱、柱距 1.0 m） |
| 速度阶梯 | 1.35 m/s 可用；2.5/4 m/s 在大世界上有正项未达标；6/8/10 m/s 路径未达成（8 m/s 仅台架） |

## 文档（`docs/` 4 篇）

| 文档 | 内容 |
|---|---|
| [`docs/README.md`](docs/README.md) | 入口：当前状态、9f+ 配置、架构、快速开始、证据索引 |
| [`docs/PIPELINE.md`](docs/PIPELINE.md) | 完整链路：进程/话题/坐标系/控制/估计/规划/数据管线/运行约定与坑 |
| [`docs/ACCEPTANCE.md`](docs/ACCEPTANCE.md) | 15 行验收（H1–H6、W0–W6）判据、政策、已知 P0 |
| [`docs/FLIGHT-ANALYSIS.md`](docs/FLIGHT-ANALYSIS.md) | 飞行分析：9f+、9f 实飞、9f vs 21a、定位/高度分解、角点问题、速度阶梯、实验历史与附录 |

> 文档已合并：旧的 CHANGELOG / INTEGRATION / SPEC / PHASE* / PIPELINE-PLAN 等已并入上述 4 篇
> （独立 CHANGELOG 已删除，历史进度日志在 `docs/FLIGHT-ANALYSIS.md` 附录，git 历史仍保留原文）。

## 快速开始（WSL + ROS2 humble）

```bash
. setup/env.sh                     # 环境
bash tests/run_all.sh              # 全量门禁（提交前必须 PASS）
bash scripts/clean_harness.sh      # 清残留进程 + DDS 共享内存 + 孤儿 fsm_node

# 飞一趟（9f 几何 + 9f+ 配置，冻结地图，1.35 m/s）
SITL_MODEL="x:sim/models/quad_1kg_250mm.json" \
EXTRA_PARMS=$PWD/fcu_interface/params/guided_limits_bench5_vert.parm \
GOAL_PVA=1 CORNER_MODE=bisector CORNER_SPEED=0.8 ARRIVE_M=0.3 \
REF_V_SLEW=3.0 REF_Z_OFFSET_M=0.118 REF_Z_HOLD=2.0 REF_Z_TOL=0.005 REF_Z_LATCH_M=1.2 \
NAV_GATE_MPS=15 NAV_SKIP_MAX=40 BAG=1 BAG_SPLIT=1 SORTIES=1 \
bash scripts/fly_hex2.sh demo01 1

# 验收 + 分析（每趟必做）
python3 scripts/accept.py --run .runs/missionA-demo01-01-01 \
    --world sim/worlds/forest_hex12.yaml --alt 2.0
python3 scripts/flight_forensics.py --tag demo01-01-01
python3 scripts/low_speed_census.py --tag demo01-01-01
```

## 目录

| 目录 | 内容 |
|---|---|
| `bridge_nodes/` | plan box（`thin/planner_source_super.py`）、MAVLink bridge+supervisor、参考整形、公共框架 |
| `hku_super/` | SUPER 规划器源码、配置（`config/`、运行时 `super_planner/config/`）、补丁（`patches/`） |
| `slam/super_lio/` | Super-LIO（LiDAR-inertial 里程计） |
| `sim/` | SITL 真值客户端、雷达/IMU 仿真与射线器、机体模型（`models/`）、世界（`worlds/`） |
| `fcu_interface/` | 飞控参数（`params/`）、消息定义（`msgs/`） |
| `scripts/` | 飞行臂/验收/出图/法医/census/清理工具 |
| `tests/` | 全量门禁与单元测试 |
| `docs/` | 4 篇文档 + `docs/evidence/` 证据（只保留最佳轮次/当前轮次/能力台架） |
| `calib/` | L→W 标定 `calib.json` |
| `setup/` | 环境脚本（`env.sh`、ROS2/SITL 工作区） |

## 硬性约定

1. **只飞 SITL，不飞真机**；所有结果在仓库内可复现。
2. **不 push**（本地 commit 即可）。
3. `docs/` 文档 ≤5 篇（当前 4 篇）。
4. 每趟飞行后：完整图集 + `accept.py` + `flight_forensics.py` + `low_speed_census.py` + 更新文档。
5. 质量不看"到点"：W4 净空必须为正、W6 跟踪、低速段普查（尤其**路径中间** stall/loop）都是门。
6. 主机雷达仿真会间歇 starvation（H6）：先 `clean_harness.sh`，只采纳 H6-clean 的趟。
