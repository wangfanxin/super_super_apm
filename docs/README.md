# super_super_apm — 密集森林里的自主巡线（SUPER + Super-LIO + ArduPilot SITL）

本仓库是"无人机在 1 m 柱距的密集森林里沿六边形航点自主飞行"的**单仓全套**：
SITL 飞机、雷达/IMU 仿真、Super-LIO 定位、SUPER 轨迹规划、plan box（目标/参考整形）、
MAVLink 桥 + supervisor、真值采集、验收与出图工具，全部在本仓库内，可直接复现每一趟飞行。

## 当前状态（2026-11-26）

| 项 | 状态 |
|---|---|
| **当前最佳（9f+，已确认）** | **phase23c-01-01**：除已知 P0（H2）外 **15/15 行全过**；W1/W1b 13/13、W4 **+0.009 m CLEAR**、W3 bias **+0.003**、W6 p90 0.280、W2 p90 1.44 |
| 历史最佳实飞（旧） | **phase9f-01-01**：W4 **+0.027 m CLEAR**、W6 p90 0.262、W3 bias -0.113（高度基准未修） |
| 机体 | `sim/models/quad_1kg_250mm.json`（1.0 kg / 250 mm）；台架实测 8.34 m/s、PVA 8 m/s 跟踪 p90 0.24 m |
| 地图 | 冻结 SPEC 地图 `sim/worlds/forest_hex12.yaml`（六边形半径 10 m、边长 10 m、3 圈、522 柱、柱距 1.0 m） |
| 速度阶梯 | 1.35 m/s 已可用；2.5/4 m/s 在**大世界**上有正项但未达标；8 m/s 仅在机体能力台架达成 |

### 9f+（新最佳配置）

9f 的飞行器/几何/角点设置：

- 机体 1 kg、固件 `fcu_interface/params/guided_limits_bench5_vert.parm`（8 m/s 竞速档 + 垂直环软化）
- 规划档 `super_planner_hex.yaml`（max_vel 1.35 / acc 6 / jerk 15 / omg 2.5 / map_size 24 / inflation_step 2）
- patch 14（换目标走连续重规划）、patch 15（PVA 尾部延长，默认关）
- plan box：`GOAL_PVA=1 CORNER_MODE=bisector CORNER_SPEED=0.8 ARRIVE_M=0.3 REF_V_SLEW=3.0`，
  **无前视 / 无 carrot / 无 pass-by 强化 / 无角点延伸**

21a 的配置修复：

- `REF_Z_OFFSET_M=0.118`（补偿 EKF 高度基准 0.12 m，使真值高度 = 任务高度）
- `REF_Z_HOLD=2.0 REF_Z_TOL=0.005 REF_Z_LATCH_M=1.2`（把指令 z 锁成常数，范围 1 cm）
- 固件垂直环 `PSC_D_VEL_P=3.0`（高度残差 std 4.4→3.3 cm，垂直速度振荡 7.5→3.4 cm/s）

复现命令（新最佳配置，飞行日直接用）：

```bash
SITL_MODEL="x:sim/models/quad_1kg_250mm.json" \
EXTRA_PARMS=$PWD/fcu_interface/params/guided_limits_bench5_vert.parm \
GOAL_PVA=1 CORNER_MODE=bisector CORNER_SPEED=0.8 ARRIVE_M=0.3 \
REF_V_SLEW=3.0 REF_Z_OFFSET_M=0.118 REF_Z_HOLD=2.0 REF_Z_TOL=0.005 REF_Z_LATCH_M=1.2 \
NAV_GATE_MPS=15 NAV_SKIP_MAX=40 BAG=1 BAG_SPLIT=1 SORTIES=1 \
bash scripts/fly_hex2.sh <tag> 1
```

## 架构一览

```
SITL (ArduCopter 4.7.1, GUIDED) ──MAVLink serial0/serial1──
        │                                   │
        │  truth tap (SIM_STATE 108)        │  fcu_client（单进程=bridge+supervisor）
        ▼                                   ▼   · ODOMETRY #331（48 Hz）：Super-LIO → EKF
 sim/src/sim_state_recv.py                   · 消息 84（50 Hz）：PVA setpoint → FCU
        │  /sim/state 200 Hz                · supervisor 门：geofence / 高度 / 规划静默 / nav 频率
        ▼                                   ▲
 sim/src/lidar_imu.py  ──/livox/lidar──► Super-LIO ──/lio/robo/odom──► bridge
        （10 Hz 点云 + 200 Hz IMU）              │
                                                ▼
                                    plan box（thin/planner_source_super.py）
                                      · 目标/航点状态机（pacer）
                                      · 参考整形（slew / z 基准 / z hold）
                                      · 起停 super_planner fsm_node
                                                │  /plan/cmd（100 Hz PVA，L 系）
                                                ▼
                                          bridge → FCU
```

## 快速开始

```bash
# 0) 环境（WSL）：ROS2 humble + SITL 构建
. setup/env.sh

# 1) 完整门禁（提交前必须 PASS）
bash tests/run_all.sh

# 2) 清理残留（DDS 共享内存 + fsm_node 硬杀）
bash scripts/clean_harness.sh

# 3) 飞一趟（示例：冻结地图、1.35 m/s、9f 几何 + 9f+ 配置）
bash scripts/fly_hex2.sh demo01 1        # 参数同上"复现命令"

# 4) 出全套图 + 分析
python3 scripts/plot_flight_topdown.py --tag demo01-01-01 --per-lap
python3 scripts/plot_flight_timeseries.py --tag demo01-01-01 --events /tmp/loops.json
python3 scripts/plot_corner_details.py --tag demo01-01-01
python3 scripts/flight_forensics.py --tag demo01-01-01       # 跟踪/定位/高度/角点法医
python3 scripts/low_speed_census.py --tag demo01-01-01       # 先找所有 <0.5 m/s 的段，再逐段分类
python3 scripts/accept.py --run .runs/missionA-demo01-01-01 --world sim/worlds/forest_hex12.yaml --alt 2.0
```

## 文档（4 篇）

| 文档 | 内容 |
|---|---|
| `docs/README.md`（本文） | 入口、当前状态、架构、快速开始、证据索引 |
| `docs/PIPELINE.md` | 完整链路（进程/话题/坐标系/控制/估计/规划/数据管线/约定与坑） |
| `docs/ACCEPTANCE.md` | 15 行验收集（H*/W*）+ 判定标准 + 证据政策 |
| `docs/FLIGHT-ANALYSIS.md` | 飞行分析：当前最佳（9f+）、9f 实飞结果、9f vs 21a、定位/高度分解、角点问题、速度阶梯、实验历史、下一步 |

> 历史进度日志已合并进 `docs/FLIGHT-ANALYSIS.md` 的附录，独立的 CHANGELOG 已删除。

## 证据（docs/evidence/）

只保留**历史最佳轮次**（phase9f：1.35 m/s 最好实飞）、**当前轮次**（phase21a：新机制实验）
和机体能力台架图；其余历史趟的图已按 owner 要求清理。

- `phase23c-*`：当前最佳（9f+ 确认）——topdown 整趟/分圈、xyz-t、速度 2-范数、LIO/估计误差、
  误差分解、高度、六角点法医、低速段 census
- `phase9f-*`：历史最佳（旧）——同一套图
- `bench-1kg-8ms.png` / `bench-pva-8ms.png`：1 kg 机体 8 m/s 能力台架
- 验收脚本要求的固定证据：`INDEX.txt`、`world-hex12.png`、`lap1-paths.png`、`lap2-paths.png`、
  `vxyz-t.png`、`lio-err-t.png`

## 硬性约定

1. **只飞 SITL，不飞真机**；所有结果都在本仓库内可复现（地图、参数、脚本、证据）。
2. **不 push**：本地 commit 即可。
3. **文档不超过 5 篇**（当前 4 篇）。
4. 每趟飞行后：完整图集 + `flight_forensics.py` + `low_speed_census.py` + `accept.py` + 更新文档。
5. 判断质量不能只看"到点"：必须看 W4 净空、W6 跟踪、低速段普查（尤其是**路径中间**的 stall/loop）。
6. 主机（WSL）雷达仿真会间歇 starvation（H6），同一配置时好时坏；先跑 `clean_harness.sh` 再飞。

## 主要脚本索引

| 脚本 | 用途 |
|---|---|
| `scripts/run-mission.sh` / `scripts/fly_hex2.sh` | 启动一次 SITL 飞行（后者带冻结检查与 n 趟循环） |
| `scripts/run_until_healthy.sh` | 带健康门的多 attempt 启动（bench/pvabench/super 分支） |
| `scripts/accept.py` | 15 行验收 |
| `scripts/flight_quality.py` | 绕圈/中段停顿/参考连续性/跟踪门禁 |
| `scripts/flight_forensics.py` | 跟踪 = 执行 + 定位分解、LIO/FCU vs 真值、角点停/回摆、高度基准 |
| `scripts/low_speed_census.py` | 先找所有 <0.5 m/s 段，再逐段分类（owner 的分析规则） |
| `scripts/plot_flight_topdown.py` / `plot_flight_timeseries.py` / `plot_corner_details.py` | 全套图 |
| `scripts/clean_harness.sh` | 清残留进程 + DDS 共享内存 + 死锁/孤儿 fsm_node |
| `scripts/check_frozen.py` | 冻结地图/航点校验（fly_hex2 起飞前强制） |
| `tests/run_all.sh` | 全量门禁 |

## 环境注意

- SITL 自定义机体 JSON 只接受**相对路径**；`run-mission.sh` 会把模型拷进 run 目录再加载。
- WSL 长时间连续跑会偶发 OOM/DDS 抖动 → 雷达 skipped 上升 → LIO 变差 → abort；`clean_harness.sh` 会清
  `ros2 daemon` 与 `/dev/shm/fastrtps_*`。
- `fsm_node` 捕获 SIGTERM 后可能不退出，`clean_harness.sh` 对它会二次 `kill -9`。
- `.runs` 是指向仓库外运行根 `sp_runs` 的软链接（目前只保留 phase9f / phase21a 两趟记录）。
