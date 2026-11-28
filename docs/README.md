# super_super_apm — 密集森林里的自主巡线（SUPER + Super-LIO + ArduPilot SITL）

本仓库是"无人机在 1 m 柱距的密集森林里沿六边形航点自主飞行"的**单仓全套**：
SITL 飞机、雷达/IMU 仿真、Super-LIO 定位、SUPER 轨迹规划、plan box（目标/参考整形）、
MAVLink 桥 + supervisor、真值采集、验收与出图工具，全部在本仓库内，可直接复现每一趟飞行。

## 当前状态（2026-11-28，三路独立审查后校正）

> 2026-11-28 的独立审查发现两处度量错误并已修复：W4 原先是每 5 个真值点才检查一次（会把擦碰判成 CLEAR），
> S1 的 END 窗口原先从"末角点首次到达"开始（把第二圈排除）。下面的数字是**修复后重算**的口径。

| 项 | 状态 |
|---|---|
| **当前最佳（可飞行基线）** | `super_planner_hex_hs_cache.yaml`（handover=defer）+ `NAV_LEAD_S=0.06` + `PASS_BY_M=0.6/PASS_BY_LONG_M=1.0` + `REF_WASHOUT_TAU=2.0/VMAX=0.15` + `REF_NO_RETRACT=1` + `REF_ALONG_FLOOR=0.9` + `GOAL_PVA=1` + `CORNER_MODE=bisector` + `CORNER_SPEED=0.9` + `ARRIVE_M=0.3` + `PRE_GOAL_M=0.8` + z 0.118/2.0/0.005/1.2；`EXTRA_PARMS=guided_limits_bench5_vert_agile.parm` |
| 代表趟 177b / 177c | 15 行 **14/15 + 14/15**（W4：177b **CONTACT −0.000 m**、177c CLEAR +0.006 m）；W1b 13/13；Q3 0.157/0.154；W6 0.231/0.241；P1 PASS（max 0.665/0.721，0 个 >1 m）；Q1 1/2 段；S1 **3.45/5.54 s**；S2 **6/6 环（7.9/7.2 s，已去 padding）**；R2R **16/21 次 rest2rest**；Q2 **FAIL −0.100/−0.094**；Q4 仍有个别角点 vmin<0.4 |
| 已知未过（1.35 "完美"还差） | **W4 点净空**（177b 已接触）、**Q2 机身净空**（−0.10 m，两趟都 FAIL）、Q1 1–2 段、Q4 角点、S1 1–6 s、S2 6 环、每角点 rest2rest（R2R 1.3–2.1 次/角点）、末目标偶发参考环 |
| 机制现状 | 换目标用 DEFER（等旧轨迹飞完再 rest2rest）；已实现 R1a（提前推进）但两路独立审查判定 **NO-GO**（A* 会绕过当前角点 → pacer 卡死），需先修"旧轨迹整段作 guide / via points"；另发现 `backup_traj_en` 是死参数、committed 会被备份刹车段截断 → 腿中段 rest2rest |
| 机体 / 地图 | `sim/models/quad_1kg_250mm.json`（1.0 kg / 250 mm）；冻结 `forest_hex12.yaml`（半径 10、522 柱、1 m 柱距）+ seed s22/s23/s24；大世界 hex12w 已接入（6000 点/帧、geofence 16、planner map ±18 m） |
| 速度 | 1.35 m/s 可用；2.0/2.5 在 1.35 基线上跟踪不合格（§104–§106）；3 m/s 的 PVA 路线待 R1a/规划层修完再试 |
| 下一步 | ① 修评测/门禁与文档（已做 W4/S1/S2/R2R，进行中）；② 修 backup 截断 + 半途换段（插桩）；③ R1a 改为 keep-old-guide/via 版本 + 离线 A* 门禁；④ 修 P0 竞态与 PVA 终端语义；然后才飞 |

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
| `docs/PIPELINE.md` | 完整链路（进程/话题/坐标系/控制/估计/规划/数据管线/约定与坑）+ **Nav2 对照调研附录** |
| `docs/ACCEPTANCE.md` | 15 行验收集（H*/W*）+ 质量行（Q*/P1/S1/S2/R2R）+ 判定标准 + 证据政策 |
| `docs/FLIGHT-ANALYSIS.md` | 飞行分析：当前最佳、管线剖析（§113）、归因（§120/§123）、速度阶梯、实验历史、下一步 |

> 历史进度日志已合并进 `docs/FLIGHT-ANALYSIS.md` 的附录，独立的 CHANGELOG 已删除。

## 证据（docs/evidence/）

- `final/sheet-BEST-177b.png`：当前最佳 177b 全套图（注意 W4 校正后为 CONTACT −0.000 m）
- `final/phase177b-01-01_*.png`、`planexec-177b.png`、`handover-audit-177b.png`、`low-speed-177b.png`：177b 单图
- `final/sheet-BEST-151a/150s22b/152s23b/153s24c.png`：四图（frozen/s22/s23/s24）证据
- `final/pipeline-target.png`、`bench-1kg-8ms.png`、`bench-pva-8ms.png`：架构图与机体台架
- `INDEX.txt`：全部保留证据与 21 个保留 run 的索引

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
| `scripts/accept.py` | 15 行验收（H2 已按配置 200 Hz 修订） |
| `scripts/quality_rows.py` | 新质量行 Q1–Q4（中段低速/机身净空/中段跟踪/角点动力学） |
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
