# super_super_apm — SUPER + Super-LIO 的六边形林区任务（ArduPilot SITL）

> **Abstract.** Two laps of a 10 m-radius hexagon through a 522-pillar forest, planned by
> [hku-mars/SUPER](https://github.com/hku-mars/SUPER), tracked by
> [Super-LIO](https://github.com/Liansheng-Wang/Super-LIO) as the only state source, flown by ArduPilot
> SITL and graded offline from the recordings by a fixed acceptance table. The value of this repo is the
> **measurement chain**, not the demo: every number in `docs/` can be regenerated from a recording with
> one command, and every configuration change was adopted only after a measured A/B — the rejected ideas
> are recorded with their numbers too (`docs/INTEGRATION.md` §99–§112).

任务：**10 m 半径正六边形、2 圈、13 个目标**，在 ±12 m 场地里穿过 **522 根柱**（柱距 1.0 m、柱截面
0.2 m）；规划器是 **SUPER**（原版编译，本仓库内置已打补丁的源码），状态源只有 **Super-LIO**（飞控
`EK3_SRC1_* = ExternalNav`），执行是 **ArduPilot SITL**，评分由 `scripts/accept.py` 从记录离线完成。

---

## 1. 现在跑到什么程度

最好的一趟（`kx6-03-01-01`，巡航上限 1.35 m/s）：**14 行全过、0 失败**。

| 行 | 含义 | 实测 |
|---|---|---|
| H1 | 主机保持实时 | 飞行窗 rtf 0.9998 |
| H6 | 雷达仿真没饿死 | raycast MAX 17.0 ms（闸门 100）、0 次跳过 |
| W1 / W1b | 13/13 到达、按序接近 | W1 13/13，W1b 13/13（最差 0.18 m） |
| W2 | 巡航速度 | p90 **1.43 m/s**（判据 ≥ 1.40），摸到 1.35 上限 |
| W3 | 高度 | 偏差 −0.054 m，无越带 |
| W4 | **不碰柱** | **+0.001 m（无接触）** |
| W6 | 计划-真值跟踪 | p50 0.187 / **p90 0.321 m** |
| — | 估计器（LIO 对真值） | 水平 p50 0.039 / p90 0.089 m |

同一巡航档共飞 6 趟：**W2 全部通过**（1.41–1.46）；W6 p90 均值 **0.31 m**；W4 有 3 趟干净（另外 3 趟
接触 −0.007…−0.072 m，作为**显式已知缺陷 `W4`** 记账，见 `docs/INTEGRATION.md` §99、§112）。

图：`docs/evidence/`（分圈俯视图、vxyz-时间、估计器误差），一条命令重出：
`bash scripts/render_evidence.sh kx6-03-01-01`。

## 2. 目录

```
setup/            env.sh（唯一知道路径的地方）、build.sh、fetch_ardupilot.sh、run_hex.sh
scripts/          起飞手臂、评分器 accept.py、以及 ~150 个分析工具
src/whtf/ros/     薄层：把 SUPER 接到飞控（planner_source_super.py、ref_lp.py）
config/           SUPER 规划器配置（冻结版 + §90 记录的速度变体）
params/           飞行用的 ArduPilot 参数（外部导航信任、guided 限幅）
worlds/           冻结世界 forest_hex12.yaml（522 柱、±12 m、边长 10.000 m）
calib/            L→W 标定（calib.json，实测量）
docs/             SPEC.md（需求）、INTEGRATION.md（全部测量记录）、CHAIN_DETAIL.md、evidence/
harness/          围绕仿真的运行框架：common/planner/bridge/supervisor/sim/tools + sim/params/rviz/frozen
third_party/      pins.env + 内置的 SUPER / Super-LIO / livox_ros_driver2 源码
```

## 3. 依赖

* Ubuntu 22.04（或 WSL2）、ROS 2 Humble、C++17、Eigen、PCL、yaml-cpp（apt 一行见 `third_party/pins.env`）。
* 磁盘 ≥ 8 GB（ArduPilot 占大头）、≥ 4 核。**主机要安静**：验收 H1 量的是"仿真有没有跑满实时"，
  机器一忙就会失败——这是设计，不是 bug。

## 4. 编译

```bash
git clone https://github.com/wangfanxin/super_super_apm.git
cd super_super_apm

bash setup/fetch_ardupilot.sh --build   # ArduPilot SITL，固定 commit（~1.5 GB，最久的一步）
bash setup/build.sh                     # colcon：super_planner（fsm_node）+ super_lio + livox
```

`setup/build.sh` 会建 `.build/ws_super`、`.build/lio_ws`，把内置包软链进去并 Release 编译；打印每件的
落点，`fsm_node` 没产出就直接失败。所有路径都能用环境变量覆盖（`ARDUPILOT_ROOT`、`LIO_LOGS`、
`WS_SUPER`、`LIO_WS`），见 `setup/env.sh`——仓库里没有任何硬编码的 clone 路径。

## 5. 运行与评分

```bash
bash setup/run_hex.sh kx7 2 2      # TAG、趟数、每趟最多重试次数
```

依次做：静态闸门（冻结地图 + SPEC 自检 + 补丁在位）→ 飞 2 趟 → 对每趟跑 `accept.py`。
记录落在 `.runs/missionA-<tag>/`（`tap.csv` 真值、`setpoints.csv` 发布的参考、`traj.log` 规划器日志）。
单独重评分：

```bash
KNOWN=W4 bash scripts/run_accept.sh <tag> forest_hex12.yaml 2.0 10.0 box 0.5 2
```

`KNOWN=W4` 把那一行标成"已接受的已知缺陷"（数字照打、JSON 里仍 `ok:false`）；不写就是硬 FAIL。

## 6. 刻意不放进仓库的东西

| 东西 | 大小 | 原因 | 怎么得到 |
|---|---|---|---|
| ArduPilot SITL | ~1.5 GB | 太大，且是原版 checkout | `setup/fetch_ardupilot.sh`（固定 commit） |
| `super_planner/log/` | 1.3 GB | 内置规划器的运行日志 | 每次运行自动生成 |
| 飞行记录、`eeprom.bin`、`*.BIN` | — | 每趟的产物 | `.runs/`（已 gitignore） |
| 作者机器的临时树（`_stage`、`_refs` 等） | GB | 不是源码 | — |

**一句必须说清的话**：记录里那些数字来自带 truth-tap 补丁的 SITL —— `tap.csv`（1.2 kHz 真值）就是它
写的。用原版 ArduPilot 能飞完任务，但不会有 `tap.csv`，`accept.py` 也就无法评分。
`third_party/pins.env` 里写了同一句话。

## 7. 第三方代码与许可

* `third_party/SUPER` —— hku-mars/SUPER（保留 LGPL-3.0 头）。**已修改**：两个补丁直接烘进内置源码，
  随时可用 `scripts/patch_super_*.py --check/--apply/--revert` 复算；静态闸门会断言它们在位。
* `third_party/Super-LIO` —— Liansheng-Wang/Super-LIO @ `f89f48d`。
* `third_party/livox_ros_driver2` —— Livox-SDK/livox_ros_driver2 @ `2144554`。
* `harness/` —— 本项目自己的运行框架（bridge / supervisor / 仿真 tap）。

完整 commit 见 `third_party/pins.env`。

## 8. 继续读

* `docs/SPEC.md` —— 需求与每一次修改记录（谁提的、为什么）。
* `docs/INTEGRATION.md` —— 9000 行、112 节的测量记录：试过什么、数字多少、为什么否掉。
* `docs/evidence/README.md` —— 每张图是什么、怎么重画。
* `docs/LEGACY_PATHS.md` —— 哪些分析工具仍假设作者的绝对路径（编译与飞行链不假设）。
