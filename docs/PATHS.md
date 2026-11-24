# 路径约定（仓库内 / 运行根 / 历史路径）

## 1. 三层布局

| 层 | 位置 | 进 git？ | 谁决定 |
|---|---|---|---|
| **仓库**（源码/配置/世界/参数/标定/文档） | 本目录（clone 到哪都行） | 是 | `setup/env.sh` 从脚本位置推出 `DSH_ROOT` |
| **运行根**（编译产物、飞行记录） | `.build/`、`.runs/`（默认在仓库内；本机在 WSL 的 `/home/fanxin/super_super_apm`） | 否 | `setup/runtime.env`（不入库）或环境变量 |
| **第三方** | 仓库内 `third_party/`；ArduPilot 在 `.build/ardupilot`（`setup/fetch_ardupilot.sh`） | SUPER/Super-LIO/livox 是；ArduPilot 否 | `third_party/pins.env` |

`setup/env.sh` 是**唯一**的路径权威：`DSH_ROOT` / `HARNESS_ROOT` / `SUPER_PLANNER_ROOT` /
`WS_SUPER` / `LIO_WS` / `ARDUPILOT_ROOT` / `LIO_LOGS` / `CALIB_FILE` / `MID360_PATTERN`。
每个入口脚本都会自己向上找 `setup/env.sh` 并 source 它（不需要先 export 任何东西）。

## 2. 历史路径（读旧记录时对照）

`docs/INTEGRATION.md` 与 `docs/evidence/` 是**历史记录**，逐字保留，里面出现的下列路径是迁移前的布局：

| 记录里的路径 | 现在的等价物 |
|---|---|
| `/home/fanxin/lio_avoid/logs/missionA-<tag>` | `<LIO_LOGS>/missionA-<tag>`（本机 `/home/fanxin/super_super_apm/.runs/`） |
| `/home/fanxin/lio_avoid/src/...` | `harness/src/...`（仿真/桥/监督器）或 `.build/ardupilot`（SITL） |
| `/home/fanxin/lio_avoid/lio_ws` | `.build/lio_ws`（本机 `/home/fanxin/super_super_apm/.build/lio_ws`） |
| `/home/fanxin/super_super_apm/upstream/SUPER` | `third_party/SUPER`（本机的构建仍用 WSL 里那份上游副本） |
| `/mnt/d/distill_self/lio_avoid` | `harness/` |
| `/mnt/d/distill_self/_stage/calib.json` | `calib/calib.json` |
| `/mnt/d/distill_self/release` | 已并入本仓库（`release` 是发布前的暂存名） |

旧的 `lio_avoid` / `lio_nav_apm` / `gazebo_apm` / `_stage` 大头已在 2026-11-24 清理；
`/home/fanxin/lio_avoid` 现在只是一个**符号链接兼容壳**（4 个链接），让历史分析工具的默认路径仍能解析。

## 3. 规则

1. 新脚本一律用 `setup/env.sh` 的变量或 `scripts/_paths.py`，**不写作者的绝对路径**。
2. 飞行/评分链（`setup/`、`scripts/fly_*`、`scripts/run_*`、`scripts/accept.py`、`scripts/check_*`、
   `harness/src/{bridge,supervisor,sim}`、`tests/`）必须路径无关，CI 会检查。
3. 历史分析工具（`scripts/` 下的一次性诊断）如果仍含旧路径，登记在下表，不再扩大。

## 4. 仍含历史路径的文件（有意保留）

见 `tests/no_abs_paths.sh` 的输出：它把"必须路径无关"的文件当失败，把历史工具当提示。
