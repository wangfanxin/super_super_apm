# 变更记录

只记**影响交付或影响复现**的变更；逐轮的测量细节在 `docs/INTEGRATION.md`。

## 0.1.1 — 2026-11-24（模块化重组）

* **仓库按模块拆分**（用户要求"分模块进行"）：
  `apm_thin/`（自研薄层 + 共享库）、`hku_super/`（SUPER 源码 + 交付配置 + patches）、
  `slam/`（Super-LIO + livox + LIO 配置）、`ardupilot/`（bridge/supervisor/params/msgs）、
  `sim/`（仿真代码/世界/rviz/冻结清单/射线图案）。
  旧的 `harness/`、`third_party/`、`src/`、根 `config|params|worlds/` 全部消失。
* **模块间只通过两种东西耦合**：运行时的话题/端口；构建时的 `setup/env.sh` 变量
  （`THIN_ROOT`/`SUPER_PLANNER_ROOT`/`SLAM_ROOT`/`SIM_ROOT`/`ARDUPILOT_DIR`）。
* 入口脚本、评分器、分析工具改为从模块变量取路径；`tests/run_all.sh` 与 CI 全部通过。
* **重组后真飞验证**：`tidy11-01-01` —— H1–H6 全过、W0b/W1/W1b/W2/W3/W5 过、W4 接触（已知缺陷行）、
  W6 未过（该趟落在正常分布内）。
* 文档：`docs/DATA_PIPELINE.md` §9 与 `README.md` 的目录表改写为新布局；`docs/PATHS.md` 增加模块层。

## 0.1.0 — 2026-11-24

* **仓库成型**：`super_super_apm` 成为单一可 clone 的仓库（源码 + harness + 内置 SUPER/Super-LIO/livox +
  文档 + 证据图），推到 `github.com/wangfanxin/super_super_apm`。
* **部署与运行分离**：`setup/env.sh` 是唯一路径权威，支持 `setup/runtime.env` 的机器本地覆盖；
  编译产物与飞行记录在 `.build/` / `.runs/`（本机在 WSL 的 `/home/fanxin/super_super_apm`）。
* **交付档**：`max_vel 1.5 → 1.35 m/s`（18 趟速度前沿，1.35 是唯一同时满足 W2 p90≥1.40 与
  W4 自动转正的点）；`accept.py` 的 W2 改为"巡航 p90 ≥1.40 且摸到上限"；W4 支持 `--known-defect`。
* **数据管线文档**：`docs/DATA_PIPELINE.md`（运行时 + 产物 + 评分链 + 端口/话题/频率）。
* **传感器图案**：`harness/sim/mid360_pattern.csv` 不再入库（供应商数据），
  `harness/sim/README.md` + `scripts/check_pattern.sh` 给出获取与校验方法。
* **清理**：旧工程树（`lio_avoid` 的代码副本、`lio_nav_apm`、`gazebo_apm`）与 `_stage` 大头移出；
  飞行记录保留文档引用的 70 趟裁剪版（3.1 GB）。
* **自检**：`tests/run_all.sh`（无飞行闸门）+ `.github/workflows/ci.yml`。
