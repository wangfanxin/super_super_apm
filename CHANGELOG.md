# 变更记录

只记**影响交付或影响复现**的变更；逐轮的测量细节在 `docs/INTEGRATION.md`。

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
