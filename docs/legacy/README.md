# docs/legacy —— 不再使用 / 不再随仓库发布的源码

## 留着备查

| 文件 | 是什么 | 为什么留着 |
|---|---|---|
| `lidar_imu.py.orig-loop` | `harness/src/sim/lidar_imu.py` 的旧循环实现（改写成非阻塞之前） | 排查"传感器为什么只跑到 5.6 Hz"时需要对照；不参与任何执行路径 |

## 已删除（不再随仓库发布）

| 文件 | 为什么删 |
|---|---|
| `scripts/pscd_window.py` | 依赖 `vertical_forensics.py`（从未进入本仓库的辅助模块），单独存在时 `import` 即失败。它要量的东西由 `dump_all_params.py` / `ekf_log.py` 覆盖。需要时 `git log -- scripts/pscd_window.py` 可取回 |

`docs/CHAIN.md` 与 `docs/INTEGRATION.md` 里仍会出现 `pscd_window.py`，那是**历史记录**，不回改。
