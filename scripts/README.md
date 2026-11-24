# scripts/ —— 飞行臂、评分器、分析工具

这个目录**混放**三类东西，命名规则如下（历史原因，短期不重排，以免打断 `docs/INTEGRATION.md` 的引用）：

| 类别 | 命名 | 例子 | 状态 |
|---|---|---|---|
| **飞行臂**（起一趟或一组飞行，打分） | `fly_*`, `run_*`, `check_static.sh` | `fly_hex2.sh`, `run_until_healthy.sh`, `run_accept.sh` | current，路径无关 |
| **评分与自检** | `accept.py`, `check_*.py`, `render_evidence.sh` | `accept.py`（15 行判据）、`check_frozen.py`、`check_pattern.sh` | current，路径无关 |
| **分析工具**（从一趟记录里算一个量） | 小写下划线，单一职责 | `reversal_table.py`、`dip_context.py`、`legspeed.py`、`plan_clearance.py` | 多数 current；一批是一次性诊断（保留备查） |

## 约定

* 所有工具**只读记录**（`.runs/missionA-<tag>/`）或只读配置，不修改任何东西。
* 路径默认值来自 `scripts/_paths.py`（`ROOT`/`LOGS`/`HARNESS`/`CALIB`/…），也可用命令行参数覆盖。
* 一个工具一个量：需要组合时在 `_scratch/` 写组合脚本，而不是把多个量塞进一个文件。
* 新工具必须 `python3 scripts/<tool>.py -h` 可用（CI 的 `tests/run_all.sh` 会跑一遍 `--help`）。

## 常用

```bash
python3 scripts/accept.py --run /path/to/missionA-<tag> --world worlds/forest_hex12.yaml \
        --alt 2.0 --body box
python3 scripts/reversal_table.py <tag> [<tag> ...]     # REV / dips
python3 scripts/plan_clearance.py <tag> --world worlds/forest_hex12.yaml
bash scripts/render_evidence.sh <tag>                    # 四张证据图
```
