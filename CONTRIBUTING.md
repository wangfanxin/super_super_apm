# 贡献与开发约定

本仓库的"正确"由 **可复现的测量** 定义，而不是由代码看起来如何定义。改动请遵守下面四条。

## 1. 改之前

```bash
bash tests/run_all.sh          # 无飞行自检：语法、静态闸门、单元测试、路径、证据链接
```

## 2. 记录的纪律

* 一次只改**一个变量**；配置改动走**变体文件**（`hku_super/config/super_planner_hex_*.yaml`），
  交付配置（`hku_super/config/super_planner_hex.yaml`）只在测量支持时改，并在 `docs/FLIGHT-ANALYSIS.md` 的实验历史里留一行（旧文档已合并）。
* 飞控参数改动走 `EXTRA_PARMS=<文件>`，不改交付参数文件。
* 每个手臂**至少 3 趟**（本仓库的跑动方差：同一配置 W6 p90 0.32–0.42），用
  `scripts/accept.py` + `scripts/flight_forensics.py` + `scripts/low_speed_census.py` 量。
* **不放松阈值**让结果通过；如果需要改判据，先写进 `docs/ACCEPTANCE.md`（谁提的、为什么、旧值是什么）。
* 结果写进 `docs/FLIGHT-ANALYSIS.md`（新开一节，写清楚"假设 → 变体 → 数字 → 结论"），失败的实验同样要写。

## 3. 代码约定

* 路径：只用 `setup/env.sh` 的变量或 `scripts/_paths.py`；**任何新文件不允许出现作者的绝对路径**。
* 提交前：`bash tests/run_all.sh` 必须 PASS；`bash scripts/check_static.sh` 也必须 PASS。
* 语言：注释与文档用中文，标识符与命令行用英文；注释里写**为什么**（含实测数字），不写"做了什么"。
* 第三方代码不改：改上游用补丁脚本（`hku_super/patches/patch_super_*.py`，带 `--check/--apply/--revert`）。

## 4. 提交信息

第一行是"改了什么"，正文写"为什么 + 数字 + 怎么验证的"。例：

    arc: 参考低通 tau=0.08 否掉

    横向 p90 0.185→0.16，但沿航迹 p5 -0.19→-0.40，净零（3 趟）。
    REF_LP_TAU 默认仍是 0；开关与单测保留。
