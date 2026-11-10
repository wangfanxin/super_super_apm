# 仍假设作者绝对路径的文件（编译与飞行链**不**在其中）

本仓库的**编译 / 飞行 / 评分**链是位置无关的：`setup/env.sh` 是唯一知道路径的地方，
`scripts/` 与 `harness/` 里的入口脚本会自己向上找到它（`setup/env.sh` 的 bootstrap 由打包脚本插入）。
从任意目录 clone 都能跑，`docs/` 里的每个数字都出自这条链。

**例外是分析工具**。`scripts/` 下约 150 个工具里，仍有一部分把作者的绝对路径写成默认值，例如

    /mnt/d/distill_self/…            （作者的工作区）
    /home/fanxin/lio_avoid/logs/…    （作者的记录目录）

它们不影响编译与飞行，而且**都接受显式参数**（`--run`、`--world`、`--tag`），所以在任何机器上只要
把路径传进去就能用，例如：

    python3 scripts/accept.py --run /data/runs/missionA-kx6-03-01-01 \
                              --world worlds/forest_hex12.yaml --alt 2.0 --body box

为什么没有全部改掉：打包时对 223 个 Python 文件做过一次机械替换，结果 23 个文件被插到了
`from __future__` 之前而报语法错误、108 个文件里的 f-string / 隐式拼接仍然残留旧路径 —— 那种"看起来
改完了其实一半没改"的状态比诚实标注更糟。所以范围被明确限定为"编译 + 飞行 + 评分"这条链，
其余在本文档里列清楚。

判断某个工具属于哪一类：如果它在 `setup/run_hex.sh`、`scripts/fly_hex*.sh`、
`scripts/run_until_healthy.sh`、`scripts/run_accept.sh` 的调用链上，它已经位置无关；否则按上面的
方式显式传参。
