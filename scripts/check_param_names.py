#!/usr/bin/env python3
"""拿"整表"校验 .parm 文件里的每一个参数名，报出这个固件里不存在的名字。

为什么需要这个工具
------------------
MAVLink 的 `PARAM_SET` 对一个**不存在的名字**不报错：ArduPilot 的
`GCS_MAVLINK::handle_param_set`（GCS_Param.cpp:265）只发一条 PARAM_ERROR 就 return，
GCS 侧完全看不到异常。于是"参数灌进去了"和"参数根本没生效"在日志上一模一样。

本工程被这件事坑过两次：
  1. `ARMING_CHECK` / `GPS_TYPE` —— 这个固件改用 `ARMING_SKIPCHK` / `GPS1_TYPE`，
     导致"预解锁检查关不掉"，排查了整整三轮。
  2. `PSC_POSXY_P` / `PSC_VELXY_P` —— ArduCopter 4.7 改成 `PSC_NE_*` / `PSC_D_*`，
     导致"为了提高 4 m/s 跟踪精度而调硬位置环"这件事**静默没发生**。

所以：**任何 .parm 在灌进去之前，先对一遍整表。**

用法
----
  # 1) 先dump整表（SITL 在跑的时候）：
  python3 scripts/dump_all_params.py            # → /tmp/params.csv（name,value）
  # 2) 校验：
  python3 scripts/check_param_names.py params/*.parm
  # 也可以直接喂解析出的名字列表：
  python3 scripts/check_param_names.py --csv /tmp/params.csv params/apm_sitl.parm

退出码：0 = 全部认识；1 = 有不认识的名字（会在 stderr 列出来）。
"""
from __future__ import annotations

import argparse
import csv
import glob
import os
import sys

DEFAULT_CSV = "/tmp/params.csv"


def load_known(path):
    known = {}
    with open(path, newline="") as fh:
        rdr = csv.reader(fh)
        head = next(rdr, None)
        if head is None or head[0].strip() != "name":
            raise SystemExit("%s: 表头不是 name,value（先跑 scripts/dump_all_params.py）" % path)
        for row in rdr:
            if len(row) >= 2:
                try:
                    known[row[0].strip()] = float(row[1])
                except ValueError:
                    continue
    return known


def parm_entries(path):
    """(name, value, lineno)，跳过空行与整行注释；行尾注释也切掉。

    行尾注释这一点是实测出来的：`EK3_SRC1_POSXY 6   # 6 = ExternalNav` 曾让整行
    解析失败、值停在旧值 3，所以本工程的所有 .parm 都写成注释单独占行。
    """
    for i, raw in enumerate(open(path, encoding="utf-8", errors="ignore"), 1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) < 2:
            yield parts[0], None, i
            continue
        yield parts[0], parts[1], i


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="+", help=".parm 文件（支持通配符，需 shell 展开）")
    ap.add_argument("--csv", default=DEFAULT_CSV, help="整表 name,value")
    ap.add_argument("-q", "--quiet", action="store_true", help="只报错，不打印已识别的")
    args = ap.parse_args()

    paths = []
    for f in args.files:
        hit = glob.glob(f) or [f]
        paths.extend(hit)
    if not paths:
        print("no files", file=sys.stderr)
        return 1
    if not os.path.exists(args.csv):
        print("缺整表 %s：先在 SITL 跑着的时候执行 scripts/dump_all_params.py" % args.csv,
              file=sys.stderr)
        return 1

    known = load_known(args.csv)
    bad_total = 0
    for path in paths:
        bad = []
        n = 0
        for name, val, lineno in parm_entries(path):
            n += 1
            if name not in known:
                bad.append((lineno, name, val))
        if bad:
            bad_total += len(bad)
            print("%s: %d 项，%d 个名字在这个固件里**不存在**" % (path, n, len(bad)))
            for lineno, name, val in bad:
                print("  line %-4d %-18s %s   ← 会被静默丢弃" % (lineno, name, val))
        elif not args.quiet:
            print("%s: %d 项，全部命中整表" % (path, n))

    if bad_total:
        print()
        print("合计 %d 个名字对不上。常见原因是固件版本改了参数名：" % bad_total)
        print("  4.7 把 PSC_POSXY_* 改成 PSC_NE_*，把 ARMING_CHECK 改成 ARMING_SKIPCHK。")
        print("去整表里 grep 相近的名字（比如 grep '^PSC_NE' /tmp/params.csv）。")
        return 1
    print()
    print("OK：所有名字都存在于整表（%d 项）" % len(known))
    return 0


if __name__ == "__main__":
    sys.exit(main())
