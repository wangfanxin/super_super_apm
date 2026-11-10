#!/usr/bin/env python3
"""lidar_stall.py -- did the LiDAR SIMULATOR keep up, and how many scans did it throw away?

WHY THIS IS A GATE AND NOT A DIAGNOSTIC: `docs/INTEGRATION.md` §87 measured that every fence escape of
round 89 coincides with a mid-flight raycast stall inside `src/sim/lidar_imu.py`:

    tag            cast p50   cast MAX   skipped   outcome
    kxD-01/02/03      21-22   32-39 ms    0-3      landed (3/3)
    kxC-01-01         24.9    60.3       1        landed
    kxC-02-01         24.6   295.0      267       FENCE
    kxB-01-01         23.8   333.5      175       FENCE
    kx8-03-01         21.1   425.8      172       FENCE
    kx8-02-01         20.3   570.1      288       FENCE

A scan whose cast takes 200-570 ms cannot be published, so the node SKIPS it ("windows the truth did not
cover"), the cloud rate collapses, Super-LIO has to scan-match across a gap, and its world pose diverges
(yaw steps up to 154 deg were recorded).  The FCU then follows the bad estimate out of the fence -- so the
sortie's control numbers describe a sensor starvation, not a control problem.

    python3 scripts/lidar_stall.py /home/fanxin/lio_avoid/logs/missionA-kxD-01-01
    -> "cast_max_ms=38.7 skipped=0"   (exit 0 when inside the gate)

Thresholds: a healthy scan costs 20-25 ms and the budget is 100 ms (the 10 Hz scan period), so
`--cast-max-ms 100` still allows a 4x spike; `--skip-max 20` allows a couple of boundary scans.
"""
import argparse
import os
import re
import sys


def parse(path):
    cast, skip = [], []
    for ln in open(path, errors="replace"):
        m = re.search(r"cloud ([\d.]+) Hz.*?cast ([\d.]+) ms, skipped (\d+)", ln)
        if m:
            cast.append(float(m.group(2)))
            skip.append(int(m.group(3)))
    if not cast:
        return None
    # the first report is the startup sample (cloud 0.5 Hz, cast noise): drop it
    c = cast[1:] if len(cast) > 2 else cast
    return dict(cast_max=max(c), cast_p50=sorted(c)[len(c) // 2], skipped=skip[-1], n=len(cast))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--cast-max-ms", type=float, default=100.0)
    ap.add_argument("--skip-max", type=int, default=20)
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()
    p = os.path.join(a.run, "lidar_imu.log") if os.path.isdir(a.run) else a.run
    r = parse(p)
    if r is None:
        print("n/a")
        return 2
    ok = (r["cast_max"] <= a.cast_max_ms) and (r["skipped"] <= a.skip_max)
    if a.json:
        print('{"cast_max_ms": %.1f, "cast_p50_ms": %.1f, "skipped": %d, "ok": %s}'
              % (r["cast_max"], r["cast_p50"], r["skipped"], "true" if ok else "false"))
    else:
        print("cast_p50_ms=%.1f cast_max_ms=%.1f skipped=%d" % (r["cast_p50"], r["cast_max"], r["skipped"]))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
