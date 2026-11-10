"""plan_anatomy.py -- how the seam's trajectory stream was actually structured, from wire.csv.

wire.csv is the bridge's record of every setpoint it put on the wire: it carries the trajectory id and
flag the thin layer attached.  Grouping by id answers the question the figures cannot: how many distinct
plans did the aircraft get, how long did each last, and were the long ones moving or holding?
"""
import csv
import sys
from collections import OrderedDict

run = sys.argv[1]
rows = list(csv.DictReader(open("/home/fanxin/lio_avoid/logs/missionA-%s/wire.csv" % run)))
print("== %s: %d wire rows" % (run, len(rows)))
seg = OrderedDict()
t0 = float(rows[0]["t_wall"])
for r in rows:
    key = (r["trajectory_id"], r["trajectory_flag"])
    t = float(r["t_wall"]) - t0
    seg.setdefault(key, []).append((t, float(r["p_l_x"]), float(r["p_l_y"]), float(r["p_l_z"]),
                                    float(r["v_l_x"]), float(r["v_l_y"])))
print("   %4s %-14s %6s %8s %8s %8s %9s %9s" % ("#", "(id,flag)", "rows", "t0", "dur", "path", "v p50", "xy span"))
ordered = sorted(seg.items(), key=lambda kv: kv[1][0][0])
for i, ((tid, flag), s) in enumerate(ordered[:40]):
    d = s[-1][0] - s[0][0]
    path = 0.0
    for a, b in zip(s, s[1:]):
        path += ((b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2) ** 0.5
    import statistics
    vs = [((r[4] ** 2 + r[5] ** 2) ** 0.5) for r in s]
    xs = [r[1] for r in s]
    ys = [r[2] for r in s]
    print("   %4d %-14s %6d %8.2f %8.2f %8.2f %9.2f %9.2f" % (
        i, "(%s,%s)" % (tid, flag), len(s), s[0][0], d, path,
        statistics.median(vs), ((max(xs) - min(xs)) ** 2 + (max(ys) - min(ys)) ** 2) ** 0.5))
print("   total distinct (id,flag): %d" % len(seg))
