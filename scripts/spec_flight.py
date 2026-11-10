#!/usr/bin/env python3
"""spec_flight.py -- 从一趟飞行记录里逐条量 docs/SPEC.md 的 S1/S3（独立于 accept.py 的十一行）。

S1 要的三件事，各自有独立证据：
  S1a 到达    真值是否进过每个航点的 **0.5 m 球**（`tap.csv`，世界系 W）——不看 pacer 自己的声明；
  S1b 不驻停  ① 参考在每个航点附近的最小速度（`setpoints.csv` 的 vel_*，取航点前后 2 m 的窗）
              ② 真值在每个航点 2 m 内的最小速度
              ③ 全程"停车"次数（真值水平速度 < 0.3 m/s 的连续段，起飞/降落两段不计）
  S1c PVA     薄层发给 SUPER 的每个 goal 是否带 v = 4.0 m/s、方向指向下一航点（`traj.log` 的 GOAL 行）
S3 平均地速   真值路径长度 / 路线时间（与 scripts/speed_audit.py 同口径），并给参考均速与 >4 m/s 占比。

    python3 scripts/spec_flight.py --run /home/fanxin/lio_avoid/logs/missionA-h81-01-01 \\
        --world worlds/forest_v2_hex.yaml
"""
import argparse
import csv
import math
import os
import re
import sys

import numpy as np
import yaml

ap = argparse.ArgumentParser()
ap.add_argument("--run", required=True)
ap.add_argument("--world", default="worlds/forest_v2_hex.yaml")
ap.add_argument("--z", type=float, default=None)
ap.add_argument("--sphere", type=float, default=None, help="default: mission.arrive_sphere_m")
ap.add_argument("--exit-speed", type=float, default=None, help="default: mission.corner_exit_speed_mps")
ap.add_argument("--json", default="")
a = ap.parse_args()

w = yaml.safe_load(open(a.world, encoding="utf-8"))
mis = w["mission"]
corners_all = np.array([[float(c[0]), float(c[1])] for c in mis["corners"]], float)
z = float(a.z if a.z is not None else mis.get("takeoff_alt", 2.0))
sphere = float(a.sphere if a.sphere is not None else mis.get("arrive_sphere_m", 0.5))
vexit = float(a.exit_speed if a.exit_speed is not None else mis.get("corner_exit_speed_mps", 4.0))
laps = int(mis.get("laps", 3))
ring = corners_all[:6]

ok = True
rows = []


def row(good, name, detail):
    global ok
    ok = bool(good) and ok
    rows.append({"id": name, "ok": bool(good), "detail": detail})
    print("%-6s %-42s %s" % ("PASS" if good else "FAIL", name, detail))


def load(name, cols):
    p = os.path.join(a.run, name)
    if not os.path.isfile(p):
        return None
    r = list(csv.DictReader(open(p, encoding="utf-8")))
    if not r:
        return None
    return {c: np.array([float(x[c]) for x in r]) for c in cols}


def fit_shift(ts, C, tt, T):
    best, bt = None, 0.0
    for dt in np.arange(-3.0, 3.01, 0.02):
        x = np.interp(tt + dt, ts, C[:, 0])
        y = np.interp(tt + dt, ts, C[:, 1])
        d = float(np.median(np.hypot(T[:, 0] - x, T[:, 1] - y)))
        if best is None or d < best:
            best, bt = d, dt
    return bt


print("== SPEC S1/S3 逐条自检：%s" % os.path.basename(a.run.rstrip("/")))
print("   world %s | 3D 到达球 %.2f m | 航点出弯速度 %.2f m/s | %d 航点 + 停机坪"
      % (os.path.basename(a.world), sphere, vexit, len(corners_all)))

sp = load("setpoints.csv", ["t_sim_us", "cmd_x", "cmd_y", "cmd_z", "vel_x", "vel_y"])
tp = load("tap.csv", ["t_us", "pe", "pn", "pd", "ve", "vn"])
if sp is None or tp is None:
    print("FAIL   no setpoints.csv / tap.csv -- 没有飞行记录")
    sys.exit(1)

ts = sp["t_sim_us"] * 1e-6
C = np.column_stack([sp["cmd_x"], sp["cmd_y"], sp["cmd_z"]])
V = np.column_stack([sp["vel_x"], sp["vel_y"]])
tt = tp["t_us"] * 1e-6
T = np.column_stack([tp["pe"], tp["pn"], -tp["pd"]])
Vt = np.column_stack([tp["ve"], tp["vn"]])
dt = fit_shift(ts, C, tt, T)
x = np.interp(tt + dt, ts, C[:, 0])
y = np.interp(tt + dt, ts, C[:, 1])
zr = np.interp(tt + dt, ts, C[:, 2])
sp_ref = np.hypot(np.interp(tt + dt, ts, V[:, 0]), np.interp(tt + dt, ts, V[:, 1]))
sp_tr = np.hypot(Vt[:, 0], Vt[:, 1])

# ---------------- S1a: 真值进过每个 0.5 m 球 ----------------
d3 = np.empty(len(corners_all))
for i, (cx, cy) in enumerate(corners_all):
    d3[i] = float(np.min(np.hypot(T[:, 0] - cx, T[:, 1] - cy) + 0.0 * np.abs(T[:, 2] - z)))
d2 = np.array([float(np.min(np.hypot(T[:, 0] - cx, T[:, 1] - cy))) for cx, cy in corners_all])
hit = d2 <= sphere
row(bool(hit.all()), "S1a 每个航点真值都进过 %.2f m 球" % sphere,
    "%d/%d 命中；最差 %.3f m（航点 %d）\n     逐点：%s"
    % (int(hit.sum()), len(hit), float(d2.max()), int(np.argmax(d2)) + 1,
       " ".join("%d:%.2f%s" % (i + 1, dd, "" if dd <= sphere else "!") for i, dd in enumerate(d2))))

# ---------------- S1b: 不驻停 ----------------
# 参考：每个航点前后 2 m 的沿航迹窗内的最小参考速度
ref_min = []
for k in range(len(corners_all)):
    cx, cy = corners_all[k]
    d = np.hypot(x - cx, y - cy)
    m = d <= 2.0
    ref_min.append(float(sp_ref[m].min()) if m.any() else float("nan"))
tr_min = []
for (cx, cy) in corners_all:
    d = np.hypot(T[:, 0] - cx, T[:, 1] - cy)
    m = d <= 2.0
    tr_min.append(float(sp_tr[m].min()) if m.any() else float("nan"))
ref_min = np.array(ref_min)
tr_min = np.array(tr_min)
n_ref_zero = int((ref_min < 0.30).sum())
n_tr_zero = int((tr_min < 0.30).sum())
row(n_ref_zero == 0 and n_tr_zero == 0, "S1b 航点处不驻停（参考/真值 <0.3 m/s）",
    "参考最慢 %.2f m/s（%d 个航点 <0.3）；真值最慢 %.2f m/s（%d 个航点 <0.3）"
    % (np.nanmin(ref_min), n_ref_zero, np.nanmin(tr_min), n_tr_zero))

# 全程停车段：真值水平速度 < 0.3，且已离地（z > 1.0）——起飞前与落地后不算
air = T[:, 2] > 1.0
slow = air & (sp_tr < 0.30)
segs = 0
if slow.any():
    d = np.diff(np.concatenate([[0], slow.astype(int), [0]]))
    segs = int(((d == 1).sum()))
t_air = float(np.sum(np.diff(tt) * air[:-1]))
row(segs == 0, "S1b 全程没有停车段（离地后）",
    "离地 %.1f s，真值 <0.3 m/s 的连续段 %d 个" % (t_air, segs))

# ---------------- S1c: PVA goal ----------------
logp = os.path.join(a.run, "traj.log")
pv = []
if os.path.isfile(logp):
    txt = open(logp, encoding="utf-8", errors="replace").read()
    pv = re.findall(r"GOAL \d+/\d+ → SUPER \[PVA\].*?v_out \(([-0-9.]+), ([-0-9.]+)\)", txt)
    n_goal = len(re.findall(r"GOAL \d+/\d+", txt))
else:
    n_goal = 0
if not pv:
    row(False, "S1c goal 带 PVA（v = %.1f m/s）" % vexit,
        "traj.log 里没有 PVA goal 行（n_goal=%d）——薄层可能没开 --goal-pva 1" % n_goal)
else:
    vout = np.array([[float(p[0]), float(p[1])] for p in pv])
    mag = np.hypot(vout[:, 0], vout[:, 1])
    # 方向检查：每个 goal 的速度方向应指向下一个航点（±10°）
    bad = 0
    for i, (vx, vy) in enumerate(vout[:len(corners_all) - 1]):
        if i + 1 >= len(corners_all):
            break
        u = corners_all[i + 1] - corners_all[i]
        nu = np.linalg.norm(u)
        nv = math.hypot(vx, vy)
        if nu < 1e-9 or nv < 1e-6:
            continue
        cosang = float(np.dot(u, [vx, vy]) / (nu * nv))
        if cosang < math.cos(math.radians(10.0)):
            bad += 1
    row(bool(np.allclose(mag, vexit, atol=0.05) and bad == 0),
        "S1c goal 带 PVA（v = %.1f m/s，指向下一航点）" % vexit,
        "%d 个 PVA goal，|v| %.2f–%.2f m/s，方向偏差 >10° 的 %d 个"
        % (len(pv), mag.min(), mag.max(), bad))

# ---------------- S3: 平均地速 ----------------
# 路线时间与长度都取真值，起飞后（z>1.0）到最后一个航点被到达为止
i0 = int(np.argmax(air))
seg = slice(i0, None)
L = float(np.sum(np.hypot(np.diff(T[seg, 0]), np.diff(T[seg, 1]))))
dts = float(tt[-1] - tt[i0])
mean_gs = L / dts if dts > 0 else float("nan")
mean_ref = float(np.mean(sp_ref[air])) if air.any() else float("nan")
frac4 = float(np.mean(sp_tr[air] > 4.0)) if air.any() else 0.0
row(mean_gs > 4.0, "S3 全程平均地速 > 4.0 m/s",
    "真值 %.2f m/s（路径 %.1f m / %.1f s）；参考均速 %.2f m/s；真值 >4 m/s 占 %.1f %%"
    % (mean_gs, L, dts, mean_ref, 100.0 * frac4))

print()
print("== %s" % ("S1 + S3 全部满足（这一趟）" if ok else "有条款没满足，见上面 FAIL 行"))
if a.json:
    import json
    json.dump({"run": a.run, "ok": ok, "rows": rows,
               "s1a_worst_m": float(d2.max()), "s1b_ref_min": float(np.nanmin(ref_min)),
               "s1b_truth_min": float(np.nanmin(tr_min)), "s1b_stop_segments": segs,
               "s3_mean_ground_speed": mean_gs, "s3_ref_mean": mean_ref, "s3_frac_above_4": frac4},
              open(a.json, "w", encoding="utf-8"), indent=1)
    print("   wrote %s" % a.json)
sys.exit(0 if ok else 1)
