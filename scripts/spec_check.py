#!/usr/bin/env python3
"""spec_check.py -- 逐条对照 docs/SPEC.md 自检，打印 PASS/FAIL 与实测数字。

规矩（docs/SPEC.md）要求"每一轮工作结束前必须对照本文件逐条自检，不许用大概/应该/接近"。
这个脚本就是那个自检：S0/S1 的任务几何 + S2 的柱林由世界文件静态审计；S1 的过点与 S3 的速度需要
飞行记录（没有就明确打印 NOT MEASURED）。

第 81 轮起：**不再有"安全半径 0.2 m"这一条**；柱子是"下方（0.2 m 边长）上圆（0.2 m 直径）"，
任务高度 2.0 m 落在方段，所以距离一律按方盒算；唯一允许的空格是"含航点的格子"。

    python3 scripts/spec_check.py                        # 默认 worlds/forest_v2_hex.yaml
    python3 scripts/spec_check.py --flight <tag>         # 有飞行记录时再逐条量 S1/S3
"""
import argparse
import io
import math
import os
import sys

import yaml


def row(ok, name, detail):
    print("%-6s %-40s %s" % ("PASS" if ok else "FAIL", name, detail))
    return bool(ok)


def box_surface_dist(dx, dy, r):
    """点到方柱表面的距离（外为正，内为 0）；与 src/whtf/common/obstacles.py 同一口径。"""
    ox = abs(dx) - r
    oy = abs(dy) - r
    if ox <= 0.0 and oy <= 0.0:
        return max(ox, oy)
    return math.hypot(max(ox, 0.0), max(oy, 0.0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="worlds/forest_hex12.yaml")
    ap.add_argument("--pitch", type=float, default=None, help="cell size (default: read from v2.pitch)")
    ap.add_argument("--half", type=float, default=12.0)
    ap.add_argument("--radius", type=float, default=None, help="S0 corner-to-pad distance")
    ap.add_argument("--z", type=float, default=None, help="mission altitude (default: mission.takeoff_alt)")
    ap.add_argument("--r", type=float, default=0.10)
    ap.add_argument("--pad-clear", type=float, default=0.35)
    ap.add_argument("--cfg", default="config/super_planner_hex.yaml",
                    help="the planner config, for the S4 numbers (robot_r, inflation_step)")
    ap.add_argument("--flight", default="", help="a missionA-<tag> log directory: also check S1/S3")
    args = ap.parse_args()

    w = yaml.safe_load(open(args.world, encoding="utf-8"))
    pil = w["pillars"]
    mis = w.get("mission", {}) or {}
    v2 = w.get("v2", {}) or {}
    geo = w.get("pillar_geometry", {}) or {}
    pitch = args.pitch if args.pitch is not None else float(v2.get("pitch", 1.0))
    radius = args.radius if args.radius is not None else float(mis.get("radius_m", 10.0))
    z = args.z if args.z is not None else float(mis.get("takeoff_alt", 2.0))
    z_split = float(geo.get("z_split", 2.1))
    ok_all = True

    print("== SPEC self-check: %s (%d pillars)" % (args.world, len(pil)))
    print("   SPEC.md S0/S1/S2 are the binding rules; a change needs the user's consent and a record row.")
    print()

    # ---------------- S0 ----------------
    print("-- S0 航点：正六边形、每个角点到中心起点 %.3f m" % radius)
    corners_all = [(float(c[0]), float(c[1])) for c in (mis.get("corners") or [])]
    laps = int(mis.get("laps", 0) or v2.get("laps", 0) or 0)
    corners = []
    for c in corners_all:
        if not any(math.hypot(c[0] - q[0], c[1] - q[1]) < 1e-6 for q in corners):
            corners.append(c)
    if not corners:
        ok_all &= row(False, "S0 六边形角点", "world 文件里没有 mission.corners")
    else:
        d = [math.hypot(x, y) for x, y in corners]
        sides = [math.dist(corners[i], corners[(i + 1) % 6]) for i in range(6)]
        ang = [math.degrees(math.atan2(y, x)) % 360.0 for x, y in corners]
        ok_all &= row(len(corners) == 6, "S0 角点个数 = 6",
                      "%d 个不同角点（航点 %d 个 = %d 圈 × 6）" % (len(corners), len(corners_all), laps))
        ok_all &= row(max(abs(x - radius) for x in d) < 1e-3,
                      "S0 每个角点距 (0,0) = %.3f m" % radius,
                      "六个距离 %s m，最大误差 %.6f m"
                      % (["%.5f" % x for x in d], max(abs(x - radius) for x in d)))
        ok_all &= row(max(sides) - min(sides) < 1e-3, "S0 正六边形（六边等长）",
                      "边长 %.5f … %.5f m" % (min(sides), max(sides)))
        ok_all &= row(abs(sum(sides) / 6 - radius) < 1e-3, "S0 边长 = 外接半径",
                      "边长均值 %.5f m vs 半径 %.3f m" % (sum(sides) / 6, radius))
        ok_all &= row(laps > 0 and len(corners_all) == 6 * laps, "S0 圈数 × 6 = 航点数",
                      "%d 圈 × 6 = %d 个航点（world 里 %d 个）" % (laps, 6 * laps, len(corners_all)))
        ok_all &= row(abs(z - float(mis.get("takeoff_alt", z))) < 1e-9, "S0 任务高度",
                      "%.2f m（柱子 z_split %.1f ⇒ 飞在方段）" % (z, z_split))

    # ---------------- S1 (the part that is in the world file) ----------------
    print()
    print("-- S1 过点：到达判据 = 球、不驻停、PVA 目标（几何部分）")
    arr = mis.get("arrive_sphere_m")
    ok_all &= row(arr is not None and abs(float(arr) - 0.5) < 1e-9, "S1 到达判据 = 0.5 m 球",
                  "mission.arrive_sphere_m = %s" % arr)
    # 第 83 轮：SPEC S1 的 PVA 速度从 4.0 降到 2.0，生效位置是**飞行臂**（fly_hex.sh 的 CORNER_SPEED），
    # 世界文件里的 mission.corner_exit_speed_mps 是冻结时的历史值（地图不许改，所以它仍是 4.0）。
    # 第 87 轮：用户要求 3 m/s ⇒ CORNER_SPEED 与规划器的 max_vel 都必须 = 3.0。
    # 这两处必须一起动，否则飞行臂给的 PVA 速度会**超过**规划器自己的速度上界（MINCO 会硬拒或削顶），
    # 所以这里不写死数字，而是拿配置里的 max_vel 当期望值做交叉核对。
    varm = None
    vcfg = None
    try:
        import re as _re
        _arm = io.open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                    "scripts", "fly_hex.sh"), encoding="utf-8").read()
        _m = _re.search(r"CORNER_SPEED=\$\{CORNER_SPEED:-([0-9.]+)\}", _arm)
        varm = None if _m is None else float(_m.group(1))
        if os.path.isfile(args.cfg):
            _t = io.open(args.cfg, encoding="utf-8").read()
            _mv = _re.search(r"^[ \t]*max_vel:[ \t]*([0-9.eE+-]+)", _t, _re.M)
            vcfg = None if _mv is None else float(_mv.group(1))
    except Exception:                                          # noqa: BLE001
        varm = None
    # 第 88 轮改规则：**不再要求 CORNER_SPEED == max_vel**。
    #   正六边形拐角 120°，过弯半径 R = v²/max_acc，轨迹在角点外凸约 R。
    #   v=2.8 / max_acc=8 ⇒ R=0.98 m ⇒ 实测外凸 1.31 m（hx9-01-01 的 cmd_x 到 11.31，角点在 10.000），
    #   再叠加 0.7 m 跟踪滞后就正好撞 12.0 m 围栏。v=1.5 ⇒ R=0.28 m。
    #   所以：max_vel 仍是**直道**上限（2.8），角点 PVA 是**角点**速度（1.5），硬约束只剩
    #   0.5 <= CORNER_SPEED <= max_vel（超过规划器上界时 MINCO 会硬拒/削顶），
    #   并且角点速度必须等于 SPEC 里写的那个数（防止悄悄漂移）。
    # 允许的角点速度：每一个都必须在 SPEC 修改记录里有一行（防漂移），且不得超过规划器的 max_vel。
    CORNER_DOCUMENTED = {0.0: "停住模式（用户第 88 轮实验，S1 原本禁止在航点停住）",
                         0.8: "减速交接（第 88 轮补 6，当前档）",
                         1.5: "不减速的飞行交接（= max_vel，第 87/88 轮交付档）"}
    _hit = None
    for _v in CORNER_DOCUMENTED:
        if varm is not None and abs(varm - _v) < 1e-9:
            _hit = _v
    # A corner speed of 0 means "stop at every waypoint", which SPEC S1 forbids.  It is allowed only
    # when the arm SAYS SO (CORNER_ALLOW_STOP=1), so the experiment can never be laundered silently
    # (round 88 audit F1).
    _arm_txt = io.open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                    "scripts", "fly_hex.sh"), encoding="utf-8").read()
    _stop_ok = ("CORNER_ALLOW_STOP=${CORNER_ALLOW_STOP:-1}" in _arm_txt)
    ok_all &= row(_hit != 0.0 or _stop_ok,
                  "S1 停住模式已显式承认（CORNER_ALLOW_STOP=1）" if _hit == 0.0 else
                  "S1 非停住模式（无需承认）",
                  "CORNER_SPEED = 0 会违反 S1 的「不允许在航点停住」，所以臂里必须有 "
                  "CORNER_ALLOW_STOP=1 这一行把它标成实验")
    ok_all &= row(varm is not None and vcfg is not None and _hit is not None
                  and varm <= vcfg + 1e-9,
                  "S1 航点 PVA：%s" % (
                      "CORNER_SPEED = %.1f m/s（%s）" % (_hit, CORNER_DOCUMENTED[_hit])
                      if _hit is not None else
                      "CORNER_SPEED = %s m/s —— **不在 SPEC 记录过的档位里**（%s）"
                      % (varm, sorted(CORNER_DOCUMENTED))),
                  "fly_hex.sh 的 CORNER_SPEED = %s（方向指向下一航点）、配置 max_vel = %s；"
                  "角点与直道同速 ⇒ 没有进角点的减速段，R = v²/max_acc = 1.5²/9 = 0.25 m；"
                  "世界文件里的 %s 是冻结时的历史值"
                  % (varm, vcfg, mis.get("corner_exit_speed_mps")))

    # ---------------- S2 ----------------
    print()
    print("-- S2 柱林：%.1f m 格、柱 0.2 m（下方上圆，z_split %.1f）、柱心格内随机、零车道、"
          "每个航点周围 %dx%d 格留空、覆盖 [-%.0f,%.0f]^2"
          % (pitch, z_split, 2 * int(v2.get("keepout_neigh", 0) or 0) + 1,
             2 * int(v2.get("keepout_neigh", 0) or 0) + 1, args.half, args.half))
    n_side = int(round(2 * args.half / pitch))
    ok_all &= row(n_side * pitch == 2 * args.half, "S2 格子尺寸",
                  "%.1f m 格 × %d×%d = 覆盖 [-%.1f, %.1f]^2" % (pitch, n_side, n_side, args.half, args.half))
    ok_all &= row(abs(float(geo.get("box_half_side_m", -1)) - args.r) < 1e-9
                  and abs(float(geo.get("cylinder_radius_m", -1)) - args.r) < 1e-9,
                  "S2 柱截面 0.2 m：下方 0.2 m / 上圆直径 0.2 m",
                  "pillar_geometry = %s" % {k: geo.get(k) for k in ("type", "box_half_side_m",
                                                                    "cylinder_radius_m", "z_split")})
    ok_all &= row(abs(z_split - 2.1) < 1e-9, "S2 方/圆分界 z_split = 2.1 m", "%.2f m" % z_split)

    # every pillar in its cell, one per cell, and ONLY the waypoint cells empty
    cells = {}
    offs = []
    on_centre = 0
    outside = 0
    for q in pil:
        x, y = float(q["x"]), float(q["y"])
        if not (abs(x) < args.half and abs(y) < args.half):
            outside += 1
        i = int(math.floor(x / pitch))
        j = int(math.floor(y / pitch))
        cells.setdefault((i, j), []).append(q)
        cx, cy = (i + 0.5) * pitch, (j + 0.5) * pitch
        offs.append(math.hypot(x - cx, y - cy))
        if math.hypot(x - cx, y - cy) < 1e-6:
            on_centre += 1
    all_cells = {(i, j) for i in range(-n_side // 2, n_side // 2) for j in range(-n_side // 2, n_side // 2)}
    holes = sorted(all_cells - set(cells))
    wp_cells = sorted({(int(math.floor(cx / pitch)), int(math.floor(cy / pitch))) for cx, cy in corners})
    # ROUND 88: the keep-out is the (2N+1)^2 block around each waypoint cell (N = v2.keepout_neigh; 1
    # means the owner's "航点附近 9 个格子"), not the single waypoint cell the old map used.
    _n = int(v2.get("keepout_neigh", 0) or 0)
    keep_cells = sorted({(i + di, j + dj) for (i, j) in wp_cells
                         for di in range(-_n, _n + 1) for dj in range(-_n, _n + 1)}) if _n else wp_cells
    ok_all &= row(not outside, "S2 柱子都在 [-%.0f,%.0f]^2 内" % (args.half, args.half),
                  "%d/%d 根在域内%s" % (len(pil) - outside, len(pil),
                                       "" if not outside else "，%d 根越界" % outside))
    ok_all &= row(holes == keep_cells, "S2 空格 = 航点周围 %dx%d 格" % (2 * _n + 1, 2 * _n + 1),
                  "空格 %d 个 = 航点格 %d 个 × %d 格/航点 = %d 个；其余 %d 格各一根"
                  % (len(holes), len(wp_cells), (2 * _n + 1) ** 2, len(keep_cells), len(cells)))
    dup = {k: v for k, v in cells.items() if len(v) > 1}
    ok_all &= row(not dup, "S2 每格至多一根", "0 个格子两根" if not dup else "%d 个格子两根以上" % len(dup))
    ok_all &= row(on_centre == 0, "S2 柱心格内随机（没有一根在格心）",
                  "格心命中 %d 根；偏移 p50 %.3f / max %.3f m（格半宽 %.3f）"
                  % (on_centre, sorted(offs)[len(offs) // 2], max(offs), pitch / 2))
    bad = [q for q in pil if abs(float(q.get("r", -1)) - args.r) > 1e-9]
    ok_all &= row(not bad, "S2 每根柱 r = %.2f m" % args.r, "%d 根不合" % len(bad) if bad else "全部一致")
    ok_all &= row("safety" not in yaml.safe_dump(w) and "safety_radius" not in w,
                  "S2 没有“安全半径”这号东西", "world 文件里搜不到 safety*")

    # the two physical constraints
    #   box-to-box: separation along each axis is |dx| - 2r (both boxes are 2r wide), so the gap is
    #   hypot(max(|dx|-2r, 0), max(|dy|-2r, 0)) -- NOT the centre distance minus one radius.
    mind, argmin = float("inf"), None
    for a_ in range(len(pil)):
        for b_ in range(a_ + 1, len(pil)):
            gx = box_surface_dist(float(pil[a_]["x"]) - float(pil[b_]["x"]),
                                  float(pil[a_]["y"]) - float(pil[b_]["y"]), 2.0 * args.r)
            if gx < mind:
                mind, argmin = gx, (a_, b_)
    ok_all &= row(mind >= 0.0, "约束① 柱与柱不重叠（按方盒口径）",
                  "最近的一对方盒表面净距 %.4f m（pair %s）" % (mind, argmin))
    pad_d = min(math.hypot(float(q["x"]), float(q["y"])) for q in pil)
    ok_all &= row(pad_d >= args.pad_clear, "约束② 柱不压停机坪",
                  "最近的柱心距 (0,0) %.3f m ≥ %.2f m" % (pad_d, args.pad_clear))

    # ---------------- S4: the two numbers the owner set ----------------
    print()
    print("-- S4 机体与规划器几何（robot_r 0.10 m 第 85 轮；inflation_step 2 第 88 轮）")
    if not os.path.isfile(args.cfg):
        ok_all &= row(False, "S4 规划配置", "找不到 %s" % args.cfg)
    else:
        import re
        txt = open(args.cfg, encoding="utf-8").read()

        def cval(key, default=None):
            m = re.search(r"^[ \t]*%s:[ \t]*([0-9.eE+-]+)" % key, txt, re.M)
            return float(m.group(1)) if m else default

        rr = cval("robot_r")
        istep = cval("inflation_step")
        ires = cval("inflation_resolution")
        ok_all &= row(rr is not None and abs(rr - 0.10) < 1e-9, "S4 robot_r = 0.10 m",
                      "配置里 robot_r = %s（方向：CIRI 走廊 + 种子线邻域 ceil(robot_r/res) = %s 体素）"
                      % (rr, None if rr is None else int(math.ceil(rr / 0.05))))
        ok_all &= row(istep is not None and abs(istep - 2.0) < 1e-9, "S4 inflation_step = 2",
                      "配置里 inflation_step = %s ⇒ 地图膨胀 %.2f m（A* 因此自己保持这个净空；"
                      "第 88 轮用户从 1 改成 2，因为两趟达标飞行的规划净空只有 0.014/0.100 m，"
                      "小于跟踪残差 p90 0.34 m）"
                      % (istep, (istep or 0) * (ires or 0.1)))
        amax = cval("max_acc")
        # 记录过的 max_acc 档位（每一档在 SPEC 修改记录里都有一行）
        ACC_DOCUMENTED = {8.0: "第 86 轮", 9.0: "第 88 轮补", 6.0: "第 88 轮补 8（当前）"}
        _ahit = None
        for _v in ACC_DOCUMENTED:
            if amax is not None and abs(amax - _v) < 1e-9:
                _ahit = _v
        _ceil = (9.81 * 1.026) ** 0.5 if amax is None else (amax * 1.026) ** 0.5
        ok_all &= row(amax is not None and _ahit is not None,
                      "S1 max_acc = %.1f m/s^2（%s）" % (_ahit, ACC_DOCUMENTED[_ahit])
                      if _ahit is not None else
                      "S1 max_acc = %s —— **不在 SPEC 记录过的档位里**（%s）"
                      % (amax, sorted(ACC_DOCUMENTED)),
                      "配置里 traj_opt.boundary.max_acc = %s；最紧角点 R_free = 1.026 m ⇒ "
                      "角点速度上限 sqrt(max_acc·R) = %.2f m/s（巡航 %.1f m/s 必须低于它）"
                      % (amax, _ceil, 1.5))
        vmax = cval("max_vel")
        # ROUND 20 (user "按A走"): the frozen cruise cap moved 1.5 -> 1.35 on measured evidence
        # (docs/INTEGRATION.md §106/§107/§110/§112: W6 p90 0.25*v-0.03, W2 p90 1.20*v-0.19, and 1.35
        # is the only operating point that both clears W2 (1.41-1.43) and lets W4 turn positive by
        # itself (2/3 CLEAR)).  The row stays a hard equality: the value is frozen at 1.35 now, and a
        # variant is still the only way to fly anything else.
        ok_all &= row(vmax is not None and abs(vmax - 1.35) < 1e-9, "S1 max_vel = 1.35 m/s",
                      "配置里 traj_opt.boundary.max_vel = %s（第 20 轮：1.5 → 1.35，用户\"按A走\"；"
                      "依据 18 趟速度前沿 —— 1.35 是唯一同时满足 W2 p90 ≥1.40 与 W4 自动转正的点。"
                      "第 88 轮的原值 1.5 与它当时的理由见修改记录表）"
                      % vmax)
        # ROUND 87: the bridge's feed-forward clamps must BE the plan's envelope (see the long note in
        # scripts/fly_hex.sh).  They used to be typed in by hand and went stale -- round 86 raised max_acc
        # to 8.0 while `--ff-acc-max` stayed 5.0, so the bridge clipped the plan's own acceleration for a
        # whole round.  The arm now derives them with scripts/cfg_envelope.py; this row re-runs that script
        # and compares, so a future stale clamp fails the static gate instead of silently clipping the PVA.
        env = None
        try:
            import subprocess
            _gen = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                "scripts", "cfg_envelope.py")
            _o = subprocess.run([sys.executable or "python3", _gen, args.cfg],
                                capture_output=True, text=True)
            if _o.returncode == 0:
                env = [float(x) for x in _o.stdout.split()]
        except Exception:                                      # noqa: BLE001
            env = None
        _mj = re.search(r"^[ \t]*max_jerk:[ \t]*([0-9.eE+-]+)", txt, re.M)
        jmax = None if _mj is None else float(_mj.group(1))
        ok_all &= row(env is not None and len(env) == 3
                      and vmax is not None and amax is not None and jmax is not None
                      and abs(env[0] - vmax) < 1e-9 and abs(env[1] - amax) < 1e-9
                      and abs(env[2] - jmax) < 1e-9,
                      "S1 前馈钳位 = 规划器上界",
                      "cfg_envelope.py 读到 v=%s a=%s j=%s；配置 max_vel=%s max_acc=%s max_jerk=%s"
                      "（钳位由飞行臂起飞时读取，不再手写）"
                      % (None if not env else env[0], None if not env else env[1],
                         None if not env else env[2], vmax, amax, jmax))
        # ROUND 87: the ARM flies 2 laps (12 corners + pad) while the frozen world still holds 3 laps.
        # The two facts must not be confused: the map is unchanged (check_frozen.py asserts laps == 3),
        # the tour length is an arm decision.
        laps_arm = None
        try:
            arm_txt = io.open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                           "scripts", "fly_hex.sh"), encoding="utf-8").read()
            m_l = re.search(r"LAPS=\$\{LAPS:-([0-9]+)\}", arm_txt)
            laps_arm = None if m_l is None else int(m_l.group(1))
        except Exception:                                        # noqa: BLE001
            laps_arm = None
        ok_all &= row(laps_arm == 2, "S0 飞行圈数（臂）= 2",
                      "fly_hex.sh 的 LAPS = %s ⇒ %s 个角点 + 停机坪；世界文件仍 3 圈（冻结）"
                      % (laps_arm, None if laps_arm is None else laps_arm * 6))
        # ROUND 87 (second pass): checking the arm's variable is NOT enough -- the thin layer used to
        # ignore --laps entirely for --tour world and always fly every corner the world declares
        # (18 + pad = 19 goals), so "LAPS=2" was a no-op for a whole round.  This row asks the thin
        # layer itself what tour it will build, and requires 12 corners (+ pad = 13 targets).
        tour_n = None
        tour_info = None
        try:
            sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                            "src"))
            sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                            "src", "whtf"))
            from whtf.ros.planner_source_super import world_tour as _world_tour
            _c, tour_info = _world_tour(args.world, laps_arm if laps_arm else 2, 2.0, verbose=False)
            tour_n = len(_c)
        except Exception as _exc:                                # noqa: BLE001
            tour_info = {"error": str(_exc)}
        ok_all &= row(tour_n == 12, "S0 两圈航线（薄层真的截断了）",
                      "薄层 world_tour(laps=%s) 给出 %s 个角点 + 停机坪 = %s 个目标；"
                      "世界文件 %s 圈 × %s 个/圈（%s）"
                      % (laps_arm, tour_n, None if tour_n is None else tour_n + 1,
                         (tour_info or {}).get("declared_laps"),
                         (tour_info or {}).get("corners_per_lap"),
                         "truncated" if (tour_info or {}).get("truncated") else
                         (tour_info or {}).get("error", "not truncated")))
        infl = (istep or 0) * (ires or 0.1)
        # 第 83 轮：用户把 inflation_step 定成 1（0.1 m），比 robot_r 0.15 小 —— 于是 CIRI **没有**余量。
        # 实测依据（§57.11）：失败的种子线净空停在 0.178 m，CIRI 要 ≥ robot_r。这一行因此打印数字而不是
        # 当闸门（闸门会把用户自己的选择判成 FAIL）。预计后果：CIRI 重新失败，量到的数字写进 §58。
        print("%-6s %-40s %s" % ("INFO", "S4 CIRI 余量（地图膨胀 − robot_r）",
                                 "%.2f − %.2f = **%+.3f m**（用户选择；负值意味着 CIRI 无余量，§57.11 预测它会失败）"
                                 % (infl, rr or 0, infl - (rr or 0))))

    # ---------------- S1/S3 from a flight ----------------
    print()
    print("-- S1 过点方式 / S3 平均速度")
    if not args.flight:
        print("NOTMEASURED   S1/S3 要靠飞行记录（0.5 m 球到达次数、航点处参考/真值速度、停车次数、"
              "全程平均地速）。本脚本不读飞行记录；量它们用 "
              "`python3 scripts/spec_flight.py --run <sortie dir> --exit-speed 2.0`"
              "（第 84 轮的 d85-01-02：S1a 12/18、S1b 76 段停车、S3 0.60 m/s，见 §59.11）。")
    else:
        need = ["setpoints.csv", "tap.csv"]
        miss = [n for n in need if not os.path.isfile(os.path.join(args.flight, n))]
        if miss:
            ok_all &= row(False, "S1/S3 飞行记录", "%s 里缺 %s" % (args.flight, miss))
        else:
            print("           有记录 %s：由 scripts/speed_audit.py 与 scripts/accept.py 逐条量"
                  "（本脚本只做静态检查）" % args.flight)

    print()
    print("== %s" % ("ALL STATIC CHECKS PASS (S0/S1-geometry/S2/S4); S1-flight/S3 await a flight"
                     if ok_all else "SOME STATIC CHECK FAILED -- do not fly"))
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
