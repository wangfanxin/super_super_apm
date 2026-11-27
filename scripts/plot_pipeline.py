#!/usr/bin/env python3
"""plot_pipeline.py -- the FINAL target data pipeline diagram (one page, two-row snake layout).

The figure is the agreed end-state after the Nav2-style thin-layer rework:
  sensors/SITL -> Super-LIO -> ROG map -> Route Manager -> SUPER -> smoother
      -> follower(RPP) -> velocity smoother -> command shaper -> bridge -> FCU EKF/PSC -> SITL (loop)
plus supervisor/safety, and the record -> accept/quality/figures -> gate chain every sortie runs.
Colour code: black = kept from today, blue = new/replaced in this design, green = recording/metrics,
red = safety/supervision.

    python3 scripts/plot_pipeline.py --out docs/evidence/final/pipeline-target.png
"""
from __future__ import annotations

import argparse
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Rectangle

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:                       # registers Microsoft YaHei from the Windows font dir
    import _figfont
    _CJK = _figfont.ensure_cjk_font(verbose=False)
except Exception:          # noqa: BLE001
    _CJK = "DejaVu Sans"
plt.rcParams["font.sans-serif"] = [_CJK, "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

NEW = "#e3f2fd"; NEW_E = "#1565c0"
KEEP = "#ffffff"; KEEP_E = "#37474f"
REC = "#e8f5e9"; REC_E = "#2e7d32"
SAFE = "#ffebee"; SAFE_E = "#b71c1c"
FONT = _CJK


def box(ax, x, y, w, h, text, fc=KEEP, ec=KEEP_E, fs=8.6, lw=1.2, bold=False, z=3):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.22,rounding_size=0.7",
                                linewidth=lw, edgecolor=ec, facecolor=fc, zorder=z))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=fs, family=FONT,
            zorder=z + 1, fontweight="bold" if bold else "normal", linespacing=1.28)


def arrow(ax, p0, p1, label="", color="#455a64", lw=1.5, rad=0.0, fs=7.4, ls="-", lx=0.0, ly=0.0):
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle="-|>", mutation_scale=12, linewidth=lw,
                                 color=color, linestyle=ls,
                                 connectionstyle="arc3,rad=%.2f" % rad,
                                 shrinkA=2, shrinkB=2, zorder=1))
    if label:
        ax.text((p0[0] + p1[0]) / 2 + lx, (p0[1] + p1[1]) / 2 + ly, label, ha="center", va="center",
                fontsize=fs, color=color, family=FONT, zorder=5,
                bbox=dict(boxstyle="round,pad=0.14", fc="white", ec="none", alpha=0.9))


def frame(ax, x, y, w, h, title, color="#78909c"):
    ax.add_patch(Rectangle((x, y), w, h, fill=False, ec=color, lw=1.0, ls=(0, (5, 3)), zorder=0))
    ax.text(x + 0.4, y + h + 0.25, title, fontsize=10, color=color, family=FONT,
            fontweight="bold", va="bottom", zorder=1)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="docs/evidence/final/pipeline-target.png")
    a = ap.parse_args()

    fig, ax = plt.subplots(figsize=(23.5, 13.2))
    ax.set_xlim(0, 100); ax.set_ylim(0, 64); ax.axis("off")

    ax.text(50, 62.8, "super_super_apm 最终目标数据管线（Nav2 式薄层重做后）",
            ha="center", va="center", fontsize=17.5, family=FONT, fontweight="bold")
    ax.text(50, 60.9, "黑=现有保留 · 蓝=设计新增/替换 · 绿=记录/评测 · 红=安全/监督     "
                      "坐标：L=规划系，W=世界(E,N,U)，NED=飞控系；速率标在箭头上",
            ha="center", va="center", fontsize=9.6, family=FONT, color="#455a64")

    # ================= 第一行：感知 → 定位/地图 → 目标/规划（左→右） =================
    frame(ax, 1, 40.5, 31.5, 17.5, "① 仿真/传感器（左） + ② 定位/地图")
    box(ax, 1.8, 50.0, 13.2, 6.6,
        "ArduPilot SITL\nArduCopter 4.7.1\n1 kg / 250 mm 机体模型", fs=8.2)
    box(ax, 1.8, 42.0, 13.2, 6.6,
        "LiDAR + IMU 仿真\nscan 标称 10 Hz（近期实测 9.4–9.5 Hz）\nIMU 标称 200 Hz（实测 ~181 Hz）\nraycast 由 SITL 位姿驱动", fs=8.2)
    box(ax, 17.2, 50.0, 13.6, 6.6, "Super-LIO\nLiDAR-惯性里程计\nL 系 pose/vel（~50 Hz）", fs=8.4)
    box(ax, 17.2, 42.0, 13.6, 6.6,
        "ROG-Map 栅格 + 膨胀\nOCCUPIED / KNOWN_FREE\n3×3 m 角点 keepout", fs=8.4)
    box(ax, 1.8, 34.0, 13.2, 5.2, "真值 tap（1 kHz）\n评测唯一真值 pn/pe/pd", fc=REC, ec=REC_E, fs=8.2)

    frame(ax, 35.5, 40.5, 37.5, 17.5, "③ 目标 · 规划 · 跟随（Nav2 式薄层重做区）", color=NEW_E)
    box(ax, 36.4, 50.0, 17.6, 6.6,
        "Route Manager（新）\n路线 poses + 弧长进度\n中间点 pass-by 记账 0.5–0.7 m（不重规划）\n"
        "GoalChecker 仅终点 xy≤0.3–0.5 m\nProgressChecker 10 s/0.5 m → Recovery", fc=NEW, ec=NEW_E, fs=7.5)
    box(ax, 36.4, 42.0, 17.6, 6.6,
        "SUPER planner\n多航点 / through-poses\nA* + MINCO 走廊优化\n"
        "固定速率重规划『剩余路径』(1–3 Hz)\n路径有效则不重规划", fc=NEW, ec=NEW_E, fs=7.6)
    box(ax, 55.9, 50.0, 16.1, 6.6, "Path smoother（可选）\n角点圆滑（S-G / Simple）\n保持走廊与障碍约束\n（也可并入 SUPER）", fc=NEW, ec=NEW_E, fs=7.8)
    box(ax, 55.9, 42.0, 16.1, 6.6, "Follower（备选：仅当规划不能连续时）\n前视 L=clamp(k·v,Lmin,Lmax)\n曲率限速 v≤√(a_lat·R)\n"
                                     "终点减速 · 下限 v_floor（不替代飞控 PID）\n"
                                     "（规划连续时：直接发 MINCO 的 PVA，不需要它）", fc=NEW, ec=NEW_E, fs=7.4)

    # ================= 第二行：指令 → 桥/飞控（右→左） =================
    frame(ax, 1, 15.5, 62, 18.5, "④ 指令整形（右→左）   +   ⑤ 桥 + 飞控（闭环回左）")
    box(ax, 1.8, 20.5, 12.8, 7.2, "PSC → 电机\nFCU 姿态/位置环\nPSC_NE_* 输出", fs=8.4)
    box(ax, 17.8, 20.5, 13.2, 7.2, "FCU EKF\nIMU + 外部导航融合\n(EKF2 ext-nav)", fs=8.4)
    box(ax, 34.2, 20.5, 12.8, 7.2, "NAV gate\nodom age / quality / reset\n不达标 → 拒绝喂 EKF", fs=8.2)
    box(ax, 50.2, 20.5, 12.4, 7.2, "bridge（MAVLink）\n#331 ODOMETRY 48 Hz\n#84 PVA setpoint 50 Hz\n"
                                     "class-76：arm/takeoff/mode/LAND", fs=7.6)
    box(ax, 50.2, 29.0, 12.4, 3.9, "Command shaper\nPVA 100 Hz · L→W\nz 基准 + stale/hold 后备", fc=NEW, ec=NEW_E, fs=7.8)
    box(ax, 34.2, 29.0, 12.8, 3.9, "Velocity smoother（可选/合并层）\n唯一 max_v / max_a / max_jerk\n（替换散落的 slew/ACC 钳位）", fc=NEW, ec=NEW_E, fs=7.8)

    # supervisor
    box(ax, 66.5, 25.0, 15.2, 8.6,
        "supervisor（安全/监督）\narm/takeoff/mode/LAND\n高度门 · geofence\n估计器断流/质量\n"
        "plan 静默 · truth-FCU 不一致\n→ ABORT / emergency", fc=SAFE, ec=SAFE_E, fs=7.8)

    # ================= 第三行：记录 → 评测 → 出图 → 质量门 =================
    frame(ax, 1, 2.0, 88, 11.0, "⑥ 记录 → 评测 → 出图 → 质量门（每趟自动）", color=REC_E)
    box(ax, 2.0, 4.5, 15.5, 6.8,
        "recorder\nsetpoints.csv · wire.csv\nnav.csv · lio.csv · scans.csv\ntap.csv（真值）\n"
        "rosbag2 → derived/traj.csv.cmd.csv", fc=REC, ec=REC_E, fs=7.2)
    box(ax, 19.5, 7.2, 13.0, 4.7, "accept.py（15 行）\nH0–H6 有效性 · W0–W6 任务/跟踪/碰撞", fc=REC, ec=REC_E, fs=7.4)
    box(ax, 19.5, 3.6, 13.0, 3.2, "quality_rows\nQ1–Q4 + P1（规划-执行偏差+尖峰）", fc=REC, ec=REC_E, fs=7.2)
    box(ax, 34.8, 4.5, 10.2, 6.8, "出图\ntopdown 整趟/分圈\nxyz-t · 速度2-范数\n角点 zoom · clearance\nP1 4 联图", fc=REC, ec=REC_E, fs=7.0)
    box(ax, 47.2, 4.2, 10.6, 7.3, "质量门\nH6 clean\nW1b 13/13\nP1 无>1 m 尖峰\nQ 行\n≥4 seed 地图\n速度阶梯 4/6/8", fc="#c8e6c9", ec=REC_E, fs=7.0, lw=2)
    box(ax, 60.0, 4.5, 13.2, 6.8,
        "离线 replay 原型（新）\n录制 CSV → Route+Follower\n先对比 44b/94a（不飞）\n通过后再 frozen → 4 图", fc=NEW, ec=NEW_E, fs=7.4)
    box(ax, 75.5, 4.5, 12.5, 6.8, "下一档实验\nhex16 / hex18_low（同柱距）\n4 → 6 → 8 m/s\n8 m/s 必须达成", fs=7.6)

    # ================= 数据流 =================
    # --- row 1: 感知 → 定位/地图 → 目标/规划/跟随（左→右，带一次折返） ---
    arrow(ax, (15.0, 53.3), (17.2, 53.3), "位姿", fs=7.0)
    arrow(ax, (15.0, 45.3), (17.2, 52.2), "scan+IMU", rad=0.10, fs=7.0)
    arrow(ax, (15.0, 36.6), (17.2, 49.6), "点云+位姿", rad=0.18, fs=7.0)
    arrow(ax, (30.8, 53.3), (36.4, 53.3), "当前位姿/速度", fs=7.0)
    arrow(ax, (30.8, 45.3), (36.4, 45.3), "占据地图", fs=7.0)
    arrow(ax, (36.4 + 8.8, 50.0), (36.4 + 8.8, 48.6), "goals", fs=7.0, lx=7.4)
    ax.text(47.5, 57.5, "goals = 一次穿过剩余多角点（不逐点重规划）", fontsize=7.0, color="#455a64",
            family=FONT, ha="center", va="center",
            bbox=dict(boxstyle="round,pad=0.12", fc="white", ec="none", alpha=0.85))
    arrow(ax, (54.0, 45.3), (55.9, 52.4), "路径/轨迹", rad=-0.12, fs=7.0, lx=1.6, ly=1.6)
    arrow(ax, (64.0, 50.0), (64.0, 48.6), "平滑路径", fs=7.0, lx=8.6)
    arrow(ax, (56.4, 42.0), (47.0, 32.6), "v_cmd", rad=-0.10, fs=7.2)
    # --- row 2: 指令整形 → 桥 → 飞控（右→左） ---
    arrow(ax, (47.0, 31.0), (50.2, 31.0), "限幅后", fs=7.0)
    arrow(ax, (56.4, 29.0), (56.4, 27.8), "PVA 100 Hz", fs=7.0, lx=7.2)
    arrow(ax, (50.2, 24.1), (47.0, 24.1), "ODOMETRY 48 Hz", fs=7.0)
    arrow(ax, (34.2, 24.1), (31.0, 24.1), "gate 后", fs=7.0)
    arrow(ax, (17.8, 24.1), (14.6, 24.1), "状态估计", fs=7.0)
    # --- 闭环：PSC → SITL（沿左边缘绕行） ---
    arrow(ax, (1.8, 24.1), (0.55, 24.1), "", color=NEW_E, lw=2.0)
    ax.add_patch(FancyArrowPatch((0.55, 24.1), (0.55, 53.3), arrowstyle="-", linewidth=2.0,
                                 color=NEW_E, zorder=1))
    arrow(ax, (0.55, 53.3), (1.8, 53.3), "", color=NEW_E, lw=2.0)
    ax.text(8.6, 58.8, "闭环：PSC 电机 → SITL 物理 → 新 scan（10 Hz）", fontsize=8.0,
            color=NEW_E, family=FONT, ha="center", va="center",
            bbox=dict(boxstyle="round,pad=0.18", fc="white", ec="none", alpha=0.9))
    # --- supervisor ---
    arrow(ax, (66.5, 27.0), (62.6, 24.6), "class-76", color=SAFE_E, rad=0.08, fs=7.0, ls="--")
    arrow(ax, (66.5, 31.0), (44.8, 42.4), "ABORT/新目标抢占", color=SAFE_E, rad=0.12, fs=7.2, ls="--")
    # --- recorder taps ---
    arrow(ax, (15.4, 34.0), (15.4, 11.4), "", color=REC_E, rad=0.0, fs=7.0, ls="--")
    ax.text(16.5, 26.5, "tap 1 kHz", rotation=90, fontsize=7.0, color=REC_E, family=FONT,
            ha="center", va="center", bbox=dict(boxstyle="round,pad=0.12", fc="white", ec="none", alpha=0.85))
    arrow(ax, (24.0, 42.0), (15.0, 11.3), "lio / scans", color=REC_E, rad=-0.10, fs=7.0, ls="--")
    arrow(ax, (50.2, 20.5), (17.2, 8.2), "setpoints / wire / nav", color=REC_E, rad=-0.10, fs=7.0, ls="--")
    # --- eval chain ---
    arrow(ax, (17.5, 8.6), (19.5, 9.4), "CSV", fs=6.8, color=REC_E)
    arrow(ax, (17.5, 7.2), (19.5, 5.4), "CSV", fs=6.8, color=REC_E)
    arrow(ax, (32.5, 9.4), (34.8, 9.2), "15 行", fs=6.8, color=REC_E)
    arrow(ax, (32.5, 5.2), (34.8, 6.4), "Q/P 行", fs=6.8, color=REC_E)
    arrow(ax, (45.0, 7.8), (47.2, 7.8), "判据", fs=6.8, color=REC_E)
    arrow(ax, (57.8, 7.8), (60.0, 7.8), "OK/否决", fs=6.8, color=REC_E)
    arrow(ax, (73.2, 7.8), (75.5, 7.8), "验证后上机", fs=6.8, color=NEW_E, ls="--")

    # legend
    ax.text(90.5, 13.2, "图例", fontsize=9.5, family=FONT, fontweight="bold")
    for i, (lab, fc, ec) in enumerate((("现有保留", KEEP, KEEP_E), ("设计新增/替换", NEW, NEW_E),
                                       ("记录/评测", REC, REC_E), ("安全/监督", SAFE, SAFE_E))):
        ax.add_patch(FancyBboxPatch((90.5, 11.4 - i * 1.6), 1.5, 0.8,
                                    boxstyle="round,pad=0.1", fc=fc, ec=ec, lw=1.2))
        ax.text(92.5, 11.8 - i * 1.6, lab, fontsize=8.2, family=FONT, va="center")

    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    fig.savefig(a.out, dpi=150, bbox_inches="tight", facecolor="white")
    print("written:", a.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
