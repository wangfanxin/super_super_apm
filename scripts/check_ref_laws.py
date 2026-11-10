#!/usr/bin/env python3
"""check_ref_laws.py -- the two governor laws on a plan that behaves like SUPER's:

  * a hexagon of R = 10 m,
  * the plan's own speed DIPS near each corner (MINCO slows and turns there -- measured on the flight:
    the plan's speed p1 was 0.34 m/s against a p50 of 1.33),
  * the direction rotates over a finite turn-in distance.

Mode 1 (chord pursuit) is expected to stop when the plan's projection closes faster than the reference
can follow; mode 2 (arc-length sync) should keep moving because its speed is the plan's own speed plus
a correction, and it can only reach 0 if the PLAN does.

    python3 scripts/check_ref_laws.py
"""
import numpy as np

DT = 0.02
V = 1.33
R = 10.0
V_CORNER = 0.40          # what the plan slows to at a corner
DIP_SIGMA = 1.2          # metres over which the dip applies
CORNER = np.array([[R * np.cos(np.deg2rad(60 * k)), R * np.sin(np.deg2rad(60 * k))] for k in range(6)])
PATH = np.vstack([CORNER, CORNER[:1]])

# walk the hexagon at the LOCAL plan speed: slow near the corners, full speed mid-leg
pts, dirs, vplan = [], [], []
for k in range(len(PATH) - 1):
    a, b = PATH[k], PATH[k + 1]
    seg = np.linalg.norm(b - a)
    u = (b - a) / seg
    d = 0.0
    while d < seg:
        # distance to the nearest corner along the whole ring
        dc = min(d, seg - d)
        v = V_CORNER + (V - V_CORNER) * (1.0 - np.exp(-(dc / DIP_SIGMA) ** 2))
        pts.append(a + u * d)
        dirs.append(u)
        vplan.append(v)
        d += v * DT
pts = np.array(pts)
dirs = np.array(dirs)
vplan = np.array(vplan)


def run(mode, kp=12.0, ks=4.0, acc=8.0, vmax=6.0):
    p_ref = pts[0].copy()
    v_prev = np.zeros(2)
    s_plan = s_ref = 0.0
    out = []
    for i in range(1, len(pts)):
        sp = vplan[i]
        u = dirs[i]
        s_plan += sp * DT
        if mode == 1:
            d = pts[i] - p_ref
            spd = max(0.0, min(vmax, kp * float(d @ u)))
            vg = u * spd
        else:
            err = s_plan - s_ref
            # the shipped law: never more than 0.5 m/s faster than the plan itself
            v_cap = min(vmax, sp + 0.5)
            spd = max(0.0, min(v_cap, sp + ks * err))
            vg = u * spd
        n = np.linalg.norm(vg)
        if n > vmax:
            vg *= vmax / n
        dv = vg - v_prev
        nd = np.linalg.norm(dv)
        cap = acc * DT
        if nd > cap and nd > 1e-9:
            vg = v_prev + dv / nd * cap
        v_prev = vg.copy()
        p_ref = p_ref + vg * DT
        s_ref += float(np.linalg.norm(vg)) * DT
        d = pts[i] - p_ref
        out.append((float(np.linalg.norm(vg)), float(d @ u), float(np.linalg.norm(d))))
    return np.array(out)


print("plan: hexagon R=10 m, speed %.2f m/s mid-leg and %.2f m/s at the corners (%.1f s), %d ticks"
      % (V, V_CORNER, len(pts) * DT, len(pts)))
for mode, name in ((1, "mode 1: chord pursuit (old)"), (2, "mode 2: arc-length sync (new)")):
    o = run(mode)
    spd, along, dist = o[:, 0], o[:, 1], o[:, 2]
    stops = int((spd < 0.2).sum())
    ep = sum(1 for i in range(1, len(spd)) if spd[i] < 0.2 and spd[i - 1] >= 0.2)
    print("\n%s" % name)
    print("   reference speed  p1 %.2f  p50 %.2f  max %.2f m/s"
          % (np.percentile(spd, 1), np.median(spd), spd.max()))
    print("   below 0.20 m/s : %5d ticks (%.1f %%) in %d episodes"
          % (stops, 100.0 * stops / len(spd), ep))
    print("   along-track lag p50 %+.2f  p90(|e|) %.2f m ; |p_ref-plan| p90 %.2f m"
          % (np.median(along), np.percentile(np.abs(along), 90), np.percentile(dist, 90)))
