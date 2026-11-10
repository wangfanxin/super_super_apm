#!/usr/bin/env python3
"""why_reversal.py -- WHY does the vehicle go backwards BETWEEN waypoints?

The owner's complaint, verbatim: "在航点中间有往复现象".  `backtrack.py` already proves the shape on the
truth (17 episodes, 9.5 m retraced on `kv1-02-01`) and proves the REFERENCE never retreats
(backward -0.0 m).  So the reversal is the VEHICLE moving against the reference's own direction.  This
script asks the next question -- what is the reference doing when that happens -- on ONE clock, using
`setpoints.csv`, which carries the published command and the FCU's own estimate side by side:

  A) the published POSITION stream's speed, `|d(cmd)/dt|`, against the published VELOCITY field.  A
     position stream that moves at 2-3 m/s while the velocity field is capped at 1.50 is the exact
     inconsistency the FCU's position loop reacts to (it chases a target that is farther than the
     velocity says), and it is measurable per tick.
  B) the ALONG-TRACK error between the command and the vehicle, in the reference's own direction
     (`along = (p_fcu - p_cmd) . u`, u = unit(cmd velocity)).  along > 0 = the vehicle is AHEAD of the
     reference, which is the state that ends in "it has to come back".
  C) the REVERSALS, counted the same way as `backtrack.py` but on this clock: a tick whose vehicle step
     has a negative component along the reference's direction.
  D) for each reversal cluster, a 6-column trace so the mechanism is readable: t, reference speed,
     |cmd step|/dt, along-track error, vehicle speed, whether the reference was fresh.

    python3 scripts/why_reversal.py --tag kv1-02-01
"""
import argparse
import csv
import os

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--tag", required=True)
ap.add_argument("--root", default="/home/fanxin/lio_avoid/logs")
ap.add_argument("--out-dir", default=None)
a = ap.parse_args()
run = os.path.join(a.root, "missionA-" + a.tag)
rows = list(csv.DictReader(open(os.path.join(run, "setpoints.csv"))))


def col(n):
    return np.array([float(r[n]) for r in rows])


t = col("t_sim_us") * 1e-6
# ★ WHICH CLOCK?  `t_wall`, NOT `t_sim_us` ★
#   MEASURED on kv1-02-01: `t_wall` is a clean 50 Hz (dt p50 0.0200, p99 0.0212), while `t_sim_us` is
#   QUANTISED TO 5 ms and carries 1892 repeated stamps out of 5982 (`dt_sim` p10 = 0.000 s,
#   p50 = 0.035 s).  Dividing a position step by that denominator invents rates of 2-9 m/s that never
#   happened -- the first version of this script did exactly that and "found" 652 ticks above 1.60 m/s.
#   The recorder is a 50 Hz ROS timer, so its own wall clock is the honest denominator; the position
#   and velocity fields in the same row are two numbers the plan box published in the SAME message.
tw = col("t_wall")
cmd = np.stack([col("cmd_x"), col("cmd_y"), col("cmd_z")], 1)
vel = np.stack([col("vel_x"), col("vel_y"), col("vel_z")], 1)
fcu = np.stack([col("fcu_x"), col("fcu_y"), col("fcu_z")], 1)
dt = np.diff(tw)
dt[dt <= 0] = np.nan
print("== %s: %d ticks, dt(from t_wall) p50 %.4f s (p10 %.4f p90 %.4f)"
      % (a.tag, len(t), np.nanmedian(dt), np.nanpercentile(dt, 10), np.nanpercentile(dt, 90)))
print("   (t_sim_us is quantised to 5 ms with %d repeated stamps -- it CANNOT be a rate denominator)"
      % int(np.sum(np.diff(t) == 0)))

# ---------------------------------------------------------------- A) the position stream vs the velocity field
dp = np.linalg.norm(np.diff(cmd, axis=0), axis=1)
pos_spd = dp / dt
vpub = np.linalg.norm(vel[:-1, :2], axis=1)
moving = vpub > 0.05
print("\nA) the published POSITION stream vs the published VELOCITY field")
print("   |d(cmd)/dt|      : p50 %.2f  p90 %.2f  p99 %.2f  max %.2f m/s"
      % (np.nanpercentile(pos_spd, 50), np.nanpercentile(pos_spd, 90),
         np.nanpercentile(pos_spd, 99), np.nanmax(pos_spd)))
print("   |vel field|      : p50 %.2f  p90 %.2f  p99 %.2f  max %.2f m/s"
      % (np.percentile(vpub, 50), np.percentile(vpub, 90), np.percentile(vpub, 99), vpub.max()))
excess = pos_spd - 1.50
bad = np.where(np.isfinite(excess) & (excess > 0.10))[0]
print("   ticks whose POSITION moves >1.60 m/s (the velocity field's own cap is 1.50): %d of %d (%.2f %%)"
      % (len(bad), len(pos_spd), 100.0 * len(bad) / max(1, len(pos_spd))))
if len(bad):
    print("   the ten worst, with the per-axis step that produced them:")
    print("     %8s %8s %8s %8s %8s %8s" % ("t", "|dp|/dt", "dx", "dy", "dz", "|v| field"))
    for k in bad[np.argsort(-pos_spd[bad])][:10]:
        d3 = cmd[k + 1] - cmd[k]
        print("     %8.2f %8.2f %8.3f %8.3f %8.3f %8.2f"
              % (t[k], pos_spd[k], d3[0], d3[1], d3[2], vpub[k]))

# ---------------------------------------------------------------- B) along-track error
u = vel[:-1].copy()
n = np.linalg.norm(u, axis=1)
ok = n > 0.30
uh = np.zeros_like(u)
uh[ok] = u[ok] / n[ok, None]
along = np.full(len(t) - 1, np.nan)
along[ok] = np.einsum("ij,ij->i", fcu[:-1][ok] - cmd[:-1][ok], uh[ok])
au = along[np.isfinite(along)]
print("\nB) ALONG-track error (vehicle - command, in the reference's own direction)")
print("   p5 %+.2f  p50 %+.2f  p95 %+.2f  max %+.2f m ; vehicle AHEAD by >0.30 m in %.1f %% of the moving time"
      % (np.percentile(au, 5), np.percentile(au, 50), np.percentile(au, 95), au.max(),
         100.0 * np.mean(au > 0.30)))
lat = np.full(len(t) - 1, np.nan)
rel = fcu[:-1] - cmd[:-1]
lat[ok] = np.linalg.norm(rel[ok] - np.einsum("ij,ij->i", rel[ok], uh[ok])[:, None] * uh[ok], axis=1)
la = lat[np.isfinite(lat)]
print("   LATERAL error (perpendicular to the reference): p50 %.2f  p90 %.2f  p99 %.2f m"
      % (np.percentile(la, 50), np.percentile(la, 90), np.percentile(la, 99)))

# ---------------------------------------------------------------- C) reversals on this clock
step = np.diff(fcu, axis=0)
back = np.full(len(t) - 1, np.nan)
back[ok] = np.einsum("ij,ij->i", step[ok], uh[ok])
# an episode = a run of consecutive ticks whose step is negative along the reference direction
neg = np.isfinite(back) & (back < -1e-4)
run_id = np.cumsum(np.concatenate([[0], np.diff(neg.astype(int)) != 0]))
episodes = []
for rid in np.unique(run_id):
    m = neg & (run_id == rid)
    if m.sum() >= 3:
        dist = float(-np.nansum(back[m]))
        if dist >= 0.15:
            episodes.append((float(t[np.argmax(m)]), int(m.sum()), dist))
print("\nC) REVERSALS of the VEHICLE against the reference's direction: %d episodes, %.2f m retraced in total"
      % (len(episodes), sum(e[2] for e in episodes)))
print("   (backtrack.py measures the same thing on the truth tap and gets 17 episodes / 9.5 m -- this")
print("    count is on the FCU's own estimate, one clock, so the two numbers should agree in shape)")

# ---------------------------------------------------------------- D) traces
print("\nD) the five longest reversals, with the reference's own numbers over the episode")
print("   %8s %6s %7s %8s %8s %8s %9s %9s" %
      ("t", "ticks", "retr m", "ref p50", "ref min", "|dp|/dt", "along p50", "veh p50"))
for (t0, m, dist) in sorted(episodes, key=lambda e: -e[2])[:5]:
    i0 = int(np.argmin(np.abs(t - t0)))
    i1 = min(len(t) - 1, i0 + m + 5)
    sl = slice(i0, i1)
    print("   %8.2f %6d %7.2f %8.2f %8.2f %8.2f %9.2f %9.2f"
          % (t0, m, dist, np.nanmedian(vpub[sl]), np.nanmin(vpub[sl]),
             np.nanmedian(pos_spd[sl]), np.nanmedian(along[sl]),
             np.nanmedian(np.linalg.norm(np.diff(fcu, axis=0)[sl], axis=1) / dt[sl])))
