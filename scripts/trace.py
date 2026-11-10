#!/usr/bin/env python3
"""trace.py -- one compact table: t, truth(x,y,z), fcu(x,y,z), cmd(x,y,z), speed, for a sortie.
Reads setpoints.csv (cmd/fcu) and tap.csv (truth), joined on the shared simulated clock."""
import argparse, csv, os
import numpy as np

R = "/home/fanxin/lio_avoid/logs/missionA-%s"

ap = argparse.ArgumentParser()
ap.add_argument("tag")
ap.add_argument("--step", type=float, default=2.0)
ap.add_argument("--t0", type=float, default=0.0)
ap.add_argument("--t1", type=float, default=1e9)
a = ap.parse_args()

sp = os.path.join(R % a.tag, "setpoints.csv")
tp = os.path.join(R % a.tag, "tap.csv")
t, cmd, fcu = [], [], []
for r in csv.DictReader(open(sp)):
    t.append(float(r["t_sim_us"]) * 1e-6)
    cmd.append((float(r["cmd_x"]), float(r["cmd_y"]), float(r["cmd_z"])))
    fcu.append((float(r["fcu_x"]), float(r["fcu_y"]), float(r["fcu_z"])))
t = np.array(t); cmd = np.array(cmd); fcu = np.array(fcu)

tt, pz, px, py = [], [], [], []
for r in csv.DictReader(open(tp)):
    tt.append(float(r["t_us"]) * 1e-6)
    px.append(float(r["pe"])); py.append(float(r["pn"])); pz.append(-float(r["pd"]))
tt = np.array(tt); tr = np.column_stack([px, py, pz])

print("%s: setpoints %.1f..%.1f (%d)  tap %.1f..%.1f (%d)"
      % (a.tag, t[0], t[-1], len(t), tt[0], tt[-1], len(tt)))
print("\n    t    truth(x,y,z)             fcu(x,y,z)              cmd(x,y,z)           |fcu-cmd|  spd")
x = a.t0
while x <= min(a.t1, t[-1]):
    i = int(np.searchsorted(t, x).clip(0, len(t) - 1))
    j = int(np.searchsorted(tt, x).clip(0, len(tt) - 1))
    d = float(np.linalg.norm((fcu[i] - cmd[i])[:2]))
    spd = float(np.linalg.norm((fcu[i] - fcu[max(i - 10, 0)])[:2])) * 5.0
    print("%6.1f  (%6.2f,%6.2f,%5.2f)  (%6.2f,%6.2f,%5.2f)  (%6.2f,%6.2f,%5.2f)  %6.2f  %5.2f"
          % (x - t[0], tr[j, 0], tr[j, 1], tr[j, 2], fcu[i, 0], fcu[i, 1], fcu[i, 2],
             cmd[i, 0], cmd[i, 1], cmd[i, 2], d, spd))
    x += a.step
