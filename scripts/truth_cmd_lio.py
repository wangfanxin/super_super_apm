#!/usr/bin/env python3
"""truth_cmd_lio.py -- four streams on ONE clock, so "who diverged" is answerable.

At 86 s of missionA-kx1-01-01 the FCU flew into the ground ("SIM Hit ground at 1.36 m/s"), the EKF
raised a variance failsafe and the flight ended at the geofence.  The question that decides the fix
is which stream moved first:

    truth   the SITL's own state            (tap.csv, sim clock)
    lio     what Super-LIO feeds the FCU     (setpoints.csv `lio_*`, W)
    fcu     the FCU's EKF estimate           (setpoints.csv `fcu_*`, W)
    cmd     the thin layer's reference       (setpoints.csv `cmd_*`, W)

`setpoints.csv` carries `t_sim_us`, and `tap.csv` carries `t_us` on the same SITL boot clock, so the
truth can be joined sample-exactly (no wall-clock fitting).  Truth is NED, so W = (e, n, -d).

    python3 scripts/truth_cmd_lio.py /home/fanxin/lio_avoid/logs/missionA-kx1-01-01 --t0 70 --t1 101
"""
import argparse
import csv
import os

import numpy as np


def load_tap(path):
    t, p = [], []
    for r in csv.DictReader(open(path)):
        t.append(int(r["t_us"]) * 1e-6)
        p.append((float(r["pe"]), float(r["pn"]), -float(r["pd"])))
    return np.array(t), np.array(p)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--t0", type=float, default=-1e9)
    ap.add_argument("--t1", type=float, default=1e9)
    ap.add_argument("--step", type=float, default=2.0)
    a = ap.parse_args()
    d = a.run
    sp = list(csv.DictReader(open(os.path.join(d, "setpoints.csv"))))
    ts = np.array([float(r["t_sim_us"]) * 1e-6 for r in sp])
    cmd = np.array([[float(r["cmd_" + c]) for c in "xyz"] for r in sp])
    fcu = np.array([[float(r["fcu_" + c]) for c in "xyz"] for r in sp])
    lio = np.array([[float(r["lio_" + c]) for c in "xyz"] for r in sp])
    t_tap, ptruth = load_tap(os.path.join(d, "tap.csv"))
    order = np.argsort(t_tap)
    t_tap, ptruth = t_tap[order], ptruth[order]

    print("== %s: setpoints %.1f..%.1f s (sim), tap %.1f..%.1f s" % (os.path.basename(d), ts[0], ts[-1],
                                                                     t_tap[0], t_tap[-1]))
    print("   %6s | %-22s | %-22s | %-22s | %-22s | %6s %6s %6s"
          % ("t_sim", "truth (W)", "lio (W)", "fcu (W)", "cmd (W)", "|l-T|", "|f-T|", "|f-c|"))
    t0 = ts[0] if a.t0 < -1e8 else a.t0
    t1 = ts[-1] if a.t1 > 1e8 else a.t1
    t = t0
    while t <= t1:
        i = int(np.argmin(np.abs(ts - t)))
        j = int(np.argmin(np.abs(t_tap - ts[i])))
        if abs(t_tap[j] - ts[i]) > 0.05:
            t += a.step
            continue
        T, L, F, C = ptruth[j], lio[i], fcu[i], cmd[i]
        print("   %6.1f | (%6.2f %6.2f %5.2f) | (%6.2f %6.2f %5.2f) | (%6.2f %6.2f %5.2f) | "
              "(%6.2f %6.2f %5.2f) | %6.2f %6.2f %6.2f"
              % (ts[i], T[0], T[1], T[2], L[0], L[1], L[2], F[0], F[1], F[2], C[0], C[1], C[2],
                 np.linalg.norm(L - T), np.linalg.norm(F - T), np.linalg.norm(F - C)))
        t += a.step

    # headline: when did each stream stop agreeing with the truth?
    L = np.stack([lio[np.argmin(np.abs(ts - tt))] for tt in ts], axis=0)
    jj = np.array([int(np.argmin(np.abs(t_tap - tt))) for tt in ts])
    TT = ptruth[jj]
    for name, S in (("lio", lio), ("fcu", fcu), ("cmd", cmd)):
        e = np.linalg.norm(S - TT, axis=1)
        first = np.where(e > 1.0)[0]
        print("   |%s - truth|: p50 %.2f p90 %.2f max %.2f m%s"
              % (name, np.percentile(e, 50), np.percentile(e, 90), e.max(),
                 ("; first >1 m at t_sim %.1f s" % ts[first[0]]) if len(first) else ""))


if __name__ == "__main__":
    main()
