#!/usr/bin/env python3
"""alt_divergence.py -- who drifted vertically: the SITL model, the LIO, or the FCU's estimate?

Round 87 (3 m/s, 2 laps) ended both attempts with the same signature: the model descends to the
ground mid-mission (`SIM Hit ground`), then the EKF failsafe, then a 100-300 m runaway.  The
question that decides what to change is WHICH of the three vertical channels moved first:

    truth   tap.csv          -pd            (the SITL model, metres up)
    lio     lio.csv          z              (Super-LIO, L frame -- compared as a DELTA, so the
                                             L->W yaw/translation does not have to be right)
    fcu     setpoints.csv    fcu_z          (the FCU's own EKF estimate, already W)
    cmd     setpoints.csv    cmd_z          (what the plan asked for)

Because the L frame differs from W by a rotation and an offset, the LIO column is printed as
`z(t) - z(t0)` against `truth(t) - truth(t0)` over the same window: a real vertical drift shows up
as the two deltas parting company, while a constant frame offset cancels.

    python3 scripts/alt_divergence.py v3l2-01-02 [--bin 2.0]
"""
import argparse
import csv
import os

import numpy as np

MISSION = "/home/fanxin/lio_avoid/logs"


def _f(path, cols):
    out = {c: [] for c in cols}
    with open(path, newline="") as fh:
        rd = csv.DictReader(fh)
        for r in rd:
            try:
                for c in cols:
                    out[c].append(float(r[c]))
            except (KeyError, ValueError):
                continue
    return {c: np.asarray(v, dtype=float) for c, v in out.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tag")
    ap.add_argument("--bin", type=float, default=2.0)
    ap.add_argument("--t-max", type=float, default=0.0, help="stop at this truth time (0 = to the end)")
    a = ap.parse_args()
    d = os.path.join(MISSION, "missionA-" + a.tag)

    tap = _f(os.path.join(d, "tap.csv"), ["t_us", "pd"])
    t = (tap["t_us"] - tap["t_us"][0]) * 1e-6
    tz = -tap["pd"]
    lio = _f(os.path.join(d, "lio.csv"), ["t_us", "z"])
    lt = (lio["t_us"] - lio["t_us"][0]) * 1e-6
    lz = lio["z"]
    sp = _f(os.path.join(d, "setpoints.csv"), ["t_sim_us", "cmd_z", "fcu_z"])
    st = (sp["t_sim_us"] - sp["t_sim_us"][0]) * 1e-6
    cz, fz = sp["cmd_z"], sp["fcu_z"]

    # the LIO recording starts with the LIO node, the tap with the SITL; align on "both flying" by
    # using each series' own t0 only as an index, then pair the tables by the TRUTH clock.
    t_end = a.t_max if a.t_max > 0 else float(min(t[-1], lt[-1], st[-1]))
    print("== %s   window 0 .. %.1f s   (bin %.1f s)" % (a.tag, t_end, a.bin))
    print("   %6s %8s %8s %8s %8s   %8s %8s"
          % ("t", "truth", "d_truth", "d_lio", "fcu", "d_fcu", "cmd"))
    i0 = int(np.argmin(np.abs(t - 1.0)))
    tz0 = tz[i0]
    l0 = float(np.interp(1.0, lt, lz))
    f0 = float(np.interp(1.0, st, fz))
    nxt = 0.0
    for i in range(len(t)):
        if t[i] < nxt or t[i] > t_end:
            continue
        nxt = t[i] + a.bin
        li = float(np.interp(t[i], lt, lz))
        fi = float(np.interp(t[i], st, fz))
        ci = float(np.interp(t[i], st, cz))
        print("   %6.1f %8.2f %8.2f %8.2f %8.2f   %8.2f %8.2f"
              % (t[i], tz[i], tz[i] - tz0, li - l0, fi, fi - f0, ci))

    # the summary the decision hangs on
    m = (t >= 2.0) & (t <= t_end)
    d_tr = np.interp(t[m], lt, lz) - l0 - (tz[m] - tz0)
    print()
    print("   LIO vertical delta minus truth vertical delta over the window: "
          "p50 %+.2f  p90 %+.2f  max %+.2f m" % (np.percentile(d_tr, 50),
                                                 np.percentile(d_tr, 90), d_tr.max()))
    print("   truth z: min %.2f  max %.2f m      FCU z: min %.2f  max %.2f m"
          % (tz[m].min(), tz[m].max(), np.interp(t[m], st, fz).min(), np.interp(t[m], st, fz).max()))
    print("   (a LIO > truth meaning 'the LIO kept climbing while the model sank' means the EKF is")
    print("    holding altitude on a vertical channel that no longer reflects the plant: the model")
    print("    then keeps descending until it hits the ground and the EKF failsafes.)")


if __name__ == "__main__":
    main()
