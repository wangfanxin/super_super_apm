#!/usr/bin/env python3
"""pscd_window.py -- the FCU's own vertical loop, second by second, over an arbitrary window.

`vertical_forensics.py` prints its 1 Hz table around the TRUTH PEAK, which on a runaway sortie is
the last (post-crash) sample.  This script prints the same columns over a window the caller picks,
so the loop can be read where the flight actually went wrong.

Columns: TPD = target z (down+, so -TPD = up), PD = actual z, DPD = desired z, DAD = desired
vertical accel, AD = actual vertical accel (all from PSCD/PSCN), plus our own cmd_z / fcu_z.

    python3 scripts/pscd_window.py v3c5-01-01 --t0 24 --t1 40
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vertical_forensics as vf  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tag")
    ap.add_argument("--t0", type=float, default=20.0)
    ap.add_argument("--t1", type=float, default=45.0)
    a = ap.parse_args()
    d = os.path.join(vf.MISSION, "missionA-" + a.tag)
    st, cz, fz = vf.load_setpoints(d)
    tt, tz = vf.load_truth(d)
    pscd = vf.load_pscd(d)
    if pscd is None:
        print("no PSCD in the BIN log")
        return 1
    i_to = int(np.argmax(tz > 1.0))
    off = float(st[int(np.argmax(cz > 1.0))] - tt[i_to]) if np.any(cz > 1.0) else st[0] - tt[i_to]
    print("== %s   t %.1f .. %.1f s  (truth clock)" % (a.tag, a.t0, a.t1))
    print("   %6s %8s %8s %8s %8s %8s %8s %8s"
          % ("t", "truth", "cmd_z", "fcu_z", "TPD", "PD", "DAD", "AD"))
    nxt = a.t0
    for i in range(len(tt)):
        t = float(tt[i])
        if t < a.t0 or t > a.t1 or t < nxt:
            continue
        nxt = t + 1.0
        j = int(np.argmin(np.abs((st - off) - t)))
        m = int(np.argmin(np.abs(pscd[:, 0] - t)))
        row = pscd[m, 1:7]
        print("   %6.2f %8.2f %8.2f %8.2f %8.2f %8.2f %8.2f %8.2f"
              % (t, tz[i], cz[j], fz[j], row[1], row[0], row[3], row[5]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
