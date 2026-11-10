#!/usr/bin/env python3
"""tap_jumps.py -- did the truth stream JUMP, and where?

Two recent sorties were rejected with `truth span 249-268 m vs FCU 11-15 m` and a first truth sample
465 m up, which the tap's own CSV does not show at its head.  A physical flight cannot do that, so
either the tap stream carries a discontinuity (a glitch) or the vehicle really went there. This finds
the largest single-sample jump and the envelope, so the two can be told apart.

    python3 scripts/tap_jumps.py kx1-02-02 kwC-02-01 kx1-01-02
"""
import csv
import os
import sys

import numpy as np

ROOT = "/home/fanxin/lio_avoid/logs"


def one(tag, stride=1):
    p = os.path.join(ROOT, "missionA-" + tag, "tap.csv")
    if not os.path.exists(p):
        print("%-12s <no tap.csv>" % tag)
        return
    t, pn, pe, pd = [], [], [], []
    rdr = csv.DictReader(open(p))
    for i, r in enumerate(rdr):
        if i % stride:
            continue
        t.append(int(r["t_us"]) * 1e-6)
        pn.append(float(r["pn"]))
        pe.append(float(r["pe"]))
        pd.append(float(r["pd"]))
    t = np.array(t); p = np.stack([np.array(pn), np.array(pe), -np.array(pd)], axis=1)
    v = float(np.median(np.diff(t)))
    jump = np.linalg.norm(np.diff(p, axis=0), axis=1)
    k = int(np.argmax(jump))
    print("== %s: %d samples, sim %.1f s, median dt %.4f s" % (tag, len(t), t[-1] - t[0], v))
    print("   envelope  E[%7.2f %7.2f]  N[%7.2f %7.2f]  U[%7.2f %7.2f]"
          % (p[:, 0].min(), p[:, 0].max(), p[:, 1].min(), p[:, 1].max(), p[:, 2].min(), p[:, 2].max()))
    print("   first sample (E,N,U) = (%7.2f %7.2f %7.2f)   last = (%7.2f %7.2f %7.2f)"
          % (p[0][0], p[0][1], p[0][2], p[-1][0], p[-1][1], p[-1][2]))
    print("   largest single step %.2f m at sim t=%.2f s (dt %.4f s -> implied %.1f m/s)"
          % (jump[k], t[k], t[k + 1] - t[k], jump[k] / max(1e-9, t[k + 1] - t[k])))
    big = np.where(jump > 1.0)[0]
    print("   steps > 1.0 m: %d%s" % (len(big), ("  first at t=%.2f" % t[big[0]]) if len(big) else ""))
    for b in big[:6]:
        print("      t=%8.3f  stepped %7.2f m  (from %7.2f %7.2f %7.2f)"
              % (t[b], jump[b], p[b][0], p[b][1], p[b][2]))


if __name__ == "__main__":
    for a in sys.argv[1:]:
        one(a)
        print()
