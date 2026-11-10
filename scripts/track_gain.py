#!/usr/bin/env python3
"""track_gain.py -- how much of the reference's velocity content does the vehicle actually reproduce?

The owner's complaint is about `vxyz-t` panel 3: the plan's `v_x`/`v_y` and the truth's do not sit on
top of each other.  Two very different faults look like that on a plot:

    a GAIN loss   -- the vehicle reproduces the shape but with smaller amplitude (bandwidth),
    a LAG         -- the same shape shifted in time,
    a STEP source -- the reference jumps and the vehicle takes a while to catch up.

This separates them: for each frequency band it reports the gain (|truth| / |plan|) and the phase lag
from the cross-spectrum of the plan's velocity against the truth's velocity.

    python3 scripts/track_gain.py --tag kh1-02-01
"""
import argparse
import csv
import os

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--tag", default="kh1-02-01")
ap.add_argument("--root", default="/home/fanxin/lio_avoid/logs")
a = ap.parse_args()
R = os.path.join(a.root, "missionA-" + a.tag)

sp = list(csv.DictReader(open(os.path.join(R, "setpoints.csv"))))
t = np.array([float(r["t_wall"]) for r in sp]); t -= t[0]
pv = np.stack([np.array([float(r["vel_x"]) for r in sp]),
               np.array([float(r["vel_y"]) for r in sp])], 1)
cmd = np.stack([np.array([float(r["cmd_x"]) for r in sp]),
                np.array([float(r["cmd_y"]) for r in sp])], 1)

tap = list(csv.DictReader(open(os.path.join(R, "tap.csv"))))
tt = np.array([float(r["t_us"]) * 1e-6 for r in tap]); tt -= tt[0]
tp = np.stack([np.array([float(r["pe"]) for r in tap]),
               np.array([float(r["pn"]) for r in tap])], 1)
k = max(2, int(0.08 / np.median(np.diff(tt))))
sm = np.stack([np.convolve(tp[:, 0], np.ones(k) / k, "same"),
               np.convolve(tp[:, 1], np.ones(k) / k, "same")], 1)

# clock: one shift, fitted on positions (same convention as tracking_diag)
best = None
for sh in np.arange(20.0, 40.0, 0.02):
    q = np.stack([np.interp(t + sh, tt, sm[:, 0]), np.interp(t + sh, tt, sm[:, 1])], 1)
    d = np.median(np.linalg.norm(q - cmd, axis=1))
    if best is None or d < best[0]:
        best = (d, float(sh))
sh = best[1]
tv = np.stack([np.interp(t + sh, tt, np.gradient(sm[:, 0], tt)),
               np.interp(t + sh, tt, np.gradient(sm[:, 1], tt))], 1)

fs = 1.0 / np.median(np.diff(t))
print("== %s: clock %+.2f s, %.1f Hz, %.1f s" % (a.tag, sh, fs, t[-1]))
print("   |v| plan p50 %.2f max %.2f | truth p50 %.2f max %.2f m/s"
      % (np.median(np.linalg.norm(pv, axis=1)), np.linalg.norm(pv, axis=1).max(),
         np.median(np.linalg.norm(tv, axis=1)), np.linalg.norm(tv, axis=1).max()))

print("\n   band      gain(|truth|/|plan|)   phase lag     coherence")
for lo, hi in ((0.05, 0.3), (0.3, 0.8), (0.8, 1.5), (1.5, 3.0), (3.0, 6.0)):
    g, lags, coh = [], [], []
    for c in (0, 1):
        x, y = pv[:, c] - pv[:, c].mean(), tv[:, c] - tv[:, c].mean()
        n = len(x)
        X, Y = np.fft.rfft(x), np.fft.rfft(y)
        f = np.fft.rfftfreq(n, d=1.0 / fs)
        m = (f >= lo) & (f < hi)
        if m.sum() < 3:
            continue
        Sxy = X[m] * np.conj(Y[m])
        g.append(np.sum(np.abs(Y[m]) ** 2) / max(np.sum(np.abs(X[m]) ** 2), 1e-12))
        lags.append(np.angle(np.sum(Sxy)))
        cxy = np.abs(Sxy) ** 2 / np.maximum(np.abs(X[m]) ** 2 * np.abs(Y[m]) ** 2, 1e-18)
        coh.append(float(np.mean(cxy)))
    if g:
        print("   %4.1f-%-4.1f Hz   %6.2f              %+6.0f ms      %.2f"
              % (lo, hi, float(np.mean(g)), 1000.0 * float(np.mean(lags)) / (2 * np.pi * 0.5 * (lo + hi)),
                 float(np.mean(coh))))
print("\n   gain << 1 means the vehicle is not reproducing that band at all (bandwidth),")
print("   phase lag is how far behind it is, coherence says whether the comparison is meaningful.")
