"""loop_response.py -- the closed loop's frequency response, measured from the FCU's own logs.

`fcu_tracking.py` shows the tracking error is cross-track; `loop_lag.py` shows a 0.1 s lag and an
achieved/demand amplitude ratio near 2.  This script says WHERE in frequency that ratio blows up: it
estimates H(f) = achieved/demanded from the PSC accel channels with a segment-averaged cross-spectrum,
plus the coherence, so a band with |H| > 1.3 and coherence > 0.5 is a resonance and a band with |H| < 0.7
is bandwidth loss.  That distinction decides the fix:

    resonance     -> the gains are too high for the reference's content   (lower PSC_NE_POS_P / VEL_P)
    bandwidth     -> the loop is too slow                                  (raise gains / feed-forward)

    python3 scripts/loop_response.py pv1-02-02 v4a-01 d84-01-01
"""
import argparse
import csv
import os

import numpy as np
from pymavlink import mavutil

MISSION = "/home/fanxin/lio_avoid/logs"


def series(rows, keys):
    return np.array([[float(r[k]) for k in keys] for r in rows])


def psd_channels(run):
    binp = os.path.join(run, "flight.BIN")
    m = mavutil.mavlink_connection(binp)
    n_rows, e_rows = [], []
    while True:
        msg = m.recv_match()
        if msg is None:
            break
        t = msg.get_type()
        if t == "PSCN":
            n_rows.append(msg.to_dict())
        elif t == "PSCE":
            e_rows.append(msg.to_dict())
    n = series(n_rows, ["TimeUS", "DAN", "AN", "DVN", "VN"])
    e = series(e_rows, ["TimeUS", "DAE", "AE", "DVE", "VE"])
    k = min(len(n), len(e))
    t = n[:k, 0] / 1e6
    dt = float(np.median(np.diff(t)))
    sp = np.hypot(n[:k, 3], e[:k, 3])
    ux, uy = n[:k, 3] / np.maximum(sp, 1e-6), e[:k, 3] / np.maximum(sp, 1e-6)
    dem = n[:k, 1] * ux + e[:k, 1] * uy
    act = n[:k, 2] * ux + e[:k, 2] * uy
    return dt, dem, act, sp


def bands(dt, dem, act, nperseg=256):
    out = []
    seg = nperseg
    step = seg // 2
    freqs = np.fft.rfftfreq(seg, dt)
    Sdd = np.zeros_like(freqs, dtype=complex)
    Sda = np.zeros_like(freqs, dtype=complex)
    Saa = np.zeros_like(freqs, dtype=complex)
    k = 0
    for i in range(0, len(dem) - seg, step):
        d = dem[i:i + seg] - dem[i:i + seg].mean()
        a = act[i:i + seg] - act[i:i + seg].mean()
        D = np.fft.rfft(d)
        A = np.fft.rfft(a)
        Sdd += D * np.conj(D)
        Saa += A * np.conj(A)
        Sda += A * np.conj(D)
        k += 1
    if k == 0:
        return None
    Sdd /= k
    Saa /= k
    Sda /= k
    H = np.abs(Sda) / np.maximum(np.abs(Sdd), 1e-12)
    coh = np.abs(Sda) ** 2 / np.maximum(np.abs(Sdd) * np.abs(Saa), 1e-18)
    for lo, hi in ((0.05, 0.2), (0.2, 0.5), (0.5, 1.0), (1.0, 2.0), (2.0, 5.0)):
        m = (freqs >= lo) & (freqs < hi)
        if not m.any():
            continue
        out.append((lo, hi, float(np.median(H[m])), float(np.median(coh[m]))))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tags", nargs="+")
    a = ap.parse_args()
    print("%-12s %-12s %8s %8s   %s" % ("tag", "band (Hz)", "|H|", "coh", "verdict"))
    for tag in a.tags:
        run = os.path.join(MISSION, "missionA-" + tag)
        if not os.path.isfile(os.path.join(run, "flight.BIN")):
            print("%-12s (no flight.BIN)" % tag)
            continue
        dt, dem, act, sp = psd_channels(run)
        res = bands(dt, dem, act)
        if not res:
            print("%-12s (no data)" % tag)
            continue
        for lo, hi, H, coh in res:
            verdict = ""
            if coh > 0.4:
                if H > 1.3:
                    verdict = "RESONANCE (amplifying)"
                elif H < 0.7:
                    verdict = "bandwidth loss"
                else:
                    verdict = "ok"
            print("%-12s %-12s %8.2f %8.2f   %s" % (tag, "%.2f-%.2f" % (lo, hi), H, coh, verdict))


if __name__ == "__main__":
    main()
