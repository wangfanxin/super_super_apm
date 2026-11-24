#!/usr/bin/env python3
"""compare_truth.py -- Phase 1: can the UNPATCHED-SITL truth (MAVLink SIM_STATE, recorded by
sim/src/sim_state_recv.py) replace the patched-firmware 1.2 kHz tap?

The two streams describe the same flight but on different clocks (tap = SITL model time; SIM_STATE =
receive wall time) and SIM_STATE has no local position (only lat/lon/alt).  This tool therefore measures,
from the data itself:

  1. the clock map   t_sim = a * t_wall + b     (a is the run's real-time factor, ~1.0)
  2. the geodetic map (lat/lon -> local NED)   affine fit against the tap's own pn/pe
  3. the state agreement on the fitted map: velocity / attitude / accel / gyro RMS and max
  4. the rate/gap profile of SIM_STATE (the tap is 1.2 kHz; SIM_STATE is whatever SITL delivers)

Output: a human report + JSON.  Exit 0 only if the mapping and the agreement are inside the thresholds
printed next to them -- i.e. "SIM_STATE is a usable truth source", not "the files are non-empty".

    python3 scripts/compare_truth.py --tap <sortie>/tap.csv --sim-state <sortie>/sim_state.csv
    python3 scripts/compare_truth.py --selftest
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

TAP_COLS = ("t_us", "inst", "pn", "pe", "pd", "vn", "ve", "vd", "qw", "qx", "qy", "qz",
            "wx", "wy", "wz", "ax", "ay", "az")
SIM_COLS = ("t_wall_us", "t_unix_us", "lat", "lon", "alt", "q1", "q2", "q3", "q4",
            "roll", "pitch", "yaw", "ax", "ay", "az", "wx", "wy", "wz",
            "vn", "ve", "vd", "pn", "pe", "pd", "lat_int", "lon_int", "std_h", "std_v")


def load_csv(path, want):
    if not os.path.isfile(path):
        raise SystemExit("missing file: %s" % path)
    a = np.genfromtxt(path, delimiter=",", names=True, invalid_raise=False)
    if a.size == 0:
        raise SystemExit("empty file: %s" % path)
    names = a.dtype.names or ()
    missing = [c for c in want if c not in names]
    if missing:
        raise SystemExit("%s: missing columns %s" % (path, missing))
    return a


def pct(x, q):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return float(np.percentile(x, q)) if x.size else float("nan")


def resample(t, y, grid):
    """Linear interpolation on a uniform grid, with a validity mask (grid inside [t.min, t.max])."""
    y = np.interp(grid, t, y) if t.size > 1 else np.zeros_like(grid)
    return y, (grid >= t.min()) & (grid <= t.max())


def quat_angle_deg(q1, q2, q3, q4, qw, qx, qy, qz):
    """Angle between two attitude quaternions (a,b,c,d) = (w,x,y,z), degrees."""
    dot = np.abs(q1 * qw + q2 * qx + q3 * qy + q4 * qz)
    dot = np.clip(dot, 0.0, 1.0)
    return np.degrees(2.0 * np.arccos(dot))


def fit_clock(tap, sim):
    """Estimate t_sim = a*t_wall + b by maximising the correlation of an EVENT signal.

    Signal = horizontal speed + 0.5*|vertical speed| + 0.3*|accel| + 0.2*|gyro|: large where the vehicle
    manoeuvres (take-off, corners, landing), flat in cruise.  The scale is searched around 1 (SITL runs
    --speedup 1); the offset comes from an FFT cross-correlation at each candidate scale, refined by a
    parabola on the correlation peak.  scipy.signal.correlate(method="fft") keeps this O(n log n).
    """
    from scipy.signal import correlate

    fs = 25.0
    t_tap = np.asarray(tap["t_us"], float) * 1e-6
    t_sim = np.asarray(sim["t_wall_us"], float) * 1e-6

    def sig_of(names, src):
        v = np.hypot(src["vn"], src["ve"]) + 0.5 * np.abs(src["vd"])
        acc = np.sqrt(src["ax"] ** 2 + src["ay"] ** 2 + src["az"] ** 2)
        gyr = np.sqrt(src["wx"] ** 2 + src["wy"] ** 2 + src["wz"] ** 2)
        return v + 0.3 * acc + 0.2 * gyr

    ok_t = np.isfinite(t_tap) & np.isfinite(tap["vn"]) & np.isfinite(tap["ve"])
    ok_s = np.isfinite(t_sim) & np.isfinite(sim["vn"]) & np.isfinite(sim["ve"])
    tt, ts = t_tap[ok_t], t_sim[ok_s]
    xg = sig_of(TAP_COLS, tap)[ok_t]
    yg = sig_of(SIM_COLS, sim)[ok_s]
    if tt.size < 100 or ts.size < 100:
        raise SystemExit("not enough samples to fit a clock (tap %d, sim %d)" % (tt.size, ts.size))
    t0 = max(tt.min(), ts.min())
    t1 = min(tt.max(), ts.max())
    grid = np.arange(t0, t1, 1.0 / fs)
    yy = np.interp(grid, ts, yg)
    yy = yy - yy.mean()

    def score(a):
        tg = a * (grid - t0) + t0
        xx = np.interp(tg, tt, xg)
        valid = (tg >= tt.min()) & (tg <= tt.max())
        if valid.sum() < 0.6 * grid.size:
            return -1.0, 0.0
        xc = xx - xx.mean()
        c = correlate(xc, yy, mode="full", method="fft")
        k = int(np.argmax(c))
        denom = (np.linalg.norm(xc) * np.linalg.norm(yy) + 1e-12)
        lag_s = (k - (yy.size - 1)) / fs
        return float(c[k] / denom), lag_s

    best = (-2.0, 1.0, 0.0)
    for a in np.arange(0.97, 1.030001, 0.001):
        s, lag = score(a)
        if s > best[0]:
            best = (s, a, lag)
    # refine scale + peak with a parabola around the winning scale
    s0, a0, lag0 = best
    for a in np.arange(a0 - 0.001, a0 + 0.0011, 0.0001):
        s, lag = score(a)
        if s > s0:
            s0, a0, lag0 = s, a, lag
    # interpret the lag: y[k] ~ x[k + lag], and x was sampled at a*(grid-t0)+t0,
    # so a sim time t_sim_wall maps to tap time a*(t_sim_wall - t0) + t0 - lag.
    # In the desired form t_sim = a * t_wall + b:  b = t0*(1-a) - lag.
    b = a0 * lag0 + t0 * (1.0 - a0)
    return {"a": float(a0), "b": float(b), "corr": float(s0), "lag_s": float(lag0),
            "note": "t_sim = a * t_wall + b; a is the measured real-time factor"}


def fit_geodetic(tap, sim, clock):
    """Affine (lat,lon,alt) -> local NED against the tap, at matched times."""
    a, b = clock["a"], clock["b"]
    t_tap = np.asarray(tap["t_us"], float) * 1e-6
    t_sim = np.asarray(sim["t_wall_us"], float) * 1e-6
    m = (t_sim >= t_tap.min() * 1.0) | True
    ts = a * t_sim + b                      # sim -> tap (sim) time
    ok = (ts >= t_tap.min()) & (ts <= t_tap.max())
    if ok.sum() < 100:
        raise SystemExit("geodetic fit: too few overlapping samples")
    pn_t = np.interp(ts[ok], t_tap, tap["pn"])
    pe_t = np.interp(ts[ok], t_tap, tap["pe"])
    pd_t = np.interp(ts[ok], t_tap, tap["pd"])
    lat = sim["lat"][ok]
    lon = sim["lon"][ok]
    lat0 = float(np.median(lat))
    lon0 = float(np.median(lon))
    dlat = np.radians(lat - lat0)
    dlon = np.radians(lon - lon0)
    # NED local from SITL home: fit [n,e] = M [dlat,dlon] + c
    X = np.column_stack([dlat, dlon, np.ones_like(dlat)])
    coef_n, *_ = np.linalg.lstsq(X, pn_t, rcond=None)
    coef_e, *_ = np.linalg.lstsq(X, pe_t, rcond=None)
    pred_n = X @ coef_n
    pred_e = X @ coef_e
    res = np.hypot(pred_n - pn_t, pred_e - pe_t)
    off_d = float(np.median(pd_t))
    return {"lat0": lat0, "lon0": lon0,
            "coef_n": [float(v) for v in coef_n], "coef_e": [float(v) for v in coef_e],
            "pos_rms_m": float(np.sqrt(np.mean(res ** 2))), "pos_p99_m": pct(res, 99),
            "pos_max_m": float(res.max()), "pd_median_rel_m": off_d}


def state_compare(tap, sim, clock):
    """Interpolate the tap onto the SIM_STATE timeline and compare the shared state fields."""
    a, b = clock["a"], clock["b"]
    t_tap = np.asarray(tap["t_us"], float) * 1e-6
    t_sim = np.asarray(sim["t_wall_us"], float) * 1e-6
    ts = a * t_sim + b
    ok = (ts >= t_tap.min()) & (ts <= t_tap.max())
    out = {}
    if ok.sum() < 50:
        return {"samples": int(ok.sum())}
    ts = ts[ok]
    for name, key in (("vn", "vn"), ("ve", "ve"), ("vd", "vd")):
        ref = np.interp(ts, t_tap, tap[key])
        meas = np.asarray(sim[key], float)[ok]
        d = meas - ref
        out[name + "_rms"] = float(np.sqrt(np.mean(d ** 2)))
        out[name + "_max"] = float(np.max(np.abs(d)))
    dq = quat_angle_deg(np.asarray(sim["q1"], float)[ok], np.asarray(sim["q2"], float)[ok],
                        np.asarray(sim["q3"], float)[ok], np.asarray(sim["q4"], float)[ok],
                        np.interp(ts, t_tap, tap["qw"]), np.interp(ts, t_tap, tap["qx"]),
                        np.interp(ts, t_tap, tap["qy"]), np.interp(ts, t_tap, tap["qz"]))
    dq = dq[np.isfinite(dq)]
    out["att_rms_deg"] = float(np.sqrt(np.mean(dq ** 2))) if dq.size else float("nan")
    out["att_max_deg"] = float(np.max(dq)) if dq.size else float("nan")
    for name, key in (("ax", "ax"), ("ay", "ay"), ("az", "az"), ("wx", "wx"), ("wy", "wy"), ("wz", "wz")):
        ref = np.interp(ts, t_tap, tap[key])
        meas = np.asarray(sim[key], float)[ok]
        out[name + "_rms"] = float(np.sqrt(np.mean((meas - ref) ** 2)))
    out["samples"] = int(ok.sum())
    return out


def rate_profile(t, label):
    t = np.asarray(t, float)
    if t.size < 3:
        return {"n": int(t.size)}
    dt = np.diff(t)
    dt = dt[dt > 0]
    return {"n": int(t.size), "span_s": float(t[-1] - t[0]),
            "rate_hz": float(1.0 / np.median(dt)) if dt.size else float("nan"),
            "p99_gap_ms": float(pct(dt, 99) * 1e3), "max_gap_ms": float(dt.max() * 1e3),
            "gaps_gt_20ms": int(np.sum(dt > 0.020))}


def selftest():
    """Build a synthetic pair from one kept tap.csv and check the fits recover the known mapping."""
    tap_path = None
    for root in ("/home/fanxin/super_super_apm/.runs", "/mnt/d/distill_self/super_super_apm/.runs"):
        if os.path.isdir(root):
            for name in sorted(os.listdir(root)):
                p = os.path.join(root, name, "tap.csv")
                if os.path.isfile(p) and os.path.getsize(p) > 1_000_000:
                    tap_path = p
                    break
        if tap_path:
            break
    if not tap_path:
        print("selftest: no tap.csv found"); return 2
    tap = load_csv(tap_path, TAP_COLS)
    t = np.asarray(tap["t_us"], float) * 1e-6
    a_true, b_true = 1.0 / 1.0005, 3.25
    t_sim = (t - b_true) / a_true
    # invert the synthetic lat/lon map: use a simple rotation+scale about a fake origin
    lat0, lon0 = math.radians(-35.0), math.radians(149.0)
    R = 6378137.0
    pn, pe = tap["pn"], tap["pe"]
    lat = np.degrees(lat0) + np.degrees(pn / R)
    lon = np.degrees(lon0) + np.degrees(pe / (R * math.cos(lat0)))
    sim = {"t_wall_us": t_sim * 1e6, "t_unix_us": t_sim * 1e6,
           "lat": lat, "lon": lon, "alt": np.asarray(tap["pd"], float) * -1.0 + 584.0,
           "q1": tap["qw"], "q2": tap["qx"], "q3": tap["qy"], "q4": tap["qz"],
           "vn": tap["vn"], "ve": tap["ve"], "vd": tap["vd"],
           "ax": tap["ax"], "ay": tap["ay"], "az": tap["az"],
           "wx": tap["wx"], "wy": tap["wy"], "wz": tap["wz"]}
    clock = fit_clock(tap, sim)
    print("selftest: recovered a=%.6f (expect %.6f), b=%.3f (expect %.3f), corr=%.3f"
          % (clock["a"], a_true, clock["b"], b_true, clock["corr"]))
    ok = abs(clock["a"] - a_true) < 2e-4 and abs(clock["b"] - b_true) < 0.05
    geo = fit_geodetic(tap, sim, clock)
    print("selftest: geodetic pos_rms=%.4f m (expect < 0.01)" % geo["pos_rms_m"])
    ok = ok and geo["pos_rms_m"] < 0.01
    st = state_compare(tap, sim, clock)
    print("selftest: velocity rms %.2e m/s (expect ~0)" % st.get("vn_rms", float("nan")))
    ok = ok and st.get("vn_rms", 9e9) < 0.02
    print("selftest: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tap", default="", help="tap.csv of the sortie")
    ap.add_argument("--sim-state", default="", help="sim_state.csv recorded by sim/src/sim_state_recv.py")
    ap.add_argument("--out", default="", help="JSON output path (default: next to the tap)")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--min-rate-hz", type=float, default=90.0, help="SIM_STATE rate floor")
    ap.add_argument("--max-vel-rms", type=float, default=0.02, help="m/s")
    ap.add_argument("--max-pos-rms", type=float, default=0.02, help="m")
    ap.add_argument("--max-att-rms", type=float, default=0.5, help="deg")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    if not args.tap or not args.sim_state:
        ap.error("--tap and --sim-state are required (or use --selftest)")

    tap = load_csv(args.tap, TAP_COLS)
    sim = load_csv(args.sim_state, SIM_COLS)
    clock = fit_clock(tap, sim)
    geo = fit_geodetic(tap, sim, clock)
    st = state_compare(tap, sim, clock)
    rates = {"tap": rate_profile(np.asarray(tap["t_us"], float) * 1e-6, "tap"),
             "sim_state": rate_profile(np.asarray(sim["t_wall_us"], float) * 1e-6, "sim_state")}

    ok_rate = rates["sim_state"].get("rate_hz", 0) >= args.min_rate_hz
    ok_gap = rates["sim_state"].get("gaps_gt_20ms", 9) == 0
    ok_clock = 0.97 <= clock["a"] <= 1.03 and clock["corr"] > 0.9
    ok_vel = st.get("vn_rms", 9e9) <= args.max_vel_rms and st.get("ve_rms", 9e9) <= args.max_vel_rms
    ok_pos = geo["pos_rms_m"] <= args.max_pos_rms
    ok_att = st.get("att_rms_deg", 9e9) <= args.max_att_rms
    ok = ok_rate and ok_gap and ok_clock and ok_vel and ok_pos and ok_att

    res = {"tap": args.tap, "sim_state": args.sim_state, "clock": clock, "geodetic": geo,
           "state": st, "rates": rates,
           "verdict": {"rate": ok_rate, "gaps": ok_gap, "clock": ok_clock, "velocity": ok_vel,
                       "geodetic": ok_pos, "attitude": ok_att, "ok": ok}}
    print("== clock map: t_sim = %.6f * t_wall + %.4f  (corr %.3f, measured rtf %.6f)"
          % (clock["a"], clock["b"], clock["corr"], clock["a"]))
    print("== geodetic fit: pos rms %.4f m, p99 %.4f m, max %.3f m; lat0/lon0 %.7f/%.7f"
          % (geo["pos_rms_m"], geo["pos_p99_m"], geo["pos_max_m"], geo["lat0"], geo["lon0"]))
    print("== state: samples %d, vel rms %.4f m/s (n/e/d), att rms %.3f deg, acc rms %.3f"
          % (st.get("samples", 0), st.get("vn_rms", float("nan")), st.get("att_rms_deg", float("nan")),
             st.get("az_rms", float("nan"))))
    print("== rates: tap %.1f Hz, sim_state %.1f Hz (p99 gap %.2f ms, gaps>20ms %d)"
          % (rates["tap"].get("rate_hz", float("nan")), rates["sim_state"].get("rate_hz", float("nan")),
             rates["sim_state"].get("p99_gap_ms", float("nan")), rates["sim_state"].get("gaps_gt_20ms", -1)))
    print("== verdict: rate %s, gaps %s, clock %s, velocity %s, geodetic %s, attitude %s -> %s"
          % tuple("PASS" if v else "FAIL" for v in (ok_rate, ok_gap, ok_clock, ok_vel, ok_pos, ok_att, ok)))
    out = args.out or os.path.join(os.path.dirname(os.path.abspath(args.tap)), "truth_compare.json")
    with open(out, "w", encoding="utf-8") as fh:
        json.dump(res, fh, indent=1, ensure_ascii=False)
    print("wrote %s" % out)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
