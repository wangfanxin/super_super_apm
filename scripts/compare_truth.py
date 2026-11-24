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




def asdict(a):
    return {n: np.asarray(a[n]) for n in a.dtype.names}


def smooth_timeline(sim, gap_factor=8.0):
    """Reconstruct the SAMPLING instants of SIM_STATE from arrival times.

    SIM_STATE carries no timestamp: the only time information is when the message reached us.  TCP/MAVLink
    delivers the stream in bursts (MEASURED: median inter-arrival 1.6 ms but effective rate 328 Hz, i.e.
    samples arrive in batches), so using the arrival time as the sample time injects tens of milliseconds of
    jitter -- harmless while parked, 0.3 m of position error at cruise speed and 0.8 deg of attitude error.
    The simulator emits the stream at a fixed period, so samples are (locally) uniformly spaced: fit
    t_i = beta0 + beta1*i per contiguous segment (split at arrival gaps), which recovers the sampling instants
    and removes the burst jitter.  Recording continuity (rate/gap checks) still uses the ARRIVAL times.
    """
    t = np.asarray(sim["t_wall_us"], float) * 1e-6
    if t.size < 3:
        return t
    i = np.arange(t.size, dtype=float)
    dt = np.diff(t)
    med = float(np.median(dt[dt > 0])) if np.any(dt > 0) else 0.002
    cuts = np.where(dt > max(0.050, gap_factor * med))[0]
    out = t.copy()
    for seg in np.split(i, cuts + 1):
        if seg.size < 3:
            continue
        A = np.column_stack([seg, np.ones(seg.size)])
        coef, *_ = np.linalg.lstsq(A, t[seg.astype(int)], rcond=None)
        out[seg.astype(int)] = A @ coef
    return out


def fit_clock_position(tap, sim, verbose=False):
    """Estimate t_sim = a*t_wall + b by MINIMISING THE POSITION RESIDUAL, not by correlating an event signal.

    The tap carries local NED and SIM_STATE carries geodetic position; both describe the same model state,
    so for the right (a, b) an affine lat/lon/alt -> NED fit leaves a residual of millimetres.  The old
    FFT/event fit was biased by ~2 % because the event signal is weak in cruise and the pressure of one
    recorded flight (its own take-off/landing peaks) dominates; the position fit has no such failure mode.
    The offset b is the recorder's start offset (SIM_STATE recording begins a little after the tap's t=0),
    which is exactly the "explicit clock mapping" Phase 1 must state.
    """
    t_tap = np.asarray(tap["t_us"], float) * 1e-6
    t_sim = np.asarray(sim["t_wall_us"], float) * 1e-6
    P = np.column_stack([np.asarray(tap["pn"], float), np.asarray(tap["pe"], float),
                         np.asarray(tap["pd"], float)])
    G = np.column_stack([np.asarray(sim["lat"], float), np.asarray(sim["lon"], float),
                         np.asarray(sim["alt"], float)])
    ok_t = np.all(np.isfinite(P), axis=1) & np.isfinite(t_tap)
    ok_s = np.all(np.isfinite(G), axis=1) & np.isfinite(t_sim)
    t_tap, P = t_tap[ok_t], P[ok_t]
    t_sim, G = t_sim[ok_s], G[ok_s]
    # centre the geodetic columns so the affine fit is well conditioned
    G = G - np.median(G, axis=0)
    stride = max(1, t_sim.size // 600)
    ts_s = t_sim[::stride]
    X = np.column_stack([G[::stride], np.ones(ts_s.size)])

    def residual(a, b, ts, Xm):
        tq = a * ts + b
        m = (tq >= t_tap[0]) & (tq <= t_tap[-1])
        if m.sum() < 200:
            return float("inf")
        Pq = np.column_stack([np.interp(tq[m], t_tap, P[:, k]) for k in range(3)])
        coef, *_ = np.linalg.lstsq(Xm[m], Pq, rcond=None)
        return float(np.sqrt(np.mean((Pq - Xm[m] @ coef) ** 2)))

    best = (float("inf"), 1.0, 0.0)
    for a in np.arange(0.98, 1.0201, 0.001):
        for b in np.arange(-10.0, 10.01, 0.1):
            r = residual(a, b, ts_s, X)
            if r < best[0]:
                best = (r, a, b)
    r0, a0, b0 = best
    # TWO refinement rounds around the coarse winner: +-3 coarse steps wide, then +-3 fine steps.  A single
    # narrow round is not enough -- the true optimum can sit between coarse grid points (MEASURED in the
    # selftest: coarse 1.0010/3.20, true 0.9995/3.25, and a +-0.001 window never contained it).
    for a_span, a_step, b_span, b_step in ((0.003, 0.0002, 0.3, 0.01), (0.0006, 0.00002, 0.06, 0.002)):
        for a in np.arange(a0 - a_span, a0 + a_span + a_step * 0.5, a_step):
            for b in np.arange(b0 - b_span, b0 + b_span + b_step * 0.5, b_step):
                r = residual(a, b, ts_s, X)
                if r < best[0]:
                    best = (r, a, b)
        r0, a0, b0 = best
    # residual over EVERY sample at the winning map (the subsampled value can be optimistic)
    r_all = residual(best[1], best[2], t_sim, np.column_stack([G, np.ones(t_sim.size)]))
    if verbose:
        print("clock search: coarse a=%.4f b=%.2f rms=%.4f -> refined a=%.5f b=%.3f rms=%.4f"
              % (a0, b0, r0, best[1], best[2], best[0]))
    return {"a": float(best[1]), "b": float(best[2]), "corr": float("nan"),
            "pos_rms_clock": float(r_all),
            "note": "t_sim = a*t_wall + b, fitted by minimising the affine geodetic-position residual"}



def fit_clock_velocity(tap, sim, verbose=False):
    """Clock map from the VELOCITY channels, which are the channel SIM_STATE reports faithfully.

    MEASURED 2026-11-25: SIM_STATE's lat/lon go through SITL's smoothed-state path
    (SIM_Aircraft::fill_fdm uses smoothing.location when use_smoothing, and SITL moves its origin at 1 Hz),
    so the position channel carries a slowly varying ~0.2-0.35 m distortion in flight, while vn/ve/vd track
    the tap to ~10 mm/s.  Fitting the clock on velocity therefore gives a mapping that is not contaminated
    by that distortion: a=0.99980, b=1.020 s on the clean sortie.
    """
    t_tap = np.asarray(tap["t_us"], float) * 1e-6
    t_sim = np.asarray(sim["t_wall_us"], float) * 1e-6
    Vt = np.column_stack([np.asarray(tap["vn"], float), np.asarray(tap["ve"], float),
                          np.asarray(tap["vd"], float)])
    Vs = np.column_stack([np.asarray(sim["vn"], float), np.asarray(sim["ve"], float),
                          np.asarray(sim["vd"], float)])

    def vrms(a, b):
        tq = a * t_sim + b
        m = (tq >= t_tap[0]) & (tq <= t_tap[-1])
        if m.sum() < 1000:
            return float("inf")
        Vq = np.column_stack([np.interp(tq[m], t_tap, Vt[:, k]) for k in range(3)])
        return float(np.sqrt(np.mean((Vs[m] - Vq) ** 2)))

    best = (float("inf"), 1.0, 0.0)
    for a in np.arange(0.995, 1.0051, 0.0002):
        for b in np.arange(-2.0, 4.01, 0.02):
            r = vrms(a, b)
            if r < best[0]:
                best = (r, a, b)
    for a in np.arange(best[1] - 0.0003, best[1] + 0.00031, 0.00002):
        for b in np.arange(best[2] - 0.04, best[2] + 0.041, 0.002):
            r = vrms(a, b)
            if r < best[0]:
                best = (r, a, b)
    if verbose:
        print("clock search (velocity): a=%.5f b=%.3f  v_rms=%.4f m/s" % (best[1], best[2], best[0]))
    return {"a": float(best[1]), "b": float(best[2]), "corr": float("nan"),
            "vel_rms_clock": float(best[0]),
            "note": "t_sim = a*t_wall + b, fitted on the vn/ve/vd channels"}


def integrate_velocity(sim):
    """Reconstruct the NED position by integrating SIM_STATE's own velocity over its sampling timeline.

    Returns (t_sim_smoothed, P_ned) with P[0] = 0 -- a constant offset has no meaning until the geodetic
    frame is tied; callers fit that offset out.  Trapezoid rule on the smoothed sampling instants.
    """
    t = np.asarray(sim["t_wall_us"], float) * 1e-6
    V = np.column_stack([np.asarray(sim["vn"], float), np.asarray(sim["ve"], float),
                         np.asarray(sim["vd"], float)])
    ok = np.all(np.isfinite(V), axis=1) & np.isfinite(t)
    t, V = t[ok], V[ok]
    P = np.zeros_like(V)
    if t.size > 1:
        dt = np.diff(t)
        dt[dt < 0] = 0.0
        P[1:] = np.cumsum((V[1:] + V[:-1]) * 0.5 * dt[:, None], axis=0)
    return t, P


def position_compare(tap, sim, clock, method="integrated"):
    """Position agreement at matched times.

    method="integrated" (default): rebuild position from the velocity channel (see integrate_velocity) and
    fit an affine NED->tap-NED map; this is the reconstruction a SIM_STATE truth source would use.
    method="direct": use the reported lat/lon/alt through the affine geodetic fit (diagnostic only --
    it inherits the SITL smoothed-state distortion).
    """
    a, b = clock["a"], clock["b"]
    t_tap = np.asarray(tap["t_us"], float) * 1e-6
    if method == "direct":
        geo = fit_geodetic(tap, sim, clock)
        geo["method"] = "direct (lat/lon/alt through the affine geodetic fit)"
        return geo
    t_est, P_est = integrate_velocity(sim)
    ts = a * t_est + b
    ok = (ts >= t_tap.min()) & (ts <= t_tap.max())
    if ok.sum() < 100:
        raise SystemExit("position compare: too few overlapping samples")
    ts = ts[ok]
    P = P_est[ok]
    pn_t = np.interp(ts, t_tap, np.asarray(tap["pn"], float))
    pe_t = np.interp(ts, t_tap, np.asarray(tap["pe"], float))
    pd_t = np.interp(ts, t_tap, np.asarray(tap["pd"], float))
    # full affine NED -> tap NED (constant rotation/scale between the two frames plus an offset)
    X = np.column_stack([P, np.ones(P.shape[0])])
    pred = np.column_stack([X @ np.linalg.lstsq(X, y, rcond=None)[0] for y in (pn_t, pe_t, pd_t)])
    res = np.hypot(np.hypot(pred[:, 0] - pn_t, pred[:, 1] - pe_t), pred[:, 2] - pd_t)
    return {"method": "integrated (SIM_STATE velocity, trapezoid)", "pos_rms_m": float(np.sqrt(np.mean(res ** 2))),
            "pos_p99_m": pct(res, 99), "pos_max_m": float(res.max()), "samples": int(ok.sum())}


def window_arrays(tap, sim, clock, window):
    """Restrict both streams to the tap-time window [from, to] (seconds).  The clock map is kept."""
    f, t = window
    a, b = clock["a"], clock["b"]
    t_tap = np.asarray(tap["t_us"], float) * 1e-6
    t_sim = np.asarray(sim["t_wall_us"], float) * 1e-6
    mt = (t_tap >= f) & (t_tap <= t)
    msim = ((a * t_sim + b) >= f) & ((a * t_sim + b) <= t)
    # tap/sim come from np.genfromtxt: structured arrays, not dicts
    if hasattr(tap, "dtype"):
        tap, sim = asdict(tap), asdict(sim)
    tw = {n: np.asarray(tap[n])[mt] for n in tap}
    sw = {n: np.asarray(sim[n])[msim] for n in sim}
    return tw, sw


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
    # The repo's own recordings; DSH_ROOT comes from setup/env.sh, the file location is the fallback so the
    # tool works from a bare clone with no environment.
    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    root = os.path.join(os.environ.get("DSH_ROOT") or repo, ".runs")
    if os.path.isdir(root):
        for name in sorted(os.listdir(root)):
            p = os.path.join(root, name, "tap.csv")
            if os.path.isfile(p) and os.path.getsize(p) > 1_000_000:
                tap_path = p
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
    clock = fit_clock_position(asdict(tap), asdict(sim) if hasattr(sim, "dtype") else sim)
    print("selftest: recovered a=%.6f (expect %.6f), b=%.3f (expect %.3f), clock pos rms=%.4f m"
          % (clock["a"], a_true, clock["b"], b_true, clock["pos_rms_clock"]))
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
    ap.add_argument("--window", default="", help="evaluate on this TAP-time window, seconds: FROM:TO")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    if not args.tap or not args.sim_state:
        ap.error("--tap and --sim-state are required (or use --selftest)")

    tap = asdict(load_csv(args.tap, TAP_COLS))
    sim_arrival = asdict(load_csv(args.sim_state, SIM_COLS))
    sim = dict(sim_arrival)
    sim["t_wall_us"] = smooth_timeline(sim_arrival) * 1e6      # sampling instants, not arrival times
    clock = fit_clock_velocity(tap, sim, verbose=True)
    clock_pos = fit_clock_position(tap, sim)      # diagnostic only: biased by the position distortion
    clock_fft = fit_clock(tap, sim)               # kept as an independent cross-check, not as the estimator
    window = None
    if args.window:
        window = tuple(float(v) for v in args.window.split(":"))
        tap_w, sim_w = window_arrays(tap, sim, clock, window)
        _, sim_arr_w = window_arrays(tap, sim_arrival, clock, window)
    else:
        tap_w, sim_w, sim_arr_w = tap, sim, sim_arrival
    geo = position_compare(tap_w, sim_w, clock)
    geo_direct = fit_geodetic(tap_w, sim_w, clock)
    st = state_compare(tap_w, sim_w, clock)
    rates = {"tap": rate_profile(np.asarray(tap_w["t_us"], float) * 1e-6, "tap"),
             "sim_state": rate_profile(np.asarray(sim_arr_w["t_wall_us"], float) * 1e-6, "sim_state")}

    ok_rate = rates["sim_state"].get("rate_hz", 0) >= args.min_rate_hz
    ok_gap = rates["sim_state"].get("gaps_gt_20ms", 9) == 0
    ok_clock = 0.97 <= clock["a"] <= 1.03
    ok_vel = st.get("vn_rms", 9e9) <= args.max_vel_rms and st.get("ve_rms", 9e9) <= args.max_vel_rms
    ok_pos = geo["pos_rms_m"] <= args.max_pos_rms
    ok_att = st.get("att_rms_deg", 9e9) <= args.max_att_rms
    ok = ok_rate and ok_gap and ok_clock and ok_vel and ok_pos and ok_att

    res = {"tap": args.tap, "sim_state": args.sim_state, "clock": clock,
           "clock_position_diagnostic": clock_pos, "clock_fft_crosscheck": clock_fft,
           "window": list(window) if window else None,
           "geodetic": geo_direct, "position": geo,
           "state": st, "rates": rates,
           "verdict": {"rate": ok_rate, "gaps": ok_gap, "clock": ok_clock, "velocity": ok_vel,
                       "geodetic": ok_pos, "attitude": ok_att, "ok": ok}}
    print("== clock map: t_sim = %.6f * t_wall + %.4f  (velocity-fit rms %.4f m/s; position-fit a=%.5f;"
          " FFT cross-check a=%.4f corr %.3f)"
          % (clock["a"], clock["b"], clock.get("vel_rms_clock", float("nan")), clock_pos["a"],
             clock_fft["a"], clock_fft["corr"]))
    if window:
        print("== window: tap %.1f .. %.1f s (the same window is applied to SIM_STATE through the clock map)"
              % window)
    print("== position (%s): rms %.4f m, p99 %.4f m, max %.3f m"
          % (geo["method"], geo["pos_rms_m"], geo["pos_p99_m"], geo["pos_max_m"]))
    print("   diagnostic, direct lat/lon/alt: pos rms %.4f m (SITL smoothed-state path)" % geo_direct["pos_rms_m"])
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
