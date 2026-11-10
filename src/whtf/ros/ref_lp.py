"""ref_lp.py -- the 2nd-order critically damped low-pass used by the thin layer's reference (round 20).

WHY THIS FILE IS SEPARATE FROM planner_source_super.py: that module imports rclpy at import time, so a
unit test cannot import it on a machine without a ROS runtime.  Everything here is pure numpy, which
makes the filter testable (`scripts/test_ref_lp.py`) and keeps the state machine's file readable.

WHAT IT IS FOR (docs/INTEGRATION.md §99.3-§102): the measured tracking residual of the delivered arm is
p90 0.34 m zero-lag, made of a ~0.08 m recording time base, a +-0.2 m along-track pendulum at the six
per-lap leg handovers, and 0.19 m cross-track.  Nine flights of FCU-parameter arms could not move it
(§100-§102), and an offline experiment (`_scratch/ref_smooth_ceiling.py`) showed that low-passing the
COMMANDED stream with tau = 0.05-0.10 s would lower the truth-to-command p90 to ~0.26 m.  This module is
that filter, applied to what the thin layer PUBLISHES (position, velocity, acceleration) instead of post
hoc, so the FCU also receives the smoother reference.

State is three triples (value, rate) per signal, integrated at the publish rate with the real dt, and it
is a strict pass-through when tau <= 0 -- the delivered behaviour must stay bit-identical.
"""
from __future__ import annotations

import numpy as np

__all__ = ["new_state", "update", "pass_through"]


def new_state(p, v, a):
    """Initialise the filter AT the first sample so there is no start transient."""
    p = np.asarray(p, dtype=float).copy()
    v = np.asarray(v, dtype=float).copy()
    a = np.asarray(a, dtype=float).copy()
    return {"p": p, "v": v, "a": a,
            "vp": np.zeros(3), "vv": np.zeros(3), "va": np.zeros(3)}


def _step(y, ydot, x, w, dt):
    """One step of  y'' = w^2 (x - y) - 2 w y'  (critically damped, unit gain at DC)."""
    acc = w * w * (x - y) - 2.0 * w * ydot
    ydot = ydot + acc * dt
    y = y + ydot * dt
    return y, ydot


def update(st, p, v, a, tau, dt):
    """Advance the filter by `dt` towards (p, v, a).  tau <= 0 returns the inputs unchanged."""
    p = np.asarray(p, dtype=float)
    v = np.asarray(v, dtype=float)
    a = np.asarray(a, dtype=float)
    if tau is None or float(tau) <= 0.0:
        return p, v, a, st
    dt = float(dt)
    if not np.isfinite(dt) or dt <= 0.0:
        return st["p"], st["v"], st["a"], st
    dt = min(dt, 0.1)                    # a stalled loop must not integrate a huge step
    w = 1.0 / float(tau)
    for key, x in (("p", p), ("v", v), ("a", a)):
        y, ydot = _step(st[key], st["v" + key], x, w, dt)
        st[key] = y
        st["v" + key] = ydot
    return st["p"], st["v"], st["a"], st


def pass_through(p, v, a):
    return np.asarray(p, dtype=float), np.asarray(v, dtype=float), np.asarray(a, dtype=float)
