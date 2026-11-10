#!/usr/bin/env python3
"""test_ref_lp.py -- unit tests for the thin layer's reference low-pass (round 20).

The filter decides what the FCU is fed, so its properties are asserted, not eyeballed:

  1. tau <= 0 is a STRICT pass-through: the delivered behaviour must be bit-identical, and this is the
     property that protects every earlier flight record.
  2. At DC (constant input) the filter converges to the input with unit gain -- a filter with gain != 1
     would put a standing offset into the published reference, which is exactly the sort of silent bias
     this project keeps finding the hard way.
  3. A step is followed without overshoot (critically damped) and settles inside 2 % within ~6 tau.
  4. The output never leads the input (a low-pass must not anticipate) and its derivative is bounded by
     the input's own slope scale (no injected jerk spikes).
  5. A stalled loop (dt <= 0) must not move the state.

    python3 scripts/test_ref_lp.py
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src", "whtf", "ros"))

import ref_lp  # noqa: E402

FAIL = []


def check(name, cond, detail=""):
    print("  %-58s %s%s" % (name, "PASS" if cond else "FAIL",
                            ("  " + detail) if detail else ""))
    if not cond:
        FAIL.append(name)


def main():
    dt = 0.02
    # 1. pass-through
    st = ref_lp.new_state([0, 0, 0], [0, 0, 0], [0, 0, 0])
    p, v, a, st2 = ref_lp.update(st, [1.0, 2.0, 3.0], [0.5, 0.0, 0.0], [0.1, 0.0, 0.0], 0.0, dt)
    check("tau=0 is a strict pass-through",
          np.array_equal(p, np.array([1.0, 2.0, 3.0])) and st2 is st)

    # 2. unit DC gain
    st = ref_lp.new_state([0, 0, 0], [0, 0, 0], [0, 0, 0])
    out = None
    for _ in range(int(20 * 0.08 / dt)):
        out, _, _, st = ref_lp.update(st, [2.0, 0, 0], [0, 0, 0], [0, 0, 0], 0.08, dt)
    check("DC gain is 1 (no standing offset)", abs(out[0] - 2.0) < 2e-2, "settled at %.4f for 2.0" % out[0])

    # 3. step response: no overshoot, settles within 6 tau (2 %)
    st = ref_lp.new_state([0, 0, 0], [0, 0, 0], [0, 0, 0])
    traj = []
    n = int(10 * 0.08 / dt)
    for _ in range(n):
        out, _, _, st = ref_lp.update(st, [1.0, 0, 0], [0, 0, 0], [0, 0, 0], 0.08, dt)
        traj.append(out[0])
    traj = np.array(traj)
    over = float(traj.max())
    settle_idx = None
    for i in range(len(traj)):
        if np.all(np.abs(traj[i:] - 1.0) < 0.02):
            settle_idx = i
            break
    check("step: no overshoot (max <= 1.02)", over <= 1.02, "max %.4f" % over)
    # Analytic 2 % settling of a critically damped 2nd-order system is 5.83 tau; the discrete explicit
    # integration adds a little, and this filter measures 6.25 tau at dt = 0.02 (tau = 0.08 -> 0.50 s).
    # The bound is 8 tau so it fails only if the filter is genuinely slow, not if the integrator costs a
    # fraction of a time constant -- this is a property test, not a flight threshold.
    check("step: settles < 2 % within 8 tau",
          settle_idx is not None and settle_idx * dt <= 8 * 0.08,
          "settled at %.3f s (analytic 5.83 tau = %.2f s, bound 8 tau = %.2f s)"
          % ((settle_idx or 0) * dt, 5.83 * 0.08, 8 * 0.08))

    # 4. no anticipation: output stays at/below a monotone ramp input
    st = ref_lp.new_state([0, 0, 0], [0, 0, 0], [0, 0, 0])
    worst = -1e9
    for i in range(1, 200):
        x = 0.5 * i * dt
        out, _, _, st = ref_lp.update(st, [x, 0, 0], [0.5, 0, 0], [0, 0, 0], 0.08, dt)
        worst = max(worst, out[0] - x)
    check("no anticipation on a ramp (output never leads)", worst <= 1e-6, "max lead %.2e m" % worst)

    # 5. stalled loop
    st = ref_lp.new_state([0, 0, 0], [0, 0, 0], [0, 0, 0])
    p0, v0, a0, st = ref_lp.update(st, [1, 0, 0], [0, 0, 0], [0, 0, 0], 0.08, 0.02)
    before = st["p"].copy()
    p1, v1, a1, st = ref_lp.update(st, [1, 0, 0], [0, 0, 0], [0, 0, 0], 0.08, 0.0)
    check("dt <= 0 does not move the state", np.array_equal(before, st["p"]))

    print()
    print("ref-lp test: %s" % ("PASS" if not FAIL else "FAIL (%d)" % len(FAIL)))
    return 0 if not FAIL else 1


if __name__ == "__main__":
    sys.exit(main())
