#!/usr/bin/env python3
"""test-teleport-guard.py -- the geofence must not fire on a coordinate-frame change.

Round 85 measured four `ABORT: geofence` in one session, at least two of them from a truth stream that
teleported (tap 410.6 m vertical against the FCU's 3.6 m; 591 m horizontal against 9.8 m).  The supervisor
now suppresses the geofence for `--geo-grace-s` after a jump whose implied speed exceeds
`TapTruth.JUMP_MPS`, and this test pins both directions:

    A) a REAL excursion (moving at 6 m/s, no teleport) still aborts;
    B) a TELEPORT (jump of 300 m in 1 ms) does not abort, and says why;
    C) the suppression expires: two seconds after the jump, a breach aborts again.

No ROS, no MAVLink: `TapTruth.on_odom` is called with a stub that has the four attributes it reads.
"""
import importlib.util
import os
import sys
import time
import types

import numpy as np

SRC = "/mnt/d/distill_self/lio_avoid/src/supervisor/supervisor.py"


def load():
    spec = importlib.util.spec_from_file_location("sup", SRC)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["sup"] = mod
    spec.loader.exec_module(mod)
    return mod


def sample(p, t_us, v=(0.0, 0.0, 0.0)):
    stamp = types.SimpleNamespace(sec=t_us // 1_000_000, nanosec=(t_us % 1_000_000) * 1000)
    pose = types.SimpleNamespace(pose=types.SimpleNamespace(
        position=types.SimpleNamespace(x=p[0], y=p[1], z=p[2]),
        orientation=types.SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0)))
    twist = types.SimpleNamespace(twist=types.SimpleNamespace(
        linear=types.SimpleNamespace(x=v[0], y=v[1], z=v[2])))
    return types.SimpleNamespace(header=types.SimpleNamespace(stamp=stamp), pose=pose, twist=twist)


def main() -> int:
    sup = load()
    fence, grace = 10.5, 2.0
    fails = []

    # A) a real excursion: 6 m/s outward for 3 s (18 m, past the 10.5 m fence), no jump -> must fire
    tap = sup.TapTruth()
    t0 = 1_000_000
    for i in range(600):
        tap.on_odom(sample((i * 0.005 * 6.0, 0.0, 2.0), t0 + i * 5000))
    breach, note = sup.geofence_verdict(tap, fence, time.monotonic(), grace)
    print("A real excursion at %.1f m: breach=%s note=%r" % (tap.p[0], breach, note))
    if not breach:
        fails.append("A: a real 6 m/s excursion was NOT treated as a breach")

    # B) a teleport: one sample 300 m away, 1 ms later -> suppressed
    tap2 = sup.TapTruth()
    tap2.on_odom(sample((0.0, 0.0, 2.0), t0))
    tap2.on_odom(sample((300.0, 0.0, 2.0), t0 + 1000))
    breach, note = sup.geofence_verdict(tap2, fence, time.monotonic(), grace)
    print("B teleport: jumps=%d breach=%s note=%r" % (tap2.n_jumps, breach, note))
    if breach or not note:
        fails.append("B: a 300 m teleport was treated as a flight")

    # C) the same teleport, two seconds later -> the grace has expired, a breach is real again
    tap2.last_jump_wall = time.monotonic() - (grace + 0.5)
    breach, note = sup.geofence_verdict(tap2, fence, time.monotonic(), grace)
    print("C after the grace: breach=%s note=%r" % (breach, note))
    if not breach:
        fails.append("C: the suppression never expires")

    # D) a DISAGREEMENT WITH NO COORDINATE EVENT: the tap drifts 18 m out while the FCU says the pad.
    #    THIS USED TO BE SUPPRESSED and no longer is (2026-11-07 audit, finding 4): a few metres of
    #    tap-vs-FCU disagreement is exactly what an EKF divergence looks like, and MEASURED on kp1-01-01
    #    four such suppressions (3.8-5.8 m) ended in `SIM Hit ground at 10.55 m/s` with the truth outside
    #    the fence.  A false abort costs a sortie; a suppressed fence costs the aircraft.
    tap3 = sup.TapTruth()
    for i in range(200):                      # a slow outward drift at 6 m/s: no teleport
        tap3.on_odom(sample((12.0 + i * 0.03, 0.0, 2.0), t0 + i * 5000))
    tap3.fcu_p = np.array([0.0, 0.0, 2.0])    # the FCU says: the aircraft is at the pad
    breach, note = sup.geofence_verdict(tap3, fence, time.monotonic(), grace, disagree_m=2.0,
                                        ignore_m=50.0)
    print("D drift, no jump (%.1f m apart): breach=%s note=%r"
          % (np.linalg.norm(tap3.p[:2] - tap3.fcu_p[:2]), breach, note))
    if not breach:
        fails.append("D: a tap-vs-FCU disagreement with NO coordinate event was suppressed -- that is "
                     "the kp1-01-01 shape, a diverging estimate, and the fence is the only lateral bound")
    if note and "NOT suppressed" not in note:
        fails.append("D: the breach did not name the disagreement in its note")

    # E) ... and when the FCU AGREES with the tap (a real 18 m excursion), the breach is real again
    tap4 = sup.TapTruth()
    for i in range(600):
        tap4.on_odom(sample((i * 0.005 * 6.0, 0.0, 2.0), t0 + i * 5000))
    tap4.fcu_p = tap4.p.copy()
    breach, note = sup.geofence_verdict(tap4, fence, time.monotonic(), grace, disagree_m=2.0)
    print("E real excursion, FCU agrees: breach=%s note=%r" % (breach, note))
    if not breach:
        fails.append("E: an excursion both sources agree on was suppressed")

    # F) a FRAME-CHANGE-SIZED disagreement (400 m) with the FCU inside: still suppressed, no jump needed.
    #    This is the persistent case the 2 s grace cannot cover (round 85: 338/410/591 m, the stream stays
    #    in the other frame for the rest of the flight).
    tap5 = sup.TapTruth()
    for i in range(50):
        tap5.on_odom(sample((0.0, 0.0, 2.0), t0 + i * 5000))
    tap5.on_odom(sample((400.0, 0.0, 2.0), t0 + 60 * 5000))     # one sample: a teleport ...
    tap5.last_jump_wall = time.monotonic() - (grace + 5.0)      # ... whose grace has long expired
    tap5.fcu_p = np.array([0.2, 0.1, 2.0])                      # the FCU has it at the pad
    breach, note = sup.geofence_verdict(tap5, fence, time.monotonic(), grace, disagree_m=2.0,
                                        ignore_m=50.0)
    print("F 400 m frame change, FCU inside: breach=%s note=%r" % (breach, note))
    if breach or not note:
        fails.append("F: a 400 m frame change was treated as a flight")

    # G) the same teleport, but the FCU ALSO says outside -> both sources agree, the breach is real
    tap6 = sup.TapTruth()
    tap6.on_odom(sample((0.0, 0.0, 2.0), t0))
    tap6.on_odom(sample((300.0, 0.0, 2.0), t0 + 1000))
    tap6.fcu_p = np.array([15.0, 0.0, 2.0])
    breach, note = sup.geofence_verdict(tap6, fence, time.monotonic(), grace, disagree_m=2.0,
                                        ignore_m=50.0)
    print("G teleport but the FCU is outside too: breach=%s note=%r" % (breach, note))
    if not breach:
        fails.append("G: a teleport whose FCU estimate is ALSO outside the fence was suppressed")

    print()
    if fails:
        for f in fails:
            print("FAIL: " + f)
        return 1
    print("TELEPORT GUARD OK (A real excursion aborts, B teleport suppressed, C grace expires)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
