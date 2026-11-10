#!/usr/bin/env python3
"""The plan's yaw -> the FCU's compass heading, proved at the four cardinal points.

ROUND 16 wrote this as a TEST-ONLY statement of the conversion, with a note that nothing would be
wired until it was green and a Mission-B regression had been taken.

ROUND 88 (2026-11-07) is when it became necessary: the 2026-11-07 audit measured the flown
consequence on `missionA-kn3-01-01` -- `yaw_cmd` equalled MINUS the true compass heading of travel
(5015 samples, median 7.3 deg, rms 28.7 deg), the raw field spanned -239..+468 deg (so it was not a
heading at all), and the FCU obeyed it to within 5.9 deg.  The nose therefore pointed wrong for most
of the route, and the 45-degree-down lidar looked sideways.

The conversion now lives in ONE place, `common.frames.l_to_wire_yaw`, and the producer that was
violating the contract (`whtf/ros/planner_source_super.py`: it published SUPER's L-frame yaw verbatim)
calls it.  `bridge/mavlink_impl.send_cmd` still passes the field through -- that is the contract, and
`tools/test-s4-l-to-wire.py` pins it bit-for-bit.

This test therefore checks the FUNCTION (four cardinal points, < 1 deg) and prints, for the record,
what a pass-through would produce on the same four points.
"""
import json
import math
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "src"))
from common import frames  # noqa: E402


def main() -> int:
    k = json.load(open(os.path.join(os.path.dirname(REPO), "_stage", "calib.json")))
    calib = frames.Calib(R_WL=np.array(k["R_WL"], dtype=float), t_WL=np.array(k["t_WL"], dtype=float),
                         n_samples=int(k.get("n_samples", 0)), rms_m=float(k.get("rms_m", float("nan"))))
    print(calib.summary())
    # theta_L is the direction of travel IN L.  The four cardinal cases, worked by hand:
    #   theta_L = 0    -> travelling +x_L, which is +y in W   -> NORTH  -> heading 0
    #   theta_L = +90  -> travelling +y_L, which is -x in W   -> WEST   -> heading 270 (-90)
    #   theta_L = 180  -> travelling -x_L, which is -y in W   -> SOUTH  -> heading 180
    #   theta_L = -90  -> travelling -y_L, which is +x in W   -> EAST   -> heading 90
    cases = [(0.0, 0.0), (90.0, -90.0), (180.0, 180.0), (-90.0, 90.0)]
    bad = []
    for deg_l, deg_want in cases:
        got = frames.l_to_wire_yaw(math.radians(deg_l), calib)
        want = math.radians(deg_want)
        err = frames.wrap_pi(got - want)
        ok = abs(err) < math.radians(1.0)
        print("   theta_L %+6.1f deg -> heading %+7.2f deg (want %+7.1f)  %s"
              % (deg_l, math.degrees(got), deg_want, "OK" if ok else "*** WRONG ***"))
        if not ok:
            bad.append(deg_l)
    # and the OLD behaviour, for the record: yaw passed through untouched
    print("\n   what a pass-through gives on the same four points (the old defect):")
    for deg_l, deg_want in cases:
        err = frames.wrap_pi(math.radians(deg_l) - math.radians(deg_want))
        print("   theta_L %+6.1f deg -> wire %+7.1f deg (want %+7.1f)  error %+7.1f deg"
              % (deg_l, deg_l, deg_want, math.degrees(err)))
    if bad:
        print("\nYAW CONVERSION TEST FAILED at %s" % bad)
        return 1
    print("\nYAW CONVERSION TEST PASSED: theta_L -> compass heading, four cardinal cases, < 1 deg")
    return 0


if __name__ == "__main__":
    sys.exit(main())
