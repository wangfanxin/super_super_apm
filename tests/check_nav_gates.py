#!/usr/bin/env python3
"""check_nav_gates.py -- the explicit external-nav refusal table (no flight, no ROS).

Phase 3 closeout: the bridge's ODOMETRY (#331) refusals are now a NAMED decision table
(`bridge_nodes/mavlink/nav_gates.refusal_reason`) instead of an implicit mix of conditions, and the sortie's
`nav.csv` records the reason code.  This test pins the table and the defaults the delivered flights use
(pass-through), so a future edit cannot silently re-order the checks or turn a gate on by accident.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "bridge_nodes"))
from mavlink import nav_gates  # noqa: E402


def main() -> int:
    fails = []
    R = nav_gates.refusal_reason

    # delivered defaults: no age gate, no quality floor, physical gate passes -> send
    if R(True, 99.0, 100, 0, 0.0) != 0:
        fails.append("delivered defaults (age off, floor 0) must SEND a 99 s old sample")
    # physical gate wins over everything
    if R(False, 0.0, 100, 0, 0.0) != 1:
        fails.append("an unreachable pose must be reported as reason 1")
    # age gate only when enabled, and strictly above the limit
    if R(True, 0.31, 100, 0, 0.3) != 2:
        fails.append("a sample older than --nav-max-age-s must be reason 2")
    if R(True, 0.29, 100, 0, 0.3) != 0:
        fails.append("a sample inside --nav-max-age-s must still be sent")
    # quality floor
    if R(True, 0.0, 40, 50, 0.0) != 3:
        fails.append("quality below the floor must be reason 3")
    if R(True, 0.0, 50, 50, 0.0) != 0:
        fails.append("quality AT the floor must still be sent (>= semantics)")
    # order: physical > stale > quality
    if R(False, 99.0, 1, 50, 0.3) != 1:
        fails.append("reason order must be physical > stale > quality")
    if R(True, 99.0, 1, 50, 0.3) != 2:
        fails.append("stale must be reported before quality")
    # the descriptions are part of the contract (they are printed on status lines)
    for code, word in ((0, "sent"), (1, "physical"), (2, "stale"), (3, "quality")):
        if word not in nav_gates.describe(code):
            fails.append("describe(%d) lost the word %r" % (code, word))

    if fails:
        print("FAIL: nav gate table")
        for f in fails:
            print("  - " + f)
        return 1
    print("PASS: nav gate table (defaults are pass-through; physical > stale > quality; reason codes named)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
