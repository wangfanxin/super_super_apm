#!/usr/bin/env python3
"""Print the planner's own motion envelope, one line: `max_vel max_acc max_jerk`.

WHY THIS FILE EXISTS (round 87)
-------------------------------
`fly_hex.sh` hands the bridge three feed-forward clamps
(`--ff-vel-max`, `--ff-acc-max`, `--ff-acc-slew`).  Their documented purpose is
**not** to be a new limit: they are supposed to *be* the plan's own limits, so
that a replan's acceleration step cannot become a command step at the FCU
(see the long note in `fly_hex.sh` and `docs/INTEGRATION.md` §64).

Between rounds 86 and 87 those three numbers were typed in by hand while the
config moved underneath them:

  * round 86 raised `traj_opt.boundary.max_acc` 5.0 -> 8.0, but `--ff-acc-max`
    stayed at **5.0** -- so for one whole round the bridge clipped 37 % of the
    acceleration the plan actually asked for, exactly at the corners where the
    acceleration is used to kill overshoot (`bow = v^2/a`);
  * round 87 raises `max_vel` 2.0 -> 3.0, and a `--ff-vel-max 2.5` would clip
    the 3.0 m/s PVA velocity at every waypoint -- i.e. the bridge would silently
    rewrite S1's own PVA target, which S1 forbids.

So the clamps are no longer typed in: they are read from the same YAML the
planner loads, by this script, at arm start.  A stale clamp is now impossible,
and `scripts/spec_check.py` re-runs this script and compares its output with the
config values it already asserts (row "S1 前馈钳位 = 规划器上界").

USAGE
    python3 scripts/cfg_envelope.py config/super_planner_hex.yaml
    -> "3.0 8.0 40.0"

Exits 1 with a message on stderr if a key is missing, so the arm can fall back
loudly instead of flying with a clamp of 0 (0 means "no clamp" in the bridge).
"""
import io
import os
import re
import sys

KEYS = ("max_vel", "max_acc", "max_jerk")


def read_envelope(path):
    txt = io.open(path, encoding="utf-8").read()
    out = []
    for k in KEYS:
        m = re.search(r"^[ \t]*%s:[ \t]*([0-9.eE+-]+)" % k, txt, re.M)
        if m is None:
            raise SystemExit("cfg_envelope: %s has no `%s:` line" % (path, k))
        out.append(float(m.group(1)))
    return out


def main(argv):
    if len(argv) != 2:
        raise SystemExit("usage: cfg_envelope.py <planner.yaml>")
    path = argv[1]
    if not os.path.isfile(path):
        raise SystemExit("cfg_envelope: no such config: %s" % path)
    v, a, j = read_envelope(path)
    print("%.6g %.6g %.6g" % (v, a, j))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
