#!/usr/bin/env python3
"""Quick check of accept.flight_rtf() against the sorties whose rtf the old gate rejected."""
import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
import _paths as _P  # noqa: E402  (repo path authority, round 21)
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import accept  # noqa: E402

D = _P.LOGS
for t in ("kx1-01-01", "kx1-01-02", "kx1-02-01", "kx1-02-02", "kwC-01-01",
          "kwC-01-02", "kw1-01-01", "kv1-02-01"):
    r = accept.flight_rtf(os.path.join(D, "missionA-" + t, "phase.csv"))
    if r is None:
        print("%-12s <no usable pair>" % t)
        continue
    rtf, desc = r
    print("%-12s rtf %.4f   %s" % (t, rtf, desc))
