#!/usr/bin/env python3
"""fingerprint.py -- print everything a frozen-map gate needs to assert, from the world file itself.

Round 88: the SPEC map changed from `forest_v2_hex.yaml` (250 pillars, 5 m hexagon, [-8,8]^2) to
`forest_hex12.yaml` (522 pillars, 10 m hexagon, [-12,12]^2, 3x3 waypoint keep-out).  Rather than
hand-copy numbers into check_frozen.py, this prints them in the exact shape the gate uses.

    python3 scripts/fingerprint.py worlds/forest_hex12.yaml
"""
import hashlib
import io
import sys

import yaml

for path in sys.argv[1:]:
    raw = open(path, "rb").read()
    w = yaml.safe_load(io.open(path, encoding="utf-8"))
    mis = w["mission"]
    v2 = w.get("v2") or {}
    n_lap = int(mis.get("laps", 1) or 1)
    cs = mis["corners"][: len(mis["corners"]) // n_lap]
    print("== %s" % path)
    print('    "sha256": "%s",' % hashlib.sha256(raw).hexdigest())
    print('    "bytes": %d,' % len(raw))
    print('    "pillars": %d,' % len(w["pillars"]))
    print('    "laps": %d,' % n_lap)
    print('    "z_m": %.1f,' % float(mis.get("takeoff_alt", 0)))
    print('    "radius_m": %.3f,' % float(mis.get("radius_m", 0) or 0))
    print('    "corners": [%s],' % ", ".join("(%.9f, %.9f)" % (float(c[0]), float(c[1])) for c in cs))
    print('    "keepout_cells": %d,' % len(v2.get("keepout_cells_empty") or []))
    print('    "keepout_neigh": %s,' % v2.get("keepout_neigh"))
    print('    "arena_walls": %s,' % ((w.get("arena") or {}).get("walls")))
    print('    "generator": %r,' % (io.open(path, encoding="utf-8").readline().strip()))
    print()
