#!/usr/bin/env python3
"""peek_world.py -- top-level blocks of a world file (what the template contributes)."""
import sys

import yaml

for p in sys.argv[1:]:
    w = yaml.safe_load(open(p, encoding="utf-8"))
    print("== %s" % p)
    print("   keys:", sorted(w.keys()))
    for k in ("world", "arena", "frame", "sim", "sensor", "spawn"):
        if k in w:
            print("   %-7s: %s" % (k, str(w[k])[:400]))
