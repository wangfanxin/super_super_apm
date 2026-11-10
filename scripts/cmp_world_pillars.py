#!/usr/bin/env python3
"""cmp_world_pillars.py -- do two world files have the same pillars (ignoring the header comment)?

The generator writes a header that embeds its own command line (`--out /tmp/...`), so two runs that
draw the SAME pillars still produce different SHA-256.  What must not change is the pillar list.
"""
import sys

import yaml

a = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))
b = yaml.safe_load(open(sys.argv[2], encoding="utf-8"))
pa = [(round(float(q["x"]), 6), round(float(q["y"]), 6), float(q["r"])) for q in a["pillars"]]
pb = [(round(float(q["x"]), 6), round(float(q["y"]), 6), float(q["r"])) for q in b["pillars"]]
print("%s: %d pillars" % (sys.argv[1], len(pa)))
print("%s: %d pillars" % (sys.argv[2], len(pb)))
same = pa == pb
print("pillars identical:", same)
if not same and len(pa) == len(pb):
    d = [(i, x, y) for i, (x, y) in enumerate(zip(pa, pb)) if x != y]
    print("first differences:", d[:5], "... total", len(d))
