#!/usr/bin/env python3
"""world_summary.py -- one-paragraph description of a world file (the square one included).

    python3 scripts/world_summary.py worlds/square15_forest.yaml
"""
import sys

import yaml

for path in sys.argv[1:]:
    w = yaml.safe_load(open(path, encoding="utf-8"))
    m = w.get("mission") or {}
    pil = w.get("pillars") or []
    print("== %s" % path)
    print("   pillars      : %d  (r = %s)" % (len(pil), {q.get("r") for q in pil}))
    print("   shape        : %s   laps: %s   alt: %s m   arrive: %s m"
          % (m.get("shape"), m.get("laps"), m.get("takeoff_alt"), m.get("arrive_sphere_m")))
    cs = m.get("corners") or []
    print("   corners      : %d  first four: %s" % (len(cs), cs[:4]))
    if pil:
        xs = [float(q["x"]) for q in pil]
        ys = [float(q["y"]) for q in pil]
        print("   pillar extent: x[%.1f, %.1f] y[%.1f, %.1f]"
              % (min(xs), max(xs), min(ys), max(ys)))
