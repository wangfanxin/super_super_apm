#!/usr/bin/env python3
"""refscan.py -- how many files reference each candidate, and where (cleanup evidence).

    python3 scripts/refscan.py super_lio_15m_gentle.yaml forest6x4.yaml ...
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SKIP_DIRS = {".git", "__pycache__", "_scratch", "logs", "upstream", "ws_super"}
EXT = (".py", ".sh", ".md", ".yaml", ".yml", ".json", ".rviz", ".txt")


def main(names):
    hits = {n: [] for n in names}
    for dirpath, dirnames, filenames in os.walk(ROOT):
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
        for fn in filenames:
            if not fn.endswith(EXT):
                continue
            p = os.path.join(dirpath, fn)
            rel = os.path.relpath(p, ROOT)
            try:
                txt = open(p, encoding="utf-8", errors="replace").read()
            except OSError:
                continue
            for n in names:
                if n in txt and os.path.basename(rel) != n:
                    hits[n].append(rel)
    for n in names:
        refs = hits[n]
        print("%-34s %3d refs" % (n, len(refs)))
        for r in refs[:6]:
            print("      %s" % r)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
