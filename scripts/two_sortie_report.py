#!/usr/bin/env python3
"""two_sortie_report.py -- the two sorties side by side, from their own accept.json.

Round 87's objective asks for TWO valid 3 m/s two-lap sorties, each producing the full eleven rows.
This script prints that side-by-side table (markdown, ready to paste into docs/INTEGRATION.md) and
applies the objective's own validity test to each: H1..H5 PASS and W1b PASS and no abort.

    python3 scripts/two_sortie_report.py v3d-01-01 v3h17-01-01
"""
import json
import os
import sys

MISSION = "/home/fanxin/lio_avoid/logs"
IDS = ["H1", "H2", "H3", "H4", "H5", "W0", "W0b", "W1", "W1b", "W2", "W3", "W4", "W5", "W6"]


def load(tag):
    p = os.path.join(MISSION, "missionA-" + tag, "accept.json")
    if not os.path.isfile(p):
        raise SystemExit("no accept.json for %s" % tag)
    d = json.load(open(p, encoding="utf-8"))
    return {r["id"]: r for r in d["rows"]}


def cell(row):
    if row is None:
        return "-"
    ok = row.get("ok")
    mark = "**PASS**" if ok else ("FAIL" if ok is False else "INFO")
    det = (row.get("detail") or "").strip().splitlines()
    first = det[0] if det else ""
    if len(first) > 96:
        first = first[:93] + "..."
    return "%s %s" % (mark, first.replace("|", "/"))


def main(argv):
    if len(argv) != 3:
        raise SystemExit("usage: two_sortie_report.py TAG1 TAG2")
    t1, t2 = argv[1], argv[2]
    r1, r2 = load(t1), load(t2)
    print("| 行 | `%s` | `%s` |" % (t1, t2))
    print("|---|---|---|")
    for k in IDS:
        print("| %s | %s | %s |" % (k, cell(r1.get(k)), cell(r2.get(k))))
    print()

    def valid(rows):
        miss = [k for k in ("H1", "H2", "H3", "H4", "H5", "W1b") if not rows.get(k, {}).get("ok")]
        return (not miss), miss

    for tag, rows in ((t1, r1), (t2, r2)):
        ok, miss = valid(rows)
        print("%s: %s%s" % (tag, "VALID (H1-H5 + W1b PASS)" if ok else "NOT VALID, missing ",
                            "" if ok else ",".join(miss)))
    both = valid(r1)[0] and valid(r2)[0]
    print()
    print("objective item 3: %s" % ("MET for these two sorties" if both else "NOT met yet"))
    return 0 if both else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
