#!/usr/bin/env python3
"""Compact view of a run's accept.json: one line per row, plus the first line of the detail.

  show_gate.py RUN [--full W4,W2]

A reader who wants the whole row text can pass --full; the default is deliberately terse so the
gate can be read at a glance after every sortie.
"""
import argparse
import json
import os

ap = argparse.ArgumentParser()
ap.add_argument("run")
ap.add_argument("--full", default="", help="comma separated row ids to print in full")
a = ap.parse_args()

path = a.run if a.run.endswith(".json") else os.path.join(a.run, "accept.json")
if not os.path.exists(path):
    raise SystemExit("no accept.json at %s -- run scripts/run_accept.sh first" % path)
d = json.load(open(path, encoding="utf-8"))
full = {s.strip().upper() for s in a.full.split(",") if s.strip()}

print("run  : %s" % d.get("run"))
print("world: %s" % os.path.basename(str(d.get("world"))))
print("-" * 78)
for r in d.get("rows", []):
    ok = r.get("ok")
    mark = {True: "PASS", False: "FAIL", None: "INFO"}[ok]
    first = str(r.get("detail", "")).strip().splitlines()
    first = first[0] if first else ""
    print("[%s] %-4s %-58s %s" % (mark, r.get("id"), r.get("title", "")[:58], first[:120]))
    if r.get("id") in full:
        for ln in str(r.get("detail", "")).splitlines()[1:]:
            print("              " + ln)
print("-" * 78)
print("RESULT: %s pass, %s fail" % (d.get("pass"), d.get("fail")))
print("GATE: %s" % ("PASS" if d.get("fail") == 0 else "FAIL"))
