#!/usr/bin/env python3
"""round87_ledger.py -- one table for every round-87 flight attempt.

Round 87 changed the tour to two laps and the speed to 3 m/s.  Ten attempts were flown across three
configurations; this script reads each sortie's own records and prints the row that says how far it
got and how it ended, so the report does not depend on memory or on a copy-paste.

Columns
    tag            sortie tag (missionA-<tag> under the harness logs)
    wire           BRIDGE_FIELDS actually used (from the bridge's own log line)
    legs           the last phase reached (leg1 .. leg13, then 'done')
    t_end          the flight's own duration in sim seconds
    truth_end      the SITL model's (x, y, z) at the end, W frame, metres
    lio_end        the LIO's (x, y, z) at the same sample, W frame, metres
    split          |truth - lio| horizontally: "estimator still with the plant" vs "they parted"
    why            the supervisor's own abort text (or 'landed')

    python3 scripts/round87_ledger.py v3l2-01-01 v3l2-01-02 ...
    python3 scripts/round87_ledger.py --all          # every v3l2/v3c5/v3pv/v3p sortie
"""
import argparse
import csv
import glob
import math
import os
import re

MISSION = "/home/fanxin/lio_avoid/logs"


def one(tag):
    d = os.path.join(MISSION, "missionA-" + tag)
    row = {"tag": tag, "wire": "?", "legs": "?", "t_end": "?", "truth_end": "?", "lio_end": "?",
           "split": "?", "why": "landed"}
    phase = os.path.join(d, "phase.csv")
    if os.path.isfile(phase):
        rows = list(csv.DictReader(open(phase, newline="")))
        if rows:
            last = rows[-1]
            legs = [r["phase"] for r in rows if r["phase"].startswith("leg")]
            row["legs"] = legs[-1] if legs else last["phase"]
            row["t_end"] = "%.1f" % (float(last["t_sim_us"]) * 1e-6)
            tx, ty, tz = (float(last["truth_x"]), float(last["truth_y"]), float(last["truth_z"]))
            lx, ly, lz = (float(last["lio_x"]), float(last["lio_y"]), float(last["lio_z"]))
            row["truth_end"] = "(%+.1f, %+.1f, %.1f)" % (tx, ty, tz)
            row["lio_end"] = "(%+.1f, %+.1f, %.1f)" % (lx, ly, lz)
            row["split"] = "%.1f" % math.hypot(tx - lx, ty - ly)
    blog = os.path.join(d, "bridge.log")
    if os.path.isfile(blog):
        txt = open(blog, encoding="utf-8", errors="replace").read()
        m = re.search(r"fields=(\w+)|the command stream reaches message 84", txt)
        # the bridge does not echo --fields; take it from the wire log's first row instead
        wpath = os.path.join(d, "wire.csv")
        if os.path.isfile(wpath):
            with open(wpath, newline="") as fh:
                rd = csv.DictReader(fh)
                for r in rd:
                    row["wire"] = r.get("fields", "?")
                    break
        elif m:
            row["wire"] = m.group(1)
    slog = os.path.join(d, "supervisor.log")
    if os.path.isfile(slog):
        for line in open(slog, encoding="utf-8", errors="replace"):
            if "ABORT:" in line:
                row["why"] = line.split("ABORT:", 1)[1].strip()
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tags", nargs="*")
    ap.add_argument("--all", action="store_true")
    a = ap.parse_args()
    tags = list(a.tags)
    if a.all:
        # round 87's own tags are <prefix>-<sortie>-<attempt>; the old `missionA-v3p-01` from an
        # earlier round matches a naive prefix glob, so the shape is checked explicitly
        pat = re.compile(r"^v3(l2|c5|pv|pvB|h2|g|s)-\d\d-\d\d$")
        for p in sorted(glob.glob(os.path.join(MISSION, "missionA-v3*"))):
            t = os.path.basename(p).replace("missionA-", "")
            if pat.match(t):
                tags.append(t)
    if not tags:
        ap.error("give tags or --all")
    rows = [one(t) for t in tags]
    w = [12, 10, 7, 8, 22, 22, 7]
    hdr = ["tag", "wire", "legs", "t_end", "truth_end", "lio_end", "split"]
    print("| " + " | ".join(h.ljust(w[i]) for i, h in enumerate(hdr)) + " | why")
    print("|" + "|".join("-" * (w[i] + 2) for i in range(len(w))) + "|---")
    for r in rows:
        print("| " + " | ".join(str(r[h]).ljust(w[i]) for i, h in enumerate(hdr)) + " | " + r["why"])
    print()
    print("split = |truth - lio| at the last sample; a few metres means the estimator was still with")
    print("the plant and the abort is a real excursion, tens of metres means they parted company")
    print("(the round-85/86 signature: the SITL model flies away while the estimate stays put).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
