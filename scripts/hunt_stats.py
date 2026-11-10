#!/usr/bin/env python3
"""hunt_stats.py -- every 3 m/s two-lap attempt, and why it did not count.

The objective's validity test is H1..H5 PASS + W1b PASS + no abort.  This script reads every
sortie's own `accept.json` and prints one line per attempt, so the report can quote the real sample
(e.g. "1 valid in N attempts") instead of an impression.  It also prints the rtf (H1's number),
because on a busy host H1 alone can invalidate a sortie that the aircraft flew fine.

    python3 scripts/hunt_stats.py v3d v3e v3h
    python3 scripts/hunt_stats.py --all        # every v3*/mx8* sortie that has an accept.json
"""
import glob
import json
import os
import sys

MISSION = "/home/fanxin/lio_avoid/logs"
NEED = ("H1", "H2", "H3", "H4", "H5", "W1b")


def one(tag):
    p = os.path.join(MISSION, "missionA-" + tag, "accept.json")
    if not os.path.isfile(p):
        return None
    d = json.load(open(p, encoding="utf-8"))
    rows = {r["id"]: r for r in d["rows"]}
    miss = [k for k in NEED if not rows.get(k, {}).get("ok")]
    rtf = "-"
    det = (rows.get("H1", {}).get("detail") or "")
    for line in det.splitlines():
        if "rtf" in line:
            try:
                rtf = line.split("rtf")[1].split()[0]
            except IndexError:
                pass
            break
    w1b = (rows.get("W1b", {}).get("detail") or "").splitlines()
    w1b_n = w1b[0][:52] if w1b else ""
    return {"tag": tag, "valid": not miss, "miss": ",".join(miss) or "-",
            "rtf": rtf, "w1b": w1b_n}


def main(argv):
    if len(argv) > 1 and argv[1] == "--all":
        tags = sorted({os.path.basename(p).replace("missionA-", "")
                       for p in glob.glob(os.path.join(MISSION, "missionA-v3*"))
                       if os.path.isfile(os.path.join(p, "accept.json"))})
    else:
        pre = argv[1:] or ["v3"]
        tags = sorted({os.path.basename(p).replace("missionA-", "")
                       for p in glob.glob(os.path.join(MISSION, "missionA-*"))
                       if os.path.isfile(os.path.join(p, "accept.json"))
                       and any(os.path.basename(p).startswith("missionA-" + x) for x in pre)})
    rows = [r for r in (one(t) for t in tags) if r]
    print("%-14s %-6s %-5s %-22s %s" % ("tag", "valid", "rtf", "missing", "W1b"))
    good = 0
    for r in rows:
        if r["valid"]:
            good += 1
        print("%-14s %-6s %-5s %-22s %s"
              % (r["tag"], "YES" if r["valid"] else "no", r["rtf"], r["miss"], r["w1b"]))
    print()
    print("valid: %d / %d attempts  (%.0f %%)" % (good, len(rows), 100.0 * good / max(1, len(rows))))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
