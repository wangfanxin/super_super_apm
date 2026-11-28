#!/usr/bin/env python3
"""recompute_metrics.py -- re-run accept.py + quality_rows.py for every retained run with the ARCHIVED argv.

Why: the scorer is code, and 2026-11-28 fixed two scorer bugs (W4 sampled every 5th truth point; S1's END
window started at the FIRST approach to the last corner).  The archived accept.json contains the argv that
produced it (`meta.scorer.argv`), so every evidence run can be re-scored identically except for the bug fix.
This script replays that argv (same paths, same flags), writes accept.json/quality.json in place, and prints
a diff table of the gated rows whose verdict changed.

USAGE: python3 scripts/recompute_metrics.py [--dry] [--limit N]
"""
import argparse, json, os, subprocess, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RUNS = "/home/fanxin/sp_runs"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry", action="store_true")
    ap.add_argument("--limit", type=int, default=0)
    a = ap.parse_args()
    dirs = sorted(d for d in os.listdir(RUNS) if d.startswith("missionA-") and os.path.isdir(os.path.join(RUNS, d)))
    if a.limit:
        dirs = dirs[:a.limit]
    changed = []
    for d in dirs:
        rd = os.path.join(RUNS, d)
        aj = os.path.join(rd, "accept.json")
        if not os.path.isfile(aj):
            print("%-40s no accept.json" % d); continue
        old = json.load(open(aj))
        argv = (old.get("meta") or {}).get("scorer", {}).get("argv") or []
        if len(argv) < 3:
            print("%-40s no archived argv" % d); continue
        cmd = [sys.executable, argv[0]] + argv[1:]
        # keep the output in place
        if "--json" in cmd:
            k = cmd.index("--json")
            if k + 1 < len(cmd):
                cmd[k + 1] = aj
        if a.dry:
            print("DRY", " ".join(cmd[:6]), "..."); continue
        r = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
        # rc=1 means "GATE: FAIL", which is a VALID result -- only rc>1 / missing output is an error
        if r.returncode not in (0, 1) or not os.path.isfile(aj):
            print("%-40s accept replay FAILED rc=%d %s" % (d, r.returncode, (r.stderr or r.stdout).strip()[-120:])); continue
        new = json.load(open(aj))
        old_rows = {x["id"]: x for x in old.get("rows", [])}
        new_rows = {x["id"]: x for x in new.get("rows", [])}
        diff = []
        for k2 in sorted(set(old_rows) & set(new_rows)):
            if bool(old_rows[k2].get("ok")) != bool(new_rows[k2].get("ok")):
                diff.append("%s: %s -> %s" % (k2, old_rows[k2].get("ok"), new_rows[k2].get("ok")))
        # quality rows (record-only rows still matter for the report)
        qcmd = [sys.executable, os.path.join(ROOT, "scripts", "quality_rows.py"), "--run", rd,
                "--json", os.path.join(rd, "quality.json")]
        if os.path.isfile(os.path.join(ROOT, "sim", "worlds", "forest_hex12.yaml")):
            qcmd += ["--world", os.path.join(ROOT, "sim", "worlds", "forest_hex12.yaml")]
        q = subprocess.run(qcmd, cwd=ROOT, capture_output=True, text=True)
        qrows = {}
        if q.returncode == 0 and os.path.isfile(os.path.join(rd, "quality.json")):
            qrows = {x["id"]: x for x in json.load(open(os.path.join(rd, "quality.json"))).get("rows", [])}
        w4 = new_rows.get("W4", {})
        print("%-40s %sW4=%s (%s) | S1/Q2/S2/R2R: %s" % (
            d, "CHANGED " if diff else "", w4.get("ok"), (w4.get("detail") or "").splitlines()[0][:60],
            ", ".join("%s=%s" % (k, qrows.get(k, {}).get("ok")) for k in ("S1", "Q2", "S2", "R2R"))))
        if diff:
            changed.append((d, diff))
    print("\n===== gated-row verdict changes: %d run(s)" % len(changed))
    for d, diff in changed:
        print("  %-40s %s" % (d, "; ".join(diff)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
