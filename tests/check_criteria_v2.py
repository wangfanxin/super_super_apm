#!/usr/bin/env python3
"""check_criteria_v2.py -- the opt-in P0 criteria switch must not move the DEFAULT verdicts.

Phase: P0 preparation (docs/P0-CRITERIA-PROPOSAL.md).  `accept.py --criteria-v2` exists so the owner can be
shown exactly what the proposed semantics would change, but the delivered criterion must stay v1 until it is
approved.  This test pins that:

  * with NO flag the 15 rows are the same ids and the same ok/None values as the sortie's stored
    `accept.json` (for the rows the stored file recorded; W0 is informational);
  * with `--criteria-v2` the row set is unchanged and ONLY H2/W4/W6/H6 may flip (the four the proposal
    touches); every other row must keep its v1 verdict;
  * the JSON tags `criteria: "v1-default"` vs `"v2-proposed"` so an evidence file cannot be misread.

SKIPs (exit 0) when there is no stored sortie to replay.
"""
import glob
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import _paths as _P  # noqa: E402  (repo path authority: LIO_LOGS or <repo>/.runs)

LOGS = _P.LOGS
ALLOWED_FLIPS = {"H2", "W4", "W6", "H6"}


def _stored_argv(run):
    """The EXACT accept.py argv the flight used (manifest-style `scorer.argv` in the stored JSON), with the
    run dir and --json path made generic.  Returns the args AFTER the executable, or None."""
    try:
        stored = json.load(open(os.path.join(run, "accept.json")))
        argv = (stored.get("meta") or {}).get("scorer", {}).get("argv") or []
    except Exception:                                   # noqa: BLE001
        return None
    if not argv or not any(str(a).endswith("accept.py") for a in argv):
        return None
    # argv[0] is the script path; keep it, substitute --run and --json values
    out = [str(argv[0])]
    i = 1
    while i < len(argv):
        a = str(argv[i])
        if a == "--run" and i + 1 < len(argv):
            out += ["--run", run]
            i += 2
            continue
        if a == "--json" and i + 1 < len(argv):
            out += ["--json", "@OUT@"]
            i += 2
            continue
        out.append(a)
        i += 1
    if "--json" not in out:
        out += ["--json", "@OUT@"]
    return out


def run_accept(run, world, extra, out):
    args = _stored_argv(run)
    if args is not None:
        cmd = [sys.executable] + [(out if a == "@OUT@" else a) for a in args]
    else:
        cmd = [sys.executable, os.path.join(ROOT, "scripts", "accept.py"),
               "--run", run, "--json", out]
        if world:
            cmd += ["--world", world, "--alt", "2.0"]
    cmd += extra
    env = dict(os.environ)
    env.setdefault("PYTHONPATH", "")
    return subprocess.run(cmd, capture_output=True, text=True, cwd=ROOT, env=env)


def main() -> int:
    d = tempfile.mkdtemp(prefix="critv2-")
    runs = sorted(glob.glob(os.path.join(LOGS, "missionA-*")))
    pick = None
    best_m = -1.0
    for run in runs:
        if os.path.isfile(os.path.join(run, "accept.json")) and \
                os.path.isfile(os.path.join(run, "tap.csv")) and \
                os.path.isfile(os.path.join(run, "setpoints.csv")):
            m = os.path.getmtime(os.path.join(run, "accept.json"))
            if m > best_m:
                pick, best_m = run, m
    if not pick:
        print("PASS: criteria-v2 switch (SKIP -- no stored sortie to replay)")
        return 0
    stored = json.load(open(os.path.join(pick, "accept.json")))
    world = None
    # the stored sortie's world comes from the manifest when recorded
    try:
        man = json.load(open(os.path.join(pick, "manifest.json")))
        world = (man.get("world") or {}).get("path")
    except Exception:                                   # noqa: BLE001
        pass
    if world and not os.path.isabs(world):
        world = os.path.join(ROOT, world)
    if not (world and os.path.isfile(world)):
        world = os.path.join(ROOT, "sim", "worlds", "forest_hex12.yaml")
    fails = []
    v1j, v2j = os.path.join(d, "v1.json"), os.path.join(d, "v2.json")
    p1 = run_accept(pick, world, [], v1j)
    if p1.returncode not in (0, 1):                     # 1 = some row failed, which is fine
        print("FAIL: v1 accept crashed: %s" % p1.stderr.strip()[-300:])
        return 1
    p2 = run_accept(pick, world, ["--criteria-v2"], v2j)
    if p2.returncode not in (0, 1):
        print("FAIL: v2 accept crashed: %s" % p2.stderr.strip()[-300:])
        return 1
    a = json.load(open(v1j))
    b = json.load(open(v2j))
    if a.get("criteria") != "v1-default":
        fails.append("v1 JSON is not tagged criteria=v1-default (%r)" % a.get("criteria"))
    if b.get("criteria") != "v2-proposed":
        fails.append("v2 JSON is not tagged criteria=v2-proposed (%r)" % b.get("criteria"))
    ra = {r["id"]: r.get("ok") for r in a["rows"]}
    rb = {r["id"]: r.get("ok") for r in b["rows"]}
    if set(ra) != set(rb):
        fails.append("row ids changed between v1 and v2: %s" % (set(ra) ^ set(rb)))
    for k in sorted(set(ra) & set(rb)):
        if ra[k] != rb[k] and k not in ALLOWED_FLIPS:
            fails.append("row %s flipped %s -> %s but is NOT part of the P0 proposal"
                         % (k, ra[k], rb[k]))
    stored_ok = {r["id"]: r.get("ok") for r in stored.get("rows", [])}
    n_cmp = 0
    for k, ok in stored_ok.items():
        if k in ("W0",):                                # informational
            continue
        if k not in ra:
            continue
        n_cmp += 1
        if ok is not None and ra[k] != ok:
            fails.append("DEFAULT v1 verdict changed for %s: stored=%s fresh=%s" % (k, ok, ra[k]))
    if n_cmp < 10:
        fails.append("only %d rows compared with the stored accept.json" % n_cmp)
    if fails:
        print("FAIL: criteria-v2 switch")
        for f in fails:
            print("  - " + f)
        return 1
    flips = [k for k in sorted(set(ra) & set(rb)) if ra[k] != rb[k]]
    print("PASS: criteria-v2 is opt-in (default v1 matches %s on %d rows; v2 flips %s only)"
          % (os.path.basename(pick), n_cmp, flips or "nothing"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
