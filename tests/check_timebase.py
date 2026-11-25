#!/usr/bin/env python3
"""check_timebase.py -- the D4 truth-time-base plumbing (no flight, one second).

Guards two things that cost a whole flight each to notice:
  1. `tap.csv` has BOTH clocks and a grader that compares it with the flight-side streams uses the
     EXPLICIT simulator clock (`t_sim_us`), not the recorder's monotonic `t_us`.  Using `t_us` adds a
     per-sortie constant (0.15 s normally, 1.35 s on phase4oE) and put W6's +-0.6 s fit on its edge.
  2. the mapping is written down (`truth_timebase.json` + `manifest.json["truth_timebase"]`).
"""
import csv
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import accept  # noqa: E402


def main() -> int:
    fails = []
    d = tempfile.mkdtemp(prefix="timebase-")
    tap = os.path.join(d, "tap.csv")
    with open(tap, "w") as fh:
        fh.write("t_us,t_sim_us,inst,pn,pe,pd,vn,ve,vd,qw,qx,qy,qz,wx,wy,wz,ax,ay,az\n")
        for i in range(3):
            vals = [str(1_000_000 + i * 1000), str(5_000_000 + i * 1000), "0",      # t_us t_sim inst
                    "0", "0", "-0.1",                                                # pn pe pd
                    "0", "0", "0",                                                   # vn ve vd
                    "1", "0", "0", "0",                                              # qw qx qy qz
                    "0", "0", "0",                                                   # wx wy wz
                    "0", "0", "-9.8"]                                                # ax ay az
            fh.write(",".join(vals) + "\n")
    got, err = accept.tap_world(tap)
    if got is None:
        fails.append("tap_world failed on a synthetic tap: %s" % err)
    else:
        t = got[0]
        if abs(t[0] - 5.0) > 1e-6:
            fails.append("tap_world used the RECORDER clock: first t=%.3f (expected 5.000 = t_sim_us)"
                         % t[0])

    # legacy file (no t_sim_us) must keep working unchanged
    tap2 = os.path.join(d, "tap_legacy.csv")
    with open(tap2, "w") as fh:
        fh.write("t_us,inst,pn,pe,pd,vn,ve,vd,qw,qx,qy,qz,wx,wy,wz,ax,ay,az\n")
        fh.write("1000000,0,0,0,-0.1,0,0,0,1,0,0,0,0,0,0,0,0,-9.8\n")
    got2, err2 = accept.tap_world(tap2)
    if got2 is None or abs(got2[0][0] - 1.0) > 1e-6:
        fails.append("tap_world broke on a legacy (t_us-only) tap: %s" % (err2 or got2[0][0]))

    # the mapping file + manifest block
    man = os.path.join(d, "manifest.json")
    with open(man, "w") as fh:
        json.dump({"tag": "x"}, fh)
    out = os.path.join(d, "truth_timebase.json")
    rc = subprocess.run([sys.executable, os.path.join(ROOT, "scripts", "write_timebase.py"),
                         "--out", out, "--manifest", man, "--sitl-t0-us", "100",
                         "--client-t0-us", "180", "--offset-us", "80"],
                        capture_output=True, text=True)
    if rc.returncode != 0:
        fails.append("write_timebase.py exited %d: %s" % (rc.returncode, rc.stderr.strip()[-200:]))
    else:
        blk = json.load(open(out))
        if blk.get("offset_us") != 80:
            fails.append("truth_timebase.json has offset_us=%r (expected 80)" % blk.get("offset_us"))
        if json.load(open(man)).get("truth_timebase", {}).get("offset_us") != 80:
            fails.append("manifest.json was not updated with the truth_timebase block")

    # the header the truth client writes must carry the column the grader looks for
    src = open(os.path.join(ROOT, "sim", "src", "sim_state_recv.py"), encoding="utf-8").read()
    if "TAP_CSV_HEADER" not in src or "t_us,t_sim_us" not in src:
        fails.append("sim_state_recv.py's tap header no longer carries t_us,t_sim_us")

    if fails:
        print("FAIL: time base plumbing")
        for f in fails:
            print("  - " + f)
        return 1
    print("PASS: tap.csv carries t_sim_us, accept.tap_world prefers it (legacy files unchanged), "
          "truth_timebase.json + manifest record the mapping")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
