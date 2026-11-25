#!/usr/bin/env python3
"""write_timebase.py -- record the EXPLICIT truth time-base mapping of a sortie (D4).

The SIM_STATE truth client reconstructs its sample times on its OWN monotonic clock and maps them onto
the simulator clock with one number: `--stamp-offset-us` = (client start wall us - SITL first-model-step
wall us).  Everything on the flight side (LiDAR cloud stamps, LIO odometry, setpoints.csv `t_sim_us`) is
on the SIM clock, while `tap.csv`'s legacy `t_us` column is on the RECORDER clock.  Without this file a
grader that compares the two (W6!) silently mixes clocks by that offset -- measured 0.15 s on a normal
sortie and 1.35 s on phase4oE, which is exactly what pushed W6's +-0.6 s fit onto its search edge.

This script writes `truth_timebase.json` next to the sortie and adds the same block to `manifest.json`:

    {"t_sitl_step0_us": ..., "t_client_start_us": ..., "offset_us": ...,
     "tap_us_column": "recorder monotonic", "t_sim_us_column": "t_sim = t_us + offset_us"}

Usage (called by scripts/run-mission.sh):
    python3 scripts/write_timebase.py --out "$OUT/truth_timebase.json" \
        --manifest "$OUT/manifest.json" --sitl-t0-us N --client-t0-us M --offset-us K
"""
from __future__ import annotations

import argparse
import json
import os


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--manifest", default="")
    ap.add_argument("--sitl-t0-us", type=int, default=0)
    ap.add_argument("--client-t0-us", type=int, default=0)
    ap.add_argument("--offset-us", type=int, required=True)
    a = ap.parse_args()
    block = {
        "t_sitl_step0_us": int(a.sitl_t0_us),
        "t_client_start_us": int(a.client_t0_us),
        "offset_us": int(a.offset_us),
        "meaning": "t_sim_us = t_us + offset_us; the SIM_STATE client publishes /sim/state on t_sim_us "
                   "and tap.csv carries both columns",
    }
    with open(a.out, "w") as fh:
        json.dump(block, fh, indent=2)
        fh.write("\n")
    if a.manifest and os.path.isfile(a.manifest):
        try:
            with open(a.manifest) as fh:
                man = json.load(fh)
            man["truth_timebase"] = block
            with open(a.manifest, "w") as fh:
                json.dump(man, fh, indent=2)
                fh.write("\n")
        except Exception as exc:                        # noqa: BLE001
            print("write_timebase: could not update manifest %s: %s" % (a.manifest, exc))
    print("write_timebase: offset_us=%d -> %s" % (a.offset_us, a.out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
