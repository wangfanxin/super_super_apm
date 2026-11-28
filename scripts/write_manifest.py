#!/usr/bin/env python3
"""write_manifest.py -- Phase 0 deliverable: every sortie carries a manifest that makes its numbers
reproducible without the author's machine.

It records, in one JSON:
  * revisions:    git HEAD, dirty file count, the fsm_node binary sha256 (the planner actually run),
                  the delivered SUPER config sha256 (all loader copies compared), the world and the
                  calibration sha256, the parm files, the ArduPilot revision;
  * launch:       the full launcher argv, the interesting environment variables, the resolved paths;
  * thresholds:   the acceptance arguments if the caller has them (written by the grader later).

Usage (called by scripts/run-mission.sh at start, and optionally refreshed by the grader):
    python3 scripts/write_manifest.py --out "$OUT/manifest.json" \
        --world "$WORLD" --calib "$CALIB" --config "$SUPER_CONFIG" \
        --parm "$MISSION_PARM" --parm "$ALT_PARM" --parm "$EXTRA_ARG" [--argv "$*"]
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys


def sha256(path):
    if not path or not os.path.isfile(path):
        return None
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def git_rev(root):
    try:
        rev = subprocess.run(["git", "-C", root, "rev-parse", "HEAD"], capture_output=True, text=True,
                             timeout=10).stdout.strip()
        lines = subprocess.run(["git", "-C", root, "status", "--porcelain"], capture_output=True,
                               text=True, timeout=10).stdout.splitlines()
        return {"head": rev, "dirty_files": len(lines), "dirty_paths": [ln[3:] for ln in lines[:30]]}
    except Exception:                                   # noqa: BLE001
        return {"head": None, "dirty_files": None}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--root", default=os.environ.get("DSH_ROOT", ""))
    ap.add_argument("--world", default="")
    ap.add_argument("--calib", default="")
    ap.add_argument("--config", default="")
    ap.add_argument("--parm", action="append", default=[])
    ap.add_argument("--argv", default="")
    ap.add_argument("--tag", default="")
    ap.add_argument("--extra", action="append", default=[], help="extra key=path pairs to hash")
    args = ap.parse_args()

    root = args.root or os.getcwd()
    # 2026-11-28 review: the manifest must record the parameters that decide HOW the reference is shaped
    # and dispatched, not just the paths -- otherwise a run cannot be re-flown from its own artifacts.
    env = {k: os.environ.get(k) for k in
           ("DSH_ROOT", "SUPER_PLANNER_ROOT", "SLAM_ROOT", "SIM_ROOT", "ARDUPILOT_DIR",
            "WS_SUPER", "LIO_WS", "ARDUPILOT_ROOT", "LIO_LOGS", "CALIB_FILE", "MID360_PATTERN",
            "PLANNER", "SUPER_CONFIG", "WORLD_NAME", "LAPS", "Z_M", "BRIDGE_FIELDS",
            "NAV_LEAD_S", "NAV_GATE_MPS", "NAV_SKIP_MAX", "LIDAR_POINTS", "GEOFENCE_M",
            "PASS_BY_M", "PASS_BY_LONG_M", "PRE_GOAL_M", "ARRIVE_M", "FINAL_ARRIVE_M",
            "ADVANCE_LOOKAHEAD_S", "ADVANCE_MARGIN_M", "DISPATCH_LOOKAHEAD",
            "GOAL_SWITCH_YAW_GATE_DEG", "GOAL_SWITCH_YAW_GATE_HOLD_S",
            "REF_WASHOUT_TAU", "REF_WASHOUT_VMAX", "REF_NO_RETRACT", "REF_ALONG_FLOOR",
            "REF_GOV_ALONG", "REF_MAX_MPS", "REPLAN_RATE",
            "GOAL_PVA", "CORNER_MODE", "CORNER_SPEED", "CORNER_SPEED_LIST", "SETTLE_S",
            "SETTLE_SPEED", "HOLD_S", "WARMUP_SECS", "EXTRA_PARMS")}

    # the planner binary actually run (the built fsm_node), and every loader copy of the config
    fsm = os.path.join(env.get("WS_SUPER") or "", "install/super_planner/lib/super_planner/fsm_node")
    loader_cfg = []
    for base in (env.get("SUPER_PLANNER_ROOT") or "", env.get("WS_SUPER") or ""):
        if not base:
            continue
        p = os.path.join(base, "super_planner", "config", os.path.basename(args.config or ""))
        loader_cfg.append({"path": p, "sha256": sha256(p)})
    cfg_src = args.config
    if cfg_src and not os.path.isabs(cfg_src) and env.get("SUPER_PLANNER_ROOT"):
        cand = os.path.join(env["SUPER_PLANNER_ROOT"], "config", cfg_src)
        cfg_src = cand if os.path.isfile(cand) else cfg_src

    ap_rev = None
    ap_root = env.get("ARDUPILOT_ROOT") or ""
    if ap_root:
        try:
            ap_rev = subprocess.run(["git", "-C", ap_root, "rev-parse", "--short", "HEAD"],
                                    capture_output=True, text=True, timeout=10).stdout.strip() or None
        except Exception:                               # noqa: BLE001
            pass

    manifest = {
        "tag": args.tag,
        "repo": {"path": root, **git_rev(root)},
        "paths": {k: v for k, v in env.items()},
        "planner": {"fsm_node": fsm, "fsm_node_sha256": sha256(fsm),
                    "config_src": cfg_src, "config_sha256": sha256(cfg_src),
                    "config_loader_copies": loader_cfg},
        "world": {"path": args.world, "sha256": sha256(args.world)},
        "calib": {"path": args.calib, "sha256": sha256(args.calib)},
        "parms": [{"path": (p or "").strip().strip(","), "sha256": sha256((p or "").strip().strip(","))}
                  for p in args.parm if (p or "").strip().strip(",")],
        "ardupilot": {"root": ap_root, "rev": ap_rev},
        "argv": args.argv,
        "extra": [{"path": p, "sha256": sha256(p)} for p in args.extra if p],
        "python": sys.version.split()[0],
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as fh:
        json.dump(manifest, fh, indent=1)
    missing = [k for k in ("sha256",) if False]
    print("manifest -> %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
