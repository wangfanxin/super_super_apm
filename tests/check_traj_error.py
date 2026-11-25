#!/usr/bin/env python3
"""check_traj_error.py -- the Phase-5 extras (record-only APE/RPE + manifest completeness), no flight.

Two things are pinned here:
  1. scripts/traj_error.py recovers a KNOWN rigid transform exactly (APE ~ 0 after alignment) and its
     APE/RPE grow when the estimate really drifts -- so the numbers it prints mean something;
  2. scripts/accept.py's extras block reports manifest completeness but can never change the 15 rows or the
     gate (it is record-only until the owner approves the P0 criterion change).
"""
import csv
import importlib.util
import json
import math
import os
import runpy
import sys
import tempfile

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import traj_error  # noqa: E402


def write_run(d, *, rot_deg, shift, drift, noise=0.0, manifest="complete"):
    os.makedirs(d, exist_ok=True)
    n = 400
    t = np.linspace(10.0, 30.0, n)                     # simulated seconds
    th = np.linspace(0.0, 2 * math.pi, n)
    truth = np.column_stack([6 * np.cos(th), 6 * np.sin(th), 2.0 + 0.2 * np.sin(2 * th)])
    # tap.csv: NED columns; truth W = (E, N, U) -> pe=E, pn=N, pd=-U
    with open(os.path.join(d, "tap.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t_us", "t_sim_us", "inst", "pn", "pe", "pd", "vn", "ve", "vd",
                    "qw", "qx", "qy", "qz", "wx", "wy", "wz", "ax", "ay", "az"])
        for k in range(n):
            tu = int(t[k] * 1e6)
            w.writerow([tu, tu, 0, truth[k, 1], truth[k, 0], -truth[k, 2],
                        0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, -9.8])
    # lio.csv: L frame, i.e. W = (-y_l, x_l, z_l) -> x_l = N, y_l = -E
    R = np.array([[math.cos(math.radians(rot_deg)), -math.sin(math.radians(rot_deg)), 0],
                  [math.sin(math.radians(rot_deg)), math.cos(math.radians(rot_deg)), 0],
                  [0, 0, 1]])
    est = (R @ truth.T).T + np.asarray(shift, float)
    if drift:
        est[:, 0] += drift * (t - t[0]) / (t[-1] - t[0])
    if noise:
        rng = np.random.default_rng(7)
        est = est + rng.normal(0, noise, est.shape)
    with open(os.path.join(d, "lio.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t_us", "x", "y", "z", "qw", "qx", "qy", "qz", "vx", "vy", "vz",
                    "wx", "wy", "wz", "frame_id", "child_frame_id"])
        for k in range(n):
            w.writerow([int(t[k] * 1e6), est[k, 1], -est[k, 0], est[k, 2],
                        1, 0, 0, 0, 0, 0, 0, 0, 0, 0, "lio_world", "base_link"])
    # setpoints.csv with an fcu stream = truth (so it aligns to zero)
    with open(os.path.join(d, "setpoints.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t_wall", "t_sim_us", "cmd_x", "cmd_y", "cmd_z", "vel_x", "vel_y", "vel_z",
                    "acc_x", "acc_y", "acc_z", "fcu_x", "fcu_y", "fcu_z", "fcu_vx", "fcu_vy",
                    "fcu_vz", "lio_x", "lio_y", "lio_z", "yaw_cmd"])
        for k in range(n):
            tu = int(t[k] * 1e6)
            w.writerow([t[k], tu, truth[k, 0], truth[k, 1], truth[k, 2], 0, 0, 0, 0, 0, 0,
                        truth[k, 0], truth[k, 1], truth[k, 2], 0, 0, 0, 0, 0, 0, 0])
    with open(os.path.join(d, "phase.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t_wall", "t_sim_us", "phase", "cmd_x", "cmd_y", "cmd_z",
                    "lio_x", "lio_y", "lio_z", "truth_x", "truth_y", "truth_z"])
        w.writerow([t[0], int(t[0] * 1e6), "takeoff", 0, 0, 0, 0, 0, 0, 0, 0, 0])
        w.writerow([t[-1], int(t[-1] * 1e6), "land", 0, 0, 0, 0, 0, 0, 0, 0, 0])
    if manifest != "absent":
        man = {"tag": "unit", "repo": {"head": "0" * 40},
               "world": {"sha256": "x"}, "calib": {"sha256": "x"},
               "planner": {"fsm_node_sha256": "x", "config_sha256": "x"},
               "argv": "run-mission x", "ardupilot": {"rev": "abc"}}
        if manifest == "complete":
            man["truth_timebase"] = {"offset_us": 123}
        with open(os.path.join(d, "manifest.json"), "w") as fh:
            json.dump(man, fh)


def main() -> int:
    fails = []
    d = tempfile.mkdtemp(prefix="trajerr-")

    # (a) exact rigid transform -> APE ~ 0 and the alignment reports the yaw we applied
    d1 = os.path.join(d, "exact")
    write_run(d1, rot_deg=30.0, shift=(2.0, -1.0, 0.5), drift=0.0)
    m = traj_error.metrics(d1, align=True)
    lio = m["streams"]["lio"]
    if lio["ape"]["rmse"] > 5e-3:
        fails.append("exact rigid transform: APE rmse %.4f m (expected ~0)" % lio["ape"]["rmse"])
    # alignment reports the yaw of the transform FROM the estimate TO the truth, i.e. -30 deg here
    if abs(lio["align"]["yaw_deg"] + 30.0) > 0.2:
        fails.append("exact rigid transform: recovered yaw %.2f (expected -30)" % lio["align"]["yaw_deg"])
    if lio["rpe"]["rmse"] > 5e-3:
        fails.append("exact rigid transform: RPE rmse %.4f (expected ~0)" % lio["rpe"]["rmse"])
    fcu = m["streams"]["fcu"]
    if fcu["ape"]["rmse"] > 5e-3:
        fails.append("fcu=truth: APE rmse %.4f (expected ~0)" % fcu["ape"]["rmse"])

    # (b) a real drift must show up as APE growth
    d2 = os.path.join(d, "drift")
    write_run(d2, rot_deg=0.0, shift=(0, 0, 0), drift=1.0)
    m2 = traj_error.metrics(d2, align=True)
    if m2["streams"]["lio"]["ape"]["rmse"] < 0.1:
        fails.append("a 1 m drift produced APE %.3f m -- the metric cannot see drift"
                     % m2["streams"]["lio"]["ape"]["rmse"])

    # (c) raw (un-aligned) mode must see the calibration offset
    raw = traj_error.metrics(d1, align=False)
    if raw["streams"]["lio"]["ape"]["rmse"] < 1.0:
        fails.append("--no-align hid the 2,-1,0.5 m calibration offset (APE %.2f)"
                     % raw["streams"]["lio"]["ape"]["rmse"])

    # (d) accept.py's record-only extras: manifest completeness + never a row
    sys.path.insert(0, os.path.join(ROOT, "scripts"))
    spec = importlib.util.spec_from_file_location("accept_under_test", os.path.join(ROOT, "scripts", "accept.py"))
    acc = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(acc)
    import types
    args = types.SimpleNamespace(run=d1, no_extras=False)
    ex = acc.extras_block(args)
    if not ex or "manifest" not in ex or "trajectory" not in ex:
        fails.append("accept.extras_block did not produce manifest+trajectory")
    else:
        if ex["manifest"]["missing"]:
            fails.append("complete manifest reported missing fields: %s" % ex["manifest"]["missing"])
        if ex["trajectory"]["streams"]["lio"]["ape"]["rmse"] > 5e-3:
            fails.append("accept extras' APE does not match traj_error")
    # incomplete manifest -> the fields are reported, never raised
    d3 = os.path.join(d, "partial")
    write_run(d3, rot_deg=0.0, shift=(0, 0, 0), drift=0.0, manifest="partial")
    ex3 = acc.extras_block(types.SimpleNamespace(run=d3, no_extras=False))
    if "truth_timebase.offset_us" not in ex3["manifest"]["missing"]:
        fails.append("a missing truth_timebase was not reported: %s" % ex3["manifest"]["missing"])
    if ex3["manifest"]["legacy_timebase"] is not True:
        fails.append("legacy_timebase flag not set for a manifest without truth_timebase")
    # absent manifest -> recorded as absent, not an exception
    d4 = os.path.join(d, "nomanifest")
    write_run(d4, rot_deg=0.0, shift=(0, 0, 0), drift=0.0, manifest="absent")
    ex4 = acc.extras_block(types.SimpleNamespace(run=d4, no_extras=False))
    if ex4["manifest"]["present"]:
        fails.append("absent manifest reported as present")

    if fails:
        print("FAIL: trajectory extras")
        for f in fails:
            print("  - " + f)
        return 1
    print("PASS: APE/RPE recover a known transform (yaw -30 deg est->truth, APE<5mm) and see drift; extras "
          "block reports manifest completeness record-only")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
