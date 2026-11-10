#!/usr/bin/env python3
"""reversal_table.py -- one row per sortie for the "mid-leg reversal" question.

Every sortie in the A/B is scored the same way, from its own `setpoints.csv` (ONE clock: the recorder's
`t_wall`, which is a clean 50 Hz; `t_sim_us` is quantised to 5 ms and must never be a rate denominator):

  REV m / n     the vehicle moving BACKWARD along the reference's own direction: total metres retraced
                and the number of episodes (>=3 consecutive ticks, >=0.15 m each).  Same definition as
                `backtrack.py`, but on the FCU's estimate so every row uses one clock.
  along p5/50/95   along-track error (vehicle - command), metres.  A negative p50 is the standing
                "the vehicle is behind the reference" bias this round is trying to remove.
  lat p50/p90   lateral error, metres (should not change: the reversal is along-track).
  dips          reference speed < 0.25 m/s while > 1.5 m from any corner (mid-leg stalls).
  seams / step  trajectory installs (trajectory_id changes) and the MEAN speed step across them.
  W1b / W4 / W6 the acceptance rows, read from `accept.json` when it exists.

    python3 scripts/reversal_table.py kv1-02-01 kw1-01-01 kw2-01-01 ...
"""
import csv
import json
import os
import sys

import numpy as np

ROOT = "/home/fanxin/lio_avoid/logs"
CORNERS = [(10.0, 0.0), (5.0, 8.66), (-5.0, 8.66), (-10.0, 0.0), (-5.0, -8.66), (5.0, -8.66)]


def load(tag):
    d = os.path.join(ROOT, "missionA-" + tag)
    rows = list(csv.DictReader(open(os.path.join(d, "setpoints.csv"))))
    g = lambda n: np.array([float(r[n]) for r in rows])
    return {k: g(k) for k in ("t_wall", "t_sim_us", "cmd_x", "cmd_y", "cmd_z", "vel_x", "vel_y",
                              "vel_z", "fcu_x", "fcu_y", "fcu_z", "fcu_vx", "fcu_vy")}


def metrics(tag):
    d = load(tag)
    tw = d["t_wall"] - d["t_wall"][0]
    dt = np.diff(tw)
    dt[dt <= 0] = np.nan
    cmd = np.stack([d["cmd_x"], d["cmd_y"]], 1)
    fcu = np.stack([d["fcu_x"], d["fcu_y"]], 1)
    vel = np.stack([d["vel_x"], d["vel_y"]], 1)
    n = np.linalg.norm(vel, axis=1)
    ok = n > 0.30
    uh = np.zeros_like(vel)
    uh[ok] = vel[ok] / n[ok, None]
    rel = fcu - cmd
    along = np.einsum("ij,ij->i", rel, uh)
    lat = np.linalg.norm(rel - along[:, None] * uh, axis=1)
    # retrace: the vehicle's step projected on the reference direction, negative
    step = np.diff(fcu, axis=0)
    back = np.einsum("ij,ij->i", step, uh[:-1])
    neg = np.isfinite(back) & (back < -1e-4)
    rid = np.cumsum(np.concatenate([[0], np.diff(neg.astype(int)) != 0]))
    rev_m, rev_n = 0.0, 0
    for r in np.unique(rid):
        m = neg & (rid == r)
        if m.sum() >= 3:
            dist = float(-np.nansum(back[m]))
            if dist >= 0.15:
                rev_m += dist
                rev_n += 1
    # mid-leg reference dips
    moving = n > 0.05
    dist_corner = np.min(np.stack([np.hypot(cmd[:, 0] - cx, cmd[:, 1] - cy) for cx, cy in CORNERS]), axis=0)
    dip = (n < 0.25) & (dist_corner > 1.5) & moving
    dips = int(np.sum(np.diff(dip.astype(int)) == 1)) + int(dip[0])
    # seams
    tid = np.array([int(float(r["vel_z"] * 0 + 0)) for r in []]) if False else None
    # trajectory installs: from wire.csv (the bridge records trajectory_id)
    wpath = os.path.join(ROOT, "missionA-" + tag, "wire.csv")
    seams, step_mean = 0, float("nan")
    if os.path.isfile(wpath):
        w = list(csv.DictReader(open(wpath)))
        wv = np.array([np.hypot(float(r["v_l_x"]), float(r["v_l_y"])) for r in w])
        wi = np.array([int(r["trajectory_id"]) for r in w])
        idx = np.nonzero(np.diff(wi) != 0)[0]
        seams = len(idx)
        drops = []
        for k in idx:
            if 3 <= k < len(wv) - 3:
                drops.append(float(np.median(wv[k:k + 3]) - np.median(wv[k - 3:k])))
        step_mean = float(np.mean(drops)) if drops else float("nan")
    out = {"tag": tag, "rev_m": rev_m, "rev_n": rev_n,
           "along": (np.percentile(along, 5), np.percentile(along, 50), np.percentile(along, 95)),
           "lat": (np.percentile(lat, 50), np.percentile(lat, 90)),
           "dips": dips, "seams": seams, "step": step_mean, "rtf": float("nan"),
           "w6": float("nan"), "w": {}}
    ap = os.path.join(ROOT, "missionA-" + tag, "accept.json")
    if os.path.isfile(ap):
        a = json.load(open(ap))
        for r in a["rows"]:
            if r["id"] in ("W1b", "W4", "W6", "W1", "H1"):
                out["w"][r["id"]] = (r["ok"], r["detail"].splitlines()[0][:60])
            if r["id"] == "H1":
                m = r["detail"]
                k = m.find("rtf ")
                if k > 0:
                    try:
                        out["rtf"] = float(m[k + 4:].split()[0])
                    except ValueError:
                        pass
            if r["id"] == "W6":
                for ln in r["detail"].splitlines():
                    if ln.startswith("p90 at that offset"):
                        try:
                            out["w6"] = float(ln.split(":")[1].split()[0])
                        except (ValueError, IndexError):
                            pass
    return out


def main(tags):
    print("%-13s %6s %4s %4s %6s  %-22s %-13s %4s %4s %7s  %s"
          % ("tag", "REV m", "n", "rtf", "dips", "along p5/50/95 (m)", "lat p50/90", "W6p90", "seams",
             "step m/s", "W1b/W4/W6"))
    for t in tags:
        try:
            m = metrics(t)
        except Exception as exc:                                   # noqa: BLE001
            print("%-13s  ERROR %s" % (t, exc))
            continue
        w = m["w"]
        ws = " ".join("%s:%s" % (k, {True: "P", False: "F", None: "I"}.get(w.get(k, (None,))[0], "?"))
                      for k in ("W1b", "W4", "W6"))
        print("%-13s %6.2f %4d %4.2f %6.2f  %+6.2f/%+6.2f/%+6.2f  %5.2f/%-6.2f %6.2f %4d %+7.2f  %s"
              % (t, m["rev_m"], m["rev_n"], m["rtf"], m["dips"], *m["along"], *m["lat"], m["w6"],
                 m["seams"], m["step"], ws))


if __name__ == "__main__":
    main(sys.argv[1:] or ["kv1-02-01"])
