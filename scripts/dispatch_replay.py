#!/usr/bin/env python3
"""dispatch_replay.py -- offline sanity check for the R1 one-corner-lookahead dispatch.

Replays a recorded truth track through cmd_stream.TourPacer with the SAME arrival/pass-by settings the
flight used, then compares two dispatch rules:
  OLD  : sent_i = pacer.i_leg (+ pre-goal advancement when within PRE_GOAL_M of the target)
  R1   : sent_i = min(pacer.i_leg + 1, n-1) for pacer.i_leg >= 1
and reports, at every goal switch, how much of the OLD committed leg is left (distance/time to the
in-flight corner).  R1 is supposed to switch with MOST of the leg still ahead (so the planner replans
mid-leg, not after the trajectory ended).

USAGE: python3 scripts/dispatch_replay.py --tag phase177b-01-01
"""
import argparse, csv, os, sys
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "bridge_nodes"))
sys.path.insert(0, os.path.join(ROOT, "bridge_nodes", "planner"))
sys.path.insert(0, os.path.join(ROOT, "scripts"))


def world_corners(path):
    """Parse mission.corners, expanding YAML anchors/aliases (&idNNN / *idNNN) -- the hex worlds use both."""
    import re
    txt = open(path, encoding="utf-8").read()
    i = txt.find("\nmission:")
    if i < 0:
        raise SystemExit("no mission block in %s" % path)
    block = txt[i:]
    j = block.find("corners:")
    if j < 0:
        raise SystemExit("no mission.corners in %s" % path)
    lines = block[j:].splitlines()[1:]
    anchors = {}
    pts = []
    for line in lines:
        s = line.strip()
        if not s.startswith("- "):
            break
        body = s[2:].strip()
        m = re.match(r"&(\w+)\s+\[(.*)\]", body)
        if m:
            name, vals = m.group(1), [float(x) for x in m.group(2).split(",")]
            anchors[name] = vals
            pts.append(vals)
            continue
        m = re.match(r"\*(\w+)", body)
        if m and m.group(1) in anchors:
            pts.append(anchors[m.group(1)])
            continue
        m = re.match(r"\[(.*)\]", body)
        if m:
            pts.append([float(x) for x in m.group(1).split(",")])
    if not pts:
        raise SystemExit("no corners parsed in %s" % path)
    return np.array(pts, float)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="phase177b-01-01")
    ap.add_argument("--world", default="sim/worlds/forest_hex12.yaml")
    ap.add_argument("--arrive-m", type=float, default=0.3)
    ap.add_argument("--pass-by-m", type=float, default=0.6)
    ap.add_argument("--pass-by-long-m", type=float, default=1.0)
    ap.add_argument("--pre-goal-m", type=float, default=0.8)
    ap.add_argument("--settle-speed", type=float, default=6.0)
    ap.add_argument("--settle-s", type=float, default=6.0)
    ap.add_argument("--advance-s", type=float, default=2.4,
                    help="R1a: advance when distance-to-target < v*advance_s + margin")
    ap.add_argument("--advance-margin", type=float, default=0.3)
    a = ap.parse_args()
    from cmd_stream import TourPacer
    corners = world_corners(os.path.join(ROOT, a.world))
    # the mission tour: pad (0,0) then the corners, as the launcher builds it
    tour = np.vstack([[0.0, 0.0, 2.0], corners])
    d = os.path.join(ROOT, ".runs", "missionA-%s" % a.tag)
    tap = list(csv.DictReader(open(os.path.join(d, "tap.csv"))))
    t = np.array([float(r["t_sim_us"]) * 1e-6 for r in tap])
    P = np.column_stack([[float(r["pe"]) for r in tap], [float(r["pn"]) for r in tap], -np.array([float(r["pd"]) for r in tap])])
    k = np.concatenate([[True], np.diff(t) > 1e-9]); t = t[k]; P = P[k]
    o = np.argsort(t); t = t[o]; P = P[o]

    class Clock:
        t = 0.0
    for mode in ("old", "lookahead", "r1a"):
        clk = Clock()
        pacer = TourPacer(tour, 0.0, a.arrive_m, a.settle_speed, a.settle_s, 40.0, 0.0,
                          seg_len_w=lambda i: float(np.linalg.norm(tour[i][:2] - tour[max(i - 1, 0)][:2])),
                          pass_by_m=a.pass_by_m, pass_by_long_m=a.pass_by_long_m)
        sent = 0; switches = []; last_t = None; latched = -1
        for i in range(1, len(t)):
            dt = t[i] - t[i - 1]
            if dt <= 0:
                continue
            clk.t += dt
            v_h = float(np.linalg.norm(P[i, :2] - P[i - 1, :2]) / dt)
            pacer.step(tuple(P[i]), v_h, clk.t)
            if mode == "lookahead" and pacer.i_leg >= 1:
                new_sent = int(min(pacer.i_leg + 1, len(tour) - 1))
            elif mode == "r1a" and pacer.i_leg >= 1 and pacer.i_leg < len(tour) - 1:
                if latched == pacer.i_leg:
                    new_sent = int(pacer.i_leg + 1)
                else:
                    d_now = float(np.linalg.norm(P[i, :2] - tour[pacer.i_leg][:2]))
                    v_raw = float(np.linalg.norm(P[i, :2] - P[i - 1, :2]) / dt) if dt > 0 else 0.0
                    vf = 0.85 * locals().get("v_f", v_raw) + 0.15 * v_raw
                    v_f = vf
                    v_est = vf
                    if d_now < v_est * a.advance_s + a.advance_margin:
                        new_sent = int(pacer.i_leg + 1); latched = pacer.i_leg
                    else:
                        new_sent = int(pacer.i_leg)
            else:
                new_sent = int(pacer.i_leg)
                # pre-goal rule: advance when within pre_goal of the in-hand target
                if 0.0 < a.pre_goal_m and pacer.i_leg < len(tour) - 1:
                    if latched == pacer.i_leg:
                        new_sent = int(pacer.i_leg + 1)
                    elif np.linalg.norm(P[i, :2] - tour[pacer.i_leg][:2]) < a.pre_goal_m:
                        new_sent = int(pacer.i_leg + 1); latched = pacer.i_leg
            if new_sent != sent:
                # how much of the leg to the CURRENT sent target is still ahead?
                remain = float(np.linalg.norm(P[i, :2] - tour[sent][:2])) if sent < len(tour) else 0.0
                switches.append((t[i], sent, new_sent, remain))
                sent = new_sent
        print("== %s mode=%s: pacer advanced to leg %d/%d; %d goal switches"
              % (a.tag, mode, pacer.i_leg + 1, len(tour), len(switches)))
        for (tt, old, new, remain) in switches:
            print("   t=%6.1f sent %2d -> %2d  (distance still ahead to old target at switch %5.2f m)" % (tt, old, new, remain))
        if switches:
            rem = np.array([s[3] for s in switches])
            print("   remaining-to-old-target at switch: p50 %.2f m, min %.2f m; switches with < 2.0 m: %d/%d"
                  % (np.median(rem), rem.min(), int(np.sum(rem < 2.0)), len(rem)))


if __name__ == "__main__":
    main()
