#!/usr/bin/env python3
"""One-shot: flip traj_opt.print_optimizer_log for ONE diagnostic sortie (round 87, §71).

Why: the optimizer's hard reject is `pena_log(3) > max_acc * penna_margin` = 8 * 0.05 = 0.4 m/s^2
(exp_traj_optimizer_s4.cpp:715-719).  With the flag on, every optimisation attempt prints the MAX
violation of |a|-max_acc, so the 3 m/s reject can be quoted as a number instead of "it failed".

    python3 scripts/set_optimizer_log.py on|off
"""
import io
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = os.path.join(ROOT, "config", "super_planner_hex.yaml")


def main(argv):
    if len(argv) != 2 or argv[1] not in ("on", "off"):
        raise SystemExit("usage: set_optimizer_log.py on|off")
    want = "true" if argv[1] == "on" else "false"
    s = io.open(CFG, encoding="utf-8").read()
    old_line = "    print_optimizer_log: false"
    new_line = "    print_optimizer_log: %s" % want
    if want == "false":
        # either value may be in the file now
        for cand in ("    print_optimizer_log: true", old_line):
            if cand in s:
                s = s.replace(cand, new_line)
                break
    else:
        if old_line not in s:
            raise SystemExit("print_optimizer_log: false not found -- already on?")
        s = s.replace(old_line, new_line)
    io.open(CFG, "w", encoding="utf-8").write(s)
    print("%s -> print_optimizer_log: %s" % (CFG, want))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
