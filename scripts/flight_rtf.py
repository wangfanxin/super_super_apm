#!/usr/bin/env python3
"""flight_rtf.py -- print the FLIGHT-window real-time factor of a sortie (one number).

    python3 scripts/flight_rtf.py /home/fanxin/lio_avoid/logs/missionA-kx1-01-02
    -> 0.9999

This is the host gate's number, and it comes from `phase.csv` (take-off .. land) rather than from the
whole tap window: the tap is started before the SITL begins stepping, so its window carries a startup
transient that has nothing to do with the flight.  See the note in `accept.py:flight_rtf`.
Prints `n/a` and exits 1 when the pair is unavailable, so a shell caller can fall back loudly.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import accept  # noqa: E402


def main():
    run = sys.argv[1]
    p = os.path.join(run, "phase.csv") if os.path.isdir(run) else run
    r = accept.flight_rtf(p)
    if r is None:
        print("n/a")
        return 1
    print("%.4f" % r[0])
    return 0


if __name__ == "__main__":
    sys.exit(main())
