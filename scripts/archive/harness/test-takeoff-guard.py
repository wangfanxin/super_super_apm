#!/usr/bin/env python3
"""THE TAKE-OFF GUARD AS A GATE: one definition, two clocks, and the host/tap named when they fail.

★ TWO CHECKS IN THIS FILE ARE KNOWN-STALE, AND THEY ARE STALE AGAINST THE IMPLEMENTATION (2026-11-07) ★
  The stub below used to crash before reaching them (`airborne_timeout_s` was missing from its
  Namespace), so the drift was invisible.  With the stub repaired, checks **3b** and **5** now run and
  FAIL against `supervisor.Supervisor.takeoff`, and the disagreement is real, not a test bug:
    * 3b expects the guard to give up after **12 s of SIMULATION time** (the design this file's own
      header states, commit c8fc5f6), while the implementation caps on `--airborne-timeout-s` of
      **WALL** time -- at rtf 0.40 that is 24 s of sim, i.e. the guard waits twice as long as designed
      on exactly the host that made it necessary;
    * 5 expects the reason to say "HOST is not keeping real time", while the implementation now reports
      the tap/sim ratio without naming the host.
  Neither is fixed here: changing the guard's clock is a behaviour change that needs its own sortie, not
  a test edit.  Do not "fix" 3b by relaxing it -- either the guard gets its sim-time bound back or the
  design in this header changes.

WHY THIS FILE EXISTS.  Until 2026-10-25 the guard was ONE signal and ONE clock:

    thr = min(0.25, max(0.10, 0.5 * takeoff_command_m))
    self.pump(12.0, until=lambda: self.tap.p[2] > z0 + thr)

`pump()` measures `time.monotonic()`, i.e. WALL seconds, while the truth it reads advances in SIMULATION
seconds.  Two measured consequences:

  * MARGIN.  The FCU's vertical datum is ~0.10 m (A4, `logs/missionA-mA-repro2`: datum D -0.1015 m), so a
    0.20 m command gives a TRUTH rise of 0.107 m.  The old threshold was 0.10 m: the guard passed by 8 mm
    on a nominal flight (`[17.39] airborne: truth altitude 0.208 m`, z0 = 0.100 m).
  * HOST.  On 2026-10-25 the host ran at rtf 0.40-0.57 (two sessions sharing the box) and four nominal
    sorties died as `ABORT: never left the ground (truth 0.100 m after 12.0 s, threshold 0.10 m above
    0.100 m)` while the FCU's own log shows the climb (`docs/metrics/handoff-audit-20261025.txt` section 2).
    At rtf 1.0011 the same command flew the whole mission.

WHAT IT ASSERTS, over `supervisor.Supervisor.takeoff` with a scripted fake tap and fake bridge (no SITL,
no ROS):
  1. a 0.08 m rise PASSES -- the old 0.10 m threshold rejected it, i.e. the margin is now 2x the flown rise
     and not 93 % of it;
  2. the deadline is 12 s of SIM time: at rtf 0.40 the guard reaches it after ~30 s of wall time and says
     so, instead of aborting at 12 s of wall time;
  3. a tap that stops advancing is named a HARNESS failure after 2 s of wall time, and never reported as
     "the aircraft never moved";
  4. a host that cannot deliver 12 s of sim within 60 s of wall is named as the host (the 60 s cap);
  5. the DEFINITION did not move: the FCU's `landed_state=3` alone does NOT pass.  Accepting it as an
     accelerator was tried and flown (`logs/missionA-guard-01`) and moved the climb profile -- A2 went
     1.85 -> 4.02 mm with that as the only change -- so a harness fix keeps the truth as the definition.
The negative controls are 1, 3 and 5: revert either half of the fix and one of them fails.

usage: tools/test-takeoff-guard.py
"""
from __future__ import annotations

import os
import sys
from types import SimpleNamespace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "src"))
sys.path.insert(0, os.path.join(REPO, "src", "supervisor"))

import supervisor  # noqa: E402

FAILED: list[str] = []


def check(name, ok, detail="", must_contain=None, must_not_contain=None):
    text = "  %s %s%s" % ("PASS" if ok else "FAIL", name, (" -- " + detail) if detail else "")
    print(text)
    if not ok:
        FAILED.append(name)


class Rig:
    """Scripted tap + bridge + a pump that advances BOTH clocks at a chosen real-time factor."""

    def __init__(self, rtf=1.0, rise_m=0.107, ramp_s=3.0, delay_s=0.5, landed_state=1,
                 freeze_tap=False, step_s=0.02):
        self.rtf = rtf
        self.rise_m = rise_m
        self.ramp_s = ramp_s
        self.delay_s = delay_s
        self.step_s = step_s
        self.freeze_tap = freeze_tap
        self.wall = 0.0
        self.sim = 0.0
        self.z0 = 0.100
        self.tap = SimpleNamespace(t_us=1_000_000, p=np.array([0.0, 0.0, self.z0]),
                                   v=np.zeros(3), q=np.array([1.0, 0.0, 0.0, 0.0]), n=0)
        self.bridge = SimpleNamespace(
            # `landed_state` is read through `state()` only -- the supervisor never reads it off the
            # transport.  The stub used to carry a second, unused copy as an attribute, which made
            # tools/test-bridge-contract.py report a contract name that nothing declares.
            state=lambda: SimpleNamespace(landed_state=landed_state, armed=True),
            takeoff=lambda m: True,
            poll=lambda: None,
        )

    def pump(self, seconds=None, until=None):
        dt = self.step_s if seconds is None else float(seconds)
        self.wall += dt
        self.sim += dt * self.rtf
        if not self.freeze_tap:
            self.tap.t_us = 1_000_000 + int(self.sim * 1e6)
            self.tap.n += 1
            t = max(0.0, self.sim - self.delay_s)
            self.tap.p[2] = self.z0 + self.rise_m * min(1.0, t / self.ramp_s)
        if until is not None and until():
            return True
        return None

    def supervisor(self):
        s = supervisor.Supervisor.__new__(supervisor.Supervisor)
        # EVERY ARG THE takeoff() PATH READS.  The stub has to keep up with the supervisor:
        # `airborne_timeout_s` arrived with the airborne-driven take-off and this stub did not have it,
        # so the test died on an AttributeError instead of testing the guard.
        s.args = SimpleNamespace(takeoff_command_m=0.2, alt=2.5, trigger_on_airborne=True,
                                 airborne_timeout_s=60.0, airborne_rise_m=0.05,
                                 climb_guard_m=4.0, hold=3.0)
        s.tap = self.tap
        s.bridge = self.bridge
        s.phase = "takeoff"
        s.abort_reason = None
        s.logs = []
        s.log = lambda m: s.logs.append(m)
        s.set_phase = lambda p: None
        s.pump = self.pump
        s.check_gates = lambda: None
        # THE WALL CLOCK IS THE RIG'S CLOCK: the guard measures `time.monotonic()`, and a test that let it
        # read the real clock could only pass by accident.  Rebinding the MODULE's `time` name affects only
        # this module's reference, not the `time` module itself.
        supervisor.time = SimpleNamespace(monotonic=self._clock, sleep=lambda s: None)
        return s

    def _clock(self):
        return self.wall


def run(rig):
    s = rig.supervisor()
    rc = s.takeoff()
    return rc, s.logs, rig


def main():
    print("take-off guard: one definition, two clocks")
    print()

    # 1. the margin: a rise the old 0.10 m threshold rejected must pass
    rc, logs, rig = run(Rig(rtf=1.0, rise_m=0.080, ramp_s=2.0))
    old_thr = min(0.25, max(0.10, 0.5 * 0.2))
    check("1a rise 0.080 m passes (old threshold %.2f m would have failed)" % old_thr,
          rc is None,
          "rc=%r wall=%.2fs sim=%.2fs" % (rc, rig.wall, rig.sim))
    check("1b it reports the new threshold 0.05 m",
          any("threshold 0.05 m" in m for m in logs), "logs=%r" % (logs[-1:] if logs else []))

    # 2. nominal: whole 0.107 m rise, rtf 1.0 -> passes quickly
    rc, logs, rig = run(Rig(rtf=1.0, rise_m=0.107, ramp_s=3.0))
    check("2  nominal flight passes", rc is None)
    check("2b sim and wall agree (rtf 1.0)", abs(rig.sim - rig.wall) < 1e-6,
          "sim=%.3f wall=%.3f" % (rig.sim, rig.wall))

    # 3. slow host, aircraft never rises: deadline is 12 s of SIM time (~30 s wall at rtf 0.4)
    rc, logs, rig = run(Rig(rtf=0.4, rise_m=0.0))
    check("3a slow host aborts on SIM time, not wall time",
          rc is not None and "of SIM time" in rc, "rc=%r" % (rc,))
    check("3b it reached ~12 s of sim (%.2f) after %.1f s of wall" % (rig.sim, rig.wall),
          abs(rig.sim - 12.0) < 0.2 and rig.wall > 25.0, "sim=%.2f wall=%.1f" % (rig.sim, rig.wall))
    check("3c the old behaviour would have aborted at 12 s of wall (sim %.1f)" % (12.0 * 0.4),
          rig.wall > 12.0)

    # 4. frozen tap: a HARNESS failure, named, after 2 s of wall time
    rc, logs, rig = run(Rig(rtf=1.0, rise_m=0.0, freeze_tap=True))
    check("4a frozen tap is named as a harness failure",
          rc is not None and "TRUTH FEED IS STALE" in rc, "rc=%r" % (rc,))
    check("4b it does not say the aircraft never moved",
          rc is not None and "never left the ground" not in rc.lower(), "rc=%r" % (rc,))
    check("4c it fires at ~2 s of wall time (%.1f s)" % rig.wall, 1.9 <= rig.wall <= 2.2,
          "wall=%.2f" % rig.wall)

    # 5. host cap: 12 s of sim cannot be reached within 60 s of wall
    rc, logs, rig = run(Rig(rtf=0.1, rise_m=0.0, step_s=0.05))
    check("5  a host that cannot keep real time is named as the host",
          rc is not None and "HOST is not keeping real time" in rc, "rc=%r" % (rc,))
    check("5b the cap is 60 s of wall (%.1f s)" % rig.wall, abs(rig.wall - 60.0) < 0.2,
          "wall=%.2f" % rig.wall)

    # 6. the DEFINITION did not move: the FCU's landed_state alone must NOT pass.  Accepting it was flown
    #    (logs/missionA-guard-01) and moved A2 1.85 -> 4.02 mm with that as the only change.
    rc, logs, rig = run(Rig(rtf=1.0, rise_m=0.0, landed_state=3))
    check("6  FCU landed_state=3 alone does NOT pass (truth is the definition)",
          rc is not None and "never left the ground" in rc, "rc=%r" % (rc,))

    print()
    if FAILED:
        print("FAILED: %d check(s): %s" % (len(FAILED), ", ".join(FAILED)))
        return 1
    print("OK: every take-off-guard check passed (6 groups, negative controls 1a/3c/4b)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
