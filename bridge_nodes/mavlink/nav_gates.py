#!/usr/bin/env python3
"""nav_gates.py -- the explicitness of the external-nav (ODOMETRY #331) refusals.

The bridge refuses to send a LIO sample to the EKF for three different reasons, and the sortie's nav.csv
has to say WHICH one happened (`refused` is a reason code, not a boolean):

    0  sent
    1  physical gate  -- the pose is unreachable for the airframe (see bridge_node.nav_gate_ok)
    2  stale          -- the sample is older than --nav-max-age-s (the FCU declares extnav unhealthy
                         after 300 ms, so forwarding a 2 s old pose is worse than sending nothing)
    3  quality        -- --nav-quality is below --nav-min-quality

Kept in its own module with no ROS/MAVLink imports so the decision table is unit-testable without a
flight or a SITL (tests/check_nav_gates.py).  Defaults reproduce the delivered behaviour exactly:
quality 100, --nav-min-quality 0, --nav-max-age-s 0 (off).
"""
from __future__ import annotations


def refusal_reason(physical_ok: bool, age_s: float, quality: int, min_quality: int = 0,
                   max_age_s: float = 0.0) -> int:
    """Return 0 (send) or 1/2/3 (refused, and why).  The order matters: a physically impossible pose is
    reported as such even if it is also stale, because that is the fault an operator has to look at."""
    if not physical_ok:
        return 1
    if max_age_s > 0.0 and age_s > max_age_s:
        return 2
    if quality < min_quality:
        return 3
    return 0


def describe(reason: int) -> str:
    return {0: "sent", 1: "physical gate", 2: "stale (age gate)", 3: "quality floor"}.get(reason,
                                                                                          "unknown")
