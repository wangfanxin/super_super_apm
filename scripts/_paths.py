#!/usr/bin/env python3
"""_paths.py -- the ONE place that resolves the paths the analysis tools default to.

Every tool in scripts/ (and the historical tools under harness/src/) used to hard-code the author's
absolute layout, e.g. the owner's pre-2026-11-24 flight-log directory.  A published repository cannot
carry that: a reader
who clones it gets defaults that point at nothing.  From round 21 on, defaults come from here:

    import _paths as _P
    run = _P.sortie("kx6-03-01-01")        # <LIO_LOGS>/missionA-kx6-03-01-01
    world = _P.world("forest_hex12.yaml")  # <repo>/worlds/forest_hex12.yaml

Resolution order (first hit wins):
  1. explicit environment variables (DSH_ROOT, LIO_LOGS, CALIB_FILE, MID360_PATTERN, WS_SUPER, LIO_WS,
     ARDUPILOT_ROOT, SUPER_PLANNER_ROOT) -- the same names setup/env.sh exports;
  2. the repository layout (this file lives in <repo>/scripts/);
  3. a documented fallback for the pre-migration layout (only for the historical one-off tools).

Nothing here imports ROS, numpy or anything third-party: `python3 -c "import _paths"` must always work.
"""
from __future__ import annotations

import os

_HERE = os.path.dirname(os.path.abspath(__file__))


def _runtime_env():
    """Read the machine-local overrides that setup/env.sh also sources (setup/runtime.env).

    A plain parser rather than `source`: this module must work from any python, and the file only ever
    contains `export KEY=value` lines.  Environment variables still win over the file.
    """
    root = os.environ.get("DSH_ROOT") or os.path.dirname(_HERE)
    out = {}
    p = os.path.join(root, "setup", "runtime.env")
    try:
        with open(p, encoding="utf-8") as fh:
            for line in fh:
                line = line.strip()
                if not line or line.startswith("#") or not line.startswith("export "):
                    continue
                k, _, v = line[len("export "):].partition("=")
                out[k.strip()] = v.strip().strip('"').strip("'")
    except OSError:
        pass
    return out


_RT = _runtime_env()


def _pick(name, default):
    return os.environ.get(name) or _RT.get(name) or default


# --- the repository -------------------------------------------------------------------------
ROOT = os.environ.get("DSH_ROOT") or _RT.get("DSH_ROOT") or os.path.dirname(_HERE)
WORKSPACE = os.path.dirname(ROOT)                                # the folder that holds <repo>

HARNESS = os.path.join(ROOT, "harness")
HARNESS_SRC = os.path.join(HARNESS, "src")
HARNESS_SIM = os.path.join(HARNESS, "sim")
HARNESS_WORLDS = os.path.join(HARNESS, "worlds")
WORLDS = os.path.join(ROOT, "worlds")
CONFIG = os.path.join(ROOT, "config")
PARAMS = os.path.join(ROOT, "params")
DOCS = os.path.join(ROOT, "docs")
EVIDENCE = os.path.join(DOCS, "evidence")
THIRD_PARTY = os.path.join(ROOT, "third_party")
SUPER = _pick("SUPER_PLANNER_ROOT", os.path.join(THIRD_PARTY, "SUPER"))
CALIB = _pick("CALIB_FILE", os.path.join(ROOT, "calib", "calib.json"))
MID360_PATTERN = _pick("MID360_PATTERN", os.path.join(HARNESS_SIM, "mid360_pattern.csv"))

# --- builds and recordings (outside git) -----------------------------------------------------
BUILD = os.path.join(ROOT, ".build")
WS_SUPER = _pick("WS_SUPER", os.path.join(BUILD, "ws_super"))
LIO_WS = _pick("LIO_WS", os.path.join(BUILD, "lio_ws"))
ARDUPILOT = _pick("ARDUPILOT_ROOT", os.path.join(BUILD, "ardupilot"))
LOGS = _pick("LIO_LOGS", os.path.join(ROOT, ".runs"))

# --- documented legacy locations (pre-2026-11-24 layout; read-only, may not exist) -------------
LEGACY_STAGE = os.path.join(BUILD, "stage")          # was <workspace>/_stage
LEGACY_REFS = os.path.join(WORKSPACE, "_refs")       # kept by the owner


def sortie(tag: str) -> str:
    """The directory of one recorded sortie: <LOGS>/missionA-<tag>."""
    return os.path.join(LOGS, "missionA-" + tag)


def world(name: str = "forest_hex12.yaml") -> str:
    return os.path.join(WORLDS, name)


def config(name: str = "super_planner_hex.yaml") -> str:
    return os.path.join(CONFIG, name)


def param(name: str) -> str:
    return os.path.join(PARAMS, name)


def newest_sortie() -> str | None:
    """The most recently modified <LOGS>/missionA-* directory, or None."""
    try:
        cands = [os.path.join(LOGS, n) for n in os.listdir(LOGS) if n.startswith("missionA-")]
    except OSError:
        return None
    cands = [c for c in cands if os.path.isdir(c)]
    return max(cands, key=os.path.getmtime) if cands else None


def summary() -> dict:
    """What a tool should print under --verbose / in its header."""
    return {"ROOT": ROOT, "HARNESS": HARNESS, "LOGS": LOGS, "CALIB": CALIB, "SUPER": SUPER,
            "WS_SUPER": WS_SUPER, "LIO_WS": LIO_WS, "ARDUPILOT": ARDUPILOT}


if __name__ == "__main__":
    for k, v in summary().items():
        print("%-12s %s" % (k, v))
    for name in ("world('forest_hex12.yaml')", "sortie('kx6-03-01-01')", "newest_sortie()"):
        print("%-32s %s" % (name, eval(name)))  # noqa: S307 - explicit, fixed list
