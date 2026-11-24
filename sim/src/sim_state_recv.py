#!/usr/bin/env python3
"""sim_state_recv.py -- Phase 1 truth source: read ArduPilot SITL's SIM_STATE (MAVLink 108) from an
UNPATCHED SITL and record it to CSV.

Why this file exists (measured 2026-11-24, probe_sim_state.py):
  * an unpatched SITL DOES emit SIM_STATE, and answers SET_MESSAGE_INTERVAL: asking 200 Hz delivered
    ~225 Hz (the stream scheduler divides the 1 kHz model loop); the exact achievable rate is measured at
    run time and printed, never assumed;
  * SIM_STATE carries NO simulation timestamp and NO local NED position:
        q1..q4, roll/pitch/yaw, xacc/yacc/zacc, xgyro/ygyro/zgyro, lat/lon/alt, vn/ve/vd,
        std_dev_horz/std_dev_vert, lat_int/lon_int
    so this recorder stores the RAW fields plus the RECEIVE wall clock (`t_wall_us`, monotonic) and a
    provisional local-NED estimate.  The exact lat/lon -> local-NED mapping is fitted later by
    scripts/compare_truth.py against the old tap stream (which has both), and the fitted mapping +
    residual are what decide whether SIM_STATE can replace the tap for grading.

CSV columns (mirror the tap where the fields exist):
    t_wall_us,t_unix_us,lat,lon,alt,q1,q2,q3,q4,roll,pitch,yaw,
    ax,ay,az,wx,wy,wz,vn,ve,vd,pn,pe,pd,lat_int,lon_int,std_h,std_v

    python3 sim/src/sim_state_recv.py --csv /tmp/sim_state.csv --seconds 5
    python3 sim/src/sim_state_recv.py --mav tcp:127.0.0.1:5782 --hz 1000 --csv "$OUT/sim_state.csv"
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import signal
import sys
import time

GLOBAL_STOP = False


def _on_signal(signum, frame):  # noqa: ARG001
    global GLOBAL_STOP
    GLOBAL_STOP = True


def parse_home(text):
    """'lat,lon,alt' -> (lat_deg, lon_deg, alt_m)."""
    parts = [p.strip() for p in text.split(",")]
    if len(parts) < 2:
        raise SystemExit("--home needs lat,lon[,alt]")
    lat = float(parts[0])
    lon = float(parts[1])
    alt = float(parts[2]) if len(parts) > 2 and parts[2] else 0.0
    return lat, lon, alt


class LocalEnu:
    """Provisional WGS84-ish lat/lon/alt -> local NED (N, E, D) around home.

    Small-area approximation with the local meridional / prime-vertical radii (mm-level over a few
    hundred metres).  The AUTHORITATIVE mapping is fitted by compare_truth.py against the tap; this is
    only so a human can read the CSV.
    """

    def __init__(self, lat0, lon0, alt0):
        self.lat0, self.lon0, self.alt0 = lat0, lon0, alt0
        a, f = 6378137.0, 1.0 / 298.257223563
        e2 = f * (2.0 - f)
        s = math.sin(math.radians(lat0))
        self.rn = a / math.sqrt(1.0 - e2 * s * s)          # prime vertical
        self.rm = a * (1.0 - e2) / (1.0 - e2 * s * s) ** 1.5  # meridional

    def ned(self, lat, lon, alt):
        dlat = math.radians(lat - self.lat0)
        dlon = math.radians(lon - self.lon0)
        n = dlat * self.rm
        e = dlon * self.rn * math.cos(math.radians(self.lat0))
        d = -(alt - self.alt0)
        return n, e, d


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mav", default="tcp:127.0.0.1:5782", help="SITL endpoint (a spare serial port)")
    ap.add_argument("--hz", type=float, default=1000.0, help="requested SIM_STATE rate")
    ap.add_argument("--csv", default="", help="write the CSV here ('' = stdout)")
    ap.add_argument("--home", default="-35.363261,149.165230,584.0", help="SITL home: lat,lon,alt")
    ap.add_argument("--seconds", type=float, default=0.0, help="stop after N s (0 = until SIGTERM)")
    ap.add_argument("--report-hz", type=float, default=5.0, help="rate of the progress line on stderr")
    args = ap.parse_args()

    signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT, _on_signal)

    from pymavlink import mavutil

    m = mavutil.mavlink_connection(args.mav, source_system=251)
    m.wait_heartbeat(timeout=20)
    print("sim_state_recv: heartbeat from system %d component %d" % (m.target_system, m.target_component),
          file=sys.stderr)

    interval_us = int(round(1e6 / max(1.0, args.hz)))
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            mavutil.mavlink.MAVLINK_MSG_ID_SIM_STATE, interval_us, 0, 0, 0, 0, 0)
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                            mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 1000000, 0, 0, 0, 0, 0)

    home = parse_home(args.home)
    enu = LocalEnu(*home)

    cols = ["t_wall_us", "t_unix_us", "lat", "lon", "alt", "q1", "q2", "q3", "q4",
            "roll", "pitch", "yaw", "ax", "ay", "az", "wx", "wy", "wz",
            "vn", "ve", "vd", "pn", "pe", "pd", "lat_int", "lon_int", "std_h", "std_v"]
    fh = open(args.csv, "w", newline="") if args.csv else sys.stdout
    w = csv.writer(fh)
    w.writerow(cols)

    t_start = time.monotonic()
    last_report = t_start
    n = 0
    got_hb = False
    try:
        while not GLOBAL_STOP:
            now = time.monotonic()
            if args.seconds and now - t_start >= args.seconds:
                break
            msg = m.recv_match(type=["SIM_STATE", "HEARTBEAT"], blocking=True, timeout=1.0)
            if msg is None:
                continue
            mtype = msg.get_type()
            if mtype == "HEARTBEAT":
                got_hb = True
                continue
            t_wall_us = int((now - t_start) * 1e6)
            lat_deg = msg.lat * 1e-7 if abs(msg.lat) > 1000 else float(msg.lat)   # dialect may give deg or 1e-7
            lon_deg = msg.lon * 1e-7 if abs(msg.lon) > 1000 else float(msg.lon)
            lat_i = int(round(lat_deg * 1e7))
            lon_i = int(round(lon_deg * 1e7))
            # alt is AMSL in metres; SITL's --home alt is the ground reference
            p = enu.ned(lat_deg, lon_deg, msg.alt)
            w.writerow([t_wall_us, int(time.time() * 1e6), "%.7f" % lat_deg, "%.7f" % lon_deg,
                        "%.3f" % msg.alt,
                        "%.7f" % msg.q1, "%.7f" % msg.q2, "%.7f" % msg.q3, "%.7f" % msg.q4,
                        "%.6f" % msg.roll, "%.6f" % msg.pitch, "%.6f" % msg.yaw,
                        "%.5f" % msg.xacc, "%.5f" % msg.yacc, "%.5f" % msg.zacc,
                        "%.6f" % msg.xgyro, "%.6f" % msg.ygyro, "%.6f" % msg.zgyro,
                        "%.5f" % msg.vn, "%.5f" % msg.ve, "%.5f" % msg.vd,
                        "%.5f" % p[0], "%.5f" % p[1], "%.5f" % p[2],
                        lat_i, lon_i, "%.4f" % msg.std_dev_horz, "%.4f" % msg.std_dev_vert])
            n += 1
            if now - last_report >= 1.0 / max(0.1, args.report_hz):
                last_report = now
                hz = n / max(1e-6, now - t_start)
                print("sim_state_recv: %d samples, %.1f Hz (requested %.0f), heartbeat %s"
                      % (n, hz, args.hz, "yes" if got_hb else "no"), file=sys.stderr)
    finally:
        fh.flush()
        if args.csv:
            fh.close()
        dt = time.monotonic() - t_start
        print("sim_state_recv: stopped, %d samples in %.2f s = %.1f Hz"
              % (n, dt, n / max(1e-6, dt)), file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
