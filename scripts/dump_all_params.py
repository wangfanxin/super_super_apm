#!/usr/bin/env python3
"""把 SITL 的整张参数表落盘（PARAM_REQUEST_LIST，等满 20 s），再从文件里找关键项。"""
from __future__ import annotations

import sys
import time

from pymavlink import mavutil

URL = "udpin:127.0.0.1:14550"
OUT = "/tmp/params.csv"
SECONDS = 20.0


def main() -> int:
    m = mavutil.mavlink_connection(URL, source_system=255)
    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    m.mav.request_data_stream_send(1, 1, 30, 10, 1)
    if m.wait_heartbeat(timeout=15) is None:
        print("NO HEARTBEAT")
        return 1

    m.mav.param_request_list_send(m.target_system, m.target_component)
    got = {}
    t0 = time.time()
    while time.time() - t0 < SECONDS:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.3)
        if msg is None:
            continue
        nm = msg.param_id
        nm = nm.decode(errors="ignore") if isinstance(nm, bytes) else str(nm)
        nm = nm.rstrip("\x00").strip()
        got[nm] = msg.param_value

    with open(OUT, "w") as fh:
        fh.write("name,value\n")
        for k in sorted(got):
            fh.write("%s,%.6f\n" % (k, got[k]))
    print("dumped %d params -> %s" % (len(got), OUT))

    for want in ("ARMING_CHECK", "GPS_TYPE", "FRAME_TYPE", "FRAME_CLASS",
                 "EK3_SRC1_POSXY", "EK3_SRC1_POSZ", "VISO_TYPE", "AHRS_EKF_TYPE",
                 "FS_GCS_ENABLE", "BRD_SAFETY_DEFLT"):
        v = got.get(want)
        print("  %-18s %s" % (want, "MISSING" if v is None else "%.1f" % v))
    return 0


if __name__ == "__main__":
    sys.exit(main())
