#!/usr/bin/env python3
"""SITL 探针：确认 MAVLink 通、模式能切 GUIDED、参数能灌。

它是"接上 ArduPilot"这一步的最小证据：不看日志猜，直接问飞控。
用法：python3 src/whtf/ros/sitl_probe.py [--url udp:127.0.0.1:14550]
"""
from __future__ import annotations

import argparse
import sys
import time

from pymavlink import mavutil

PARAMS = ["PSC_POSXY_P", "PSC_VELXY_P", "PSC_VELXY_D", "ATC_ANG_RLL_P",
          "PSC_JERK_XY", "GUIDED_OPTIONS", "FS_GCS_ENABLE"]


def wait_heartbeat(m, timeout=20.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if msg is not None:
            return msg
    return None


def get_param(m, name, timeout=3.0):
    m.mav.param_request_read_send(m.target_system, m.target_component,
                                  name.encode(), -1)
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if msg is None:
            continue
        pname = msg.param_id
        if isinstance(pname, bytes):
            pname = pname.decode(errors="ignore")
        if pname.rstrip("\x00") == name:
            return msg.param_value
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="udpin:127.0.0.1:14550",
                    help="pymavlink 连接串；SITL 用 udpclient 发，故这里用 udpin 监听")
    ap.add_argument("--set-mode", default="GUIDED")
    ap.add_argument("--parm", default="", help="要灌的 .parm 文件")
    args = ap.parse_args()

    print("[probe] connect", args.url)
    m = mavutil.mavlink_connection(args.url, source_system=250)
    hb = wait_heartbeat(m)
    if hb is None:
        print("[probe] NO HEARTBEAT -> SITL 没起来或端口不对")
        return 1
    print("[probe] heartbeat: sys=%d comp=%d type=%d autopilot=%d"
          % (m.target_system, m.target_component, hb.type, hb.autopilot))
    print("[probe] mode=%s armed=%d" % (m.flightmode, (hb.base_mode & 128) != 0))

    print("\n[probe] params (defaults)")
    for p in PARAMS:
        v = get_param(m, p)
        print("  %-16s %s" % (p, "?" if v is None else ("%.4f" % v)))

    if args.parm:
        print("\n[probe] load parm:", args.parm)
        try:
            m.param_load(args.parm)
        except Exception as exc:                     # noqa: BLE001
            print("  param_load failed:", exc)
        time.sleep(3.0)
        print("[probe] params after load")
        for p in PARAMS:
            v = get_param(m, p)
            print("  %-16s %s" % (p, "?" if v is None else ("%.4f" % v)))

    if args.set_mode:
        print("\n[probe] set mode ->", args.set_mode)
        mapping = m.mode_mapping() or {}
        if args.set_mode not in mapping:
            print("  unknown mode; available:", list(mapping)[:12])
        else:
            m.set_mode(mapping[args.set_mode])
            time.sleep(1.5)
            hb2 = m.recv_match(type="HEARTBEAT", blocking=True, timeout=3.0)
            print("  mode now:", m.flightmode if hb2 else "no hb")
    print("\n[probe] OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
