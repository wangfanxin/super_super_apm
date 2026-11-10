#!/usr/bin/env python3
"""SITL 里程计中继 + 自动飞行编排 + 验收评分。

它做的事
--------
  ① 连 SITL（SITL 端 `udpclient`，我们 `udpin`，可多客户端共存）
  ② **从第 0 秒起** 就以 50 Hz 把 LOCAL_POSITION_NED 转成 ENU 发三个 odom 话题：
       /lidar_slam/odom   → SUPER（fsm_node）
       /lio/robo/odom     → bridge（位姿）
       /lio/odom          → bridge（带真实 twist）
     ★ NED→ENU 换轴：x_E = y_N，y_N = x_E，z_U = -z_D；速度同理 ★
       不换轴 SUPER 会把"北"当"东"，整条轨迹镜像出去。
  ③ 写参数 → 切 GUIDED → 解锁 → 起飞到 2 m → 落地文件 /tmp/takeoff_ok
     （shell 等这个文件再启 SUPER/bridge：位置指令一旦早到，会把 GUIDED 的
       TakeOff 子模式顶掉——bridge 源码里也写了同样一条实测结论。）
  ④ 退出时对着 12 个盒算命中/速度/最小净空 —— 用的是 SITL 的**真实**位置

为什么整条流程是状态机而不是一串 sleep
--------------------------------------
SUPER 的 fsm_node 一启动就要 odom。如果先花 20 s 读参数表再开流，SUPER 会卡在
INIT 里等到超时（历史现场：日志刷 `No odom` ×50）。所以**发布在主循环里跑**，
这里每个 tick 只推进一步，互不阻塞。

用法：
  python3 src/whtf/ros/sitl_odom_relay.py --mav udpin:127.0.0.1:14550 --seconds 100
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion

from pymavlink import mavutil

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
sys.path.insert(0, os.path.join(ROOT, "src"))

from whtf.common.obstacles import load_world, min_clearance  # noqa: E402

WORLD = os.path.join(ROOT, "worlds", "forest_square15.yaml")
Z_FLY = 2.0
CORNERS = [(-7.5, -7.5), (7.5, -7.5), (7.5, 7.5), (-7.5, 7.5)]

#: 回读校验名单。
#: ★ 名字必须是**这个固件里真实存在的** ★
#:   `ARMING_CHECK` / `GPS_TYPE` 在本版（见 AP_Arming.cpp: ARMING_CHECK replaced with
#:   ARMING_SKIPCHK, PR #31568）里**根本不存在**：整表 1370 项里查不到。
#:   正确名字是 `ARMING_SKIPCHK`（位掩码，语义相反：设 1 = 跳过）与 `GPS1_TYPE`。
#:   ArduPilot 的 handle_param_set 对不存在的名字只回 PARAM_ERROR 然后 return
#:   （GCS_Param.cpp:265），所以"写不存在的参数"表现为**静默失败**——这就是
#:   前几轮一直设不上、还误判成"类型不对"的原因。
VERIFY_NAMES = (
    "ARMING_SKIPCHK", "GPS1_TYPE", "VISO_TYPE",
    "EK3_SRC1_POSXY", "EK3_SRC1_POSZ", "EK3_SRC1_VELXY", "EK3_SRC1_YAW",
    "AHRS_EKF_TYPE",
    "PSC_NE_POS_P", "PSC_NE_VEL_P", "PSC_NE_VEL_D", "ATC_ANG_RLL_P",
)

#: 跳过全部预解锁检查的位掩码。
#: Check 枚举是 (1<<1)..(1<<20)（AP_Arming.h），get_enabled_checks 用
#: `(1<<21)-1 & ~1` = 2097150，全部置位后 should_skip_all_checks() 才为真。
SKIP_ALL_CHECKS = 2097150


def ned_to_enu(v):
    return np.array([v[1], v[0], -v[2]], dtype=float)


def _txt(msg, field):
    s = getattr(msg, field)
    return s.decode(errors="ignore").rstrip("\x00") if isinstance(s, bytes) else str(s).rstrip("\x00")


def fmt(v):
    """打印用：大整数不要变成 2.09715e+06（ARMING_SKIPCHK 是位掩码）。"""
    return ("%.0f" % v) if abs(v) >= 1000 else ("%g" % v)


class Relay(Node):
    def __init__(self, args):
        super().__init__("whtf_sitl_relay")
        self.args = args
        self.pillars, self.z_split, _ = load_world(WORLD)

        be = QoSProfile(depth=10)
        be.reliability = ReliabilityPolicy.BEST_EFFORT
        self.pub_slam = self.create_publisher(Odometry, "/lidar_slam/odom", be)
        self.pub_robo = self.create_publisher(Odometry, "/lio/robo/odom", be)
        self.pub_lio = self.create_publisher(Odometry, "/lio/odom", be)

        # ★ THE GO ★
        #   lio_avoid 的 bridge 有一个**单向闸门**：relay.enabled 默认 False，
        #   只有收到 /plan/trigger（latched, PoseStamped，内容不看）之后才开始往线上发 setpoint。
        #   它这么设计是有理由的——"位置 setpoint 会把 GUIDED 的 TakeOff 子模式顶掉"
        #   （bridge_node.on_trigger 的 docstring 原话，实测过）。
        #   所以在我们的编排里，这个 trigger 必须在**确认已经悬停之后**才发。
        #   现场症状：plan_rx=11820 plan_age=0.01 armed=1 mode=GUIDED 但 sent=0 relay=0,
        #   飞机一动不动——就是缺这一条。
        lat = QoSProfile(depth=1)
        lat.reliability = ReliabilityPolicy.RELIABLE
        lat.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub_trigger = self.create_publisher(PoseStamped, "/plan/trigger", lat)
        self.trigger_until = 0.0
        self.next_trigger = 0.0
        self.n_trigger = 0

        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.n_msg = 0
        self.have_pos = False
        self.trace = []
        self.times = []
        self.vel = []
        self.refs = []

        # ★ 参考轨迹也要记下来 ★
        #   只记"飞机在哪"没法回答"为什么速度上不去"：是参考本来只有 4 m/s，
        #   还是飞机跟丢了？所以顺手订 /plan/cmd（bridge 收到的同一条流），
        #   把参考位置/速度与真实位置并排落盘，跟踪误差可以事后独立复算。
        self.ref = None
        self.n_ref = 0
        try:
            from quadrotor_msgs.msg import PositionCommand as RefT
            self.create_subscription(RefT, args.ref_topic, self.on_ref, 10)
            print("[relay] logging reference from %s" % args.ref_topic)
        except Exception as exc:                    # noqa: BLE001
            print("[relay] reference logging disabled: %s" % exc)

    def on_ref(self, m):
        self.ref = (float(m.position.x), float(m.position.y), float(m.position.z),
                    float(m.velocity.x), float(m.velocity.y), float(m.velocity.z))
        self.n_ref += 1

    def publish(self):
        stamp = rclpy.clock.Clock().now().to_msg()
        for pub in (self.pub_slam, self.pub_robo, self.pub_lio):
            m = Odometry()
            m.header.stamp = stamp
            m.header.frame_id = "world"
            m.child_frame_id = "body"
            m.pose.pose.position.x = float(self.p[0])
            m.pose.pose.position.y = float(self.p[1])
            m.pose.pose.position.z = float(self.p[2])
            m.pose.pose.orientation = Quaternion(w=1.0)
            m.twist.twist.linear.x = float(self.v[0])
            m.twist.twist.linear.y = float(self.v[1])
            m.twist.twist.linear.z = float(self.v[2])
            pub.publish(m)

    def go(self):
        """发 /plan/trigger：bridge 的命令流闸门。位置内容它不看，只看"收到了"。

        重复发是有意的：DDS 的 TRANSIENT_LOCAL 只保留 depth 个样本，而 bridge 是
        **起飞之后**才启的进程，晚订阅要靠 latched 补发。发 12 s 就停——
        之后如果 bridge 因为"离开 GUIDED"自己 disable 掉，我们不去把它顶回来
        （那条是安全属性，不该被回放掩盖）。
        """
        msg = PoseStamped()
        msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = float(self.p[0])
        msg.pose.position.y = float(self.p[1])
        msg.pose.position.z = float(self.p[2])
        msg.pose.orientation.w = 1.0
        self.pub_trigger.publish(msg)
        self.n_trigger += 1
        if self.n_trigger == 1:
            print("[relay] published /plan/trigger (the bridge's GO gate)")
            # 只在第一次打开 12 s 的补发窗口；后续补发不再延长（否则会无限延长）
            self.trigger_until = time.time() + 12.0

    def report(self, trace_csv=""):
        if not self.trace:
            print("[relay] no trace"); return 2
        P = np.array(self.trace)
        T = np.array(self.times)
        # ★ 速度必须用**真实时间戳**算 ★
        #   老实现写死 dt=0.02，可发布循环一旦抖动（rclpy 回调、SC 调度），
        #   同一段位移就会被算成更快的速度 —— 实测报出 9.11 m/s，而 SUPER 的
        #   max_vel 是 6.0：那个数字是量出来的错，不是飞出来的快。
        DT = np.diff(T)
        good = (DT > 0.002) & (DT < 0.2)
        # 只对**空中**那段评分：起飞前的静止样本会把速度分位数拖成 0。
        air = P[:, 2] > 0.5
        boxes = CORNERS * 3
        d = np.array([[math.dist(p[:2], b) for b in boxes] for p in P])
        idx_air = np.where(air)[0]
        F = P[idx_air] if len(idx_air) > 50 else P
        dmin_air = d[idx_air].min(axis=1) if len(idx_air) > 50 else d.min(axis=1)
        min_d = d[idx_air].min(axis=0) if len(idx_air) > 50 else d.min(axis=0)
        hits = int(np.sum(min_d <= 0.25))
        # 逐样本速度：只在 dt 合理的相邻对里算
        pair_ok = good & air[:-1] & air[1:]
        spd = np.linalg.norm(np.diff(P, axis=0), axis=1)[pair_ok] / DT[pair_ok]
        far_pair = (d[:-1].min(axis=1) > 2.0) & pair_ok
        if far_pair.sum() > 10:
            cruise = float(np.percentile(np.linalg.norm(np.diff(P, axis=0), axis=1)[far_pair]
                                        / DT[far_pair], 90))
        else:
            cruise = float(np.percentile(spd, 90)) if len(spd) else 0.0
        vmax = float(np.percentile(spd, 95)) if len(spd) > 10 else 0.0
        vmed = float(np.median(spd)) if len(spd) else 0.0

        # ★ 速度到底该怎么量 ★
        #   上面的 `spd` 有两个已知偏差，不能单独用来下结论：
        #     ① 位置是 FCU 以 20 Hz 推过来的（stream 6），我们以 50 Hz 采样，
        #        于是相邻样本一半是 0、一半是"两拍位移挤在一拍内" → p90 被抬高约 2.5 倍。
        #        实测：这样算出 8.85 m/s，而 SUPER 的 max_vel 只有 6.0 —— 数字是量出来的错。
        #     ② 反过来，若发布循环抖动，dt 变大会低估。
        #   可信的两个口径：
        #     A) FCU 自己的速度估计（LOCAL_POSITION_NED.vx/vy/vz）—— EKF 输出，20 Hz 有效
        #     B) 位置在 **0.5 s 窗口** 上的差分 —— 与采样率无关
        Vn = np.linalg.norm(np.array(self.vel), axis=1)
        mv = air & (Vn > 0.5)
        far = mv & (d.min(axis=1) > 2.0)
        v_ekf_p90 = float(np.percentile(Vn[far], 90)) if far.sum() > 20 else (
            float(np.percentile(Vn[mv], 90)) if mv.sum() > 20 else 0.0)
        v_ekf_med = float(np.median(Vn[far])) if far.sum() > 20 else 0.0
        v_ekf_max = float(Vn[mv].max()) if mv.sum() > 20 else 0.0
        win = []
        for i in idx_air:
            j = int(np.searchsorted(T, T[i] + 0.5))
            if j < len(T) and T[j] - T[i] >= 0.45:
                win.append(np.linalg.norm(P[j] - P[i]) / (T[j] - T[i]))
        v_win_p90 = float(np.percentile(win, 90)) if len(win) > 20 else 0.0
        clr = min(min_clearance(p[0], p[1], self.pillars, p[2], self.z_split) for p in F)
        # 空中时长与实际采样率（自检用：发布循环有没有掉帧）
        t_air = float(len(idx_air)) * (float(np.median(DT)) if len(DT) else 0.02)
        hz = 1.0 / float(np.median(DT)) if len(DT) and np.median(DT) > 0 else 0.0

        if trace_csv:
            try:
                os.makedirs(os.path.dirname(trace_csv), exist_ok=True)
                refs = np.array(self.refs, dtype=float) if self.refs else np.full((len(P), 6), np.nan)
                with open(trace_csv, "w") as fh:
                    fh.write("t_s,x_E,y_N,z_U,vx,vy,vz,d_nearest_box,"
                             "ref_x,ref_y,ref_z,ref_vx,ref_vy,ref_vz\n")
                    for i, p in enumerate(P):
                        fh.write("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                                 "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n"
                                 % (T[i] - T[0], p[0], p[1], p[2], self.vel[i][0],
                                    self.vel[i][1], self.vel[i][2], d[i].min(),
                                    refs[i, 0], refs[i, 1], refs[i, 2],
                                    refs[i, 3], refs[i, 4], refs[i, 5]))
                print("  trace csv           : %s (%d rows)" % (trace_csv, len(P)))
            except OSError as exc:
                print("  trace csv FAILED    : %s" % exc)

        print()
        print("== SITL closed loop ==")
        print("  samples             : %d (airborne %d, %.1f s, %.1f Hz)"
              % (len(P), len(idx_air), t_air, hz))
        print("  z range             : %.2f .. %.2f m" % (P[:, 2].min(), P[:, 2].max()))
        print("  xy extent           : x %.2f..%.2f  y %.2f..%.2f"
              % (F[:, 0].min(), F[:, 0].max(), F[:, 1].min(), F[:, 1].max()))
        print("  box hits (<=0.25 m) : %d / 12" % hits)
        print("  min dist per box    : %s" % " ".join("%.2f" % x for x in min_d))
        print("  speed |v| EKF       : 中位 %.2f  p90 %.2f  峰值 %.2f m/s   (远离盒段)"
              % (v_ekf_med, v_ekf_p90, v_ekf_max))
        print("  speed 位置 0.5s 窗  : p90 %.2f m/s" % v_win_p90)
        # 参考 vs 跟踪：这是"速度为什么上不去"的判据来源
        if self.refs:
            R = np.array(self.refs, dtype=float)
            goodr = air & np.isfinite(R[:, 0])
            if goodr.sum() > 50:
                Rv = np.linalg.norm(R[goodr, 3:6], axis=1)
                Rp = R[goodr, 0:3]
                Pa = P[goodr]
                err = np.linalg.norm((Rp - Pa)[:, :2], axis=1)
                moved = Rv > 0.5
                print("  参考速度 p90        : %.2f m/s（>0.5 m/s 段）  参考峰值 %.2f"
                      % (float(np.percentile(Rv[moved], 90)) if moved.sum() > 20 else 0.0,
                         float(Rv.max())))
                print("  跟踪误差 水平 rms/p95: %.3f / %.3f m  (最大 %.3f)"
                      % (float(np.sqrt((err ** 2).mean())), float(np.percentile(err, 95)),
                         float(err.max())))
        print("  speed 逐样本(旧口径): 中位 %.2f p90 %.2f —— 受 20 Hz 位置更新影响，仅供参考"
              % (vmed, cruise))
        print("  min trunk clearance : %.3f m" % clr)
        # 判据用 EKF 速度：那是 FCU 自己的量，不受我们采样方式影响
        ok = hits >= 10 and v_ekf_p90 >= 4.0 and clr > 0.0
        print("  VERDICT: %s" % ("PASS" if ok else "FAIL"))
        return 0 if ok else 2


class Pilot:
    """MAVLink 侧自动流程。每个 tick 推进一步，绝不阻塞 odom 发布。"""

    S_HB, S_SET, S_POS, S_GUIDED, S_ARM, S_TAKEOFF, S_VERIFY, S_FLY = range(8)
    NAMES = ("HB", "SET", "POS", "GUIDED", "ARM", "TAKEOFF", "VERIFY", "FLY")

    def __init__(self, m, args, node):
        self.m = m
        self.args = args
        self.node = node
        self.state = self.S_HB
        self.t_state = time.time()
        self.have_hb = False
        self.armed = False
        self.mode = ""
        self.sets = [(kv.split("=", 1)[0].strip(), float(kv.split("=", 1)[1]))
                     for kv in args.set_params.split(",") if "=" in kv]
        self.arm_try = 0
        self.next_arm = 0.0
        self.next_mode = 0.0
        self.mode_try = 0
        self.stream_asked = False
        self.next_stream = 0.0
        self.mapping = {}
        self.takeoff_sent = 0.0
        self.takeoff_n = 0
        self.verify = {}
        self.wrote = False
        self.verify_asked = False
        self.texts = []

    # ---------- 收包回调 ----------
    def on_heartbeat(self, msg):
        self.have_hb = True
        self.armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
        self.mode = mavutil.mode_string_v10(msg)

    def on_param(self, msg):
        nm = _txt(msg, "param_id")
        if nm in VERIFY_NAMES:
            self.verify[nm] = msg.param_value

    def on_position(self, msg):
        self.node.p = ned_to_enu((msg.x, msg.y, msg.z))
        self.node.v = ned_to_enu((msg.vx, msg.vy, msg.vz))
        if self.args.fake_z is not None:
            self.node.p[2] = self.args.fake_z
        self.node.have_pos = True
        self.node.n_msg += 1

    def on_text(self, msg):
        s = _txt(msg, "text")
        if s:
            self.texts.append(s)
            print("[FCU] %s" % s)

    # ---------- 状态机 ----------
    def enter(self, st):
        self.state = st
        self.t_state = time.time()
        print("[relay] -> %s" % self.NAMES[st])

    def write_params(self):
        for nm, val in self.sets:
            # param_type 按 ArduPilot 自己的说明填：它在 handle_param_set 里**只**用
            # 参数自身的 var_type 做转换，packet.param_type 完全不参与（GCS_Param.cpp:265）。
            # 所以 6/9 都行，别在这一点上再猜。
            self.m.mav.param_set_send(self.m.target_system, self.m.target_component,
                                      nm.encode(), val, 6)
            time.sleep(0.05)
        print("[relay] set-params sent: %s"
              % ", ".join("%s=%s" % (n, fmt(v)) for n, v in self.sets))

    def ask_position_stream(self, quiet=False):
        """向 FCU 要 LOCAL_POSITION_NED。

        ★ `request_data_stream` 的第 3 个参数是 **MAV_DATA_STREAM 枚举 0..12**，
          不是消息 id。老代码传 (30, 25) 想当然当成"消息 30 = 位置"，
          SITL 收到越界枚举直接丢掉，于是 relay 一帧位置都没收到（msgs=0），
          飞机明明在动，我们发出去的 odom 却永远是 (0,0,0)。
          现代接口是 SET_MESSAGE_INTERVAL（cmd 511）：按**消息 id** + 周期(µs) 要。
        ★ 而且必须**反复要**：FCU 启动期发出去的 511 可能永久不生效。
        """
        for msg_id, hz in ((32, 50), (30, 20), (33, 10)):
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                float(msg_id), float(int(1e6 / hz)), 0, 0, 0, 0, 0)
        # 老的 POSITION 流（stream 6）也要一份，双保险
        self.m.mav.request_data_stream_send(
            self.m.target_system, self.m.target_component, 6, 20, 1)
        if not quiet:
            print("[relay] asked LOCAL_POSITION_NED via SET_MESSAGE_INTERVAL(msg 32 @ 50 Hz)"
                  " + stream 6 @ 20 Hz; will re-ask every 3 s until one arrives")

    def tick(self, now):
        el = now - self.t_state
        st = self.state

        if st == self.S_HB:
            if self.have_hb:
                print("[relay] heartbeat ok, mode=%s" % self.mode)
                self.mapping = self.m.mode_mapping() or {}
                self.enter(self.S_SET)
            elif el > 25.0:
                print("[relay] NO HEARTBEAT after 25 s")
                self.touch_flag()
                self.enter(self.S_FLY)

        elif st == self.S_SET:
            # 等 SITL 把 --defaults 重载完（日志里 "Loaded defaults" 会刷几十次）再写
            if el >= self.args.param_delay:
                self.write_params()
                self.enter(self.S_POS)

        elif st == self.S_POS:
            # ★ 要一次就够是错的 ★
            #   bridge 的源码里写着同一条实测结论：511（SET_MESSAGE_INTERVAL）
            #   如果在 FCU 还在启动时就发出去，之后**永远是 0 Hz**；规则是
            #   "request, MEASURE, and keep asking until it arrives"。
            #   本轮现场：t=2 s 发了 511，到 t=30 s 一帧都没有；而同一时刻发出的
            #   老式 stream 6 请求在 t≈37 s（EKF 用上 GPS 之后）就开始按 20 Hz 到货。
            if now >= self.next_stream:
                self.next_stream = now + 3.0
                self.ask_position_stream(quiet=self.stream_asked)
                self.stream_asked = True
            if self.node.have_pos:
                print("[relay] position alive after %.1f s: p=(%.2f,%.2f,%.2f)"
                      % (el, self.node.p[0], self.node.p[1], self.node.p[2]))
                self.enter(self.S_GUIDED)
            elif el > 70.0:
                print("[relay] NO LOCAL_POSITION_NED after 70 s; texts: %s" % (self.texts[-4:] or "none"))
                self.touch_flag()
                self.enter(self.S_VERIFY)

        elif st == self.S_GUIDED:
            # ★ 必须**确认**模式真的切过去了再解锁/起飞 ★
            #   Copter 进 GUIDED 要求已经有位置估计；GPS/EKF 还没收敛时 set_mode 被静默拒绝。
            #   老代码只 set_mode 一次、然后 sleep 1 s 就 arm —— 结果整场都在 STABILIZE 里，
            #   TAKEOFF 命令当然无效（现场：mode=STABILIZE armed=1 msgs=0，飞机一动不动）。
            if self.mode == "GUIDED":
                print("[relay] GUIDED confirmed")
                self.enter(self.S_ARM)
            elif now >= self.next_mode:
                self.next_mode = now + 2.0
                self.mode_try += 1
                if "GUIDED" in self.mapping:
                    self.m.set_mode(self.mapping["GUIDED"])
                if self.mode_try == 1:
                    print("[relay] set_mode(GUIDED) requested; retrying every 2 s until it sticks")
            elif el > 45.0:
                print("[relay] GUIDED never confirmed (mode=%s, %d tries)" % (self.mode, self.mode_try))
                self.touch_flag()
                self.enter(self.S_VERIFY)

        elif st == self.S_ARM:
            if self.armed:
                print("[relay] ARMED")
                self.enter(self.S_TAKEOFF)
                return
            if self.arm_try < 6 and now >= self.next_arm:
                # 第 3 次起用 force-arm 魔数 21196。注意它**只**跳过 pre_arm_checks，
                # 换来的代价是改跑 mandatory_checks —— 而 GUIDED 需要位置，
                # 所以 GPS 没收敛时 force 也解不开（这是实测结论，不是猜）。
                force = 21196.0 if self.arm_try >= 2 else 0.0
                self.m.mav.command_long_send(
                    self.m.target_system, self.m.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                    1, force, 0, 0, 0, 0, 0)
                print("[relay] arm attempt %d (force=%d)" % (self.arm_try + 1, int(force)))
                self.arm_try += 1
                self.next_arm = now + 3.0
            elif self.arm_try >= 6 and now >= self.next_arm + 3.0:
                print("[relay] ARM FAILED after 6 attempts; texts: %s" % (self.texts[-4:] or "none"))
                self.enter(self.S_VERIFY)

        elif st == self.S_TAKEOFF:
            if self.takeoff_sent == 0.0 or (not self.z_ok() and now - self.takeoff_sent > 4.0
                                            and self.takeoff_n < 6):
                self.m.mav.command_long_send(
                    self.m.target_system, self.m.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                    0, 0, 0, 0, 0, 0, float(self.args.takeoff_z))
                self.takeoff_sent = now
                self.takeoff_n += 1
                print("[relay] takeoff #%d -> z=%.2f (now z=%.2f)"
                      % (self.takeoff_n, self.args.takeoff_z, self.node.p[2]))
            if self.z_ok():
                self.touch_flag()
                print("[relay] airborne at z=%.2f after %.1f s" % (self.node.p[2], el))
                self.node.go()
                self.enter(self.S_VERIFY)
            elif el > 30.0:
                print("[relay] TAKEOFF gave up at z=%.2f" % self.node.p[2])
                self.touch_flag()
                self.enter(self.S_VERIFY)

        elif st == self.S_VERIFY:
            # 回读必须走 PARAM_REQUEST_LIST（整表）：单发 PARAM_REQUEST_READ 在
            # 这条链路上实测不回（历史现场：等满 2 s 一个 PARAM_VALUE 都没有）。
            if not self.verify_asked:
                self.m.mav.param_request_list_send(self.m.target_system, self.m.target_component)
                self.verify_asked = True
            if el > self.args.verify_s:
                got = self.verify
                print("[relay] verify (%d/%d): %s"
                      % (len(got), len(VERIFY_NAMES),
                         ", ".join("%s=%s" % (k, fmt(got[k])) for k in VERIFY_NAMES if k in got)))
                miss = [k for k in VERIFY_NAMES if k not in got]
                if miss:
                    print("[relay] verify MISSING: %s" % ", ".join(miss))
                self.enter(self.S_FLY)

        # S_FLY：什么都不做，odom 继续发到时间到

    def z_ok(self):
        return self.node.p[2] >= self.args.takeoff_z - 0.35

    def touch_flag(self):
        if self.args.takeoff_flag:
            try:
                with open(self.args.takeoff_flag, "w") as fh:
                    fh.write("z=%.2f\n" % self.node.p[2])
            except OSError as exc:
                print("[relay] cannot write flag: %s" % exc)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mav", default="udpin:127.0.0.1:14550")
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--no-arm", action="store_true")
    ap.add_argument("--fake-z", type=float, default=None,
                    help="对照实验用：把发布的 z 强行设成该值（隔离'地面起点'假设）")
    ap.add_argument("--set-params", default="",
                    help='启动后强设参数，"NAME=VAL,NAME=VAL"（名字必须是固件里真实存在的）')
    ap.add_argument("--param-delay", type=float, default=2.0,
                    help="收到心跳后等多久再写参数（让 SITL 把 --defaults 重载完）")
    ap.add_argument("--takeoff-z", type=float, default=Z_FLY)
    ap.add_argument("--takeoff-flag", default="/tmp/takeoff_ok",
                    help="起飞完成（或放弃）后 touch 的文件，shell 等它再启 SUPER/bridge")
    ap.add_argument("--verify-s", type=float, default=30.0,
                    help="整表回读窗口；1370 个参数要 20 s 以上，窗口太短会误报 MISSING")
    ap.add_argument("--trace-csv", default="",
                    help="把真实飞行轨迹（含参考、速度、到最近盒距离）落盘成 csv，作为验收证据")
    ap.add_argument("--ref-topic", default="/plan/cmd",
                    help="记录参考轨迹用；与 bridge 收到的同一条流")
    args = ap.parse_args()

    if args.takeoff_flag:
        try:
            os.unlink(args.takeoff_flag)
        except OSError:
            pass

    rclpy.init()
    node = Relay(args)

    print("[relay] connect", args.mav)
    m = mavutil.mavlink_connection(args.mav, source_system=252)
    # SITL 端是 udpclient，我们要先把对端地址教会，再等心跳
    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                         mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    m.mav.request_data_stream_send(1, 1, 30, 25, 1)
    time.sleep(0.4)

    pilot = Pilot(m, args, node)
    if args.no_arm:
        pilot.state = Pilot.S_FLY      # 只当纯 odom 中继用

    t0 = time.time()
    next_pub = 0.0
    next_hb = 0.0
    next_log = 5.0
    while rclpy.ok() and time.time() - t0 < args.seconds:
        now = time.time()
        # ① 收包（永不阻塞：SITL 的流速率不保证，阻塞会把发布节拍绑在它身上）
        while True:
            msg = m.recv_match(blocking=False)
            if msg is None:
                break
            t = msg.get_type()
            if t == "HEARTBEAT":
                pilot.on_heartbeat(msg)
            elif t == "PARAM_VALUE":
                pilot.on_param(msg)
            elif t == "LOCAL_POSITION_NED":
                pilot.on_position(msg)
            elif t == "STATUSTEXT":
                pilot.on_text(msg)
        # ② 心跳 & 状态机
        if now >= next_hb:
            next_hb = now + 1.0
            m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                 mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        pilot.tick(now)
        # ③ odom 发布（50 Hz，从 t=0 起就发）
        if now - t0 >= next_pub:
            next_pub += 0.02
            node.publish()
            node.trace.append(node.p.copy())
            node.times.append(now)
            node.vel.append(node.v.copy())
            node.refs.append(node.ref if node.ref is not None else (np.nan,) * 6)
        # trigger 补发：bridge 是起飞后才起的进程，靠 latched + 重发把闸门打开
        if node.trigger_until and now < node.trigger_until and now >= node.next_trigger:
            node.next_trigger = now + 1.0
            node.go()
        if now - t0 >= next_log:
            next_log += 5.0
            print("[relay] t=%.0fs state=%s z=%.2f p=(%.2f,%.2f,%.2f) armed=%d mode=%s msgs=%d"
                  % (now - t0, pilot.NAMES[pilot.state], node.p[2],
                     node.p[0], node.p[1], node.p[2], int(pilot.armed), pilot.mode, node.n_msg))
        rclpy.spin_once(node, timeout_sec=0.001)

    pilot.touch_flag()
    rc = node.report(args.trace_csv)
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
