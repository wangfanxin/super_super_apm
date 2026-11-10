"""三层：L1 box_scheduler / L3 traj_server / L4 pid_control。

三个类各自对应设计文档里的一层，输入输出按契约切好；v1 先放在同一进程里跑，
拆节点时把每个类搬出去即可，接口不用改。

  L1  语义 + 窗口 + 版本 + 推进   —— 不生成几何
  L3  时钟所有者：t0、提交时域、新鲜度、fallback
  L4  参考 → 加速度指令（方案1：PID + 前馈）
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

from whtf.common.poly import Trajectory, quintic
from whtf.planner.base import Box, PlanRequest, PlanResult, run_backend

Vec = np.ndarray
State = Tuple[Vec, Vec, Vec]


# ---------------------------------------------------------------- L1
class BoxScheduler:
    """把 12 个盒编译成边界状态序列，并决定"现在该规划哪一段窗口"。"""

    def __init__(self, boxes: List[Box], v_cruise: float = 5.0, a_lat: float = 3.0,
                 r_flyby: float = 0.9, v_max: float = 6.0, a_max: float = 3.0,
                 window_n: int = 3, advance_r: float = 0.6,
                 finish_point: Optional[Vec] = None):
        self.boxes = boxes
        self.v_cruise = v_cruise
        self.a_lat = a_lat
        self.r_flyby = r_flyby
        self.v_max = v_max
        self.a_max = a_max
        self.window_n = window_n
        self.advance_r = advance_r
        self.finish = finish_point if finish_point is not None else boxes[-1].center.copy()

        self.i_committed = 0      # 正在执行到第几个盒
        self.i_reached = 0        # 已确认经过第几个盒（评分用）
        self.i_planned = 0        # 已规划到第几个盒
        self.ver = 1
        self.done = False
        self.min_dist = [float("inf")] * len(boxes)   # 每个盒的最小接近距离

    # ---- 语义编译（设计文档 §6.1）----
    def _via(self, j: int, p_prev: Vec) -> State:
        c = self.boxes[j].center
        if j >= len(self.boxes) - 1:
            return (c.copy(), np.zeros(3), np.zeros(3))          # 末点静止
        if self.boxes[j].kind == "stop":
            return (c.copy(), np.zeros(3), np.zeros(3))

        p_prev = self.boxes[j - 1].center if j > 0 else p_prev
        p_next = self.boxes[j + 1].center
        d_in = c - p_prev
        d_out = p_next - c
        n_in, n_out = np.linalg.norm(d_in), np.linalg.norm(d_out)
        if n_in < 1e-6 or n_out < 1e-6:
            return (c.copy(), np.zeros(3), np.zeros(3))
        d_in, d_out = d_in / n_in, d_out / n_out

        cosang = float(np.clip(d_in @ d_out, -1.0, 1.0))
        if cosang < math.cos(math.radians(150.0)):              # 折返：退化
            return (c.copy(), np.zeros(3), np.zeros(3))

        v_corner = min(self.v_cruise, math.sqrt(self.a_lat * self.r_flyby))
        v = v_corner * d_out                                        # 沿出向通过
        n = d_out - d_in
        a = (v_corner ** 2 / self.r_flyby) * (n / np.linalg.norm(n)) if np.linalg.norm(n) > 1e-6 \
            else np.zeros(3)
        if np.linalg.norm(a) > self.a_max:
            a = a / np.linalg.norm(a) * self.a_max
        return (c.copy(), v, a)

    def make_request(self, p_cur: Vec, v_cur: Vec, a_cur: Vec) -> PlanRequest:
        """构造窗口请求：从当前状态出发，穿过窗口内剩余的盒。"""
        end = min(self.i_committed + self.window_n, len(self.boxes))
        via: List[State] = [(p_cur.copy(), v_cur.copy(), a_cur.copy())]
        for j in range(self.i_committed, end):
            via.append(self._via(j, p_cur))
        if len(via) == 1:                     # 已过末盒：去悬停点
            via.append((self.finish.copy(), np.zeros(3), np.zeros(3)))
        self.i_planned = end
        return PlanRequest(x0=via[0], x1=via[-1], via=via,
                           boxes=self.boxes[self.i_committed:end],
                           v_max=self.v_max, a_max=self.a_max, ver=self.ver)

    # ---- 推进（只更新索引，不触发规划）----
    def observe(self, p: Vec) -> None:
        for j in range(len(self.boxes)):
            self.min_dist[j] = min(self.min_dist[j], float(np.linalg.norm(p - self.boxes[j].center)))
        while (self.i_committed < len(self.boxes)
               and self.min_dist[self.i_committed] < self.advance_r):
            self.i_committed += 1
            self.i_reached = self.i_committed
            self.ver += 1
        if self.i_committed >= len(self.boxes):
            self.done = all(d < self.advance_r for d in self.min_dist)

    def hit_report(self) -> dict:
        hits = [i for i, d in enumerate(self.min_dist) if d <= self.boxes[i].half[0]]
        return {"hits": len(hits), "n": len(self.boxes),
                "hit_idx": hits, "min_dist": self.min_dist}


# ---------------------------------------------------------------- L3
class TrajServer:
    """唯一知道"现在几点对应轨迹哪一点"的地方。"""

    def __init__(self, v_max: float = 6.0):
        self.traj: Optional[Trajectory] = None
        self.t0 = 0.0
        self.v_max = v_max
        self.n_replans = 0

    def set_trajectory(self, traj: Trajectory, now: float) -> None:
        """新轨迹的 t0 由这里赋值 —— 这是 L-Time-1（t0 ≡ 提交时刻）的唯一实现点。"""
        traj.t0 = now
        self.traj = traj
        self.t0 = now
        self.n_replans += 1

    def age(self, now: float) -> float:
        return float("inf") if self.traj is None else now - self.t0

    def sample(self, now: float):
        """返回 (p, v, a, source)。走完则钉在终点、速度加速度归零。"""
        t = self.traj
        if t is None:
            return np.zeros(3), np.zeros(3), np.zeros(3), "none"
        tau = now - self.t0
        if tau < 0.0:
            return t.eval_local(0.0, 0), np.zeros(3), np.zeros(3), "future"
        if tau <= t.duration:
            return (t.eval_local(tau, 0), t.eval_local(tau, 1),
                    t.eval_local(tau, 2), "active")
        return t.eval_local(t.duration, 0), np.zeros(3), np.zeros(3), "hold"

    def brake(self, p: Vec, v: Vec, a: Vec, a_brake: float = 2.5):  # noqa: ARG002
        """闭式刹车段（设计文档 §11.2）。O(1)，永不失败。"""
        speed = float(np.linalg.norm(v))
        T = min(max(speed / max(a_brake, 1e-3), 0.2), 2.0)
        p_e = p + v * T * 0.5
        coef = quintic(p, v, a, p_e, np.zeros(3), np.zeros(3), T)
        from whtf.common.poly import Segment
        return Trajectory(segs=[Segment(coef, T)])


# ---------------------------------------------------------------- L4
class PidController:
    """方案 1：位置 PID + 速度/加速度前馈 -> 加速度指令。

    设计文档 §8.2 的 Kp/Kd 量级：Kp 3–6 (1/s²)、Kd 3.5–7 (1/s)；高速时关积分。
    """

    def __init__(self, kp: float = 4.0, kd: float = 5.0, ki: float = 0.0,
                 a_max: float = 3.0, v_max: float = 6.0):
        self.kp, self.kd, self.ki = kp, kd, ki
        self.a_max, self.v_max = a_max, v_max
        self.integ = np.zeros(3)

    def update(self, p_ref: Vec, v_ref: Vec, a_ref: Vec, p: Vec, v: Vec,
               dt: float) -> np.ndarray:
        e_p = p_ref - p
        e_v = v_ref - v
        self.integ += e_p * dt
        self.integ = np.clip(self.integ, -1.0, 1.0)      # 抗饱和
        a_cmd = a_ref + self.kp * e_p + self.kd * e_v + self.ki * self.integ
        n = float(np.linalg.norm(a_cmd))
        if n > self.a_max:
            a_cmd = a_cmd / n * self.a_max
        return a_cmd
