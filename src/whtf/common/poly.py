"""分段多项式：时间参数化轨迹的唯一表示。

为什么单独一个模块
------------------
L2（规划）产出它是 `t0=None` 的形状；L3（参考服务器）给它填 `t0` 并按墙钟求值。
把这个契约写进类型里，就不可能在"规划器偷偷把 t0 设成 now"这件事上出错。

表示
----
一条 `Trajectory` = 若干段五次多项式，每段有独立的时长。
五次多项式恰好能钉住两端的 (p, v, a) 六个条件 —— 这正是"经过航点盒且不减速"所需要的。

  p(t) = c0 + c1 t + c2 t^2 + c3 t^3 + c4 t^4 + c5 t^5      t ∈ [0, T]

端点条件（x0 = (p0,v0,a0) , x1 = (p1,v1,a1)）解出 6 个系数：
  c0 = p0
  c1 = v0
  c2 = a0 / 2
  其余三个由三元一次方程组给出（见 `quintic`）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

Vec = np.ndarray


def quintic(p0: Vec, v0: Vec, a0: Vec, p1: Vec, v1: Vec, a1: Vec, T: float) -> np.ndarray:
    """解出五次多项式系数，形状 (6, 3)，行是 c0..c5。

    T <= 0 时退化成"静止在 p0"（长度 1 的常数多项式），保证永不抛异常。
    """
    p0, v0, a0, p1, v1, a1 = (np.asarray(x, dtype=float) for x in (p0, v0, a0, p1, v1, a1))
    if not np.isfinite(T) or T <= 1e-9:
        c = np.zeros((6, 3))
        c[0] = p0
        return c

    T2, T3, T4, T5 = T * T, T**3, T**4, T**5
    # A * [c3,c4,c5]^T = b ，其中 A 是 3x3
    A = np.array([
        [T3,        T4,         T5],
        [3 * T2,    4 * T3,     5 * T4],
        [6 * T,     12 * T2,    20 * T3],
    ])
    b = np.vstack([
        p1 - (p0 + v0 * T + 0.5 * a0 * T2),
        v1 - (v0 + a0 * T),
        a1 - a0,
    ])
    c345 = np.linalg.solve(A, b)
    c = np.zeros((6, 3))
    c[0] = p0
    c[1] = v0
    c[2] = 0.5 * a0
    c[3], c[4], c[5] = c345[0], c345[1], c345[2]
    return c


def eval_poly(c: np.ndarray, t: float, order: int = 0) -> np.ndarray:
    """在 t 处求第 order 阶导数（0=p, 1=v, 2=a, 3=jerk）。"""
    if order == 0:
        return c[0] + c[1] * t + c[2] * t**2 + c[3] * t**3 + c[4] * t**4 + c[5] * t**5
    if order == 1:
        return c[1] + 2 * c[2] * t + 3 * c[3] * t**2 + 4 * c[4] * t**3 + 5 * c[5] * t**4
    if order == 2:
        return 2 * c[2] + 6 * c[3] * t + 12 * c[4] * t**2 + 20 * c[5] * t**3
    if order == 3:
        return 6 * c[3] + 24 * c[4] * t + 60 * c[5] * t**2
    raise ValueError("order must be 0..3")


@dataclass
class Segment:
    """一段五次多项式。`t0` 由 L3 赋值，L2 必须返回 None。"""
    coef: np.ndarray
    duration: float
    t0: Optional[float] = None

    def eval_local(self, tau: float, order: int = 0) -> np.ndarray:
        tau = min(max(tau, 0.0), self.duration)
        return eval_poly(self.coef, tau, order)


@dataclass
class Trajectory:
    """一条由若干段拼成的轨迹。段间按构造是 C2 的（端点条件相同）。"""
    segs: List[Segment] = field(default_factory=list)
    t0: Optional[float] = None

    # ---- 构造 ----
    @staticmethod
    def through(states: Sequence[Tuple[Vec, Vec, Vec]], durations: Sequence[float]) -> "Trajectory":
        """按给定 (p,v,a) 序列与每段时长，拼出 C2 轨迹。"""
        segs = []
        for i, T in enumerate(durations):
            p0, v0, a0 = states[i]
            p1, v1, a1 = states[i + 1]
            segs.append(Segment(quintic(p0, v0, a0, p1, v1, a1, T), float(T)))
        return Trajectory(segs)

    # ---- 查询 ----
    @property
    def duration(self) -> float:
        return sum(s.duration for s in self.segs)

    def _locate(self, tau: float) -> Tuple[int, float]:
        if not self.segs:
            return 0, 0.0
        tau = min(max(tau, 0.0), self.duration)
        acc = 0.0
        for i, s in enumerate(self.segs):
            if tau <= acc + s.duration or i == len(self.segs) - 1:
                return i, tau - acc
            acc += s.duration
        return len(self.segs) - 1, self.segs[-1].duration

    def eval_local(self, tau: float, order: int = 0) -> np.ndarray:
        i, lt = self._locate(tau)
        return self.segs[i].eval_local(lt, order)

    def eval_abs(self, t: float, order: int = 0) -> np.ndarray:
        if self.t0 is None:
            raise RuntimeError("Trajectory.t0 is None: it must be assigned by L3 before absolute evaluation")
        return self.eval_local(t - self.t0, order)

    # ---- 审计 ----
    def peak(self, n: int = 200) -> dict:
        """采样统计 v/a 峰值，用于可行性检查与验收。"""
        if not self.segs:
            return {"v": 0.0, "a": 0.0}
        vmax = amax = 0.0
        for s in self.segs:
            for k in range(n + 1):
                tau = s.duration * k / n
                vmax = max(vmax, float(np.linalg.norm(s.eval_local(tau, 1))))
                amax = max(amax, float(np.linalg.norm(s.eval_local(tau, 2))))
        return {"v": vmax, "a": amax}

    def continuity(self, n: int = 20) -> dict:
        """段边界处的 0/1/2 阶跳变（用于验收"缝"）。"""
        out = {"dp": 0.0, "dv": 0.0, "da": 0.0}
        for i in range(len(self.segs) - 1):
            a, b = self.segs[i], self.segs[i + 1]
            for order, key in ((0, "dp"), (1, "dv"), (2, "da")):
                out[key] = max(
                    out[key],
                    float(np.linalg.norm(a.eval_local(a.duration, order) - b.eval_local(0.0, order))),
                )
        return out


def allocate_time(states: Sequence[Tuple[Vec, Vec, Vec]], v_max: float, a_max: float,
                  samples: int = 25, iters: int = 10) -> List[float]:
    """给一串通过状态分配每段时长（迭代放大到动力学可行）。

    `samples` 是每段的峰值检查采样数：段变多以后（稠密重采样会到上百段），
    采样数必须降下来，否则这里会成为整个规划的耗时瓶颈。
    """
    n = len(states) - 1
    if n <= 0:
        return []
    durs = []
    for i in range(n):
        dp = np.linalg.norm(np.asarray(states[i + 1][0]) - np.asarray(states[i][0]))
        v_avg = 0.5 * (np.linalg.norm(states[i][1]) + np.linalg.norm(states[i + 1][1]))
        v_ref = max(v_avg, 0.5 * v_max)
        durs.append(max(dp / max(v_ref, 1e-3), math.sqrt(2 * dp / max(a_max, 1e-3))))

    for _ in range(iters):
        traj = Trajectory.through(states, durs)
        vmax = amax = 0.0
        for s in traj.segs:
            for k in range(samples + 1):
                tau = s.duration * k / samples
                vmax = max(vmax, float(np.linalg.norm(s.eval_local(tau, 1))))
                amax = max(amax, float(np.linalg.norm(s.eval_local(tau, 2))))
        scale = max(vmax / v_max, math.sqrt(max(amax, 1e-9) / a_max), 1.0)
        if scale <= 1.02:
            break
        durs = [d * scale for d in durs]
    return durs
