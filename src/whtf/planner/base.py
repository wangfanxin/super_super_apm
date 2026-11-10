"""L2 的接口契约：规划器只需要回答 plan(x0, x1, map) -> Trajectory | Fail。

设计文档 §7.1 的 ⑤ 是分水岭：**边界条件是输入，不是输出**。
所以这里的 `PlanRequest.x0/x1` 是只读的，任何后端都无权改写它们 ——
这条约束由 `run_backend()` 在调用前后各校验一次来强制执行。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Protocol, Tuple

import numpy as np

from whtf.common.poly import Trajectory

Vec = np.ndarray


class FailKind:
    INFEASIBLE = "INFEASIBLE"   # 目标不可达（盒在障碍里 / 无通路）
    TIMEOUT = "TIMEOUT"         # 超过 deadline
    LIMITS = "LIMITS"           # 时间参数化后仍越界
    BOUNDARY = "BOUNDARY"       # 边界对齐误差超限 —— 实现 bug，绝不容忍


@dataclass
class Box:
    """一个航点盒 = 可行集 Ω + 通过条件 C（设计文档 §3.1）。"""
    center: Vec
    half: Vec
    kind: str = "flyby"          # pass | flyby | stop
    dwell_s: float = 0.0
    heading: Optional[float] = None
    speed_cap: float = 0.0


@dataclass
class PlanRequest:
    """一次规划请求。x0/x1 由 L1 编译 + L3 时钟共同决定，后端只读。

    `via` 是 L1 编译出的完整边界状态序列（含首尾），L2 必须**逐点**满足它。
    `x0`/`x1` 是 `via[0]`/`via[-1]` 的冗余副本，只用于 run_backend() 的一致性校验。
    """
    x0: Tuple[Vec, Vec, Vec]     # (p, v, a) at t0
    x1: Tuple[Vec, Vec, Vec]     # (p, v, a) at t0 + duration
    via: List[Tuple[Vec, Vec, Vec]] = field(default_factory=list)
    boxes: List[Box] = field(default_factory=list)   # 途经的盒（窗口）
    v_max: float = 6.0
    a_max: float = 3.0
    j_max: float = 15.0
    deadline: Optional[float] = None
    ver: int = 0


@dataclass
class PlanResult:
    ok: bool
    traj: Optional[Trajectory] = None
    kind: Optional[str] = None
    detail: str = ""
    hits: List[Tuple[int, float, Vec]] = field(default_factory=list)  # (box_idx, t, p)


class PlannerBackend(Protocol):
    """后端协议。必须是纯函数式的：不改 req，不读时钟，返回的 traj.t0 为 None。"""
    name: str

    def plan(self, req: PlanRequest) -> PlanResult:
        ...


def run_backend(be: PlannerBackend, req: PlanRequest) -> PlanResult:
    """统一入口：强制校验「边界条件不被改写」。

    这是设计文档 §7.4 里 `BOUNDARY` 那一类失败的唯一正确做法：
    一旦发现后端改了 x0/x1，就判为实现 bug，绝不重试、绝不"用它"。
    """
    res = be.plan(req)
    if res.ok and res.traj is not None:
        p0, v0, a0 = req.x0
        p1, v1, a1 = req.x1
        if np.linalg.norm(res.traj.eval_local(0.0, 0) - p0) > 1e-3:
            return PlanResult(False, kind=FailKind.BOUNDARY, detail="start position moved")
        if np.linalg.norm(res.traj.eval_local(0.0, 1) - v0) > 1e-2:
            return PlanResult(False, kind=FailKind.BOUNDARY, detail="start velocity moved")
        if np.linalg.norm(res.traj.eval_local(0.0, 2) - a0) > 5e-2:
            return PlanResult(False, kind=FailKind.BOUNDARY, detail="start acceleration moved")
        T = res.traj.duration
        if np.linalg.norm(res.traj.eval_local(T, 0) - p1) > 1e-3:
            return PlanResult(False, kind=FailKind.BOUNDARY, detail="end position moved")
        if np.linalg.norm(res.traj.eval_local(T, 1) - v1) > 1e-2:
            return PlanResult(False, kind=FailKind.BOUNDARY, detail="end velocity moved")
        if np.linalg.norm(res.traj.eval_local(T, 2) - a1) > 5e-2:
            return PlanResult(False, kind=FailKind.BOUNDARY, detail="end acceleration moved")
    return res
