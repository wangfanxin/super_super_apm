#!/usr/bin/env bash
# One round-81 sortie: the HEX mission of docs/SPEC.md, flown the way the spec says.
#
#   fly_hex.sh TAG [ATTEMPTS]
#
# Every default below is a SPEC rule, not a tuning choice -- if one of them has to change, docs/SPEC.md
# changes first and the 修改记录 gets a row.
#
#   WORLD_NAME    forest_hex12.yaml  round 88 map: [-12,12]^2, 522 pillars, 1.0 m cells, no lane,
#                                     the 3x3 block around each waypoint empty (54 cells)
#   SUPER_CONFIG  super_planner_hex.yaml  robot_r 0.10, inflation_step 2, max_vel 1.5, max_acc 6.0,
#                                      max_jerk 15.0 (penna_jerk 常开), goal_pva_en true,
#                                      hot_start_en true, backup_traj_en false, planning_horizon 16
#   S1 到达判据   ARRIVE_M=0.5          a 0.5 m SPHERE around the waypoint (SPEC S1).  SETTLE_SPEED is
#                                      raised above the corner speed on purpose: the arrival test is
#                                      "inside the gate AND (slow OR settle_s elapsed)".
#   S1 PVA 目标   GOAL_PVA=1 CORNER_SPEED=0.0 CORNER_MODE=outgoing
#                                      position = the exact waypoint, velocity = 0 -> the vehicle STOPS
#                                      at every waypoint.  ★That is a SPEC S1 violation on purpose: it is
#                                      the owner's round-88 experiment, and it measured best
#                                      (kh1-02-01 W6 p50 0.219/p90 0.492 vs 0.266/0.770 at 1.5 m/s).
#                                      It therefore needs CORNER_ALLOW_STOP=1 -- see the gate below.★
#                                      history of the exit speed: 4.0 (81) -> 2.0 (84) -> 3.0 (87)
#                                      -> 1.5 (88) -> 0.8 -> 0.0 (88, current)
#                                      waypoint, acceleration = 0.  SUPER needs goal_pva_en=true for this
#                                      (super_planner.cpp:757-768), otherwise the terminal velocity is
#                                      forced to zero and the v=2 goal is silently dropped.
#                                      The pad leg and the LAST waypoint are sent v=0 by the thin layer
#                                      (planner_source_super.py:727), which is what S1 says: they stop.
#   no pre-goal / no lookahead          the goal must BE the waypoint, so --pre-goal-m and
#                                      --goal-lookahead-m are 0: the 4 m/s exit velocity is what carries
#                                      the aircraft through the corner, not a shifted goal.
#   no pass-by                          the arrival test is the sphere; the pass-by half-plane would
#                                      declare arrivals 13 m away (that is why it exists; see cmd_stream).
#   Z_M 2.0, LAPS 3                     the mission stays level at the trunk's square section.
set -u
export SUPER_CONFIG=${SUPER_CONFIG:-super_planner_hex.yaml}
export WORLD_NAME=${WORLD_NAME:-forest_hex12.yaml}
export Z_M=${Z_M:-2.0}
export LAPS=${LAPS:-2}
# Round 87 (user): "3 m/s 且飞完两圈".  The WORLD still holds 3 laps / 18 corners (frozen, SPEC S5,
# check_frozen.py still asserts it); the TOUR flies the first `LAPS` laps of those corners, i.e.
# 2 x 6 = 12 waypoints (+ the pad = 13 W1 targets).  This is an arm decision, not a map change.
export TOUR=${TOUR:-world}
# S1: the 0.5 m sphere, and it has to fire WHILE MOVING (no dwell at a waypoint)
export ARRIVE_M=${ARRIVE_M:-0.5}
export SETTLE_SPEED=${SETTLE_SPEED:-6.0}
export SETTLE_S=${SETTLE_S:-0.2}
export HOLD_S=${HOLD_S:-0}
export PASS_BY_M=${PASS_BY_M:-0.0}
export PASS_BY_LONG_M=${PASS_BY_LONG_M:-0.0}
# S1: the PVA goal
export GOAL_PVA=${GOAL_PVA:-1}
# SPEC S1 (round 87): the PVA exit speed is 3.0 m/s —— 与 `traj_opt.boundary.max_vel` 同步给到 3.0。
# 轨迹：4.0（第 81 轮）→ 2.0（第 84 轮）→ 3.0（本轮）。仍是"过点不停车"：0.5 m 球一到就换下一个航点。
# ★ 第 88 轮实验（用户口述"角点速度设置为 0"）★ 每个航点**停住**再走。
#   动机：沿航迹误差是**漂移**的（kf1-01-01 的四段时基 −0.36 / −0.38 / −0.56 / −0.78 s，越飞越落后），
#   而"每到一个航点就停一下"会把参考与真值在每个角点重新同步，累积不了。
#   代价：这是 **SPEC S1 明确禁止的"在航点停住"**（"参考速度在航点处归零…算违反本条"），所以本趟是
#   **诊断实验**，不是交付配置；两圈时间也会明显变长。
# ★ 0 → 0.8（第 88 轮补 6）★ 两个增益方向的 A/B 都证明：0.3-0.8 Hz 的过冲是被参考的"角点台阶"
#   **激励**出来的（VEL_P 3.2 → 增益 6.68 更差；POS_P 0.8 → 增益 3.25 只降 15 % 而 W6 p90 翻倍）。
#   所以减小台阶本身：角点从"完全停住"改成 0.8 m/s（仍是停-起式重同步，但台阶高度减半）。
# ★ 三个角点档的实测排序：**0（最好）> 0.8 > 1.5**，所以当前生效 = 0 ★
#   corner 0.8（kl1-01-01）：0.3-0.8 Hz 增益 5.86、W6 p50 0.281 / p90 0.813
#   corner 0  （kh1-02-01）：0.3-0.8 Hz 增益 3.83、W6 p50 0.219 / p90 0.492（PASS）
#   机理：不真正停住，参考与真值就少了"在角点重新对齐"的那一下，飞机在角点附近持续荡。
# 角点停住是 SPEC S1 明文禁止的行为，所以它必须被**显式承认**（CORNER_ALLOW_STOP=1），
# 否则 spec_check 会 FAIL、check_static.sh 会拦住起飞（第 88 轮审查 F1）。
export CORNER_ALLOW_STOP=${CORNER_ALLOW_STOP:-1}
export CORNER_SPEED=${CORNER_SPEED:-0.0}
# 第 88 轮：与 max_vel 一起再退到 2.0（雷达仿真在新图只有 6.5-7.7 Hz，估计器撑不住更快）
# 第 88 轮：与 max_vel 同步退到 2.8（3.0 时参考冻住/模型飞走，两张图都复现；见 INTEGRATION §74）
export CORNER_MODE=${CORNER_MODE:-outgoing}
export PRE_GOAL_M=${PRE_GOAL_M:-0}
export GOAL_LOOKAHEAD_M=${GOAL_LOOKAHEAD_M:-0}
# the launch leg from a hover: no speed cap now, because the PVA goal already carries 4 m/s at c1
export ENTRY_MAX_MPS=${ENTRY_MAX_MPS:-0}
export EXTRA_PARMS=${EXTRA_PARMS:-${DSH_ROOT}/params/guided_limits_fast.parm}
# THE SPEC'S PVA GOAL HAS TO REACH THE FCU, ALL THREE PARTS OF IT (user, round 85: "规划的pva都应该送到飞控").
# `run-mission.sh` defaults `BRIDGE_FIELDS=pos` for every planner except ego, so until round 85 the bridge
# masked SUPER's velocity AND acceleration off the wire (type_mask 2552) and the FCU chased a position STEP
# at 50 Hz.  MEASURED, same map / same arm / same sortie count, only this changed:
#     pos     -> 3 and 11 waypoints of 19, reference frozen 76 % of the time, four 40 s HOLDs, geofence abort
#     posvel  -> 19/19 waypoints within 0.50 m in BOTH sorties, no abort, 8 pass / 3 fail, rtf 1.000
# and the FCU's own log says why: with `pos` the error is CROSS-TRACK (lateral p50 0.42 m, along-track
# p50 0.06 m), the signature of a position loop with no velocity feed-forward.
#
# posvelacc is the full PVA.  IT SHIPS WITH CLAMPS, and the clamps are the plan's own limits, not new ones:
# ArduPilot's `shape_pos_vel_accel_xy` adds the acceleration it receives to the velocity target UNSHAPED,
# and a replan replaces the polynomial, so two consecutive 20 ms samples can carry a 5-6 m/s^2 acceleration
# STEP (scripts/accel_quality.py: 4.7 % of steps > 1.0 m/s^2, max 6.73).  posvelacc was flown unclamped
# once before (lio_avoid/docs/STATUS.md:158, Flight D) and it cost 2.7x tracking and a -0.226 m pillar
# penetration.  So the bridge clamps the FORWARDED feed-forward to the plan's own envelope
# (|v| <= max_vel, |a| <= max_acc, |da/dt| <= max_jerk) and leaves the position untouched.
#
# ROUND 87: those three numbers are no longer typed in here.  They were typed in for rounds 85-86 and the
# config moved underneath them -- round 86 raised max_acc 5.0 -> 8.0 while this line stayed at 5.0, so the
# bridge clipped the very acceleration the plan uses to shorten the corner bow (bow = v^2/a) for a whole
# round; and this round's max_vel 3.0 would have been clipped by the stale 2.5, which is S1's PVA being
# rewritten by the bridge.  The envelope is now READ FROM THE SAME YAML the planner loads
# (scripts/cfg_envelope.py), so a stale clamp cannot happen, and the values are echoed into the session log.
export BRIDGE_FIELDS=${BRIDGE_FIELDS:-posvelacc}
_ENVDIR="$(cd "$(dirname "$0")/.." && pwd)"
_ENVLINE="$(python3 "$_ENVDIR/scripts/cfg_envelope.py" "$_ENVDIR/config/$SUPER_CONFIG" 2>/dev/null)" || _ENVLINE=""
FF_V=""; FF_A=""; FF_J=""
if [ -n "${_ENVLINE:-}" ]; then read -r FF_V FF_A FF_J <<<"$_ENVLINE"; fi
# ★ 不再有"静默回退"（第 88 轮审查 F10）★ 以前 cfg_envelope 一失败就退回 3.0/8.0/40.0，
#   而配置是 1.5/6.0/15.0 —— 桥会把加速度放宽 33 %、把 jerk 放宽 2.7 倍，且没有任何报错。
#   现在读不到就直接拒绝起飞。
if [ -z "${FF_V:-}" ] || [ -z "${FF_A:-}" ] || [ -z "${FF_J:-}" ]; then
  echo "[fly_hex] FATAL: scripts/cfg_envelope.py 没能从 config/$SUPER_CONFIG 读出 (v,a,j)。"
  echo "          拒绝用猜测的钳位起飞（以前会静默用 3.0/8.0/40.0）。"
  exit 4
fi
echo "[fly_hex] feed-forward envelope from config/$SUPER_CONFIG: vel<=$FF_V acc<=$FF_A slew<=$FF_J"
# `--fcu-odom-topic` makes the bridge publish the FCU's OWN estimate on /fcu/odom.  The supervisor reads
# it as a second opinion for the geofence (round 85: the tap is emitted in "the simulator origin, i.e. the
# EKF origin", so when the EKF re-seats, the tap reports hundreds of metres that the aircraft never flew --
# measured 410.6 m of altitude against the FCU's 3.6 m, and four geofence aborts of which at least two were
# false alarms).  Publishing it is free: the bridge already parses those messages.
export BRIDGE_ARGS="${BRIDGE_ARGS:---ff-vel-max $FF_V --ff-acc-max $FF_A --ff-acc-slew $FF_J --fcu-odom-topic /fcu/odom}"
# Round 88: the hexagon's corners are at 10.000 m, so the supervisor's geofence must be wider than
# that or the first corner overshoot aborts the sortie (the old map had 5 m corners and 10.5 m).
# The arena walls are at 12.5 m; 12.0 m is the widest fence that still guards them.
# ★ 第 88 轮用户拍板 A：围栏 12.0 → 13.0 m ★
#   依据 ke5-01-02：c4(−10,0) 处 规划 −10.52 / 飞机 −11.77 / 真值 −12.01 ⇒ 撞 12.0 m 围栏。
#   角点到围栏只有 2.0 m，而角点外冲实测 1.25 m、横向残差 p90 0.28 m —— 余量不够是把两圈飞不完的
#   直接原因。13.0 m 让"角点 10.0 + 外冲"之后还有 ~1.7 m。
#   注意：arena 墙 ±12.5 m 只存在于雷达仿真里（SITL 物理是自由飞行），所以围栏是**中止判据**，
#   不是一堵墙；世界文件与航点**没有改动**（SPEC S5）。
export GEOFENCE_M=${GEOFENCE_M:-13.0}
# ★ 参考调速器（第 88 轮打开；第 87 轮 §73.1 的钥匙）★
#   不打开时，薄层发布的是 SUPER 每拍的原始样本（位置是"滑一下"、速度是样条自己的值），两者不自洽，
#   飞控带着 posvelacc 前馈就会**跑到参考前面去**（实测 hx8-01-02：cmd 9.13 / truth 12.16 = 超前 3.0 m，
#   而估计器 lio→W 11.89 是对的）。打开后参考是一条自洽轨迹：
#       v = clamp(v_super + kp*(p_super - p_ref), vmax)，p_ref += v*dt
#   位置与速度同源、处处连续、|v| 有界、|dv/dt| 有界；ALONG=1 只沿航迹前进，横向偏差恒为 0（不切角）。
#   第 87 轮打开它的那一趟 v3h132-01-01 是唯一"规划净空 +0.090 m、跟踪 p50 0.25 m"的趟。
#   第 88 轮修正两处（hx9-01-01 撞围栏后查出来的）：
#     MAXLAG 0.15 → 0：夹回来的那 0.15 m 位移**不在发布速度里**，位置流局部跑到 2.4 m/s，
#       而速度字段仍 ≤1.07 —— 这正是飞控把参考「追过头」的输入。归零后 p_ref 的导数恒等于
#       发布速度 v（一阶滞后 δ=v_super/kp），处处自洽。
#     KP 6 → 12：滞后 δ 在 2.8 m/s 时从 0.47 m 降到 0.23 m（原来被 clamp 掩盖着）。
#   CORNER_SPEED 2.8 → 1.5：120° 拐角处半径 R=v²/max_acc，2.8 m/s 是 0.98 m ⇒ 路径外凸 1.3 m，
#     围栏 12.0 − 1.3 − 跟踪 0.7 = 正好撞线（实测 cmd_x 到 11.31、truth 12.02）。1.5 m/s 时 R=0.28 m。
export REF_GOV_KP=${REF_GOV_KP:-12.0}
# ★ 参考加速度上限：试过 4.0，**更差，已回到 8.0**（第 88 轮补 3 的 A/B）★
#   acc 4.0（kf1-01-01，配 13 m 围栏）：两圈飞完、10 过 2 败，但跟踪明显变差 ——
#     规划停住 11.2 % → 23.1 %、沿航迹 p90 0.637 → 1.711 m、中位偏差 0.229 → 0.451 m、W6 p50 0.505。
#   机理：参考自己的加速度限到 4 后，它爬升更慢、掉进"停住"的拍子更多，飞机跟着落后再猛追。
#   acc 8.0（ke1-01-01）：W6 p50 0.266、横向 p50 0.106、停住 11.2 %。
#   所以两者取各自的好：**参考加速度 8.0 + 围栏 13.0**。
# ★ 参考加速度 8.0 → 2.5（第 88 轮补 10）★
#   实测（scripts/seam_direction.py，kn3-01-01 全部 11 次轨迹安装）：每次换轨迹，发布速度在 ~0.1 s 内
#   从 1.40 掉到 1.01（或 1.22→0.65 等），**方向几乎不变（≤4°）**。也就是"参考在每个航点速度台阶式下降"，
#   而飞机还带着 1.4 m/s 的动能 ⇒ 冲过角点 ⇒ 位置环把它拉回刚过的那点 ⇒ 往复。
#   8 m/s² 的限幅允许 0.1 s 内变 0.8 m/s，所以这个台阶原样通过。降到 2.5 后同样的台阶变成 0.16 s 的斜坡，
#   角点的"停"才真的是停（规划本身已经在角点减速到 0：CORNER_SPEED=0）。
# **试过 2.5：更差，回到 8.0**（第 88 轮补 10 的 A/B）
#   动机：换轨迹时发布速度台阶下降（1.40 → 1.01，11 次安装全都如此，scripts/seam_direction.py）。
#   结果：真值后退总量 7.5 m（acc 8）→ **9.1 m**（acc 2.5）⇒ 台阶不是往复的主因。
#   低加速度上限只是让参考更长时间慢于飞机，冲出去的量反而更大。
# ★ 减速沿单独限速（第 88 轮补 11）★ 加速仍是 8.0；减速限到 2.0：
#   SUPER 的 plan 会在腿中间把速度掉到 0.2 m/s（11 次/140 s，离角点 3.7-4.9 m），
#   参考一拍跟到底、飞机冲过去再回来 —— 这是"航点之间也有往复"的直接原因。
# ★ 参考的横向上限 = 任务的规划上限（第 88 轮审查 F1）★ 以前是 6.0（run_until_healthy 的默认），
#   即规划器 max_vel 1.5 的 4 倍；参考因此在 0.5 m 缺口时被顶到 6 m/s，而桥只钳速度不钳位置。
# ★ 第 20 轮（用户"按A走"）：1.5 → 1.35，与交付配置的 max_vel 同步 ★
#   这两处是同一个上限的两道钳位（配置给规划器，这里给薄层的参考），必须相等；不等时小的那个赢，
#   实测见 §89（规划器 2.5 + 这里 1.5 = 完全没有提速）。依据：18 趟前沿，1.35 是唯一
#   "W2 过 1.40 且 W4 自己转正"的点（W6 0.298、W4 2/3 CLEAR、2/3 趟 14/0）。详见 §106/§107/§110/§112。
export REF_MAX_MPS=${REF_MAX_MPS:-1.35}
export REF_GOV_ACC=${REF_GOV_ACC:-8.0}
# 试过 2.0：**更差**（真值后退 7.5 → 16.4 m、W6 p90 0.433 → 1.007）⇒ 关（0 = 与加速同限 8.0）。
#   连同对称 2.5（后退 9.1 m）一起看：**给参考的下降沿加阻尼两次都让飞机更荡**，
#   说明瓶颈不在参考，而在姿态/角速率内环的相位滞后（下一层才该动那里）。
export REF_GOV_DEC=${REF_GOV_DEC:-0.0}
# 1 = 一阶投影：**实测最好**（ke1-01-01 W6 p50 0.266，横向 p50 0.106）。它的问题是在拐弯处停住
#     （44 次/趟、19 次贴角点），所以配一个"不许停"的下限 REF_ALONG_FLOOR = 规划速度的 0.6。
# 2 = 弧长同步：实测更差（ke3-01-01 W6 p50 0.314 / p90 0.911），而且没加追赶帽时会跑飞
#     （ke2-01-01 参考 6 m/s、飞机到 y=68.9 m）。保留可用，默认关。
export REF_GOV_ALONG=${REF_GOV_ALONG:-1}
export REF_GOV_KS=${REF_GOV_KS:-4.0}
# 下限试过 0.6（ke4-01-01）：那一趟 SUPER 把路线规划到了场外（cmd x 到 24.7 m），无法归因于这个下限，
# 但也没有证据说它有用 —— 所以默认 0（关），开关保留。**当前生效的是 ke1-01-01 的实测最优组合**：
#   mode 1 + 加速度前馈（W6 p50 0.266 / 横向 p50 0.106），inflation 2，max_acc 9。
export REF_ALONG_FLOOR=${REF_ALONG_FLOOR:-0.0}
# 参考最多比规划自己快 0.5 m/s（ke2-01-01：没有这个帽时参考被命令到 6 m/s，飞机飞出场地到 y=68.9 m）
export REF_CATCHUP_MPS=${REF_CATCHUP_MPS:-0.5}
# ★ 超前补偿（第 88 轮）★  位置环实测滞后 0.3-0.4 s ⇒ 参考必须是"0.35 s 以后的那个点"，
#   否则飞机永远落后 v*0.4 ≈ 0.6 m（ja1-01-01 的 W6 p50 0.418 m、W4 擦碰 -0.097 m）。
#   拐角不超前（规划转向 >1.5 rad/s 时 fade→0），所以不引入抄近道。
# 实测结论：超前 0.35 s **让残差变差**（W6 p50 0.375 → 0.552），所以默认 0 = 关。
#   ja1-02-02  lead 0.00 : W1 13/13, W1b 13/13 (worst 0.11 m), W6 p50 0.375 p90 0.827
#   jb1-01-02  lead 0.35 : W1 13/13, W1b 13/13 (worst 0.08 m), W6 p50 0.552 p90 1.394
# 机理：本机的残差主要是**横向**跟不上规划的蛇形，不是在航迹上落后；把一个沿航迹的关注点提前，
# 只是让位置环要更大的速度，外冲反而更大。开关保留（--ref-lead-s / REF_LEAD_S），以后内环更快再说。
# ★ 加速度前馈上限（第 88 轮补）★ 以前薄层把 a 恒发 0（实测 wire.csv 全程 |a| = 0.000），
#   沿航迹残差 p90 0.61 m 就是它的形状。薄层现在发 a = dv/dt；这一行只是**二道保险**，
#   真正的钳位在桥里（`--ff-acc-max` 由 scripts/cfg_envelope.py 从配置读，现在是 9.0）。
#   改 max_acc 时这里跟着改（spec_check 会核对配置，桥会按配置钳）。
# 薄层的加速度前馈上限 = 规划器自己的 max_acc（同一条 envelope，不再手写 9.0）
export REF_FF_ACC_MAX=${REF_FF_ACC_MAX:-$FF_A}
export REF_LEAD_S=${REF_LEAD_S:-0.0}
# ★ 参考低通（第 20 轮）★  τ 秒；0 = 关（交付行为，逐字节不变）。
#   依据 docs/INTEGRATION.md §99.3–§102：W6 零时移 p90 0.34 m = 0.08 m 时基 + 沿航迹钟摆 + 横向 0.19 m；
#   九趟飞控参数手臂（POS_P 1.5 / WP_ACC 6 / ARRIVE_M 1.0 等）都压不下去，而离线把命令流低通到
#   τ=0.05–0.10 s 能把真值-命令 p90 降到 ~0.26 m。这个开关就是把那个低通放到发布路径上。
#   单测：scripts/test_ref_lp.py（τ=0 严格直通、直流增益 1、无过冲、不超前、dt≤0 不动）。
#   用法：REF_LP_TAU=0.08 bash _scratch/fly_arm.sh super_planner_hex.yaml <tag> 3 2
export REF_LP_TAU=${REF_LP_TAU:-0.0}
export REF_LEAD_TURN_RAD_S=${REF_LEAD_TURN_RAD_S:-1.5}
export REF_GOV_MAXLAG=${REF_GOV_MAXLAG:-0.0}   # 0 = 不夹：夹回来的位移不在发布速度里
# W4 must measure the trunk as the BOX it is below z_split 2.1 m: the flight height 2.0 m sits in it
export BODY=${BODY:-box}
# S1 (round 81): the arrival criterion is a 0.5 m SPHERE, so W1b must be scored at 0.5 m, not 1.5 m
export WBOX=${WBOX:-0.5}
export REF_Z_HOLD=${REF_Z_HOLD:-2.0}
export REF_Z_TOL=${REF_Z_TOL:-0.05}   # 竖直只喂位置：z 钉在 2.00±0.05，v_z=0 才是真话
# the hexagon's bounding box is +-5.0 / +-4.33; the fenced square must contain it
export SQUARE_M=${SQUARE_M:-20.0}      # the hexagon's bounding box is +-10 x +-8.66; 20 m is its span
export SQUARE_HALF=${SQUARE_HALF:-10.0}
# ★★★ 起飞前的两道闸（第 88 轮审查 F1/F7）★★★
#   以前 check_static.sh 只被人手工跑过，没有任何飞行脚本调用它；配置的三份副本也只靠人记得
#   跑 install_config_hex.sh。两件都在这里强制。
bash "$_ENVDIR/scripts/install_config_hex.sh" >/tmp/fly_hex_install.log 2>&1 || {
  echo "[fly_hex] FATAL: 配置三份副本不一致（见 /tmp/fly_hex_install.log）"; exit 5; }
bash "$_ENVDIR/scripts/check_static.sh" || {
  echo "[fly_hex] FATAL: STATIC GATE FAILED —— 不飞（运行 bash scripts/check_static.sh 看哪一行）"; exit 6; }
exec bash ${DSH_ROOT}/scripts/fly_speed_run.sh "${1:-hex}" "${2:-3}"
