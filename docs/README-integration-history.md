# super_super_apm —— 航点 → SUPER → bridge → ArduPilot

## 0. 当前状态（第 88 轮：**新图 `forest_hex12.yaml` 上飞完两圈**；真凶是雷达仿真的光栅化，不是规划）

> ### ⚠ 两条硬约束（用户第 81 轮）：`docs/SPEC.md` 是唯一有效的规矩；**地图与航点已冻结**
>
> `scripts/check_frozen.py` 把新世界文件的 SHA-256（`d30918ef…`）、**522 根柱**、六个角点（半径 10.000 m）
> 写进代码，`scripts/fly_hex2.sh` **每次起飞前跑它**（13 行），任何一条不过就拒绝起飞；本轮没有改动
> 世界文件（13/13 PASS）。世界文件一旦本轮冻结就不再改。

| 项 | 结果 |
|---|---|
| **★★★ 交付：新图两圈飞完；最好一趟 W6 p90 已到 0.43 m** | **`kn3-01-01`**（acc 6 / jerk 15 / corner 0）：0 abort、H1–H5 **全 PASS**、W1/W1b **13/13（最差 0.06 m）**、**W6 p50 0.230 / p90 0.433**、第 2 圈真值净空 **+0.081 m（无接触）**、rtf 1.0005；W6 只差“时基四段跨度 0.16 > 0.15 s”这一条。复算 `bash scripts/render_evidence.sh kn3-01-01` |
| **★★★ 质量指标：全面优于旧冻结图** | 规划**停住 69.1 % → 11.2 %**、重规划跳变 **> 0.3 m 27 次 → 0 次**、横向误差 **p50 0.283 → 0.106 m / p90 0.818 → 0.277 m**、沿航迹 p50 **+0.135 → −0.02 m**、角点自由半径 **0.245 → 1.026 m**；最好一趟（角点 0）W6 p50 **0.219** / p90 **0.492** |
| **★★★ Super-LIO 的 SIGFPE：定位并修好** | 三个 core 的 backtrace 完全一致：`pcl::transformPointCloud` ← `SuperLIO::Output()` ← `stateProcess()` 定时器；故障指令 `div %r9`、**rax = r9 = 0** ⇒ 空点云进了 PCL 的整数 `0/0`。`Output()` 在定时器线程、点云在回调线程，无锁。节点里加两行守卫（空云直接 return），重编 `liblio.so`（旧库备份 `liblio.so.bak-round88`）；**加守卫后 4 趟 0 次 SIGFPE**（此前约每 2–4 次尝试一次）。验证：`bash scripts/lio_guard_verify.sh` |
| **★★★ 真凶：雷达仿真的 trunk 光栅化** | `_cast_trunks_chunk` 的 85.7 ms 不在算术而在**522 次 Python 循环 × 每次 ~15 个 numpy 调用**。新写 `_cast_trunks_flat` 把窗口摊平成一张 (柱, 射线) 对表再一次向量化：**67.3 → 10.7 ms（6.3×）**，并与逐柱循环**逐位一致**（`scripts/check_raycast_flat.py`：0 处不一致）、对 `brute_force` 3.6e-5 m |
| **★★★ 它修好了什么** | cloud **7.00 → 9.97 Hz**、每帧 **3 909 → 20 000 点**、**丢弃扫描 372 → 0**、cast 21–25 → **11–17 ms**。因果链：**7 Hz 还丢帧的雷达云 → LIO 吃不饱 → 飞控外部位姿时断时续 → 位置环发散**，第 74.7 节的"飞机跑在规划前面 3 m"就是这条链的末端 |
| **第二把钥匙：参考必须自洽** | `--ref-gov-maxlag 0.15 → 0`、`kp 6 → 12`：旧夹取把 `p_ref` 硬拉向规划点，而那段位移**不在发布的速度里**（实测速度字段 ≤1.07 m/s 时位置流跑到 2.4 m/s）。归零后 `p_ref` 的导数**恒等于**发布速度（滞后 `δ=v_super/kp`）；竖直同理 `REF_Z_TOL 0.20 → 0.05`（`v_z=0` 才不是谎话） |
| **直道与角点统一 1.5 m/s** | `max_vel` = `CORNER_SPEED` = **1.5**。120° 角点 `R=v²/max_acc`：2.8 m/s ⇒ R=0.98 m，叠加 ~0.4 s 位置环滞后 ≈1.1 m，正好用光"角点 10 m 到围栏 12 m"的 2 m 余量 —— 本轮四次撞线**全在角点**（`hx9-01-01` x=+12.02、`hy0-01-01` x=−12.01） |
| **位置环 PIO（`hz1-01-01`）** | 参考全程 ≤1.50 m/s，真值跑到 **4.1–5.2 m/s** 并掉高触地。机理：`PSC_NE_POS_P 3.0 × 1.5 m` = +4.5 m/s 的追赶需求，`WP_SPD 6.5` 不拦。按用户"pid 参数不够就调固件"，只动振荡的那一层：`WP_SPD 6.5→2.0`、`WP_ACC 10→4`、`JERK 150→80`、`POS_P 3.0→1.2`、`VEL_P 3.0→2.0`，**姿态环一个字没动** |
| **试过并否定：参考超前补偿** | `--ref-lead-s`（目标点沿速度提前 0.35 s、拐角按转向率淡出）：W6 p50 **0.375 → 0.552 / 0.598**，更差。原因：本机残差是**横向跟不上绕柱蛇形**，不是沿航迹落后。默认 0，开关与代码保留 |
| **估计器（不是瓶颈）** | `docs/evidence/lio-err-t.png`：水平 p50 **0.036** / p90 0.057 / max 0.084 m，竖直 p50 **0.012** / p90 0.024 m。比规划净空小一个量级 |
| 仍然不达标的三行（**如实**） | **W2** 4.0 m/s 是第 81 轮旧规格、与本轮 1.5 冲突（实测 p50 1.35 / max 3.45，**阈值一个字没改**）；**W4** 最小柱面净空 −0.096 m，根因是**规划自己**两圈的净空只有 **+0.013 / +0.067 m**（`inflation_step=1` ⇒ 0.10 m 膨胀，小于 MINCO 拐角切角量），比跟踪残差 p90 0.39 m 小一个量级；**W6** p50 0.375 m（其中横向 ~0.15 m） |
| 环境类失败（非本链缺陷） | ① **Super-LIO 偶发 `SIGFPE`**：历史 886 趟里 **46 趟（5.2 %）**，靠重试；② **SITL 模型飞走**（模型与飞控估计几秒内差 70–95 m，LIO 本身仍与真值差 4 cm）；③ **RTF 门 0.98**：好几趟落在 0.978–0.979 被判"不可评分"，**飞的时候不要跑分析/画图** |
| 图（本轮三类 + 地图图） | `docs/evidence/`：`lap1-paths.png`、`lap2-paths.png`（每圈俯视图，标题写该圈规划/真值最小净空与 \|规划−真值\| p50/p90）、`vxyz-t.png`、`lio-err-t.png`、`world-hex12.png`；旧图族全部删除，`scripts/check_evidence_links.sh` 断言它们不在，也断言没有第二种命名方案 |
| 文档 | `docs/INTEGRATION.md` **§75**（真凶、两把钥匙、PIO、超前补偿的否定、十一行、质量指标、下一步三候选）；`docs/SPEC.md` S1 重写 + 修改行；`docs/evidence/README.md` |
| 用户已拍板（第 88 轮补） | ① `rog_map.inflation_step` **1 → 2**（规划净空 0.1 → 0.2 m，针对 W4 接触）；② `max_acc` **8.0 → 9.0**（角点速度上限 2.86 → **3.04 m/s**，3.0 m/s 回到界内）。两处都已落进配置 + SPEC + `spec_check`，静态门 PASS；**尚未用新参数重飞**（下一轮的第一件事） |
| 复算一条链 | `bash scripts/check_static.sh` → `bash scripts/check_evidence_links.sh` → `python3 scripts/spec_check.py` → `python3 scripts/check_raycast_flat.py` → `python3 scripts/tracking_diag.py ke1-01-01` → `bash scripts/render_evidence.sh ke1-01-01` → `python3 scripts/plot_hex_laps.py --tag ke1-01-01 --laps 2 --world worlds/forest_hex12.yaml --prefix lap` |

### 0.1 第 85–86 轮（历史：`posvel` 把航点拉到 19/19；W4 的地板是规划自己扎进柱子）

| 项 | 结果 |
|---|---|
| 用户第 85 轮的两句话 | "robot_r 降到 0.10 m"、"**但是现在飞的一团糟，这是为什么？求解不出来吗？跟踪非常差**" |
| `robot_r 0.10` | 已改：`docs/SPEC.md` S4 + `config/` 三份副本 md5 一致 `e7827164…` + `spec_check` PASS。七条腿的走廊余量 **b − robot_r = 0.145 … 0.424 m 全正**（`set_robot_r.sh 0.10`） |
| **"跟踪差"的答案（一条）** | `run-mission.sh` 对非 ego planner 默认 **`BRIDGE_FIELDS=pos`**，于是 bridge 用 `type_mask 2552` 把 SUPER 的**速度与加速度全屏蔽**，飞机只收到 50 Hz 的**位置阶跃**。FCU 自己的 `PSCN/PSCE` 日志（`scripts/fcu_tracking.py`）显示误差是**横向**的：\|DP−P\| p50 **0.610 m** 而纵向 p50 只有 **+0.063 m** —— 这就是"位置环没有速度前馈"的签名 |
| 为什么横向误差这么大（物理上限） | 新工具 `scripts/path_curvature.py`：冻结地图把 SUPER 的路径逼到 **R p10 = 0.39–0.48 m**；45° 倾角只能给 9.81 m/s² ⇒ 这种弯的速度上限 **1.96–2.17 m/s**，而参考跑到 **4.1 m/s**（2 m/s、R 0.39 m 时需 **10.3 m/s²**）。第 79 轮的宽车道世界 R p10 = **1.79 m**（上限 4.19 m/s），同一套控制器只差 **0.090 m** ⇒ 差别不在控制器，在**弯太急** |
| 参考还会"停" | `scripts/plan_anatomy.py`：`d85-01-02` 全程只有 **13 条轨迹**，其中**四条长度正好 40.18 s 的 HOLD**（= 薄层 `--leg-timeout-s`）⇒ 两段 **81.6 s / 83.2 s 静止**，期间真值漂 6.2 m。MINCO 失败 3759 次是它的上游原因 |
| **PVA 到底有没有上过线**（用户第 85 轮追问） | **没有。** `wire.csv` 的 `type_mask` 记着每一轮：`v4a`/`d84`/`d85`/`r10` 全是 **2552（pos，只有位置）**，`pv1` 是 **2496（posvel）**，`pva1` 才是 **2048（posvelacc）**。SUPER 的 v/a 一直在 `setpoints.csv` 里（那是桥的日志），只是没上线——两份日志不摆在一起看不出来。现在按用户要求**全 PVA 上线**，并给前馈加了钳位（`--ff-vel-max 2.5 / --ff-acc-max 5.0 / --ff-acc-slew 40`，= SUPER 自己的 max_vel/max_acc/max_jerk） |
| 全 PVA 的代价（`pva1` 两趟） | 航点最准：**W1/W1b 19/19，最差 0.13 m**；但 **W3 挂**（真值冲到 7.09 m；第二趟 bias **+1.176 m**），W6 0.87–1.16。~~机制：ArduPilot 把加速度不整形加到速度目标上 ⇒ 被 z 向加速度顶起来~~ **这个归因已被第 85 轮的 a_z=0 实验推翻：**`az0` 两趟（a_z 强制 0）仍然一趟 **80.89 m**、一趟 2.76 m；异常时 FCU 日志里有 `SIM Hit ground`、`EKF variance/Failsafe`、`Potential Thrust Loss`，tap 与 FCU 同时被带走、LIO 不动 ⇒ 是 **EKF/SITL 失稳**，与加速度通道无关（详见 `docs/INTEGRATION.md` §63）。加速度上线确实会让 FCU 的 z 目标偏离命令（14.6 % vs 1.1 % 的样本高于命令 0.5 m），但它不是那些跳变的成因 |
| **命名消歧** | 规划器配置 **`config/super_lio_v2_08.yaml` → `config/super_planner_hex.yaml`**（旧名把 SUPER 规划器和 Super-LIO（SLAM）混在一起）；两份安装副本 + 11 个引用脚本 + CHAIN/README 同步，三份 md5 `ad9855c2…`，旧名在活文件里零命中（只剩改名脚本与配置头的说明）；**Super-LIO 的配置是 `lio_avoid/lio/config/livox_360.yaml`** |
| **新增 H5：真值源自洽闸** | `accept.py` 比较 tap 与 FCU 自己估计的**空间跨度**（不需对时）：正常趟 8.11 vs 9.31 m（PASS），异常趟 **293.9 vs 8.6**、**114.5 vs 8.6（垂直 410.6 vs 3.6）**、**591.1 vs 9.8** 均 FAIL ⇒ 该趟标“不可评分”。**本轮 4 次 `ABORT: geofence` 里有 2 次能确定是假警报**（tap 报 410 m，FCU 只有 3.6 m） |
| **收尾两趟 `pva2`** | `pva2-01-02`（H5 PASS）**8 过 4 败**：W1 6/19、W1b 7/19、W3 PASS +0.118、**W4 −0.079 m**、W6 1.308；`pva2-02-02` H5 FAIL（不可评分）。PVA 三路确实进了飞控：FCU 的 `DA` 有 **71.8 %** 样本非零、峰值 11.7 m/s² |
| **supervisor 侧的同一道闸（最终版）** | `TapTruth` 记“隐含速度 > 30 m/s 的瞬移”；`geofence_verdict` 在两种情况下**抑制 geofence abort**：① 刚发生瞬移（2 s 内）；② 它与 **FCU 自己的位置**（`/fcu/odom`，需新鲜 <0.5 s）相差 > 2 m。实飞证据：`pva4-01-01` 的 supervisor 日志里连续打出 `geofence suppressed: the truth stream is 16.7 / 34.6 / 48.5 / 61.4 / 79.1 / 100.3 m from the FCU's own estimate` ——**那一趟因此没有被误杀**，而在此之前它会在第一个 16.7 m 就被 abort。离线五条测试全 PASS（`lio_avoid/src/tools/test-teleport-guard.py`） |
| **新的头号堵点：真值流自己在漂** | 最近 8 趟里 **6 趟 H5 FAIL**（tap 与 FCU 的空间跨度差 251–697 m）。这意味着这些趟的 W0b/W3/W4/W6 **不能用来说飞机好坏**——它们读的是一个正在漂移的坐标系。下一件事就是修真值流（tap 的“origin = EKF origin”与 EKF 重设原点的交互） |
| **第 86 轮：对比测试地图（无柱 / 矮柱）** | 用户："做一个 compare_map，[-7,7] 里没有柱子，飞一遍看看行不行"。做了两张、各飞一趟：`worlds/compare_open.yaml`（[-7,7]² 全空，56/250 柱）**失败**——`H5 FAIL`，**FCU 自己的估计跨度 25 972 m**，空场没有特征 ⇒ LIO/EKF 散掉，空场测的是估计器；`worlds/compare_low.yaml`（250 柱全留、截到 **z1=1.0 m**，低于 2.0 m 飞行高度）**9 过 3 败**：**W1/W1b 19/19（最差 0.28 m）**、W2 PASS（max 4.91）、高度偏差 **+0.019 m**、H5 PASS。同一套栈在冻结图上 MINCO 失败 **3759**、参考冻住 **76.5 %**，矮柱图上只有 **27** 次、**16.5 %** ⇒ **"一团糟"的主因是柱林对规划器的压力，不是控制回路**。剩下的 W3（垂直 6.03 m excursion）、W4（矮柱图下只剩"下降越过树桩"的度量伪影，>1.5 m 无接触）、W6（跟踪本底 1.9 m）见 `docs/INTEGRATION.md` §66 |
| **为什么现在超调明显、以前没有** | 不是飞机变差了，是**到达判据变了**（SPEC S1 第 81 轮：" 经过航点、不驻停"）。第 79 轮臂：`ARRIVE_M 0.15` + `SETTLE_SPEED 1.0` + `SETTLE_S 2.0` ⇒ 实质**停住再走**，实测到角点时速度 **p50 0.34 / max 0.49 m/s**、距角点 0.01–0.08 m；现在：`ARRIVE_M 0.50` + `SETTLE_SPEED 6.0` + PVA 出弯 2 m/s ⇒ 实测到角点时 **p50 2.12 / max 2.74 m/s**、出弯后弦线弓高 **p50 0.76 / max 1.54 m**。而**弓高就是 `R = v²/max_acc`**（5.0 m/s² 下：2.05→0.84、2.12→0.90、2.59→1.34、2.74→1.50 m，与实测 0.76/0.86/1.04/**1.54** 一一对应）。第 79 轮之所以看不出来，是宽世界的 **2.0 m 车道**把同一类外冲吸收了（那一趟真值最远冲出任务方框 **2.4 m** 仍 11/0 通过） |
| **`max_acc = 8`（第 86 轮用户指定）实测** | `traj_opt.boundary.max_acc` 5.0 → **8.0**（三份 md5 `581461ca…`，`spec_check` 新增一行 PASS）。同一张冻结图、只改这一项：过点速度 **2.30 → 2.77 m/s**、出弯弓高 **1.51 → 1.12 m（−26 %）**；两趟都是 **5 过 7 败**，**W4 −0.091 / −0.093 m** 不变。原因写在 §68.2：弓高 = v²/a，抬 `max_acc` 的同时 MINCO 也允许更快的过点速度，净收益被打折。按 `v²/8`，要弓高 ≤0.145–0.245 m 需过点速度 ≤**1.08–1.40 m/s**，而现在是 2.77 |
| 全流程链路图 | **`docs/CHAIN.md`**：14 段 topic 清单（内容/名义/实测频率）、三个 mask、SPEC+薄层+SUPER+FCU+判据五层的判定参数、名义与实测不一致的 7 项、一条真实时间线，以及本轮改过的文件清单（"有没有偷偷搞破坏"） |
| **A/B（同地图、同臂、同 `robot_r`，只改这一项）** | `pos`：3 / 11 / 3 / 2 个航点、参考冻结 76.5 %、四次 geofence abort｜**`posvel`：两趟都 W1 18/19 + W1b 19/19（真值最差 0.17 / 0.22 m）+ 8 过 3 败 + rtf 1.0002 / 1.0003**，一趟 `RUN_EXIT=0` |
| **降速有没有用（用户质疑的那一条）** | `max_vel` 5.5 → **2.0**（出弯本来就是 2.0），两趟 `soft`：参考确实被压到 **max 2.01 / 2.04 m/s**（之前 4.06–4.33），但 **W4 纹丝不动：−0.089 / −0.092 m**，航点反而 10–13/19 → **用户的直觉对：不是速度** |
| 为什么不是速度（闭环频响） | `scripts/loop_response.py`：同一套增益在**两趟里都**在 **0.2–0.5 Hz 放大 1.73 / 1.86 倍**；差别在参考主频：第 79 轮宽车道 **0.16 Hz**（|H|≈1，沿航迹增益 1.04）0.00 s 滞后）vs 冻结六边形 **0.30–0.67 Hz**（增益 1.73–1.98，0.10 s 滞后）。降速按 f=v/λ 只能把主频往下挪（1 m/s → 0.42 Hz，仍在共振带） |
| 温和增益的效果 | `params/guided_soft.parm`（PSC_NE_POS_P 3.0→1.5、VEL_P 3.0→2.0、ATC_ANG 6.0→4.5、WP_SPD 2.5）：**W6 跟踪 0.76–0.87 → 0.41**（砍一半），**W4 仍 −0.089**；两趟都 7 过 4 败 |
| **W4 的真正地板** | `scripts/plan_clearance.py`：**规划自己**的轨迹对柱面的净空 p50 **0.329**、p10 **0.234**、**min −0.078 m**（0.3 % 的样本在柱体内）；`scripts/plan_attribution.py` 说 19 条轨迹里 **9 条**都有（每条 1–6 个样本≈0.1 s）⇒ **MINCO 的样条在 SFC 约束点之间鼓出几厘米**，上游数值细节，与速度/增益/`robot_r` 无关 |
| W4=0 能不能做到 | **这张冻结地图里不能**：要求"规划净空 ≥ 跟踪误差"（≥0.4 m），而最紧三条腿的弯道止于 **b = 0.245 m**。四个选项见 `docs/INTEGRATION.md` §61.7（A 改判据、B inflation 2、C 重开地图、**D 修 SUPER 的 MINCO 约束采样**） |
| 剩下的三行是什么 | **W4 −0.089 / −0.098 m 接触**（弯 0.39 m + 净空 0.145–0.245 m，见上一行）；**W1 18/19**（薄层自己的到达记账少一个，真值侧 19/19）；**W6 0.76–0.87**（它比的是整段"记录下来的规划"与真值，参考在冻/重启） |
| 附带发现（安全） | `r10-02-02` 收尾时 **tap 与 FCU/LIO 差 338 m**、`az0-01-01` 410 m、`pva3` 697 m。**机制已定位（`scripts/tap_frame_forensics.py`）：tap 没说诞 —— 它的位置与速度逐样本自洽（比值 **1.0x**），是 **SITL 的物理模型被抛飞（19 m/s）而飞控 EKF（吃 LIO 外部导航）留在场内**；每一趟都带 `SIM Hit ground` / `EKF Failsafe` / `Potential Thrust Loss`。所以那些 `ABORT: geofence` 与那些 W 行都不是飞机的表现（新增的 **H5** 与 supervisor 的两道闸会把它们标成不可评分/抑制 abort）。正常趟里 tap 可信（`d85-01-02` tap vs LIO p50 **0.011 m**） |
| SPEC 自检（`pv1-02-02`） | S0 **PASS**｜S1 过 0.5 m 球 **PASS 18/18**（`spec_flight` 最差 0.203 m）｜S1 不停车 **FAIL**（参考在 18 个航点处为 0）｜S2 **PASS**｜S3 **FAIL**（1.83 m/s，>4 m/s 占 8.8 %）｜S4 **PASS**｜S5 **PASS**｜S6 **PASS**（两趟都报） |
| 图 | 这一轮**没有重画**（等三行结论定下来再出图）；上一轮的 `d85-*` 六张还在 `docs/evidence/` |

### 0.1 第 84 轮（历史：出弯速度降到 2 m/s，四趟 attempt 全败）

> ### ⚠ 两条硬约束（用户第 81 轮）：`docs/SPEC.md` 是唯一有效的规矩；**地图与航点已冻结**
>
> `scripts/check_frozen.py` 把世界文件的 SHA-256（`a5a43f8f…`）、250 根柱、六个航点坐标写进代码，
> `scripts/fly_hex2.sh` **每次起飞前跑它**，任何一条不过就拒绝起飞。第 84 轮没有改动地图或航点（8/8 PASS）。

| 项 | 结果 |
|---|---|
| 这一轮改了什么 | **只有一个数字**：SPEC S1 的航点 PVA 速度 **4.0 → 2.0 m/s**（用户："把出弯速度改2m/s"）。生效位置是**臂**（`scripts/fly_hex.sh` 的 `CORNER_SPEED`，默认 2.0），**不是**世界文件（那里 `mission.corner_exit_speed_mps: 4.0` 是冻结时的历史值）。SPEC S1 + 修改记录第 14 行已写；`spec_check.py` 的 S1 改成校验臂、把这行当 INFO 打出来 |
| 会话 `d85`（SPEC S6 两趟） | 4 次 attempt，**0 次过闸**：`d85-01-01` geofence abort（199 s，真值 (−10.51,−4.14)）｜**`d85-01-02` 唯一可评分**（rtf 0.9861，跑到 harness 超时）｜`d85-02-01/-02` 两次都在 **30 s** 时 geofence abort（真值 (+10.50,−2.58) / (+10.53,−2.63)，几乎同一位置，可复现） |
| 十一行（`d85-01-02`，可评分那趟） | **6 过 5 败**：W1 **8/19**（到的是 1,2,3,4,7,8,9,10）、W1b 13/19（最差 **3.33 m @w5**）、W2 p50 0.26 / p95 2.29（>4 m/s 仅 **0.70 s**）、W3 bias −0.098 PASS、**W4 −0.065 m 接触**（0.30 % 采样在柱体内）、W6 p50 0.148 / p90 1.055 / max 3.084 |
| 参考本身 | **74 % 的时间速度为 0**（50 段停车、间距 p50 1.69 s；mean 0.34 m/s、max 4.06 m/s、指令里程 111.3 m / 264.6 s）；真值 291.8 s 只飞 **163.7 m = 0.56 m/s** |
| 为什么更差 | `inflation_step 1` ⇒ 地图膨胀 **0.10 m < `robot_r` 0.15 m**。CIRI 因此变轻松（**1003 → 21** 次失败），但 A\* 顺着离柱面 0.178 m 的种子线抄、多面体贴着柱子长，MINCO 还要在里头转出 **2 m/s 的终点速度** ⇒ **MINCO 失败 476 → 3759 次**、PlanFromRest 3761 次 ⇒ 薄层发 HOLD ⇒ 参考冻住。**出弯速度不是瓶颈，走廊净空才是** |
| 第二趟为什么吓人 | 参考只上线 **12.5 s**（694 个 setpoint，规划包络最大 5.03 m）；据掉指令的那一瞬三路还互相认得（tap (5.10, −0.60) / FCU (4.25, 0.21) / LIO→W (4.51, −0.17)，互差 ≤ 1.2 m）。之后 tap 单独记录到一次很猛的位移：航向 2 s 转 105°、滚转贴住 **45°**、速度到 **11.19 m/s**（峰值 14.75）、到 30 m 外被 geofence 抓住——**但那一段没有 FCU/LIO 记录**（setpoints.csv 只覆盖 relay 开着的 13.9 s），而且这趟的 tap 自己 gravity 检查 FAIL、position span 报 84.5 m。所以能钉死的只有一句：**那段位移不是规划发的**；它是不是真的发生过，要读 `flight.BIN` 的 attitude/desired 才能定 |
| 估计器 | 仍然不是问题：LIO vs 真值水平 p50 **0.011 m**（第一趟）/ **0.004 m**（第二趟），`nav_hz` 46.9（配置 48） |
| 图（第 84 轮） | `d85-01-lap1-paths.png` `d85-01-lap2-paths.png`（分圈俯视图，两圈）｜`d85-01-vxyz-t.png`｜`d85-01-lio-err-t.png`｜`d85-02-vxyz-t.png`｜`d85-02-lio-err-t.png`。**上一轮的 `d84-*` / `r15b-*` 七张已按用户要求删除**（"旧图记得一定要删除"），存档在 `_evidence_archive_round83.tar.gz` |
| 结论 | 单改出弯速度解决不了问题：相对"角点停"那一趟，**冻住的时间 26.5 % → 74 %、真值均速 1.81 → 0.56 m/s**。要往前走只有两条路，都要用户点头：**A** `inflation_step` 回 **2**（余量 +0.05 m；§58 实测 MINCO 576 次、CIRI 96 次）或 **B** `robot_r` 降到 **0.10 m**（与 10 cm 膨胀对齐） |
| 仍然互斥 | S3（均速 > 4 m/s）与 W4（零接触）在冻结地图里仍然不可能同时成立：最紧三条腿 **b = 0.245 m**，实测横向 \|p90\| 0.31 m（详见 §57.9、§58） |

### 0.2 第 83 轮（历史：robot_r 0.20 → 0.15 那一批，出弯速度还是 4 m/s）

> （硬约束同上，不再重复）

| 项 | 结果 |
|---|---|
| 仿真几何（SPEC S2：下方 0.2 m / 上圆 0.2 m） | **改好了并验证**：方柱对角 (5−r)√2 = 6.929646 m、圆柱 6.971068 m、差 0.041421 = r(√2−1)；20 000 条射线全部命中点都在表面上；`cast()` vs 标量参考一致；`--selftest` / `--prune-check` 全 PASS |
| 仿真性能 | **从 234–393 ms 降到 52–60 ms/帧**（float32 + **按方位角窗口**只算候选射线；两次把 culled cast 与标量参考比对的压力测试抓出自己的两个 bug 并修掉）；修掉"`best_t=inf` 的射线从不参与求交"的真 bug；`run-mission.sh` 把 BLAS 线程钉成 1（默认线程池实测 1422 ms/帧） |
| 仿真实际速率 | 整机一起跑 **8.5–9.4 Hz**（`c10-01-01` 9.07 Hz；`r15a-01-01` **9.35 Hz**、cast 62 ms、每帧 20 000 点）——**仍不是 10 Hz**，日志里照实写 |
| 航线来源 | 新增 `--tour world`：航线从**世界文件的 `mission.corners`** 读，地图冻结 = 航点冻结 |
| PVA 接通 | `goal_pva_en: true` + `/planning/click_goal_pva`；不开这个开关 SUPER 会把终点速度强制清零 |
| 验收口径 | `accept.py` 的 W1b 目标环改成**从世界文件读**（六边形不再被四个方角评）；`WBOX=0.5` 对齐 SPEC S1 的球 |
| **能不能飞**（`robot_r = 0.20` 时期的阶梯） | **4 m/s：4 趟全部 0 个航点**（MINCO 不可行）｜**3 m/s：只到 leg2 超时**｜**2 m/s：卡在 leg4**｜**1 m/s：3 个航点后 CIRI 失败循环**｜**0（角点停）：19/19 全到**。**robot_r 0.15 之后**：4 m/s 变成 3156 次 MINCO 失败、2 m/s 变成 576 次 + 跑飞（见下面的第 83 轮表） |
| **卡在哪** | `robot_r 0.20`：卡**走廊**（`[CIRI] maxVolInsEllipsoid failed`，3814 条种子线 98.6 % 余量为负）。**`robot_r 0.15`（第 83 轮）之后走廊失败 = 0**，改卡 **MINCO**（4 m/s 终点速度在 ±0.1 m 走廊里转不过来）与**跟踪**（飞机偏出规划 6 m） |
| 附带发现（安全） | 规划停住后薄层发 hold，但**在 GUIDED 里拿着停住的 setpoint 飞机仍会漂出去**（`c10`：真值到 (−12.3, −10.5)，被 10.5 m geofence 抓住）。**规划停顿本身就是危险状态**，别当控制问题查 |
| 诊断趟 `d84-01-01`（0 出弯，**不是验收**） | 9 过 2 败：W1b **19/19**（最差 0.18 m）、W2 15.47 s >4 m/s、W3 PASS；**W4 −0.094 m 接触**、W6 零滞后 p50 1.17 m / 时基 spread 0.42 s |
| 为什么接触 | 冻结地图最紧三条腿的净空 **b = 0.245 m**（= 中心线两侧各 ±0.12 m），而实测**横向**误差 \|p50\| **0.074–0.090 m**、\|p90\| **0.306–0.317 m**（**没有系统偏置**：p50 +0.01）⇒ **约 37 % 的时间超出 ±0.12 m**；真正大的是**纵向滞后** p50 0.06–0.23 m，它不直接威胁柱子。`contact_where.py` 定位到的接触点：参考已贴到 0.10 m、真值又偏 **0.479 m**（那一趟有 1760 次规划失败，参考在停），离地时间 **2.4 % 在柱体内 / 31 % 小于 0.245 m** |

**图（第 83 轮那一批的标题与结论；文件已删，第 84 轮的 `d85-*` 在本节开头）**

* `d84-lap1/2/3-paths.png` —— **分圈俯视图**：红 = SUPER 规划的路径、蓝 = 实飞轨迹；标题给本圈的规划/真值最小柱面净空与 |规划−真值| p50/p90（圈边界实测）；
* `d84-vxyz-t.png` —— **v_xy / v_z / x,y / z 对时间**；
* `d84-lio-err-t.png` —— **估计器误差对时间**：水平 p50 0.018 / p90 0.044 m、垂直 p50 0.014 m ⇒ **估计器不是瓶颈**。

**这七张已按用户第 84 轮的话（"旧图记得一定要删除！"）删掉**，打包在
`D:\distill_self\_evidence_archive_round83.tar.gz`（3.19 MB）；`scripts/check_evidence_links.sh` 会断言它们**不在**。

**第 83 轮：`robot_r` 已按用户指示改成 0.15 m**（`scripts/set_robot_r.sh 0.15`，装到两处 config、md5 一致；
`docs/SPEC.md` S4 与修改记录已更新；地图与航点**未动**，`check_frozen.py` 8/8 PASS）。

实测（两趟，`r15a` 出弯 4 m/s、`r15b` 出弯 2 m/s）：

| 阶段 | 4 m/s | 2 m/s |
|---|---|---|
| **CIRI 走廊失败**（原来 1784–3814 次） | **0** | **96** |
| MINCO 失败 | 3156 | 576 |
| PlanFromRest 失败 | 3155 | 651 |
| 结果 | leg2/leg3 TIMEOUT | 到 27.8 s 时 geofence abort（真值 +10.52, −2.10） |

**结论**：`robot_r 0.15` 把走廊那一步彻底解决（§57.11 预测的 98.6 % → 0.5 % 得到印证）；
现在卡在 **MINCO（4 m/s 的终点速度在 ±0.1 m 走廊里转不过来）** 与 **跟踪（飞机偏出规划 6 m 跑飞）**。

**跑飞不是估计器**：同一趟的 LIO vs 真值水平 p50 **0.005 m** / p90 0.016 m，外部导航馈送 47 Hz
（配置 48），规划的位置包络一直在场内（x ≤ 4.22）—— 是飞机自己飞出去的，`EKF variance` 的时间戳在
geofence abort **之后**。

**下一步该调的是速度包线（臂参数，不动 SPEC 的数字）**：`REF_MAX_MPS`（现 6.0）、FCU `WP_SPD`（6.5）、
`ATC_ANGLE_MAX`（45°）、出弯 2 m/s。第 79 轮能稳定飞的是 4.5/2.0 m/s 级别的包线，6 m/s 从来没稳过。
⚠ 但 S3（全程均速 >4 m/s）与"更温和的包线"直接冲突 —— 这一点要用户知道。
**第 84 轮就是按这条做的（出弯 2 m/s）：没有救回来** —— 参考冻住的时间 26.5 % → 74 %，四趟 attempt 零通过（见本节开头与 `docs/INTEGRATION.md` §59）。

> **下面第 79 轮的内容保留为历史**（它用的是 2.0 m 车道的变体，已按新规矩作废，不再作为结论）。

> **v2 柱林（第 77–79 轮，见 `docs/INTEGRATION.md` §54）**：0.8 m 格 × 20×20、柱截面 0.2 m、
> ~~安全半径 0.2 m~~（**该项已被用户删除，见 §56.1；下面这段历史里所有涉及它的数字都不要再引用**），
> **柱心在每个格子内随机**（偏离格心中位 0.37 m，**没有一根**在格心）。两张地图，一张是规格、一张是飞行合格版：
>
> | 地图 | 柱子 | 任务四边车道 | 飞行结果 |
> |---|---|---|---|
> | **`worlds/forest_v2_08.yaml`（规格版，用户指定，未再改动）** | 391 | 0.96 m | 13/13、W4 在 −0.08…+0.23 之间摆（**不可复现**）、W6 0.62–0.80 → 10/11 |
> | **`worlds/forest_v2_wide.yaml`（飞行合格版，新文件）** | 233 | **2.0 m**（移空紧贴每条腿的一排格子） | **11/0 GATE PASS × 7 趟**（共 9 趟） |
>
> **默认臂就是通过的那一套**（`bash scripts/fly_v2.sh <tag>`，所有值都可用环境变量覆盖）：
> `PASS_BY_M=0.5 PASS_BY_LONG_M=1.0 HOLD_S=0`（角点**滚过去**、不驻停）+ `ARRIVE_M=0.15 SETTLE_SPEED=1.0
> SETTLE_S=2.0`（真到点）+ `ENTRY_MAX_MPS=2.0`（发射段限速）+ `params/guided_limits_fast.parm`
> （45° / 10 m/s² / WP_SPD 6.5 / P 3.0 / VEL 3.0 / JERK 150）+ `super_planner_hex.yaml`
> （`hot_start_en=true`、`backup_traj_en=false`、`planning_horizon=16`、`max_vel/max_acc 5.5/5.0`、A* 网格 480）。
>
> **7 趟 11/0**（W4 最小柱面）：`v3p-01` +0.600、`v3w-01` +0.544、`v3t-01` +0.541、`v4a-01` +0.535、
> `v3r-01` +0.486、`v3v-01` +0.475、`v3q-01` +0.428；**2 趟失败**都是 W6 = 0.61 / 0.65（判据 0.60），
> 其余十行全过。
> 通过那几趟的真值均速 1.5–1.6 m/s、真值峰值 4.28–4.73 m/s、>4 m/s 持续 2.2–4.8 s、13/13 航点且
> 真值几何复核最差 0.05–0.09 m。
>
> **为什么规格版过不了**（§54.7 有 22 趟的参数搜索表）：**90° 转弯半径 v²/a = 1.1 m 是 0.48 m 车道半宽的
> 2.3 倍**——滚过角点就出车道进柱排（擦碰 −0.07…−0.09，可复现），停在角点就 W6 超标（角点段 p90 0.95 m）。
> 宽车道（2.0 m）把这两件事同时解决：转弯装得下、跟踪余量够。
>
> 图：第 83 轮已按用户要求删除（看 `docs/evidence/README.md` 与 `_evidence_archive_round83.tar.gz`）。
>
> 下面 `## 1` 之后描述的是**早期架构**（`sitl_odom_relay.py` + `waypoint_mission` +
> `scripts/run_and_score.sh`、12 个航点**盒**）。现在跑的是 **lio_avoid harness + 薄层**
> （`src/whtf/ros/planner_source_super.py`）、柱林世界、`scripts/accept.py` 的十一行判定。
> 旧文保留（是集成过程的历史，其它文档引用它），但**以本节为准**；
> 完整记录见 `docs/INTEGRATION.md` §22–§54 与 `docs/PIPELINE.md`。
>
> **文档（第 79 轮已清理）**：`docs/` 只保留三个文档 —— `INTEGRATION.md`（完整记录 §0–§54）、
> `PIPELINE.md`（链路与状态机）、`MISSION.md`（任务与场地）；图在 `docs/evidence/`（索引见其
> `README.md`，第 79 轮只留 6 张图 + 3 张数据表）。旧文档（`STATUS.md`、`RECON.md`、`HANDOFF-ego.md`、
> `SPEED*.md` 共 18 个）与 v1 世界的旧图已打包到仓库外
> `D:\distill_self\_docs_archive_round61.tar.gz`（5.2 MB）与 `_evidence_archive_round79.tar.gz`。

| 指标 | 目标 | 实测（v2 柱林，`v4a-01`） | |
|---|---|---|---|
| 13 个目标（竖直起飞腿 + 12 角点 × 3 圈） | 13/13 | **13/13**，真值几何独立复核 `W1b` 最差 **0.06 m**（盒 1.5 m） | ✅ |
| 逐柱零接触（W4 最小柱面间隙，方盒口径） | > 0 | **+0.535 m**（v3p +0.600、v3w +0.544、v3t +0.541），0.00 % 采样进入柱体 | ✅ |
| GATE PASS（`accept.py` 十一行） | 11 过 0 败 | **11/0**：`v4a-01`、`v3w-01`、`v3v-01`、`v3t-01`、`v3p-01`、`v3q-01`、`v3r-01`（**7/9 趟**） | ✅ |
| 巡航速度 ≥4 m/s 且持续 ≥1 s（W2） | ≥1.0 s | **3.18 s**（真值峰值 5.06 m/s） | ✅ |
| 高度（W3 bias / p95 偏移） | ±0.30 / ±0.80 m | **−0.084 / +0.39 m** | ✅ |
| 规划-真值跟踪（W6，去时间基后 p90） | ≤ 0.60 m | **0.48 / 0.52 / 0.48 / 0.59**（四段） | ✅ |
| 全程平均地速 | ≈5 m/s | 1.49–1.56 m/s（角点要滚得慢，见 §54.7） | ❌ |

| v1 世界的历史对照（`sc-01` / `se-01` / `sj-02`，84 柱） | | |
|---|---|---|
| 13 个目标 | 13/13 | **13/13**（W1b 最差 0.27–0.50 m） |
| W4 最小柱面间隙 | > 0 | **+0.192 / +0.171 / +0.108** |
| GATE | 11/0 | **11/0** |

### 角点不停车（第 65 轮，`scripts/fly_pregoal.sh`）

SUPER 每条轨迹的终点速度恒为 0，而 pacer 只在到达后才发下一个目标 → 每个角点必停。
**离角点还剩 2–4 m 时就把下一个角点的目标发出去**（薄层 `--pre-goal-m`），当前角点于是成为轨迹的
**内部点**，不再停车；A\* 只有这么点余量，抄不出大弯（`--goal-lead 1` 在 15 m 外换目标，抄掉 8–9 m、W1b 7/13）。

| 臂 | 平均地速 | 参考在角点归零 | W4 净空 | W2 | 判据 |
|---|---|---|---|---|---|
| `sc-01`（每角停，规划 4.2/3.6） | 2.13 | **10 次 / 2.3 s** | +0.192 | 1.05 s | 11/0 |
| `pgb-01`（pre-goal 2 m，4.2/3.6） | 2.33 | 4 次 / 0.2 s | +0.215 | 0.46 s | 10/1（W2） |
| `pge-01`（pre-goal 4 m，5.2/4.8） | 2.47 | —— | +0.159 | ✅ | 11/0 |
| **`pgf-01`（pre-goal 4 m，5.6/5.4）** | **2.62**（去停机坪悬停 **2.75**） | **0 次 / 0.0 s** | **+0.275** | ✅ | **11/0 PASS** |
| `pgg-01`（6.2/6.0） | 2.68 | —— | +0.153 | ✅ | 10/1（**W6**） |
| `pgh-01`（6.2/6.0 + 横向增益 3.0） | 2.60 | —— | +0.135 | ✅ | 11/0 |

**角点 12 次停车 → 0 次**，均值 **2.13 → 2.62（+23%）**，`pgf-01` 的净空是 **+0.275**，W1b 仍 13/13（最差 0.44 m）。
规划器抬到 6.2/6.0 时 W6 破，所以上限取 **5.6/5.4**。

> **⚠ 净空还不可复现**（第 67–68 轮）：同一套配置的 `pgl-01` 是 **−0.058 m 接触**（跟踪向内偏 0.262 m，
> 而参考当时只剩 +0.204 m）、`pgq-01` 与 `pgo-01` 参数完全相同却从 +0.246 掉到 **−0.001**。
> **20 趟全族统计：7 趟 GATE PASS、4 趟毫米级接触（−0.001…−0.107）、其余死在 W6**；W1b 全部 13/13。
> 站得住的结论只有 **W4 ↔ W6 = −0.55…−0.62**（净空与跟踪同步变差，同一个“跟踪质量”驱动）；
> 第 68 轮曾把方差归因到 LIO 的 z 跨度，第 69 轮复查发现**那条相关性主要来自一个离群点**（去掉后 +0.65→+0.26），已更正。
>
> 所以能稳定作为验收结论的仍是**停-走档**（`sc-01/se-01/sj-02`，3/3 全过，2.08–2.19 m/s，+0.108…+0.192）；
> 角点不停族的最好纪录是 `pgf-01` **2.62 m/s / +0.275 m**（11/0 PASS）与 `ptc-01` **+0.281 m**。详见 §46–§50。
>
> "现实上界"按实测分解约 **3.26 m/s**（角点区占 29 % 路程、2.1 m/s；直线 4.2）；参考**路径本身**的
> 时间最优是 **3.53 m/s**，规划器只给 2.89。第 69–71 轮把"按倍速重放 SUPER 自己的多项式"这条路
> 查到底：多项式已能 100 Hz 新鲜发布（`scripts/patch_poly_at_cmd_rate.py`），但**多项式 τ=0 的点是
> 轨迹起点（飞机当时位姿），与"当前指令点"差 0.43 m**（沿航迹滞后），两者无法互相锁定。
> 要走完这 22 %，需要让 planner 内部把"求值用的 τ"暴露出来——更深一档的改动，已记录在 §50.3。

图：那几张（`pgf-01-*`、`sc-01-*`）第 83 轮已按要求删除，归档在 `_evidence_archive_round83.tar.gz`；重画用 `scripts/plot_lap_paths.py`、`scripts/plot_vz_t.py`。

### 为什么均值到不了 5 m/s（四条上界，离线 + 实测都有）

| 上界 | 数值 | 依据 |
|---|---|---|
| 几何 | 5.82 m/s | 最大合法圆角 R 2.86 m、直线无柱的理想值 —— `scripts/corner_space.py` |
| rest2rest | 3.65 m/s | 12 条 15 m 的腿、每条都以静止结束（解析） |
| **跟踪（第 58 轮新增）** | **2.5–3.5 m/s** | 紧处偏差 p90 从 2.2 m/s 的 0.34 m 涨到 3.2 m/s 的 0.48 m；在 5 m/s 处外推吃掉 **0.75 m**，比整个世界自由半通道（0.665 m）还宽 —— `scripts/deviation_census_space.py`，r 0.75–0.81 |
| **零接触实测** | **2.46 m/s** | 46 次有效评分飞行的前沿：`frontier.png`（已归档；重画 `scripts/speed_frontier.py`） |

最快**有效**飞行 3.20 m/s（`rp-01`，但净空 −0.099 m，必碰）。
另外，**"撤掉那 12 次角点停顿"这条最后的路也已被实测封死**（`missionA-sf-01`，第 53 轮）：
在上限 4.2/3.6（跟踪有余量、W6 通过）上改成动中交接，角点处的**向内偏差从 0.13 m 跳到 0.468 m**
（`closest_command.py`：指令净空 +0.424 m、真值 −0.044 m，而规划自己 0 % 的样本贴近边线），
于是接触；而且它**并没有变快**（角点停留仍 36.4 %、均值仍 ~2.1）。结论：**角点上的停不是浪费，
它就是净空本身** —— 停的时候飞机落在世界审计过的角点圆盘里，不停的时候走的是自己切的圆弧。
要越过 2.46 只有三条路，且都已用证据关掉：加宽柱距（用户排除）、加大规划净空（会把 1.55 m 柱间
通道挤到 0.02 m）、换参考接口（给飞控完整轨迹而非点，未实现）。详见 `docs/INTEGRATION.md` §28.6。

### 净空到底是什么（第 58 轮：46 趟逐点分解）

净空是一条减法：**参考在那一处给的余量 − 跟踪在最坏那一点吃掉的量**。

| 量 | 数值 |
|---|---|
| 紧处（真值净空 <0.6 m）平均向内偏差 | **+0.088 m**（p10 +0.038）—— 很小，从来不是机制 |
| **单趟最深偏差**（在 W4 自己那个最紧点） | 中位 **+0.299**，p90 **+0.534**，最大 **+0.632** m |
| 参考在那一处的余量（中位） | **+0.451 m** |
| 验收档 | **0.45 − (0.28…0.36) = +0.11…+0.19 m**，与实测 W4 +0.108/+0.171/+0.192 逐位吻合 |

把 15 次接触按"谁的错"分类：**7 次是发出去的参考路径本身钻进了柱子**（最小间隙到 −0.110 m），
**8 次是参考干净、飞机自己偏出 0.28–0.63 m**，**0 次落在两者之外** —— 判据完备。
参考路径本身的体检结果必须记下来：**46 趟里 12 趟参考穿进柱面、35 趟低于规划自己的 0.325 m 边距**；
而**验收档的参考是完好的**（sc-01 +0.315、se-01 +0.347、sh-01 +0.346），说明这条链没有系统性配准误差。
图：`deviation.png`（已归档；重画 `scripts/deviation_figure.py`）；数据：`docs/evidence/deviation_space.tsv`。

### 2.46 与 GATE PASS 之间只差一行，而那一行是**时间**（第 59 轮）

零接触最高均值 `pm-01`（**2.46 m/s**，净空 +0.117）**只死在 W6 一行**（W4/W2/W1b/W3 全过）。
把 W6 的残差投影到规划路径的切向/法向（`scripts/w6_decompose.py`）：

| run | p90 \|r\| | 沿航迹 rms（时间） | 横向 rms（路径） |
|---|---|---|---|
| sc-01 验收档 | 0.497 | 0.250 | 0.192 |
| pi-01（PASS 最快） | 0.550 | 0.277 | 0.178 |
| **pm-01（零接触最快）** | **0.647** | **0.340** | **0.216** |

**沿航迹比横向大 1.3–1.6 倍**：W6 追的是"早一点/晚一点"，而不是"飞偏了"。
从 2.0 到 2.5 m/s，横向只涨 12 %（0.192→0.216），沿航迹涨 **36 %**（0.250→0.340）。
所以 W2（要峰值）与 W6（怕峰值）共用一个旋钮，`(4.2, 3.6)` 周围只有 1/6 通过是**斜率**不是噪声。
要把 2.46 变成 PASS 需把时间抖动降 ~20 % —— 那是飞控环/接口的事，规划改不了。

### 这个通过率可不可信？（第 56 轮的排除性检验，第 57 轮按真值重算）

W6 的残差是"运输延迟 × 速度"，所以"通过率"必须排除主机负载这个混淆项。
`scripts/w6_vs_host.py` 对 48 次有效评分计算（第 57 轮起有效性用**真值** `W1b` 判）：W6 与
**rtf 的相关系数 +0.038**、与 LIO 交付频率 +0.104、与点云频率 +0.074；通过组 rtf 均值 **0.9977**、
失败组 **0.9982**（失败组反而略高）。
**结论：W6 的通过/失败不是主机造成的**，是这条链（10 Hz 点云 → LIO → 50 Hz 桥 → GUIDED）在
2.2 m/s 附近的固有抖动。

### 验收工具本身被查过吗？（第 57 轮）

`W1`（13/13）原来是**规划盒自报**的。第 57 轮用真值独立复核，发现它在 `--goal-lookahead-m > 0`
时会给**错误的通过**（抄近路的轨迹从角点旁 3–5 m 穿过那条"越线"平面，横向偏差却很小）：
54 次评分里 **7 次不一致**，最差的 `sk-01` 自报 13/13、真值只有 **4/13（最差 5.20 m）**。
于是 `accept.py` 增加**只读真值**的独立行 `W1b`，闸门由十行变十一行。

全库重评（`scripts/rescore_all.py`，结果在 `docs/evidence/rescore_w1b.tsv`）：**55 趟 → 8 趟
GATE PASS**（pg-01, ph-01, pi-01, pw-01, sc-01, se-01, sh-01, sj-02），验收三趟仍 **11 过 0 败**，
`sk-01` 被抓住（`W1b` 4/13）。同一个"自报当证据"的错还出现在**前沿图的有效性过滤**里：
换真值后有效条数 48 → **46**（剔除 4 条抄近路，又补回 2 条被 pacer **少报**的），
前沿的三个关键数字不变。逐点距离还显示**同一圈内按周期 4 重复**
（sc-01：0.41 / 0.34 / 0.20 / 0.10 复现三次），即三圈是可重复的。
`scripts/verify_waypoints.py`（单趟复核）、`scripts/w1_audit.py`（全库审计）、`scripts/rescore_all.py`（全库重评）。

### 复现（都是单条命令）

```bash
bash scripts/fly_robust.sh <tag>     # 推荐档：计划上限 4.2/3.6 —— 该点实测 3/3 通过
bash scripts/run_robust_at.sh 4.2 4.5 <tag>   # 想 W2 宽裕就换这档，但 W6 会贴近判据线
bash scripts/fly_pi01_arm.sh <tag>   # 更快档：5.0/4.5，W2 余量更大但 W6 是掷硬币（同族 7 次过 3 次）
bash scripts/run_accept.sh <tag>     # 十一行判定（accept.py，含真值行 W1b）
python3 scripts/verify_waypoints.py /home/fanxin/lio_avoid/logs/missionA-<tag>   # 只看航点几何
python3 scripts/rescore_all.py       # 全库按十一行重评（结果 docs/evidence/rescore_w1b.tsv）
python3 scripts/deviation_census_space.py   # 净空分解：参考给多少 / 跟踪吃掉多少（§37）
python3 scripts/speed_frontier.py    # 重算全表（46 次有效飞行）+ 重画 `frontier.png`（已归档；重画 `scripts/speed_frontier.py`）
python3 scripts/check_param_names.py params/*.parm   # 灌参数前必做：名字对不上固件会静默失效
```

### 图

仓库里**只保留最新一批**（`docs/evidence/`，9 个文件 / 1.4 MB）：

| 文件 | 内容 |
|---|---|
| **`pgf-01-lap1/2/3-paths.png`** | **每圈一张俯视图**（角点不停那一趟，11/0 PASS）：**红 = SUPER 规划路径，蓝 = 实飞轨迹**，84 柱、本圈 4 个角点、本圈最近柱面与 \|规划 − 真值\| 都印在标题上 |
| **`pgf-01-vxy-z-t.png`** | **v_xy 与 z 对时间**：规划（红虚）vs 真值（蓝/绿实）、>4 m/s 阴影（3.87 s）、13 个到达竖线、三圈底色 |
| **`sc-01-vxy-z-t.png`** | 对照（停-走档）：角点速度为 **0**、>4 m/s 只有 1.05 s |

`deviation_space.tsv`、`deviation_census.tsv`、`rescore_w1b.tsv` 是三张图/表背后的逐趟数据（索引见 `docs/evidence/README.md`）。

三张分析图（速度前沿、净空分解、状态机时序）**已归档**，一条命令可重画：

```bash
python3 scripts/speed_frontier.py         # frontier.png（46 次有效飞行 + 两条离线上界）
python3 scripts/deviation_figure.py       # deviation.png（净空给多少 / 跟踪吃掉多少）
python3 scripts/plot_pipeline_states.py   # pipeline-states.png（三个状态机时序）
python3 scripts/plot_lap_paths.py <tag>   # 分圈俯视图（红 = 规划，蓝 = 实飞）
python3 scripts/plot_vz_t.py <tag>        # v_xy 与 z 对时间
```

归档包：`D:\distill_self\_evidence_archive_round72.tar.gz`（6.2 MB，逐趟旧图）、
`D:\distill_self\_evidence_archive_round73.tar.gz`（三张分析图）。

---

## 1. 整条链（真跑起来的就是这条）

```
   ┌────────────── 规划（SUPER，原生）──────────────┐
   │                                                 │
/cloud_registered ──►  ROG-Map  ──►  A*  ──► CIRI 走廊  ──►  GCOPTER(MINCO)
   ▲  (adapter 造的世界点云)                        │
   │                                                ▼
/lidar_slam/odom ─────────────────────────►  fsm_node ──► /planning/pos_cmd (mars PositionCommand, 100 Hz)
   ▲  (relay 扮演 LIO)                              ▲
   │                                          waypoint_mission
   │                                          12 个航点、switch_dis=1.0
   │
   │        ┌─ pc_adapter ──► /plan/cmd (quadrotor_msgs PositionCommand, 50 Hz, frame=lio_world)
   │        │        │
   │        │        ▼
   │        │   lio_avoid/bridge ──MAVLink 331(外部导航) + 84(定点指令)──► ArduPilot GUIDED
   │        │        ▲                                                        │
   │        └────────┴── /lio/robo/odom, /lio/odom ◄── relay ◄── LOCAL_POSITION_NED
   │                                                                         │
   └───────────────────────── /plan/trigger（GO 闸门）◄── relay ◄────────────┘
```

两条坐标系之间**没有旋转**：relay 在 `/lio/robo/odom`、`/lio/odom`、`/lidar_slam/odom`
上发的是同一个 ENU 世界，SUPER 也在同一个世界里规划，所以 bridge 的 L→W 标定必须是
**单位矩阵**（`config/calib_identity.json`）。bridge 默认会给 +90°，那是 lio_avoid 那台的
物理事实，用在这里会让轨迹镜像 90°（15 m 正方形映到自己身上，验收会变成"看运气"）。

---

## 2. 一条命令跑通（WSL2 + ROS 2 Humble）

```bash
bash scripts/run_and_score.sh 165        # ≈4 分钟：起 SITL → 解锁起飞 → SUPER 规划 → 飞 3 圈 → 打分
```

它会依次做完：装配置 → 合并 SITL 参数 → 起 SITL → relay（写参数/切 GUIDED/解锁/起飞/发 GO）
→ 等 `/tmp/takeoff_ok` → 起 adapter → `fsm_node` + `waypoint_mission` → bridge → 计时 → 打分。

跑完输出（真实数字，原始轨迹 `sitl_trace.csv` 已归档，见 `docs/evidence/README.md`）：

```
== SITL closed loop ==
  samples             : 8250（空中 6245，125.0 s，50 Hz）
  z range             : -0.01 .. 3.19 m
  box hits (<=0.25 m) : 12 / 12
  min dist per box    : 0.01 0.01 0.01 ...（12 个都是 1 cm）
  speed |v| EKF       : 中位 3.08  p90 3.73  峰值 4.18 m/s
  speed 位置 0.5s 窗  : p90 3.37 m/s
  参考速度 p90        : 4.89 m/s   参考峰值 5.90
  跟踪误差 水平 rms/p95: 2.12 / 4.21 m（角点等待参考时的大值主导）
  min trunk clearance : +0.214 m
  VERDICT: FAIL        ← 只因速度 p90 3.73 < 4.0；命中与净空都过
```

离线复算（独立于 relay 的打印，见 `docs/evidence/README.md`）：

```
  盒命中 (<=0.25 m)   : 12 / 12
  过点顺序            : 0 1 2 3 0 1 2 3 0 1 2 3
  单圈用时            : 29.2 28.8 s
  树干最小净空        : 0.214 m
```

**离线复算**（不依赖 ROS/SITL，只吃 csv；判据和 relay 一致）：

```bash
python3 scripts/score_trace.py --profile      # 命中 / 速度 / 净空 / 圈速 / 过点顺序
```

**灌参数前必做**（本工程被这件事坑过两次）：

```bash
python3 scripts/check_param_names.py params/*.parm    # 名字对不上固件 → 静默失效
```

---

## 3. 为什么是这些参数（每一条都有现场证据，详见 `docs/INTEGRATION.md`）

| 位置 | 值 | 不这么设会怎样 |
|---|---|---|
| `config/super_forest_15m.yaml` `rog_map.fix_map_origin` | `[0,0,2.0]` | 它是**地图中心**（`rog_map.cpp:45`），不是最小角。写成 `[-20,-20,-2]` 会让地图顶面正好落在飞行高度 2.0，CIRI 的包围盒被裁、种子线跑到盒外 → `SearchPolytopeOnPath failed` → 一条轨迹都不发 |
| 同上 `virtual_ground_height` | `-0.1` | 地图中心放对之后飞行高度在地图中间，地面可以恢复真实值；`-5.0` 是旧 bug 的临时绕法 |
| 同上 `traj_opt.boundary.max_acc` | `6.5` | 上限由倾角决定：`g·tan(ATC_ANGLE_MAX)`。原来 15.0 → 参考从静止"弹射"，飞机跟不动（参考 p90 5.9 vs 真实 3.9）。也不能乱加大——配 `max_vel 8` 会把优化器逼到 `Omg or thr or Pos violation`，规划直接卡死 |
| 同上 `traj_opt.boundary.max_vel` | `7.0` | 一条边只有 15 m，两端还要加减速，参考峰值约为 max_vel 的 0.85 |
| 同上 `astar.map_voxel_num` | `[300,300,60]` | ×`resolution 0.05` = 15×15 m、以当前点为原点 → 向前只能看 7.5 m，一条轨迹被切成 ~8 m 一段。放大到 480/640 会让 GCOPTER 在更长走廊上不可行（见 STATUS 23.1），所以只能停在这里 —— 这也是速度上不去的根 |
| `config/waypoint_forest_15m.yaml` `publish_dt` | `99999`（只发一次） | 10 Hz 重发 goal 会让 FSM 反复走 `GENERATE_TRAJ → PlanFromRest`，而它生成的是 **rest2rest** 轨迹（两端速度都为 0） |
| `params/apm_guided_fast.parm` `ATC_ANGLE_MAX` | `45` | 水平加速度能力 = `g·tan(45°) = 9.8 m/s²`。再往上（55°）只把峰值抬到 4.61，却把净空从 +0.238 压到 +0.120 |
| 同上 `PSC_NE_POS_P / PSC_NE_VEL_P` | `3.0 / 4.0` | 位置-only setpoint 的稳态滞后 ∝ v/`PSC_NE_POS_P`；但 1.4→5.0 试下来是"峰值↑、p90 与净空↓" |
| 同上 `WP_ACC / WP_JERK` | `2.5 / 1.0`（固件默认） | GUIDED 的运动学上限来自 `WP_*`（`mode_guided.cpp:255`），但放开到 6.5/20 实测更差（6/12，见 STATUS 23.2） |
| `params/apm_sitl.parm` `GUID_TIMEOUT` | `0` | 默认 3 s：收不到定点指令就退出 GUIDED，而 bridge 只在 GUIDED 时才发 → 死锁 |
| 同上 `ARMING_SKIPCHK` | `2097150` | 位掩码 bit1..bit20 全置位才是"跳过全部预解锁检查"（`get_enabled_checks()==0`）；写 1023 只跳过前 9 项 |
| 同上 `EK3_SRC1_POSXY` | `3`（GPS） | SITL 里没有 LIO。外部导航只能从 FCU 自己的位置反推，是自环；真机那份在 `params/apm_extnav.parm`（=6） |
| `bridge --calib-file` | `config/calib_identity.json` | 缺省是 +90° yaw（见上） |
| `bridge --fields` | `pos` | `posvel` 会让飞机跑到 ±7.5 m 方形外面（实测飞到 (12.5,-8.1)） |

---

## 4. 换成真机 / 接真 LIO

1. **位置源**：用 `params/apm_extnav.parm`（`EK3_SRC1_POSXY=6, VELXY=6, YAW=6, POSZ=1`，
   `ARMING_SKIPCHK 0`），并让 LIO 通过 MAVLink 视觉消息进 EKF。当前 bridge 已经在发
   外部导航（331，来自 `/lio/robo/odom`），所以 LIO 接通后**不需要改 bridge**。
2. **把 relay 换掉**：真机上不需要 `sitl_odom_relay.py` 的模拟位姿与解锁逻辑；
   `/lidar_slam/odom` 由 Super-LIO 直接提供，`/plan/trigger` 由 supervisor 在
   "已起飞并悬停"后发（这就是 lio_avoid 原本的分工）。
3. **参数**：`ATC_ANGLE_MAX 45`、`PSC_NE_POS_P 3.0` 这类是按 SITL 调的，真机必须按
   质量/桨/推重比在台架 + 低速试飞里复调；`FS_*`、`FENCE_*` 在真机必须真实生效
   （`apm_sitl.parm` 里把它们关掉了，那份只用于 SITL）。灌之前先跑
   `scripts/check_param_names.py`。
4. **看板**：`bash scripts/run_rviz.sh`（`config/rviz_super.rviz`：世界点云、ROG-Map 占据、
   SUPER 的 committed/exp 轨迹、SFC 走廊、A\* 前端路径、FCU 位置）。

---

## 5. 目录

```
super_super_apm/
├── config/
│   ├── super_forest_15m.yaml      ★SUPER 规划器配置（拷贝到 super_planner/config/）
│   ├── waypoint_forest_15m.yaml   ★waypoint_mission 配置
│   ├── calib_identity.json        bridge 的 L→W = 单位矩阵（理由写在文件里）
│   └── rviz_super.rviz            RViz 看板
├── worlds/
│   ├── forest_square15.yaml       52 根树干；下半段方盒 / 上半段圆柱（z_split=2.10）
│   ├── waypoints_square15.txt     12 行 `x y z switch_dis`
│   └── gen_world 相关脚本见 src/tools/build_world.py
├── params/
│   ├── apm_extnav.parm            ★真机：EKF 用外部导航
│   ├── apm_sitl.parm              ★SITL 覆盖（GPS 位置源 / 跳过预解锁检查 / GUID_TIMEOUT=0）
│   └── apm_guided_fast.parm       GUIDED 跟踪参数（4.7 的新名字，见 STATUS 21.9）
├── src/whtf/
│   ├── common/obstacles.py        方盒/圆柱的统一符号距离 + world 加载（评分与建造共用）
│   ├── ros/sitl_odom_relay.py     SITL 里扮演 LIO + 自动解锁/起飞 + GO + 轨迹落盘 + 打分
│   ├── ros/pc_adapter.py          SUPER 的 mars msg → quadrotor_msgs；并造世界点云
│   └── tools/build_world.py       确定性世界生成器（每条边中点塞一棵挡路树干）
├── scripts/
│   ├── run_and_score.sh           ★一条命令跑完 + 打分
│   ├── run_sitl_loop.sh           编排（SITL / relay / adapter / SUPER / bridge）
│   ├── score_trace.py             ★离线复算验收（只读 csv）
│   ├── check_param_names.py       ★灌参数前对固件整表校验名字
│   ├── analyze_ref.py             参考 vs 实际的时间序列（看"滞后"还是"参考在跳"）
│   ├── run_rviz.sh / gen_world.sh / status.sh / dump_all_params.py ...
└── docs/
    ├── INTEGRATION.md             ★完整记录（§0–§51：每次实验的实测数字与结论）
    ├── PIPELINE.md                ★数据管线 + 三个状态机的逐条转移
    ├── MISSION.md                 任务与场地几何（15 m 正方形 × 3 圈、84 柱）
    └── evidence/                  只留最新一批图与数据（9 个文件 / 1.4 MB，索引见其 README.md）
```

---

## 6. 边界（明确没做的事）

- **SITL 里没有真 Super-LIO**：位姿是 `sitl_odom_relay.py` 用 `LOCAL_POSITION_NED` 转出来的。
  这验证的是 `航点 → SUPER → /plan/cmd → MAVLink → ArduPilot 位置环 → 真实位置`
  这一段闭环；LIO 的定位精度/漂移不在本轮证据范围内。切换路径见 §4。
  不用 Gazebo：看板走 RViz（`config/rviz_super.rviz` + `scripts/run_rviz.sh`）。
- **RViz 配置未在本环境实跑**（WSL 无显示），话题名取自 SUPER 源码里的
  `visualization/*`；第一次开时按需勾选。
- **真机参数未复调**（见 §4.3），`params/apm_guided_fast.parm` 里的数字是 SITL 的起点。
- **巡航速度没到 4 m/s**（早期架构下飞机 p90 3.73 / 参考 4.89）。根因与 12 组对照实验见
  `docs/INTEGRATION.md`（原 STATUS 二十三，已归档）：SUPER 的 FSM 里**新 goal 一定走 `PlanFromRest`（rest2rest，两端速度 0）**，
  加上 A\* 局部网格只有 ±7.5 m，一条参考轨迹被切成 ~8 m 一段，飞机永远处在加速段。
  解法是写一个 look-ahead 的 mission 驱动、让 `ReplanOnce` 一路续规划
  （`waypoint_mission` 的 `cmd_type` 在 ROS2 版里是死配置，没有 PATH 模式可切）。
  这件事没做，所以不写在"已完成"里。
