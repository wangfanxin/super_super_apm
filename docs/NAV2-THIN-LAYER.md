# Nav2 调研：它的"FSM / 到达判断 / 路径跟随"是怎么做的，我们的薄层应该怎么改

> 2026-11-27。调研对象：**ros-navigation/navigation2**，main 分支快照 commit **`c3dea921ad12`**
> （2026-11-26T18:38:07Z），各包版本 **1.5.0**。源码快照位置（本机，不在本仓库内）：
> `/mnt/d/distill_self/refs/nav2src/`（通过 GitHub API tarball 获取；`raw.githubusercontent.com` 在本机被限速，
> tarball 端点可用）。版本信息取自 `nav2_controller/package.xml`、`nav2_waypoint_follower/package.xml`。
>
> 本文只回答一个问题：**"科学、合理的薄层（把规划变成指令、判断到点、处理异常）应该长什么样"**，
> 并逐条对照我们现在的 plan-box（`bridge_nodes/thin/planner_source_super.py`）+ FSM（`super_planner/src/super_core/fsm.cpp`）。

---

## 1. Nav2 的整体拓扑（先建立坐标系）

| 组件 | 包 | 作用 |
|---|---|---|
| `NavigateToPose` action | `nav2_bt_navigator` | 单目标导航：规划→跟随→恢复，行为树驱动 |
| `NavigateThroughPoses` action | `nav2_bt_navigator` | **多目标"穿过"**：一次规划穿过一串 poses，只在最后一个点判定到达 |
| `FollowWaypoints` action | `nav2_waypoint_follower` | **逐点**导航：对每个 waypoint 调一次 `NavigateToPose`，到点后执行任务插件 |
| `ComputePathToPose` / `ComputePathThroughPoses` | `nav2_planner`（planner_server） | 全局路径（后者带 viapoints） |
| `FollowPath` | `nav2_controller`（controller_server） | 路径跟随，**内部循环里判到达** |
| recovery behaviors | `nav2_behaviors` | Spin / Wait / BackUp / DriveOnHeading / AssistedTeleop |
| path smoothing | `nav2_smoother`、`nav2_constrained_smoother` | 路径平滑（角点圆滑）后再交给控制器 |
| velocity smoothing | `nav2_velocity_smoother` | 对 cmd_vel 做加速度/加加速度限幅（运动学壳） |

关键区别先说：**Nav2 没有传统 FSM 类**，它的"状态机"是一棵 **BehaviorTree.CPP v4 的行为树**；
到达判断也不是一个 if，而是 **goal checker + progress checker + 行为树恢复** 的分层组合。
"穿过中间点不停"是 `NavigateThroughPoses`，而"到每个点停一下"是 `FollowWaypoints`——这两者 Nav2 是分开的 API。

---

## 2. Nav2 的"FSM"：行为树管道（逐节点语义）

### 2.1 单目标：`navigate_to_pose_w_replanning_and_recovery.xml`

```xml
<RecoveryNode number_of_retries="6" reset_distance="3.0" name="NavigateRecovery">
  <PipelineSequence name="NavigateWithReplanning">
    <ProgressCheckerSelector .../> <GoalCheckerSelector .../>
    <PathHandlerSelector .../> <ControllerSelector .../> <PlannerSelector .../>
    <RateController hz="1.0">
      <RecoveryNode number_of_retries="1" name="ComputePathToPose">
        <Fallback name="FallbackComputePathToPose">
          <ReactiveSequence name="CheckIfNewPathNeeded">
            <Inverter><GlobalUpdatedGoal/></Inverter>
            <IsGoalNearby path="{path}" proximity_threshold="4.0" .../>
            <TruncatePathLocal .../> <ValidatePath .../>
          </ReactiveSequence>
          <ComputePathToPose goal="{goal}" path="{path}" .../>
        </Fallback>
        <Sequence> <WouldAPlannerRecoveryHelp .../> <ClearEntireCostmap .../> </Sequence>
      </RecoveryNode>
    </RateController>
    <RecoveryNode number_of_retries="1" name="FollowPath">
      <FollowPath path="{path}" controller_id=... goal_checker_id=... progress_checker_id=... />
      <Sequence> <WouldAControllerRecoveryHelp .../> <ClearEntireCostmap .../> </Sequence>
    </RecoveryNode>
  </PipelineSequence>
  <Sequence>
    <Fallback> <WouldAControllerRecoveryHelp/> <WouldAPlannerRecoveryHelp/> </Fallback>
    <ReactiveFallback name="RecoveryFallback">
      <GoalUpdated/>
      <RoundRobin name="RecoveryActions" wrap_around="true">
        <Sequence name="ClearingActions"> <ClearEntireCostmap local/> <ClearEntireCostmap global/> </Sequence>
        <Spin spin_dist="1.57"/> <Wait wait_duration="5.0"/> <SubTree ID="BackupWithReset" backup_dist="0.30" .../>
      </RoundRobin>
    </ReactiveFallback>
  </Sequence>
</RecoveryNode>
```

可直接借用的设计要点：
1. **固定频率重规划**（`RateController hz="1.0"`）而不是"消费掉一个点就重规划"。重规划的是**剩余路径**；
   并且先做 `IsGoalNearby/TruncatePathLocal/ValidatePath`，路径还有效时**不重规划**。
2. **重试预算 + 恢复动作轮转**：外层 `number_of_retries="6"`，恢复动作是 RoundRobin（清代价地图 → Spin → Wait → BackUp），
   每次失败只做一步恢复，**恢复期间收到新目标（`GoalUpdated`）立即抢占**。
3. "规划失败/控制失败"是两类不同错误码，是否值得恢复由 `WouldAPlannerRecoveryHelp` / `WouldAControllerRecoveryHelp` 判断——不是无脑重启。

### 2.2 多目标"穿过"：`navigate_through_poses_w_replanning_and_recovery.xml`

```xml
<RateController hz="0.333">
  <RecoveryNode ... name="ComputePathThroughPoses">
    <ReactiveSequence>
      <RemovePassedGoals input_goals="{goals}" output_goals="{goals}" radius="0.7"
                         input_waypoint_statuses=... output_waypoint_statuses=.../>
      <ComputePathThroughPoses goals="{goals}" path="{path}" .../>
    </ReactiveSequence>
    ...
```

关键语义（**这正是我们缺的那一层**）：
- 一次 `ComputePathThroughPoses` 穿过**所有还没过的 poses**；0.333 Hz 重复规划剩余路线；
- **中间点由 `RemovePassedGoals`（默认 radius 0.7 m）"消耗"**，只更新 waypoint 状态，**不触发停车/不触发 controller 重起**；
- **只有最后一个 pose 由 goal checker 判定到达**；反馈里带 `distance_remaining / position_tracking_error / heading_tracking_error / number_of_poses_remaining`。
- BT 节点源码：`nav2_behavior_tree/plugins/action/remove_passed_goals_action.cpp`（构造函数默认 `viapoint_achieved_radius_(0.5)`，XML 里传入 0.7）。

### 2.3 逐点停：`nav2_waypoint_follower` 的 `FollowWaypoints`

`nav2_waypoint_follower/src/waypoint_follower.cpp`：action server `follow_waypoints`，**对每个 waypoint 调一次
`NavigateToPose`**（`nav_to_pose_client_`），等 result 回调 → 执行一次 `waypoint_task_executor`
（`WaitAtWaypoint` / `InputAtWaypoint` / `PhotoAtWaypoint`）→ 下一个。失败策略：`stop_on_failure`（继续或停）。
也就是说：**"每个点停一下"在 Nav2 里是一个上层的、显式的选择**，不是路径跟随这件事本身的语义。

> 结论：我们现在的 pacer 是"每角点发一次 goal、到了就换点并触发重规划"——语义上等价于
> **FollowWaypoints（逐点停）**，但我们期望的却是 **NavigateThroughPoses（连续穿过）**。这是设计层面的错位，
> 不是参数问题。

---

## 3. Nav2 的"到达判断"：三层，各管一件事

### 3.1 GoalChecker（插件接口）——"这个目标算到达了吗"

`nav2_core/include/nav2_core/goal_checker.hpp`：

```cpp
virtual bool isGoalReached(
  const geometry_msgs::msg::Pose & query_pose,
  const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & velocity,
  const nav_msgs::msg::Path & transformed_global_plan) = 0;
```

内置插件（`nav2_controller/plugins/`）：
- **SimpleGoalChecker**：`xy_goal_tolerance 0.25 m`、`yaw_goal_tolerance 0.25 rad`、`path_length_tolerance 1.0 m`、`stateful true`。
  注意两点：
  - 它要求**剩余路径长度 ≤ 1.0 m** 才判 XY 到达（`isGoalXYReached` 开头：`calculate_path_length(transformed_global_plan) > path_length_tolerance_ → false`）——
    防止"路径还长但正好擦过目标附近"就判到达；
  - `stateful`：一旦进入容差窗口就保持"已到达"，直到离开 **容差 + buffer** 才复位。
- **StoppedGoalChecker**：在 Simple 基础上再要求 `|v| ≤ 0.25 m/s`、`|ω| ≤ 0.25 rad/s`（"停下才算到"）。
- 还有 PositionGoalChecker / AxisGoalChecker / AdaptiveToleranceGoalChecker（容差随速度自适应）。

### 3.2 ProgressChecker（插件接口）——"它还在动吗/是不是卡住了"

`nav2_controller/plugins/simple_progress_checker.cpp`：
- `required_movement_radius = 0.5 m`；
- `movement_time_allowance = 10.0 s`；
- `check(current_pose)`：在 10 s 窗口内是否移动过 ≥0.5 m，没动就报"无进展"，交给行为树恢复。
（另有 `pose_progress_checker` 版本。）

### 3.3 controller_server 的循环结构——到达与控制在同一个环里

`nav2_controller/src/controller_server.cpp`（FollowPath action 的执行体）：

```cpp
setPlannerPath(goal->path);
progress_checkers_[current_progress_checker_]->reset();
nav2::Rate loop_rate(this, params_->controller_frequency);
while (rclcpp::ok()) {
  waitForCostmap();
  updateGlobalPath();
  const auto current_robot_pose = getCurrentRobotPose();
  transformedPlanAndGoal(current_robot_pose);
  if (isGoalReached(current_robot_pose)) {   // ← 只有这里判"到点"
    RCLCPP_INFO(get_logger(), "Reached the goal!");
    break;
  }
  computeAndPublishVelocity(current_robot_pose);
  loop_rate.sleep();
}
```

**要点**：到达判断发生在"跟随同一个已规划路径"的循环里；不是"到点 → 重新规划 → 再跟随"。
规划在 BT 层以 1 Hz（穿过模式 0.333 Hz）独立进行，两者解耦。

---

## 4. Nav2 的"路径跟随"科学：Regulated Pure Pursuit（RPP）

`nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp`
+ `.../include/.../regulation_functions.hpp`。核心是"**前视点 + 曲率限速**"，而不是"沿航迹误差积分/比例"：

1. **前视点**（lookahead carrot）：
   ```
   L = clamp(|v| · lookahead_time, min_lookahead_dist, max_lookahead_dist)   // 速度自适应
   carrot = 沿路径弧长前推 L 的点（getLookAheadPoint）
   ```
2. **曲率与角速度**：
   ```
   k = 2·y_carrot / (x²+y²)      // 三点圆 / 弦长公式（calculateCurvature）
   ω_cmd = k · v
   ```
3. **速度约束**（applyConstraints）：
   ```
   v_curv  = curvatureConstraint(v, k, min_radius)
           = v·(1 - |R - R_min|/R_min) ，当 R < R_min；否则 v        // 弯越急越慢
   v_cost  = costConstraint(v, pose_cost, ...)                        // 靠近障碍减速
   v = min(v_curv, v_cost)
   v = max(v, regulated_linear_scaling_min_speed)                     // 有下限，≠ 0
   v = approachVelocityConstraint(v, path, min_approach_linear_velocity,
                                  approach_velocity_scaling_dist)      // 接近终点减速
   v = clamp(|v|, 0, max_linear_vel)
   ```
4. 旋转优先/原地转向（`use_rotate_to_heading`、`rotate_to_heading_angular_vel`）、角加速度限幅（`max_angular_accel`）等。
5. 还有 `nav2_graceful_controller`（平滑过渡控制器）、`nav2_mppi_controller`（采样最优控制）可选。

→ 对照我们的薄层：我们现在是
`v_cmd = clamp(KP·(沿航迹缺口), 0, |v_plan|+0.5)`，再补一个"比例下限"。
这其实是 **没有前视、没有曲率项、没有接近终点减速** 的退化跟随器；角点处"沿航迹缺口→0"必然把速度打到 0.27。
RPP 的做法是：**速度由路径曲率决定（弯道限速，但有下限），方向由前视点决定**——角点仍然会减速，但不会停，
而且减速量有几何依据（`v ≤ sqrt(a_lat·R)` 的等价形式）。

## 5. 还有两块我们也缺：路径平滑与指令平滑

- **`nav2_smoother`**（`simple_smoother.cpp` / `savitzky_golay_smoother.cpp`，另有 `nav2_constrained_smoother`）：
  在控制之前把路径角点圆滑（SimpleSmoother 迭代平滑，`do_refinement`、`refinement_num` 等；S-G 平滑是滑动多项式拟合）。
  我们的 patch 21"guide 圆角"是这件事的手工雏形，但没有独立的 smoother 层与单元测试。
- **`nav2_velocity_smoother`**：对最终 `cmd_vel` 做 `max_velocity/ max_accel / max_decel / max_jerk` 的独立限幅。
  我们的等价物是薄层里的 `REF_GOV_ACC/DEC`、FF 加速度上限、bridge clamp——散落在多处，没有单一"运动学壳"。

---

## 6. 逐条 gap：我们的薄层 vs Nav2

| 维度 | 我们（plan-box + SUPER FSM） | Nav2 | 后果 |
|---|---|---|---|
| 多目标语义 | 每角点发一个 goal；到了就换点并**重规划**；等价 FollowWaypoints（逐点停） | `NavigateThroughPoses`：一次规划穿过剩余 poses，0.333 Hz 重规划剩余路线；**中间点只做 RemovePassedGoals(0.7 m) 记账** | 角点手递手重规划 → 参考在角点停-转（我们的 0.02–0.12 m/s） |
| 到达判断 | pacer 半径/pass-by/dwell，既是记账又是控制触发器 | GoalChecker（状态化容差 + 剩余路径长度门槛 + 可选 stopped）+ ProgressChecker | "到点"与"控制"耦合；到点即重启 |
| 跟随律 | 沿航迹比例调速 + 比例下限；无前视/无曲率/无终点减速 | RPP：前视 + 曲率限速 + 障碍限速 + 终点减速 + 速度下限 | 角点速度不可解释；下限调大就切弯/振荡 |
| 重规划 | FSM：Traj finish → GENERATE_TRAJ → PlanFromRest；ReplanOnce 10 Hz | BT：1 Hz（穿过 0.333 Hz）重规划剩余路径，路径有效时不重规划 | 每个角点至少一次 rest-to-rest 重规划，交接处速度/位置不连续 |
| 恢复 | 无独立恢复层；失败就再生成/PlanFromRest；无重试预算 | RecoveryNode(6 次) + RoundRobin(清图/Spin/Wait/BackUp)，GoalUpdated 可抢占 | 失败循环、原地绕圈（P1 图里的 loop） |
| 路径平滑 | patch 21 guide 圆角（crude）；无独立 smoother/测试 | `nav2_smoother` 独立层，可配置、有单测 | 角点 guide 仍是折线 → MINCO/参考在弯里减速 |
| 指令平滑 | 散落的 Slew/ACC/FF 钳位 | `nav2_velocity_smoother` 单一运动学壳 | 层与层互相覆盖（我们已见：slew 被 no_retract 覆盖） |

---

## 7. 建议的"科学薄层"设计（按 Nav2 的分层，映射到我们栈）

### 7.1 目标/到达语义（Route Manager，替代 pacer 的"发目标"职责）
- 维护**路线 poses 列表**（我们的 12/18 个角点 + 终点）与**已通过索引**。
- 只给 SUPER 一个"**穿过多个角点**"的规划请求：
  - 近期：patch 22（`keep_old_goal_guide`）方向——新目标到达时把旧角点保留为 guide 必经点；
  - 中期：给 SUPER 增加 via-points/waypoint list（工程量大，需 owner 批准）；
  - 期间 pacer **只做进度记账**（按路径投影/半径 0.5–0.7 m 消耗角点，等价 RemovePassedGoals），
    **不再因为"到点"而触发重规划**。
- **终点才判到达**：xy 容差 0.3–0.5 m（+可选 stopped）；角点用"通过半径/投影"记账，不判停。

### 7.2 跟随律（Thin Layer = 跟随器，而不是参考积分器）
在已 committed 的 SUPER 轨迹上实现 RPP 等价物：
```
L   = clamp(k_look * v, Lmin, Lmax)              # 前视距离（随速度）
p_l = traj(τ + L/v) 或按弧长前推 L 的点           # 前视点
k   = 2·y_l / (x_l² + y_l²)                       # 轨迹局部曲率
v   = min(v_plan(τ), sqrt(a_lat_max / max(|k|, 1e-3)), v_max,
          v_approach(dist_to_final_goal))         # 曲率/终点限速
v   = max(v, v_floor)                             # 有下限（0.3–0.5），避免停
ω   = k·v
a   = 限幅(v - v_prev, ±a_max·dt)
发布 (p_ref = ∫v·dt 或直接 v feedforward, v, a)     # 保持 p/v 自洽
```
关键：**速度由"轨迹几何 + 终点距离"决定**，而不是由"沿航迹缺口"决定；因此角点会按曲率减速到可跟踪速度，
但不会因为"缺口消失"归零。`v_floor` 只在轨迹自身在动时生效（等价我们 94a 的 `REF_ALONG_FLOOR`，但加曲率项更合理）。

### 7.3 进度与恢复（FSM 层）
- **ProgressChecker**：10 s 内位移 <0.5 m → 触发一次"重规划/清图"；
- **恢复轮转**：重规划一次 → 仍失败 → Wait 3–5 s → （可选）BackUp 0.3 m → 再重规划；最多 6 次；
- **新目标可抢占恢复**（GoalUpdated 语义）；
- 不允许"无限 PlanFromRest"。

### 7.4 平滑
- 规划层：对 SUPER guide 或 committed 路径做独立平滑（S-G / Simple 等价物），并保持走廊约束；
- 指令层：把现有 Slew/FF/ACC 收敛成**一个** `velocity_smoother` 模块（max_v, max_a, max_jerk），所有覆盖层之后只经过它一次。

### 7.5 迁移步骤（不飞真机、不盲调）
1. **离线原型**：用已有录像（`tap.csv` + `setpoints.csv`）把新跟随器跑一遍；
   指标用我们现有 P1（规划-执行偏差 + 尖峰）、Q3、W6、角点真值速度，对比 44b/94a；
2. 通过后，在 SITL 上只改薄层（不改 SUPER）跑 1 趟 frozen；
3. 再加 Route Manager 的"通过型目标"（配合 patch 22），跑 4 图；
4. 最后才谈速度阶梯。

---

## 8. 可直接借鉴的 Nav2 参数（供我们设计取值）

| Nav2 参数 | 默认/示例 | 我们的对应建议 |
|---|---|---|
| `xy_goal_tolerance` | 0.25 m | 终点 0.3–0.5 m（角点用通过半径 0.5–0.7 m 记账） |
| `path_length_tolerance`（SimpleGoalChecker） | 1.0 m | 终点到达必须"剩余路径 ≤1 m"，防擦过误判 |
| `stateful` + `xy_goal_tolerance_buffer` | true / 0.0 | 沿用思路，避免容差边缘抖动 |
| `required_movement_radius` / `movement_time_allowance` | 0.5 m / 10 s | 进度检查：10 s 内 <0.5 m → 恢复 |
| `RemovePassedGoals radius` | 0.7 m | 角点消耗半径（只记账） |
| `RateController hz` | 1.0 Hz（穿过 0.333 Hz） | 重规划频率：我们现有 10 Hz 可以保留，但**只在剩余路径失效/目标更新时重规划** |
| `lookahead_time / min,max_lookahead_dist` | 1.5 s / 0.45–0.9 m | 前视距离 = clamp(1.0–1.5 s × v, 0.5, 2.0 m) |
| `regulated_linear_scaling_min_radius` | 0.9 m | 弯道限速的几何半径阈值（按我们 3 m 柱距走廊标定） |
| `regulated_linear_scaling_min_speed` | 0.25 m/s | 跟随下限 0.3–0.5 m/s（只在规划在动时） |
| `max_angular_accel` | 3.2 rad/s² | 角速度限幅（我们目前只有速度限幅） |

---

## 9. 参考链接
- 仓库：<https://github.com/ros-navigation/navigation2>（快照 commit `c3dea921ad12`，2026-11-26）
- 关键文件（相对快照 `nav2src/`）：
  - `nav2_core/include/nav2_core/goal_checker.hpp`
  - `nav2_controller/plugins/simple_goal_checker.cpp`、`stopped_goal_checker.cpp`、`simple_progress_checker.cpp`
  - `nav2_controller/src/controller_server.cpp`（FollowPath 循环，`isGoalReached` → break）
  - `nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`、
    `navigate_through_poses_w_replanning_and_recovery.xml`
  - `nav2_behavior_tree/plugins/action/remove_passed_goals_action.cpp`
  - `nav2_waypoint_follower/src/waypoint_follower.cpp`
  - `nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp`、
    `include/nav2_regulated_pure_pursuit_controller/regulation_functions.hpp`
  - `nav2_msgs/action/{NavigateToPose,NavigateThroughPoses,FollowWaypoints}.action`
  - `nav2_smoother/src/simple_smoother.cpp`（+ `savitzky_golay_smoother.cpp`、`nav2_constrained_smoother`）
- 官方文档（供交叉核对）：Waypoint Follower <https://docs.nav2.org/configuration_and_development/configuration_guide/core_servers/waypoint_follower/>；
  Nav2 文档主页 <https://docs.nav2.org/>
