# 六、复杂物体抓取与精密操作控制

## 6.1 复杂物体抓取任务与控制要求

本章面向实验室复杂器具的抓取与精密操作任务，围绕机械臂运动规划、抓取位姿生成、末端执行器接触感知、夹爪闭环控制和抓取实验验证展开。系统采用分层执行结构：上层根据目标物体配置生成抓取位姿和预抓取位姿；中层通过 MoveIt2 完成机械臂全局运动规划，并在接近、下降和提升阶段采用 Cartesian trajectory 提高末端局部运动的可控性；下层由夹爪驱动模块根据双指接触状态和虚拟触觉力估计完成闭合、保持和异常保护。

本章实验分为两类证据。第一类为冻结的复杂物体抓取基准实验，用于评价复杂物体抓取成功率；第二类为力反馈闭环原型实验，用于验证夹爪触觉力反馈控制逻辑能否在不同目标上稳定参与执行。两类实验目的不同，样本规模不同，因此不直接进行成功率提升幅度推断。

## 6.2 机械臂运动学与分段动作规划

机械臂运动执行采用“OMPL 远距离规划 + Cartesian 近距离精密操作”的分段策略。远距离阶段由 MoveIt2/OMPL 规划到预抓取位姿，负责在关节限位、碰撞约束和工作空间范围内寻找可行路径。靠近物体后的下降和提升阶段使用 Cartesian trajectory，使末端 TCP 沿指定方向完成短距离直线运动，降低近目标区域不必要的腕部旋转，并便于将接触检测和夹爪控制插入抓取状态机。

实际执行流程为：目标配置生成 grasp TCP pose 和 pre-grasp pose；MoveIt2 规划并执行至 pre-grasp；Cartesian descend 到 grasp pose；夹爪根据接触和力反馈状态闭合；Gazebo grasp-fix fixed joint 建立持有关系；holding monitor 验证持有状态；最后执行 Cartesian lift 并进行 lift 后 holding 检查。

## 6.3 抓取位姿与末端轨迹控制

本章正式测试对象为高压探针工具、工装夹具和 IGBT 模块。三类物体具有不同几何特征：高压探针工具包含细长手柄/套件结构，对抓取中心和接触稳定性敏感；工装夹具包含盒状结构和工具区域，接触建立较快但需要避免过度闭合；IGBT 模块为规则但抓取宽度较小的电气器件，基准实验中曾出现单侧接触失败，因此适合验证双指接触过渡和力反馈保持逻辑。

系统基于目标物体位姿、局部抓取点、抓取方向、TCP clearance、pre-grasp distance、lift distance 和 carried collision geometry 生成抓取目标。末端轨迹控制保持碰撞检查开启，Cartesian path 需要满足 fraction 阈值后才允许执行。

## 6.4 夹爪触觉感知与接触状态判断

夹爪侧反馈包括 `/gripper/contact/fingers` 的左右指接触状态、`/gripper/contact/force` 的左右指力值以及 holding monitor 的持有状态。当前稳定基准实验采用 contact-only 模式：`enable_contact_force=false`，保留双指接触检测、fixed joint attach 和 holding/lift 验证，不使用 force gate 作为成功判据。

需要明确区分：当前 force-control 原型使用的力源为 VIRTUAL_ESTIMATE，即由仿真插件根据接触记忆和夹爪闭合量生成的虚拟触觉力估计，不是 Gazebo raw wrench，也不是实体 FSR402 传感器数据。

## 6.5 机械臂位置控制与夹爪力反馈协同控制

### 6.5.1 控制系统架构

系统采用机械臂位置控制与夹爪触觉力反馈相结合的力—位协同控制。机械臂仍由 MoveIt2 和 Cartesian trajectory 负责位置轨迹控制，力反馈不直接控制机械臂 Cartesian DOF。夹爪闭合自由度由力反馈闭环调节，其输入为左右指虚拟触觉力估计和接触状态。

【图6-1 机械臂位置控制与夹爪力反馈协同控制框图】路径：results/chapter6/report_assets/force_position_coordinated_control_architecture.png

### 6.5.2 虚拟触觉力估计与滤波

force-control 原型采用 ForceSample 抽象描述力反馈样本，包括左右指力、force source、有效性和时间戳。当前实验 force source 为 VIRTUAL_ESTIMATE。控制器对左右指力进行 3-frame moving average，并同时保留 raw force 与 filtered force 记录。由于虚拟力主要呈离散值，Probe 和 IGBT 中常见 0 N / 20 N，Fixture 中约为 13.8 N，因此该实验不能解释为精确 8N 力跟踪。

### 6.5.3 力反馈增量控制律

定义平均夹持力与力误差：

```text
F_mean = (F_L + F_R) / 2

e_F = F_target - F_mean
```

抓取建立阶段采用单向柔顺闭合与力感知保持：

```text
if e_F > deadband:
    Δq_g = clip(K_f e_F, 0, Δq_max)
else:
    Δq_g = 0
```

即力不足时继续小步闭合；达到或超过保持条件时停止继续增加闭合量。该策略避免在 fixed-joint attach 建立前主动反向松爪破坏双指持续接触条件。

### 6.5.4 双指接触与力保持状态机

夹爪控制状态机包括 POSITION_CLOSE、CONTACT_TRANSITION、FORCE_REGULATION 和 FORCE_HOLD。无接触时执行位置闭合；单侧接触时进入 CONTACT_TRANSITION，保持已接触侧并继续补偿未接触侧；双指有效接触且力样本有效时进入 FORCE_REGULATION；满足力保持和左右力平衡条件后进入 FORCE_HOLD。

【图6-2 IGBT 力反馈状态过程示意图】路径：results/chapter6/report_assets/igbt_force_control_state_flow.png

### 6.5.5 异常保护机制

控制器保留 FORCE_LIMIT、FORCE_UNBALANCED 和 FORCE_SENSOR_INVALID 等异常分支。当前实验中 VIRTUAL_ESTIMATE 不作为真实物理安全力上限触发源，避免虚拟离散力被误解释为真实过载。后续接入 FSR402 或可靠 raw wrench 后，可复用 ForceSample 接口并重新标定 target、deadband 与 safety threshold。

## 6.6 复杂物体抓取实验

### 6.6.1 实验对象与评价方法

复杂物体抓取基准实验选择高压探针工具、工装夹具和 IGBT 模块，每类目标执行 10 次完整 trial。成功标准包括 pre-grasp planning success、Cartesian descend success、有效双指接触、attach success、holding confirmed、物体被实际抬升、lift 后未掉落，并且无碰撞或规划异常。

### 6.6.2 30 次复杂物体抓取基准结果

| 对象 | 实验次数 | 成功次数 | 失败次数 | 抓取成功率 |
|---|---:|---:|---:|---:|
| 高压探针工具 | 10 | 10 | 0 | 100.00% |
| 工装夹具 | 10 | 10 | 0 | 100.00% |
| IGBT 模块 | 10 | 8 | 2 | 80.00% |
| 总体 | 30 | 28 | 2 | 93.33% |

总体抓取成功率为 28 / 30 × 100% = 93.33%，满足复杂物体抓取成功率不低于 90% 的比赛指标。IGBT 的两次失败均为 CONTACT_LEFT_ONLY / FAILED_CONTACT，说明该目标对双指接触对准和接触持续性更敏感。

【图6-3 不同复杂物体抓取成功率】路径：results/chapter6/report_assets/01_grasp_success_rate.png
【图6-4 复杂物体抓取失败阶段统计】路径：results/chapter6/report_assets/02_failure_stage.png

### 6.6.3 Lift 执行状态分析

30 次基准实验中，primary lift success 为 20/30，占 66.67%；state-verified task completion 为 8/30，占 26.67%；最终任务完成为 28/30，占 93.33%。部分 Cartesian lift trajectory 在末端执行阶段返回 MoveIt error_code=-4 / GOAL_TOLERANCE_VIOLATED，但实际 TCP 已接近 lift final target 且 holding 状态保持健康，因此记录为 STATE_VERIFIED_TASK_COMPLETION。该类样本不写作 primary trajectory execution success。

【图6-5 笛卡尔提升完成模式统计】路径：results/chapter6/report_assets/03_lift_completion.png

## 6.7 力反馈闭环原型实验

### 6.7.1 Probe 实验

高压探针工具 N=3 实验中，FORCE_AWARE_DECISION、ACTIVE_FORCE_ADJUSTMENT、FORCE_HOLD_REACHED 和 TASK_SUCCESS 均为 3/3。该目标提供了 force_error 改变 delta_q 并进一步改变 gripper command 的直接证据，说明力反馈不只是阈值判断，而是参与了夹爪闭合命令调节。

【图6-6 Probe 力反馈响应曲线】路径：results/chapter6/report_assets/04_force_control_probe.png

### 6.7.2 Fixture 实验

工装夹具 N=3 实验中，FORCE_AWARE_DECISION=3/3，FORCE_HOLD_REACHED=3/3，TASK_SUCCESS=3/3，ACTIVE_FORCE_ADJUSTMENT=0/3。三次进入 FORCE_REGULATION 时左右虚拟力均约 13.8 N，已超过 target=8N 的保持条件，控制器选择 delta_q=0 停止继续闭合。因此 Fixture 证明的是 force-aware hold 和跨物体泛化，而不是非零增量力跟踪。

【图6-7 Fixture 力反馈响应曲线】路径：results/chapter6/report_assets/05_force_control_fixture.png

### 6.7.3 IGBT 实验

IGBT 模块 N=3 实验中，FORCE_AWARE_DECISION=3/3，ACTIVE_FORCE_ADJUSTMENT=3/3，FORCE_HOLD_REACHED=3/3，TASK_SUCCESS=3/3。Trial 1 中，左指接触时间为 24.349 s，右指接触时间为 24.491 s，左右接触建立时间差为 0.142 s；从首次接触到双指接触耗时 0.142 s，从双指接触到 FORCE_HOLD 耗时 0.091 s，从首次接触到 FORCE_HOLD 总耗时 0.233 s。

IGBT 抓取过程中首先建立单侧接触，控制器未继续增加已接触侧的闭合量，同时保留另一侧接触建立过程；约 0.142 s 后形成双指接触，并在随后约 0.091 s 内进入力保持状态。

【图6-8 IGBT 力反馈响应曲线】路径：results/chapter6/report_assets/06_force_control_igbt.png

### 6.7.4 三目标综合结果

| 对象 | N | 力反馈参与决策 | 主动控制调整 | FORCE_HOLD | 完整任务成功 |
|---|---:|---:|---:|---:|---:|
| 高压探针工具 | 3 | 3/3 | 3/3 | 3/3 | 3/3 |
| 工装夹具 | 3 | 3/3 | 0/3 | 3/3 | 3/3 |
| IGBT 模块 | 3 | 3/3 | 3/3 | 3/3 | 3/3 |
| 总体 | 9 | 9/9 | 6/9 | 9/9 | 9/9 |

在冻结的 30 次复杂物体抓取基准实验中，总体成功率为 93.33%；在随后开展的三类目标共 9 次力反馈闭环原型验证中，9 次均进入有效力保持状态并完成抓取任务。由于两组实验样本规模和验证目的不同，本报告不直接据此推断力反馈控制带来的成功率提升幅度。

【图6-9 三目标力反馈闭环原型结果汇总】路径：results/chapter6/report_assets/07_force_control_summary.png

## 6.8 本章小结

本章建立了面向复杂物体抓取的分段规划与闭环执行体系：机械臂采用 OMPL 远距离规划与 Cartesian 近距离下降/抬升，夹爪侧融合双指接触反馈、虚拟触觉力估计、MA3 滤波和增量式闭合控制。冻结的复杂物体抓取基准实验达到 28/30 = 93.33%，满足复杂物体抓取成功率不低于 90% 的指标要求。力反馈闭环原型实验在三类目标共 9 次验证中均进入 FORCE_HOLD 并完成任务，说明该控制链具备进一步接入实体 FSR402 或其他硬件触觉传感器的工程基础。

需要强调的是，当前力反馈实验基于 VIRTUAL_ESTIMATE 仿真力估计，不代表实体触觉传感器闭环控制已完成，也不能表述为全机械臂笛卡尔力/位混合控制。后续工作应在保持现有 ForceSample 接口和状态机结构的基础上，接入真实左右指触觉传感器并重新标定力阈值与安全策略。
