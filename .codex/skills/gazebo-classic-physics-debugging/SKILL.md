---
name: gazebo-classic-physics-debugging
description: 'Debug Gazebo Classic physics blowups and constraint fights. Use when objects launch at absurd velocities (100+ m/s twist), dynamically created fixed joints conflict with contact constraints, SetWorldPose pose-driven models interact with ODE, reading joint reaction forces via GetForceTorque, or estimating kp/kd/ERP force magnitudes.'
---

# Gazebo Classic 物理约束调试速查

> 来源：2026-07 lab_cobot_ws 实测（Gazebo Classic 11 / ODE / ROS 2 Humble）。
> 标注【实测】的条目均有第一手运行证据；【未验证】为凭记忆、使用前需确认。

## 1. 约束爆炸（constraint fight）诊断

**症状**：物块 twist 突然达到荒谬量级（实测 `lin=(5.5, -181.6, -110.9)` m/s），被弹飞出场；
常伴随后续 MoveIt 执行失败（爆炸冲量经关节反灌机械臂，打破轨迹容差）。

**根因**：两个刚性约束对同一 link 提出矛盾要求，ODE 求解器用巨大校正速度"解决"矛盾。
典型场景【实测】：物块经 fixed joint 焊在手指上（约束 A：保持相对位姿），机械臂把它压进桌面
（约束 B：接触不得穿透）。位置控制的臂无顺应性，矛盾持续累积直到爆炸。

**量级估算**（判断日志里的力/速度是否"爆炸级"）：
- 接触力 ≈ kp × 穿透深度。kp=1e6、穿透 1cm → ~1e4 N。
- 作用在 0.05kg 物块上：a ≈ 2e5 m/s²，单个 1ms 物理步 Δv ≈ 200 m/s——与实测 181 m/s 同量级【实测吻合】。

**修法（按有效性排序）**：
1. **机制性根治：不让矛盾发生**。放置场景用"悬空释放"——下降只到目标上方 2cm 即松爪，
   物块自由落体（0.05kg 落 5cm 完全无害），带焊物块永不接触台面【实测：修复后空闲 3 连跑 +
   满核负载 E2E 均通过，修复前负载下必炸】。
2. 余量设计要覆盖误差预算：视觉 z 误差(1~3cm) + 执行容差 + 姿态变化。名义余量 5mm 必炸，
   1cm 边际（负载敏感、时好时坏），5cm 稳【实测】。
3. detach 时 `SetLinearVel/SetAngularVel(Zero)` 清速度**挡不住已发生的穿透**——清零后下一
   物理步接触求解器照样把物块弹出【实测无效，勿依赖】。

**负载敏感 ≠ 随机 flaky**【实测】：满核 CPU 负载下 RTF 波动放大穿透深度，边际余量的系统
"空闲通过、负载必炸"。看到这种模式先查物理余量，不要归咎于环境抖动。

## 2. 动态创建 fixed joint 与力回读

```cpp
// attach【实测可用】
fixed_joint_ = world_->Physics()->CreateJoint("fixed", model_);
fixed_joint_->Load(parent_link, child_link, ignition::math::Pose3d());  // 以当前相对位姿焊接
fixed_joint_->Init();
fixed_joint_->SetProvideFeedback(true);  // 之后才能读反力

// 读约束反力【实测可用，读到 110.6N】
const auto wrench = fixed_joint_->GetForceTorque(0);
double reaction = std::max(wrench.body1Force.Length(), wrench.body2Force.Length());

// detach【实测可用】
fixed_joint_->Detach();
fixed_joint_.reset();
```

- 不调用 `SetProvideFeedback(true)` 时 GetForceTorque 返回零【未验证——本次直接加了该调用】。
- detach 后立即防重吸附：闭爪状态下会满足几何条件再次 attach，需要 suppress 标志直到重新张开【实测】。
- 物块保留质量/重力/碰撞——fixed joint 是动力学约束，非瞬移，明显优于 SetEntityState 循环。

## 3. 力阈值保险丝在位姿驱动架构下不可行（重要结论）

**背景**：想用"fixed joint 反力超阈值自动 detach"防约束爆炸。**实测两次翻车，结论：不可行。**

实测数据链：
1. attach 时物块还坐在台面上：fixed joint 与台面支撑约束共存，**稳态反力 110.6N**
   （0.05kg 物块，重力仅 0.5N！）→ 50N 阈值立即误触发，PICK 失败。
2. 阈值提到 500N + 连续 3 tick 确认后：**搬运途中再次误触发**，物块半路掉落，
   夹爪空手走完流程"Place complete"（任务层不感知丢失）。

**根因推导**：底盘是 `SetWorldPose` 位姿驱动——每 tick 整个机器人模型被瞬移
（0.45 m/s × 1ms ≈ 0.45mm/tick）。物块不属于机器人模型、靠 fixed joint 跟随，
ODE 用 ERP 校正力拉动它：**F ≈ m·Δx·erp/dt² ≈ 0.05×4.5e-4×0.2/1e-6 ≈ 4500N 量级**。
即正常搬运的校正力与异常压入力（1e3~1e4N）完全重叠，**任何阈值都无法分离正常/异常**。

**修法**：放弃力信号，回到第 1 节的机制性根治（悬空释放）。若保留保险丝代码，默认禁用
（threshold=0）并写测试锁定"必须保持禁用"。

## 4. SetWorldPose 位姿驱动模型的连锁陷阱

| 症状 | 根因 | 修法/对策 |
|---|---|---|
| 机器人可推穿墙/桌，Nav2 避障只在规划层生效 | 每 tick SetWorldPose 覆盖一切物理，碰撞求解器无法阻挡 | 如实标注局限；避障验证只能信规划层 |
| /odom 零漂移、与真值恒等 | odom 发布的就是插件刚写入的积分位姿 | 不要当作真里程计精度证据；协方差是占位常数 |
| 外部 `set_entity_state` 复位机器人无效 | 下一 tick 插件把位姿写回内部积分值 | 删除此类调用（死代码）；复位需改插件内部状态 |
| fixed joint 连接的外部物体受巨大校正力 | 见第 3 节 ERP 公式 | 力信号不可用于异常检测 |
| 轮子加了 collision 后可能与位姿驱动打架（抖动） | 接触脉冲 vs 每 tick 强写位姿 | 【未验证，反事实推演】本项目用 collide_bitmask 0x00 禁用轮碰撞规避 |

注意：`<collide_bitmask>0x00</collide_bitmask>` 会让该 link 的 collision 及其 mu/fdir1/kp/kd
**全部失效**（按位与为 0 永不产生接触）——看起来配了滚子摩擦实际不生效【实测确认配置行为】。

## 5. 接触参数量级速查

- `kp`（接触刚度）：1e6 是 Gazebo 常规值【本项目实装且运行正常】；穿透力 ≈ kp×深度。
- `kd`（接触阻尼）：1.0 偏小但常见【本项目实装】。
- `minDepth`：0.001（1mm 内不产生接触力，标准稳定化）【本项目实装】。
- ODE ERP 默认 0.2【未验证具体默认值，估算时可用】。
- gazebo_ros2_control 的 `position` 命令接口 = 运动学强制（无限刚度），真挤压轻小物体
  易触发第 1 节爆炸【推断，本项目为此把夹爪闭合指令调成与物块免接触】。

## 6. 排障最小流程

1. 抓 `/gazebo/model_states` 的 twist——量级 >10 m/s 即爆炸，定位发生时刻。
2. 对照任务日志找爆炸前的最后一个动作（下压？release？attach？）。
3. 检查该时刻是否存在"两个约束抢同一 link"：fixed joint + 接触 / 位置控制 + 接触。
4. 用第 1 节公式核对量级，确认后走机制性根治，不要调参数硬扛。
