# 实验室项目底盘移植收尾设计

日期：2026-07-12  
分支：`feature/mecanum3-chassis-port`

## 目标

保持项目最开始的完整仿真启动方式不变：

```bash
ros2 launch lab_cobot_bringup lab_cobot.launch.py
```

用户不需要额外启动底盘专用 launch。完整启动完成后，Nav2、任务节点以及人工调试继续使用实验室项目原有的 `/cmd_vel` 接口，通过移植自 `mecanum_ws` 的麦轮解算驱动 Gazebo 中的移动机械臂。

## 范围

本次是底盘移植的收尾，不重新设计实验室项目的运行逻辑。范围只包括底盘模型、底盘运动适配、运行验证和必要的中文文档，不改变以下内容：

- 不修改已经确认的麦轮逆解公式和轮序；
- 不改为轮地接触动力学；
- 不改变机械臂、夹爪、感知和任务状态机逻辑；
- 不改变 Nav2 的话题、参数、行为树、地图和启动逻辑；
- 不改变任务指令、任务状态和任务触发方式；
- 不把 `rover_twist_relay` 塞入 `lab_cobot_gazebo/world.launch.py`；
- 不增加新的默认启动命令。

## 总体架构

```text
ros2 launch lab_cobot_bringup lab_cobot.launch.py
  -> lab_cobot_gazebo/world.launch.py
       -> Gazebo、机器人、ros2_control、wheel_velocity_controller
       -> lab_cobot_planar_drive（同步 Gazebo 更新）
       -> gazebo_odom_bridge（唯一 /odom 与 TF 发布者）
  -> 底盘适配节点（唯一轮速命令发布者）
       <- /cmd_vel（保持实验室项目原接口）
       -> /wheel_velocity_controller/commands
```

底盘适配节点只承担实验室项目 `/cmd_vel` 与新底盘之间的接口适配，内部麦轮解算保持 `mecanum_ws` 已确认版本。它负责限速、死区、加减速、超时停车和麦轮逆解。`wheel_velocity_controller` 负责四个可视轮关节的转动。`lab_cobot_planar_drive` 在 Gazebo 每个物理更新周期读取同一组轮速，使用完全匹配的麦轮正解更新整机平面位姿。`gazebo_odom_bridge` 从 Gazebo 实际状态生成唯一 `/odom` 和 `odom -> base_footprint`。

节点内部是否保留兼容用 `/rover_twist` 订阅不影响系统运行；正式运行链、README 和验收均只以实验室项目原来的 `/cmd_vel` 为准。

## 固定运动学约定

底盘运动参数必须在 relay、URDF 插件配置和测试中保持一致：

```text
wheel_radius = 0.07 m
wheel_separation_width = 0.24 m
wheel_separation_length = 0.175 m
轮序 = front_left, front_right, back_left, back_right
控制器符号 = [-FL, -FR, -BL, -BR]
```

坐标约定遵循 ROS REP-103：

- `linear.x > 0`：车体前进；
- `linear.y > 0`：车体向左横移；
- `angular.z > 0`：从上方看逆时针旋转；
- x、y、z 分量可组合，因此支持斜向移动和边走边转。

## 启动行为

`lab_cobot.launch.py` 的原有职责、启动参数和上层节点保持不变，仅保证移植后的底盘适配节点随原启动流程启动：

1. `rover_twist_relay` 只启动一个实例；
2. relay 显式获得 `mecanum3` 几何参数、限速、加速度和超时参数；
3. Gazebo 插件和 relay 的限制参数相互匹配；
4. 控制器尚未激活时 relay 发布的零轮速不会造成错误运动；
5. Nav2 输出 `/cmd_vel` 时不新增 remap，也不改变 Nav2 配置；
6. `launch_mission:=false` 时仍可以从 `/cmd_vel` 手动验证底盘。

分模块的 `world.launch.py` 继续只负责 Gazebo，不反向依赖 bringup。这避免完整启动时产生两个 relay，也维持 ROS 2 包职责边界。

## 安全与异常处理

- 输入速度先执行有限值检查；NaN 或无穷大不得进入轮速输出；
- 速度限制为 `vx=0.5 m/s`、`vy=0.3 m/s`、`wz=1.2 rad/s`；
- 加速度限制为平移 `0.5 m/s²`、旋转 `1.5 rad/s²`；
- relay 超过 `0.25 s` 未收到指令时立即发布零轮速；
- Gazebo 插件超过 `0.3 s` 未收到有效轮速时停车，作为第二层保护；
- 仿真时间回退、暂停或大跨度跳变时清空历史运动状态；
- `/odom` 必须只有 `gazebo_odom_bridge` 一个发布者；
- `/wheel_velocity_controller/commands` 必须只有 `rover_twist_relay` 一个发布者。

## 验证方案

### 静态与单元测试

- 锁定一键 launch 中 relay 数量和参数；
- 锁定逆解和正解互为逆过程；
- 覆盖前进、后退、左右横移、四个斜向、顺逆时针旋转；
- 覆盖组合平移旋转、限速、加速度、死区、超时和异常数值；
- 检查不存在重复轮速或重复里程计发布节点。

### 动态 Gazebo 验收

使用原始命令启动完整系统，但关闭任务节点避免自动任务干扰：

```bash
ros2 launch lab_cobot_bringup lab_cobot.launch.py \
  launch_mission:=false use_rviz:=false
```

动态验收项目：

1. 静止时底盘 `z`、roll、pitch 无可见波动；
2. 正负 `linear.x` 产生正确前进和后退；
3. 正负 `linear.y` 产生正确左右横移；
4. 同时设置 x、y 可斜向运动；
5. 正负 `angular.z` 产生正确方向原地旋转；
6. 停止发布后在超时时间内停车；
7. 四个轮关节视觉转动与车体运动方向一致；
8. `/odom` 位姿和速度随 Gazebo 实际运动更新；
9. Nav2、MoveIt、感知和控制器节点仍能正常启动。

## 文档交付

README 的中文运行章节继续保留项目原来的任务运行方式，只补充与底盘移植直接相关的说明：

- 完整启动、无任务调试和 headless 三种原有用法；
- `/cmd_vel` 底盘接口说明；
- 前进、横移、斜移和旋转的发布示例；
- 如何检查唯一发布者、控制器状态和超时停车；
- `world.launch.py` 只用于 Gazebo 分模块调试，单独运行时不自动启动 relay。

## 完成标准

只有同时满足以下条件才算完成：

- 原始一键命令能够启动完整系统；
- 麦轮参数和解算保持原移植版本不变；
- 六类基础运动和组合运动方向正确；
- 超时停车、仿真暂停和异常时间处理有效；
- 底盘显示正常且不再上下晃动；
- 轮速和里程计均没有重复发布者；
- 相关构建、单元测试、launch 测试和动态 Gazebo 验收通过；
- 中文 README 与实际运行方式一致。

## 明确不做的改动

- 不用底盘调试入口替换 `/task/instruction` 正式任务入口；
- 不改变 `mission_node` 的状态机、导航目标或抓放顺序；
- 不改变 Nav2、MoveIt、感知、语音和夹爪节点的启动条件；
- 不为了底盘测试引入新的常驻遥控节点；
- 不将底盘移植扩大为全项目运行架构重构。
