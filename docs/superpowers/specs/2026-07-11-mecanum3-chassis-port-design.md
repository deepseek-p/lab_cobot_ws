# Mecanum3 底盘移植设计

## 目标

把 `/home/lenovo/mecanum_ws` 中已经调好的 `mecanum3` 底盘移植到
`/home/lenovo/lab_cobot_ws`，完整保留源项目的底盘外观、轮序、麦轮逆解、
速度限制、加速度斜坡、命令超时机制以及 Gazebo 运动模型。

移植后的 `lab_cobot_ws` 必须可以独立构建和运行，运行时不能依赖
`mecanum_ws` 的源码或 `install` 目录。

## 采用方案

把所需底盘资源和运行实现复制到 Lab Cobot 现有软件包中，仅适配包注册、
可执行文件名称、整机模型名称、launch 接线以及机械臂与底盘之间的固定连接。

以下源项目行为属于不可改变的契约：

- `rover_twist_relay.py` 保持 `r=0.07`、`W=0.24`、`L=0.175`。
- 四轮输出顺序和符号保持 `[-FL, -FR, -BL, -BR]`。
- 速度限制保持 `vx=0.5`、`vy=0.3`、`wz=1.2`。
- 加速度限制保持平面方向 `0.5`、角速度方向 `1.5`。
- relay 命令超时保持 0.25 秒，运动节点命令超时保持 0.3 秒。
- 运动节点继续积分平面 Twist，并通过 `/set_entity_state` 更新 Gazebo 整机位姿。

本次不使用当前项目的 `mecanum_wheel_visualizer` 或
`lab_cobot_mecanum_drive` 替换上述行为。

## 底盘模型

移植内容包括源项目的 `mecanum3` 车体、四个悬挂臂、四个轮子模型、反向轮
模型、被动可视化滚子、惯性参数、关节轴、悬挂限位以及 Gazebo 表面参数。

所有资源复制到 `lab_cobot_description`，使 `package://` 地址完全在
`lab_cobot_ws` 内解析。源 Xacro 改造成可组合宏，对外继续提供当前整机使用的
`base_footprint` 和 `base_link`。原底盘关节名称保留，因为这些名称承载了
源项目轮序含义，并被原控制器配置使用。

## 机械臂安装

在 `mecanum3` 底盘中心增加固定转接板。当前 0.30 米立柱、UR5e、夹爪、
激光雷达、RGB-D 相机和可选腕部相机继续通过 Lab Cobot 现有装配关系安装。

转接板补偿源底盘顶面高度，使 UR5e 基座高度尽量保持当前已验证值。转接板
只承担结构连接，不参与底盘运动解算。

## 运行数据流

```text
/cmd_vel 或 /rover_twist
  -> 移植后的 rover_twist_relay
  -> /wheel_velocity_controller/commands
  -> 原 mecanum3 四个轮关节（负责可视化旋转）

/cmd_vel 或 /rover_twist
  -> 移植后的 mecanum_gazebo_kinematic_drive
  -> /set_entity_state
  -> 整机平面位姿
  -> 移植后的里程计桥接节点
  -> /odom 与 odom -> base_footprint
```

默认整机启动路径不再启动当前 Lab Cobot 麦轮可视化节点和模型插件，避免两个
运动系统或两个发布者同时控制同一底盘。

## 软件包改动

### `lab_cobot_description`

- 保存复制过来的底盘和轮子 mesh。
- 用源 `mecanum3` 几何与关节替换简化几何底盘宏，并包装成整机可组合宏。
- 将原四个轮关节与现有夹爪、UR5e 关节一起注册到 `gazebo_ros2_control`。
- 更新轮速控制器关节列表，但不改变控制话题。
- 增加固定转接板并保留现有立柱接口。

### `lab_cobot_bringup`

- 把源麦轮逆解 relay 注册为本包可执行节点。
- 使用原几何参数、限速、斜坡、死区和超时参数启动该节点。
- 默认路径不再启动 `mecanum_wheel_visualizer`。

### `lab_cobot_gazebo`

- 加入源运动节点和里程计桥接节点，只做包级适配。
- 使用整机模型名 `lab_cobot` 以及当前 Gazebo 状态服务和话题启动。
- 从机器人描述中移除 `liblab_cobot_mecanum_drive.so`。

### `lab_cobot_navigation`

- 保持 `/cmd_vel`、`/odom`、`odom` 和 `base_footprint` 接口不变。
- 只有实测源底盘外轮廓与当前配置不一致时，才调整 footprint 或机器人半径。

## 测试策略

实施过程遵循测试驱动开发。

1. 先添加纯前进、纯横移、纯旋转的逆解契约测试，以及限速、斜坡、死区和
   超时测试；确认测试在源 relay 引入前按预期失败。
2. 添加 Xacro 契约测试，检查源关节名、轮子几何、滚子、转接板、根坐标系，
   并确认旧 Lab Cobot 驱动插件已经移除。
3. 添加 launch 契约测试，证明系统只启动一个逆解 relay、一个运动节点和
   一条里程计链路。
4. 构建受影响的软件包，运行其单元测试和 launch 测试。
5. 无界面启动 Gazebo，验证整机生成以及控制器激活。
6. 分别发布有界前进、横移和旋转命令，测量 Gazebo 位姿、里程计和轮速命令
   的方向。
7. 启动完整 Lab Cobot，验证 TF、Nav2、MoveIt、感知和任务空闲状态。
8. 环境稳定时发送标准 A 到 B 任务，以 `/task/status == DONE` 作为端到端
   验收条件。

## 验收标准

- 不加载 `mecanum_ws/install` 时，`lab_cobot_ws` 仍能独立构建。
- Gazebo 和 RViz 显示原版 mecanum3 底盘与完整麦轮。
- 源麦轮逆解数值、轮序和符号没有改变。
- 前进、横移和旋转命令符合 ROS 坐标方向。
- 轮子可视化旋转与源控制器输出一致。
- 只有一个节点控制整机运动，里程计和 TF 只有一个发布权威。
- 现有 UR5e、夹爪、传感器、MoveIt、Nav2、感知和任务节点仍能启动。
- 无界面完整系统启动稳定；结构、控制器和导航验证通过后再尝试端到端
  `DONE`。

## 风险与处理

- **支撑范围较小：** `mecanum3` 小于当前底盘。运动学位姿控制避免物理倾覆，
  中心转接板减少视觉不平衡；这一仿真边界会明确记录。
- **关节符号不一致：** 精确复制源关节轴、relay 轮序和输出符号，并用契约
  测试固定行为。
- **重复运动或里程计：** 在启用导航前，通过 launch 测试和运行时发布者检查
  排除重复控制链路。
- **mesh 单位或路径错误：** 保留源 scale，Gazebo 启动前执行 Xacro 展开和
  mesh 存在性测试。
- **机械臂可达性变化：** 转接板保持已验证的 UR5e 基座高度；若抓放仍有偏差，
  在底盘验收后单独调整抓放参数，不改变底盘运动学。
- **Gazebo 启动竞态：** 保留当前 Lab Cobot 的延迟启动顺序，把控制器和状态
  服务就绪作为明确运行门槛。

## 回滚边界

移植期间 `/home/lenovo/mecanum_ws` 始终只读。`lab_cobot_ws` 的改动按职责形成
聚焦提交，因此可以单独回滚底盘移植，而不影响任务、操作和感知模块。
