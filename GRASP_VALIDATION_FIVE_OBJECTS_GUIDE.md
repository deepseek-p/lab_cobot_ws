# Grasp Validation Five Objects Guide

本文档整理当前 ROS2 项目中 tooling_zone 五个物体的 `grasp_validation` 独立抓取验证方法。该验证链只用于 Gazebo 世界内的抓取可行性验证，不替代、不修改正式 A-B 搬运任务。

## 一、任务链整体说明

独立 validation 任务链如下：

```text
/grasp_validation/target
        |
        v
grasp_validation_node
        |
        v
读取 grasp_target_config.py
        |
        v
获取目标物体 world pose
        |
        v
转换到 base_link
        |
        v
调用 PickPlace / MoveIt
        |
        v
pre-grasp
        |
        v
Cartesian descend
        |
        v
夹爪闭合
        |
        v
contact / attach 验证
```

关键边界：

- validation 不进入 `mission_node` 正式 A-B 状态机。
- validation 不修改 `/task/instruction` 的语义。
- validation 不修改正式 `aruco_sample` 抓取行为。
- validation 只验证 Gazebo 世界中的目标是否能够完成接近、下降、接触、attach、抬升和保持。
- 目标配置集中在 `src/lab_cobot_bringup/lab_cobot_bringup/grasp_target_config.py`。
- 执行入口是 `/grasp_validation/target`，状态输出是 `/grasp_validation/status`。

## 二、五个测试物体

### high_voltage_probe_kit

Gazebo 模型：

```text
model: model://safety_probe_kit
link: link
world pose: x=-4.30, y=-2.60, z=0.7489984341574721, yaw=-0.15
```

抓取区域：

```text
grasp_region: red screwdriver handle thick body
```

推荐抓取点：

```text
handle_grasp_local_point:
x=0.082130309
y=0.0
z=0.02671527

grasp_offset:
x=0.082130309
y=0.0
z=0.040
```

抓取姿态：

```text
grasp_yaw=0.0
handle_long_axis_yaw=0.0
tcp_clearance=0.000
```

策略说明：

- 抓红色螺丝刀刀柄最粗主体区域。
- 避免抓金属杆、刀头、细长杆件或其它黄色工具。
- 该目标已经用于验证“物体抓取点 -> PickPlace TCP target -> pre-grasp -> Cartesian descend”的基本链路。

### board_test_fixture

Gazebo 模型：

```text
model: model://pcb_test_fixture
link: link
world pose: x=-4.70, y=-2.60, z=0.7776, yaw=0.05
```

抓取区域：

```text
grasp_region: bottom_light_beige_coarse_rectangular_block
```

推荐抓取点：

```text
grasp_offset:
x=0.082585
y=-0.0729625
z=0.006

base_block_local_center:
x=0.082585
y=-0.0729625
z=-0.0000085

base_block_size:
x=0.128356
y=0.054419
z=0.055183
```

抓取姿态：

```text
grasp_yaw=pi/2
tcp_clearance=0.020
grasp_z_adjust=0.005
```

策略说明：

- 抓底部浅色/米白色大矩形粗块主体。
- 不抓中间白色连接件、细杆、上部结构或黄色圆柱。
- 夹爪应跨过粗块较短方向闭合。

### material_spare_igbt

Gazebo 模型：

```text
model: model://igbt_module_plain
link: link
world pose: x=-3.62, y=-2.60, z=0.753, yaw=0.38
```

抓取区域：

```text
grasp_region: slider_body_assembly_black_display_plus_moving_thick_block
```

推荐抓取点：

```text
grasp_local_point:
x=-0.01100000
y=0.05510000
z=-0.02000000

grasp_offset:
x=-0.01100000
y=0.05510000
z=-0.02000000
```

抓取姿态：

```text
grasp_yaw=pi/2
grasp_long_axis_yaw=pi/2
tcp_clearance=0.013
grasp_z_adjust=-0.005
```

策略说明：

- 当前目标是卡尺滑块/黑色显示主体与活动厚块组合区域。
- 不抓长尺杆、薄片、测量尖爪或很窄的滑轨。
- 不要把该目标回退到错误的 `digital_caliper_bottom_thick_block`。
- 当前调试重点是小步调整 `grasp_local_point` 的 x/y/z，姿态应来自该物体自身方向，不要复制螺丝刀 yaw。

### tooling_hand_tools

Gazebo 模型：

```text
model: model://tooling_hand_tools
link: link
world pose: x=-4.36, y=-1.96, z=0.761, yaw=0.12
```

抓取区域：

```text
grasp_region: closed_pliers_white_handle_body
```

推荐抓取点：

```text
grasp_local_point:
x=0.0
y=-0.05719125
z=-0.00016920
```

抓取姿态：

```text
grasp_yaw=pi/2
grasp_long_axis_yaw=pi/2
tcp_clearance=0.018
```

策略说明：

- 抓闭合手钳的白色手柄/主体区域。
- 不抓钳嘴、尖端或金属细杆。
- mesh 分析显示白色手柄区域尺寸约为 `0.10398 x 0.20562 x 0.02166`，当前抓取点位于两个白色手柄主体的中心线上。

### tooling_fixture_box

Gazebo 模型：

```text
model: model://fixture_box_plain
link: link
world pose: x=-3.88, y=-2.04, z=0.758, yaw=-0.28
```

抓取区域：

```text
grasp_region: central_adjustable_wrench_handle_shank
```

推荐抓取点：

```text
grasp_local_point:
x=0.0
y=0.0
z=0.00007236
```

抓取姿态：

```text
grasp_yaw=pi/2
grasp_long_axis_yaw=pi/2
tcp_clearance=0.020
```

策略说明：

- 抓活动扳手中部较规则的柄/杆主体区域。
- 避免抓扳手头部、尖端或边缘薄结构。
- 目标区域尺寸约为 `0.10411 x 0.31999 x 0.01614`。

## 三、每个物体调试过程

### high_voltage_probe_kit

- 早期曾反复检查 world z、base_link z、finger link origin 与真实 contact/collision 几何不一致的问题。
- 后续确认 validation 不应重复手算 TCP/finger contact plane。
- 当前链路以正式 PickPlace 坐标语义为参考：object grasp point in `base_link` -> `_pick_tcp_target()` -> `_pick_approach_target()` -> MoveIt target link `gripper_tcp`。
- 抓取点从模型中心改到红色螺丝刀粗柄区域。

### board_test_fixture

- 初始抓取点落在细结构或连接件附近，导致 Cartesian descend 或 contact 失败。
- 后续改为抓底部浅色/米白色粗矩形块。
- 当前使用 `base_block_local_center`、`base_block_size` 和 `grasp_candidate_1` 描述底部粗块。
- 曾做过小幅 `grasp_z_adjust`，避免夹指与旁边结构发生轻微干涉。

### material_spare_igbt

- 原 `digital_caliper_bottom_thick_block` 判断错误，抓取点会偏到非目标区域。
- 后续改为 `slider_body_assembly_black_display_plus_moving_thick_block`。
- 当前主要调参对象是 `grasp_local_point`，例如 x/y/z 的毫米级平移。
- 不要修改 yaw，不要复制螺丝刀姿态，不要把卡尺当作螺丝刀处理。

### tooling_hand_tools

- 当前推荐抓白色手柄主体区域 `closed_pliers_white_handle_body`。
- 不抓钳嘴、尖端或金属细杆。
- SDF 中 visual 和 collision 都是同一个 `pliers_closed.dae` mesh，当前抓取区域来自 mesh 的白色手柄材质包围盒。

### tooling_fixture_box

- 当前推荐抓 `central_adjustable_wrench_handle_shank`。
- 不抓扳手头部和细尖边缘。
- 该目标使用自己的 north-side validation base pose，使目标落在较舒适的机械臂工作区。

## 四、统一调参方法

如果夹爪偏移，优先调整：

1. `grasp_local_point.x`
2. `grasp_local_point.z`

必要时再小步调整 `grasp_local_point.y`。

不要优先修改：

- yaw
- quaternion
- roll
- pitch

建议调整范围：

```text
x: 每次 0.005 ~ 0.010 m
z: 每次 0.003 ~ 0.005 m
```

避免一次修改几十毫米。每次只改一个方向，实跑观察 Gazebo 中夹爪中心、两指高度和目标实体相对关系。

## 五、运行方法

先加载环境：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

启动 validation 模式。推荐每次指定一个 `validation_target`，这样机器人出生在对应目标的 validation-only base pose：

```bash
ros2 launch lab_cobot_bringup lab_cobot.launch.py \
  launch_grasp_validation:=true \
  validation_target:=tooling_hand_tools \
  launch_navigation:=false \
  launch_perception:=false \
  launch_mission:=false
```

发送目标：

```bash
ros2 topic pub --once /grasp_validation/target \
  std_msgs/msg/String "{data: 'tooling_hand_tools'}"
```

查看状态：

```bash
ros2 topic echo /grasp_validation/status
```

五个目标名：

```text
tooling_fixture_box
tooling_hand_tools
board_test_fixture
high_voltage_probe_kit
material_spare_igbt
```

## 六、失败诊断

### FAILED_DESCEND

常见原因：

- Cartesian 路径发生碰撞。
- TCP 目标过低或过高。
- 抓取点落到目标细结构或障碍结构旁边。
- 目标相对 base_link 太远，机械臂构型过低。
- 桌面 collision、目标 collision 或夹爪 palm/finger collision 阻止下降。

重点查看日志：

```text
TARGET_BASE_POSE
GRASP_WORLD_POINT
GRASP_BASE_POINT
DESCEND_START_TCP_POSE
DESCEND_END_TCP_POSE
DESCEND_STAGE*_TARGET
DESCEND_STAGE*_FRACTION
PRE_GRASP_JOINTS
```

### FAILED_CONTACT

常见原因：

- 夹爪没有真正接触目标。
- 抓取区域选错，例如抓到细杆、边缘、尖端或非实体中心。
- 高度略高，物体没有进入两指之间。
- 高度略低，夹指或 palm 被旁边几何卡住。
- yaw 方向使两指沿长边闭合，无法形成双指接触。

重点查看日志：

```text
GRASP_LOCAL_POINT_FINAL
GRASP_WORLD_POINT
GRASP_BASE_POINT
TARGET_BASE_POSE
TCP_TARGET_Z
FINAL_GRASP_YAW
FINAL_GRASP_QUAT
```

## 七、最终原则

1. 保持正式 A-B 链不变。
2. validation 只验证抓取，不负责正式搬运任务。
3. 不同物体使用自己的抓取点。
4. 不要复制螺丝刀 yaw 到其它物体。
5. 抓取点应根据物体真实几何选择。
6. 调整优先级：位置 > 高度 > 姿态。
7. 每次只小步调整一个变量，避免把多个问题混在一起。
8. 目标应尽量位于 base_link 前方舒适工作区，避免机械臂大幅前伸、压低或贴桌。
