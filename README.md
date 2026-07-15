# CS-202618 实验室移动协作机器人

面向实验室智能管理的移动协作机器人仿真系统。项目基于 ROS 2 Humble、Gazebo Classic、Nav2、MoveIt 2 和 ArUco 感知，完成一体化麦克纳姆底盘 + UR5e + 平行双指夹爪在双工位场景中的识别、抓取、搬运、放置和返航。

赛题：CS-202618 中车株洲电力机车有限公司「面向实验室智能管理的协作机器人环境感知与动作规划方法研究」。

## 当前状态

当前实现已完成导航地图、任务编排、抓取失败恢复、contact grasp 抓取、ArUco 相机位姿、MoveIt 等运行链路的修复和加固，默认路径不读 Gazebo 真值、不使用瞬移吸附。

- `colcon build`：8 个包构建通过
- `colcon test`：相关功能与底盘回归测试通过；已知既有 lint 与 mission E2E 问题另行记录
- 静态地图为 `slam_toolbox` 实跑产物（来源与质量门见 `maps/map_provenance.yaml`）
- 默认 `use_truth_pose=false`、`use_sim_attach=false`：视觉走相机检测，抓取走 contact grasp 插件
- headless 端到端（禁真值/禁吸附桥）已多次验证到 `DONE`
- `mission_node` 是正式任务入口；旧的 `pick_place` console entry point 已移除
- 末端执行器是平行双指夹爪；样件固定由 contact grasp 插件的 fixed-joint 实现

### 仿真保真度边界（诚实声明）

- **底盘**：默认由 `rover_twist_relay` 完成麦轮逆解、`lab_cobot_planar_drive` 同步完成正解和**有界平面位姿积分（`SetWorldPose`）**，再由 `gazebo_odom_bridge` 从 Gazebo 状态发布 `/odom`。运动学链真实，但底盘不受碰撞阻挡，里程计无轮地接触造成的漂移；不是麦轮滚子接触动力学。
- **抓取**：attach 触发是几何封套与双指触觉门控后的 fixed-joint，不是真实摩擦力闭合。搬运期间仅临时屏蔽被抓样件自身的 Gazebo 碰撞，释放时先解除 fixed-joint、恢复原碰撞掩码并清零速度；样件随后依靠重力落到 B 台面。质量、重力、桌面和底盘安全碰撞配置保持不变。
- **视觉**：默认走 RGB-D + solvePnP 真实检测管线；`/gazebo/model_states` 仅保留为 `use_truth_pose:=true` 显式调试路径。

## 系统流程

```text
/task/instruction
  -> mission_node
  -> Nav2 AMCL/EKF + DWB
  -> /cmd_vel -> rover_twist_relay(移植的麦轮逆解)
  -> /wheel_velocity_controller/commands -> lab_cobot_planar_drive(同步正解+平面位姿积分)
  -> gazebo_odom_bridge -> /odom
  -> aruco_detector(RGB-D solvePnP) -> TF/PoseStamped
  -> MoveIt 2 + pymoveit2
  -> ContactGripperDriver
  -> /gripper_position_controller/commands
  -> lab_cobot_grasp_fix(几何封套 -> fixed joint attach/detach)
  -> /task/status
```

任务状态机：

```text
NAV_TO_PICK -> DETECT -> PICK -> NAV_TO_PLACE -> PLACE -> RETURN_HOME -> DONE
```

失败时会执行退让、停止底盘、释放夹爪持有对象等清理动作，避免任务失败后留下不一致的仿真状态。

## 包结构

| 包 | 职责 |
|---|---|
| `lab_cobot_description` | 一体化机器人 URDF/SRDF：麦克纳姆底盘、立柱、UR5e、平行双指夹爪、激光、IMU、相机 |
| `lab_cobot_gazebo` | 双工位实验室 world、样件模型、自定义麦轮驱动/抓取插件、Gazebo spawn 和控制器启动 |
| `lab_cobot_navigation` | Nav2、AMCL、EKF、地图、工位 waypoints、导航 launch |
| `lab_cobot_moveit` | UR5e MoveIt 2 配置、controller 配置、`move_group` launch |
| `lab_cobot_perception` | ArUco 检测、相机反投影、Gazebo model pose fallback、TF/PoseStamped 输出 |
| `lab_cobot_manipulation` | pick/place 执行逻辑、MoveIt 调用、平行夹爪驱动边界和抓取序列 |
| `lab_cobot_bringup` | 一键全栈 launch、跨工位任务状态机、mission 编排、调试用软附着 bridge（默认关闭） |
| `pymoveit2` | vendored MoveIt 2 Python 接口，第三方许可见 `THIRD_PARTY_LICENSES.md` |

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- Nav2、MoveIt 2、robot_localization、slam_toolbox
- Python 3.10，OpenCV/cv_bridge/numpy
- 深度学习感知手动验证环境：ultralytics==8.4.90、open3d==0.19.0、
  torch==2.12.1+cu130、faster_whisper==1.2.1
- YOLO-World 离线权重路径：`~/lab_cobot_models/yolo_world_lab_slim.pt`
  （用 `tools/prepare_yolo_world_model.py` 制备，权重不进 git）

示例依赖安装：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-moveit \
  ros-humble-robot-localization \
  ros-humble-slam-toolbox \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-cv-bridge \
  python3-opencv
```

## 构建

```bash
cd ~/lab_cobot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如果 `pymoveit2` 的 symlink-install 缓存污染导致符号链接创建失败，清理后重建：

```bash
rm -rf build install log
colcon build --symlink-install
```

## 运行

启动完整仿真：

```bash
cd ~/lab_cobot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch lab_cobot_bringup lab_cobot.launch.py
```

另开终端发送任务：

```bash
source /opt/ros/humble/setup.bash
source ~/lab_cobot_ws/install/setup.bash
ros2 topic pub --once /task/instruction std_msgs/msg/String "{data: '把样件从A送到B'}"
```

查看状态：

```bash
ros2 topic echo /task/status
```

headless 运行：

```bash
ros2 launch lab_cobot_bringup lab_cobot.launch.py gui:=false use_rviz:=false
```

关键可选参数：

| 参数 | 默认值 | 作用 |
|---|---|---|
| `use_refine_detect` | `false` | 同时启用腕部精修相机、`/perception/wrist` ArUco 检测实例和 PICK 悬停后的位姿精修；失败时自动沿用粗位姿。 |

只启动全栈但不启动任务节点：

```bash
ros2 launch lab_cobot_bringup lab_cobot.launch.py gui:=false use_rviz:=false launch_mission:=false
```

分模块调试：

```bash
ros2 launch lab_cobot_description view_robot.launch.py
ros2 launch lab_cobot_gazebo world.launch.py
ros2 launch lab_cobot_moveit move_group.launch.py
ros2 launch lab_cobot_navigation navigation.launch.py
ros2 launch lab_cobot_navigation mapping.launch.py
```

调试底盘时仍从总启动入口启动完整的 `/cmd_vel` 适配链，但关闭任务节点和 RViz：

```bash
ros2 launch lab_cobot_bringup lab_cobot.launch.py launch_mission:=false use_rviz:=false
```

另开终端发布一个前进指令：

```bash
source /opt/ros/humble/setup.bash
source ~/lab_cobot_ws/install/setup.bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

单独运行 `ros2 launch lab_cobot_gazebo world.launch.py` 会启动 Gazebo、控制器、底盘插件和 `gazebo_odom_bridge`，但不会启动 `rover_twist_relay`；需要完整运行链路时请使用上述 `lab_cobot_bringup` 总启动命令。

## 验证

本机如果存在 user-level `anyio` pytest 插件问题，给 pytest/colcon 测试命令加上 `PYTEST_ADDOPTS='-p no:anyio'`。

完整构建和测试：

```bash
cd ~/lab_cobot_ws
source /opt/ros/humble/setup.bash
PYTEST_ADDOPTS='-p no:anyio' colcon build --cmake-force-configure
PYTEST_ADDOPTS='-p no:anyio' colcon test --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose
```

地图检查：

```bash
python3 src/lab_cobot_navigation/maps/check_map.py
```

期望汇总：

```text
PASS: 相关功能与底盘回归测试通过；已知既有 lint 与 mission E2E 问题另行记录
PASS: map covers four walls, has low obstacle noise, and key points are free
```

## 运行注意

- 麦轮底盘移植已合并；`feature/navigation` 是当前开发分支。

- `lab_cobot.launch.py` 默认延迟启动 MoveIt/Nav2/感知/mission，以等待 Gazebo、spawn 和控制器就绪。
- 平行夹爪通过 `gripper_position_controller` 驱动手指开合；样件固定由 `lab_cobot_grasp_fix` 插件在几何封套满足时创建 fixed joint 实现（非接触力检测，也非 SetEntityState 瞬移）；旧的 `gripper_attach_bridge` 仅在 `use_sim_attach:=true` 时作为调试后端启动。
- WSLg 下 launch 会设置 D3D12 和 Qt 相关环境变量以提高 Gazebo/RViz 稳定性。
- headless 结束 launch 时，MoveIt/rclpy 可能输出 SIGINT/shutdown 噪声；判断任务结果以 `/task/status` 是否到 `DONE` 为准。
- Gazebo GUI、物理步进和渲染性能会影响端到端任务耗时。

## 文档

以下为内部工作区文档（不随发布快照分发，仓库中不存在 `docs/` 目录）：

- `docs/HANDOVER.md`：项目交接和运行记录
- `docs/STATUS_HONEST.md`：阶段性状态说明
- `docs/plans/2026-06-28-mobile-manipulator-cross-station-design.md`：系统设计
- `docs/plans/2026-06-28-mobile-manipulator-cross-station-plan.md`：实现计划
- `docs/notes/eyrc-smoke-test.md`：参考工程冒烟测试
- `docs/notes/overnight-progress.md`：历史推进记录

## 许可

本仓库自有代码采用 Apache-2.0；第三方组件许可见 `THIRD_PARTY_LICENSES.md`。
