# CS-202618 实验室移动协作机器人

面向实验室智能管理的移动协作机器人仿真系统。项目基于 ROS 2 Humble、Gazebo Classic、Nav2、MoveIt 2 和 ArUco 感知，完成一体化麦克纳姆底盘 + UR5e + 平行双指夹爪在双工位场景中的识别、抓取、搬运、放置和返航。

赛题：CS-202618 中车株洲电力机车有限公司「面向实验室智能管理的协作机器人环境感知与动作规划方法研究」。

## 当前状态

当前实现已完成导航地图、任务编排、抓取失败恢复、平行夹爪软附着、ArUco 位姿、MoveIt 等运行链路的修复和加固。

- `colcon build --cmake-force-configure`：8 个包构建通过
- `colcon test --return-code-on-test-failure`：229 个测试，0 错误，0 失败，2 跳过
- 静态地图已重建，四墙覆盖 `x/y=[-3.525, 3.525]`，起点和两个工位停靠点均为 free
- `launch_mission:=false` 时不会启动 `mission_node`
- headless 端到端任务已验证到 `DONE`
- `mission_node` 是正式任务入口；旧的 `pick_place` console entry point 已移除
- 末端执行器是平行双指夹爪；旧的真空吸盘模型和 `/suction/switch` 运行路径已移除

## 系统流程

```text
/task/instruction
  -> mission_node
  -> Nav2 AMCL/EKF + DWB
  -> /cmd_vel -> 麦克纳姆底盘
  -> aruco_detector -> TF/PoseStamped
  -> MoveIt 2 + pymoveit2
  -> SimAttachGripperDriver
  -> /gripper_position_controller/commands
  -> gripper_attach_bridge soft attach/detach
  -> /task/status
```

任务状态机：

```text
NAV_TO_PICK -> DETECT -> PICK -> NAV_TO_PLACE -> PLACE -> RETURN_HOME -> DONE
```

失败时会执行退让、停止底盘、释放夹爪软附着对象等清理动作，避免任务失败后留下不一致的仿真状态。

## 包结构

| 包 | 职责 |
|---|---|
| `lab_cobot_description` | 一体化机器人 URDF/SRDF：麦克纳姆底盘、立柱、UR5e、平行双指夹爪、激光、IMU、相机 |
| `lab_cobot_gazebo` | 双工位实验室 world、样件模型、Gazebo spawn 和控制器启动 |
| `lab_cobot_navigation` | Nav2、AMCL、EKF、地图、工位 waypoints、导航 launch |
| `lab_cobot_moveit` | UR5e MoveIt 2 配置、controller 配置、`move_group` launch |
| `lab_cobot_perception` | ArUco 检测、相机反投影、Gazebo model pose fallback、TF/PoseStamped 输出 |
| `lab_cobot_manipulation` | pick/place 执行逻辑、MoveIt 调用、平行夹爪驱动边界和抓取序列 |
| `lab_cobot_bringup` | 一键全栈 launch、跨工位任务状态机、mission 编排、夹爪软附着 bridge |
| `pymoveit2` | vendored MoveIt 2 Python 接口，第三方许可见 `THIRD_PARTY_LICENSES.md` |

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- Nav2、MoveIt 2、robot_localization、slam_toolbox
- Python 3.10，OpenCV/cv_bridge/numpy

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
cd ~/projects/lab_cobot_ws
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
cd ~/projects/lab_cobot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch lab_cobot_bringup lab_cobot.launch.py
```

另开终端发送任务：

```bash
source /opt/ros/humble/setup.bash
source ~/projects/lab_cobot_ws/install/setup.bash
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

## 验证

本机如果存在 user-level `anyio` pytest 插件问题，给 pytest/colcon 测试命令加上 `PYTEST_ADDOPTS='-p no:anyio'`。

完整构建和测试：

```bash
cd ~/projects/lab_cobot_ws
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
Summary: 229 tests, 0 errors, 0 failures, 2 skipped
PASS: map covers four walls, has low obstacle noise, and key points are free
```

## 运行注意

- `lab_cobot.launch.py` 默认延迟启动 MoveIt/Nav2/感知/mission，以等待 Gazebo、spawn 和控制器就绪。
- 平行夹爪通过 `gripper_position_controller` 驱动手指开合；样件搬运在仿真中由 `gripper_attach_bridge` 固定到 `gripper_tcp`，不是 Gazebo 接触物理抓取。
- WSLg 下 launch 会设置 D3D12 和 Qt 相关环境变量以提高 Gazebo/RViz 稳定性。
- headless 结束 launch 时，MoveIt/rclpy 可能输出 SIGINT/shutdown 噪声；判断任务结果以 `/task/status` 是否到 `DONE` 为准。
- Gazebo GUI、物理步进和渲染性能会影响端到端任务耗时。

## 文档

- `docs/HANDOVER.md`：项目交接和运行记录
- `docs/STATUS_HONEST.md`：阶段性状态说明
- `docs/plans/2026-06-28-mobile-manipulator-cross-station-design.md`：系统设计
- `docs/plans/2026-06-28-mobile-manipulator-cross-station-plan.md`：实现计划
- `docs/notes/eyrc-smoke-test.md`：参考工程冒烟测试
- `docs/notes/overnight-progress.md`：历史推进记录

## 许可

本仓库自有代码采用 Apache-2.0；第三方组件许可见 `THIRD_PARTY_LICENSES.md`。
