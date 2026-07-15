# lab_cobot_ws — 实验室移动协作机器人仿真

ROS 2 Humble + Gazebo Classic 11 仿真系统。麦克纳姆底盘 + UR5e + 平行双指夹爪，双工位 ArUco 识别→抓取→搬运→放置→返航。

赛题：CS-202618 中车株洲「面向实验室智能管理的协作机器人环境感知与动作规划方法研究」。

## 快速接手

```bash
cd ~/lab_cobot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch lab_cobot_bringup lab_cobot.launch.py
```

headless 运行：加 `gui:=false use_rviz:=false`。单独调试底盘：加 `launch_mission:=false`。

## 包结构（8 包）

| 包 | 职责 |
|---|---|
| `lab_cobot_description` | URDF/SRDF：mecanum3 底盘、立柱、UR5e、夹爪、激光、IMU、腕相机 |
| `lab_cobot_gazebo` | Gazebo world、麦轮 planar drive / grasp fix 插件、odom bridge |
| `lab_cobot_bringup` | 一键 launch、mission_node 任务状态机、rover_twist_relay / passive_mecanum |
| `lab_cobot_navigation` | Nav2 AMCL/EKF/DWB、地图、waypoints |
| `lab_cobot_moveit` | MoveIt 2 配置、table_scene_initializer |
| `lab_cobot_perception` | ArUco detect（bench + wrist）、solvePnP→TF/PoseStamped |
| `lab_cobot_manipulation` | pick/place 序列、GripperDriver、scene_obstacles |
| `pymoveit2` | vendored MoveIt 2 Python 接口 |

## 关键参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `use_truth_pose` | false | 不读 Gazebo model_states |
| `use_sim_attach` | false | 不走软附着 bridge |
| `require_finger_contact` | true | 双指触觉门控 attach |
| `use_wrist_detect` | true | 腕相机 eye-in-hand 检测 |
| `use_refine_detect` | true | PICK 悬停精修 |
| `use_planning_scene_obstacles` | true | MoveIt 台面碰撞盒 |

## 任务流程

```text
NAV_TO_PICK → DETECT → PICK → NAV_TO_PLACE → PLACE → RETURN_HOME → DONE
```

底盘链路: `/cmd_vel → rover_twist_relay(逆解) → wheel_velocity_controller → lab_cobot_planar_drive(正解+位姿积分) → gazebo_odom_bridge → /odom`

## 测试

```bash
PYTEST_ADDOPTS='-p no:anyio' colcon test --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose
```

定向回归（不启动 Gazebo 的快速反馈）：
```bash
colcon test --packages-select lab_cobot_description lab_cobot_gazebo lab_cobot_bringup
```

## 可用技能

位于 `.claude/skills/`（不上传仓库）。处理对应模块时主动调用：

| 场景 | 技能 |
|---|---|
| 构建/工作区 | `colcon-workspace` |
| URDF/xacro/模型 | `urdf-gz-plugins` |
| Gazebo 物理/插件 | `physics-tuning`, `collision-geometry`, `controller-frequency-tuning` |
| TF/坐标系 | `coordinate-frames-and-tf`, `ekf-sensor-fusion` |
| 导航/Nav2 | `costmap-architecture`, `dwb-controller`, `nav2-simple-commander` |
| 感知/相机 | `depth-camera-pipeline`, `object-detection-pipeline`, `robot-perception` |
| Bringup/Launch | `launch-files`, `robot-bringup`, `logging-and-diagnostics` |
| 惯性/质量 | `inertia-calculation` |
| 测试 | `robotics-testing`, `ros2-testing` |
| 通用 ROS 2 | `ros2-engineering-skills` |

## 文档

位于 `docs/` 目录（随仓库分发）：

- `docs/运行与验证.md` — 完整运行步骤、验证命令、常见问题排查
- `docs/superpowers/specs/` — 设计文档（mecanum3 底盘移植、视觉安全流等）
- `docs/superpowers/plans/` — 实现计划

## 注意事项

- 修改 C++ Gazebo 插件后必须 `colcon build` + 重启 Gazebo
- WSLg 下 launch 会自动设置 D3D12/Qt 环境变量
- 任务验收以 `/task/status` 到 `DONE`、样件稳定落桌为准
- CRLF 换行符警告可忽略（WSL/Windows 交叉环境正常现象）
- 底盘位姿积分走 SetWorldPose，不受碰撞阻挡，非滚子接触动力学
- 抓取是 fixed-joint，非真实摩擦力闭合
