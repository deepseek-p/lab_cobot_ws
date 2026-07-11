# Mecanum3 底盘移植实施计划

> **供智能代理执行：** 必须使用 `superpowers:subagent-driven-development`（推荐）或 `superpowers:executing-plans`，按任务逐项实施；使用复选框跟踪进度。

**目标：** 把 `mecanum_ws` 中已验证的 mecanum3 底盘、逆解、位姿运动和里程计链路原样移植进 `lab_cobot_ws`，并完成构建与运行验证。

**架构：** 原 `rover_twist_relay` 继续把 `/cmd_vel` 转成四轮速度供轮子动画；原 `mecanum_gazebo_kinematic_drive` 同时接收 Twist 并通过 `/set_entity_state` 推进整机；原 `gazebo_odom_bridge` 从 Gazebo link state 发布唯一 `/odom` 与 `odom -> base_footprint`。当前项目原有的麦轮 visualizer 和 drive 插件从默认链路移除，UR5e 通过底盘顶面转接 link 安装。

**技术栈：** ROS 2 Humble、Gazebo Classic 11、Xacro、gazebo_ros2_control、rclpy、rclcpp、pytest、ament/colcon。

---

## 文件结构

- 新建 `src/lab_cobot_description/urdf/inc/mecanum3_base.xacro`：封装原底盘、悬挂、轮子和滚子。
- 新建 `src/lab_cobot_description/meshes/mecanum3/*.stl`：5 个源模型资源。
- 修改 `src/lab_cobot_description/urdf/lab_cobot.urdf.xacro`：组装新底盘、转接 link、控制接口并移除旧 drive 插件。
- 修改 `src/lab_cobot_description/urdf/inc/pillar.xacro`：允许立柱安装到 `chassis_mount_link`。
- 修改 `src/lab_cobot_description/config/lab_cobot_controllers.yaml`：切换到源四轮 joint 名。
- 新建 `src/lab_cobot_bringup/lab_cobot_bringup/rover_twist_relay.py`：移植原麦轮逆解。
- 新建 `src/lab_cobot_gazebo/src/mecanum_gazebo_kinematic_drive.cpp`：移植原位姿运动节点。
- 新建 `src/lab_cobot_gazebo/src/gazebo_odom_bridge.cpp`：移植并参数化原里程计桥。
- 修改两个 CMake、package.xml 和 launch：注册并按正确顺序启动新链路。
- 新增/修改测试：逆解合同、Xacro/mesh 合同、C++/launch 接线及运行 smoke。

### 任务 1：用失败测试锁定原麦轮逆解

**文件：**
- 新建：`src/lab_cobot_bringup/test/test_rover_twist_relay.py`
- 后续新建：`src/lab_cobot_bringup/lab_cobot_bringup/rover_twist_relay.py`

- [ ] **步骤 1：编写纯函数合同测试**

```python
import pytest

from lab_cobot_bringup.rover_twist_relay import (
    SimpleTwist,
    apply_deadband,
    limit_twist,
    ramp_twist,
    twist_to_wheel_speeds,
    zero_if_timed_out,
)


@pytest.mark.parametrize(
    ("twist", "expected"),
    [
        (SimpleTwist(0.14, 0.0, 0.0), [-2.0, -2.0, -2.0, -2.0]),
        (SimpleTwist(0.0, 0.14, 0.0), [2.0, -2.0, -2.0, 2.0]),
        (SimpleTwist(0.0, 0.0, 0.14), [0.83, -0.83, 0.83, -0.83]),
    ],
)
def test_source_mecanum3_inverse_kinematics_is_preserved(twist, expected):
    assert twist_to_wheel_speeds(twist, 0.07, 0.24, 0.175) == pytest.approx(expected)
```

同时加入源项目 `limit_twist`、`ramp_twist`、`apply_deadband`、
`zero_if_timed_out` 的边界测试。

- [ ] **步骤 2：运行 RED 测试**

```bash
cd ~/lab_cobot_ws
source /opt/ros/humble/setup.bash
PYTEST_ADDOPTS='-p no:anyio' python3 -m pytest -q \
  src/lab_cobot_bringup/test/test_rover_twist_relay.py
```

预期：因 `rover_twist_relay` 尚不存在而失败。

- [ ] **步骤 3：移植最小纯函数和节点实现**

复制源 `rover_twist_relay.py`，只把节点内部计算改为调用：

```python
def twist_to_wheel_speeds(twist, wheel_radius, width, length):
    k_geom = length + width
    v_fl = (twist.vx - twist.vy - twist.wz * k_geom) / wheel_radius
    v_fr = (twist.vx + twist.vy + twist.wz * k_geom) / wheel_radius
    v_bl = (twist.vx + twist.vy - twist.wz * k_geom) / wheel_radius
    v_br = (twist.vx - twist.vy + twist.wz * k_geom) / wheel_radius
    return [-v_fl, -v_fr, -v_bl, -v_br]
```

默认参数必须保持 `0.07/0.24/0.175` 和源限速、斜坡、超时值。

- [ ] **步骤 4：运行测试至 GREEN**

重复步骤 2，预期全部通过。

### 任务 2：用失败合同锁定 mecanum3 模型

**文件：**
- 新建：`src/lab_cobot_description/test/test_mecanum3_chassis_contracts.py`
- 新建：`src/lab_cobot_description/urdf/inc/mecanum3_base.xacro`
- 新建：`src/lab_cobot_description/meshes/mecanum3/` 下 5 个 STL

- [ ] **步骤 1：编写模型合同测试**

测试展开 `lab_cobot.urdf.xacro` 后必须包含：

```python
EXPECTED_WHEEL_JOINTS = [
    "front_left_joint",
    "front_right_joint",
    "back_left_joint",
    "back_right_joint",
]
EXPECTED_WHEEL_LINKS = [
    "front_left_wheel_1",
    "front_right_wheel_1",
    "back_left_wheel_1",
    "back_right_wheel_1",
]

assert all(root.find(f".//joint[@name='{name}']") is not None
           for name in EXPECTED_WHEEL_JOINTS)
assert all(root.find(f".//link[@name='{name}']") is not None
           for name in EXPECTED_WHEEL_LINKS)
assert len([link for link in root.findall("link")
            if "_barrel_" in link.attrib.get("name", "")]) == 60
assert root.find(".//link[@name='chassis_mount_link']") is not None
assert "liblab_cobot_mecanum_drive.so" not in urdf_text
```

逐个解析 `package://lab_cobot_description/meshes/mecanum3/` URI，断言文件存在；
断言底盘 mesh `scale="0.001 0.001 0.001"`。

- [ ] **步骤 2：运行 RED 测试**

```bash
PYTEST_ADDOPTS='-p no:anyio' python3 -m pytest -q \
  src/lab_cobot_description/test/test_mecanum3_chassis_contracts.py
```

预期：缺少新 Xacro、mesh 和 joint 而失败。

- [ ] **步骤 3：复制源模型资源**

只复制以下源文件到 `lab_cobot_description/meshes/mecanum3/`：

```text
base_link.stl
arms.stl
mecanum_wheel.stl
mecanum_wheel_rev.stl
mecanum_barrel.stl
```

- [ ] **步骤 4：把源 Xacro 包装成组合宏**

保留源四悬挂、四轮和每轮 15 个滚子；mesh URI 改为
`package://lab_cobot_description/meshes/mecanum3/...`。不复制源文件中的第二个
`gazebo_ros2_control` 插件。增加：

```xml
<link name="chassis_mount_link"/>
<joint name="chassis_mount_joint" type="fixed">
  <parent link="base_link"/>
  <child link="chassis_mount_link"/>
  <origin xyz="0 0 0.165" rpy="0 0 0"/>
</joint>
```

主碰撞使用与源外轮廓相符的简化几何，visual 保持原 mesh；立柱高度使用
`0.289 m`，使 UR5e 安装面保持约 `0.530 m`。

- [ ] **步骤 5：运行模型测试至 GREEN**

重复步骤 2，并运行现有 description 测试。

### 任务 3：切换 ros2_control 与整机组装

**文件：**
- 修改：`src/lab_cobot_description/urdf/lab_cobot.urdf.xacro`
- 修改：`src/lab_cobot_description/urdf/inc/pillar.xacro`
- 修改：`src/lab_cobot_description/config/lab_cobot_controllers.yaml`
- 修改：`src/lab_cobot_description/CMakeLists.txt`

- [ ] **步骤 1：先扩充失败断言**

断言 `ros2_control` 四轮顺序严格为：

```text
front_left_joint
front_right_joint
back_left_joint
back_right_joint
```

并断言旧 `wheel_fl/fr/rl/rr_joint` 与旧 drive 插件不再出现在生成 URDF。

- [ ] **步骤 2：确认测试按预期失败**

运行任务 2 的测试命令。

- [ ] **步骤 3：实施最小组装改动**

包含并实例化 `mecanum3_base.xacro`；立柱 parent 改为
`chassis_mount_link`、高度改为 `0.289`；顶层 ros2_control 和 controller YAML
切换为源 joint 名；删除旧 drive 插件块。CMake 安装 `meshes` 目录。

- [ ] **步骤 4：展开 Xacro 并运行测试**

```bash
xacro src/lab_cobot_description/urdf/lab_cobot.urdf.xacro > /tmp/lab_cobot.urdf
check_urdf /tmp/lab_cobot.urdf
PYTEST_ADDOPTS='-p no:anyio' python3 -m pytest -q src/lab_cobot_description/test
```

预期：Xacro、URDF 与 description 测试全部通过。

### 任务 4：移植位姿运动与里程计节点

**文件：**
- 新建：`src/lab_cobot_gazebo/src/mecanum_gazebo_kinematic_drive.cpp`
- 新建：`src/lab_cobot_gazebo/src/gazebo_odom_bridge.cpp`
- 修改：`src/lab_cobot_gazebo/CMakeLists.txt`
- 修改：`src/lab_cobot_gazebo/package.xml`
- 新建：`src/lab_cobot_gazebo/test/test_mecanum3_runtime_contract.py`

- [ ] **步骤 1：编写失败接线合同**

断言 CMake 注册并安装两个 executable；drive 同时订阅 `/cmd_vel` 和
`/rover_twist`、调用 `/set_entity_state`、从 model state 初始化；bridge 订阅
link state，并发布 `/odom` 和 `odom -> base_footprint`。断言旧插件不再安装或
导出为默认运行依赖。

- [ ] **步骤 2：运行合同并确认失败**

```bash
PYTEST_ADDOPTS='-p no:anyio' python3 -m pytest -q \
  src/lab_cobot_gazebo/test/test_mecanum3_runtime_contract.py
```

- [ ] **步骤 3：复制源 C++ 并做必要参数化**

drive 数学、限速、斜坡、超时和积分保持不变；默认模型改由 launch 显式传入
`lab_cobot`。bridge 将 Gazebo link state 话题和目标 link 参数化，精确匹配
`lab_cobot::base_footprint` 或运行时实际根 link，输出 child frame 固定
`base_footprint`。

- [ ] **步骤 4：注册依赖和 executable**

CMake/package 加入 `geometry_msgs`、`gazebo_msgs`、`nav_msgs`、`rclcpp`、
`tf2`、`tf2_ros`，并把两个 executable 安装到 `lib/lab_cobot_gazebo`。

- [ ] **步骤 5：运行合同至 GREEN**

重复步骤 2。

### 任务 5：切换完整 launch 接线

**文件：**
- 修改：`src/lab_cobot_bringup/CMakeLists.txt`
- 修改：`src/lab_cobot_bringup/package.xml`
- 修改：`src/lab_cobot_bringup/launch/lab_cobot.launch.py`
- 修改：`src/lab_cobot_gazebo/launch/world.launch.py`
- 修改：`src/lab_cobot_bringup/test/test_lab_cobot_launch.py`
- 修改：`src/lab_cobot_gazebo/test/test_world_launch.py`

- [ ] **步骤 1：编写失败 launch 合同**

断言默认 launch 中：

```text
有且仅有 rover_twist_relay
有且仅有 mecanum_gazebo_kinematic_drive
有且仅有 gazebo_odom_bridge
没有 mecanum_wheel_visualizer
drive model_name == lab_cobot
drive service_name == /set_entity_state
drive 限速 == 0.5/0.3/1.2
drive 加速度 == 0.5/1.5
drive timeout == 0.3，rate == 50
```

同时断言 world spawn 后才启动控制器和底盘节点，且没有硬编码
`mecanum_ws` 路径。

- [ ] **步骤 2：运行 RED 测试**

```bash
PYTEST_ADDOPTS='-p no:anyio' python3 -m pytest -q \
  src/lab_cobot_bringup/test/test_lab_cobot_launch.py \
  src/lab_cobot_gazebo/test/test_world_launch.py
```

- [ ] **步骤 3：修改 launch 和安装规则**

安装并启动 relay；world 在四轮控制器 active 后启动 drive、bridge、relay；
bringup 删除旧 visualizer action。所有节点使用 `use_sim_time=true`。检查导航
EKF，确保它不与 bridge 同时发布 `/odom` 或 `odom -> base_footprint`。

- [ ] **步骤 4：运行 launch 合同至 GREEN**

重复步骤 2。

### 任务 6：构建与静态回归

- [ ] **步骤 1：构建受影响包**

```bash
cd ~/lab_cobot_ws
source /opt/ros/humble/setup.bash
PYTEST_ADDOPTS='-p no:anyio' colcon build --symlink-install \
  --packages-select lab_cobot_description lab_cobot_gazebo lab_cobot_bringup \
  --event-handlers console_direct+
```

预期：三个包构建成功。

- [ ] **步骤 2：运行受影响包测试**

```bash
source install/setup.bash
PYTEST_ADDOPTS='-p no:anyio' colcon test \
  --packages-select lab_cobot_description lab_cobot_gazebo lab_cobot_bringup \
  --event-handlers console_direct+
colcon test-result --verbose
```

预期：0 errors、0 failures；已有诚实 E2E 可按其独立预算运行。

### 任务 7：Gazebo 三向运动和完整系统验证

- [ ] **步骤 1：无界面启动 world**

```bash
source /opt/ros/humble/setup.bash
source ~/lab_cobot_ws/install/setup.bash
ros2 launch lab_cobot_gazebo world.launch.py gui:=false
```

- [ ] **步骤 2：验证控制器与唯一发布者**

```bash
ros2 control list_controllers
ros2 topic info -v /wheel_velocity_controller/commands
ros2 topic info -v /odom
ros2 topic info -v /tf
```

预期：轮速控制器 active；轮速命令只有 relay 发布；`/odom` 只有 bridge 发布；
`odom -> base_footprint` 只有一个权威。

- [ ] **步骤 3：分别验证前进、横移、旋转与超时停止**

依次发送 1 秒有界命令并读取 Gazebo model state 与 `/odom` 相对增量：

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.14}}"
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {y: 0.14}}"
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{angular: {z: 0.14}}"
```

预期：分别得到正 X、正 Y、正 yaw；停止发布 0.5 秒后位姿增量接近零。

- [ ] **步骤 4：启动完整系统**

```bash
timeout 180 ros2 launch lab_cobot_bringup lab_cobot.launch.py \
  gui:=false use_rviz:=false launch_mission:=false
```

预期：Gazebo、控制器、TF、Nav2、MoveIt 和感知均启动，无重复底盘执行器。

- [ ] **步骤 5：条件允许时运行标准任务**

```bash
ros2 topic pub --once /task/instruction std_msgs/msg/String \
  "{data: '把样件从A送到B'}"
ros2 topic echo /task/status
```

最终验收：状态到达 `DONE`；若只剩抓放几何偏差，保留已通过的底盘合同，另行
调整抓放参数，不修改原麦轮解算。
