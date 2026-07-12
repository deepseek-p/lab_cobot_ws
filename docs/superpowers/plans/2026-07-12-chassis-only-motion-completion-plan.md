# Chassis-Only Motion Completion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 完成麦轮底盘移植收尾，使实验室项目原始一键启动与 `/cmd_vel` 运行逻辑不变，并验证新底盘运动、停车和里程计链路。

**Architecture:** 保留 `lab_cobot_bringup/lab_cobot.launch.py`、Nav2、mission、MoveIt 和感知的原有组织方式。总启动中的底盘适配节点继续接收 `/cmd_vel`，使用已移植的 `mecanum_ws` 麦轮逆解发布四轮速度；同步 Gazebo 插件用匹配参数正解并由现有 odom bridge 发布唯一 `/odom`。

**Tech Stack:** ROS 2 Humble、Python launch/rclpy、Gazebo Classic 11 ModelPlugin、ros2_control、pytest、colcon test

---

## 文件结构

- 修改 `src/lab_cobot_bringup/test/test_lab_cobot_launch.py`：锁定原一键启动结构和底盘适配参数。
- 修改 `src/lab_cobot_bringup/launch/lab_cobot.launch.py`：只给现有底盘适配节点显式传入新底盘参数，不改其他节点和启动阶段。
- 修改 `src/lab_cobot_bringup/test/test_rover_twist_relay.py`：补充 `/cmd_vel` 异常数值安全契约。
- 修改 `src/lab_cobot_bringup/lab_cobot_bringup/rover_twist_relay.py`：在不改变有限输入麦轮公式的前提下拒绝非有限输入。
- 修改 `README.md`：修正已经过时的底盘节点名称，保留原运行命令并补充底盘调试示例。
- 动态验收不增加常驻测试节点；使用 `/tmp` 中的一次性 rclpy 验证脚本。

### Task 1: 锁定原一键启动结构与新底盘参数

**Files:**
- Modify: `src/lab_cobot_bringup/test/test_lab_cobot_launch.py`
- Modify: `src/lab_cobot_bringup/launch/lab_cobot.launch.py`

- [ ] **Step 1: 写失败的 launch 契约测试**

在现有 relay 测试附近增加断言，要求原启动仍包含一次 world、一次 relay，且参数显式匹配新底盘：

```python
def test_original_bringup_passes_migrated_chassis_parameters():
    launch_description = _load_launch_description()
    relay = _node("lab_cobot_bringup", "rover_twist_relay", launch_description)
    parameters = _parameter_dict(relay)

    assert parameters["rover"] == "mecanum3"
    assert parameters["mecanum3.wheel_radius"] == 0.07
    assert parameters["mecanum3.wheel_separation_width"] == 0.24
    assert parameters["mecanum3.wheel_separation_length"] == 0.175
    assert parameters["max_vx"] == 0.5
    assert parameters["max_vy"] == 0.3
    assert parameters["max_wz"] == 1.2
```

- [ ] **Step 2: 运行测试确认因参数未显式传入而失败**

Run:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest -q -p no:anyio \
  src/lab_cobot_bringup/test/test_lab_cobot_launch.py \
  -k original_bringup_passes_migrated_chassis_parameters
```

Expected: FAIL，缺少 `rover` 或 `mecanum3.wheel_radius`。

- [ ] **Step 3: 只给现有 relay 增加显式参数**

将 `rover_twist_relay` 的 parameters 改为：

```python
parameters=[{
    "use_sim_time": True,
    "rover": "mecanum3",
    "mecanum3.wheel_radius": 0.07,
    "mecanum3.wheel_separation_width": 0.24,
    "mecanum3.wheel_separation_length": 0.175,
    "max_vx": 0.5,
    "max_vy": 0.3,
    "max_wz": 1.2,
    "max_accel_xy": 0.5,
    "max_accel_wz": 1.5,
    "command_timeout": 0.25,
}],
```

不得改动 world、navigation、mission、MoveIt、perception 或 TimerAction。

- [ ] **Step 4: 运行 bringup launch 测试**

Run:

```bash
python3 -m pytest -q -p no:anyio \
  src/lab_cobot_bringup/test/test_lab_cobot_launch.py
```

Expected: 全部 PASS。

- [ ] **Step 5: 提交启动参数锁定**

```bash
git add src/lab_cobot_bringup/launch/lab_cobot.launch.py \
  src/lab_cobot_bringup/test/test_lab_cobot_launch.py
git commit -m "fix(bringup): lock migrated chassis motion parameters"
```

### Task 2: 拒绝异常 `/cmd_vel`，保持有限输入解算不变

**Files:**
- Modify: `src/lab_cobot_bringup/test/test_rover_twist_relay.py`
- Modify: `src/lab_cobot_bringup/lab_cobot_bringup/rover_twist_relay.py`

- [ ] **Step 1: 写失败的有限值测试**

```python
import math


def test_sanitize_twist_rejects_non_finite_components():
    assert sanitize_twist(SimpleTwist(math.nan, 0.0, 0.0)) == SimpleTwist()
    assert sanitize_twist(SimpleTwist(0.0, math.inf, 0.0)) == SimpleTwist()
    assert sanitize_twist(SimpleTwist(0.0, 0.0, -math.inf)) == SimpleTwist()


def test_sanitize_twist_preserves_finite_components():
    command = SimpleTwist(0.2, -0.1, 0.4)
    assert sanitize_twist(command) == command
```

- [ ] **Step 2: 运行测试确认函数不存在**

Run:

```bash
python3 -m pytest -q -p no:anyio \
  src/lab_cobot_bringup/test/test_rover_twist_relay.py \
  -k sanitize_twist
```

Expected: collection ERROR 或 FAIL，`sanitize_twist` 尚不存在。

- [ ] **Step 3: 实现最小有限值保护**

```python
import math


def sanitize_twist(twist):
    if not all(math.isfinite(value) for value in (twist.vx, twist.vy, twist.wz)):
        return SimpleTwist()
    return twist
```

在 `on_twist_received` 中仅增加这一层，再执行已有 `limit_twist` 和 `apply_deadband`。不得修改 `twist_to_wheel_speeds` 公式、轮序或符号。

- [ ] **Step 4: 运行 relay 全部测试**

Run:

```bash
python3 -m pytest -q -p no:anyio \
  src/lab_cobot_bringup/test/test_rover_twist_relay.py
```

Expected: 全部 PASS。

- [ ] **Step 5: 提交异常输入保护**

```bash
git add src/lab_cobot_bringup/lab_cobot_bringup/rover_twist_relay.py \
  src/lab_cobot_bringup/test/test_rover_twist_relay.py
git commit -m "fix(bringup): reject invalid chassis velocity commands"
```

### Task 3: 修正中文运行文档但保持原命令

**Files:**
- Modify: `README.md`

- [ ] **Step 1: 检查 README 中已经过时的底盘链路**

Run:

```bash
grep -n -E "mecanum_wheel_visualizer|lab_cobot_mecanum_drive|/rover_twist" README.md
```

Expected: 找到与当前默认实现不一致的旧节点名称。

- [ ] **Step 2: 更新系统流程和底盘调试说明**

将流程更新为：

```text
/task/instruction
  -> mission_node
  -> Nav2 AMCL/EKF + DWB
  -> /cmd_vel -> rover_twist_relay(移植的麦轮逆解)
  -> /wheel_velocity_controller/commands
  -> lab_cobot_planar_drive(同步正解+平面位姿积分)
  -> gazebo_odom_bridge -> /odom
```

保留原完整启动命令，并增加不启动任务节点的底盘调试示例：

```bash
ros2 launch lab_cobot_bringup lab_cobot.launch.py \
  launch_mission:=false use_rviz:=false

ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

明确 `world.launch.py` 单独启动时不包含 `/cmd_vel` 适配节点。

- [ ] **Step 3: 检查文档不存在旧默认链路**

Run:

```bash
grep -n -E "mecanum_wheel_visualizer|lab_cobot_mecanum_drive" README.md
```

Expected: 无输出。

- [ ] **Step 4: 提交中文文档**

```bash
git add README.md
git commit -m "docs: document migrated chassis in original workflow"
```

### Task 4: 构建、回归测试与原启动动态验收

**Files:**
- No persistent source files expected

- [ ] **Step 1: 构建受影响包**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  lab_cobot_description lab_cobot_gazebo lab_cobot_bringup
```

Expected: 3 packages finished，0 failed。

- [ ] **Step 2: 运行受影响包测试**

Run:

```bash
source install/setup.bash
colcon test --packages-select \
  lab_cobot_description lab_cobot_gazebo lab_cobot_bringup
colcon test-result --verbose
```

Expected: 新增和既有行为测试均通过；若仓库既有无关 lint 债务仍存在，单独列出且不得误报为本次回归。

- [ ] **Step 3: 使用原始完整启动入口启动无任务系统**

Run:

```bash
ros2 launch lab_cobot_bringup lab_cobot.launch.py \
  gui:=false use_rviz:=false launch_mission:=false \
  use_dl_perception:=false
```

Expected: Gazebo、控制器、relay、Nav2、MoveIt 和基础感知启动；不启动 mission 自动运动。

- [ ] **Step 4: 验证接口唯一性**

Run:

```bash
ros2 topic info /wheel_velocity_controller/commands -v
ros2 topic info /odom -v
ros2 control list_controllers
```

Expected: 轮速只有 `rover_twist_relay` 一个发布者；`/odom` 只有 `gazebo_odom_bridge` 一个发布者；四个控制器 active。

- [ ] **Step 5: 验证六类运动和超时停车**

用一次性 rclpy 脚本按 20 Hz 依次发布：

```python
commands = {
    "forward": (0.20, 0.00, 0.00),
    "backward": (-0.20, 0.00, 0.00),
    "left": (0.00, 0.15, 0.00),
    "right": (0.00, -0.15, 0.00),
    "diagonal": (0.15, 0.10, 0.00),
    "rotate": (0.00, 0.00, 0.50),
}
```

每项持续 1.5 秒并采样 `/gazebo/model_states`，要求主运动分量符号正确；每项结束停止发布 0.6 秒，要求位姿不再变化。

- [ ] **Step 6: 再次验证底盘稳定性**

连续采样至少 5 秒，要求：

```text
z span <= 1e-5 m
roll span <= 1e-5 rad
pitch span <= 1e-5 rad
```

- [ ] **Step 7: 检查工作树和最终提交记录**

Run:

```bash
git diff --check
git status -sb
git log -5 --oneline
```

Expected: 工作树干净，所有实现和文档均已提交。
