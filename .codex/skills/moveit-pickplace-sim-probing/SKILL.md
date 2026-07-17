---
name: moveit-pickplace-sim-probing
description: "Use when a full mission is too slow or noisy and MoveIt PickPlace needs a minimal Gazebo probe: set base pose by /cmd_vel, compute object pose from /gazebo/model_states, call PickPlace.pick() directly, and record tactile contacts, contact status, and sample twist."
---

# MoveIt PickPlace sim probing

来源：2026-07 lab_cobot_ws Goal-T 调试。命令原文从 Codex 会话日志恢复。

## 1. 为什么不用完整 mission

- 症状：完整 `/task/instruction` 失败只给 `FAILED`，一次循环太慢，导航/视觉/抓取混在一起。
- 根因：Goal-T 的问题集中在 pick 阶段的 tactile close、contact gate 和样件物理速度。
- 修法：只启动 world、MoveGroup、底盘可视化/驱动，然后在一个 Python probe 中：
  - 订阅 `/gazebo/model_states`。
  - 用 `/cmd_vel` 把 base 驱到 A 台前固定姿态。
  - 从 `lab_cobot` 和 `aruco_sample` 的 model pose 计算 `base_link` 中的 object xyz。
  - 构造 `PickPlace(target_object='aruco_sample', use_tactile_grasp=True)`。
  - 直接调用 `pick.pick(obj)`。
  - 同步记录左右 bumper hit、`/gripper/contact/status`、样件最大线速度和最终 pose。

## 2. 前置启动

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch lab_cobot_gazebo world.launch.py gui:=false require_finger_contact:=true
ros2 launch lab_cobot_moveit move_group.launch.py use_sim_time:=true
ros2 run lab_cobot_bringup mecanum_wheel_visualizer --ros-args -p use_sim_time:=true -p publish_odom:=false
```

## 3. 成败字段

- `DRIVE_RESULT True`：底盘到达 `(2.0, 0.72, pi/2)` 附近。
- `OBJECT_BASE_LINK [...]`：传入 `PickPlace.pick()` 的目标。
- `PICK_RESULT True`：最小 pick 成功。
- `CONTACT_HITS L R`：左右都应 > 0，且碰撞名以 `aruco_sample::` 开头。
- `STATUSES [...]`：应包含 `attached aruco_sample`；失败时保留 `refused ...` 偏移。
- `MAX_SAMPLE_SPEED`：T-4 目标 `<1.0`；若 >10 m/s，可按物理爆炸处理。
- `FINAL_SAMPLE`：样件不能飞离 A 台或掉到异常位置。

## 4. 本次实际使用的探针命令原文

```bash
source /opt/ros/humble/setup.bash; source install/setup.bash; export ROS_LOCALHOST_ONLY=1; python3 - <<'PY'
import math, threading, time
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from gazebo_msgs.msg import ContactsState, ModelStates
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from lab_cobot_manipulation.pick_place_node import PickPlace
BASE_LINK_WORLD_Z = 0.155
TARGET_BASE = (2.0, 0.72, math.pi / 2.0)
def yaw_from_quat(q):
    return math.atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z))
def norm_angle(a): return math.atan2(math.sin(a), math.cos(a))
def touches_target(msg):
    return any(str(s.collision1_name).startswith('aruco_sample::') or str(s.collision2_name).startswith('aruco_sample::') for s in msg.states)
class Probe(Node):
    def __init__(self):
        super().__init__('tactile_pick_probe')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.latest_models = None; self.left_hits = 0; self.right_hits = 0; self.statuses = []
        self.max_sample_speed = 0.0; self.track_speed = False
        self.create_subscription(ModelStates, '/gazebo/model_states', self._models_cb, 10)
        self.create_subscription(ContactsState, '/gripper/left_finger_contacts', self._left_cb, 10)
        self.create_subscription(ContactsState, '/gripper/right_finger_contacts', self._right_cb, 10)
        self.create_subscription(String, '/gripper/contact/status', self._status_cb, 10)
    def _models_cb(self, msg):
        self.latest_models = msg
        if self.track_speed and 'aruco_sample' in msg.name:
            i = msg.name.index('aruco_sample'); t = msg.twist[i]
            self.max_sample_speed = max(self.max_sample_speed, math.sqrt(t.linear.x*t.linear.x+t.linear.y*t.linear.y+t.linear.z*t.linear.z))
    def _left_cb(self, msg):
        if touches_target(msg): self.left_hits += 1
    def _right_cb(self, msg):
        if touches_target(msg): self.right_hits += 1
    def _status_cb(self, msg): self.statuses.append(str(msg.data))
    def model_pose(self, name):
        msg = self.latest_models
        if msg is None or name not in msg.name: return None
        i = msg.name.index(name); p = msg.pose[i].position
        return (p.x, p.y, p.z, yaw_from_quat(msg.pose[i].orientation))
    def publish_cmd(self, vx, vy, wz):
        msg = Twist(); msg.linear.x = float(vx); msg.linear.y = float(vy); msg.angular.z = float(wz); self.cmd_pub.publish(msg)
    def stop(self, seconds=0.5):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            self.publish_cmd(0,0,0); time.sleep(0.05)
    def drive_to_target(self, timeout=80.0):
        tx, ty, tyaw = TARGET_BASE; start = time.monotonic()
        while time.monotonic()-start < timeout:
            pose = self.model_pose('lab_cobot')
            if pose is None: time.sleep(0.05); continue
            x,y,_z,yaw = pose; ex = tx-x; ey = ty-y; eyaw = norm_angle(tyaw-yaw)
            if math.hypot(ex,ey) < 0.015 and abs(eyaw) < 0.025:
                self.stop(); return True
            vx_w = max(-0.28, min(0.28, 0.9*ex)); vy_w = max(-0.28, min(0.28, 0.9*ey))
            c = math.cos(yaw); s = math.sin(yaw)
            self.publish_cmd(c*vx_w+s*vy_w, -s*vx_w+c*vy_w, max(-0.55, min(0.55, 1.2*eyaw)))
            time.sleep(0.05)
        self.stop(); return False
    def sample_in_base_link(self):
        base = self.model_pose('lab_cobot'); sample = self.model_pose('aruco_sample')
        if base is None or sample is None: return None
        bx,by,_bz,byaw = base; sx,sy,sz,_ = sample; dx = sx-bx; dy = sy-by; c = math.cos(byaw); s = math.sin(byaw)
        return [c*dx+s*dy, -s*dx+c*dy, sz-BASE_LINK_WORLD_Z]
rclpy.init(); probe = Probe(); pick = PickPlace(target_object='aruco_sample', use_tactile_grasp=True)
executor = MultiThreadedExecutor(num_threads=4); executor.add_node(probe); executor.add_node(pick)
threading.Thread(target=executor.spin, daemon=True).start()
try:
    deadline = time.monotonic()+10
    while time.monotonic()<deadline and (probe.model_pose('lab_cobot') is None or probe.model_pose('aruco_sample') is None): time.sleep(0.1)
    print('INITIAL_BASE', probe.model_pose('lab_cobot'), flush=True); print('INITIAL_SAMPLE', probe.model_pose('aruco_sample'), flush=True)
    drive_ok = probe.drive_to_target(); print('DRIVE_RESULT', drive_ok, 'BASE', probe.model_pose('lab_cobot'), flush=True)
    time.sleep(1.0); obj = probe.sample_in_base_link(); print('OBJECT_BASE_LINK', obj, flush=True)
    if not drive_ok or obj is None: raise SystemExit(2)
    probe.left_hits = probe.right_hits = 0; probe.statuses.clear(); probe.max_sample_speed = 0.0; probe.track_speed = True
    result = pick.pick(obj); time.sleep(2.0); probe.track_speed = False
    print('PICK_RESULT', result, flush=True); print('CONTACT_HITS', probe.left_hits, probe.right_hits, flush=True)
    print('STATUSES', probe.statuses, flush=True); print('MAX_SAMPLE_SPEED', '%.6f' % probe.max_sample_speed, flush=True)
    print('FINAL_BASE', probe.model_pose('lab_cobot'), flush=True); print('FINAL_SAMPLE', probe.model_pose('aruco_sample'), flush=True)
finally:
    probe.stop(); executor.shutdown(); pick.destroy_node(); probe.destroy_node(); rclpy.shutdown()
PY
```

## 5. 本次失败样本

- `PICK_RESULT False`
- `CONTACT_HITS 0 73`
- `STATUSES ['refused aruco_sample offset=(0.080,-0.146,-0.014)', ...]`
- `MAX_SAMPLE_SPEED 9.612864`
- `FINAL_SAMPLE (-2.496..., -3.158..., 0.0348..., ...)`

解释：右侧接触存在但样件被推飞；后续应回到 contact/collision 分离排查，而不是只调 MoveIt 目标。
