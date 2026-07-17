---
name: gazebo-contact-sensors-and-collisions
description: Use when Gazebo Classic ROS 2 contact sensors or bumper topics silently publish no hits, collision names differ after URDF to SDF conversion, tactile probes conflict with primary gripper collisions, SetNeverDropContacts is needed, or light objects are pushed/launched by position-controlled finger contacts.
---

# Gazebo contact sensors and collisions

来源：2026-07 lab_cobot_ws Goal-T 实测。未完成 SIM 复核的项显式标注“未验证”。

## 1. Contact sensor 零命中

- 症状：`/gripper/left_finger_contacts` 或 `/gripper/right_finger_contacts` 有心跳但 `states` 始终为空；`CONTACT_HITS 0 0`。
- 根因：`<contact><collision>...</collision></contact>` 写的不是 Gazebo SDF 里的 collision 名。URDF/xacro 名会被转换，写错时 Gazebo 不报错，只是静默零匹配。
- 修法：先展开并检查 SDF，再把 sensor 的 `<collision>` 写成转换后的实际名。
  ```bash
  source /opt/ros/humble/setup.bash
  xacro src/lab_cobot_description/urdf/lab_cobot.urdf.xacro > /tmp/lab_cobot.urdf
  gz sdf -p /tmp/lab_cobot.urdf | rg 'gripper_.*collision|tactile_probe'
  ```
- 本项目当前实装：contact sensor 指向 tactile probe collision：
  - `gripper_left_finger_tactile_probe_collision_1`
  - `gripper_right_finger_tactile_probe_collision_1`
- 历史规格里用过 `gripper_left_finger_collision` 这类主碰撞名；不要照抄旧名，必须以当前 `gz sdf -p` 输出为准。

## 2. probe collision 与 primary collision 分工

- 症状：触觉闭合期间 bumper 有接触或单侧接触，但样件被指尖推走，`/gazebo/model_states` 速度超过 1 m/s，甚至被弹飞。
- 根因：position 接口的 primary finger collision 会以无限刚度挤压轻物体；tactile probe 应只负责接触检测，不应让主碰撞继续参与推挤。
- 修法：把两类 collision 分开配置：
  - tactile probe：保留接触检测，设置 contact-only。
  - primary finger collision：禁用碰撞 bit，避免位置控制主碰撞推物体。
- 当前代码形态：
  - `IsTactileProbeCollisionName(name)`：包含 `tactile_probe`。
  - `IsPrimaryFingerCollisionName(name)`：匹配 `gripper_left_finger_collision` / `gripper_right_finger_collision`，但排除 tactile probe。
  - `ConfigureTactileProbeCollisions()`：`surface->collideWithoutContact = true`、`collideWithoutContactBitmask = 0xffff`、`SetMaxContacts(10)`。
  - `ConfigurePrimaryFingerCollisions()`：`collision->SetCollideBits(0)`。
- 未验证：上述“禁 primary、保 probe”的最终 SIM 成功尚未完成；当时只完成了 build/gtest，T-4 仍需复跑确认。

## 3. 两个实测失败案例

- 症状：禁碰撞后 `CONTACT_HITS 0 0`。
- 根因：把 tactile probe 的碰撞也禁掉了，bumper 没有可匹配的 collision 参与 contact generation。
- 修法：不要禁 tactile probe；只禁 primary finger collision。

- 症状：保留 probe 后仍失败，`CONTACT_HITS 0 73`，`MAX_SAMPLE_SPEED 9.612864`，状态里出现 `refused aruco_sample offset=...`，样件最终被推离桌面。
- 根因：probe 能产生接触，但 primary finger collision 仍在被 position controller 推进，样件被机械挤开。
- 修法：按第 2 节分离 probe/primary；修完必须用 SIM 重新看 `CONTACT_HITS`、`MAX_SAMPLE_SPEED` 和最终样件位姿。

## 4. 验证 bumper topic 真有目标接触

- 症状：topic 在发，但无法判断是否真的碰到目标物体。
- 根因：Gazebo bumper 空消息也会按 update rate 发布；topic 有频率不等于有目标接触。
- 修法：检查 `ContactsState.states[*].collision{1,2}_name` 是否包含目标模型前缀。
  ```bash
  ros2 topic echo --once /gripper/left_finger_contacts
  ros2 topic echo --once /gripper/right_finger_contacts
  ```
- 判断标准：PICK 期间左右两侧都至少出现一次 `aruco_sample::`。
- 自动探针里使用过的过滤：
  ```python
  any(
      s.collision1_name.startswith("aruco_sample::")
      or s.collision2_name.startswith("aruco_sample::")
      for s in msg.states
  )
  ```

## 5. SetNeverDropContacts 的位置

- 症状：contact gate 永远不满足，插件只能报 `refused ... no_finger_contact` 或等待超时。
- 根因：Gazebo Classic ContactManager 在无订阅者/消费者时可能丢 contact；门控在插件内部读 contact buffer，必须让 contact manager 不丢弃。
- 修法：在 Gazebo plugin `Load()` 中拿到 `world_->Physics()->GetContactManager()` 后立即调用：
  ```cpp
  auto contact_manager = world_->Physics()->GetContactManager();
  if (contact_manager) {
    contact_manager->SetNeverDropContacts(true);
  }
  ```
- 时机：在 `ConnectWorldUpdateBegin` 前完成；否则 `OnUpdate()` 里的 contact gate 会看不到稳定接触。
