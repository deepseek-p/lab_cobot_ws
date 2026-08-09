// Helpers for classifying gripper finger collisions in Gazebo.
#ifndef LAB_COBOT_GAZEBO__FINGER_COLLISION_CONFIG_HPP_
#define LAB_COBOT_GAZEBO__FINGER_COLLISION_CONFIG_HPP_

#include <string>

namespace lab_cobot_gazebo
{

struct TactileProbeSurfaceSettings
{
  bool collide_without_contact;
  unsigned int collide_without_contact_bitmask;
  unsigned int collide_bits;
  unsigned int max_contacts;
};

struct PrimaryFingerCollisionSettings
{
  unsigned int collide_bits;
  unsigned int surface_collide_bitmask;
};

inline bool IsTactileProbeCollisionName(const std::string & name)
{
  return name.find("tactile_probe") != std::string::npos;
}

inline bool IsPrimaryFingerCollisionName(const std::string & name)
{
  if (IsTactileProbeCollisionName(name)) {
    return false;
  }
  return name.find("gripper_left_finger_collision") != std::string::npos ||
         name.find("gripper_right_finger_collision") != std::string::npos;
}

inline bool ShouldDisablePrimaryFingerCollisions(bool require_finger_contact)
{
  return require_finger_contact;
}

inline std::string SelectAttachLinkName(
  bool require_finger_contact,
  const std::string & finger_link_name,
  const std::string & stable_gripper_link_name)
{
  if (require_finger_contact && !stable_gripper_link_name.empty()) {
    return stable_gripper_link_name;
  }
  return finger_link_name;
}

inline PrimaryFingerCollisionSettings PrimaryFingerCollisionConfig()
{
  return {0u, 0u};
}

inline TactileProbeSurfaceSettings TactileProbeSurfaceConfig(
  const bool enable_contact_force = false)
{
  // 默认仅做接触判定，避免双 probe 的接触响应破坏稳定抓取。G4 采集时
  // 显式打开低时长的物理接触响应，使 Gazebo bumper 的 wrench 非零。
  if (enable_contact_force) {
    return {false, 0u, 0xffffu, 10u};
  }
  return {true, 0xffffu, 0xffffu, 10u};
}

}  // namespace lab_cobot_gazebo

#endif  // LAB_COBOT_GAZEBO__FINGER_COLLISION_CONFIG_HPP_
