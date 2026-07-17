// fixed-joint 持有期样件碰撞响应生命周期的纯函数配置。
#ifndef LAB_COBOT_GAZEBO__ATTACHED_OBJECT_COLLISION_HPP_
#define LAB_COBOT_GAZEBO__ATTACHED_OBJECT_COLLISION_HPP_

namespace lab_cobot_gazebo
{

enum class AttachedObjectCollisionPhase
{
  kGraspGate,
  kFixedJointHeld,
  kReleased,
};

struct ObjectCollisionResponseSettings
{
  unsigned int collide_bits;
  unsigned int surface_collide_bitmask;
};

inline ObjectCollisionResponseSettings ObjectCollisionResponseForPhase(
  AttachedObjectCollisionPhase phase)
{
  if (phase == AttachedObjectCollisionPhase::kFixedJointHeld) {
    return {0u, 0u};
  }
  return {0xffffu, 0xffffu};
}

}  // namespace lab_cobot_gazebo

#endif  // LAB_COBOT_GAZEBO__ATTACHED_OBJECT_COLLISION_HPP_
