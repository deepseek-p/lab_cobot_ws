// 附着期样件碰撞响应生命周期的纯函数单元测试。
#include <gtest/gtest.h>

#include "lab_cobot_gazebo/attached_object_collision.hpp"

namespace
{

using lab_cobot_gazebo::AttachedObjectCollisionPhase;
using lab_cobot_gazebo::ObjectCollisionResponseForPhase;

TEST(AttachedObjectCollision, GraspGateKeepsCollisionResponseEnabled)
{
  const auto settings = ObjectCollisionResponseForPhase(
    AttachedObjectCollisionPhase::kGraspGate);

  EXPECT_EQ(settings.collide_bits, 0xffffu);
  EXPECT_EQ(settings.surface_collide_bitmask, 0xffffu);
}

TEST(AttachedObjectCollision, FixedJointHoldDisablesCollisionResponse)
{
  const auto settings = ObjectCollisionResponseForPhase(
    AttachedObjectCollisionPhase::kFixedJointHeld);

  EXPECT_EQ(settings.collide_bits, 0u);
  EXPECT_EQ(settings.surface_collide_bitmask, 0u);
}

TEST(AttachedObjectCollision, ReleaseRestoresCollisionResponse)
{
  const auto settings = ObjectCollisionResponseForPhase(
    AttachedObjectCollisionPhase::kReleased);

  EXPECT_EQ(settings.collide_bits, 0xffffu);
  EXPECT_EQ(settings.surface_collide_bitmask, 0xffffu);
}

}  // namespace
