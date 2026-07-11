#include <cmath>

#include <gtest/gtest.h>

#include "lab_cobot_gazebo/runtime_motion.hpp"

using lab_cobot_gazebo::rotateBaseToWorld;
using lab_cobot_gazebo::rotateWorldToBase;

TEST(RuntimeMotion, BaseVelocityIsRotatedIntoWorldFrame)
{
  const auto world = rotateBaseToWorld(1.0, 0.0, M_PI_2);
  EXPECT_NEAR(world.x, 0.0, 1e-12);
  EXPECT_NEAR(world.y, 1.0, 1e-12);
}

TEST(RuntimeMotion, WorldVelocityIsRotatedBackIntoBaseFrame)
{
  const auto base = rotateWorldToBase(0.0, 1.0, M_PI_2);
  EXPECT_NEAR(base.x, 1.0, 1e-12);
  EXPECT_NEAR(base.y, 0.0, 1e-12);
}

TEST(RuntimeMotion, OppositeTransformsRoundTrip)
{
  const auto world = rotateBaseToWorld(0.4, -0.2, 0.73);
  const auto base = rotateWorldToBase(world.x, world.y, 0.73);
  EXPECT_NEAR(base.x, 0.4, 1e-12);
  EXPECT_NEAR(base.y, -0.2, 1e-12);
}
