#include <cmath>

#include <gtest/gtest.h>

#include "lab_cobot_gazebo/runtime_motion.hpp"

using lab_cobot_gazebo::rotateBaseToWorld;
using lab_cobot_gazebo::rotateWorldToBase;
using lab_cobot_gazebo::wheelSpeedsToTwist;

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

TEST(RuntimeMotion, InvertsConfirmedRelayWheelOrderAndSigns)
{
  // Relay output order is [-FL, -FR, -BL, -BR].  This vector is the
  // exact relay result for vx=0.28, vy=-0.14, wz=0.6 with r=.07,
  // width=.24 and length=.175.
  const auto twist = wheelSpeedsToTwist(
    {-2.442857142857143, -5.557142857142857,
      1.557142857142857, -9.557142857142857},
    0.07, 0.24, 0.175);
  EXPECT_NEAR(twist.vx, 0.28, 1e-12);
  EXPECT_NEAR(twist.vy, -0.14, 1e-12);
  EXPECT_NEAR(twist.wz, 0.6, 1e-12);
}

TEST(RuntimeMotion, PureAxesRespectRelayConvention)
{
  const auto forward = wheelSpeedsToTwist({-2.0, -2.0, -2.0, -2.0});
  EXPECT_NEAR(forward.vx, 0.14, 1e-12);
  EXPECT_NEAR(forward.vy, 0.0, 1e-12);
  EXPECT_NEAR(forward.wz, 0.0, 1e-12);

  const auto lateral = wheelSpeedsToTwist({2.0, -2.0, -2.0, 2.0});
  EXPECT_NEAR(lateral.vx, 0.0, 1e-12);
  EXPECT_NEAR(lateral.vy, 0.14, 1e-12);
  EXPECT_NEAR(lateral.wz, 0.0, 1e-12);
}
