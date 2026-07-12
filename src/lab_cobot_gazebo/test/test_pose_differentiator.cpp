#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "lab_cobot_gazebo/pose_differentiator.hpp"

namespace
{
constexpr double kTolerance = 1e-9;
using lab_cobot_gazebo::PoseDifferentiator;

TEST(PoseDifferentiator, ReportsStraightMotionInBaseFrame)
{
  PoseDifferentiator differentiator(0.5);
  EXPECT_FALSE(differentiator.update(1.0, 2.0, 0.0, 10.0).valid);
  const auto velocity = differentiator.update(1.2, 2.0, 0.0, 10.2);
  EXPECT_TRUE(velocity.valid);
  EXPECT_NEAR(velocity.vx, 1.0, kTolerance);
  EXPECT_NEAR(velocity.vy, 0.0, kTolerance);
  EXPECT_NEAR(velocity.wz, 0.0, kTolerance);
}

TEST(PoseDifferentiator, RotatesWorldDeltaIntoCurrentBaseFrame)
{
  PoseDifferentiator differentiator(0.5);
  differentiator.update(0.0, 0.0, M_PI_2, 1.0);
  const auto velocity = differentiator.update(0.0, 0.2, M_PI_2, 1.2);
  EXPECT_TRUE(velocity.valid);
  EXPECT_NEAR(velocity.vx, 1.0, kTolerance);
  EXPECT_NEAR(velocity.vy, 0.0, kTolerance);
}

TEST(PoseDifferentiator, WrapsYawAcrossPi)
{
  PoseDifferentiator differentiator(0.5);
  differentiator.update(0.0, 0.0, M_PI - 0.1, 1.0);
  const auto velocity = differentiator.update(0.0, 0.0, -M_PI + 0.1, 1.2);
  EXPECT_TRUE(velocity.valid);
  EXPECT_NEAR(velocity.wz, 1.0, kTolerance);
}

TEST(PoseDifferentiator, UsesIntervalMidpointYawForForwardArc)
{
  PoseDifferentiator differentiator(0.5);
  differentiator.update(0.0, 0.0, 0.0, 1.0);
  const auto velocity = differentiator.update(
    std::sin(0.2), 1.0 - std::cos(0.2), 0.2, 1.2);
  EXPECT_TRUE(velocity.valid);
  EXPECT_NEAR(velocity.vx, 2.0 * std::sin(0.1) / 0.2, kTolerance);
  EXPECT_NEAR(velocity.vy, 0.0, kTolerance);
}

TEST(PoseDifferentiator, MidpointYawStaysCorrectAcrossPiWrap)
{
  PoseDifferentiator differentiator(0.5);
  differentiator.update(0.0, 0.0, M_PI - 0.1, 1.0);
  const auto velocity = differentiator.update(-0.2, 0.0, -M_PI + 0.1, 1.2);
  EXPECT_TRUE(velocity.valid);
  EXPECT_NEAR(velocity.vx, 1.0, kTolerance);
  EXPECT_NEAR(velocity.vy, 0.0, kTolerance);
}

TEST(PoseDifferentiator, RejectsInvalidMaximumDt)
{
  EXPECT_THROW(PoseDifferentiator(0.0), std::invalid_argument);
  EXPECT_THROW(PoseDifferentiator(-0.1), std::invalid_argument);
  EXPECT_THROW(PoseDifferentiator(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
  EXPECT_THROW(PoseDifferentiator(std::numeric_limits<double>::infinity()), std::invalid_argument);
}

TEST(PoseDifferentiator, InvalidatesPausedTimeAndResetsBaseline)
{
  PoseDifferentiator differentiator(0.5);
  differentiator.update(0.0, 0.0, 0.0, 1.0);
  const auto paused = differentiator.update(1.0, 0.0, 0.0, 1.0);
  EXPECT_FALSE(paused.valid);
  EXPECT_DOUBLE_EQ(paused.vx, 0.0);
  const auto resumed = differentiator.update(1.1, 0.0, 0.0, 1.1);
  EXPECT_TRUE(resumed.valid);
  EXPECT_NEAR(resumed.vx, 1.0, kTolerance);
}

TEST(PoseDifferentiator, InvalidatesClockRollbackAndResetsBaseline)
{
  PoseDifferentiator differentiator(0.5);
  differentiator.update(0.0, 0.0, 0.0, 2.0);
  const auto rollback = differentiator.update(1.0, 0.0, 0.0, 1.0);
  EXPECT_FALSE(rollback.valid);
  EXPECT_DOUBLE_EQ(rollback.vx, 0.0);
  const auto resumed = differentiator.update(1.1, 0.0, 0.0, 1.1);
  EXPECT_TRUE(resumed.valid);
  EXPECT_NEAR(resumed.vx, 1.0, kTolerance);
}

TEST(PoseDifferentiator, InvalidatesLargeGapAndResetsBaseline)
{
  PoseDifferentiator differentiator(0.25);
  differentiator.update(0.0, 0.0, 0.0, 1.0);
  const auto gap = differentiator.update(2.0, 0.0, 0.0, 2.0);
  EXPECT_FALSE(gap.valid);
  EXPECT_DOUBLE_EQ(gap.vx, 0.0);
  const auto resumed = differentiator.update(2.1, 0.0, 0.0, 2.1);
  EXPECT_TRUE(resumed.valid);
  EXPECT_NEAR(resumed.vx, 1.0, kTolerance);
}
}  // namespace
