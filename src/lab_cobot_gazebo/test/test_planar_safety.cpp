#include <cmath>

#include <gtest/gtest.h>

#include "lab_cobot_gazebo/planar_safety.hpp"

namespace safety = lab_cobot_gazebo::planar_safety;

namespace
{
constexpr double kPi = 3.14159265358979323846;
const safety::AxisAlignedBox kStationA{1.6, 2.4, 1.2, 1.8};
const safety::AxisAlignedBox kStationB{-2.4, -1.6, 1.2, 1.8};
constexpr double kMargin = 0.35;

safety::OrientedBox chassis(double x, double y, double yaw = 0.0)
{
  return {{x, y}, yaw, 0.42, 0.30};
}
}  // namespace

TEST(PlanarSafety, ValidatesFinitePositiveGeometryAndMargin)
{
  EXPECT_TRUE(safety::isValid(chassis(0.0, 0.0)));
  EXPECT_FALSE(safety::isValid({{0.0, 0.0}, 0.0, 0.0, 0.30}));
  EXPECT_FALSE(safety::isValid({{NAN, 0.0}, 0.0, 0.42, 0.30}));
  EXPECT_TRUE(safety::isValid(kStationA));
  EXPECT_FALSE(safety::isValid(safety::AxisAlignedBox{2.4, 1.6, 1.2, 1.8}));
  EXPECT_TRUE(safety::isValidMargin(kMargin));
  EXPECT_FALSE(safety::isValidMargin(-0.01));
}

TEST(PlanarSafety, AllowsOutsideAndBlocksSafetyBoundaryAtBothStations)
{
  EXPECT_FALSE(safety::intersects(chassis(2.0, 0.49), kStationA, kMargin));
  EXPECT_TRUE(safety::intersects(chassis(2.0, 0.70), kStationA, kMargin));
  EXPECT_TRUE(safety::intersects(chassis(-2.0, 0.70), kStationB, kMargin));
  EXPECT_FALSE(safety::intersects(chassis(-2.0, 0.49), kStationB, kMargin));
}

TEST(PlanarSafety, RotationCanMoveAChassisCornerIntoTheSafetyZone)
{
  const auto unrotated = chassis(2.0, 0.665, 0.0);
  const auto rotated = chassis(2.0, 0.665, kPi / 4.0);
  EXPECT_FALSE(safety::intersects(unrotated, kStationA, kMargin));
  EXPECT_TRUE(safety::intersects(rotated, kStationA, kMargin));
}

TEST(PlanarSafety, UnsafeChassisMayReduceOverlapButMayNotDeepenIt)
{
  const auto current = chassis(2.0, 0.72);
  const auto leaving = chassis(2.0, 0.71);
  const auto entering = chassis(2.0, 0.73);
  EXPECT_TRUE(safety::intersects(current, kStationA, kMargin));
  EXPECT_GT(
    safety::signedSeparation(leaving, safety::expanded(kStationA, kMargin)),
    safety::signedSeparation(current, safety::expanded(kStationA, kMargin)));
  EXPECT_TRUE(safety::isMotionAllowed(current, leaving, {kStationA, kStationB}, kMargin));
  EXPECT_FALSE(safety::isMotionAllowed(current, entering, {kStationA, kStationB}, kMargin));
}

TEST(PlanarSafety, SafeChassisMayNotEnterButCanMoveFreelyOutside)
{
  EXPECT_FALSE(
    safety::isMotionAllowed(
      chassis(2.0, 0.49), chassis(2.0, 0.70), {kStationA, kStationB}, kMargin));
  EXPECT_TRUE(
    safety::isMotionAllowed(
      chassis(0.0, 0.0), chassis(0.1, 0.0), {kStationA, kStationB}, kMargin));
}

TEST(PlanarSafety, SweptTranslationCannotCrossEitherStationWithSafeEndpoints)
{
  EXPECT_TRUE(
    safety::isMotionAllowed(
      chassis(2.0, 0.0), chassis(2.0, 3.0), {kStationA, kStationB}, kMargin));
  EXPECT_FALSE(
    safety::isSweptMotionAllowed(
      chassis(2.0, 0.0), chassis(2.0, 3.0), {kStationA, kStationB}, kMargin));
  EXPECT_FALSE(
    safety::isSweptMotionAllowed(
      chassis(-2.0, 0.0), chassis(-2.0, 3.0), {kStationA, kStationB}, kMargin));
}

TEST(PlanarSafety, SweptRotationCannotPassAnUnsafeIntermediateAngle)
{
  const auto start = chassis(2.0, 0.63, -kPi / 2.0);
  const auto finish = chassis(2.0, 0.63, kPi / 2.0);
  EXPECT_FALSE(safety::intersects(start, kStationA, kMargin));
  EXPECT_FALSE(safety::intersects(finish, kStationA, kMargin));
  EXPECT_FALSE(
    safety::isSweptMotionAllowed(
      start, finish, {kStationA, kStationB}, kMargin));
}

TEST(PlanarSafety, ConservativeSegmentRejectsCornerContactBetweenSafeSamples)
{
  const auto start = chassis(2.0, 0.63, -kPi / 2.0);
  const auto finish = chassis(2.0, 0.63, kPi / 2.0);
  EXPECT_FALSE(safety::intersects(start, kStationA, kMargin));
  EXPECT_FALSE(safety::intersects(finish, kStationA, kMargin));
  // A one-segment discrete endpoint check misses the unsafe +/-45 degree arc.
  EXPECT_FALSE(
    safety::isSweptMotionAllowed(
      start, finish, {kStationA, kStationB}, kMargin, 1.0));
}

TEST(PlanarSafety, SweptExitRequiresEveryUnsafeSubstepToImprove)
{
  EXPECT_TRUE(
    safety::isSweptMotionAllowed(
      chassis(2.0, 0.72), chassis(2.0, 0.60), {kStationA, kStationB}, kMargin));
  EXPECT_FALSE(
    safety::isSweptMotionAllowed(
      chassis(2.0, 0.72), chassis(2.0, 0.74), {kStationA, kStationB}, kMargin));
}
