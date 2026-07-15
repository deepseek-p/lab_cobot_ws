// 抓取几何封套纯函数单元测试(真行为测试,替代原源码文本断言)。
#include <gtest/gtest.h>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include <limits>
#include <vector>

#include "lab_cobot_gazebo/canonical_attach.hpp"
#include "lab_cobot_gazebo/grasp_envelope.hpp"

namespace
{

using ignition::math::Quaterniond;
using ignition::math::Pose3d;
using ignition::math::Vector3d;
using lab_cobot_gazebo::AttachCorrectionAllowed;
using lab_cobot_gazebo::CanonicalAttachedObjectPose;
using lab_cobot_gazebo::PoseAtParentLocalZ;
using lab_cobot_gazebo::GraspEnvelopeLimits;
using lab_cobot_gazebo::GraspFramePosition;
using lab_cobot_gazebo::NearestOffsetIndexInsideEnvelope;
using lab_cobot_gazebo::ObjectOffsetInGraspFrame;
using lab_cobot_gazebo::OffsetInsideGraspEnvelope;

// URDF 实装限值(lab_cobot.urdf.xacro 的插件参数)
GraspEnvelopeLimits UrdfLimits()
{
  return {0.090, 0.065, 0.055, -0.060, 0.085};
}

TEST(GraspEnvelope, FramePositionIsFingerMidpointPlusRotatedOffset)
{
  const Vector3d left(1.0, 0.1, 0.5);
  const Vector3d right(1.0, -0.1, 0.5);
  // 绕 z 转 90°: offset (0.01, 0, 0.03) 旋成 (0, 0.01, 0.03)
  const Quaterniond rot(0.0, 0.0, M_PI / 2.0);
  const auto pos = GraspFramePosition(left, right, rot, {0.01, 0.0, 0.03});
  EXPECT_NEAR(pos.X(), 1.0, 1e-12);
  EXPECT_NEAR(pos.Y(), 0.01, 1e-12);
  EXPECT_NEAR(pos.Z(), 0.53, 1e-12);
}

TEST(GraspEnvelope, ObjectOffsetIsExpressedInGraspFrame)
{
  // 抓取系绕 z 转 90°,物块在世界系 +y 方向 0.02 -> 抓取系 +x 0.02
  const Quaterniond rot(0.0, 0.0, M_PI / 2.0);
  const auto offset = ObjectOffsetInGraspFrame(
    {1.0, 0.52, 0.5}, {1.0, 0.5, 0.5}, rot);
  EXPECT_NEAR(offset.X(), 0.02, 1e-12);
  EXPECT_NEAR(offset.Y(), 0.0, 1e-12);
  EXPECT_NEAR(offset.Z(), 0.0, 1e-12);
}

TEST(GraspEnvelope, AcceptsOffsetInsideAllBounds)
{
  EXPECT_TRUE(OffsetInsideGraspEnvelope({0.0, 0.0, 0.0}, UrdfLimits()));
  EXPECT_TRUE(OffsetInsideGraspEnvelope({0.02, 0.01, -0.048}, UrdfLimits()));
  EXPECT_TRUE(OffsetInsideGraspEnvelope({0.061, 0.052, 0.021}, UrdfLimits()));
}

TEST(GraspEnvelope, RejectsPerAxisBoundaryViolations)
{
  const auto limits = UrdfLimits();
  EXPECT_FALSE(OffsetInsideGraspEnvelope({0.066, 0.0, 0.0}, limits));   // x 超界
  EXPECT_FALSE(OffsetInsideGraspEnvelope({0.0, 0.056, 0.0}, limits));   // y 超界
  EXPECT_FALSE(OffsetInsideGraspEnvelope({0.0, 0.0, -0.061}, limits));  // z 下界
  EXPECT_FALSE(OffsetInsideGraspEnvelope({0.0, 0.0, 0.086}, limits));   // z 上界
}

TEST(GraspEnvelope, RejectsWhenTotalDistanceExceedsLimitEvenIfAxesPass)
{
  // 各轴分量均在界内,但合成距离超 max_center_distance -> 拒绝
  const auto limits = UrdfLimits();
  const Vector3d offset(0.058, 0.043, 0.072);
  ASSERT_LE(std::abs(offset.X()), limits.max_abs_x);
  ASSERT_LE(std::abs(offset.Y()), limits.max_abs_y);
  ASSERT_LE(offset.Z(), limits.max_z);
  ASSERT_GT(offset.Length(), limits.max_center_distance);
  EXPECT_FALSE(OffsetInsideGraspEnvelope(offset, limits));
}

TEST(GraspEnvelope, BoundaryValuesAreInclusive)
{
  const auto limits = UrdfLimits();
  EXPECT_TRUE(OffsetInsideGraspEnvelope({0.060, 0.0, 0.0}, limits));
  EXPECT_TRUE(OffsetInsideGraspEnvelope({0.065, 0.0, 0.0}, limits));
  EXPECT_TRUE(OffsetInsideGraspEnvelope({0.0, 0.055, 0.0}, limits));
  EXPECT_TRUE(OffsetInsideGraspEnvelope({0.0, 0.0, 0.085}, limits));
  EXPECT_TRUE(OffsetInsideGraspEnvelope({0.0, 0.0, -0.060}, limits));
}

TEST(GraspEnvelope, NearestOffsetChoosesClosestInsideCandidate)
{
  const std::vector<Vector3d> offsets = {
    {0.040, 0.0, 0.0},
    {0.010, 0.0, 0.0},
    {0.020, 0.0, 0.0},
  };

  EXPECT_EQ(NearestOffsetIndexInsideEnvelope(offsets, UrdfLimits()), 1);
}

TEST(GraspEnvelope, NearestOffsetSkipsOutOfEnvelopeCandidates)
{
  const std::vector<Vector3d> offsets = {
    {0.070, 0.0, 0.0},
    {0.025, 0.0, 0.0},
  };

  EXPECT_EQ(NearestOffsetIndexInsideEnvelope(offsets, UrdfLimits()), 1);
}

TEST(GraspEnvelope, NearestOffsetReturnsMinusOneWhenNoCandidateMatches)
{
  const std::vector<Vector3d> offsets = {
    {0.070, 0.0, 0.0},
    {0.0, 0.056, 0.0},
  };

  EXPECT_EQ(NearestOffsetIndexInsideEnvelope(offsets, UrdfLimits()), -1);
}

TEST(GraspEnvelope, NearestOffsetReturnsMinusOneForEmptyCandidateList)
{
  EXPECT_EQ(NearestOffsetIndexInsideEnvelope({}, UrdfLimits()), -1);
}

TEST(GraspEnvelope, NearestOffsetKeepsLowerIndexOnTie)
{
  const std::vector<Vector3d> offsets = {
    {0.010, 0.0, 0.0},
    {-0.010, 0.0, 0.0},
  };

  EXPECT_EQ(NearestOffsetIndexInsideEnvelope(offsets, UrdfLimits()), 0);
}

TEST(GraspEnvelope, CanonicalAttachUsesTcpLocalPositiveZAndKeepsObjectRotation)
{
  const Quaterniond down(M_PI, 0.0, 0.0);
  const Quaterniond object_rotation(0.0, 0.0, 0.37);
  const Pose3d tcp_pose(2.0, 1.32, 0.82983, down.Roll(), down.Pitch(), down.Yaw());
  const auto pose = CanonicalAttachedObjectPose(tcp_pose, 0.045, object_rotation);
  EXPECT_NEAR(pose.Pos().X(), 2.0, 1e-9);
  EXPECT_NEAR(pose.Pos().Y(), 1.32, 1e-9);
  EXPECT_NEAR(pose.Pos().Z(), 0.78483, 1e-9);
  EXPECT_NEAR(pose.Rot().Yaw(), object_rotation.Yaw(), 1e-9);
  EXPECT_LT(pose.Pos().Distance({2.0, 1.32, 0.78483}), 0.005);
}

TEST(GraspEnvelope, LumpedTcpUsesTool0FixedTransformBeforeObjectOffset)
{
  const Quaterniond wrist_to_flange(0.0, -M_PI / 2.0, -M_PI / 2.0);
  const Quaterniond flange_to_tool0(M_PI / 2.0, 0.0, M_PI / 2.0);
  const auto wrist_to_tool0 = wrist_to_flange * flange_to_tool0;
  EXPECT_NEAR(wrist_to_tool0.Roll(), 0.0, 1e-9);
  EXPECT_NEAR(wrist_to_tool0.Pitch(), 0.0, 1e-9);
  EXPECT_NEAR(wrist_to_tool0.Yaw(), 0.0, 1e-9);

  const Quaterniond down(M_PI, 0.0, 0.0);
  const Pose3d wrist3(2.0, 1.32, 0.93483, down.Roll(), down.Pitch(), down.Yaw());
  const auto tool0 = wrist3 * Pose3d(0.0, 0.0, 0.0,
    wrist_to_tool0.Roll(), wrist_to_tool0.Pitch(), wrist_to_tool0.Yaw());
  const auto tcp = PoseAtParentLocalZ(tool0, 0.105);
  const auto object = CanonicalAttachedObjectPose(
    tcp, 0.045 - 0.003, Quaterniond::Identity);
  EXPECT_NEAR(tcp.Pos().Z(), 0.82983, 1e-9);
  EXPECT_NEAR(object.Pos().X(), 2.0, 1e-9);
  EXPECT_NEAR(object.Pos().Y(), 1.32, 1e-9);
  EXPECT_NEAR(object.Pos().Z(), 0.78783, 1e-9);
  EXPECT_GT(object.Pos().Z() - 0.035, 0.752);
}

TEST(GraspEnvelope, AttachCorrectionUsesFiniteTwentyMillimetreGuard)
{
  EXPECT_TRUE(AttachCorrectionAllowed({0.0, 0.0, 0.0}, {0.019, 0.0, 0.0}, 0.020));
  EXPECT_FALSE(AttachCorrectionAllowed({0.0, 0.0, 0.0}, {0.021, 0.0, 0.0}, 0.020));
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(AttachCorrectionAllowed({nan, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.020));
  EXPECT_FALSE(AttachCorrectionAllowed({0.0, 0.0, 0.0}, {nan, 0.0, 0.0}, 0.020));
  EXPECT_FALSE(AttachCorrectionAllowed({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, nan));
  EXPECT_FALSE(AttachCorrectionAllowed({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, -0.001));
}

}  // namespace
