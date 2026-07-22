// Unit tests for tactile finger contact gate helpers.
#include <gtest/gtest.h>

#include <vector>

#include "lab_cobot_gazebo/finger_collision_config.hpp"
#include "lab_cobot_gazebo/finger_contact_gate.hpp"

namespace
{

using lab_cobot_gazebo::BothFingersTouching;
using lab_cobot_gazebo::ConsecutiveContactGate;
using lab_cobot_gazebo::ContactPair;
using lab_cobot_gazebo::FingerTouchesTarget;
using lab_cobot_gazebo::IsPrimaryFingerCollisionName;
using lab_cobot_gazebo::IsTactileProbeCollisionName;
using lab_cobot_gazebo::PrimaryFingerCollisionConfig;
using lab_cobot_gazebo::SelectAttachLinkName;
using lab_cobot_gazebo::ShouldDisablePrimaryFingerCollisions;
using lab_cobot_gazebo::TactileProbeSurfaceConfig;

ContactPair Pair(
  const char * model1,
  const char * link1,
  const char * model2,
  const char * link2)
{
  return {model1, link1, model2, link2};
}

TEST(FingerContactGate, LeftFingerTouchIsDetected)
{
  const auto pair = Pair(
    "lab_cobot", "gripper_left_finger", "aruco_sample", "link");

  EXPECT_TRUE(
    FingerTouchesTarget(
      pair, "lab_cobot", "gripper_left_finger", "aruco_sample"));
  EXPECT_FALSE(
    FingerTouchesTarget(
      pair, "lab_cobot", "gripper_right_finger", "aruco_sample"));
}

TEST(FingerContactGate, BothFingersMustTouchSameTarget)
{
  const std::vector<ContactPair> pairs = {
    Pair("lab_cobot", "gripper_left_finger", "aruco_sample", "link"),
    Pair("lab_cobot", "gripper_right_finger", "aruco_sample", "link"),
  };

  EXPECT_TRUE(
    BothFingersTouching(
      pairs,
      "lab_cobot",
      "gripper_left_finger",
      "gripper_right_finger",
      "aruco_sample"));
}

TEST(FingerContactGate, UnrelatedCollisionPairsAreIgnored)
{
  const std::vector<ContactPair> pairs = {
    Pair("lab_cobot", "gripper_left_finger", "table", "top"),
    Pair("lab_cobot", "wrist_3_link", "aruco_sample", "link"),
  };

  EXPECT_FALSE(
    BothFingersTouching(
      pairs,
      "lab_cobot",
      "gripper_left_finger",
      "gripper_right_finger",
      "aruco_sample"));
}

TEST(FingerContactGate, ConsecutiveGateRequiresNTicks)
{
  ConsecutiveContactGate gate(3);

  EXPECT_FALSE(gate.Update(true));
  EXPECT_FALSE(gate.Update(true));
  EXPECT_TRUE(gate.Update(true));
}

TEST(FingerContactGate, ConsecutiveGateResetsOnMissingContact)
{
  ConsecutiveContactGate gate(2);

  EXPECT_FALSE(gate.Update(true));
  EXPECT_FALSE(gate.Update(false));
  EXPECT_FALSE(gate.Update(true));
  EXPECT_TRUE(gate.Update(true));
}

TEST(FingerCollisionConfig, PrimaryFingerCollisionsExcludeTactileProbes)
{
  EXPECT_TRUE(IsPrimaryFingerCollisionName("gripper_left_finger_collision"));
  EXPECT_TRUE(IsPrimaryFingerCollisionName("gripper_right_finger_collision"));
  EXPECT_FALSE(
    IsPrimaryFingerCollisionName(
      "gripper_left_finger_tactile_probe_collision_1"));
  EXPECT_FALSE(IsPrimaryFingerCollisionName("ur_wrist_3_link_collision"));
}

TEST(FingerCollisionConfig, TactileProbeCollisionNamesAreDetected)
{
  EXPECT_TRUE(
    IsTactileProbeCollisionName(
      "gripper_left_finger_tactile_probe_collision_1"));
  EXPECT_TRUE(
    IsTactileProbeCollisionName(
      "gripper_right_finger_tactile_probe_collision_1"));
  EXPECT_FALSE(IsTactileProbeCollisionName("gripper_left_finger_collision"));
}

TEST(FingerCollisionConfig, PrimaryFingerCollisionsStayEnabledByDefault)
{
  EXPECT_FALSE(ShouldDisablePrimaryFingerCollisions(false));
  EXPECT_TRUE(ShouldDisablePrimaryFingerCollisions(true));
}

TEST(FingerCollisionConfig, PrimaryFingerSurfaceDisablesContactGeneration)
{
  const auto config = PrimaryFingerCollisionConfig();

  EXPECT_EQ(config.collide_bits, 0u);
  EXPECT_EQ(config.surface_collide_bitmask, 0u);
}

TEST(FingerCollisionConfig, TactileProbeSurfaceDisablesContactForces)
{
  const auto config = TactileProbeSurfaceConfig();

  EXPECT_TRUE(config.collide_without_contact);
  EXPECT_EQ(config.collide_without_contact_bitmask, 0xffffu);
  EXPECT_EQ(config.collide_bits, 0xffffu);
  EXPECT_EQ(config.max_contacts, 10u);
}

TEST(FingerCollisionConfig, ForceRecordingModeEnablesProbeContactResponse)
{
  const auto config = TactileProbeSurfaceConfig(true);

  EXPECT_FALSE(config.collide_without_contact);
  EXPECT_EQ(config.collide_without_contact_bitmask, 0u);
  EXPECT_EQ(config.collide_bits, 0xffffu);
}

TEST(FingerCollisionConfig, DefaultAttachLinkStaysOnFingerLink)
{
  EXPECT_EQ(
    SelectAttachLinkName(
      false,
      "gripper_left_finger",
      "gripper_base"),
    "gripper_left_finger");
}

TEST(FingerCollisionConfig, TactileAttachLinkUsesStableGripperBase)
{
  EXPECT_EQ(
    SelectAttachLinkName(
      true,
      "gripper_left_finger",
      "gripper_base"),
    "gripper_base");
}

TEST(FingerCollisionConfig, TactileAttachLinkFallsBackWhenStableLinkMissing)
{
  EXPECT_EQ(
    SelectAttachLinkName(
      true,
      "gripper_left_finger",
      ""),
    "gripper_left_finger");
}

}  // namespace
