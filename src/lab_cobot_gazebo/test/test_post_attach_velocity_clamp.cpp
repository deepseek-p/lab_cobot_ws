// Unit tests for post-attach velocity clamp counting.
#include <gtest/gtest.h>

#include "lab_cobot_gazebo/post_attach_velocity_clamp.hpp"

namespace
{

using lab_cobot_gazebo::AdvancePostAttachVelocityClamp;

TEST(PostAttachVelocityClamp, ClampsForExactlyTenSteps)
{
  int remaining_steps = 10;

  for (int index = 0; index < 10; ++index) {
    const auto step = AdvancePostAttachVelocityClamp(remaining_steps);
    EXPECT_TRUE(step.should_clamp);
    remaining_steps = step.remaining_steps;
  }

  EXPECT_EQ(remaining_steps, 0);
  EXPECT_FALSE(AdvancePostAttachVelocityClamp(remaining_steps).should_clamp);
}

TEST(PostAttachVelocityClamp, CounterSaturatesAtZero)
{
  const auto zero = AdvancePostAttachVelocityClamp(0);
  const auto negative = AdvancePostAttachVelocityClamp(-3);

  EXPECT_FALSE(zero.should_clamp);
  EXPECT_EQ(zero.remaining_steps, 0);
  EXPECT_FALSE(negative.should_clamp);
  EXPECT_EQ(negative.remaining_steps, 0);
}

}  // namespace
