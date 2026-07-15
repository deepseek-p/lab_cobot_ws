// Pure helpers for counting post-attach velocity clamp steps.
#ifndef LAB_COBOT_GAZEBO__POST_ATTACH_VELOCITY_CLAMP_HPP_
#define LAB_COBOT_GAZEBO__POST_ATTACH_VELOCITY_CLAMP_HPP_

namespace lab_cobot_gazebo
{

struct PostAttachVelocityClampStep
{
  bool should_clamp{false};
  int remaining_steps{0};
};

inline PostAttachVelocityClampStep AdvancePostAttachVelocityClamp(
  const int remaining_steps)
{
  if (remaining_steps <= 0) {
    return {false, 0};
  }
  return {true, remaining_steps - 1};
}

}  // namespace lab_cobot_gazebo

#endif  // LAB_COBOT_GAZEBO__POST_ATTACH_VELOCITY_CLAMP_HPP_
