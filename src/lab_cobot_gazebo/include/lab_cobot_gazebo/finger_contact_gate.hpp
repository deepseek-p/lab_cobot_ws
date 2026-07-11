// Finger contact gate helpers for tactile attach decisions.
#ifndef LAB_COBOT_GAZEBO__FINGER_CONTACT_GATE_HPP_
#define LAB_COBOT_GAZEBO__FINGER_CONTACT_GATE_HPP_

#include <string>
#include <vector>

namespace lab_cobot_gazebo
{

struct ContactPair
{
  std::string model1;
  std::string link1;
  std::string model2;
  std::string link2;
};

struct FingerTouchState
{
  bool left{false};
  bool right{false};
};

inline bool PairContainsModelAndLink(
  const ContactPair & pair,
  const std::string & model_name,
  const std::string & link_name)
{
  return (pair.model1 == model_name && pair.link1 == link_name) ||
         (pair.model2 == model_name && pair.link2 == link_name);
}

inline bool PairContainsModel(
  const ContactPair & pair,
  const std::string & model_name)
{
  return pair.model1 == model_name || pair.model2 == model_name;
}

inline bool FingerTouchesTarget(
  const ContactPair & pair,
  const std::string & robot_model_name,
  const std::string & finger_link_name,
  const std::string & target_model_name)
{
  return PairContainsModelAndLink(pair, robot_model_name, finger_link_name) &&
         PairContainsModel(pair, target_model_name);
}

inline FingerTouchState FingerTouchStateForTarget(
  const std::vector<ContactPair> & pairs,
  const std::string & robot_model_name,
  const std::string & left_finger_link_name,
  const std::string & right_finger_link_name,
  const std::string & target_model_name);

inline bool BothFingersTouching(
  const std::vector<ContactPair> & pairs,
  const std::string & robot_model_name,
  const std::string & left_finger_link_name,
  const std::string & right_finger_link_name,
  const std::string & target_model_name)
{
  const auto state = FingerTouchStateForTarget(
    pairs,
    robot_model_name,
    left_finger_link_name,
    right_finger_link_name,
    target_model_name);
  return state.left && state.right;
}

inline FingerTouchState FingerTouchStateForTarget(
  const std::vector<ContactPair> & pairs,
  const std::string & robot_model_name,
  const std::string & left_finger_link_name,
  const std::string & right_finger_link_name,
  const std::string & target_model_name)
{
  FingerTouchState state;
  for (const auto & pair : pairs) {
    state.left = state.left ||
      FingerTouchesTarget(
      pair, robot_model_name, left_finger_link_name, target_model_name);
    state.right = state.right ||
      FingerTouchesTarget(
      pair, robot_model_name, right_finger_link_name, target_model_name);
  }
  return state;
}

class ConsecutiveContactGate
{
public:
  explicit ConsecutiveContactGate(const int threshold)
  : threshold_(threshold < 1 ? 1 : threshold)
  {
  }

  bool Update(const bool touching)
  {
    if (!touching) {
      count_ = 0;
      return false;
    }
    count_ += 1;
    return count_ >= threshold_;
  }

  void Reset()
  {
    count_ = 0;
  }

  int Count() const
  {
    return count_;
  }

private:
  int threshold_{1};
  int count_{0};
};

}  // namespace lab_cobot_gazebo

#endif  // LAB_COBOT_GAZEBO__FINGER_CONTACT_GATE_HPP_
