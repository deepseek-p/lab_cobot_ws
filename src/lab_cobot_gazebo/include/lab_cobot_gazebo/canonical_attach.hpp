#ifndef LAB_COBOT_GAZEBO__CANONICAL_ATTACH_HPP_
#define LAB_COBOT_GAZEBO__CANONICAL_ATTACH_HPP_

#include <cmath>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

namespace lab_cobot_gazebo
{

inline ignition::math::Pose3d PoseAtParentLocalZ(
  const ignition::math::Pose3d & parent_pose,
  const double local_offset_z)
{
  return parent_pose * ignition::math::Pose3d(
    0.0, 0.0, local_offset_z, 0.0, 0.0, 0.0);
}

// Define one object-centre position from the real TCP. Keep the object's
// current rotation to avoid an unnecessary angular snap at attachment.
inline ignition::math::Pose3d CanonicalAttachedObjectPose(
  const ignition::math::Pose3d & tcp_pose,
  const double local_offset_z,
  const ignition::math::Quaterniond & current_object_rotation)
{
  return ignition::math::Pose3d(
    tcp_pose.Pos() + tcp_pose.Rot().RotateVector({0.0, 0.0, local_offset_z}),
    current_object_rotation);
}

inline bool AttachCorrectionAllowed(
  const ignition::math::Vector3d & current_position,
  const ignition::math::Vector3d & canonical_position,
  const double max_correction)
{
  const auto finite = [](const ignition::math::Vector3d & value) {
      return std::isfinite(value.X()) && std::isfinite(value.Y()) &&
             std::isfinite(value.Z());
    };
  return finite(current_position) && finite(canonical_position) &&
         std::isfinite(max_correction) && max_correction >= 0.0 &&
         current_position.Distance(canonical_position) <= max_correction;
}

}  // namespace lab_cobot_gazebo

#endif  // LAB_COBOT_GAZEBO__CANONICAL_ATTACH_HPP_
