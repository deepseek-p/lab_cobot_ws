// 抓取几何封套判定纯函数(仅依赖 ignition::math,可被 gtest 直接单测)。
// 语义:物块中心在"抓取系"(双指连杆中点 + 可配偏移,姿态取左指)下的
// 偏移落在盒式封套内即判定可焊接。这是几何邻近判定,不是接触力检测。
#ifndef LAB_COBOT_GAZEBO__GRASP_ENVELOPE_HPP_
#define LAB_COBOT_GAZEBO__GRASP_ENVELOPE_HPP_

#include <cmath>
#include <vector>

#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

namespace lab_cobot_gazebo
{

struct GraspEnvelopeLimits
{
  double max_center_distance;
  double max_abs_x;
  double max_abs_y;
  double min_z;
  double max_z;
};

// 抓取系原点:双指连杆位置中点 + 按抓取姿态旋转后的 center_offset。
inline ignition::math::Vector3d GraspFramePosition(
  const ignition::math::Vector3d & left_finger_pos,
  const ignition::math::Vector3d & right_finger_pos,
  const ignition::math::Quaterniond & grasp_rotation,
  const ignition::math::Vector3d & center_offset)
{
  return (left_finger_pos + right_finger_pos) * 0.5 +
         grasp_rotation.RotateVector(center_offset);
}

// 物块中心在抓取系下的偏移(把世界系位移旋进抓取系)。
inline ignition::math::Vector3d ObjectOffsetInGraspFrame(
  const ignition::math::Vector3d & object_pos,
  const ignition::math::Vector3d & grasp_frame_pos,
  const ignition::math::Quaterniond & grasp_rotation)
{
  return grasp_rotation.Inverse().RotateVector(object_pos - grasp_frame_pos);
}

// 盒式封套判定。
inline bool OffsetInsideGraspEnvelope(
  const ignition::math::Vector3d & offset,
  const GraspEnvelopeLimits & limits)
{
  return offset.Length() <= limits.max_center_distance &&
         std::abs(offset.X()) <= limits.max_abs_x &&
         std::abs(offset.Y()) <= limits.max_abs_y &&
         offset.Z() >= limits.min_z &&
         offset.Z() <= limits.max_z;
}

inline int NearestOffsetIndexInsideEnvelope(
  const std::vector<ignition::math::Vector3d> & offsets,
  const GraspEnvelopeLimits & limits)
{
  int best_index = -1;
  double best_distance = 0.0;
  for (std::size_t index = 0; index < offsets.size(); ++index) {
    const auto & offset = offsets[index];
    if (!OffsetInsideGraspEnvelope(offset, limits)) {
      continue;
    }
    const double distance = offset.Length();
    if (best_index < 0 || distance < best_distance) {
      best_index = static_cast<int>(index);
      best_distance = distance;
    }
  }
  return best_index;
}

}  // namespace lab_cobot_gazebo

#endif  // LAB_COBOT_GAZEBO__GRASP_ENVELOPE_HPP_
