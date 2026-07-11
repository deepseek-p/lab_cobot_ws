#pragma once

#include <cmath>

namespace lab_cobot_gazebo
{

struct PlanarVelocity
{
  double x;
  double y;
};

inline PlanarVelocity rotateBaseToWorld(double x, double y, double yaw)
{
  return {
    std::cos(yaw) * x - std::sin(yaw) * y,
    std::sin(yaw) * x + std::cos(yaw) * y};
}

inline PlanarVelocity rotateWorldToBase(double x, double y, double yaw)
{
  return {
    std::cos(yaw) * x + std::sin(yaw) * y,
    -std::sin(yaw) * x + std::cos(yaw) * y};
}

}  // namespace lab_cobot_gazebo
