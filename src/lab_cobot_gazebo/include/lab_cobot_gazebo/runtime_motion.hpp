#pragma once

#include <array>
#include <cmath>

namespace lab_cobot_gazebo
{

struct PlanarVelocity
{
  double x;
  double y;
};

struct PlanarTwist
{
  double vx;
  double vy;
  double wz;
};

inline PlanarTwist wheelSpeedsToTwist(
  const std::array<double, 4> & controller_speeds,
  double wheel_radius = 0.07,
  double wheel_separation_width = 0.24,
  double wheel_separation_length = 0.175)
{
  // RoverTwistRelay publishes [-FL, -FR, -BL, -BR].  Undo that
  // controller convention while applying the standard mecanum inverse.
  const double fl = controller_speeds[0];
  const double fr = controller_speeds[1];
  const double bl = controller_speeds[2];
  const double br = controller_speeds[3];
  const double k_geom = wheel_separation_length + wheel_separation_width;
  return {
    -wheel_radius * (fl + fr + bl + br) / 4.0,
    wheel_radius * (fl - fr - bl + br) / 4.0,
    wheel_radius * (fl - fr + bl - br) / (4.0 * k_geom)};
}

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
