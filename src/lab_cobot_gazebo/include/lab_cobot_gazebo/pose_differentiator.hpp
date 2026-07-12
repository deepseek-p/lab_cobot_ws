#pragma once

#include <cmath>
#include <stdexcept>

namespace lab_cobot_gazebo
{
struct PoseVelocity
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
  bool valid{false};
};

class PoseDifferentiator
{
public:
  explicit PoseDifferentiator(double max_dt = 0.5)
  : max_dt_(max_dt)
  {
    if (!std::isfinite(max_dt_) || max_dt_ <= 0.0) {
      throw std::invalid_argument("max_dt must be finite and positive");
    }
  }

  PoseVelocity update(double x, double y, double yaw, double sim_time)
  {
    if (!initialized_) {
      reset(x, y, yaw, sim_time);
      return {};
    }

    const double dt = sim_time - time_;
    if (dt <= 0.0 || dt > max_dt_) {
      reset(x, y, yaw, sim_time);
      return {};
    }

    const double world_vx = (x - x_) / dt;
    const double world_vy = (y - y_) / dt;
    const double delta_yaw = std::atan2(std::sin(yaw - yaw_), std::cos(yaw - yaw_));
    const double midpoint_yaw = yaw_ + 0.5 * delta_yaw;
    const double cosine = std::cos(midpoint_yaw);
    const double sine = std::sin(midpoint_yaw);
    const PoseVelocity velocity{
      cosine * world_vx + sine * world_vy,
      -sine * world_vx + cosine * world_vy,
      delta_yaw / dt,
      true};
    reset(x, y, yaw, sim_time);
    return velocity;
  }

private:
  void reset(double x, double y, double yaw, double sim_time)
  {
    x_ = x;
    y_ = y;
    yaw_ = yaw;
    time_ = sim_time;
    initialized_ = true;
  }

  double max_dt_;
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  double time_{0.0};
  bool initialized_{false};
};
}  // namespace lab_cobot_gazebo
