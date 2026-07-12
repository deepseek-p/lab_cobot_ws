#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <mutex>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "lab_cobot_gazebo/runtime_motion.hpp"

namespace gazebo
{
namespace
{
double sdfDouble(const sdf::ElementPtr & sdf, const std::string & name, double fallback)
{
  return sdf->HasElement(name) ? sdf->Get<double>(name) : fallback;
}

std::string sdfString(
  const sdf::ElementPtr & sdf, const std::string & name, const std::string & fallback)
{
  return sdf->HasElement(name) ? sdf->Get<std::string>(name) : fallback;
}

double approach(double current, double target, double maximum_step)
{
  return current + std::clamp(target - current, -maximum_step, maximum_step);
}
}  // namespace

class LabCobotPlanarDrive final : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = std::move(model);
    wheel_radius_ = sdfDouble(sdf, "wheel_radius", 0.07);
    wheel_width_ = sdfDouble(sdf, "wheel_separation_width", 0.24);
    wheel_length_ = sdfDouble(sdf, "wheel_separation_length", 0.175);
    max_vx_ = sdfDouble(sdf, "max_vx", 0.5);
    max_vy_ = sdfDouble(sdf, "max_vy", 0.3);
    max_wz_ = sdfDouble(sdf, "max_wz", 1.2);
    command_timeout_ = sdfDouble(sdf, "command_timeout", 0.3);
    const auto topic = sdfString(
      sdf, "wheel_command_topic", "/wheel_velocity_controller/commands");

    // This model intentionally uses a planar kinematic chassis. Gravity remains
    // enabled for the world and all independent objects.
    model_->SetGravityMode(false);
    ros_node_ = gazebo_ros::Node::Get(sdf);
    command_subscription_ = ros_node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      topic, rclcpp::QoS(10),
      std::bind(&LabCobotPlanarDrive::onCommand, this, std::placeholders::_1));
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&LabCobotPlanarDrive::onUpdate, this, std::placeholders::_1));
  }

private:
  void onCommand(const std_msgs::msg::Float64MultiArray::SharedPtr message)
  {
    if (message->data.size() != wheel_commands_.size()) {
      RCLCPP_WARN_THROTTLE(
        ros_node_->get_logger(), *ros_node_->get_clock(), 2000,
        "Expected four mecanum wheel velocities, received %zu", message->data.size());
      return;
    }
    std::array<double, 4> next{};
    for (std::size_t i = 0; i < next.size(); ++i) {
      if (!std::isfinite(message->data[i])) {
        return;
      }
      next[i] = message->data[i];
    }
    std::lock_guard<std::mutex> lock(command_mutex_);
    wheel_commands_ = next;
    ++command_generation_;
  }

  void onUpdate(const common::UpdateInfo & info)
  {
    const double now = info.simTime.Double();
    if (!initialized_) {
      const auto pose = model_->WorldPose();
      x_ = pose.Pos().X();
      y_ = pose.Pos().Y();
      z_ = pose.Pos().Z();
      yaw_ = pose.Rot().Yaw();
      last_update_time_ = now;
      initialized_ = true;
    }

    const double dt = now - last_update_time_;
    last_update_time_ = now;
    if (dt <= 0.0 || dt > 0.1) {
      holdPlanarPose();
      return;
    }

    std::array<double, 4> commands{};
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      commands = wheel_commands_;
      if (applied_generation_ != command_generation_) {
        applied_generation_ = command_generation_;
        last_command_time_ = now;
      }
    }
    if (last_command_time_ < 0.0 || now - last_command_time_ > command_timeout_) {
      commands.fill(0.0);
    }

    auto target = lab_cobot_gazebo::wheelSpeedsToTwist(
      commands, wheel_radius_, wheel_width_, wheel_length_);
    target.vx = std::clamp(target.vx, -max_vx_, max_vx_);
    target.vy = std::clamp(target.vy, -max_vy_, max_vy_);
    target.wz = std::clamp(target.wz, -max_wz_, max_wz_);
    vx_ = approach(vx_, target.vx, linear_acceleration_ * dt);
    vy_ = approach(vy_, target.vy, linear_acceleration_ * dt);
    wz_ = approach(wz_, target.wz, angular_acceleration_ * dt);

    const auto world_velocity = lab_cobot_gazebo::rotateBaseToWorld(vx_, vy_, yaw_);
    x_ += world_velocity.x * dt;
    y_ += world_velocity.y * dt;
    yaw_ += wz_ * dt;
    holdPlanarPose();
  }

  void holdPlanarPose()
  {
    const auto world_velocity = lab_cobot_gazebo::rotateBaseToWorld(vx_, vy_, yaw_);
    model_->SetWorldPose(ignition::math::Pose3d(x_, y_, z_, 0.0, 0.0, yaw_));
    model_->SetLinearVel(ignition::math::Vector3d(world_velocity.x, world_velocity.y, 0.0));
    model_->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, wz_));
  }

  physics::ModelPtr model_;
  event::ConnectionPtr update_connection_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_subscription_;
  std::mutex command_mutex_;
  std::array<double, 4> wheel_commands_{};
  std::size_t command_generation_{0};
  std::size_t applied_generation_{0};
  double wheel_radius_{0.07};
  double wheel_width_{0.24};
  double wheel_length_{0.175};
  double max_vx_{0.5};
  double max_vy_{0.3};
  double max_wz_{1.2};
  double command_timeout_{0.3};
  double linear_acceleration_{0.5};
  double angular_acceleration_{1.5};
  double x_{0.0};
  double y_{0.0};
  double z_{0.0};
  double yaw_{0.0};
  double vx_{0.0};
  double vy_{0.0};
  double wz_{0.0};
  double last_update_time_{0.0};
  double last_command_time_{-1.0};
  bool initialized_{false};
};

GZ_REGISTER_MODEL_PLUGIN(LabCobotPlanarDrive)
}  // namespace gazebo
