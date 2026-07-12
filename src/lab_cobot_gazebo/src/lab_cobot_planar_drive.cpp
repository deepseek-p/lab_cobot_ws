#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <mutex>
#include <sstream>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "lab_cobot_gazebo/planar_safety.hpp"
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

bool sdfBox(
  const sdf::ElementPtr & sdf, const std::string & name,
  lab_cobot_gazebo::planar_safety::AxisAlignedBox & result)
{
  if (!sdf->HasElement(name)) {
    return false;
  }
  std::istringstream input(sdf->Get<std::string>(name));
  double center_x{};
  double center_y{};
  double size_x{};
  double size_y{};
  std::string trailing;
  if (!(input >> center_x >> center_y >> size_x >> size_y) || input >> trailing ||
    !std::isfinite(center_x) || !std::isfinite(center_y) ||
    !std::isfinite(size_x) || !std::isfinite(size_y) || size_x <= 0.0 || size_y <= 0.0)
  {
    return false;
  }
  result = {center_x - size_x * 0.5, center_x + size_x * 0.5,
    center_y - size_y * 0.5, center_y + size_y * 0.5};
  return true;
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
    chassis_length_ = sdfDouble(sdf, "chassis_length", -1.0);
    chassis_width_ = sdfDouble(sdf, "chassis_width", -1.0);
    table_safety_margin_ = sdfDouble(sdf, "table_safety_margin", -1.0);
    if (!std::isfinite(chassis_length_) || chassis_length_ <= 0.0 ||
      !std::isfinite(chassis_width_) || chassis_width_ <= 0.0 ||
      !lab_cobot_gazebo::planar_safety::isValidMargin(table_safety_margin_) ||
      !sdfBox(sdf, "table_a", table_a_) || !sdfBox(sdf, "table_b", table_b_))
    {
      gzerr << "lab_cobot_planar_drive has invalid chassis/table safety configuration" << std::endl;
      return;
    }
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
    const double next_x = x_ + world_velocity.x * dt;
    const double next_y = y_ + world_velocity.y * dt;
    const double next_yaw = std::atan2(std::sin(yaw_ + wz_ * dt), std::cos(yaw_ + wz_ * dt));
    const lab_cobot_gazebo::planar_safety::OrientedBox current{
      {x_, y_}, yaw_, chassis_length_, chassis_width_};
    const lab_cobot_gazebo::planar_safety::OrientedBox next{
      {next_x, next_y}, next_yaw, chassis_length_, chassis_width_};
    if (lab_cobot_gazebo::planar_safety::isMotionAllowed(
        current, next, {table_a_, table_b_}, table_safety_margin_))
    {
      x_ = next_x;
      y_ = next_y;
      yaw_ = next_yaw;
    } else {
      vx_ = 0.0;
      vy_ = 0.0;
      wz_ = 0.0;
      RCLCPP_WARN_THROTTLE(
        ros_node_->get_logger(), *ros_node_->get_clock(), 2000,
        "Blocked planar motion at a table safety boundary");
    }
    holdPlanarPose();
  }

  void holdPlanarPose()
  {
    model_->SetWorldPose(ignition::math::Pose3d(x_, y_, z_, 0.0, 0.0, yaw_));
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
  double chassis_length_{0.0};
  double chassis_width_{0.0};
  double table_safety_margin_{0.0};
  lab_cobot_gazebo::planar_safety::AxisAlignedBox table_a_{};
  lab_cobot_gazebo::planar_safety::AxisAlignedBox table_b_{};
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
