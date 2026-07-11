#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "gazebo_msgs/msg/model_states.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "lab_cobot_gazebo/runtime_motion.hpp"

using namespace std::chrono_literals;

class MecanumGazeboKinematicDrive : public rclcpp::Node
{
public:
  MecanumGazeboKinematicDrive()
  : Node("mecanum_gazebo_kinematic_drive")
  {
    model_name_ = this->declare_parameter<std::string>("model_name", "mecanum3");
    service_name_ = this->declare_parameter<std::string>(
      "service_name", "/set_entity_state");
    model_states_topic_ = this->declare_parameter<std::string>(
      "model_states_topic", "/model_states");
    max_vx_ = this->declare_parameter<double>("max_vx", 0.5);
    max_vy_ = this->declare_parameter<double>("max_vy", 0.3);
    max_wz_ = this->declare_parameter<double>("max_wz", 1.2);
    max_accel_xy_ = this->declare_parameter<double>("max_accel_xy", 0.5);
    max_accel_wz_ = this->declare_parameter<double>("max_accel_wz", 1.5);
    command_timeout_ = this->declare_parameter<double>("command_timeout", 0.3);
    z_height_ = this->declare_parameter<double>("z_height", 0.0);
    update_rate_ = this->declare_parameter<double>("update_rate", 50.0);

    if (update_rate_ < 1.0) {
      update_rate_ = 50.0;
    }

    client_ = this->create_client<gazebo_msgs::srv::SetEntityState>(service_name_);

    sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/rover_twist",
      10,
      std::bind(&MecanumGazeboKinematicDrive::onTwist, this, std::placeholders::_1));
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      10,
      std::bind(&MecanumGazeboKinematicDrive::onTwist, this, std::placeholders::_1));

    sub_model_states_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
      model_states_topic_,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&MecanumGazeboKinematicDrive::onModelStates, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / update_rate_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MecanumGazeboKinematicDrive::onTimer, this));

    last_update_time_ = this->now();
    last_command_time_ = this->now();

    RCLCPP_INFO(
      this->get_logger(),
      "kinematic drive started for model '%s' using service '%s'",
      model_name_.c_str(), service_name_.c_str());
  }

private:
  static double clamp(double value, double limit)
  {
    return std::max(-limit, std::min(limit, value));
  }

  static double rampValue(double current, double target, double max_step)
  {
    const double delta = target - current;
    if (delta > max_step) {
      return current + max_step;
    }
    if (delta < -max_step) {
      return current - max_step;
    }
    return target;
  }

  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q_msg)
  {
    tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
  }

  static geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    q.normalize();

    geometry_msgs::msg::Quaternion q_msg;
    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();
    q_msg.w = q.w();
    return q_msg;
  }

  void onTwist(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    target_vx_ = clamp(msg->linear.x, max_vx_);
    target_vy_ = clamp(msg->linear.y, max_vy_);
    target_wz_ = clamp(msg->angular.z, max_wz_);
    last_command_time_ = this->now();
  }

  void onModelStates(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
  {
    if (initialized_) {
      return;
    }

    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == model_name_) {
        x_ = msg->pose[i].position.x;
        y_ = msg->pose[i].position.y;
        yaw_ = yawFromQuaternion(msg->pose[i].orientation);
        initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "initialized pose from /model_states");
        return;
      }
    }
  }

  void onTimer()
  {
    const auto now = this->now();
    double dt = (now - last_update_time_).seconds();
    last_update_time_ = now;

    if (dt <= 0.0) {
      return;
    }
    if (dt > 0.2) {
      dt = 1.0 / update_rate_;
    }

    if (!initialized_) {
      return;
    }

    if (!client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "waiting for %s", service_name_.c_str());
      return;
    }
    if (request_in_flight_.exchange(true)) {
      return;
    }

    double desired_vx = target_vx_;
    double desired_vy = target_vy_;
    double desired_wz = target_wz_;

    if ((now - last_command_time_).seconds() > command_timeout_) {
      desired_vx = 0.0;
      desired_vy = 0.0;
      desired_wz = 0.0;
    }

    current_vx_ = rampValue(current_vx_, desired_vx, max_accel_xy_ * dt);
    current_vy_ = rampValue(current_vy_, desired_vy, max_accel_xy_ * dt);
    current_wz_ = rampValue(current_wz_, desired_wz, max_accel_wz_ * dt);

    const double vx = current_vx_;
    const double vy = current_vy_;
    const double wz = current_wz_;

    const double cos_yaw = std::cos(yaw_);
    const double sin_yaw = std::sin(yaw_);
    x_ += (cos_yaw * vx - sin_yaw * vy) * dt;
    y_ += (sin_yaw * vx + cos_yaw * vy) * dt;
    yaw_ += wz * dt;

    while (yaw_ > M_PI) {
      yaw_ -= 2.0 * M_PI;
    }
    while (yaw_ < -M_PI) {
      yaw_ += 2.0 * M_PI;
    }

    auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
    request->state.name = model_name_;
    request->state.reference_frame = "world";
    request->state.pose.position.x = x_;
    request->state.pose.position.y = y_;
    request->state.pose.position.z = z_height_;
    request->state.pose.orientation = quaternionFromYaw(yaw_);
    const auto world_velocity = lab_cobot_gazebo::rotateBaseToWorld(vx, vy, yaw_);
    request->state.twist.linear.x = world_velocity.x;
    request->state.twist.linear.y = world_velocity.y;
    request->state.twist.angular.z = wz;

    try {
      client_->async_send_request(
        request,
        [this](rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedFuture) {
          request_in_flight_.store(false);
        });
    } catch (...) {
      request_in_flight_.store(false);
      throw;
    }
  }

  std::string model_name_;
  std::string service_name_;
  std::string model_states_topic_;
  double max_vx_{0.5};
  double max_vy_{0.3};
  double max_wz_{1.2};
  double max_accel_xy_{0.5};
  double max_accel_wz_{1.5};
  double command_timeout_{0.3};
  double z_height_{0.0};
  double update_rate_{50.0};

  bool initialized_{false};
  std::atomic_bool request_in_flight_{false};
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  double target_vx_{0.0};
  double target_vy_{0.0};
  double target_wz_{0.0};
  double current_vx_{0.0};
  double current_vy_{0.0};
  double current_wz_{0.0};

  rclcpp::Time last_update_time_;
  rclcpp::Time last_command_time_;

  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr client_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr sub_model_states_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MecanumGazeboKinematicDrive>());
  rclcpp::shutdown();
  return 0;
}
