#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <mutex>
#include <string>

#include <gazebo/common/Event.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <ignition/math/Vector3.hh>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sdf/sdf.hh>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "lab_cobot_gazebo/mecanum_kinematics.hpp"

namespace gazebo
{
namespace
{
std::string SdfString(
  const sdf::ElementPtr & sdf,
  const std::string & key,
  const std::string & fallback)
{
  if (sdf && sdf->HasElement(key)) {
    return sdf->Get<std::string>(key);
  }
  return fallback;
}

double SdfDouble(
  const sdf::ElementPtr & sdf,
  const std::string & key,
  const double fallback)
{
  if (sdf && sdf->HasElement(key)) {
    return sdf->Get<double>(key);
  }
  return fallback;
}

bool SdfBool(
  const sdf::ElementPtr & sdf,
  const std::string & key,
  const bool fallback)
{
  if (sdf && sdf->HasElement(key)) {
    return sdf->Get<bool>(key);
  }
  return fallback;
}

using lab_cobot_gazebo::FiniteOrZero;
// 限幅语义与头文件 ClampAbs 一致,保留原名减少改动面
inline double Clamp(const double value, const double limit)
{
  return lab_cobot_gazebo::ClampAbs(value, limit);
}

ignition::math::Vector3d ClampVectorLength(
  const ignition::math::Vector3d & value,
  const double max_length)
{
  const double length = value.Length();
  if (length <= max_length || length <= 1.0e-9) {
    return value;
  }
  return value * (max_length / length);
}
}  // namespace

class LabCobotMecanumDrive : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    base_link_name_ = SdfString(sdf, "base_link", "base_link");
    control_mode_ = SdfString(sdf, "control_mode", "force_from_wheel_joints");
    wheel_command_topic_ = SdfString(
      sdf, "wheel_command_topic", "/wheel_velocity_controller/commands");
    wheel_radius_ = SdfDouble(sdf, "wheel_radius", 0.08);
    wheelbase_radius_ = SdfDouble(sdf, "wheelbase_radius", 0.47);
    linear_gain_ = SdfDouble(sdf, "linear_gain", 8.0);
    angular_gain_ = SdfDouble(sdf, "angular_gain", 10.0);
    max_force_ = SdfDouble(sdf, "max_force", 450.0);
    max_torque_ = SdfDouble(sdf, "max_torque", 260.0);
    max_linear_speed_ = SdfDouble(sdf, "max_linear_speed", 0.45);
    max_angular_speed_ = SdfDouble(sdf, "max_angular_speed", 0.9);
    max_linear_accel_ = SdfDouble(sdf, "max_linear_accel", 0.8);
    max_angular_accel_ = SdfDouble(sdf, "max_angular_accel", 1.5);
    command_timeout_ = SdfDouble(sdf, "command_timeout", 0.25);
    publish_odom_ = SdfBool(sdf, "publish_odom", true);
    odom_topic_ = SdfString(sdf, "odom_topic", "/odom");
    odom_frame_ = SdfString(sdf, "odom_frame", "odom");
    base_frame_ = SdfString(sdf, "base_frame", "base_footprint");
    odom_publish_period_ = 1.0 / std::max(1.0, SdfDouble(sdf, "odom_rate", 20.0));

    auto wheel_joint = sdf ? sdf->GetElement("wheel_joint") : sdf::ElementPtr();
    for (size_t index = 0; index < wheel_joint_names_.size(); ++index) {
      if (wheel_joint) {
        wheel_joint_names_[index] = wheel_joint->Get<std::string>();
        wheel_joint = wheel_joint->GetNextElement("wheel_joint");
      }
    }

    base_link_ = model_->GetLink(base_link_name_);
    if (!base_link_) {
      base_link_ = model_->GetLink(model_->GetName() + "::" + base_link_name_);
    }
    if (!base_link_) {
      base_link_ = model_->GetLink();
    }
    for (size_t index = 0; index < wheel_joint_names_.size(); ++index) {
      wheel_joints_[index] = model_->GetJoint(wheel_joint_names_[index]);
    }

    if (UsesWheelCommandWatchdog() || publish_odom_) {
      ros_node_ = gazebo_ros::Node::Get(sdf);
    }
    if (publish_odom_ && ros_node_) {
      odom_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    }
    if (UsesWheelCommandWatchdog()) {
      wheel_command_sub_ =
        ros_node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        wheel_command_topic_,
        rclcpp::QoS(10),
        std::bind(&LabCobotMecanumDrive::OnWheelCommand, this, std::placeholders::_1));
    }

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&LabCobotMecanumDrive::OnUpdate, this));

    gzmsg << "lab_cobot_mecanum_drive loaded from wheel joints"
          << " [" << wheel_joint_names_[0]
          << ", " << wheel_joint_names_[1]
          << ", " << wheel_joint_names_[2]
          << ", " << wheel_joint_names_[3] << "]"
          << " to link " << base_link_name_
          << " using " << control_mode_
          << " command_topic " << wheel_command_topic_ << std::endl;
  }

private:
  void OnUpdate()
  {
    if (UsesMeasuredWheelJoints() && !AllWheelJointsAvailable()) {
      if (!reported_missing_dependencies_) {
        gzerr << "lab_cobot_mecanum_drive missing wheel joints" << std::endl;
        reported_missing_dependencies_ = true;
      }
      return;
    }

    const auto twist = (control_mode_ == "pose_from_wheel_commands") ?
      TwistFromWheelCommands() : TwistFromWheelSpeeds();
    if (control_mode_ == "velocity_from_wheel_joints") {
      if (!base_link_) {
        if (!reported_missing_dependencies_) {
          gzerr << "lab_cobot_mecanum_drive missing base link" << std::endl;
          reported_missing_dependencies_ = true;
        }
        return;
      }
      ApplyVelocityModel(twist);
      PublishOdometry(PhysicsOdomPose(), PhysicsOdomTwist());
      return;
    }
    if (
      control_mode_ == "pose_from_wheel_joints" ||
      control_mode_ == "pose_from_wheel_commands")
    {
      ApplyPoseModel(twist);
      return;
    }

    if (control_mode_ == "force_from_wheel_joints") {
      if (!base_link_) {
        if (!reported_missing_dependencies_) {
          gzerr << "lab_cobot_mecanum_drive missing base link" << std::endl;
          reported_missing_dependencies_ = true;
        }
        return;
      }
      ApplyForceModel(twist);
      PublishOdometry(PhysicsOdomPose(), PhysicsOdomTwist());
      return;
    }

    if (!base_link_) {
      if (!reported_missing_dependencies_) {
        gzerr << "lab_cobot_mecanum_drive missing base link" << std::endl;
        reported_missing_dependencies_ = true;
      }
      return;
    }
    if (!reported_missing_dependencies_) {
      gzerr << "lab_cobot_mecanum_drive unknown control_mode "
            << control_mode_ << std::endl;
      reported_missing_dependencies_ = true;
    }
  }

  void OnWheelCommand(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    for (size_t index = 0; index < commanded_wheel_speeds_.size(); ++index) {
      commanded_wheel_speeds_[index] = index < msg->data.size() ?
        FiniteOrZero(msg->data[index]) : 0.0;
    }
    last_command_time_ = model_ && model_->GetWorld() ?
      model_->GetWorld()->SimTime().Double() : 0.0;
  }

  bool UsesWheelCommandWatchdog() const
  {
    return control_mode_ == "pose_from_wheel_commands" ||
           control_mode_ == "pose_from_wheel_joints" ||
           control_mode_ == "velocity_from_wheel_joints" ||
           control_mode_ == "force_from_wheel_joints";
  }

  bool UsesMeasuredWheelJoints() const
  {
    return control_mode_ == "pose_from_wheel_joints" ||
           control_mode_ == "velocity_from_wheel_joints" ||
           control_mode_ == "force_from_wheel_joints";
  }

  bool HasRecentWheelCommand()
  {
    double last_command_time;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      last_command_time = last_command_time_;
    }
    const double now = model_ && model_->GetWorld() ?
      model_->GetWorld()->SimTime().Double() : 0.0;
    return last_command_time >= 0.0 && now - last_command_time <= command_timeout_;
  }

  void ApplyVelocityModel(const std::array<double, 3> & twist)
  {
    const double now = model_->GetWorld()->SimTime().Double();
    double dt = last_update_time_ > 0.0 ? now - last_update_time_ : 0.001;
    dt = std::max(0.001, std::min(dt, 0.05));
    last_update_time_ = now;

    const auto world_pose = model_->WorldPose();
    const auto desired_linear_world = world_pose.Rot().RotateVector(
      ignition::math::Vector3d(twist[0], twist[1], 0.0));
    const auto current_linear_world = model_->WorldLinearVel();
    const auto linear_delta = ClampVectorLength(
      ignition::math::Vector3d(
        desired_linear_world.X() - current_linear_world.X(),
        desired_linear_world.Y() - current_linear_world.Y(),
        0.0),
      max_linear_accel_ * dt);
    model_->SetLinearVel(
      ignition::math::Vector3d(
        current_linear_world.X() + linear_delta.X(),
        current_linear_world.Y() + linear_delta.Y(),
        current_linear_world.Z()));

    const auto current_angular_world = model_->WorldAngularVel();
    const double angular_delta = Clamp(
      twist[2] - current_angular_world.Z(),
      max_angular_accel_ * dt);
    model_->SetAngularVel(
      ignition::math::Vector3d(
        current_angular_world.X(),
        current_angular_world.Y(),
        current_angular_world.Z() + angular_delta));
  }

  void ApplyPoseModel(const std::array<double, 3> & twist)
  {
    InitializePlanarPoseIfNeeded();

    const double now = model_->GetWorld()->SimTime().Double();
    double dt = last_update_time_ > 0.0 ? now - last_update_time_ : 0.001;
    dt = std::max(0.001, std::min(dt, 0.05));
    last_update_time_ = now;

    const ignition::math::Vector3d desired_linear_base(twist[0], twist[1], 0.0);
    const ignition::math::Vector3d current_linear_base(
      current_twist_base_[0],
      current_twist_base_[1],
      0.0);
    const auto linear_delta = ClampVectorLength(
      desired_linear_base - current_linear_base,
      max_linear_accel_ * dt);
    current_twist_base_[0] += linear_delta.X();
    current_twist_base_[1] += linear_delta.Y();
    current_twist_base_[2] += Clamp(
      twist[2] - current_twist_base_[2],
      max_angular_accel_ * dt);

    const ignition::math::Quaterniond planar_rotation(0.0, 0.0, planar_yaw_);
    const ignition::math::Vector3d linear_world = planar_rotation.RotateVector(
      ignition::math::Vector3d(
        current_twist_base_[0],
        current_twist_base_[1],
        0.0));

    planar_x_ += linear_world.X() * dt;
    planar_y_ += linear_world.Y() * dt;
    planar_yaw_ = std::atan2(
      std::sin(planar_yaw_ + current_twist_base_[2] * dt),
      std::cos(planar_yaw_ + current_twist_base_[2] * dt));

    ignition::math::Pose3d next_pose(planar_x_, planar_y_, planar_z_, 0.0, 0.0, planar_yaw_);
    model_->SetWorldPose(next_pose);
    PublishOdometry(next_pose, current_twist_base_);
  }

  void InitializePlanarPoseIfNeeded()
  {
    if (planar_pose_initialized_ || !model_) {
      return;
    }
    const auto pose = model_->WorldPose();
    planar_x_ = pose.Pos().X();
    planar_y_ = pose.Pos().Y();
    planar_z_ = pose.Pos().Z();
    planar_yaw_ = pose.Rot().Yaw();
    planar_pose_initialized_ = true;
  }

  ignition::math::Pose3d PhysicsOdomPose() const
  {
    return model_ ? model_->WorldPose() : ignition::math::Pose3d();
  }

  std::array<double, 3> PhysicsOdomTwist() const
  {
    if (!model_) {
      return {0.0, 0.0, 0.0};
    }
    const auto world_pose = model_->WorldPose();
    const auto linear_base =
      world_pose.Rot().Inverse().RotateVector(model_->WorldLinearVel());
    const auto angular_base =
      world_pose.Rot().Inverse().RotateVector(model_->WorldAngularVel());
    return {
      Clamp(linear_base.X(), max_linear_speed_),
      Clamp(linear_base.Y(), max_linear_speed_),
      Clamp(angular_base.Z(), max_angular_speed_),
    };
  }

  void PublishOdometry(
    const ignition::math::Pose3d & pose,
    const std::array<double, 3> & twist)
  {
    if (!publish_odom_ || !odom_pub_ || !model_) {
      return;
    }

    const auto sim_time = model_->GetWorld()->SimTime();
    const double now = sim_time.Double();
    if (last_odom_publish_time_ > 0.0 && now - last_odom_publish_time_ < odom_publish_period_) {
      return;
    }
    last_odom_publish_time_ = now;

    nav_msgs::msg::Odometry msg;
    msg.header.stamp.sec = static_cast<int32_t>(sim_time.sec);
    msg.header.stamp.nanosec = static_cast<uint32_t>(sim_time.nsec);
    msg.header.frame_id = odom_frame_;
    msg.child_frame_id = base_frame_;
    msg.pose.pose.position.x = pose.Pos().X();
    msg.pose.pose.position.y = pose.Pos().Y();
    msg.pose.pose.position.z = pose.Pos().Z();
    msg.pose.pose.orientation.x = pose.Rot().X();
    msg.pose.pose.orientation.y = pose.Rot().Y();
    msg.pose.pose.orientation.z = pose.Rot().Z();
    msg.pose.pose.orientation.w = pose.Rot().W();
    msg.twist.twist.linear.x = twist[0];
    msg.twist.twist.linear.y = twist[1];
    msg.twist.twist.angular.z = twist[2];
    msg.pose.covariance[0] = 0.02;
    msg.pose.covariance[7] = 0.02;
    msg.pose.covariance[35] = 0.05;
    msg.twist.covariance[0] = 0.02;
    msg.twist.covariance[7] = 0.02;
    msg.twist.covariance[35] = 0.05;
    odom_pub_->publish(msg);
  }

  void ApplyForceModel(const std::array<double, 3> & twist)
  {
    const auto world_pose = base_link_->WorldPose();
    const auto current_linear_base =
      world_pose.Rot().Inverse().RotateVector(base_link_->WorldLinearVel());
    const auto current_angular_base =
      world_pose.Rot().Inverse().RotateVector(base_link_->WorldAngularVel());

    ignition::math::Vector3d velocity_error_base(
      twist[0] - current_linear_base.X(),
      twist[1] - current_linear_base.Y(),
      0.0);
    const auto inertial = base_link_->GetInertial();
    const double mass = inertial ? std::max(1.0, inertial->Mass()) : 1.0;
    const double force_limit = std::max(
      0.0,
      std::min(max_force_, mass * max_linear_accel_));
    const auto force_base = ClampVectorLength(
      velocity_error_base * mass * linear_gain_,
      force_limit);
    const auto force_world = world_pose.Rot().RotateVector(force_base);
    base_link_->AddForce(force_world);

    const double angular_error = twist[2] - current_angular_base.Z();
    const double yaw_inertia = inertial ? std::max(1.0e-6, inertial->IZZ()) : mass;
    const double torque_limit = std::max(
      0.0,
      std::min(max_torque_, yaw_inertia * max_angular_accel_));
    const double torque_z = Clamp(
      yaw_inertia * angular_gain_ * angular_error,
      torque_limit);
    base_link_->AddTorque(ignition::math::Vector3d(0.0, 0.0, torque_z));
  }

  bool AllWheelJointsAvailable() const
  {
    return std::all_of(
      wheel_joints_.begin(),
      wheel_joints_.end(),
      [](const physics::JointPtr & joint) {return static_cast<bool>(joint);});
  }

  std::array<double, 3> TwistFromWheelSpeeds()
  {
    if (!HasRecentWheelCommand()) {
      return {0.0, 0.0, 0.0};
    }

    return TwistFromSpeeds(
      {
        wheel_joints_[0]->GetVelocity(0),
        wheel_joints_[1]->GetVelocity(0),
        wheel_joints_[2]->GetVelocity(0),
        wheel_joints_[3]->GetVelocity(0),
      });
  }

  std::array<double, 3> TwistFromWheelCommands()
  {
    std::array<double, 4> speeds;
    double last_command_time;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      speeds = commanded_wheel_speeds_;
      last_command_time = last_command_time_;
    }

    const double now = model_->GetWorld()->SimTime().Double();
    if (last_command_time < 0.0 || now - last_command_time > command_timeout_) {
      speeds = {0.0, 0.0, 0.0, 0.0};
    }

    return TwistFromSpeeds(speeds);
  }

  std::array<double, 3> TwistFromSpeeds(const std::array<double, 4> & speeds) const
  {
    // 正解公式的单一权威实现在 mecanum_kinematics.hpp(gtest 直接覆盖)
    return lab_cobot_gazebo::TwistFromWheelSpeeds(
      speeds, wheel_radius_, wheelbase_radius_,
      max_linear_speed_, max_angular_speed_);
  }

  physics::ModelPtr model_;
  physics::LinkPtr base_link_;
  event::ConnectionPtr update_connection_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_command_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::string base_link_name_{"base_link"};
  std::string control_mode_{"force_from_wheel_joints"};
  std::string wheel_command_topic_{"/wheel_velocity_controller/commands"};
  std::string odom_topic_{"/odom"};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_footprint"};
  std::array<std::string, 4> wheel_joint_names_{
    "wheel_fl_joint",
    "wheel_fr_joint",
    "wheel_rl_joint",
    "wheel_rr_joint"};
  std::array<physics::JointPtr, 4> wheel_joints_;
  double wheel_radius_{0.08};
  double wheelbase_radius_{0.47};
  double linear_gain_{8.0};
  double angular_gain_{10.0};
  double max_force_{450.0};
  double max_torque_{260.0};
  double max_linear_speed_{0.45};
  double max_angular_speed_{0.9};
  double max_linear_accel_{0.8};
  double max_angular_accel_{1.5};
  double command_timeout_{0.25};
  double odom_publish_period_{0.05};
  double last_update_time_{0.0};
  double last_command_time_{-1.0};
  double last_odom_publish_time_{0.0};
  double planar_x_{0.0};
  double planar_y_{0.0};
  double planar_z_{0.0};
  double planar_yaw_{0.0};
  std::array<double, 3> current_twist_base_{0.0, 0.0, 0.0};
  std::array<double, 4> commanded_wheel_speeds_{0.0, 0.0, 0.0, 0.0};
  std::mutex command_mutex_;
  bool publish_odom_{true};
  bool planar_pose_initialized_{false};
  bool reported_missing_dependencies_{false};
};

GZ_REGISTER_MODEL_PLUGIN(LabCobotMecanumDrive)
}  // namespace gazebo
