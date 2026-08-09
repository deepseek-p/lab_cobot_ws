#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Event.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <rclcpp/rclcpp.hpp>
#include <sdf/sdf.hh>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "lab_cobot_gazebo/attached_object_collision.hpp"
#include "lab_cobot_gazebo/finger_collision_config.hpp"
#include "lab_cobot_gazebo/finger_contact_gate.hpp"
#include "lab_cobot_gazebo/grasp_envelope.hpp"
#include "lab_cobot_gazebo/post_attach_velocity_clamp.hpp"

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

int SdfInt(
  const sdf::ElementPtr & sdf,
  const std::string & key,
  const int fallback)
{
  if (sdf && sdf->HasElement(key)) {
    return sdf->Get<int>(key);
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

std::vector<std::string> SdfStringList(
  const sdf::ElementPtr & sdf,
  const std::string & key,
  const std::string & fallback)
{
  std::vector<std::string> values;
  std::set<std::string> seen;
  if (sdf && sdf->HasElement(key)) {
    auto elem = sdf->GetElement(key);
    while (elem) {
      const auto value = elem->Get<std::string>();
      if (!value.empty() && seen.insert(value).second) {
        values.push_back(value);
      }
      elem = elem->GetNextElement(key);
    }
  }
  if (values.empty()) {
    values.push_back(fallback);
  }
  return values;
}

ignition::math::Vector3d SdfVector3(
  const sdf::ElementPtr & sdf,
  const std::string & key,
  const ignition::math::Vector3d & fallback)
{
  if (sdf && sdf->HasElement(key)) {
    return sdf->Get<ignition::math::Vector3d>(key);
  }
  return fallback;
}
}  // namespace

class LabCobotGraspFix : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model_->GetWorld();
    object_model_names_ = SdfStringList(sdf, "object_model", "aruco_sample");
    object_link_name_ = SdfString(sdf, "object_link", "link");
    tcp_link_name_ = SdfString(sdf, "tcp_link", "gripper_tcp");
    left_joint_name_ = SdfString(sdf, "left_joint", "gripper_left_finger_joint");
    right_joint_name_ = SdfString(sdf, "right_joint", "gripper_right_finger_joint");
    stable_attach_link_name_ = SdfString(sdf, "stable_attach_link", "ur_wrist_3_link");
    close_threshold_ = SdfDouble(sdf, "close_threshold", 0.006);
    release_threshold_ = SdfDouble(sdf, "release_threshold", 0.003);
    max_center_distance_ = SdfDouble(sdf, "max_center_distance", 0.080);
    max_abs_x_ = SdfDouble(sdf, "max_abs_x", 0.040);
    max_abs_y_ = SdfDouble(sdf, "max_abs_y", 0.018);
    min_z_ = SdfDouble(sdf, "min_z", -0.060);
    max_z_ = SdfDouble(sdf, "max_z", 0.025);
    grip_count_threshold_ = std::max(1, SdfInt(sdf, "grip_count_threshold", 3));
    require_finger_contact_ = SdfBool(sdf, "require_finger_contact", false);
    enable_contact_force_ = SdfBool(sdf, "enable_contact_force", false);
    virtual_force_sensor_ = SdfBool(sdf, "virtual_force_sensor", true);
    virtual_force_stiffness_ = SdfDouble(sdf, "virtual_force_stiffness", 4500.0);
    virtual_force_baseline_ = SdfDouble(sdf, "virtual_force_baseline", 0.3);
    virtual_force_max_ = SdfDouble(sdf, "virtual_force_max", 20.0);
    contact_count_threshold_ = std::max(1, SdfInt(sdf, "contact_count_threshold", 3));
    // 约束力保险丝默认禁用(0):底盘为 SetWorldPose 位姿驱动,搬运途中
    // 机器人每 tick 被瞬移,fixed joint 的 ERP 校正力在正常搬运时即达
    // 数百 N(实测坐台共存 110N/搬运中可更高),与压入台面的异常力无法
    // 用阈值区分,启用会把物块半路丢掉。压桌爆炸由 place 悬空释放
    // (PLACE_RELEASE_CLEARANCE)机制性根治,不依赖本保险丝。
    breakaway_force_ = SdfDouble(sdf, "breakaway_force", 0.0);
    breakaway_count_threshold_ =
      std::max(1, SdfInt(sdf, "breakaway_count_threshold", 3));
    grasp_center_offset_ = SdfVector3(
      sdf,
      "grasp_center_offset",
      ignition::math::Vector3d(0.0, 0.0, 0.0));
    contact_status_topic_ = SdfString(sdf, "contact_status_topic", "/gripper/contact/status");
    contact_release_topic_ = SdfString(sdf, "contact_release_topic", "/gripper/contact/release");
    hold_status_topic_ = SdfString(
      sdf, "hold_status_topic", "/gripper/contact/hold_status");

    ros_node_ = gazebo_ros::Node::Get(sdf);
    auto contact_manager = world_->Physics()->GetContactManager();
    if (contact_manager) {
      contact_manager->SetNeverDropContacts(true);
    }
    if (lab_cobot_gazebo::ShouldDisablePrimaryFingerCollisions(require_finger_contact_)) {
      ConfigurePrimaryFingerCollisions();
    }
    ConfigureTactileProbeCollisions();
    contact_status_pub_ = ros_node_->create_publisher<std_msgs::msg::String>(
      contact_status_topic_,
      rclcpp::QoS(10));
    // per-finger 接触快照(50Hz):bumper 对 probe 接触上报率过低(实测 1/50),
    // driver 分侧停步以本 topic 为主信号,插件 1kHz ContactManager 为权威源。
    fingers_status_pub_ = ros_node_->create_publisher<std_msgs::msg::String>(
      "/gripper/contact/fingers",
      rclcpp::QoS(10));
    // 指尖力曲线(N):enable_contact_force=true 时优先发布 Gazebo
    // ContactManager 原始 wrench；稳定 A→B 默认用非侵入式虚拟指尖力,
    // 避免物理 probe 接触响应把样块顶飞。
    contact_force_pub_ = ros_node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/gripper/contact/force", rclcpp::QoS(10));
    // 持有心跳由抓取插件（fixed joint 的唯一所有者）发布；消费方不能只凭
    // 一次 "attached" 事件就假定物块仍在手上。
    hold_status_pub_ = ros_node_->create_publisher<std_msgs::msg::String>(
      hold_status_topic_, rclcpp::QoS(10));
    contact_release_sub_ = ros_node_->create_subscription<std_msgs::msg::Empty>(
      contact_release_topic_,
      rclcpp::QoS(10),
      [this](std_msgs::msg::Empty::SharedPtr) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        release_requested_ = true;
      });

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&LabCobotGraspFix::OnUpdate, this));
    gzmsg << "lab_cobot_grasp_fix loaded for "
          << object_model_names_.size() << " grasp candidate(s)" << std::endl;
  }

private:
  struct FingerGraspFrame
  {
    ignition::math::Vector3d position;
    ignition::math::Quaterniond rotation;
    physics::LinkPtr attach_link;
    bool valid{false};
  };

  void OnUpdate()
  {
    auto left_joint = model_->GetJoint(left_joint_name_);
    auto right_joint = model_->GetJoint(right_joint_name_);
    if (!left_joint || !right_joint) {
      return;
    }
    PublishFingerForcesIfDue(left_joint, right_joint);

    const bool release_requested = ConsumeReleaseRequested();
    const bool open =
      left_joint->Position(0) <= release_threshold_ &&
      right_joint->Position(0) <= release_threshold_;
    if (fixed_joint_) {
      EnforceAttachedObjectPose();
      if (release_requested || open) {
        DetachObject();
        suppress_attach_until_open_ = true;
        return;
      }
      if (!HoldingPoseIsValid()) {
        PublishHoldStatus("lost " + attached_object_name_ + " relative_pose_invalid");
        DetachObject("hold_pose_invalid");
        suppress_attach_until_open_ = true;
        return;
      }
      PublishHoldStatusIfDue();
      const auto clamp_step = lab_cobot_gazebo::AdvancePostAttachVelocityClamp(
        post_attach_velocity_clamp_remaining_steps_);
      post_attach_velocity_clamp_remaining_steps_ = clamp_step.remaining_steps;
      if (clamp_step.should_clamp && attached_object_link_) {
        attached_object_link_->SetLinearVel(ignition::math::Vector3d::Zero);
        attached_object_link_->SetAngularVel(ignition::math::Vector3d::Zero);
      }
      // 约束力保险丝(默认禁用,见 Load 中说明):仅当显式配置
      // breakaway_force > 0 时启用;连续计数滤除单帧数值脉冲。
      if (breakaway_force_ > 0.0) {
        const auto wrench = fixed_joint_->GetForceTorque(0);
        const double reaction = std::max(
          wrench.body1Force.Length(),
          wrench.body2Force.Length());
        if (reaction > breakaway_force_) {
          breakaway_count_ += 1;
          if (breakaway_count_ >= breakaway_count_threshold_) {
            gzwarn << "lab_cobot_grasp_fix breakaway: joint reaction "
                   << reaction << " N exceeds " << breakaway_force_
                   << " N for " << breakaway_count_
                   << " ticks, detaching " << attached_object_name_ << std::endl;
            DetachObject("breakaway");
            suppress_attach_until_open_ = true;
          }
        } else {
          breakaway_count_ = 0;
        }
      }
      return;
    }
    if (require_finger_contact_ && fingers_status_pub_) {
      // 50Hz 节流;含封套外场景(手指已触但候选被封套拒时 driver 同样需要停步)。
      if (++fingers_publish_countdown_ >= 20) {
        fingers_publish_countdown_ = 0;
        const auto finger_pairs = CurrentContactPairs();
        const auto left_name = left_joint->GetChild()->GetName();
        const auto right_name = right_joint->GetChild()->GetName();
        bool left_touch = false;
        bool right_touch = false;
        for (const auto & target : object_model_names_) {
          for (const auto & pair : finger_pairs) {
            left_touch = left_touch || lab_cobot_gazebo::FingerTouchesTarget(
              pair, model_->GetName(), left_name, target);
            right_touch = right_touch || lab_cobot_gazebo::FingerTouchesTarget(
              pair, model_->GetName(), right_name, target);
          }
        }
        std_msgs::msg::String fingers_msg;
        fingers_msg.data = std::string("fingers left=") +
          (left_touch ? "1" : "0") + " right=" + (right_touch ? "1" : "0");
        fingers_status_pub_->publish(fingers_msg);
      }
    }
    if (release_requested) {
      suppress_attach_until_open_ = true;
      grip_count_ = 0;
      finger_contact_count_ = 0;
      PublishStatus("released none");
      return;
    }
    if (open) {
      suppress_attach_until_open_ = false;
      grip_count_ = 0;
      finger_contact_count_ = 0;
      return;
    }
    if (suppress_attach_until_open_) {
      grip_count_ = 0;
      finger_contact_count_ = 0;
      return;
    }

    const bool closed =
      left_joint->Position(0) >= close_threshold_ &&
      right_joint->Position(0) >= close_threshold_;
    if (!closed) {
      grip_count_ = 0;
      return;
    }

    const auto grasp_frame = FingerGraspFrameFromJoints(left_joint, right_joint);
    if (!grasp_frame.valid) {
      grip_count_ = 0;
      finger_contact_count_ = 0;
      PublishRefusedStatus("invalid_grasp_frame");
      return;
    }
    GraspCandidate candidate;
    ignition::math::Vector3d refused_offset;
    std::string refused_name;
    if (!NearestCandidateInsideGraspEnvelope(
        grasp_frame, &candidate, &refused_name, &refused_offset))
    {
      grip_count_ = 0;
      finger_contact_count_ = 0;
      if (refused_name.empty()) {
        PublishRefusedStatus("none", "no_candidate_model");
      } else {
        PublishRefusedStatus(refused_name, FormatOffset(refused_offset));
      }
      return;
    }

    if (pending_object_name_ != candidate.name) {
      pending_object_name_ = candidate.name;
      grip_count_ = 0;
      finger_contact_count_ = 0;
    }
    if (require_finger_contact_) {
      // 判定期每 tick 清零(2026-07-13 复核保留):双 probe 接触瞬间存在引擎级
      // 间歇速度爆发(1kHz 插桩实证,与清零/穿透深浅/分侧停均无关,五种干预
      // 实测均无法根治)。清零的实际作用是位移抑制——把爆发按在原地使抓取
      // 继续(E2E 4/5 最稳版本);配合 fingers 快照直连的分侧停(压穿透、削
      // 能量积累),大幅爆发(曾 GUI 下打飞 7m)频率显著降低。
      candidate.link->SetLinearVel(ignition::math::Vector3d::Zero);
      candidate.link->SetAngularVel(ignition::math::Vector3d::Zero);
      const auto pairs = CurrentContactPairs();
      const auto touch_state = lab_cobot_gazebo::FingerTouchStateForTarget(
        pairs,
        model_->GetName(),
        left_joint->GetChild()->GetName(),
        right_joint->GetChild()->GetName(),
        candidate.name);
      const double now = world_ ? world_->SimTime().Double() : 0.0;
      if (touch_state.left) {
        last_left_finger_contact_time_ = now;
      }
      if (touch_state.right) {
        last_right_finger_contact_time_ = now;
      }
      // Gazebo contact pairs can flicker between physics ticks while the ROS-side
      // tactile driver still observes both fingers within a fresh 0.2s window.
      // Use the same short freshness window here so a real two-sided touch is not
      // lost before the autonomous attach decision runs.
      const bool touching =
        (now - last_left_finger_contact_time_ <= kFingerContactFreshSec) &&
        (now - last_right_finger_contact_time_ <= kFingerContactFreshSec);
      if (!touching) {
        finger_contact_count_ = 0;
        grip_count_ = 0;
        PublishRefusedStatus(candidate.name, "no_finger_contact");
        return;
      }
      finger_contact_count_ += 1;
      if (finger_contact_count_ < contact_count_threshold_) {
        grip_count_ = 0;
        return;
      }
    }
    grip_count_ += 1;
    if (grip_count_ >= grip_count_threshold_) {
      AttachObject(left_joint, right_joint, candidate.name);
    }
  }

  FingerGraspFrame FingerGraspFrameFromJoints(
    const physics::JointPtr & left_joint,
    const physics::JointPtr & right_joint) const
  {
    FingerGraspFrame frame;
    auto left_link = left_joint->GetChild();
    auto right_link = right_joint->GetChild();
    if (!left_link || !right_link) {
      return frame;
    }

    frame.rotation = left_link->WorldPose().Rot();
    // 抓取系几何的单一权威实现在 grasp_envelope.hpp(gtest 直接覆盖)
    frame.position = lab_cobot_gazebo::GraspFramePosition(
      left_link->WorldPose().Pos(),
      right_link->WorldPose().Pos(),
      frame.rotation,
      grasp_center_offset_);
    frame.attach_link = left_link;
    frame.valid = true;
    return frame;
  }

  struct GraspCandidate
  {
    std::string name;
    physics::LinkPtr link;
    ignition::math::Vector3d offset;
  };

  bool NearestCandidateInsideGraspEnvelope(
    const FingerGraspFrame & grasp_frame,
    GraspCandidate * candidate_out,
    std::string * nearest_name_out = nullptr,
    ignition::math::Vector3d * nearest_offset_out = nullptr) const
  {
    std::vector<GraspCandidate> candidates;
    std::vector<ignition::math::Vector3d> offsets;
    for (const auto & name : object_model_names_) {
      auto object_model = world_->ModelByName(name);
      if (!object_model || object_model->IsStatic()) {
        continue;
      }
      auto object_link = object_model->GetLink(object_link_name_);
      if (!object_link) {
        continue;
      }
      const auto offset = lab_cobot_gazebo::ObjectOffsetInGraspFrame(
        object_link->WorldPose().Pos(),
        grasp_frame.position,
        grasp_frame.rotation);
      candidates.push_back({name, object_link, offset});
      offsets.push_back(offset);
    }
    if (candidates.empty()) {
      return false;
    }
    int nearest_any_index = 0;
    for (std::size_t index = 1; index < offsets.size(); ++index) {
      if (offsets[index].Length() < offsets[nearest_any_index].Length()) {
        nearest_any_index = static_cast<int>(index);
      }
    }
    if (nearest_name_out) {
      *nearest_name_out = candidates[nearest_any_index].name;
    }
    if (nearest_offset_out) {
      *nearest_offset_out = candidates[nearest_any_index].offset;
    }
    const auto selected_index = lab_cobot_gazebo::NearestOffsetIndexInsideEnvelope(
      offsets,
      {max_center_distance_, max_abs_x_, max_abs_y_, min_z_, max_z_});
    if (selected_index < 0) {
      return false;
    }
    if (candidate_out) {
      *candidate_out = candidates[static_cast<std::size_t>(selected_index)];
    }
    return true;
  }

  void AttachObject(
    const physics::JointPtr & left_joint,
    const physics::JointPtr & right_joint,
    const std::string & object_name)
  {
    const auto grasp_frame = FingerGraspFrameFromJoints(left_joint, right_joint);
    auto attach_link = grasp_frame.attach_link;
    auto stable_attach_link = model_->GetLink(stable_attach_link_name_);
    const auto attach_link_name = lab_cobot_gazebo::SelectAttachLinkName(
      require_finger_contact_,
      attach_link ? attach_link->GetName() : "",
      stable_attach_link ? stable_attach_link->GetName() : "");
    if (attach_link_name != (attach_link ? attach_link->GetName() : "")) {
      attach_link = model_->GetLink(attach_link_name);
    }
    auto object_model = world_->ModelByName(object_name);
    if (!attach_link || !object_model) {
      return;
    }
    auto object_link = object_model->GetLink(object_link_name_);
    if (!object_link) {
      return;
    }

    const int collision_count = ApplyObjectCollisionResponse(
      object_link,
      lab_cobot_gazebo::AttachedObjectCollisionPhase::kFixedJointHeld);
    gzmsg << "lab_cobot_grasp_fix disabled collision response for "
          << object_name << " (" << collision_count << " collision(s))"
          << std::endl;
    fixed_joint_ = world_->Physics()->CreateJoint("fixed", model_);
    fixed_joint_->Load(attach_link, object_link, ignition::math::Pose3d());
    fixed_joint_->Init();
    // 开启约束反力回读,供 breakaway 保险丝使用(ODE 默认不回填 feedback)
    fixed_joint_->SetProvideFeedback(true);
    object_link->SetLinearVel(ignition::math::Vector3d::Zero);
    object_link->SetAngularVel(ignition::math::Vector3d::Zero);
    attached_object_link_ = object_link;
    attached_reference_link_ = attach_link;
    attached_relative_pose_ =
      attach_link->WorldPose().Inverse() * object_link->WorldPose();
    last_hold_status_time_ = -1.0;
    post_attach_velocity_clamp_remaining_steps_ = kPostAttachVelocityClampSteps;
    attached_object_name_ = object_name;
    pending_object_name_.clear();
    PublishStatus("attached " + object_name);
    gzmsg << "lab_cobot_grasp_fix attached " << object_name
          << " to " << attach_link->GetName()
          << " with " << kPostAttachVelocityClampSteps
          << " post-attach velocity clamp step(s)" << std::endl;
  }

  void DetachObject(const std::string & reason = "")
  {
    physics::LinkPtr object_link;
    const auto object_name = attached_object_name_.empty() ? "none" : attached_object_name_;
    auto object_model = world_->ModelByName(object_name);
    if (object_model) {
      object_link = object_model->GetLink(object_link_name_);
    }

    fixed_joint_->Detach();
    fixed_joint_.reset();
    post_attach_velocity_clamp_remaining_steps_ = 0;
    if (object_link) {
      const int collision_count = ApplyObjectCollisionResponse(
        object_link,
        lab_cobot_gazebo::AttachedObjectCollisionPhase::kReleased);
      gzmsg << "lab_cobot_grasp_fix restored collision response for "
            << object_name << " (" << collision_count << " collision(s))"
            << std::endl;
      object_link->SetLinearVel(ignition::math::Vector3d::Zero);
      object_link->SetAngularVel(ignition::math::Vector3d::Zero);
    }
    grip_count_ = 0;
    finger_contact_count_ = 0;
    breakaway_count_ = 0;
    PublishStatus(
      "released " + object_name +
      (reason.empty() ? "" : " " + reason));
    gzmsg << "lab_cobot_grasp_fix released " << object_name
          << (reason.empty() ? "" : " (" + reason + ")") << std::endl;
    attached_object_name_.clear();
    attached_object_link_.reset();
    attached_reference_link_.reset();
    last_hold_status_time_ = -1.0;
    pending_object_name_.clear();
  }

  int ApplyObjectCollisionResponse(
    const physics::LinkPtr & object_link,
    lab_cobot_gazebo::AttachedObjectCollisionPhase phase) const
  {
    if (!object_link) {
      return 0;
    }
    const auto settings = lab_cobot_gazebo::ObjectCollisionResponseForPhase(phase);
    int configured = 0;
    for (const auto & collision : object_link->GetCollisions()) {
      if (!collision) {
        continue;
      }
      auto surface = collision->GetSurface();
      if (surface) {
        surface->collideBitmask = settings.surface_collide_bitmask;
      }
      collision->SetCollideBits(settings.collide_bits);
      configured += 1;
    }
    return configured;
  }

  bool ConsumeReleaseRequested()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const bool requested = release_requested_;
    release_requested_ = false;
    return requested;
  }

  void PublishStatus(const std::string & status)
  {
    if (!contact_status_pub_) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = status;
    contact_status_pub_->publish(msg);
  }

  bool HoldingPoseIsValid() const
  {
    if (!fixed_joint_ || !attached_object_link_ || !attached_reference_link_) {
      return false;
    }
    const auto relative_pose =
      attached_reference_link_->WorldPose().Inverse() * attached_object_link_->WorldPose();
    const double position_error =
      (relative_pose.Pos() - attached_relative_pose_.Pos()).Length();
    const auto rotation_error =
      attached_relative_pose_.Rot().Inverse() * relative_pose.Rot();
    // ignition-math v6 没有 Quaternion::Angle();单位四元数的旋转角为
    // 2*acos(|w|)，abs 同时消除 q 与 -q 的等价表示。
    const double orientation_error = 2.0 * std::acos(
      std::min(
        1.0, std::abs(rotation_error.W())));
    return position_error <= hold_max_position_error_ &&
           orientation_error <= hold_max_orientation_error_;
  }

  void EnforceAttachedObjectPose() const
  {
    if (!attached_object_link_ || !attached_reference_link_) {
      return;
    }
    attached_object_link_->SetWorldPose(
      attached_reference_link_->WorldPose() * attached_relative_pose_);
    attached_object_link_->SetLinearVel(ignition::math::Vector3d::Zero);
    attached_object_link_->SetAngularVel(ignition::math::Vector3d::Zero);
  }

  void PublishHoldStatusIfDue()
  {
    const double now = world_ ? world_->SimTime().Double() : 0.0;
    if (last_hold_status_time_ >= 0.0 &&
      now - last_hold_status_time_ < hold_status_period_)
    {
      return;
    }
    last_hold_status_time_ = now;
    PublishHoldStatus("holding " + attached_object_name_);
  }

  void PublishHoldStatus(const std::string & status)
  {
    if (!hold_status_pub_) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = status;
    hold_status_pub_->publish(msg);
  }

  std::vector<lab_cobot_gazebo::ContactPair> CurrentContactPairs() const
  {
    std::vector<lab_cobot_gazebo::ContactPair> pairs;
    if (!world_ || !world_->Physics()) {
      return pairs;
    }
    const auto contact_manager = world_->Physics()->GetContactManager();
    if (!contact_manager) {
      return pairs;
    }
    const auto count = contact_manager->GetContactCount();
    const auto & contacts = contact_manager->GetContacts();
    for (unsigned int index = 0; index < count && index < contacts.size(); ++index) {
      const auto contact = contacts[index];
      if (!contact || !contact->collision1 || !contact->collision2) {
        continue;
      }
      const auto collision1 = contact->collision1;
      const auto collision2 = contact->collision2;
      const auto model1 = collision1->GetModel();
      const auto model2 = collision2->GetModel();
      const auto link1 = collision1->GetLink();
      const auto link2 = collision2->GetLink();
      if (!model1 || !model2 || !link1 || !link2) {
        continue;
      }
      pairs.push_back(
        {model1->GetName(), link1->GetName(), model2->GetName(), link2->GetName()});
    }
    return pairs;
  }

  void PublishFingerForcesIfDue(
    const physics::JointPtr & left_joint,
    const physics::JointPtr & right_joint)
  {
    if (!contact_force_pub_) {
      return;
    }
    // 力采集模式按每个仿真步发布；双指接触的峰值可能短于 20ms，50Hz
    // 节流会把真实接触脉冲完全漏掉。默认模式仍保持 50Hz 低开销心跳。
    if (!enable_contact_force_ && ++force_publish_countdown_ < 20) {
      return;
    }
    force_publish_countdown_ = 0;
    double left_force = 0.0;
    double right_force = 0.0;
    const auto contact_manager = world_ && world_->Physics() ?
      world_->Physics()->GetContactManager() : nullptr;
    if (contact_manager) {
      const auto count = contact_manager->GetContactCount();
      const auto & contacts = contact_manager->GetContacts();
      const auto left_name = left_joint->GetChild()->GetName();
      const auto right_name = right_joint->GetChild()->GetName();
      for (unsigned int index = 0; index < count && index < contacts.size(); ++index) {
        const auto contact = contacts[index];
        if (!contact || !contact->collision1 || !contact->collision2) {
          continue;
        }
        const auto collision1 = contact->collision1;
        const auto collision2 = contact->collision2;
        const auto model1 = collision1->GetModel();
        const auto model2 = collision2->GetModel();
        const auto link1 = collision1->GetLink();
        const auto link2 = collision2->GetLink();
        if (!model1 || !model2 || !link1 || !link2) {
          continue;
        }
        const bool target1 = std::find(
          object_model_names_.begin(), object_model_names_.end(), model1->GetName()) !=
          object_model_names_.end();
        const bool target2 = std::find(
          object_model_names_.begin(), object_model_names_.end(), model2->GetName()) !=
          object_model_names_.end();
        const bool left1 = model1->GetName() == model_->GetName() && link1->GetName() == left_name;
        const bool left2 = model2->GetName() == model_->GetName() && link2->GetName() == left_name;
        const bool right1 = model1->GetName() == model_->GetName() &&
          link1->GetName() == right_name;
        const bool right2 = model2->GetName() == model_->GetName() &&
          link2->GetName() == right_name;
        if (!((left1 || left2 || right1 || right2) && (target1 || target2))) {
          continue;
        }
        for (int wrench_index = 0; wrench_index < contact->count; ++wrench_index) {
          if (left1) {
            left_force += contact->wrench[wrench_index].body1Force.Length();
          } else if (left2) {
            left_force += contact->wrench[wrench_index].body2Force.Length();
          }
          if (right1) {
            right_force += contact->wrench[wrench_index].body1Force.Length();
          } else if (right2) {
            right_force += contact->wrench[wrench_index].body2Force.Length();
          }
        }
      }
    }
    if (virtual_force_sensor_ && !enable_contact_force_) {
      left_force = std::max(
        left_force,
        VirtualFingerForce(left_joint, last_left_finger_contact_time_));
      right_force = std::max(
        right_force,
        VirtualFingerForce(right_joint, last_right_finger_contact_time_));
    }
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {left_force, right_force};
    contact_force_pub_->publish(msg);
  }

  double VirtualFingerForce(
    const physics::JointPtr & joint,
    const double last_contact_time) const
  {
    if (!virtual_force_sensor_ || !joint || !world_) {
      return 0.0;
    }
    const double now = world_->SimTime().Double();
    const bool fresh_contact =
      now - last_contact_time <= kFingerContactFreshSec;
    // 抓住后 fixed_joint_ 才是“仍在持有”的权威信号；接触 pair 可能因
    // collideWithoutContact 或物块被约束而短暂消失,但闭合指位仍代表夹持力。
    if (!fresh_contact && !fixed_joint_) {
      return 0.0;
    }
    const double compression = std::max(0.0, joint->Position(0) - close_threshold_);
    if (compression <= 0.0) {
      return 0.0;
    }
    return std::min(
      virtual_force_max_,
      virtual_force_baseline_ + virtual_force_stiffness_ * compression);
  }

  void ConfigureTactileProbeCollisions()
  {
    if (!model_) {
      return;
    }
    int configured = 0;
    for (const auto & link : model_->GetLinks()) {
      if (!link) {
        continue;
      }
      for (const auto & collision : link->GetCollisions()) {
        if (!collision) {
          continue;
        }
        if (!lab_cobot_gazebo::IsTactileProbeCollisionName(collision->GetName())) {
          continue;
        }
        auto surface = collision->GetSurface();
        if (!surface) {
          continue;
        }
        const auto config = lab_cobot_gazebo::TactileProbeSurfaceConfig(
          enable_contact_force_);
        surface->collideWithoutContact = config.collide_without_contact;
        surface->collideWithoutContactBitmask =
          config.collide_without_contact_bitmask;
        collision->SetCollideBits(config.collide_bits);
        collision->SetMaxContacts(config.max_contacts);
        configured += 1;
      }
    }
    if (configured > 0) {
      gzmsg << "lab_cobot_grasp_fix configured "
            << configured << " tactile probe collision(s)"
            << (enable_contact_force_ ? " with physical contact response" : " as contact-only")
            << std::endl;
    }
  }

  void ConfigurePrimaryFingerCollisions()
  {
    if (!model_) {
      return;
    }
    int configured = 0;
    for (const auto & link : model_->GetLinks()) {
      if (!link) {
        continue;
      }
      for (const auto & collision : link->GetCollisions()) {
        if (!collision) {
          continue;
        }
        if (!lab_cobot_gazebo::IsPrimaryFingerCollisionName(collision->GetName())) {
          continue;
        }
        const auto config = lab_cobot_gazebo::PrimaryFingerCollisionConfig();
        auto surface = collision->GetSurface();
        if (surface) {
          surface->collideBitmask = config.surface_collide_bitmask;
        }
        collision->SetCollideBits(config.collide_bits);
        configured += 1;
      }
    }
    if (configured > 0) {
      gzmsg << "lab_cobot_grasp_fix disabled "
            << configured
            << " primary finger collision(s); tactile probes remain active"
            << std::endl;
    }
  }

  void PublishRefusedStatus(
    const std::string & object_name,
    const std::string & reason)
  {
    const double now = world_ ? world_->SimTime().Double() : 0.0;
    if (
      last_refused_status_time_ >= 0.0 &&
      now - last_refused_status_time_ < refused_status_period_)
    {
      return;
    }
    last_refused_status_time_ = now;
    PublishStatus("refused " + object_name + " " + reason);
  }

  void PublishRefusedStatus(const std::string & reason)
  {
    PublishRefusedStatus(
      pending_object_name_.empty() ? object_model_names_.front() : pending_object_name_,
      reason);
  }

  std::string FormatOffset(const ignition::math::Vector3d & offset) const
  {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3)
           << "offset=(" << offset.X()
           << "," << offset.Y()
           << "," << offset.Z() << ")";
    return stream.str();
  }

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  event::ConnectionPtr update_connection_;
  physics::JointPtr fixed_joint_;
  physics::LinkPtr attached_object_link_;
  physics::LinkPtr attached_reference_link_;
  ignition::math::Pose3d attached_relative_pose_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr contact_status_pub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr contact_release_sub_;
  std::vector<std::string> object_model_names_;
  std::string attached_object_name_;
  std::string pending_object_name_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fingers_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr contact_force_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hold_status_pub_;
  int fingers_publish_countdown_{0};
  int force_publish_countdown_{0};
  std::string object_link_name_;
  std::string tcp_link_name_;
  std::string left_joint_name_;
  std::string right_joint_name_;
  std::string stable_attach_link_name_;
  std::string contact_status_topic_;
  std::string contact_release_topic_;
  std::string hold_status_topic_;
  double close_threshold_{0.006};
  double release_threshold_{0.003};
  double max_center_distance_{0.080};
  double max_abs_x_{0.040};
  double max_abs_y_{0.018};
  double min_z_{-0.060};
  double max_z_{0.025};
  int grip_count_threshold_{3};
  int grip_count_{0};
  bool require_finger_contact_{false};
  bool enable_contact_force_{false};
  bool virtual_force_sensor_{true};
  double virtual_force_stiffness_{4500.0};
  double virtual_force_baseline_{0.3};
  double virtual_force_max_{20.0};
  int contact_count_threshold_{3};
  int finger_contact_count_{0};
  static constexpr int kPostAttachVelocityClampSteps = 10;
  static constexpr double kFingerContactFreshSec = 0.2;
  int post_attach_velocity_clamp_remaining_steps_{0};
  double breakaway_force_{0.0};
  int breakaway_count_threshold_{3};
  int breakaway_count_{0};
  ignition::math::Vector3d grasp_center_offset_{0.0, 0.0, 0.0};
  double refused_status_period_{0.2};
  double last_refused_status_time_{-1.0};
  double hold_status_period_{0.1};
  double last_hold_status_time_{-1.0};
  double last_left_finger_contact_time_{-1.0e9};
  double last_right_finger_contact_time_{-1.0e9};
  double hold_max_position_error_{0.05};
  double hold_max_orientation_error_{0.50};
  std::mutex state_mutex_;
  bool release_requested_{false};
  bool suppress_attach_until_open_{false};
};

GZ_REGISTER_MODEL_PLUGIN(LabCobotGraspFix)
}  // namespace gazebo
