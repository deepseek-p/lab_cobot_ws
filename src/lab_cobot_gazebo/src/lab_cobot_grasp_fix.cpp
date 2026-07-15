#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
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
#include <std_msgs/msg/string.hpp>

#include "lab_cobot_gazebo/canonical_attach.hpp"
#include "lab_cobot_gazebo/finger_collision_config.hpp"
#include "lab_cobot_gazebo/finger_contact_gate.hpp"
#include "lab_cobot_gazebo/grasp_envelope.hpp"

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
    tcp_parent_link_name_ = SdfString(sdf, "tcp_parent_link", "ur_wrist_3_link");
    tcp_offset_local_z_ = SdfDouble(sdf, "tcp_offset_local_z", 0.105);
    left_joint_name_ = SdfString(sdf, "left_joint", "gripper_left_finger_joint");
    right_joint_name_ = SdfString(sdf, "right_joint", "gripper_right_finger_joint");
    stable_attach_link_name_ = SdfString(sdf, "stable_attach_link", "ur_wrist_3_link");
    object_offset_local_z_ = SdfDouble(sdf, "object_offset_local_z", 0.045);
    attach_clearance_ = SdfDouble(sdf, "attach_clearance", 0.003);
    max_attach_correction_ = SdfDouble(sdf, "max_attach_correction", 0.020);
    if (!std::isfinite(tcp_offset_local_z_) || tcp_offset_local_z_ < 0.0 ||
      !std::isfinite(object_offset_local_z_) || object_offset_local_z_ <= 0.0 ||
      !std::isfinite(attach_clearance_) || attach_clearance_ < 0.0 ||
      attach_clearance_ >= object_offset_local_z_ ||
      !std::isfinite(max_attach_correction_) || max_attach_correction_ <= 0.0)
    {
      throw std::runtime_error(
              "invalid canonical attach offsets or correction limit");
    }
    close_threshold_ = SdfDouble(sdf, "close_threshold", 0.006);
    release_threshold_ = SdfDouble(sdf, "release_threshold", 0.003);
    max_center_distance_ = SdfDouble(sdf, "max_center_distance", 0.080);
    max_abs_x_ = SdfDouble(sdf, "max_abs_x", 0.040);
    max_abs_y_ = SdfDouble(sdf, "max_abs_y", 0.018);
    min_z_ = SdfDouble(sdf, "min_z", -0.060);
    max_z_ = SdfDouble(sdf, "max_z", 0.025);
    grip_count_threshold_ = std::max(1, SdfInt(sdf, "grip_count_threshold", 3));
    require_finger_contact_ = SdfBool(sdf, "require_finger_contact", false);
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

    const bool release_requested = ConsumeReleaseRequested();
    const bool open =
      left_joint->Position(0) <= release_threshold_ &&
      right_joint->Position(0) <= release_threshold_;
    if (fixed_joint_) {
      if (release_requested || open) {
        DetachObject();
        suppress_attach_until_open_ = true;
        return;
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
      candidate.link->SetLinearVel(ignition::math::Vector3d::Zero);
      candidate.link->SetAngularVel(ignition::math::Vector3d::Zero);
      const auto pairs = CurrentContactPairs();
      const bool touching = lab_cobot_gazebo::BothFingersTouching(
        pairs,
        model_->GetName(),
        left_joint->GetChild()->GetName(),
        right_joint->GetChild()->GetName(),
        candidate.name);
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

    auto tcp_link = model_->GetLink(tcp_link_name_);
    ignition::math::Pose3d tcp_pose;
    if (tcp_link) {
      tcp_pose = tcp_link->WorldPose();
    } else {
      auto tcp_parent_link = model_->GetLink(tcp_parent_link_name_);
      if (tcp_parent_link) {
        tcp_pose = lab_cobot_gazebo::PoseAtParentLocalZ(
          tcp_parent_link->WorldPose(), tcp_offset_local_z_);
      } else {
        grip_count_ = 0;
        finger_contact_count_ = 0;
        suppress_attach_until_open_ = true;
        PublishRefusedStatus(object_name, "missing_tcp_parent_link");
        return;
      }
    }
    const auto current_pose = object_link->WorldPose();
    const auto canonical_pose = lab_cobot_gazebo::CanonicalAttachedObjectPose(
      tcp_pose, object_offset_local_z_ - attach_clearance_, current_pose.Rot());
    const double correction = current_pose.Pos().Distance(canonical_pose.Pos());
    if (!lab_cobot_gazebo::AttachCorrectionAllowed(
        current_pose.Pos(), canonical_pose.Pos(), max_attach_correction_))
    {
      grip_count_ = 0;
      finger_contact_count_ = 0;
      suppress_attach_until_open_ = true;
      std::ostringstream reason;
      reason << std::fixed << std::setprecision(4)
             << "canonical_correction=" << correction;
      PublishRefusedStatus(object_name, reason.str());
      return;
    }
    const auto original_linear_velocity = object_link->WorldLinearVel();
    const auto original_angular_velocity = object_link->WorldAngularVel();
    const auto rollback = [&]() {
      RestoreObjectCollisions();
      object_link->SetWorldPose(current_pose);
      object_link->SetLinearVel(original_linear_velocity);
      object_link->SetAngularVel(original_angular_velocity);
      grip_count_ = 0;
      finger_contact_count_ = 0;
      suppress_attach_until_open_ = true;
    };
    object_link->SetWorldPose(canonical_pose);
    object_link->SetLinearVel(ignition::math::Vector3d::Zero);
    object_link->SetAngularVel(ignition::math::Vector3d::Zero);
    auto new_joint = world_->Physics()->CreateJoint("fixed", model_);
    if (!new_joint) {
      rollback();
      PublishRefusedStatus(object_name, "fixed_joint_create_failed");
      return;
    }
    SuppressObjectCollisions(object_link);
    try {
      new_joint->Load(attach_link, object_link, ignition::math::Pose3d());
      new_joint->Init();
    // 开启约束反力回读,供 breakaway 保险丝使用(ODE 默认不回填 feedback)
      new_joint->SetProvideFeedback(true);
    } catch (const std::exception & error) {
      try {
        new_joint->Detach();
      } catch (...) {
        // Rollback below must run even if the physics backend rejects Detach.
      }
      rollback();
      PublishRefusedStatus(
        object_name, std::string("fixed_joint_init_failed=") + error.what());
      return;
    } catch (...) {
      new_joint->Detach();
      rollback();
      PublishRefusedStatus(object_name, "fixed_joint_init_failed=unknown");
      return;
    }
    fixed_joint_ = new_joint;
    object_link->SetLinearVel(ignition::math::Vector3d::Zero);
    object_link->SetAngularVel(ignition::math::Vector3d::Zero);
    attached_object_name_ = object_name;
    pending_object_name_.clear();
    PublishStatus("attached " + object_name);
    gzmsg << "lab_cobot_grasp_fix attached " << object_name
          << " canonical_correction=" << correction
          << " to " << attach_link->GetName() << std::endl;
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
    RestoreObjectCollisions();
    if (object_link) {
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
    pending_object_name_.clear();
  }

  struct CollisionMaskSnapshot
  {
    physics::CollisionPtr collision;
    unsigned int surface_collide_bitmask;
    bool has_surface;
  };

  void SuppressObjectCollisions(const physics::LinkPtr & object_link)
  {
    RestoreObjectCollisions();
    if (!object_link) {
      return;
    }
    for (const auto & collision : object_link->GetCollisions()) {
      if (!collision) {
        continue;
      }
      auto surface = collision->GetSurface();
      object_collision_masks_.push_back({
        collision,
        surface ? surface->collideBitmask : 0xffffu,
        static_cast<bool>(surface)});
      collision->SetCollideBits(0u);
      if (surface) {
        surface->collideBitmask = 0u;
      }
    }
    gzmsg << "lab_cobot_grasp_fix suppressed "
          << object_collision_masks_.size()
          << " attached-object collision(s)" << std::endl;
  }

  void RestoreObjectCollisions()
  {
    for (const auto & snapshot : object_collision_masks_) {
      if (!snapshot.collision) {
        continue;
      }
      snapshot.collision->SetCollideBits(snapshot.surface_collide_bitmask);
      auto surface = snapshot.collision->GetSurface();
      if (snapshot.has_surface && surface) {
        surface->collideBitmask = snapshot.surface_collide_bitmask;
      }
    }
    object_collision_masks_.clear();
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
        const auto config = lab_cobot_gazebo::TactileProbeSurfaceConfig();
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
            << configured << " tactile probe collision(s) as contact-only"
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
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr contact_status_pub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr contact_release_sub_;
  std::vector<std::string> object_model_names_;
  std::string attached_object_name_;
  std::string pending_object_name_;
  std::string object_link_name_;
  std::string tcp_link_name_;
  std::string tcp_parent_link_name_;
  double tcp_offset_local_z_{0.105};
  std::string left_joint_name_;
  std::string right_joint_name_;
  std::string stable_attach_link_name_;
  std::string contact_status_topic_;
  std::string contact_release_topic_;
  double object_offset_local_z_{0.045};
  double attach_clearance_{0.003};
  double max_attach_correction_{0.020};
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
  int contact_count_threshold_{3};
  int finger_contact_count_{0};
  double breakaway_force_{0.0};
  int breakaway_count_threshold_{3};
  int breakaway_count_{0};
  ignition::math::Vector3d grasp_center_offset_{0.0, 0.0, 0.0};
  double refused_status_period_{0.2};
  double last_refused_status_time_{-1.0};
  std::mutex state_mutex_;
  bool release_requested_{false};
  bool suppress_attach_until_open_{false};
  std::vector<CollisionMaskSnapshot> object_collision_masks_;
};

GZ_REGISTER_MODEL_PLUGIN(LabCobotGraspFix)
}  // namespace gazebo
