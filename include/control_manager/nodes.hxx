#pragma once
#include <optional>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "control_manager/ros_support.hxx"
#include "control_manager/core/lateral_controller.hxx"
#include "control_manager/core/tracking.hxx"
#include "control_manager/core/route_progress.hxx"
#include "control_manager/core/guidance.hxx"
#include "control_manager/core/mission.hxx"

namespace control_manager {
class SensorInterfaceNode : public NodeBase {
 public: SensorInterfaceNode();
 private:
  void laneCallback(msg::LanePose::ConstSharedPtr sample);
  void publishFeedback(const msg::VehicleFeedback &sample);
  rclcpp::Publisher<msg::LanePose>::SharedPtr lane_publisher_;
  rclcpp::Publisher<msg::VehicleFeedback>::SharedPtr feedback_publisher_;
  std::string base_frame_;
  double input_timeout_s_;
  // Encoder feedback, like GNSS, is relayed without normalization. The legacy
  // normalized VehicleFeedback publisher is independent of these two inputs.
  template<class T> void relay(const std::string &source, const std::string &destination) {
    if (topic(source).empty() || topic(destination).empty())
      throw std::invalid_argument(source + " and " + destination + " require nonempty topics");
    auto publisher = output<T>(destination, rclcpp::SensorDataQoS());
    auto subscription = create_subscription<T>(topic(source), rclcpp::SensorDataQoS(),
      [publisher](typename T::ConstSharedPtr value) { publisher->publish(*value); });
    if (std::string(subscription->get_topic_name()) == publisher->get_topic_name())
      throw std::invalid_argument("ingress source and destination resolve to the same topic: " + source);
    subscriptions_.push_back(subscription);
  }
};
class GlobalStateEstimatorNode : public NodeBase {
 public: GlobalStateEstimatorNode();
 private:
  void update();
  msg::GlobalPose estimatePose();
  msg::MotionState estimateMotion();
  std::shared_ptr<Input<nav_msgs::msg::Odometry>> gnss_;
  std::shared_ptr<Input<sensor_msgs::msg::Imu>> imu_;
  std::shared_ptr<Input<msg::VehicleFeedback>> feedback_;
  rclcpp::Publisher<msg::GlobalPose>::SharedPtr pose_publisher_;
  rclcpp::Publisher<msg::MotionState>::SharedPtr motion_publisher_;
  core::PoseMode pose_mode_;
  core::MotionMode motion_mode_;
  std::string map_frame_, base_frame_;
  double timeout_s_, max_position_std_m_, max_yaw_variance_, max_speed_variance_, wheelbase_m_;
  bool have_valid_heading_{false};
};
class LanePoseEstimatorNode : public NodeBase {
 public: LanePoseEstimatorNode();
 private:
  void observationCallback(msg::LanePose::ConstSharedPtr observation);
  rclcpp::Publisher<msg::LanePose>::SharedPtr publisher_;
  double timeout_s_;
};
class ObjectManagerNode : public NodeBase {
 public: ObjectManagerNode();
 private:
  rclcpp::Publisher<msg::ObjectState>::SharedPtr publisher_;
};
class PathPlanningNode : public NodeBase {
 public: PathPlanningNode();
 private:
  void update();
  std::shared_ptr<Input<nav_msgs::msg::Path>> global_;
  std::shared_ptr<Input<msg::GlobalPose>> pose_;
  std::shared_ptr<Input<msg::LanePose>> lane_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  core::GuidanceMode guidance_mode_{core::GuidanceMode::GPS};
  std::string frame_;
  double timeout_s_, lane_min_confidence_;
  bool datum_verified_;
};
class MissionBasedLimitNode : public NodeBase {
 public: MissionBasedLimitNode();
 private:
  void update();
  core::MissionDecision sectionLimit();
  struct SectionPolicy {
    std::string map_section;
    double max_speed_mps, margin_before_m, margin_after_m;
    bool stop_required;
  };
  std::shared_ptr<Input<msg::GlobalPose>> pose_;
  std::shared_ptr<Input<msg::MissionMap>> map_;
  std::vector<SectionPolicy> section_policies_;
  core::MissionConfig mission_config_{};
  std::shared_ptr<Input<msg::ObjectState>> objects_;
  rclcpp::Publisher<msg::MissionLimit>::SharedPtr publisher_;
  double max_speed_mps_, timeout_s_, pose_timeout_s_;
  bool baseline_enabled_, traffic_light_applies_, sections_enabled_;
  std::string frame_;
};
class MotionPlanningNode : public NodeBase {
 public: MotionPlanningNode();
 private:
  void update();
  std::shared_ptr<Input<msg::MotionState>> motion_;
  std::shared_ptr<Input<nav_msgs::msg::Path>> path_;
  std::shared_ptr<Input<msg::ObjectState>> objects_;
  std::shared_ptr<Input<msg::MissionLimit>> limit_;
  rclcpp::Publisher<msg::MotionPlan>::SharedPtr publisher_;
  double timeout_s_, cruise_speed_mps_;
  bool objects_enabled_, mission_enabled_;
  std::string frame_, base_frame_;
};
class MotionControlNode : public NodeBase {
 public: MotionControlNode();
 private:
  void update();
  std::unique_ptr<core::TrackingController> tracking_;
  std::unique_ptr<core::RouteProgressTracker> route_progress_;
  std::size_t route_phase_{0};
  core::VehicleLimits limits_;
  std::vector<core::Point> previous_path_;
  std::chrono::steady_clock::time_point previous_tick_;
  std::optional<std::chrono::steady_clock::time_point> heading_bootstrap_start_;
  bool heading_bootstrap_enabled_, have_valid_heading_{false};
  bool paused_{false}, resume_bootstrap_allowed_{false};
  std::uint64_t resume_sequence_{0};
  std::int64_t resume_after_ns_{0};
  bool run_completed_{false};  // Explicit AUTO completion; cleared only by node restart.
  bool goal_approach_seen_{false};
  double heading_bootstrap_timeout_s_, max_cross_track_m_, goal_tolerance_m_;
  std::shared_ptr<Input<msg::GlobalPose>> pose_;
  std::shared_ptr<Input<msg::MotionState>> motion_;
  std::shared_ptr<Input<msg::MotionPlan>> plan_;
  std::shared_ptr<Input<msg::ControlStatus>> control_status_;
  rclcpp::Publisher<msg::VehicleCommand>::SharedPtr publisher_;
  std::string frame_, base_frame_;
  double timeout_s_;
};
class VehicleInterfaceNode : public NodeBase {
 public: VehicleInterfaceNode();
 private:
  void update();
  core::ActuatorProtectionConfig config_;
  core::ActuatorConversionConfig conversion_;
  std::shared_ptr<Input<msg::VehicleCommand>> selected_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr actuator_publisher_;
  std::string frame_;
};
}  // namespace control_manager
