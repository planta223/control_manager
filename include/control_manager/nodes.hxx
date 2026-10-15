#pragma once
#include "control_manager/ros_support.hxx"
#include "control_manager/core/lateral_controller.hxx"
#include "control_manager/core/tracking.hxx"

namespace control_manager {
class SensorInterfaceNode : public NodeBase {
 public: SensorInterfaceNode();
 private:
  template<class T> void relay(const std::string &source, const std::string &destination) {
    auto publisher = output<T>(destination, rclcpp::SensorDataQoS());
    subscriptions_.push_back(create_subscription<T>(topic(source), rclcpp::SensorDataQoS(),
      [publisher](typename T::ConstSharedPtr value) { publisher->publish(*value); }));
  }
};
class GnssStateEstimatorNode : public NodeBase {
 public: GnssStateEstimatorNode();
 private:
  void update();
  std::shared_ptr<Input<nav_msgs::msg::Odometry>> gnss_;
  rclcpp::Publisher<msg::GlobalPose>::SharedPtr publisher_;
  core::GlobalEstimationMode mode_;
  std::shared_ptr<Input<sensor_msgs::msg::Imu>> imu_;
  std::string map_frame_, base_frame_;
  double timeout_s_, max_position_std_m_, max_yaw_variance_;
  bool reference_verified_;
};
class MotionStateEstimatorNode : public NodeBase {
 public: MotionStateEstimatorNode();
 private:
  void update();
  std::shared_ptr<Input<nav_msgs::msg::Odometry>> gnss_;
  rclcpp::Publisher<msg::MotionState>::SharedPtr publisher_;
  core::MotionEstimationMode mode_;
  std::string map_frame_, base_frame_;
  double timeout_s_, max_speed_variance_, wheelbase_m_;
  // TODO: attach measured wheel speed/steering feedback inputs here ONLY after
  // topic/type, physical units, calibration, frame and source-time contract agreement.
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
  std::shared_ptr<Input<msg::LaneState>> lane_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  rclcpp::Publisher<msg::GuidanceStatus>::SharedPtr guidance_publisher_;
  std::uint8_t requested_guidance_{msg::GuidanceStatus::GPS};
  std::string frame_;
  double timeout_s_, lane_min_confidence_;
  bool datum_verified_;
};
class MissionBasedLimitNode : public NodeBase {
 public: MissionBasedLimitNode();
 private:
  void update();
  std::shared_ptr<Input<msg::ObjectState>> objects_;
  rclcpp::Publisher<msg::MissionLimit>::SharedPtr publisher_;
  double max_speed_mps_, timeout_s_;
  bool baseline_enabled_, traffic_light_applies_;
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
  std::string frame_, base_frame_;
};
class MotionControlNode : public NodeBase {
 public: MotionControlNode();
 private:
  void update();
  void publish(const core::Command &command, const core::TrackingResult &result);
  std::unique_ptr<core::TrackingController> tracking_;
  std::vector<core::Point> previous_path_;
  std::chrono::steady_clock::time_point previous_tick_;
  std::shared_ptr<Input<msg::GlobalPose>> pose_;
  std::shared_ptr<Input<msg::MotionState>> motion_;
  std::shared_ptr<Input<msg::MotionPlan>> plan_;
  std::shared_ptr<Input<msg::ControlStatus>> control_status_;
  rclcpp::Publisher<msg::VehicleCommand>::SharedPtr publisher_;
  rclcpp::Publisher<msg::TrackingStatus>::SharedPtr tracking_publisher_;
  std::string frame_, base_frame_;
  double timeout_s_;
};
class SafetyLimitNode : public NodeBase {
 public: SafetyLimitNode();
 private:
  void update();
  core::SafetyConfig config_;
  std::shared_ptr<Input<msg::GuidanceStatus>> guidance_;
  std::shared_ptr<Input<msg::GlobalPose>> pose_;
  std::shared_ptr<Input<msg::MotionState>> motion_;
  std::shared_ptr<Input<msg::VehicleCommand>> selected_;
  std::shared_ptr<Input<msg::ObjectState>> objects_;
  rclcpp::Publisher<msg::VehicleCommand>::SharedPtr publisher_;
  bool estop_{false}, require_obstacle_state_;
  double object_timeout_s_;
  std::string frame_, map_frame_;
};
class VehicleInterfaceNode : public NodeBase {
 public: VehicleInterfaceNode();
 private:
  void update();
  std::shared_ptr<Input<msg::VehicleCommand>> final_;
  double timeout_s_;
};
}  // namespace control_manager
