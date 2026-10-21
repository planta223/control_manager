#pragma once
#include "control_manager/ros_support.hxx"
namespace control_manager {
class ControlSelectorNode : public NodeBase {
 public: ControlSelectorNode();
 private:
  void update();
  core::StopSteeringHold stop_steering_;  // selected output hold.
  core::ControlMode mode_{core::ControlMode::IDLE};
  bool paused_;
  std::int64_t last_pause_request_ns_{0};
  std::int64_t resume_after_ns_{0};
  std::uint64_t resume_sequence_{0};
  std::shared_ptr<Input<msg::VehicleCommand>> manual_, auto_;
  rclcpp::Publisher<msg::VehicleCommand>::SharedPtr selected_publisher_;
  rclcpp::Publisher<msg::ControlStatus>::SharedPtr status_publisher_;
  std::string frame_;
  double timeout_s_;
};
}  // namespace control_manager
