#pragma once
#include "control_manager/ros_support.hxx"
namespace control_manager {
class ControlManagerNode : public NodeBase {
 public: ControlManagerNode();
 private:
  void update();
  core::ControlMode mode_{core::ControlMode::IDLE};
  std::shared_ptr<Input<msg::VehicleCommand>> manual_, auto_, final_;
  rclcpp::Publisher<msg::VehicleCommand>::SharedPtr selected_publisher_;
  rclcpp::Publisher<msg::ControlStatus>::SharedPtr status_publisher_;
  std::string frame_;
  double timeout_s_;
};
}  // namespace control_manager
