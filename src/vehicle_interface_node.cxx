#include "control_manager/nodes.hxx"
namespace control_manager {
VehicleInterfaceNode::VehicleInterfaceNode() : NodeBase("vehicle_interface") {
  const bool dry_run = param<bool>("dry_run", true);
  conversion_.wheel_diameter_m = positive("vehicle.wheel_diameter_m", 0.0);
  conversion_.motor_to_wheel_ratio = positive("vehicle.motor_to_wheel_ratio", 1.0);
  conversion_.steering_full_scale_deg = positive("trans.steering_full_scale_deg", 30.0);
  conversion_.motor_full_scale_rpm = positive("trans.motor_full_scale_rpm", 1500.0);
  frame_ = param<std::string>("base_frame", "base_link");
  config_.command_timeout_s = positive("command_timeout_s", 0.5);
  config_.operating_limits_verified = param<bool>("operating_limits_verified", false);
  config_.limits = vehicleLimits();
  selected_ = input<msg::VehicleCommand>("selected_cmd");
  if (!dry_run) actuator_publisher_ = output<std_msgs::msg::Float32MultiArray>("actuator_raw", rclcpp::QoS(1));
  RCLCPP_INFO(get_logger(), "%s", dry_run ?
    "[DRY_RUN] actuator protection active; actuator_raw disabled" :
    "[ACTUATOR_RAW] right-positive steering; UDP transmission belongs to external trans");
  tick([this] { update(); });
}
void VehicleInterfaceNode::update() {
  auto command = core::stop(now().nanoseconds(), "SELECTED_COMMAND_UNAVAILABLE");
  if (current(selected_, config_.command_timeout_s) && selected_->value->header.frame_id == frame_)
    command = toCore(*selected_->value);
  const auto protected_command = core::protectActuatorCommand(
    command, now().nanoseconds(), config_);
  if (actuator_publisher_) {
    const auto values = core::actuatorRaw(protected_command, conversion_);
    std_msgs::msg::Float32MultiArray raw;
    raw.data.assign(values.begin(), values.end());
    actuator_publisher_->publish(raw);
  }
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    "[%s] steer=%.4f rad speed=%.3f m/s brake=%d valid=%d reason=%s",
    actuator_publisher_ ? "ACTUATOR_RAW" : "DRY_RUN",
    protected_command.target_steer_rad, protected_command.target_speed_mps,
    protected_command.target_brake, protected_command.valid, protected_command.reason.c_str());
  // Existing trans consumes actuator_raw; this node owns no actuator socket or packet encoder.
}
}  // namespace control_manager
