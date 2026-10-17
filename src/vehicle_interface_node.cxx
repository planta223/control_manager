#include "control_manager/nodes.hxx"
namespace control_manager {
VehicleInterfaceNode::VehicleInterfaceNode() : NodeBase("vehicle_interface") {
  const bool dry_run = param<bool>("dry_run", true);
  if (!dry_run) throw std::invalid_argument(
    "dry_run=false rejected: packet, steering calibration and motor raw/mps contract are NOT_VERIFIED; UDP backend does not exist");
  frame_ = param<std::string>("base_frame", "base_link");
  config_.command_timeout_s = positive("command_timeout_s", 0.5);
  config_.operating_limits_verified = param<bool>("operating_limits_verified", false);
  config_.limits = vehicleLimits();
  selected_ = input<msg::VehicleCommand>("selected_cmd");
  subscriptions_.push_back(create_subscription<std_msgs::msg::Bool>(topic("estop"),
    rclcpp::QoS(1).reliable().transient_local(), [this](std_msgs::msg::Bool::ConstSharedPtr value) {
      estop_ = value->data;  // Latches true until explicit false, independent of mode/source.
    }));
  RCLCPP_INFO(get_logger(), "[DRY_RUN] actuator protection active; no actuator socket or UDP sender exists");
  tick([this] { update(); });
}
void VehicleInterfaceNode::update() {
  auto command = core::stop(now().nanoseconds(), "SELECTED_COMMAND_UNAVAILABLE");
  if (current(selected_, config_.command_timeout_s) && selected_->value->header.frame_id == frame_)
    command = toCore(*selected_->value);
  const auto protected_command = core::protectActuatorCommand(
    command, estop_, now().nanoseconds(), config_);
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    "[DRY_RUN] steer=%.4f rad speed=%.3f m/s brake=%d valid=%d reason=%s",
    protected_command.target_steer_rad, protected_command.target_speed_mps,
    protected_command.target_brake, protected_command.valid, protected_command.reason.c_str());
  // TODO: only this protected result may reach a reviewed hardware backend. Packet,
  // calibration, watchdog and exclusive-actuator ownership remain unimplemented.
}
}  // namespace control_manager
