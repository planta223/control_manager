#include "control_manager/nodes.hxx"
namespace control_manager {
VehicleInterfaceNode::VehicleInterfaceNode() : NodeBase("vehicle_interface") {
  const bool dry_run = param<bool>("dry_run", true);
  if (!dry_run) throw std::invalid_argument(
    "dry_run=false rejected: packet, steering calibration and motor raw/mps contract are NOT_VERIFIED; UDP backend does not exist");
  timeout_s_ = positive("command_timeout_s", 0.5);
  final_ = input<msg::VehicleCommand>("final_cmd");
  RCLCPP_INFO(get_logger(), "[DRY_RUN] actuator interface owns final command; no actuator socket or UDP sender exists");
  tick([this] { update(); });
}
void VehicleInterfaceNode::update() {
  if (!current(final_, timeout_s_)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[DRY_RUN] final command missing/stale; stop required");
    return;
  }
  const auto &c = *final_->value;
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    "[DRY_RUN] steer=%.4f rad speed=%.3f m/s brake=%d valid=%d reason=%s",
    c.target_steer_rad, c.target_speed_mps, c.target_brake, c.valid, c.reason.c_str());
  // TODO: a separately reviewed backend needs packet/version/units, stop/watchdog,
  // calibration and sole-actuator ownership agreement before any socket is added.
}
}  // namespace control_manager
