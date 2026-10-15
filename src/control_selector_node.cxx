#include "control_manager/control_selector_node.hxx"
namespace control_manager {
ControlSelectorNode::ControlSelectorNode() : NodeBase("control_selector") {
  const auto mode = param<std::string>("mode", "IDLE");
  if (mode == "MANUAL") mode_ = core::ControlMode::MANUAL;
  else if (mode == "AUTO") mode_ = core::ControlMode::AUTO;
  else if (mode != "IDLE") throw std::invalid_argument("mode must be IDLE, MANUAL or AUTO");
  frame_ = param<std::string>("base_frame", "base_link");
  timeout_s_ = positive("command_timeout_s", 0.5);
  manual_ = input<msg::VehicleCommand>("manual_cmd");
  auto_ = input<msg::VehicleCommand>("auto_cmd");
  interface_status_ = input<msg::VehicleInterfaceStatus>("vehicle_interface_status");
  selected_publisher_ = output<msg::VehicleCommand>("selected_cmd");
  status_publisher_ = output<msg::ControlStatus>("status");
  subscriptions_.push_back(create_subscription<std_msgs::msg::UInt8>(topic("mode_request"), 10,
    [this](std_msgs::msg::UInt8::ConstSharedPtr request) {
      const auto next = request->data <= static_cast<std::uint8_t>(core::ControlMode::AUTO)
          ? static_cast<core::ControlMode>(request->data) : core::ControlMode::IDLE;
      if (request->data > static_cast<std::uint8_t>(core::ControlMode::AUTO))
        RCLCPP_ERROR(get_logger(), "Invalid mode request: switching to IDLE");
      if (next != mode_) {
        mode_ = next;
        manual_->value.reset(); auto_->value.reset(); interface_status_->value.reset();
        // Require new source callbacks after switching; never fall back to other mode.
        selected_publisher_->publish(toRos(core::stop(now().nanoseconds(), "MODE_CHANGED"), frame_));
      }
    }));
  tick([this] { update(); });
}
void ControlSelectorNode::update() {
  auto read = [this](const std::shared_ptr<Input<msg::VehicleCommand>> &in) {
    if (!current(in, timeout_s_) || in->value->header.frame_id != frame_)
      return core::stop(now().nanoseconds(), "SOURCE_UNAVAILABLE");
    if (!in->value->valid)
      return core::stop(now().nanoseconds(), "INVALID_SOURCE_COMMAND");
    return toCore(*in->value);
  };
  const auto selected = core::selectCommand(mode_, read(manual_), read(auto_), now().nanoseconds());
  selected_publisher_->publish(toRos(selected, frame_));
  msg::ControlStatus status;
  status.header = header(frame_);
  status.mode = static_cast<std::uint8_t>(mode_);
  const auto final = interface_status_->value
    ? toCore(interface_status_->value->protected_command) : core::Command{};
  const bool protection_fresh = current(interface_status_, timeout_s_) &&
    interface_status_->value->header.frame_id == frame_ &&
    interface_status_->value->protected_command.header.frame_id == frame_ &&
    core::fresh(final.timestamp_ns, now().nanoseconds(), timeout_s_);
  status.fsm_state = static_cast<std::uint8_t>(core::fsmState(
    static_cast<bool>(interface_status_->value), protection_fresh, final, mode_));
  status.reason = !interface_status_->value ? "WAITING_FOR_VEHICLE_INTERFACE" :
    (!protection_fresh ? "VEHICLE_INTERFACE_STATUS_STALE" : final.reason);
  status_publisher_->publish(status);
}
}  // namespace control_manager
