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
  selected_publisher_ = output<msg::VehicleCommand>("selected_cmd");
  status_publisher_ = output<msg::ControlStatus>("status");
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
  // This FSM describes selection only; final actuator protection belongs to Vehicle Interface.
  status.fsm_state = static_cast<std::uint8_t>(core::fsmState(
    true, core::fresh(selected.timestamp_ns, now().nanoseconds(), timeout_s_), selected, mode_));
  status.reason = selected.reason;
  status_publisher_->publish(status);
}
}  // namespace control_manager
