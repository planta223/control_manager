#include "control_manager/control_manager_node.hxx"
namespace control_manager {
ControlManagerNode::ControlManagerNode() : NodeBase("control_manager") {
  const auto mode = param<std::string>("mode", "IDLE");
  if (mode == "MANUAL") mode_ = core::ControlMode::MANUAL;
  else if (mode == "AUTO") mode_ = core::ControlMode::AUTO;
  else if (mode != "IDLE") throw std::invalid_argument("mode must be IDLE, MANUAL or AUTO");
  frame_ = param<std::string>("base_frame", "base_link");
  timeout_s_ = positive("command_timeout_s", 0.5);
  manual_ = input<msg::VehicleCommand>("manual_cmd");
  auto_ = input<msg::VehicleCommand>("auto_cmd");
  final_ = input<msg::VehicleCommand>("final_cmd");
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
        manual_->value.reset(); auto_->value.reset(); final_->value.reset();
        // Require new source callbacks after switching; never fall back to other mode.
        selected_publisher_->publish(toRos(core::stop(now().nanoseconds(), "MODE_CHANGED"), frame_));
      }
    }));
  tick([this] { update(); });
}
void ControlManagerNode::update() {
  auto read = [this](const std::shared_ptr<Input<msg::VehicleCommand>> &in) {
    if (!current(in, timeout_s_) || in->value->header.frame_id != frame_)
      return core::stop(now().nanoseconds(), "SOURCE_UNAVAILABLE");
    return toCore(*in->value);
  };
  const auto selected = core::selectCommand(mode_, read(manual_), read(auto_), now().nanoseconds());
  selected_publisher_->publish(toRos(selected, frame_));
  msg::ControlStatus status;
  status.header = header(frame_);
  status.mode = static_cast<std::uint8_t>(mode_);
  const auto final = final_->value ? toCore(*final_->value) : core::Command{};
  status.fsm_state = static_cast<std::uint8_t>(core::fsmState(
    static_cast<bool>(final_->value), current(final_, timeout_s_), final, mode_));
  status.reason = !final_->value ? "WAITING_FOR_SAFETY" :
    (!current(final_, timeout_s_) ? "FINAL_COMMAND_STALE" : final.reason);
  status_publisher_->publish(status);
}
}  // namespace control_manager
