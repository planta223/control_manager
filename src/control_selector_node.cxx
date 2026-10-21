#include "control_manager/control_selector_node.hxx"
#include "control_manager/ingress.hxx"
namespace control_manager {
ControlSelectorNode::ControlSelectorNode() : NodeBase("control_selector") {
  const auto mode = param<std::string>("mode", "IDLE");
  if (mode == "MANUAL") mode_ = core::ControlMode::MANUAL;
  else if (mode == "AUTO") mode_ = core::ControlMode::AUTO;
  else if (mode != "IDLE") throw std::invalid_argument("mode must be IDLE, MANUAL or AUTO");
  frame_ = param<std::string>("base_frame", "base_link");
  timeout_s_ = positive("command_timeout_s", 0.5);
  paused_ = param<bool>("start_paused", true);
  subscriptions_.push_back(create_subscription<msg::PauseRequest>(topic("pause_request"), 10,
    [this](msg::PauseRequest::ConstSharedPtr request) {
      if (mode_ != core::ControlMode::AUTO || !ingress::validStampFormat(request->header.stamp) ||
          (!request->header.frame_id.empty() && request->header.frame_id != frame_)) return;
      const auto stamp = rclcpp::Time(request->header.stamp).nanoseconds();
      if (!core::fresh(stamp, now().nanoseconds(), timeout_s_) || stamp <= last_pause_request_ns_) return;
      last_pause_request_ns_ = stamp;
      if (paused_ && !request->pause) {
        ++resume_sequence_;
        resume_after_ns_ = now().nanoseconds();
      }
      paused_ = request->pause;
    }));
  manual_ = input<msg::VehicleCommand>("manual_cmd");
  auto_ = input<msg::VehicleCommand>("auto_cmd");
  selected_publisher_ = output<msg::VehicleCommand>("selected_cmd");
  status_publisher_ = output<msg::ControlStatus>("status");
  tick([this] { update(); });
}
void ControlSelectorNode::update() {
  auto read = [this](const std::shared_ptr<Input<msg::VehicleCommand>> &in) {
    if (!in || !in->value || !ingress::validStampFormat(in->value->header.stamp) ||
        !current(in, timeout_s_) || in->value->header.frame_id != frame_)
      return core::stop(now().nanoseconds(), "SOURCE_UNAVAILABLE");
    if (!in->value->valid)
      return core::stop(now().nanoseconds(), "INVALID_SOURCE_COMMAND");
    if (in == auto_ && rclcpp::Time(in->value->header.stamp).nanoseconds() < resume_after_ns_)
      return core::stop(now().nanoseconds(), "RESUME_WAITING_FOR_NEW_AUTO_COMMAND");
    return toCore(*in->value);
  };
  const bool paused = mode_ == core::ControlMode::AUTO && paused_;
  // previous arbitration output did not preserve stop steering.
  // const auto selected = paused ? core::stop(now().nanoseconds(), "AUTO_PAUSED", true)
  //   : core::selectCommand(mode_, read(manual_), read(auto_), now().nanoseconds());
  // pause/stale/invalid paths retain the last selected steering command.
  const auto selected = stop_steering_.apply(paused ? core::stop(now().nanoseconds(), "AUTO_PAUSED", true)
    : core::selectCommand(mode_, read(manual_), read(auto_), now().nanoseconds()));
  selected_publisher_->publish(toRos(selected, frame_));
  msg::ControlStatus status;
  status.header = header(frame_);
  status.mode = static_cast<std::uint8_t>(mode_);
  status.paused = paused;
  status.resume_sequence = resume_sequence_;
  // This FSM describes selection only; final actuator protection belongs to Vehicle Interface.
  status.fsm_state = static_cast<std::uint8_t>(core::fsmState(
    true, core::fresh(selected.timestamp_ns, now().nanoseconds(), timeout_s_), selected, mode_));
  status.reason = selected.reason;
  status_publisher_->publish(status);
}
}  // namespace control_manager
