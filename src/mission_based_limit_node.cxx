#include "control_manager/nodes.hxx"
#include <algorithm>
namespace control_manager {
MissionBasedLimitNode::MissionBasedLimitNode() : NodeBase("mission_based_limit") {
  frame_ = param<std::string>("map_frame", "map");
  max_speed_mps_ = nonnegative("mission_max_speed_mps", 0.0);
  timeout_s_ = positive("object_timeout_s", 0.5);
  baseline_enabled_ = param<bool>("mission_baseline_enabled", false);
  traffic_light_applies_ = param<bool>("traffic_light_applies", false);
  objects_ = input<msg::ObjectState>("object_state");
  publisher_ = output<msg::MissionLimit>("mission_limit");
  tick([this] { update(); });
}
void MissionBasedLimitNode::update() {
  msg::MissionLimit limit;
  limit.header = header(frame_);
  limit.max_speed_mps = max_speed_mps_;
  limit.valid = baseline_enabled_;
  limit.stop_required = !baseline_enabled_;
  limit.reason = baseline_enabled_ ? "CONFIGURED_BASELINE" : "MISSION_POLICY_NOT_CONFIGURED";
  const bool available = current(objects_, timeout_s_) && objects_->value->valid;
  if (available && objects_->value->speed_limit_valid) {
    const double speed = objects_->value->speed_limit_mps;
    if (!std::isfinite(speed) || speed < 0.0) {
      limit.valid = false; limit.stop_required = true; limit.reason = "INVALID_SPEED_LIMIT";
    } else {
      limit.max_speed_mps = std::min(limit.max_speed_mps, speed);
    }
  }
  if (traffic_light_applies_) {
    if (!available || !objects_->value->traffic_light_valid ||
        objects_->value->traffic_light_state == msg::ObjectState::UNKNOWN ||
        objects_->value->traffic_light_state > msg::ObjectState::GREEN) {
      limit.valid = false; limit.stop_required = true; limit.reason = "TRAFFIC_LIGHT_UNAVAILABLE";
    } else if (objects_->value->traffic_light_state != msg::ObjectState::GREEN) {
      limit.stop_required = true; limit.reason = "TRAFFIC_LIGHT_STOP";
    }
  }
  // TODO: map mission sections / curve rules / sign lifetime. No steering computation.
  publisher_->publish(limit);
}
}  // namespace control_manager
