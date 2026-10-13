#include "control_manager/nodes.hxx"
#include "control_manager/core/guidance.hxx"
#include "control_manager/core/planning_interfaces.hxx"
namespace control_manager {
PathPlanningNode::PathPlanningNode() : NodeBase("path_planning") {
  frame_ = param<std::string>("map_frame", "map");
  timeout_s_ = positive("state_timeout_s", 0.5);
  datum_verified_ = param<bool>("map_datum_verified", false);
  const auto mode = param<std::string>("guidance_mode", "GPS_ONLY");
  if (mode == "LANE_ONLY") requested_guidance_ = msg::GuidanceStatus::LANE_ONLY;
  else if (mode == "HYBRID") requested_guidance_ = msg::GuidanceStatus::HYBRID;
  else if (mode != "GPS_ONLY")
    throw std::invalid_argument("guidance_mode must be GPS_ONLY, LANE_ONLY or HYBRID");
  if (param<bool>("lane_correction_enabled", false))
    throw std::invalid_argument("lane correction not implemented; keep lane_correction_enabled=false");
  if (!param<std::string>("map_file", "").empty())
    throw std::invalid_argument("map loader not implemented; publish a validated global_path instead");
  // Static map snapshot is latched, no receipt-age expiry. Provider must validate datum.
  global_ = input<nav_msgs::msg::Path>("global_path", rclcpp::QoS(1).transient_local());
  state_ = input<msg::VehicleState>("vehicle_state");
  lane_ = input<msg::LaneState>("lane_state");  // Future algorithms only, never a validity shortcut.
  publisher_ = output<nav_msgs::msg::Path>("reference_path");
  guidance_publisher_ = output<msg::GuidanceStatus>("guidance_status");
  subscriptions_.push_back(create_subscription<std_msgs::msg::UInt8>(topic("guidance_mode_request"), 10,
    [this](std_msgs::msg::UInt8::ConstSharedPtr request) {
      requested_guidance_ = request->data;
      update();  // Publish empty path immediately for unsupported/invalid requests.
    }));
  tick([this] { update(); });
}
void PathPlanningNode::update() {
  const bool localization_available = current(state_, timeout_s_) &&
    state_->value->localization_valid && state_->value->header.frame_id == frame_ &&
    std::isfinite(state_->value->x_m) && std::isfinite(state_->value->y_m) &&
    std::isfinite(state_->value->yaw_rad);
  const bool global_path_available = datum_verified_ && global_->value &&
    validPath(*global_->value, frame_);
  const auto decision = core::evaluateGuidance(requested_guidance_, localization_available,
                                               global_path_available);
  nav_msgs::msg::Path path;
  if (decision.valid) path = *global_->value;
  path.header = header(frame_);  // Planning generation time, not sensor measurement time.
  msg::GuidanceStatus status;
  status.header = path.header;
  status.requested_mode = decision.requested_mode;
  status.actual_mode = decision.actual_mode ? static_cast<std::uint8_t>(*decision.actual_mode)
                                           : msg::GuidanceStatus::NONE;
  status.active_source = decision.active_source;
  status.valid = decision.valid;
  status.degraded = decision.degraded;
  status.reason = decision.reason;
  publisher_->publish(path);  // Empty path -> invalid MotionPlan -> AUTO stop.
  guidance_publisher_->publish(status);
}
}  // namespace control_manager
