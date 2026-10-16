#include "control_manager/nodes.hxx"
#include "control_manager/core/guidance.hxx"
#include "control_manager/core/planning_interfaces.hxx"
namespace control_manager {
PathPlanningNode::PathPlanningNode() : NodeBase("path_planning") {
  frame_ = param<std::string>("map_frame", "map");
  timeout_s_ = positive("state_timeout_s", 0.5);
  lane_min_confidence_ = nonnegative("lane_min_confidence", 0.5);
  if (lane_min_confidence_ > 1.0) throw std::invalid_argument("lane_min_confidence must be <= 1");
  datum_verified_ = param<bool>("map_datum_verified", false);
  const auto mode = param<std::string>("guidance_mode", "GPS");
  if (mode == "LANE") guidance_mode_ = msg::GuidanceStatus::LANE;
  else if (mode == "HYBRID") guidance_mode_ = msg::GuidanceStatus::HYBRID;
  else if (mode != "GPS")
    throw std::invalid_argument("guidance_mode must be GPS, LANE or HYBRID");
  if (param<bool>("lane_correction_enabled", false))
    throw std::invalid_argument("lane correction not implemented; keep lane_correction_enabled=false");
  if (!param<std::string>("map_file", "").empty())
    throw std::invalid_argument("map loader not implemented; publish a validated global_path instead");
  // Subscribe only to the inputs required by the read-only startup mode.
  if (guidance_mode_ != msg::GuidanceStatus::LANE) {
    // Static map snapshot is latched, no receipt-age expiry. Provider validates datum.
    global_ = input<nav_msgs::msg::Path>("global_path", rclcpp::QoS(1).transient_local());
    pose_ = input<msg::GlobalPose>("global_pose");
  }
  if (guidance_mode_ != msg::GuidanceStatus::GPS) {
    if (!enabled("lane_enabled", false))
      throw std::invalid_argument("LANE/HYBRID guidance requires lane_enabled=true");
    lane_ = input<msg::LanePose>("lane_pose");
  }
  publisher_ = output<nav_msgs::msg::Path>("reference_path");
  guidance_publisher_ = output<msg::GuidanceStatus>("guidance_status");
  tick([this] { update(); });
}
void PathPlanningNode::update() {
  const bool localization_available = current(pose_, timeout_s_) &&
    pose_->value->valid && pose_->value->header.frame_id == frame_ &&
    std::isfinite(pose_->value->x_m) && std::isfinite(pose_->value->y_m) &&
    std::isfinite(pose_->value->yaw_rad);
  const bool global_path_available = datum_verified_ && global_ && global_->value &&
    validPath(*global_->value, frame_);
  std::string lane_error;
  if (!lane_) lane_error = "LANE_INPUT_DISABLED";
  else if (!current(lane_, timeout_s_)) lane_error = "LANE_POSE_UNAVAILABLE";
  else if (!std::isfinite(lane_->value->lateral_error_m) ||
           !std::isfinite(lane_->value->heading_error_rad) ||
           !std::isfinite(lane_->value->confidence) ||
           lane_->value->confidence < 0.0 || lane_->value->confidence > 1.0)
    lane_error = "LANE_VALUES_INVALID";
  else if (lane_->value->confidence < lane_min_confidence_) lane_error = "LANE_CONFIDENCE_LOW";
  // Header frame/error signs are not enough to infer lane geometry; no transform is assumed.
  const auto decision = core::evaluateGuidance(guidance_mode_, localization_available,
                                               global_path_available, lane_error);
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
