#include "control_manager/nodes.hxx"
#include "control_manager/core/planning_interfaces.hxx"
namespace control_manager {
PathPlanningNode::PathPlanningNode() : NodeBase("path_planning") {
  frame_ = param<std::string>("map_frame", "map");
  timeout_s_ = positive("state_timeout_s", 0.5);
  datum_verified_ = param<bool>("map_datum_verified", false);
  if (param<bool>("lane_correction_enabled", false))
    throw std::invalid_argument("lane correction not implemented; keep lane_correction_enabled=false");
  if (!param<std::string>("map_file", "").empty())
    throw std::invalid_argument("map loader not implemented; publish a validated global_path instead");
  // Static map snapshot is latched, no receipt-age expiry. Provider must validate datum.
  global_ = input<nav_msgs::msg::Path>("global_path", rclcpp::QoS(1).transient_local());
  state_ = input<msg::VehicleState>("vehicle_state");
  lane_ = input<msg::LaneState>("lane_state");
  publisher_ = output<nav_msgs::msg::Path>("reference_path");
  tick([this] { update(); });
}
void PathPlanningNode::update() {
  nav_msgs::msg::Path path;
  if (datum_verified_ && current(state_, timeout_s_) && state_->value->localization_valid &&
      state_->value->header.frame_id == frame_ && global_->value && validPath(*global_->value, frame_))
    path = *global_->value;
  // Invalid/missing LaneState -> unmodified global path. Valid lane also falls back
  // explicitly until LaneCorrection is implemented; never invent lateral geometry.
  path.header = header(frame_);  // Planning generation time, not sensor measurement time.
  publisher_->publish(path);
}
}  // namespace control_manager
