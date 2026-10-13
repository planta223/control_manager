#include "control_manager/nodes.hxx"
#include <algorithm>
namespace control_manager {
MotionPlanningNode::MotionPlanningNode() : NodeBase("motion_planning") {
  frame_ = param<std::string>("map_frame", "map");
  timeout_s_ = positive("input_timeout_s", 0.5);
  cruise_speed_mps_ = nonnegative("cruise_speed_mps", 0.0);
  state_ = input<msg::VehicleState>("vehicle_state");
  path_ = input<nav_msgs::msg::Path>("reference_path");
  objects_ = input<msg::ObjectState>("object_state");
  limit_ = input<msg::MissionLimit>("mission_limit");
  publisher_ = output<msg::MotionPlan>("motion_plan");
  tick([this] { update(); });
}
void MotionPlanningNode::update() {
  msg::MotionPlan plan;
  plan.header = header(frame_);
  plan.stop_request = true;
  plan.reason = "MISSING_INVALID_OR_STALE_INPUT";
  if (current(state_, timeout_s_) && state_->value->localization_valid && state_->value->speed_valid &&
      state_->value->header.frame_id == frame_ && current(path_, timeout_s_) &&
      validPath(*path_->value, frame_) && current(limit_, timeout_s_) && limit_->value->valid &&
      limit_->value->header.frame_id == frame_ && std::isfinite(limit_->value->max_speed_mps) &&
      limit_->value->max_speed_mps >= 0.0 && current(objects_, timeout_s_) &&
      objects_->value->valid && objects_->value->obstacle_valid) {
    plan.reference_path = *path_->value;
    plan.valid = true;
    // Baseline stops for every detected obstacle; avoidance is deliberately unimplemented.
    plan.stop_request = limit_->value->stop_required || objects_->value->obstacle_detected;
    plan.target_speed_mps = plan.stop_request ? 0.0 : std::min(cruise_speed_mps_, limit_->value->max_speed_mps);
    plan.reason = plan.stop_request ? "MISSION_OR_OBSTACLE_STOP" : "BASELINE_PATH_AND_SPEED";
  }
  publisher_->publish(plan);
}
}  // namespace control_manager
