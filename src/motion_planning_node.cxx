#include "control_manager/nodes.hxx"
#include "control_manager/ingress.hxx"
#include <algorithm>
namespace control_manager {
MotionPlanningNode::MotionPlanningNode() : NodeBase("motion_planning") {
  frame_ = param<std::string>("map_frame", "map");
  base_frame_ = param<std::string>("base_frame", "base_link");
  timeout_s_ = positive("input_timeout_s", 0.5);
  cruise_speed_mps_ = nonnegative("cruise_speed_mps", nonnegative("cruise_speed_kph", 0.0) / 3.6);
  motion_ = input<msg::MotionState>("motion_state");
  path_ = input<nav_msgs::msg::Path>("reference_path");
  objects_enabled_ = enabled("object_state_enabled", true);
  mission_enabled_ = enabled("mission_limit_enabled", true);
  if (objects_enabled_) objects_ = input<msg::ObjectState>("object_state");
  if (mission_enabled_) limit_ = input<msg::MissionLimit>("mission_limit");
  publisher_ = output<msg::MotionPlan>("motion_plan");
  tick([this] { update(); });
}
void MotionPlanningNode::update() {
  msg::MotionPlan plan;
  plan.header = header(frame_);
  plan.stop_request = true;
  plan.reason = "MISSING_INVALID_OR_STALE_INPUT";
  const auto current_input = [this](const auto &in) {
    return in && in->value && ingress::validStampFormat(in->value->header.stamp) &&
      current(in, timeout_s_);
  };
  if (current_input(motion_) && motion_->value->speed_valid &&
      std::isfinite(motion_->value->speed_mps) && motion_->value->speed_mps >= 0.0 &&
      motion_->value->header.frame_id == base_frame_ && current_input(path_) &&
      (validPath(*path_->value, frame_) || validPath(*path_->value, base_frame_)) &&
      (!mission_enabled_ || (current_input(limit_) && limit_->value->valid &&
      limit_->value->header.frame_id == frame_ && std::isfinite(limit_->value->max_speed_mps) &&
      limit_->value->max_speed_mps >= 0.0)) &&
      (!objects_enabled_ || (current_input(objects_) &&
      objects_->value->valid && objects_->value->obstacle_valid))) {
    plan.header.frame_id = path_->value->header.frame_id;
    plan.reference_path = *path_->value;
    plan.valid = true;
    // Baseline stops for every detected obstacle; avoidance is deliberately unimplemented.
    plan.stop_request = (mission_enabled_ && limit_->value->stop_required) ||
                        (objects_enabled_ && objects_->value->obstacle_detected);
    const double speed = mission_enabled_ ? std::min(cruise_speed_mps_, limit_->value->max_speed_mps)
                                          : cruise_speed_mps_;
    plan.target_speed_mps = plan.stop_request ? 0.0 : speed;
    plan.reason = plan.stop_request ? "MISSION_OR_OBSTACLE_STOP" : "BASELINE_PATH_AND_SPEED";
    if (!objects_enabled_) plan.reason += ":OBJECT_DISABLED";
    if (!mission_enabled_) plan.reason += ":MISSION_DISABLED";
  }
  publisher_->publish(plan);
}
}  // namespace control_manager
