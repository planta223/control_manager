#include "control_manager/nodes.hxx"
namespace control_manager {
MotionControlNode::MotionControlNode() : NodeBase("motion_control") {
  frame_ = param<std::string>("map_frame", "map");
  base_frame_ = param<std::string>("base_frame", "base_link");
  timeout_s_ = positive("input_timeout_s", 0.5);
  core::PurePursuitConfig config;
  config.wheelbase_m = positive("vehicle.wheelbase_m", 1.328);
  config.lookahead_min_m = positive("lookahead_min_m", 2.0);
  config.lookahead_max_m = positive("lookahead_max_m", 6.0);
  config.lookahead_gain_s = nonnegative("lookahead_gain_s", 1.0);
  config.goal_tolerance_m = nonnegative("goal_tolerance_m", 0.3);
  lateral_ = std::make_unique<core::AdaptivePurePursuit>(config);
  state_ = input<msg::VehicleState>("vehicle_state");
  plan_ = input<msg::MotionPlan>("motion_plan");
  publisher_ = output<msg::VehicleCommand>("auto_cmd");
  tick([this] { update(); });
}
void MotionControlNode::update() {
  auto command = core::stop(now().nanoseconds(), "INVALID_OR_STALE_STATE_OR_PLAN");
  if (current(state_, timeout_s_) && current(plan_, timeout_s_) &&
      state_->value->localization_valid && state_->value->speed_valid &&
      state_->value->header.frame_id == frame_ && plan_->value->valid &&
      plan_->value->header.frame_id == frame_ && std::isfinite(plan_->value->target_speed_mps) &&
      plan_->value->target_speed_mps >= 0.0) {
    const auto &plan = *plan_->value;
    if (plan.stop_request || plan.target_speed_mps == 0.0) {
      command = core::stop(now().nanoseconds(), "PLAN_STOP", true);
    } else if (validPath(plan.reference_path, frame_)) {
      std::vector<core::Point> path;
      path.reserve(plan.reference_path.poses.size());
      for (const auto &p : plan.reference_path.poses) path.push_back({p.pose.position.x, p.pose.position.y});
      const auto result = lateral_->compute(toCore(*state_->value), path);
      if (result.valid && result.goal_reached) command = core::stop(now().nanoseconds(), "GOAL_REACHED", true);
      else if (result.valid) {
        command = {result.steering_rad, plan.target_speed_mps, false, true,
                   now().nanoseconds(), "PURE_PURSUIT_AND_SPEED_PASSTHROUGH"};
      }
    }
  }
  // Longitudinal control = physical speed pass-through only. No PID/raw conversion.
  publisher_->publish(toRos(command, base_frame_));
}
}  // namespace control_manager
