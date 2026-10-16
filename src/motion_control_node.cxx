#include "control_manager/nodes.hxx"
#include <algorithm>
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
  core::PidConfig pid;
  pid.enabled = param<bool>("pid.enabled", false);
  pid.kp_lateral = nonnegative("pid.kp_lateral", 0.0);
  pid.ki_lateral = nonnegative("pid.ki_lateral", 0.0);
  pid.kd_lateral = nonnegative("pid.kd_lateral", 0.0);
  pid.kp_heading = nonnegative("pid.kp_heading", 0.0);
  pid.integral_limit_m_s = nonnegative("pid.integral_limit_m_s", 0.0);
  pid.max_steering_rad = nonnegative("pid.max_steering_rad", 0.0);
  pid.max_dt_s = positive("pid.max_dt_s", 0.5);
  pid.goal_tolerance_m = config.goal_tolerance_m;
  tracking_ = std::make_unique<core::TrackingController>(pid, config);
  const auto mode = param<std::string>("tracking_mode", "PID");
  if (mode == "PURE_PURSUIT") tracking_->request(static_cast<std::uint8_t>(core::TrackingMode::PURE_PURSUIT));
  else if (mode == "ADAPTIVE_PURE_PURSUIT")
    tracking_->request(static_cast<std::uint8_t>(core::TrackingMode::ADAPTIVE_PURE_PURSUIT));
  else if (mode != "PID")
    throw std::invalid_argument("tracking_mode must be PID, PURE_PURSUIT or ADAPTIVE_PURE_PURSUIT");
  pose_ = input<msg::GlobalPose>("global_pose");
  motion_ = input<msg::MotionState>("motion_state");
  plan_ = input<msg::MotionPlan>("motion_plan");
  control_status_ = std::make_shared<Input<msg::ControlStatus>>();
  subscriptions_.push_back(create_subscription<msg::ControlStatus>(topic("status"), 10,
    [this](msg::ControlStatus::ConstSharedPtr status) {
      if (!control_status_->value || control_status_->value->mode != status->mode ||
          status->mode != msg::ControlStatus::AUTO || status->fsm_state == msg::ControlStatus::FAULT)
        tracking_->reset();
      control_status_->value = status;
      control_status_->received = std::chrono::steady_clock::now();
    }));
  publisher_ = output<msg::VehicleCommand>("auto_cmd");
  previous_tick_ = std::chrono::steady_clock::now();
  tick([this] { update(); });
}
void MotionControlNode::update() {
  const auto tick_time = std::chrono::steady_clock::now();
  const double dt_s = std::chrono::duration<double>(tick_time - previous_tick_).count();
  previous_tick_ = tick_time;
  auto stop = [this](const std::string &reason, bool valid = false) {
    tracking_->reset(); previous_path_.clear();
    publisher_->publish(toRos(core::stop(now().nanoseconds(), reason, valid), base_frame_));
  };
  const auto selection_error = tracking_->selectionError();
  if (!selection_error.empty()) { stop(selection_error); return; }
  // Existing PID/PP require a pose and path in the same global frame. A local
  // lane path needs an agreed tracking-frame adapter, not a fabricated zero pose.
  if (current(plan_, timeout_s_) && plan_->value->valid &&
      (plan_->value->header.frame_id != frame_ ||
       (!plan_->value->reference_path.header.frame_id.empty() &&
        plan_->value->reference_path.header.frame_id != frame_))) {
    stop("LOCAL_REFERENCE_PATH_NOT_SUPPORTED"); return;
  }
  if (!current(pose_, timeout_s_) || !current(motion_, timeout_s_) || !current(plan_, timeout_s_) ||
      !pose_->value->valid || !motion_->value->speed_valid ||
      motion_->value->header.frame_id != base_frame_ || pose_->value->header.frame_id != frame_ || !plan_->value->valid ||
      plan_->value->header.frame_id != frame_ || !std::isfinite(plan_->value->target_speed_mps) ||
      plan_->value->target_speed_mps < 0.0) {
    stop("INVALID_OR_STALE_STATE_OR_PLAN"); return;
  }
  const auto &plan = *plan_->value;
  const auto state = trackingInput(*pose_->value, *motion_->value);
  // Stop plans retain the previous speed/brake policy but do not claim valid tracking.
  if (!std::isfinite(state.x_m) || !std::isfinite(state.y_m) || !std::isfinite(state.yaw_rad) ||
      !std::isfinite(state.speed_mps) || state.speed_mps < 0.0) {
    stop("INVALID_TRACKING_STATE"); return;
  }
  if (plan.stop_request || plan.target_speed_mps == 0.0) { stop("PLAN_STOP", true); return; }
  if (!validPath(plan.reference_path, frame_) ||
      !core::fresh(rclcpp::Time(plan.reference_path.header.stamp).nanoseconds(),
                   now().nanoseconds(), timeout_s_)) {
    stop("INVALID_OR_STALE_REFERENCE_PATH"); return;
  }
  std::vector<core::Point> path;
  path.reserve(plan.reference_path.poses.size());
  for (const auto &p : plan.reference_path.poses) path.push_back({p.pose.position.x, p.pose.position.y});
  const bool same_path = path.size() == previous_path_.size() &&
    std::equal(path.begin(), path.end(), previous_path_.begin(),
      [](const core::Point &a, const core::Point &b) { return a.x_m == b.x_m && a.y_m == b.y_m; });
  if (!same_path) tracking_->reset();  // Path changes must not carry integral/derivative history.
  previous_path_ = path;
  // Observe arbitration status only for PID history, never own ControlMode/safety.
  // P-only preview during IDLE/MANUAL/FAULT permits existing startup/recovery flow
  // without accumulating windup or a circular FAULT -> invalid AUTO -> FAULT latch.
  const bool accumulate_pid = current(control_status_, timeout_s_) &&
    control_status_->value->header.frame_id == base_frame_ &&
    control_status_->value->mode == msg::ControlStatus::AUTO &&
    control_status_->value->fsm_state >= msg::ControlStatus::READY &&
    control_status_->value->fsm_state <= msg::ControlStatus::STOPPED;
  const auto result = tracking_->compute(state, path, dt_s, accumulate_pid);
  auto command = core::stop(now().nanoseconds(), result.reason);
  if (result.valid && result.goal_reached) command = core::stop(now().nanoseconds(), "GOAL_REACHED", true);
  else if (result.valid)
    command = {result.steering_rad, plan.target_speed_mps, false, true,
               now().nanoseconds(), result.reason};
  // Longitudinal policy remains MotionPlan target speed pass-through. No speed PID.
  publisher_->publish(toRos(command, base_frame_));
}
}  // namespace control_manager
