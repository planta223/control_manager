#include "control_manager/nodes.hxx"
#include "control_manager/ingress.hxx"
#include <algorithm>
namespace control_manager {
MotionControlNode::MotionControlNode() : NodeBase("motion_control") {
  frame_ = param<std::string>("map_frame", "map");
  base_frame_ = param<std::string>("base_frame", "base_link");
  timeout_s_ = positive("input_timeout_s", 0.5);
  const auto mode = param<std::string>("tracking_mode", "PID");
  if (mode != "PID" && mode != "PURE_PURSUIT")
    throw std::invalid_argument("tracking_mode must be PID or PURE_PURSUIT");
  limits_ = vehicleLimits();
  core::PurePursuitConfig config;
  config.wheelbase_m = positive("vehicle.wheelbase_m", 1.328);
  config.lookahead_min_m = positive("lookahead_min_m", 2.0);
  config.lookahead_max_m = positive("lookahead_max_m", 6.0);
  config.lookahead_gain_s = nonnegative("lookahead_gain_s", 1.0);
  config.goal_tolerance_m = nonnegative("goal_tolerance_m", 0.3);
  goal_tolerance_m_=config.goal_tolerance_m;
  core::PidConfig pid;
  pid.enabled = param<bool>("pid.enabled", false);
  pid.kp_lateral = nonnegative("pid.kp_lateral", 0.0);
  pid.ki_lateral = nonnegative("pid.ki_lateral", 0.0);
  pid.kd_lateral = nonnegative("pid.kd_lateral", 0.0);
  pid.kp_heading = nonnegative("pid.kp_heading", 0.0);
  pid.integral_limit_m_s = nonnegative("pid.integral_limit_m_s", 0.0);
  pid.max_steering_rad = std::max(-limits_.min_steer_rad, limits_.max_steer_rad);
  pid.max_dt_s = positive("pid.max_dt_s", 0.5);
  pid.goal_tolerance_m = config.goal_tolerance_m;
  max_cross_track_m_ = positive("max_cross_track_m", 3.0);
  heading_bootstrap_timeout_s_ = positive("heading_bootstrap_timeout_s", 3.0);
  core::StoppedHeadingConfig heading_memory;
  heading_memory.enabled = param<bool>("stopped_heading.enabled", true);
  heading_memory.stop_speed_mps = nonnegative("stopped_heading.stop_speed_mps", 0.1);
  heading_memory.reacquire_speed_mps = positive("stopped_heading.reacquire_speed_mps", 0.7);
  heading_memory.max_distance_m = positive("stopped_heading.max_distance_m", 1.0);
  heading_memory.max_reacquire_time_s = positive("stopped_heading.max_reacquire_time_s", 3.0);
  stopped_heading_ = std::make_unique<core::StoppedHeadingMemory>(heading_memory);
  heading_bootstrap_enabled_ = param<bool>("heading_bootstrap_enabled", false) &&
    (mode == "PURE_PURSUIT" || (pid.enabled &&
     (pid.kp_lateral > 0.0 || pid.ki_lateral > 0.0 || pid.kd_lateral > 0.0 || pid.kp_heading > 0.0)));
  tracking_ = std::make_unique<core::TrackingController>(pid, config);
  if (param<bool>("route_progress_enabled", false)) {
    const auto count=param<std::int64_t>("route_progress.expected_points",0);
    const auto front=param<std::int64_t>("route_progress.front_end_index",0);
    const auto rear=param<std::int64_t>("route_progress.rear_start_index",0);
    if (count<4 || front<=0 || rear<=front || rear>=count-1)
      throw std::invalid_argument("route_progress point-count / phase indices do not define three phases");
    core::RouteProgressConfig progress;
    progress.expected_points=static_cast<std::size_t>(count);
    progress.phase_end_indices={static_cast<std::size_t>(front),static_cast<std::size_t>(rear)};
    progress.initial_search_m=positive("route_progress.initial_search_m",10.0);
    progress.search_back_m=positive("route_progress.search_back_m",3.0);
    progress.search_forward_m=positive("route_progress.search_forward_m",10.0);
    route_progress_=std::make_unique<core::RouteProgressTracker>(progress);
  }
  if (mode == "PURE_PURSUIT") tracking_->request(static_cast<std::uint8_t>(core::TrackingMode::PURE_PURSUIT));
  pose_ = input<msg::GlobalPose>("global_pose");
  motion_ = input<msg::MotionState>("motion_state");
  plan_ = input<msg::MotionPlan>("motion_plan");
  control_status_ = std::make_shared<Input<msg::ControlStatus>>();
  subscriptions_.push_back(create_subscription<msg::ControlStatus>(topic("status"), 10,
    [this](msg::ControlStatus::ConstSharedPtr status) {
      if (!control_status_->value || control_status_->value->mode != status->mode ||
          status->mode != msg::ControlStatus::AUTO || status->fsm_state == msg::ControlStatus::FAULT) {
        tracking_->reset();
        goal_approach_seen_=false;
      }
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
    goal_approach_seen_=false;
    publisher_->publish(toRos(core::stop(now().nanoseconds(), reason, valid), base_frame_));
  };
  if (run_completed_) { stop("GOAL_REACHED", true); return; }
  const auto selection_error = tracking_->selectionError();
  if (!selection_error.empty()) { stop(selection_error); return; }
  const auto current_input = [this](const auto &in) {
    return in && in->value && ingress::validStampFormat(in->value->header.stamp) &&
      current(in, timeout_s_);
  };
  if (current_input(control_status_) && control_status_->value->header.frame_id == base_frame_) {
    const auto &status = *control_status_->value;
    paused_ = status.mode == msg::ControlStatus::AUTO && status.paused;
    if (status.resume_sequence < resume_sequence_) {
      stop("RESUME_SEQUENCE_REGRESSED_RESTART_REQUIRED"); return;
    }
    if (status.resume_sequence > resume_sequence_) {
      resume_sequence_ = status.resume_sequence;
      resume_after_ns_ = rclcpp::Time(status.header.stamp).nanoseconds();
      if (!have_valid_heading_) heading_bootstrap_start_.reset();
    }
  }
  const bool auto_status = current_input(control_status_) &&
    control_status_->value->header.frame_id == base_frame_ && control_status_->value->mode == msg::ControlStatus::AUTO;
  const bool heading_observations = auto_status && current_input(pose_) && current_input(motion_) &&
    pose_->value->header.frame_id == frame_ && pose_->value->position_valid &&
    std::isfinite(pose_->value->yaw_rad) &&
    (pose_->value->valid || pose_->value->initial_heading_unavailable || pose_->value->heading_unavailable) &&
    motion_->value->header.frame_id == base_frame_ && motion_->value->speed_valid;
  const bool intentional_stop = auto_status && (paused_ ||
    (current_input(plan_) && plan_->value->valid && plan_->value->header.frame_id == frame_ &&
     (plan_->value->stop_request || plan_->value->target_speed_mps == 0.0)));
  const auto memory = stopped_heading_->update(
    std::chrono::duration<double>(tick_time.time_since_epoch()).count(), heading_observations, intentional_stop,
    pose_->value ? pose_->value->x_m : 0.0, pose_->value ? pose_->value->y_m : 0.0,
    motion_->value ? motion_->value->speed_mps : 0.0,
    heading_observations && pose_->value->valid ? std::optional<double>(pose_->value->yaw_rad) : std::nullopt);
  const bool held_heading = memory.yaw_rad.has_value();
  if (heading_observations && pose_->value->valid) have_valid_heading_ = true;
  // Remember heading even during an intentional pause; do not advance route progress.
  if (paused_) { stop("AUTO_PAUSED", true); return; }
  // Do not advance the remembered route using samples cached during the pause.
  if (resume_after_ns_ > 0) {
    if (!current_input(pose_) || !current_input(motion_) ||
        rclcpp::Time(pose_->value->header.stamp).nanoseconds() < resume_after_ns_ ||
        rclcpp::Time(motion_->value->header.stamp).nanoseconds() < resume_after_ns_) {
      stop("RESUME_WAITING_FOR_FRESH_STATE"); return;
    }
    resume_after_ns_ = 0;
  }
  // After acquisition a resume never grants another blind straight-drive window.
  if (current_input(pose_) && pose_->value->header.frame_id == frame_ &&
      pose_->value->valid && std::isfinite(pose_->value->yaw_rad)) {
    have_valid_heading_ = true;
  }
  // Existing PID/PP require a pose and path in the same global frame. A local
  // lane path needs an agreed tracking-frame adapter, not a fabricated zero pose.
  if (current_input(plan_) && plan_->value->valid &&
      (plan_->value->header.frame_id != frame_ ||
       (!plan_->value->reference_path.header.frame_id.empty() &&
        plan_->value->reference_path.header.frame_id != frame_))) {
    stop("LOCAL_REFERENCE_PATH_NOT_SUPPORTED"); return;
  }
  if (!current_input(pose_) || !current_input(motion_) || !current_input(plan_) ||
      (!pose_->value->valid && !(pose_->value->position_valid &&
       (held_heading || (!have_valid_heading_ && pose_->value->initial_heading_unavailable)))) ||
      !motion_->value->speed_valid ||
      motion_->value->header.frame_id != base_frame_ || pose_->value->header.frame_id != frame_ || !plan_->value->valid ||
      plan_->value->header.frame_id != frame_ || !std::isfinite(plan_->value->target_speed_mps) ||
      plan_->value->target_speed_mps < 0.0) {
    stop(memory.reason.empty() ? "INVALID_OR_STALE_STATE_OR_PLAN" : memory.reason); return;
  }
  const auto &plan = *plan_->value;
  auto state = trackingInput(*pose_->value, *motion_->value);
  if (held_heading) {
    state.yaw_rad = *memory.yaw_rad;
    state.localization_valid = true;  // Control-local input; GlobalPose.valid is unchanged.
  }
  // Stop plans retain the previous speed/brake policy but do not claim valid tracking.
  if (!std::isfinite(state.x_m) || !std::isfinite(state.y_m) || !std::isfinite(state.yaw_rad) ||
      !std::isfinite(state.speed_mps) || state.speed_mps < 0.0) {
    stop("INVALID_TRACKING_STATE"); return;
  }
  if (plan.stop_request || plan.target_speed_mps == 0.0) { stop("PLAN_STOP", true); return; }
  if (!ingress::validStampFormat(plan.reference_path.header.stamp) ||
      !validPath(plan.reference_path, frame_) ||
      !core::fresh(rclcpp::Time(plan.reference_path.header.stamp).nanoseconds(),
                   now().nanoseconds(), timeout_s_)) {
    stop("INVALID_OR_STALE_REFERENCE_PATH"); return;
  }
  std::vector<core::Point> path;
  path.reserve(plan.reference_path.poses.size());
  for (const auto &p : plan.reference_path.poses) path.push_back({p.pose.position.x, p.pose.position.y});
  std::optional<core::PathMatch> route_match;
  if (route_progress_) {
    const auto progress=route_progress_->update(state.x_m,state.y_m,
      (pose_->value->valid || held_heading) ? std::optional<double>(state.yaw_rad) : std::nullopt,path,max_cross_track_m_);
    if (!progress.match.valid) { stop(progress.reason); return; }
    route_match=progress.match;
    if (progress.phase!=route_phase_) {
      route_phase_=progress.phase;
      RCLCPP_INFO(get_logger(),"Route phase=%s progress=%.2f m",
        route_phase_==1 ? "MIDDLE" : "REAR",progress.progress_m);
    }
  } else {
    const double path_distance = core::distanceToPath(state.x_m, state.y_m, path);
    if (!std::isfinite(path_distance)) { stop("INVALID_TRACKING_GEOMETRY"); return; }
    if (path_distance > max_cross_track_m_) { stop("PATH_DEVIATION_EXCEEDED"); return; }
  }
  if (!pose_->value->valid && !held_heading) {
    if (have_valid_heading_) {
      stop("HEADING_UNAVAILABLE_AFTER_ACQUISITION"); return;
    }
    if (!heading_bootstrap_enabled_) { stop("HEADING_BOOTSTRAP_DISABLED"); return; }
    if (!current_input(control_status_) || control_status_->value->header.frame_id != base_frame_) {
      stop("HEADING_BOOTSTRAP_CONTROL_STATUS_UNAVAILABLE"); return;
    }
    if (control_status_->value->mode != msg::ControlStatus::AUTO || control_status_->value->paused) {
      stop("HEADING_BOOTSTRAP_WAITING_FOR_AUTO", true); return;
    }
    if (!heading_bootstrap_start_) heading_bootstrap_start_ = tick_time;
    if (std::chrono::duration<double>(tick_time - *heading_bootstrap_start_).count() >=
        heading_bootstrap_timeout_s_) {
      stop("HEADING_BOOTSTRAP_TIMEOUT"); return;
    }
    tracking_->reset(); previous_path_.clear();
    const core::Command straight{0.0, plan.target_speed_mps, false, true,
                                now().nanoseconds(), "HEADING_BOOTSTRAP_ACTIVE"};
    publisher_->publish(toRos(core::limitAutomaticCommand(straight, limits_), base_frame_));
    return;  // Never run PID / Pure Pursuit using an unavailable yaw.
  }
  const bool same_path = path.size() == previous_path_.size() &&
    std::equal(path.begin(), path.end(), previous_path_.begin(),
      [](const core::Point &a, const core::Point &b) { return a.x_m == b.x_m && a.y_m == b.y_m; });
  if (!same_path) {
    tracking_->reset();  // Path changes must not carry integral/derivative history.
    goal_approach_seen_=false;
  }
  previous_path_ = path;
  // Selected-command status controls PID history and end passage; final protection stays in Vehicle Interface.
  // P-only preview during IDLE/MANUAL/FAULT permits existing startup/recovery flow
  // without accumulating windup or a circular FAULT -> invalid AUTO -> FAULT latch.
  const bool accumulate_pid = current_input(control_status_) &&
    control_status_->value->header.frame_id == base_frame_ &&
    control_status_->value->mode == msg::ControlStatus::AUTO &&
    control_status_->value->fsm_state >= msg::ControlStatus::READY &&
    control_status_->value->fsm_state <= msg::ControlStatus::STOPPED;
  const auto error=core::pathError(state,path,route_match ? &*route_match : nullptr);
  auto result = tracking_->compute(state, path, dt_s, accumulate_pid,
                                    route_match ? &*route_match : nullptr);
  // A sample may jump over the arrival radius. Require an observed AUTO approach
  // on this route, association near the end, and passage inside the end corridor.
  if (accumulate_pid && goal_approach_seen_ && error.valid &&
      error.remaining_path_m<=goal_tolerance_m_ && error.goal_forward_m<=0.0 &&
      std::abs(error.goal_lateral_m)<=goal_tolerance_m_)
    result={0.0,true,true,error.lateral_error_m,error.heading_error_rad,"GOAL_REACHED"};
  goal_approach_seen_=accumulate_pid && result.valid && !result.goal_reached && error.valid &&
    error.goal_forward_m>0.0 && error.remaining_path_m<=max_cross_track_m_ &&
    error.goal_distance_m<=max_cross_track_m_;
  auto command = core::stop(now().nanoseconds(), result.reason);
  if (result.valid && result.goal_reached) {
    // IDLE/MANUAL previews must not finish a run that has not been selected as AUTO.
    run_completed_ = current_input(control_status_) &&
      control_status_->value->header.frame_id == base_frame_ &&
      control_status_->value->mode == msg::ControlStatus::AUTO;
    command = core::stop(now().nanoseconds(), "GOAL_REACHED", true);
  }
  else if (result.valid)
    command = {result.steering_rad, plan.target_speed_mps, false, true,
               now().nanoseconds(), held_heading ? "HEADING_HOLD_ACTIVE:" + result.reason : result.reason};
  command = core::limitAutomaticCommand(command, limits_);
  // MotionPlan target speed capped at the vehicle limit. No speed PID or reverse tracking.
  publisher_->publish(toRos(command, base_frame_));
}
}  // namespace control_manager
