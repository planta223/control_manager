#include "control_manager/nodes.hxx"
#include "control_manager/ingress.hxx"
#include <algorithm>
namespace control_manager {
MotionPlanningNode::MotionPlanningNode() : NodeBase("motion_planning") {
  frame_ = param<std::string>("map_frame", "map");
  base_frame_ = param<std::string>("base_frame", "base_link");
  timeout_s_ = positive("input_timeout_s", 0.5);
  cruise_speed_mps_ = nonnegative("cruise_speed_mps", nonnegative("cruise_speed_kph", 0.0) / 3.6);
  if (param<bool>("speed_profile.enabled", false)) {
    const auto limits=vehicleLimits();
    core::SpeedProfileConfig config{std::min(cruise_speed_mps_,limits.max_speed_mps),
      vehicleAccelerationLimit("max_lateral_accel_g", "speed_profile.max_lateral_accel_mps2", 9.80665, true),
      vehicleAccelerationLimit("max_decel_kphps", "speed_profile.planned_decel_mps2", 1.0 / 3.6, true),
      positive("vehicle.wheelbase_m",1.328),std::min(-limits.min_steer_rad,limits.max_steer_rad)};
    config.curvature_sample_m=positive("speed_profile.curvature_sample_m",1.0);
    config.response_time_s=nonnegative("speed_profile.response_time_s",0.5);
    config.stop_before_end_m=nonnegative("goal_tolerance_m",0.3);
    config.max_accel_mps2=vehicleAccelerationLimit("max_accel_kphps", "speed_profile.max_accel_mps2", 1.0/3.6, true);
    // The planned-deceleration lookup above already declared the shared ceiling.
    const double decel_ceiling=get_parameter("vehicle.limits.max_decel_kphps").as_double()/3.6;
    const double command_decel=positive("speed_profile.max_decel_mps2",decel_ceiling);
    if (decel_ceiling>0.0 && command_decel>decel_ceiling+1e-12)
      throw std::invalid_argument("speed_profile.max_decel_mps2 exceeds vehicle.limits.max_decel_kphps");
    if (config.planned_decel_mps2>command_decel+1e-12)
      throw std::invalid_argument("planned deceleration must not exceed speed profile command deceleration");
    speed_profile_=std::make_unique<core::PathSpeedProfile>(config);
    speed_reference_=std::make_unique<core::SpeedReferencePlanner>(config.max_accel_mps2, command_decel, config.max_speed_mps);
    planning_status_=input<msg::ControlStatus>("status");
    max_path_distance_m_=positive("speed_profile.max_path_distance_m",3.0);
    pose_=input<msg::GlobalPose>("global_pose");
    if (param<bool>("route_progress_enabled",false)) {
      const auto n=param<std::int64_t>("route_progress.expected_points",0),
                 front=param<std::int64_t>("route_progress.front_end_index",0),
                 rear=param<std::int64_t>("route_progress.rear_start_index",0);
      if (n<4 || front<=0 || rear<=front || rear>=n-1)
        throw std::invalid_argument("invalid speed planner route progress profile");
      core::RouteProgressConfig progress;
      progress.expected_points=static_cast<std::size_t>(n);
      progress.phase_end_indices={static_cast<std::size_t>(front),static_cast<std::size_t>(rear)};
      progress.initial_search_m=positive("route_progress.initial_search_m",10.0);
      progress.search_back_m=positive("route_progress.search_back_m",3.0);
      progress.search_forward_m=positive("route_progress.search_forward_m",10.0);
      speed_progress_=std::make_unique<core::RouteProgressTracker>(progress);
    }
  }
  motion_ = input<msg::MotionState>("motion_state");
  path_ = input<nav_msgs::msg::Path>("reference_path");
  objects_enabled_ = enabled("object_state_enabled", true);
  mission_enabled_ = enabled("mission_limit_enabled", true);
  if (objects_enabled_) objects_ = input<msg::ObjectState>("object_state");
  if (mission_enabled_) limit_ = input<msg::MissionLimit>("mission_limit");
  publisher_ = output<msg::MotionPlan>("motion_plan");
  planning_tick_=std::chrono::steady_clock::now();
  tick([this] { update(); });
}
void MotionPlanningNode::update() {
  const auto tick_time=std::chrono::steady_clock::now();
  const double dt_s=std::chrono::duration<double>(tick_time-planning_tick_).count();
  planning_tick_=tick_time;
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
    if (speed_profile_ && !plan.stop_request) {
      std::string error;
      if (!current_input(pose_) || !pose_->value->position_valid ||
          pose_->value->header.frame_id!=frame_ || path_->value->header.frame_id!=frame_ ||
          !std::isfinite(pose_->value->x_m) || !std::isfinite(pose_->value->y_m) ||
          (pose_->value->valid && !std::isfinite(pose_->value->yaw_rad)))
        error="SPEED_PROFILE_POSE_UNAVAILABLE";
      std::vector<core::Point> points;
      if (error.empty()) {
        for (const auto &p:path_->value->poses) points.push_back({p.pose.position.x,p.pose.position.y});
        error=speed_profile_->setPath(points);
      }
      core::PathMatch match;
      if (error.empty() && speed_progress_) {
        const auto progress=speed_progress_->update(pose_->value->x_m,pose_->value->y_m,
          pose_->value->valid ? std::optional<double>(pose_->value->yaw_rad) : std::nullopt,
          points,max_path_distance_m_);
        match=progress.match;
        if (!match.valid) error=progress.reason;
      } else if (error.empty()) {
        auto state=trackingInput(*pose_->value,*motion_->value);
        state.localization_valid=true;  // Position-only planning; heading admission stays in Motion Control.
        if (!std::isfinite(state.yaw_rad)) state.yaw_rad=0.0;
        match=core::pathError(state,points).match;
        if (!match.valid || match.distance_m>max_path_distance_m_) error="SPEED_PROFILE_PATH_DEVIATION";
      }
      if (error.empty()) {
        auto sample=speed_profile_->sample(match,motion_->value->speed_mps);
        if (!current_input(planning_status_) || planning_status_->value->header.frame_id!=base_frame_)
          error="SPEED_PLAN_STATUS_UNAVAILABLE";
        else if (planning_status_->value->mode!=msg::ControlStatus::AUTO || planning_status_->value->paused) {
          speed_reference_->reset();
          planning_resume_sequence_=planning_status_->value->resume_sequence;
          sample.target_speed_mps=0.0; sample.reason="SPEED_PLAN_PAUSED";
        } else {
          if (!planning_resume_sequence_ || *planning_resume_sequence_!=planning_status_->value->resume_sequence)
            speed_reference_->reset();
          planning_resume_sequence_=planning_status_->value->resume_sequence;
          // Mission caps participate in the same reference planning as map caps.
          sample.target_speed_mps=std::min(speed,sample.target_speed_mps);
          sample=speed_reference_->update(sample,motion_->value->speed_mps,dt_s);
        }
        if (!sample.valid) error=sample.reason;
        else if (error.empty()) {
          plan.target_speed_mps=sample.target_speed_mps;
          plan.stop_request=plan.target_speed_mps==0.0;
          plan.reason=sample.reason;
        }
      }
      if (!error.empty()) {
        plan.valid=false; plan.stop_request=true; plan.target_speed_mps=0.0; plan.reason=error;
      }
    }
    if (!objects_enabled_) plan.reason += ":OBJECT_DISABLED";
    if (!mission_enabled_) plan.reason += ":MISSION_DISABLED";
  }
  if (speed_reference_ && (!plan.valid || plan.stop_request)) speed_reference_->reset();
  publisher_->publish(plan);
}
}  // namespace control_manager
