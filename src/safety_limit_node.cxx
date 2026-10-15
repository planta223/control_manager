#include "control_manager/nodes.hxx"
namespace control_manager {
SafetyLimitNode::SafetyLimitNode() : NodeBase("safety_limit") {
  frame_ = param<std::string>("base_frame", "base_link");
  map_frame_ = param<std::string>("map_frame", "map");
  config_.operating_limits_verified = param<bool>("operating_limits_verified", false);
  config_.max_steer_rad = nonnegative("max_steer_rad", 0.0);
  config_.max_speed_mps = nonnegative("max_speed_mps", 0.0);
  config_.emergency_distance_m = nonnegative("emergency_distance_m", 0.0);
  config_.state_timeout_s = positive("state_timeout_s", 0.5);
  config_.command_timeout_s = positive("command_timeout_s", 0.5);
  object_timeout_s_ = positive("object_timeout_s", 0.5);
  require_obstacle_state_ = param<bool>("require_obstacle_state", true);
  pose_ = input<msg::GlobalPose>("global_pose");
  motion_ = input<msg::MotionState>("motion_state");
  guidance_ = input<msg::GuidanceStatus>("guidance_status");
  selected_ = input<msg::VehicleCommand>("selected_cmd");
  objects_ = input<msg::ObjectState>("object_state");
  publisher_ = output<msg::VehicleCommand>("final_cmd");
  subscriptions_.push_back(create_subscription<std_msgs::msg::Bool>(topic("estop"),
    rclcpp::QoS(1).reliable().transient_local(), [this](std_msgs::msg::Bool::ConstSharedPtr value) {
      estop_ = value->data;  // True persists until an explicit false/reset. All modes.
    }));
  tick([this] { update(); });
}
void SafetyLimitNode::update() {
  auto command = core::stop(now().nanoseconds(), "SELECTED_COMMAND_UNAVAILABLE");
  if (current(selected_, config_.command_timeout_s) && selected_->value->header.frame_id == frame_)
    command = toCore(*selected_->value);
  core::GlobalPose pose;
  core::MotionState motion;
  if (current(pose_, config_.state_timeout_s) && pose_->value->header.frame_id == map_frame_)
    pose = toCore(*pose_->value);
  if (current(motion_, config_.state_timeout_s) && motion_->value->header.frame_id == frame_)
    motion = toCore(*motion_->value);
  // Only the planner's fresh LANE request removes the global dependency. Missing,
  // stale or unknown status conservatively requires GlobalPose (also in MANUAL).
  // Do not require status.valid: LANE can be unimplemented without requiring GNSS.
  const bool require_global_pose = !(current(guidance_, config_.state_timeout_s) &&
    guidance_->value->header.frame_id == map_frame_ &&
    guidance_->value->requested_mode == msg::GuidanceStatus::LANE);
  core::ObstacleObservation obstacle;
  const bool object_valid = current(objects_, object_timeout_s_) && objects_->value->valid &&
    objects_->value->obstacle_valid && objects_->value->header.frame_id == frame_;
  if (object_valid) obstacle = {true, objects_->value->obstacle_detected,
                               objects_->value->nearest_obstacle_distance_m};
  auto final = core::applySafety(command, pose, motion, require_global_pose, obstacle, estop_, now().nanoseconds(), config_);
  if (!estop_ && require_obstacle_state_ && !object_valid)
    final = core::stop(now().nanoseconds(), "OBSTACLE_STATE_UNAVAILABLE");
  publisher_->publish(toRos(final, frame_));
}
}  // namespace control_manager
