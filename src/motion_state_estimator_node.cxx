#include "control_manager/nodes.hxx"
namespace control_manager {
MotionStateEstimatorNode::MotionStateEstimatorNode() : NodeBase("motion_state_estimator") {
  const auto mode = param<std::string>("estimation_mode", "GNSS");
  if (mode != "GNSS" && mode != "WHEEL")
    throw std::invalid_argument("estimation_mode must be GNSS or WHEEL");
  mode_ = mode == "GNSS" ? core::MotionEstimationMode::GNSS : core::MotionEstimationMode::WHEEL;
  map_frame_ = param<std::string>("map_frame", "map");
  base_frame_ = param<std::string>("base_frame", "base_link");
  timeout_s_ = positive("state_timeout_s", 0.5);
  max_speed_variance_ = positive("max_speed_variance_m2ps2", 1.0);
  wheelbase_m_ = nonnegative("vehicle.wheelbase_m", 0.0);  // Supplied by vehicle.yaml; 0 = unavailable.
  if (mode_ == core::MotionEstimationMode::GNSS)
    gnss_ = input<nav_msgs::msg::Odometry>("sensor_gnss", rclcpp::SensorDataQoS());
  else RCLCPP_ERROR(get_logger(), "WHEEL_FEEDBACK_CONTRACT_NOT_IMPLEMENTED; MotionState will remain invalid");
  publisher_ = output<msg::MotionState>("motion_state");
  tick([this] { update(); });
}
void MotionStateEstimatorNode::update() {
  msg::MotionState out;
  out.header.frame_id = base_frame_;
  out.reason = "GNSS_VELOCITY_UNAVAILABLE";
  if (mode_ == core::MotionEstimationMode::WHEEL) {
    // TODO: validated actual speed + actual road-wheel angle + wheelbase_m_ > 0,
    // independent measurement freshness/calibration gates, then v/L*tan(delta).
    // No contract exists yet: no invented subscriptions, conversion, or yaw rate.
    out.reason = "WHEEL_FEEDBACK_CONTRACT_NOT_IMPLEMENTED";
  } else if (gnss_->value) {
    const auto &odom = *gnss_->value;
    out.header.stamp = odom.header.stamp;  // Twist is in child_frame, measured at source time.
    out.speed_mps = odom.twist.twist.linear.x;
    const double variance = odom.twist.covariance[0];
    out.speed_valid = current(gnss_, timeout_s_) && odom.header.frame_id == map_frame_ &&
      odom.child_frame_id == base_frame_ && std::isfinite(out.speed_mps) &&
      std::isfinite(variance) && variance >= 0.0 && variance <= max_speed_variance_;
    out.reason = out.speed_valid ? "GNSS_SPEED_VALID" : "INVALID_OR_STALE_GNSS_VELOCITY";
  }
  // GNSS yaw-rate is unobserved; do not differentiate heading or infer IMU fusion.
  out.yaw_rate_valid = false;
  publisher_->publish(out);
}
}  // namespace control_manager
