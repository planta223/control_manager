#include "control_manager/nodes.hxx"
namespace control_manager {
StateEstimatorNode::StateEstimatorNode() : NodeBase("state_estimator") {
  const auto mode = param<std::string>("estimation_mode", "GNSS_ONLY");
  const bool fusion = param<bool>("fusion_enable", false);
  if (mode != "GNSS_ONLY" || fusion)
    throw std::invalid_argument("GNSS_IMU/fusion not implemented; only GNSS_ONLY with fusion_enable=false is supported");
  map_frame_ = param<std::string>("map_frame", "map");
  base_frame_ = param<std::string>("base_frame", "base_link");
  reference_verified_ = param<bool>("localization_reference_verified", false);
  timeout_s_ = positive("state_timeout_s", 0.5);
  max_position_std_m_ = positive("max_position_std_m", 1.0);
  max_yaw_variance_ = positive("max_yaw_variance_rad2", 0.25);
  max_speed_variance_ = positive("max_speed_variance_m2ps2", 1.0);
  gnss_ = input<nav_msgs::msg::Odometry>("sensor_gnss", rclcpp::SensorDataQoS());
  publisher_ = output<msg::VehicleState>("vehicle_state");
  if (!reference_verified_) RCLCPP_WARN(get_logger(), "Rear-axle reference / GNSS lever arm not verified; localization remains invalid");
  tick([this] { update(); });
}
void StateEstimatorNode::update() {
  msg::VehicleState out;
  out.header.frame_id = map_frame_;
  // With no sensor, timestamp stays zero. Invalid heartbeats never fabricate observations.
  if (gnss_->value) {
    const auto &odom = *gnss_->value;
    out.header = odom.header;
    out.x_m = odom.pose.pose.position.x; out.y_m = odom.pose.pose.position.y;
    const auto &q = odom.pose.pose.orientation;
    const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    const bool orientation_valid = std::isfinite(norm) && std::abs(norm - 1.0) < 1e-3;
    if (orientation_valid)
      out.yaw_rad = std::atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));
    out.speed_mps = odom.twist.twist.linear.x;
    const auto variance_ok = [](double v, double maximum) {
      return std::isfinite(v) && v >= 0.0 && v <= maximum;
    };
    const auto &cov = odom.pose.covariance;
    const bool position_ok = variance_ok(cov[0], max_position_std_m_*max_position_std_m_) &&
      variance_ok(cov[7], max_position_std_m_*max_position_std_m_) &&
      cov[0] + cov[7] <= max_position_std_m_*max_position_std_m_;
    const bool frames_ok = odom.header.frame_id == map_frame_ && odom.child_frame_id == base_frame_;
    out.localization_valid = reference_verified_ && current(gnss_, timeout_s_) && frames_ok &&
      position_ok && variance_ok(cov[35], max_yaw_variance_) && orientation_valid &&
      std::isfinite(out.x_m) && std::isfinite(out.y_m);
    out.speed_valid = current(gnss_, timeout_s_) && frames_ok && std::isfinite(out.speed_mps) &&
      variance_ok(odom.twist.covariance[0], max_speed_variance_);
    // GNSS yaw-rate is unobserved. TODO: calibrated optional IMU assist, no EKF here.
    out.yaw_rate_valid = false;
  }
  publisher_->publish(out);
}
}  // namespace control_manager
