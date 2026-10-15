#include "control_manager/nodes.hxx"
namespace control_manager {
GnssStateEstimatorNode::GnssStateEstimatorNode() : NodeBase("gnss_state_estimator") {
  const auto mode = param<std::string>("estimation_mode", "GNSS");
  if (mode != "GNSS" && mode != "GNSS_IMU")
    throw std::invalid_argument("estimation_mode must be GNSS or GNSS_IMU");
  mode_ = mode == "GNSS" ? core::GlobalEstimationMode::GNSS : core::GlobalEstimationMode::GNSS_IMU;
  if (mode_ == core::GlobalEstimationMode::GNSS_IMU) {
    imu_ = input<sensor_msgs::msg::Imu>("sensor_imu", rclcpp::SensorDataQoS());
    RCLCPP_ERROR(get_logger(), "GNSS_IMU_NOT_IMPLEMENTED; GlobalPose will remain invalid");
  }
  map_frame_ = param<std::string>("map_frame", "map");
  base_frame_ = param<std::string>("base_frame", "base_link");
  reference_verified_ = param<bool>("localization_reference_verified", false);
  timeout_s_ = positive("state_timeout_s", 0.5);
  max_position_std_m_ = positive("max_position_std_m", 1.0);
  max_yaw_variance_ = positive("max_yaw_variance_rad2", 0.25);
  gnss_ = input<nav_msgs::msg::Odometry>("sensor_gnss", rclcpp::SensorDataQoS());
  publisher_ = output<msg::GlobalPose>("global_pose");
  if (!reference_verified_) RCLCPP_WARN(get_logger(), "Rear-axle reference / GNSS lever arm not verified; localization remains invalid");
  tick([this] { update(); });
}
void GnssStateEstimatorNode::update() {
  msg::GlobalPose out;
  out.header.frame_id = map_frame_;
  // With no sensor, timestamp stays zero. Invalid heartbeats never fabricate observations.
  out.reason = "GNSS_UNAVAILABLE";
  if (mode_ == core::GlobalEstimationMode::GNSS_IMU) {
    out.reason = "GNSS_IMU_NOT_IMPLEMENTED";
    publisher_->publish(out);
    return;
  }
  if (gnss_->value) {
    const auto &odom = *gnss_->value;
    out.header = odom.header;
    out.x_m = odom.pose.pose.position.x; out.y_m = odom.pose.pose.position.y;
    const auto &q = odom.pose.pose.orientation;
    const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    const bool orientation_valid = std::isfinite(norm) && std::abs(norm - 1.0) < 1e-3;
    if (orientation_valid)
      out.yaw_rad = std::atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));
    const auto variance_ok = [](double v, double maximum) {
      return std::isfinite(v) && v >= 0.0 && v <= maximum;
    };
    const auto &cov = odom.pose.covariance;
    const bool position_ok = variance_ok(cov[0], max_position_std_m_*max_position_std_m_) &&
      variance_ok(cov[7], max_position_std_m_*max_position_std_m_) &&
      cov[0] + cov[7] <= max_position_std_m_*max_position_std_m_;
    const bool frames_ok = odom.header.frame_id == map_frame_ && odom.child_frame_id == base_frame_;
    out.valid = reference_verified_ && current(gnss_, timeout_s_) && frames_ok &&
      position_ok && variance_ok(cov[35], max_yaw_variance_) && orientation_valid &&
      std::isfinite(out.x_m) && std::isfinite(out.y_m);
    out.position_variance_x_m2 = cov[0];
    out.position_variance_y_m2 = cov[7];
    out.yaw_variance_rad2 = cov[35];
    out.reason = out.valid ? "GNSS_POSE_VALID" : "INVALID_OR_STALE_GNSS_POSE";
  }
  publisher_->publish(out);
}
}  // namespace control_manager
