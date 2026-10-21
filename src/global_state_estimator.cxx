#include "control_manager/nodes.hxx"
#include "control_manager/ingress.hxx"
namespace control_manager {
GlobalStateEstimatorNode::GlobalStateEstimatorNode() : NodeBase("global_state_estimator") {
  const auto pose_mode = param<std::string>("pose_mode", "GNSS");
  if (pose_mode == "GNSS") pose_mode_ = core::PoseMode::GNSS;
  else if (pose_mode == "GNSS_IMU") pose_mode_ = core::PoseMode::GNSS_IMU;
  else if (pose_mode == "NONE") pose_mode_ = core::PoseMode::NONE;
  else throw std::invalid_argument("pose_mode must be GNSS, GNSS_IMU or NONE");
  const auto motion_mode = param<std::string>("motion_mode", "GNSS");
  if (motion_mode == "GNSS") motion_mode_ = core::MotionMode::GNSS;
  else if (motion_mode == "GNSS_IMU") motion_mode_ = core::MotionMode::GNSS_IMU;
  else if (motion_mode == "WHEEL") motion_mode_ = core::MotionMode::WHEEL;
  else if (motion_mode == "NONE") motion_mode_ = core::MotionMode::NONE;
  else throw std::invalid_argument("motion_mode must be GNSS, GNSS_IMU, WHEEL or NONE");
  map_frame_ = param<std::string>("map_frame", "map");
  base_frame_ = param<std::string>("base_frame", "base_link");
  timeout_s_ = positive("state_timeout_s", 0.5);
  max_position_std_m_ = positive("max_position_std_m", 3.0);
  max_yaw_variance_ = positive("max_yaw_variance_rad2", 0.25);
  max_speed_variance_ = positive("max_speed_variance_m2ps2", 1.0);
  wheelbase_m_ = nonnegative("vehicle.wheelbase_m", 0.0);  // vehicle.yaml; 0 means unavailable.

  // Shared transport, independent output dependencies. All inputs are ingress-owned.
  if (pose_mode_ != core::PoseMode::NONE || motion_mode_ == core::MotionMode::GNSS ||
      motion_mode_ == core::MotionMode::GNSS_IMU) {
    if (!enabled("gnss_enabled", true))
      throw std::invalid_argument("selected pose/motion mode requires gnss_enabled=true");
    gnss_ = input<nav_msgs::msg::Odometry>("sensor_gnss", rclcpp::SensorDataQoS());
  }
  if (pose_mode_ == core::PoseMode::GNSS_IMU || motion_mode_ == core::MotionMode::GNSS_IMU) {
    if (!enabled("imu_enabled", false))
      throw std::invalid_argument("GNSS_IMU mode requires imu_enabled=true (algorithm still NOT_IMPLEMENTED)");
    imu_ = input<sensor_msgs::msg::Imu>("sensor_imu", rclcpp::SensorDataQoS());
  }
  // WHEEL is reserved/unimplemented; no placeholder feedback subscription.
  pose_publisher_ = output<msg::GlobalPose>("global_pose");
  motion_publisher_ = output<msg::MotionState>("motion_state");
  if (pose_mode_ == core::PoseMode::GNSS_IMU || motion_mode_ == core::MotionMode::GNSS_IMU)
    RCLCPP_ERROR(get_logger(), "GNSS_IMU_NOT_IMPLEMENTED for the selected output(s); no fusion or gyro frame assumption");
  if (motion_mode_ == core::MotionMode::WHEEL)
    RCLCPP_ERROR(get_logger(), "WHEEL_FEEDBACK_CONTRACT_NOT_IMPLEMENTED; MotionState remains invalid");
  tick([this] { update(); });
}
void GlobalStateEstimatorNode::update() {
  pose_publisher_->publish(estimatePose());
  motion_publisher_->publish(estimateMotion());  // Never gated by pose validity or pose mode.
}
msg::GlobalPose GlobalStateEstimatorNode::estimatePose() {
  msg::GlobalPose out;
  out.header.frame_id = map_frame_;
  if (pose_mode_ == core::PoseMode::NONE) {
    out.reason = "POSE_DISABLED";  // Normal configuration; zero stamp and invalid pose.
    return out;
  }
  if (pose_mode_ == core::PoseMode::GNSS_IMU) {
    // TODO verified extrinsics, covariance, time alignment and fusion algorithm.
    out.reason = "GNSS_IMU_NOT_IMPLEMENTED";
    return out;
  }
  out.reason = "GNSS_UNAVAILABLE";
  if (gnss_->value) {
    const auto &odom = *gnss_->value;
    if (!ingress::validStampFormat(odom.header.stamp)) {
      out.reason = "INVALID_GNSS_TIMESTAMP";
      return out;  // Invalid with stamp=0; never propagate an unrepresentable time.
    }
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
    // GNSS owns the antenna-to-rear-axle correction; preserve its base_link pose.
    out.position_valid = current(gnss_, timeout_s_) && frames_ok && position_ok &&
      std::isfinite(out.x_m) && std::isfinite(out.y_m);
    // GNSS contract: unobserved yaw is 1e12; legacy uses 1e11 as its boundary.
    if (current(gnss_, timeout_s_) && frames_ok && orientation_valid &&
        std::isfinite(cov[35]) && cov[35] >= 0.0 && cov[35] < 1e11) have_valid_heading_ = true;
    out.initial_heading_unavailable = !have_valid_heading_ && orientation_valid &&
      std::isfinite(cov[35]) && cov[35] >= 1e11;
    out.heading_unavailable = current(gnss_, timeout_s_) && frames_ok && orientation_valid &&
      std::isfinite(cov[35]) && cov[35] > max_yaw_variance_;
    out.valid = out.position_valid && variance_ok(cov[35], max_yaw_variance_) && orientation_valid;
    out.position_variance_x_m2 = cov[0];
    out.position_variance_y_m2 = cov[7];
    out.yaw_variance_rad2 = cov[35];
    out.reason = out.valid ? "GNSS_POSE_VALID" : "INVALID_OR_STALE_GNSS_POSE";
  }
  return out;  // Source measurement stamp is retained, including on expiration.
}
msg::MotionState GlobalStateEstimatorNode::estimateMotion() {
  msg::MotionState out;
  out.header.frame_id = base_frame_;
  if (motion_mode_ == core::MotionMode::NONE) {
    out.reason = "MOTION_DISABLED";  // Normal configuration; independent of GlobalPose.
    return out;
  }
  if (motion_mode_ == core::MotionMode::GNSS_IMU) {
    // Intent: GNSS speed + IMU gyro yaw rate. Sensor-to-body rotation, source
    // validity and time alignment are not verified, so neither value is claimed valid.
    out.reason = "GNSS_IMU_NOT_IMPLEMENTED";
    return out;
  }
  if (motion_mode_ == core::MotionMode::WHEEL) {
    // TODO verified motor/steering calibration, source time and alignment, then v/L*tan(delta).
    // Raw encoder relays are available, but no estimator adapter is implemented.
    // using wheelbase_m_ from vehicle.yaml. No WHEEL algorithm exists yet.
    out.reason = "WHEEL_FEEDBACK_CONTRACT_NOT_IMPLEMENTED";
    return out;
  }
  out.reason = "GNSS_VELOCITY_UNAVAILABLE";
  if (gnss_->value) {
    const auto &odom = *gnss_->value;
    if (!ingress::validStampFormat(odom.header.stamp)) {
      out.reason = "INVALID_GNSS_TIMESTAMP";
      return out;  // Invalid with stamp=0; never normalize a malformed wire stamp.
    }
    out.header.stamp = odom.header.stamp;  // Twist is in child_frame, measured at source time.
    out.speed_mps = odom.twist.twist.linear.x;
    const double variance = odom.twist.covariance[0];
    out.speed_valid = current(gnss_, timeout_s_) && odom.header.frame_id == map_frame_ &&
      odom.child_frame_id == base_frame_ && std::isfinite(out.speed_mps) &&
      std::isfinite(variance) && variance >= 0.0 && variance <= max_speed_variance_;
    out.reason = out.speed_valid ? "GNSS_SPEED_VALID" : "INVALID_OR_STALE_GNSS_VELOCITY";
  }
  // GNSS yaw-rate is unobserved. Do not differentiate heading or invent gyro fusion.
  out.yaw_rate_valid = false;
  return out;
}
}  // namespace control_manager
