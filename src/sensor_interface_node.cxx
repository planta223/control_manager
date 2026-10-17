#include "control_manager/nodes.hxx"
#include "control_manager/ingress.hxx"
namespace control_manager {
SensorInterfaceNode::SensorInterfaceNode() : NodeBase("sensor_interface") {
  // Single ingress for real-time external observations and actual vehicle feedback.
  // Map/Path and operator/manual commands have separate owners.
  if (enabled("gnss_enabled", true)) relay<nav_msgs::msg::Odometry>("gnss", "sensor_gnss");
  if (enabled("rgb_enabled", true)) relay<sensor_msgs::msg::Image>("rgb", "sensor_rgb");
  if (enabled("lidar_enabled", true)) relay<sensor_msgs::msg::PointCloud2>("lidar", "sensor_lidar");
  if (param<bool>("imu_enabled", false)) relay<sensor_msgs::msg::Imu>("imu", "sensor_imu");
  if (param<bool>("depth_enabled", false)) relay<sensor_msgs::msg::Image>("depth", "sensor_depth");
  // Raw relay remains transparent: preserve stamp/frame/covariance, no TF guess.
  input_timeout_s_ = positive("input_timeout_s", 0.5);
  base_frame_ = param<std::string>("base_frame", "base_link");
  // Future raw motor/wheel and steering encoders enter through this node only.
  // Reserve configurable inputs without guessing a message type or raw conversion.
  (void)topic("external_motor_feedback");
  (void)topic("external_steering_feedback");
  const bool motor_enabled = enabled("motor_feedback_enabled", false);
  const bool steering_enabled = enabled("steering_feedback_enabled", false);
  if (motor_enabled || steering_enabled)
    throw std::invalid_argument(
      "motor/steering encoder adapter not implemented; keep motor_feedback_enabled and "
      "steering_feedback_enabled=false until type/unit/calibration/time contracts are agreed");
  const auto external_lane = topic("external_lane");
  if (enabled("lane_enabled", false)) {
    if (external_lane.empty())
      throw std::invalid_argument("lane_enabled requires a nonempty topics.external_lane");
    lane_publisher_ = output<msg::LanePose>("lane_observation");
    // Explicit opt-in adapter for an external publisher using the four-field LanePose
    // wire shape. A different external type needs a conversion callback here, not in planning.
    auto subscription = create_subscription<msg::LanePose>(external_lane, rclcpp::SensorDataQoS(),
      [this](msg::LanePose::ConstSharedPtr sample) { laneCallback(sample); });
    const auto source = std::string(subscription->get_topic_name());
    const auto final_lane = get_node_topics_interface()->resolve_topic_name(topic("lane_pose"));
    if (source == lane_publisher_->get_topic_name() || source == final_lane)
      throw std::invalid_argument("external_lane, lane_observation and lane_pose must resolve to different topics");
    subscriptions_.push_back(subscription);
  }

  // No external feedback type/topic/scale is known. Publish explicit unavailable
  // state with zero stamp/false flags, never synthetic or last-command feedback.
  if (enabled("vehicle_feedback_enabled", true)) {
    feedback_publisher_ = output<msg::VehicleFeedback>("vehicle_feedback");
    tick([this] {
      msg::VehicleFeedback unavailable;
      unavailable.header.frame_id = base_frame_;
      publishFeedback(unavailable);
    });
  }
}
void SensorInterfaceNode::laneCallback(msg::LanePose::ConstSharedPtr sample) {
  if (!ingress::admissibleLane(*sample, now().nanoseconds(), input_timeout_s_)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Rejected Lane observation: non-finite/range error or invalid/stale source timestamp");
    // Do not fabricate an invalid observation or renew the last good stamp. The
    // planner expires its last accepted observation using its existing timeout.
    return;
  }
  lane_publisher_->publish(*sample);  // Exact values, source timestamp and frame.
}
void SensorInterfaceNode::publishFeedback(const msg::VehicleFeedback &sample) {
  feedback_publisher_->publish(ingress::checkedFeedback(sample, now().nanoseconds(),
                                                       input_timeout_s_, base_frame_));
  // No yaw-rate, filtering, fusion, reference path or estimated motion here.
}
}  // namespace control_manager
