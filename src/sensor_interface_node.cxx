#include "control_manager/nodes.hxx"
#include "control_manager/ingress.hxx"
#include <encoder_motor/msg/motor_feedback.hpp>
#include <encoder_motor/msg/steering_feedback.hpp>
namespace control_manager {
SensorInterfaceNode::SensorInterfaceNode() : NodeBase("sensor_interface") {
  // Single ingress for real-time external observations and actual vehicle feedback.
  // Map/Path and operator/manual commands have separate owners.
  if (enabled("gnss_enabled", true)) relay<nav_msgs::msg::Odometry>("gnss", "sensor_gnss");
  for (const auto &key : {"cone", "obstacle", "traffic"})
    if (enabled(std::string(key) + "_enabled", false)) perceptionRelay(key);
  // 미구현: IMU 융합 / depth 판단 / Lane 경로 생성. 아래는 기존 입력 전달만 수행.
  if (param<bool>("imu_enabled", false)) relay<sensor_msgs::msg::Imu>("imu", "sensor_imu");
  if (param<bool>("depth_enabled", false)) relay<sensor_msgs::msg::Image>("depth", "sensor_depth");
  // Raw relay remains transparent: preserve stamp/frame/covariance, no TF guess.
  input_timeout_s_ = positive("input_timeout_s", 0.5);
  // Encoder observations follow the GNSS raw-relay contract: do not convert
  // units/signs, renew MCU timestamps, discard invalidity, or combine clocks.
  if (enabled("motor_feedback_enabled", false))
    relay<encoder_motor::msg::MotorFeedback>("external_motor_feedback", "sensor_motor_feedback");
  if (enabled("steering_feedback_enabled", false))
    relay<encoder_motor::msg::SteeringFeedback>("external_steering_feedback", "sensor_steering_feedback");
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
}
void SensorInterfaceNode::perceptionRelay(const std::string &key) {
  const auto source = topic("detect_" + key), destination = topic("sensor_" + key);
  if (source.empty() || destination.empty())
    throw std::invalid_argument(key + " requires nonempty input/output topics");
  const auto type = perceptionType(key);
  auto publisher = create_generic_publisher(destination, type, rclcpp::SensorDataQoS());
  auto subscription = create_generic_subscription(source, type, rclcpp::SensorDataQoS(),
    [publisher](std::shared_ptr<rclcpp::SerializedMessage> sample) { publisher->publish(*sample); });
  if (std::string(subscription->get_topic_name()) == publisher->get_topic_name())
    throw std::invalid_argument("perception input and output resolve to the same topic: " + key);
  // Preserve the complete result, including the source timestamp and valid=false.
  // Parsing/mission decisions belong to the later Object Manager adapter.
  subscriptions_.push_back(subscription);
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
}  // namespace control_manager
