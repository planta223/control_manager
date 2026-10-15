#include "control_manager/nodes.hxx"
#include "control_manager/ingress.hxx"
namespace control_manager {
LanePoseEstimatorNode::LanePoseEstimatorNode() : NodeBase("lane_state_estimator") {
  timeout_s_ = positive("state_timeout_s", 0.5);
  publisher_ = output<msg::LanePose>("lane_pose");
  auto subscription = create_subscription<msg::LanePose>(topic("lane_observation"), 10,
    [this](msg::LanePose::ConstSharedPtr observation) { observationCallback(observation); });
  if (std::string(subscription->get_topic_name()) == publisher_->get_topic_name())
    throw std::invalid_argument("lane_observation and lane_pose must resolve to different topics");
  subscriptions_.push_back(subscription);
}
void LanePoseEstimatorNode::observationCallback(msg::LanePose::ConstSharedPtr observation) {
  // Minimal state admission, not a Lane AI algorithm: reject aged transport samples,
  // non-finite errors and confidence outside [0,1]. Planning retains its confidence threshold.
  if (!ingress::admissibleLane(*observation, now().nanoseconds(), timeout_s_)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Rejected lane observation: invalid fields/confidence or stale source timestamp");
    return;
  }
  publisher_->publish(*observation);  // Preserve errors, confidence, frame and source stamp.
  // No periodic replay: silence expires in the planner. No geometry/filter is invented.
}
}  // namespace control_manager
