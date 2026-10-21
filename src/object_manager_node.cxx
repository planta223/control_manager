#include "control_manager/nodes.hxx"
namespace control_manager {
ObjectManagerNode::ObjectManagerNode() : NodeBase("object_manager") {
  if (!enabled("object_state_enabled", false)) return;  // NONE: no result inputs, output or timer.
  bool any_input = false;
  for (const auto &key : {"cone", "obstacle", "traffic"}) {
    if (!enabled(std::string(key) + "_enabled", false)) continue;
    any_input = true;
    subscriptions_.push_back(create_generic_subscription(topic(std::string("sensor_") + key),
      perceptionType(key), rclcpp::SensorDataQoS(),
      [](std::shared_ptr<rclcpp::SerializedMessage>) {
        // TODO: agreed result -> ObjectState adapter; receipt alone is not validity.
      }));
  }
  if (!any_input)
    throw std::invalid_argument("object_state_enabled requires at least one of cone_enabled/obstacle_enabled/traffic_enabled");
  publisher_ = output<msg::ObjectState>("object_state");
  const auto frame = param<std::string>("base_frame", "base_link");
  RCLCPP_WARN(get_logger(), "Result-to-ObjectState adapter not implemented: fields remain invalid/UNKNOWN");
  tick([this, frame] {
    msg::ObjectState state;
    state.header = header(frame);  // Unavailability report time, NOT a detection time.
    state.traffic_light_state = msg::ObjectState::UNKNOWN;
    publisher_->publish(state);
  });
  // TODO: parse external results and aggregate per-observation source freshness.
}
}  // namespace control_manager
