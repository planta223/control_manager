#include "control_manager/nodes.hxx"
#include "control_manager/object_manager/traffic_light.hxx"
#include "control_manager/object_manager/traffic_sign.hxx"
#include "control_manager/object_manager/obstacle.hxx"
namespace control_manager {
ObjectManagerNode::ObjectManagerNode() : NodeBase("object_manager") {
  input<sensor_msgs::msg::Image>("sensor_rgb", rclcpp::SensorDataQoS());
  input<sensor_msgs::msg::PointCloud2>("sensor_lidar", rclcpp::SensorDataQoS());
  if (param<bool>("depth_enabled", false)) input<sensor_msgs::msg::Image>("sensor_depth", rclcpp::SensorDataQoS());
  publisher_ = output<msg::ObjectState>("object_state");
  const auto frame = param<std::string>("base_frame", "base_link");
  RCLCPP_WARN(get_logger(), "Perception not implemented: ObjectState fields remain invalid/UNKNOWN");
  tick([this, frame] {
    msg::ObjectState state;
    state.header = header(frame);  // Unavailability report time, NOT a detection time.
    state.traffic_light_state = msg::ObjectState::UNKNOWN;
    publisher_->publish(state);
  });
  // TODO: instantiate detectors after validation; aggregate per-observation freshness.
  // A received RGB/point cloud alone must NEVER mark a perception result valid.
}
}  // namespace control_manager
