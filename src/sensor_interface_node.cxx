#include "control_manager/nodes.hxx"
namespace control_manager {
SensorInterfaceNode::SensorInterfaceNode() : NodeBase("sensor_interface") {
  relay<nav_msgs::msg::Odometry>("gnss", "sensor_gnss");
  relay<sensor_msgs::msg::Image>("rgb", "sensor_rgb");
  relay<sensor_msgs::msg::PointCloud2>("lidar", "sensor_lidar");
  if (param<bool>("imu_enabled", false)) relay<sensor_msgs::msg::Imu>("imu", "sensor_imu");
  if (param<bool>("depth_enabled", false)) relay<sensor_msgs::msg::Image>("depth", "sensor_depth");
  // Transparent boundary: preserve sensor stamps, frames and covariance. No TF guess.
}
}  // namespace control_manager
