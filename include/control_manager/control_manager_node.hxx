#pragma once

#include <rclcpp/rclcpp.hpp>

namespace control_manager {

class ControlManagerNode : public rclcpp::Node {
 public:
  explicit ControlManagerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
};

}  // namespace control_manager
