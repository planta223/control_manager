#include <exception>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "control_manager/nodes.hxx"
#include "control_manager/control_manager_node.hxx"
#ifndef CONTROL_MANAGER_NODE_CLASS
#error "Build target must specify its node class"
#endif
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<control_manager::CONTROL_MANAGER_NODE_CLASS>();
    rclcpp::spin(node);
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("control_manager"), "%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
