#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "control_manager/control_manager_node.hxx"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<control_manager::ControlManagerNode>();
    rclcpp::spin(node);
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("control_manager"), "%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
