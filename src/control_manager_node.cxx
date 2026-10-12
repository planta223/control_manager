#include "control_manager/control_manager_node.hxx"

#include <stdexcept>
#include <string>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace control_manager {

ControlManagerNode::ControlManagerNode(const rclcpp::NodeOptions &options)
    : Node("control_manager", options) {
  rcl_interfaces::msg::ParameterDescriptor mode_descriptor;
  mode_descriptor.description = "Scaffold mode. Only idle is implemented.";
  mode_descriptor.read_only = true;
  const auto mode = declare_parameter<std::string>("mode", "idle", mode_descriptor);

  rcl_interfaces::msg::ParameterDescriptor dry_run_descriptor;
  dry_run_descriptor.description = "Scaffold operation. Must remain true.";
  dry_run_descriptor.read_only = true;
  const auto dry_run = declare_parameter<bool>("dry_run", true, dry_run_descriptor);

  if (mode != "idle" || !dry_run) {
    throw std::invalid_argument(
        "control_manager scaffold supports only mode=idle and dry_run=true; "
        "autonomous/manual control and actuator output are not implemented");
  }

  RCLCPP_INFO(get_logger(),
              "Scaffold ready: mode=idle, dry_run=true. "
              "Autonomous/manual control and actuator output are not implemented.");
}

}  // namespace control_manager
