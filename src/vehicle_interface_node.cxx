#include "control_manager/nodes.hxx"
namespace control_manager {
VehicleInterfaceNode::VehicleInterfaceNode() : NodeBase("vehicle_interface") {
  const bool dry_run = param<bool>("dry_run", true);
  conversion_.wheel_diameter_m = positive("vehicle.wheel_diameter_m", 0.0);
  conversion_.steering_full_scale_deg = positive("trans.steering_full_scale_deg", 30.0);
  conversion_.motor_full_scale_rpm = positive("trans.motor_full_scale_rpm", 891.387);
  conversion_.steering_zero_deg = param<double>("trans.steering_zero_deg", 0.0);
  if (!core::validActuatorConversion(conversion_))
    throw std::invalid_argument("trans.steering_zero_deg must be finite and inside the steering full scale");
  frame_ = param<std::string>("base_frame", "base_link");
  config_.command_timeout_s = positive("command_timeout_s", 0.5);
  config_.operating_limits_verified = param<bool>("operating_limits_verified", false);
  config_.limits = vehicleLimits();
  const double degrees = 180.0 / std::acos(-1.0);
  if (config_.limits.min_steer_rad * degrees + conversion_.steering_zero_deg <
        -conversion_.steering_full_scale_deg - 1e-9 ||
      config_.limits.max_steer_rad * degrees + conversion_.steering_zero_deg >
        conversion_.steering_full_scale_deg + 1e-9)
    throw std::invalid_argument("offset steering limits exceed trans full scale; use control_console --steering-zero-deg");
  selected_ = input<msg::VehicleCommand>("selected_cmd");
  (void)topic("stop_status");  // expose common observer wiring to console discovery.
  if (!dry_run) actuator_publisher_ = output<std_msgs::msg::Float32MultiArray>("actuator_raw", rclcpp::QoS(1));
  // publish a timestamped observation alongside raw commands.
  if (!dry_run) command_event_publisher_ = create_publisher<vehicle_recorder::msg::CommandEvent>(
    param<std::string>("command_events_topic", "/vehicle/command_events"), 100);
  RCLCPP_INFO(get_logger(), "%s", dry_run ?
    "[DRY_RUN] actuator protection active; actuator_raw disabled" :
    "[ACTUATOR_RAW] right-positive steering; UDP transmission belongs to external trans");
  tick([this] { update(); });
}
void VehicleInterfaceNode::update() {
  auto command = core::stop(now().nanoseconds(), "SELECTED_COMMAND_UNAVAILABLE");
  if (current(selected_, config_.command_timeout_s) && selected_->value->header.frame_id == frame_)
    command = toCore(*selected_->value);
  // previous protected stop reset steering to zero.
  // const auto protected_command = core::protectActuatorCommand(command, now().nanoseconds(), config_);
  // update memory only AFTER final limits/freshness protection succeeds.
  const auto protected_command = stop_steering_.apply(core::protectActuatorCommand(
    command, now().nanoseconds(), config_));
  if (actuator_publisher_) {
    const auto values = core::actuatorRaw(protected_command, conversion_);
    std_msgs::msg::Float32MultiArray raw;
    raw.data.assign(values.begin(), values.end());
    // publication time is separate from trans's actual socket send time.
    vehicle_recorder::msg::CommandEvent event;
    event.header = header(frame_);
    event.command_topic = actuator_publisher_->get_topic_name();
    event.stage = vehicle_recorder::msg::CommandEvent::COMMAND;
    event.steering_raw = values[0]; event.speed_raw = values[1]; event.brake = values[2] != 0.0F;
    actuator_publisher_->publish(raw);
    command_event_publisher_->publish(event);
  }
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    "[%s] steer=%.4f rad speed=%.3f m/s brake=%d valid=%d reason=%s",
    actuator_publisher_ ? "ACTUATOR_RAW" : "DRY_RUN",
    protected_command.target_steer_rad, protected_command.target_speed_mps,
    protected_command.target_brake, protected_command.valid, protected_command.reason.c_str());
  // Existing trans consumes actuator_raw; this node owns no actuator socket or packet encoder.
}
}  // namespace control_manager
