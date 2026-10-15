# Read-only diagnostic of the actuator-facing protection result, NOT a command input.
# dry_run=true means no hardware transmission, not measured actuator feedback.
# Header is diagnostic publication time; protected_command retains source time.
std_msgs/Header header
bool dry_run
control_manager/VehicleCommand protected_command
