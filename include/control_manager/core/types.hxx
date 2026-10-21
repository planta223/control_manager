#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace control_manager::core {
enum class ControlMode : std::uint8_t { IDLE, MANUAL, AUTO };
enum class VehicleFsmState : std::uint8_t { INIT, READY, DRIVING, STOPPED, FAULT };
enum class PoseMode { GNSS, GNSS_IMU, NONE };  // Fusion is NOT_IMPLEMENTED.
enum class MotionMode { GNSS, GNSS_IMU, WHEEL, NONE };  // Wheel feedback contract is pending.
struct GlobalPose {
  double x_m{0.0}, y_m{0.0}, yaw_rad{0.0};
  double position_variance_x_m2{0.0}, position_variance_y_m2{0.0}, yaw_variance_rad2{0.0};
  bool valid{false};
  std::int64_t timestamp_ns{0};
};
struct MotionState {
  double speed_mps{0.0}, yaw_rate_radps{0.0};
  bool speed_valid{false}, yaw_rate_valid{false};
  std::int64_t timestamp_ns{0};
};
// Algorithm input assembled AFTER independent pose/motion validation in motion_control.
// Not an estimator output, ROS contract, or cached combined state.
struct TrackingState {
  double x_m{0.0}, y_m{0.0}, yaw_rad{0.0}, speed_mps{0.0}, yaw_rate_radps{0.0};
  bool localization_valid{false}, speed_valid{false}, yaw_rate_valid{false};
  std::int64_t timestamp_ns{0};
};
struct Point { double x_m{0.0}, y_m{0.0}; };
struct PathMatch {
  bool valid{false};
  std::size_t segment{0};
  double fraction{0.0}, distance_m{0.0};
};
struct LanePose {
  double lateral_error_m{0.0}, heading_error_rad{0.0}, confidence{0.0};
  std::int64_t timestamp_ns{0};
};
struct Command {
  double target_steer_rad{0.0}, target_speed_mps{0.0};
  bool target_brake{true}, valid{false};
  std::int64_t timestamp_ns{0};
  std::string reason;
};
struct VehicleLimits {
  double min_steer_rad{0.0}, max_steer_rad{0.0};
  double min_speed_mps{0.0}, max_speed_mps{0.0};
};
bool validVehicleLimits(const VehicleLimits &limits);
Command limitAutomaticCommand(const Command &command, const VehicleLimits &limits);
struct ActuatorProtectionConfig {
  bool operating_limits_verified{false};
  VehicleLimits limits;
  double command_timeout_s{0.5};
};
// Existing trans input: normalized right-positive steering, forward-positive motor speed, brake.
struct ActuatorConversionConfig {
  double wheel_diameter_m{0.0}, motor_to_wheel_ratio{1.0};
  double steering_full_scale_deg{30.0}, motor_full_scale_rpm{1500.0};
};
bool validActuatorConversion(const ActuatorConversionConfig &config);
// 고규범 수정 (2026-10-21): stop/invalid commands retain the last valid moving command.
// Keep one instance per output stage; never substitute unverified steering feedback.
class StopSteeringHold {
 public:
  Command apply(Command command);
 private:
  double last_steer_rad_{0.0};  // No prior command at process startup: neutral reference only.
};
// 고규범 수정: resolve all stop/invalid angles with StopSteeringHold before raw conversion.
std::array<float, 3> actuatorRaw(const Command &command, const ActuatorConversionConfig &config);
bool fresh(std::int64_t stamp_ns, std::int64_t now_ns, double timeout_s);
Command stop(std::int64_t now_ns, const std::string &reason, bool valid = false);
Command selectCommand(ControlMode mode, const Command &manual, const Command &automatic,
                      std::int64_t now_ns);
Command protectActuatorCommand(const Command &command,
                               std::int64_t now_ns, const ActuatorProtectionConfig &config);
VehicleFsmState fsmState(bool has_final, bool final_fresh, const Command &final,
                         ControlMode mode);
}  // namespace control_manager::core
