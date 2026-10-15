#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace control_manager::core {
enum class ControlMode : std::uint8_t { IDLE, MANUAL, AUTO };
enum class VehicleFsmState : std::uint8_t { INIT, READY, DRIVING, STOPPED, FAULT };
enum class GlobalEstimationMode { GNSS, GNSS_IMU };  // Fusion is NOT_IMPLEMENTED.
enum class MotionEstimationMode { GNSS, WHEEL };  // Wheel feedback contract is pending.
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
struct LaneState {
  double lateral_error_m{0.0}, longitudinal_error_m{0.0}, confidence{0.0};
  std::int64_t timestamp_ns{0};
};
struct Command {
  double target_steer_rad{0.0}, target_speed_mps{0.0};
  bool target_brake{true}, valid{false};
  std::int64_t timestamp_ns{0};
  std::string reason;
};
struct SafetyConfig {
  bool operating_limits_verified{false};
  double max_steer_rad{0.0}, max_speed_mps{0.0};
  double state_timeout_s{0.5}, command_timeout_s{0.5};
  double emergency_distance_m{0.0};
};
struct ObstacleObservation {
  bool valid{false}, detected{false};
  double distance_m{0.0};
};
bool fresh(std::int64_t stamp_ns, std::int64_t now_ns, double timeout_s);
Command stop(std::int64_t now_ns, const std::string &reason, bool valid = false);
Command selectCommand(ControlMode mode, const Command &manual, const Command &automatic,
                      std::int64_t now_ns);
Command applySafety(const Command &command, const GlobalPose &pose, const MotionState &motion,
                    bool require_global_pose,
                    const ObstacleObservation &obstacle, bool estop,
                    std::int64_t now_ns, const SafetyConfig &config);
VehicleFsmState fsmState(bool has_final, bool final_fresh, const Command &final,
                         ControlMode mode);
}  // namespace control_manager::core
