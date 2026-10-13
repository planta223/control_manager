#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace control_manager::core {
enum class ControlMode : std::uint8_t { IDLE, MANUAL, AUTO };
enum class VehicleFsmState : std::uint8_t { INIT, READY, DRIVING, STOPPED, FAULT };
enum class EstimationMode { GNSS_ONLY, GNSS_IMU };
struct VehicleState {
  double x_m{0.0}, y_m{0.0}, yaw_rad{0.0}, speed_mps{0.0}, yaw_rate_radps{0.0};
  bool localization_valid{false}, speed_valid{false}, yaw_rate_valid{false};
  std::int64_t timestamp_ns{0};
};
struct Point { double x_m{0.0}, y_m{0.0}; };
struct LaneState {
  double lateral_error_m{0.0}, heading_error_rad{0.0}, confidence{0.0};
  bool valid{false};
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
Command applySafety(const Command &command, const VehicleState &state,
                    const ObstacleObservation &obstacle, bool estop,
                    std::int64_t now_ns, const SafetyConfig &config);
VehicleFsmState fsmState(bool has_final, bool final_fresh, const Command &final,
                         ControlMode mode);
}  // namespace control_manager::core
