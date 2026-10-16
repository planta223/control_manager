#pragma once
#include <cstdint>
#include <string>

namespace control_manager::core {
// Guidance is independent of ControlMode and both estimator modes.
enum class GuidanceMode : std::uint8_t { GPS = 0, LANE = 1, HYBRID = 2 };
struct GuidanceDecision {
  bool valid{false};
  std::string reason;
};
// Availability is validated by the ROS adapter (timestamp, frame, finite values).
// LanePose cannot enable either unsupported algorithm; GPS ignores it.
GuidanceDecision evaluateGuidance(std::uint8_t requested_mode,
                                  bool global_pose_available, bool global_path_available,
                                  const std::string &lane_error);
}  // namespace control_manager::core
