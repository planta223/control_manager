#pragma once
#include <cstdint>
#include <optional>
#include <string>

namespace control_manager::core {
// Guidance is independent of ControlMode and EstimationMode.
enum class GuidanceMode : std::uint8_t { GPS_ONLY = 0, LANE_ONLY = 1, HYBRID = 2 };
struct GuidanceDecision {
  std::uint8_t requested_mode{0};  // Preserve invalid wire requests for diagnostics.
  std::optional<GuidanceMode> actual_mode;
  std::string active_source{"NONE"};
  bool valid{false};
  bool degraded{false};  // Reserved for an explicitly implemented fallback policy.
  std::string reason;
};
// Availability is validated by the ROS adapter (timestamp, frame, finite values).
// LaneState cannot enable either unsupported algorithm; GPS_ONLY ignores it.
GuidanceDecision evaluateGuidance(std::uint8_t requested_mode,
                                  bool localization_available, bool global_path_available);
}  // namespace control_manager::core
