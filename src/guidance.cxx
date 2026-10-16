#include "control_manager/core/guidance.hxx"

namespace control_manager::core {
GuidanceDecision evaluateGuidance(std::uint8_t requested_mode, bool global_pose_available,
                                  bool global_path_available, const std::string &lane_error) {
  GuidanceDecision out;
  switch (requested_mode) {
    case static_cast<std::uint8_t>(GuidanceMode::GPS):
      if (!global_pose_available) out.reason = "GLOBAL_POSE_UNAVAILABLE";
      else if (!global_path_available) out.reason = "GLOBAL_PATH_UNAVAILABLE";
      else {
        out.valid = true;
        out.reason = "GPS_ACTIVE";
      }
      break;
    case static_cast<std::uint8_t>(GuidanceMode::LANE):
      // No dependency on global pose/map. Four lane observations do not define a path.
      out.reason = lane_error.empty() ? "LANE_NOT_IMPLEMENTED" : lane_error;
      break;
    case static_cast<std::uint8_t>(GuidanceMode::HYBRID):
      if (!global_pose_available) out.reason = "GLOBAL_POSE_UNAVAILABLE";
      else if (!global_path_available) out.reason = "GLOBAL_PATH_UNAVAILABLE";
      else out.reason = lane_error.empty() ? "HYBRID_NOT_IMPLEMENTED" : lane_error;
      break;
    default: out.reason = "INVALID_GUIDANCE_MODE"; break;
  }
  return out;  // No silent fallback; unsupported algorithms never produce a usable path.
}
}  // namespace control_manager::core
