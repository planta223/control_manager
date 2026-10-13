#include "control_manager/core/guidance.hxx"

namespace control_manager::core {
GuidanceDecision evaluateGuidance(std::uint8_t requested_mode,
                                  bool localization_available, bool global_path_available) {
  GuidanceDecision out;
  out.requested_mode = requested_mode;
  switch (requested_mode) {
    case static_cast<std::uint8_t>(GuidanceMode::GPS_ONLY):
      if (!localization_available) out.reason = "LOCALIZATION_UNAVAILABLE";
      else if (!global_path_available) out.reason = "GLOBAL_PATH_UNAVAILABLE";
      else {
        out.actual_mode = GuidanceMode::GPS_ONLY;
        out.active_source = "GLOBAL_PATH";
        out.valid = true;
        out.reason = "GPS_ONLY_ACTIVE";
      }
      break;
    case static_cast<std::uint8_t>(GuidanceMode::LANE_ONLY):
      out.reason = "LANE_ONLY_NOT_IMPLEMENTED";
      break;
    case static_cast<std::uint8_t>(GuidanceMode::HYBRID):
      // TODO: implement bounded lane correction and explicit degraded GPS/lane
      // fallback policy before enabling HYBRID. Never report fake success.
      out.reason = "HYBRID_NOT_IMPLEMENTED";
      break;
    default:
      out.reason = "INVALID_GUIDANCE_MODE";
      break;
  }
  return out;
}
}  // namespace control_manager::core
