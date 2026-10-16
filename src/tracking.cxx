#include "control_manager/core/tracking.hxx"
namespace control_manager::core {
TrackingController::TrackingController(PidConfig pid_config, PurePursuitConfig pp_config)
    : pid_(pid_config), legacy_pp_(pp_config) {}
void TrackingController::request(std::uint8_t mode) {
  if (mode != requested_) reset();
  requested_ = mode;
}
void TrackingController::reset() { pid_.reset(); }
std::string TrackingController::selectionError() const {
  if (requested_ != static_cast<std::uint8_t>(TrackingMode::PID) &&
      requested_ != static_cast<std::uint8_t>(TrackingMode::PURE_PURSUIT))
    return "INVALID_TRACKING_MODE";
  return {};
}
TrackingResult TrackingController::compute(const TrackingState &s, const std::vector<Point> &path,
                                            double dt_s, bool accumulate_pid) {
  TrackingResult out;
  out.reason = selectionError();
  if (!out.reason.empty()) { reset(); return out; }
  if (requested_ == static_cast<std::uint8_t>(TrackingMode::PID)) {
    if (!accumulate_pid) reset();
    out = pid_.compute(s, path, dt_s);
    if (!accumulate_pid) {
      reset();
      if (out.valid && !out.goal_reached) out.reason = "PID_PREVIEW_STATE_RESET";
    }
    return out;
  }
  const auto error = pathError(s, path);
  if (!error.valid) { out.reason = "INVALID_TRACKING_GEOMETRY"; return out; }
  const auto result = legacy_pp_.compute(s, path);
  return {result.steering_rad, result.valid, result.goal_reached,
          error.lateral_error_m, error.heading_error_rad,
          result.valid ? (result.goal_reached ? "GOAL_REACHED" : "PURE_PURSUIT_ACTIVE")
                       : "PURE_PURSUIT_COMPUTE_FAILED"};
}
}  // namespace control_manager::core
