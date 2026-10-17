#include "control_manager/core/tracking.hxx"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace control_manager::core {
PidLateralController::PidLateralController(PidConfig config) : config_(config) {
  for (double value : {config.kp_lateral, config.ki_lateral, config.kd_lateral,
                      config.kp_heading, config.integral_limit_m_s, config.max_steering_rad,
                      config.goal_tolerance_m})
    if (!std::isfinite(value) || value < 0.0) throw std::invalid_argument("invalid PID gain/limit");
  if (!std::isfinite(config.max_dt_s) || config.max_dt_s <= 0.0)
    throw std::invalid_argument("PID max_dt_s must be positive");
}
void PidLateralController::reset() {
  previous_valid_ = false; integral_m_s_ = 0.0; previous_error_m_ = 0.0;
}
TrackingResult PidLateralController::compute(const TrackingState &s,
                                            const std::vector<Point> &path, double dt_s, const PathMatch *match) {
  TrackingResult out;
  auto fail = [this, &out](const std::string &reason) {
    reset(); out.valid = false; out.steering_rad = 0.0; out.reason = reason; return out;
  };
  if (!config_.enabled || config_.max_steering_rad <= 0.0 ||
      (config_.kp_lateral == 0.0 && config_.ki_lateral == 0.0 &&
       config_.kd_lateral == 0.0 && config_.kp_heading == 0.0))
    return fail("PID_NOT_CONFIGURED");
  if (!std::isfinite(dt_s) || dt_s <= 0.0 || dt_s > config_.max_dt_s)
    return fail("PID_INVALID_DT");
  const auto error = pathError(s, path, match);
  if (!error.valid) return fail("INVALID_TRACKING_GEOMETRY");
  out.lateral_error_m = error.lateral_error_m;
  out.heading_error_rad = error.heading_error_rad;
  if (goalReached(error,config_.goal_tolerance_m)) {
    reset(); out.valid = true; out.goal_reached = true; out.reason = "GOAL_REACHED"; return out;
  }
  // Forward-only baseline; no steering guesses beyond the last segment or for a U-turn.
  if ((error.final_segment && error.goal_forward_m <= 0.0) ||
      std::cos(error.heading_error_rad) <= 0.0) return fail("PID_PATH_NOT_FORWARD");
  double derivative_mps = 0.0;
  double integral_m_s = integral_m_s_;
  if (previous_valid_) {
    derivative_mps = (error.lateral_error_m - previous_error_m_) / dt_s;
    const double candidate_m_s = integral_m_s_ + error.lateral_error_m*dt_s;
    if (!std::isfinite(candidate_m_s)) return fail("PID_NONFINITE_RESULT");
    integral_m_s = std::clamp(candidate_m_s, -config_.integral_limit_m_s, config_.integral_limit_m_s);
  }
  // First sample after any reset has zero I and D (no derivative kick on re-entry).
  const double steer_rad = config_.kp_lateral*error.lateral_error_m +
    config_.ki_lateral*integral_m_s + config_.kd_lateral*derivative_mps +
    config_.kp_heading*error.heading_error_rad;
  if (!std::isfinite(derivative_mps) || !std::isfinite(steer_rad)) return fail("PID_NONFINITE_RESULT");
  out.steering_rad = std::clamp(steer_rad, -config_.max_steering_rad, config_.max_steering_rad);
  out.valid = true; out.reason = "PID_ACTIVE";
  integral_m_s_ = integral_m_s; previous_error_m_ = error.lateral_error_m; previous_valid_ = true;
  return out;
}
}  // namespace control_manager::core
