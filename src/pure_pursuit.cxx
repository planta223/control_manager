#include "control_manager/core/tracking.hxx"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace control_manager::core {
AdaptivePurePursuit::AdaptivePurePursuit(PurePursuitConfig config) : config_(config) {
  if (!std::isfinite(config.wheelbase_m) || config.wheelbase_m <= 0.0 ||
      !std::isfinite(config.lookahead_min_m) || config.lookahead_min_m <= 0.0 ||
      !std::isfinite(config.lookahead_max_m) || config.lookahead_max_m < config.lookahead_min_m ||
      !std::isfinite(config.lookahead_gain_s) || config.lookahead_gain_s < 0.0 ||
      !std::isfinite(config.goal_tolerance_m) || config.goal_tolerance_m < 0.0)
    throw std::invalid_argument("invalid Pure Pursuit geometry/tuning");
}
LateralResult AdaptivePurePursuit::compute(const TrackingState &s,
                                          const std::vector<Point> &path) const {
  return computeImpl(s,path,nullptr);
}
LateralResult AdaptivePurePursuit::compute(const TrackingState &s,
                                          const std::vector<Point> &path, const PathMatch &match) const {
  return computeImpl(s,path,&match);
}
LateralResult AdaptivePurePursuit::computeImpl(const TrackingState &s,
                                             const std::vector<Point> &path, const PathMatch *match) const {
  const auto error=pathError(s,path,match);
  if (!error.valid) return {};
  if (goalReached(error,config_.goal_tolerance_m)) return {0.0,true,true};
  const auto segment=error.match.segment;
  const auto fraction=error.match.fraction;
  const Point projected{path[segment].x_m+fraction*(path[segment+1].x_m-path[segment].x_m),
                        path[segment].y_m+fraction*(path[segment+1].y_m-path[segment].y_m)};
  double remaining = std::clamp(config_.lookahead_min_m + config_.lookahead_gain_s * s.speed_mps,
                                config_.lookahead_min_m, config_.lookahead_max_m);
  Point target = path.back(), start = projected;
  for (std::size_t i = segment + 1; i < path.size(); ++i) {
    const double length = std::hypot(path[i].x_m - start.x_m, path[i].y_m - start.y_m);
    if (length > 0.0 && remaining <= length) {
      const double t = remaining / length;
      target = {start.x_m + t * (path[i].x_m - start.x_m),
                start.y_m + t * (path[i].y_m - start.y_m)};
      break;
    }
    remaining -= length;
    start = path[i];
  }
  const double dx = target.x_m - s.x_m, dy = target.y_m - s.y_m;
  const double forward = std::cos(s.yaw_rad) * dx + std::sin(s.yaw_rad) * dy;
  const double left = -std::sin(s.yaw_rad) * dx + std::cos(s.yaw_rad) * dy;
  const double chord2 = dx * dx + dy * dy;
  if (forward <= 0.0 || chord2 <= std::numeric_limits<double>::epsilon()) return {};
  const double angle = std::atan2(2.0 * config_.wheelbase_m * left, chord2);
  return {angle, std::isfinite(angle), false};
}
}  // namespace control_manager::core
