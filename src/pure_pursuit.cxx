#include "control_manager/core/lateral_controller.hxx"
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
  if (!s.localization_valid || !s.speed_valid || path.size() < 2 ||
      !std::isfinite(s.x_m) || !std::isfinite(s.y_m) || !std::isfinite(s.yaw_rad) ||
      !std::isfinite(s.speed_mps) || s.speed_mps < 0.0) return {};
  for (const auto &p : path)
    if (!std::isfinite(p.x_m) || !std::isfinite(p.y_m)) return {};

  // Project onto the nearest non-degenerate segment, then advance by arc length.
  double best = std::numeric_limits<double>::infinity();
  std::size_t segment = 0;
  Point projected;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double dx = path[i + 1].x_m - path[i].x_m;
    const double dy = path[i + 1].y_m - path[i].y_m;
    const double length2 = dx * dx + dy * dy;
    if (length2 <= std::numeric_limits<double>::epsilon()) continue;
    const double t = std::clamp(((s.x_m - path[i].x_m) * dx +
                                (s.y_m - path[i].y_m) * dy) / length2, 0.0, 1.0);
    const Point p{path[i].x_m + t * dx, path[i].y_m + t * dy};
    const double distance = std::hypot(s.x_m - p.x_m, s.y_m - p.y_m);
    if (distance < best) { best = distance; segment = i; projected = p; }
  }
  if (!std::isfinite(best)) return {};
  if (segment + 2 == path.size() &&
      std::hypot(path.back().x_m - s.x_m, path.back().y_m - s.y_m) <= config_.goal_tolerance_m)
    return {0.0, true, true};
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
