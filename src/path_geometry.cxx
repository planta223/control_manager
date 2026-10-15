#include "control_manager/core/tracking.hxx"
#include <algorithm>
#include <cmath>
#include <limits>

namespace control_manager::core {
PathError pathError(const TrackingState &s, const std::vector<Point> &path) {
  if (!s.localization_valid || !s.speed_valid || path.size() < 2 ||
      !std::isfinite(s.x_m) || !std::isfinite(s.y_m) || !std::isfinite(s.yaw_rad) ||
      !std::isfinite(s.speed_mps) || s.speed_mps < 0.0) return {};
  for (const auto &p : path) {
    const double dx = p.x_m - s.x_m, dy = p.y_m - s.y_m;
    if (!std::isfinite(p.x_m) || !std::isfinite(p.y_m) ||
        !std::isfinite(dx*dx + dy*dy)) return {};
  }
  PathError out;
  double best = std::numeric_limits<double>::infinity();
  std::size_t best_segment = 0, last_segment = 0;
  const double yaw = std::remainder(s.yaw_rad, 2.0 * std::acos(-1.0));
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double dx = path[i+1].x_m - path[i].x_m, dy = path[i+1].y_m - path[i].y_m;
    const double length2 = dx*dx + dy*dy;
    if (!std::isfinite(length2)) return {};  // Includes overflow from finite inputs.
    if (length2 <= std::numeric_limits<double>::epsilon()) continue;
    last_segment = i;
    const double length = std::sqrt(length2), tx = dx/length, ty = dy/length;
    const double vx = s.x_m - path[i].x_m, vy = s.y_m - path[i].y_m;
    const double along = vx*tx + vy*ty;
    if (!std::isfinite(along)) return {};
    const double projection = std::clamp(along, 0.0, length);
    const double ex = path[i].x_m + projection*tx - s.x_m;
    const double ey = path[i].y_m + projection*ty - s.y_m;
    const double distance = std::hypot(ex, ey);
    if (!std::isfinite(distance)) return {};
    if (distance < best) {
      best = distance; best_segment = i;
      out.lateral_error_m = tx*ey - ty*ex;
      const double angle = std::atan2(dy, dx) - yaw;
      out.heading_error_rad = std::atan2(std::sin(angle), std::cos(angle));
      out.goal_forward_m = (path.back().x_m - s.x_m)*tx + (path.back().y_m - s.y_m)*ty;
    }
  }
  out.goal_distance_m = std::hypot(path.back().x_m - s.x_m, path.back().y_m - s.y_m);
  out.final_segment = best_segment == last_segment;
  out.valid = std::isfinite(best) && std::isfinite(out.lateral_error_m) &&
    std::isfinite(out.heading_error_rad) && std::isfinite(out.goal_distance_m) &&
    std::isfinite(out.goal_forward_m);
  return out;
}
}  // namespace control_manager::core
