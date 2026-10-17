#include "control_manager/core/tracking.hxx"
#include <algorithm>
#include <cmath>
#include <limits>

namespace control_manager::core {
double distanceToPath(double x, double y, const std::vector<Point> &path) {
  double best = std::numeric_limits<double>::infinity();
  if (!std::isfinite(x) || !std::isfinite(y) || path.size() < 2) return best;
  for (const auto &p : path)
    if (!std::isfinite(p.x_m) || !std::isfinite(p.y_m)) return best;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double dx = path[i+1].x_m - path[i].x_m, dy = path[i+1].y_m - path[i].y_m;
    const double length2 = dx*dx + dy*dy;
    if (!std::isfinite(length2)) return std::numeric_limits<double>::infinity();
    if (length2 <= std::numeric_limits<double>::epsilon()) continue;
    const double dot = (x - path[i].x_m)*dx + (y - path[i].y_m)*dy;
    if (!std::isfinite(dot)) return std::numeric_limits<double>::infinity();
    const double t = std::clamp(dot / length2, 0.0, 1.0);
    best = std::min(best, std::hypot(path[i].x_m + t*dx - x, path[i].y_m + t*dy - y));
  }
  return best;
}
PathError pathError(const TrackingState &s, const std::vector<Point> &path, const PathMatch *match) {
  if (!s.localization_valid || !s.speed_valid || path.size() < 2 ||
      !std::isfinite(s.x_m) || !std::isfinite(s.y_m) || !std::isfinite(s.yaw_rad) ||
      !std::isfinite(s.speed_mps) || s.speed_mps < 0.0) return {};
  if (match && (!match->valid || match->segment>=path.size()-1 ||
      !std::isfinite(match->fraction) || match->fraction<0 || match->fraction>1)) return {};
  for (const auto &p : path) {
    const double dx = p.x_m - s.x_m, dy = p.y_m - s.y_m;
    if (!std::isfinite(p.x_m) || !std::isfinite(p.y_m) ||
        !std::isfinite(dx*dx + dy*dy)) return {};
  }
  PathError out;
  double best = std::numeric_limits<double>::infinity();
  double total_length=0.0, matched_station=0.0, final_tx=0.0, final_ty=0.0;
  std::size_t best_segment = 0, last_segment = 0;
  const double yaw = std::remainder(s.yaw_rad, 2.0 * std::acos(-1.0));
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double dx = path[i+1].x_m - path[i].x_m, dy = path[i+1].y_m - path[i].y_m;
    const double length2 = dx*dx + dy*dy;
    if (!std::isfinite(length2)) return {};  // Includes overflow from finite inputs.
    if (length2 <= std::numeric_limits<double>::epsilon()) continue;
    last_segment = i;
    const double length = std::sqrt(length2), tx = dx/length, ty = dy/length;
    const double segment_start=total_length;
    total_length+=length; final_tx=tx; final_ty=ty;
    if (!std::isfinite(total_length)) return {};
    if (match && i!=match->segment) continue;
    const double vx = s.x_m - path[i].x_m, vy = s.y_m - path[i].y_m;
    const double along = vx*tx + vy*ty;
    if (!std::isfinite(along)) return {};
    const double projection = match ? match->fraction*length : std::clamp(along, 0.0, length);
    const double ex = path[i].x_m + projection*tx - s.x_m;
    const double ey = path[i].y_m + projection*ty - s.y_m;
    const double distance = std::hypot(ex, ey);
    if (!std::isfinite(distance)) return {};
    if (distance < best) {
      best = distance; best_segment = i;
      matched_station=segment_start+projection;
      out.match={true,i,projection/length,distance};
      out.lateral_error_m = tx*ey - ty*ex;
      const double angle = std::atan2(dy, dx) - yaw;
      out.heading_error_rad = std::atan2(std::sin(angle), std::cos(angle));
    }
  }
  out.goal_distance_m = std::hypot(path.back().x_m - s.x_m, path.back().y_m - s.y_m);
  out.remaining_path_m=std::max(0.0,total_length-matched_station);
  const double goal_dx=path.back().x_m-s.x_m, goal_dy=path.back().y_m-s.y_m;
  out.goal_forward_m=goal_dx*final_tx+goal_dy*final_ty;
  out.goal_lateral_m=-goal_dx*final_ty+goal_dy*final_tx;
  out.final_segment = best_segment == last_segment;
  out.valid = std::isfinite(best) && std::isfinite(out.lateral_error_m) &&
    std::isfinite(out.heading_error_rad) && std::isfinite(out.goal_distance_m) &&
    std::isfinite(out.goal_forward_m) && std::isfinite(out.goal_lateral_m) &&
    std::isfinite(out.remaining_path_m);
  return out;
}
bool goalReached(const PathError &error, double tolerance) {
  return error.valid && std::isfinite(tolerance) && tolerance>=0.0 &&
    error.remaining_path_m<=tolerance && error.goal_distance_m<=tolerance;
}
}  // namespace control_manager::core
