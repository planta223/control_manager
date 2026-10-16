#include "control_manager/core/mission.hxx"
#include <algorithm>
#include <cmath>
#include <limits>

namespace control_manager::core {
MissionDecision evaluateMission(const GlobalPose &pose, const std::vector<Point> &path,
  const std::vector<double> &stations, const std::vector<MissionSection> &sections,
  const MissionConfig &config) {
  auto invalid = [](const std::string &reason) { MissionDecision d; d.reason = reason; return d; };
  if (!std::isfinite(config.default_speed_mps) || config.default_speed_mps < 0.0 ||
      !std::isfinite(config.max_path_distance_m) || config.max_path_distance_m <= 0.0 ||
      !std::isfinite(config.association_tolerance_m) || config.association_tolerance_m < 0.0 ||
      !std::isfinite(config.position_sigma_multiplier) || config.position_sigma_multiplier <= 0.0)
    return invalid("INVALID_MISSION_CONFIG");
  if (!pose.valid || !std::isfinite(pose.x_m) || !std::isfinite(pose.y_m) ||
      !std::isfinite(pose.position_variance_x_m2) || pose.position_variance_x_m2 < 0.0 ||
      !std::isfinite(pose.position_variance_y_m2) || pose.position_variance_y_m2 < 0.0)
    return invalid("MISSION_POSE_UNAVAILABLE");
  if (path.size() < 2 || path.size() != stations.size()) return invalid("INVALID_MISSION_MAP");
  for (std::size_t i = 0; i < path.size(); ++i) {
    if (!std::isfinite(path[i].x_m) || !std::isfinite(path[i].y_m) ||
        !std::isfinite(stations[i]) || stations[i] < 0.0 ||
        (i && stations[i] <= stations[i - 1])) return invalid("INVALID_MISSION_MAP");
  }
  for (const auto &s : sections) {
    if (s.name.empty() || !std::isfinite(s.start_s_m) || !std::isfinite(s.end_s_m) ||
        s.start_s_m < stations.front() || s.end_s_m > stations.back() || s.end_s_m <= s.start_s_m ||
        !std::isfinite(s.max_speed_mps) || s.max_speed_mps < 0.0 ||
        !std::isfinite(s.margin_before_m) || s.margin_before_m < 0.0 ||
        !std::isfinite(s.margin_after_m) || s.margin_after_m < 0.0)
      return invalid("INVALID_MISSION_SECTION");
  }
  struct Projection { double distance, station; bool usable; };
  std::vector<Projection> projections;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double dx = path[i + 1].x_m - path[i].x_m, dy = path[i + 1].y_m - path[i].y_m;
    const double length2 = dx * dx + dy * dy;
    if (!std::isfinite(length2)) return invalid("INVALID_MISSION_MAP");
    if (length2 <= std::numeric_limits<double>::epsilon()) {
      projections.push_back({0.0, 0.0, false}); continue;
    }
    const double t = std::clamp(((pose.x_m - path[i].x_m) * dx +
                                (pose.y_m - path[i].y_m) * dy) / length2, 0.0, 1.0);
    const double distance = std::hypot(pose.x_m - path[i].x_m - t * dx,
                                       pose.y_m - path[i].y_m - t * dy);
    projections.push_back({distance, stations[i] + t * (stations[i + 1] - stations[i]), true});
    best = std::min(best, distance);
  }
  if (!std::isfinite(best)) return invalid("INVALID_MISSION_MAP");
  if (best > config.max_path_distance_m) return invalid("MISSION_OFF_PATH");
  // Trace-based radius is conservative for unknown XY covariance correlation.
  const double radius = config.position_sigma_multiplier *
    std::sqrt(pose.position_variance_x_m2 + pose.position_variance_y_m2);
  if (!std::isfinite(radius)) return invalid("MISSION_POSE_UNAVAILABLE");
  double low = std::numeric_limits<double>::infinity(), high = -low;
  int clusters = 0;
  bool in_cluster = false;
  for (const auto &p : projections) {
    if (!p.usable) continue;
    const bool candidate = p.distance <= best + radius + config.association_tolerance_m;
    if (candidate) {
      if (!in_cluster) ++clusters;
      low = std::min(low, p.station - radius);
      high = std::max(high, p.station + radius);
    }
    in_cluster = candidate;
  }
  // Separate candidate groups along the route indicate a crossing/nearby return leg.
  // Never select one by array order or silently reuse a previous valid limit.
  if (clusters != 1) return invalid("MISSION_SECTION_UNCERTAIN");
  MissionDecision decision;
  decision.max_speed_mps = config.default_speed_mps;
  decision.stop_required = false;
  decision.valid = true;
  decision.reason = "MISSION_DEFAULT_SPEED";
  for (const auto &s : sections) {
    if (high >= s.start_s_m - s.margin_before_m && low <= s.end_s_m + s.margin_after_m) {
      decision.max_speed_mps = std::min(decision.max_speed_mps, s.max_speed_mps);
      if (!decision.stop_required) decision.reason = "MISSION_SECTION:" + s.name;
    }
    // Margins widen the speed policy; stops apply to the original section only.
    if (s.stop_required && high >= s.start_s_m && low <= s.end_s_m) {
      decision.stop_required = true;
      decision.reason = "MISSION_SECTION_STOP:" + s.name;
    }
  }
  if (decision.stop_required) decision.max_speed_mps = 0.0;
  return decision;
}
}  // namespace control_manager::core
