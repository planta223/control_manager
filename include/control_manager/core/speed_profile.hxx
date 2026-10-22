#pragma once
#include "control_manager/core/types.hxx"

namespace control_manager::core {
struct SpeedProfileConfig {
  double max_speed_mps, max_lateral_accel_mps2, planned_decel_mps2;
  double wheelbase_m, max_steering_rad;
  double curvature_sample_m{1.0}, response_time_s{0.5}, stop_before_end_m{0.3};
};
struct SpeedProfileSample {
  bool valid{false};
  double target_speed_mps{0.0}, station_m{0.0};
  std::string reason;
};
// Static open route. Curvature speed caps and backwards braking envelope;
// the selector applies time-based acceleration/deceleration command limits.
class PathSpeedProfile {
 public:
  explicit PathSpeedProfile(SpeedProfileConfig config);
  std::string setPath(const std::vector<Point> &path);
  SpeedProfileSample sample(const PathMatch &match, double measured_speed_mps) const;
 private:
  Point pointAt(double station) const;
  double speedAt(double station) const;
  SpeedProfileConfig config_;
  std::vector<Point> path_;
  std::vector<double> stations_, speeds_;
};
}  // namespace control_manager::core
