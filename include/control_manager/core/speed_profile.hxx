#pragma once
#include "control_manager/core/types.hxx"
#include <optional>

namespace control_manager::core {
struct SpeedProfileConfig {
  double max_speed_mps, max_lateral_accel_mps2, planned_decel_mps2;
  double wheelbase_m, max_steering_rad;
  double curvature_sample_m{1.0}, response_time_s{0.5}, stop_before_end_m{0.3};
  double max_accel_mps2{1.0};
};
struct SpeedProfileSample {
  bool valid{false};
  double target_speed_mps{0.0}, station_m{0.0};
  double speed_limit_mps{0.0};
  bool measured_overspeed{false};
  std::string reason;
};
// Static open route: curvature caps, backwards braking and forwards acceleration.
// Time-based reference planning and the final selector also enforce command rates.
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

// Feedforward reference continuity, initialized from measured speed on start/reset.
// Measurement lag must not restart the ramp every tick; this is not a speed PID.
class SpeedReferencePlanner {
 public:
  SpeedReferencePlanner(double max_accel_mps2, double max_decel_mps2, double max_speed_mps);
  SpeedProfileSample update(SpeedProfileSample sample, double measured_speed_mps, double dt_s);
  void reset() { previous_speed_.reset(); }
 private:
  double max_accel_mps2_, max_decel_mps2_, max_speed_mps_;
  std::optional<double> previous_speed_;
};
}  // namespace control_manager::core
