#pragma once
#include <array>
#include <optional>
#include "control_manager/core/types.hxx"

namespace control_manager::core {
struct RouteProgressConfig {
  std::size_t expected_points{0};
  std::array<std::size_t,2> phase_end_indices{};  // FRONT end / REAR start, zero-based points.
  double initial_search_m{10.0}, search_back_m{3.0}, search_forward_m{10.0};
};
struct RouteProgressResult {
  PathMatch match;
  double progress_m{0.0};
  std::size_t phase{0};  // FRONT / MIDDLE / REAR = 0 / 1 / 2.
  std::string reason;
};
// Keeps route association across invalid inputs and PID resets. Only construction
// starts a new run; a changed route requires an explicit node restart.
class RouteProgressTracker {
 public:
  explicit RouteProgressTracker(RouteProgressConfig config);
  RouteProgressResult update(double x, double y, std::optional<double> yaw,
                             const std::vector<Point> &path, double max_distance_m);
 private:
  RouteProgressConfig config_;
  std::vector<Point> path_;
  std::vector<double> stations_;
  bool initialized_{false};
  double progress_m_{0.0};
  std::size_t phase_{0};
};
}  // namespace control_manager::core
