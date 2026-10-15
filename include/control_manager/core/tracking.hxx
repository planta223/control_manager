#pragma once
#include <string>
#include "control_manager/core/lateral_controller.hxx"

namespace control_manager::core {
enum class TrackingMode : std::uint8_t {
  PID = 0, PURE_PURSUIT = 1, ADAPTIVE_PURE_PURSUIT = 2, NONE = 255
};
struct TrackingResult {
  double steering_rad{0.0};
  bool valid{false}, goal_reached{false};
  double lateral_error_m{0.0}, heading_error_rad{0.0};
  std::string reason;
};
struct PidConfig {
  // Disabled placeholders. Gains and output bound require bench/vehicle tuning.
  bool enabled{false};
  double kp_lateral{0.0}, ki_lateral{0.0}, kd_lateral{0.0}, kp_heading{0.0};
  double integral_limit_m_s{0.0}, max_steering_rad{0.0};
  double max_dt_s{0.5}, goal_tolerance_m{0.3};
};
struct PathError {
  bool valid{false}, final_segment{false};
  double lateral_error_m{0.0}, heading_error_rad{0.0};
  double goal_distance_m{0.0}, goal_forward_m{0.0};
};
// Signed correction: path to the left of the vehicle => positive e_y.
// Tangent comes from the nearest non-degenerate segment in path order.
PathError pathError(const TrackingState &, const std::vector<Point> &);
class PidLateralController {
 public:
  explicit PidLateralController(PidConfig config);
  TrackingResult compute(const TrackingState &, const std::vector<Point> &, double dt_s);
  void reset();
 private:
  PidConfig config_;
  bool previous_valid_{false};
  double integral_m_s_{0.0}, previous_error_m_{0.0};
};
// ROS-free selector; leaves speed/brake, guidance and arbitration to their owners.
class TrackingController {
 public:
  TrackingController(PidConfig pid_config, PurePursuitConfig pp_config);
  void request(std::uint8_t mode);
  std::uint8_t requested() const { return requested_; }
  // Empty string means implemented selection (not input/config validity).
  std::string selectionError() const;
  TrackingResult compute(const TrackingState &, const std::vector<Point> &, double dt_s,
                         bool accumulate_pid = true);
  void reset();
 private:
  std::uint8_t requested_{static_cast<std::uint8_t>(TrackingMode::PID)};
  PidLateralController pid_;
  AdaptivePurePursuit legacy_pp_;  // Existing speed-adaptive algorithm, unchanged.
};
}  // namespace control_manager::core
