#pragma once
#include "control_manager/core/types.hxx"

namespace control_manager::core {
struct LateralResult {
  double steering_rad{0.0};
  bool valid{false}, goal_reached{false};
};
class LateralController {
 public:
  virtual ~LateralController() = default;
  virtual LateralResult compute(const VehicleState &, const std::vector<Point> &) const = 0;
};
struct PurePursuitConfig {
  double wheelbase_m{1.328};
  double lookahead_min_m{2.0}, lookahead_max_m{6.0}, lookahead_gain_s{1.0};
  double goal_tolerance_m{0.3};
};
// Forward-only baseline. No actuator calibration, rate limiter, or loop progress tracking.
class AdaptivePurePursuit final : public LateralController {
 public:
  explicit AdaptivePurePursuit(PurePursuitConfig config);
  LateralResult compute(const VehicleState &, const std::vector<Point> &) const override;
 private:
  PurePursuitConfig config_;
};
}  // namespace control_manager::core
