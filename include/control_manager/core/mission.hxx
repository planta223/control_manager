#pragma once
#include "control_manager/core/types.hxx"

namespace control_manager::core {
struct MissionSection {
  std::string name;
  double start_s_m, end_s_m, max_speed_mps, margin_before_m, margin_after_m;
  bool stop_required;
};
struct MissionConfig {
  double default_speed_mps, max_path_distance_m, association_tolerance_m, position_sigma_multiplier;
};
struct MissionDecision {
  double max_speed_mps{0.0};
  bool stop_required{true}, valid{false};
  std::string reason;
};
MissionDecision evaluateMission(const GlobalPose &pose, const std::vector<Point> &path,
  const std::vector<double> &stations, const std::vector<MissionSection> &sections,
  const MissionConfig &config);
}  // namespace control_manager::core
