#include "control_manager/core/types.hxx"
#include <cmath>

namespace control_manager::core {
bool fresh(std::int64_t stamp, std::int64_t now, double timeout_s) {
  return stamp > 0 && now >= stamp && std::isfinite(timeout_s) && timeout_s > 0.0 &&
         static_cast<double>(now - stamp) / 1e9 <= timeout_s;
}
Command stop(std::int64_t now, const std::string &reason, bool valid) {
  return {0.0, 0.0, true, valid, now, reason};
}
Command selectCommand(ControlMode mode, const Command &manual, const Command &automatic,
                      std::int64_t now) {
  if (mode == ControlMode::MANUAL) return manual;
  if (mode == ControlMode::AUTO) return automatic;
  return stop(now, "IDLE", true);
}
Command protectActuatorCommand(const Command &c, bool estop,
                               std::int64_t now, const ActuatorProtectionConfig &cfg) {
  if (estop) return stop(now, "E_STOP");
  if (!c.valid || !fresh(c.timestamp_ns, now, cfg.command_timeout_s))
    return stop(now, "INVALID_OR_STALE_COMMAND");
  if (!std::isfinite(c.target_steer_rad) || !std::isfinite(c.target_speed_mps) ||
      c.target_speed_mps < 0.0 || (c.target_brake && c.target_speed_mps != 0.0))
    return stop(now, "INVALID_COMMAND_VALUES");
  if (!cfg.operating_limits_verified || !std::isfinite(cfg.max_steer_rad) ||
      !std::isfinite(cfg.max_speed_mps) || cfg.max_steer_rad <= 0.0 || cfg.max_speed_mps <= 0.0)
    return stop(now, "OPERATING_LIMITS_NOT_VERIFIED");
  if (std::abs(c.target_steer_rad) > cfg.max_steer_rad || c.target_speed_mps > cfg.max_speed_mps)
    return stop(now, "COMMAND_OUT_OF_RANGE");
  return c;  // Preserve source time; periodic forwarding must not renew a dead source.
}
VehicleFsmState fsmState(bool has_final, bool is_fresh, const Command &c, ControlMode mode) {
  if (!has_final) return VehicleFsmState::INIT;
  if (!is_fresh || !c.valid) return VehicleFsmState::FAULT;
  if (mode == ControlMode::IDLE) return VehicleFsmState::READY;
  return c.target_brake || c.target_speed_mps == 0.0 ? VehicleFsmState::STOPPED
                                                 : VehicleFsmState::DRIVING;
}
}  // namespace control_manager::core
