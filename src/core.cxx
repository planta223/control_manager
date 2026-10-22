#include "control_manager/core/types.hxx"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace control_manager::core {
bool fresh(std::int64_t stamp, std::int64_t now, double timeout_s) {
  return stamp > 0 && now >= stamp && std::isfinite(timeout_s) && timeout_s > 0.0 &&
         static_cast<double>(now - stamp) / 1e9 <= timeout_s;
}
Command stop(std::int64_t now, const std::string &reason, bool valid) {
  // previous factory result retained below for reference.
  // return {0.0, 0.0, true, valid, now, reason};
  // this neutral factory must pass through each node's StopSteeringHold.
  return {0.0, 0.0, true, valid, now, reason};
}
Command StopSteeringHold::apply(Command command) {
  // remember only finite, valid, non-braking output.
  if (command.valid && !command.target_brake && std::isfinite(command.target_steer_rad) &&
      std::isfinite(command.target_speed_mps)) {
    last_steer_rad_ = command.target_steer_rad;
  } else {
    command.target_steer_rad = last_steer_rad_;
  }
  return command;
}
Command selectCommand(ControlMode mode, const Command &manual, const Command &automatic,
                      std::int64_t now) {
  if (mode == ControlMode::MANUAL) return manual;
  if (mode == ControlMode::AUTO) return automatic;
  return stop(now, "IDLE", true);
}
AccelerationLimiter::AccelerationLimiter(double max_accel_mps2, double max_decel_mps2)
    : max_accel_mps2_(max_accel_mps2), max_decel_mps2_(max_decel_mps2) {
  if (!std::isfinite(max_accel_mps2_) || max_accel_mps2_ < 0.0 ||
      !std::isfinite(max_decel_mps2_) || max_decel_mps2_ < 0.0)
    throw std::invalid_argument("acceleration/deceleration limits must be finite and nonnegative");
}
Command AccelerationLimiter::apply(Command command, double dt_s) {
  if (max_accel_mps2_ == 0.0 && max_decel_mps2_ == 0.0) { reset(); return command; }
  if (!command.valid || command.target_brake || command.target_speed_mps == 0.0) {
    reset(); return command;  // Braking and invalid input must never be ramped.
  }
  if (!std::isfinite(command.target_speed_mps) || command.target_speed_mps < 0.0 ||
      !std::isfinite(dt_s) || dt_s <= 0.0) {
    reset(); return stop(command.timestamp_ns, "INVALID_ACCELERATION_LIMIT_INPUT");
  }
  // Do not catch up a delayed timer with a large speed step (normal tick: 20 ms).
  double next=command.target_speed_mps;
  if (max_accel_mps2_>0.0 && next>previous_speed_mps_)
    next=std::min(next,previous_speed_mps_+max_accel_mps2_*std::min(dt_s,0.1));
  if (max_decel_mps2_>0.0 && next<previous_speed_mps_)
    next=std::max(next,previous_speed_mps_-max_decel_mps2_*std::min(dt_s,0.1));
  if (next < command.target_speed_mps) command.reason += ":ACCELERATION_LIMITED";
  if (next > command.target_speed_mps) command.reason += ":DECELERATION_LIMITED";
  command.target_speed_mps = previous_speed_mps_ = next;
  return command;  // Preserve the source timestamp and steering/brake contract.
}
bool validVehicleLimits(const VehicleLimits &l) {
  return std::isfinite(l.min_steer_rad) && std::isfinite(l.max_steer_rad) &&
         std::isfinite(l.min_speed_mps) && std::isfinite(l.max_speed_mps) &&
         l.min_steer_rad < 0.0 && l.max_steer_rad > 0.0 &&
         l.min_speed_mps <= 0.0 && l.max_speed_mps > 0.0;
}
Command limitAutomaticCommand(const Command &c, const VehicleLimits &l) {
  if (!c.valid) return c;
  if (!validVehicleLimits(l)) return stop(c.timestamp_ns, "INVALID_VEHICLE_LIMITS");
  if (!std::isfinite(c.target_steer_rad) || !std::isfinite(c.target_speed_mps) ||
      c.target_speed_mps < 0.0 || (c.target_brake && c.target_speed_mps != 0.0))
    return stop(c.timestamp_ns, "INVALID_COMMAND_VALUES");
  auto out = c;
  out.target_steer_rad = std::clamp(c.target_steer_rad, l.min_steer_rad, l.max_steer_rad);
  out.target_speed_mps = std::min(c.target_speed_mps, l.max_speed_mps);
  if (out.target_steer_rad != c.target_steer_rad || out.target_speed_mps != c.target_speed_mps)
    out.reason += "_LIMITED";
  return out;
}
Command protectActuatorCommand(const Command &c,
                               std::int64_t now, const ActuatorProtectionConfig &cfg) {
  if (!c.valid || !fresh(c.timestamp_ns, now, cfg.command_timeout_s))
    return stop(now, "INVALID_OR_STALE_COMMAND");
  if (!std::isfinite(c.target_steer_rad) || !std::isfinite(c.target_speed_mps) ||
      (c.target_brake && c.target_speed_mps != 0.0))
    return stop(now, "INVALID_COMMAND_VALUES");
  if (!cfg.operating_limits_verified || !validVehicleLimits(cfg.limits))
    return stop(now, "OPERATING_LIMITS_NOT_VERIFIED");
  if (c.target_steer_rad < cfg.limits.min_steer_rad || c.target_steer_rad > cfg.limits.max_steer_rad ||
      c.target_speed_mps < cfg.limits.min_speed_mps || c.target_speed_mps > cfg.limits.max_speed_mps)
    return stop(now, "COMMAND_OUT_OF_RANGE");
  return c;  // Preserve source time; periodic forwarding must not renew a dead source.
}
bool validActuatorConversion(const ActuatorConversionConfig &cfg) {
  for (const double value : {cfg.wheel_diameter_m, cfg.motor_to_wheel_ratio,
                             cfg.steering_full_scale_deg, cfg.motor_full_scale_rpm})
    if (!std::isfinite(value) || value <= 0.0) return false;
  return true;
}
std::array<float, 3> actuatorRaw(const Command &c, const ActuatorConversionConfig &cfg) {
  // previous brake conversion forced steering to zero.
  // if (!c.valid || c.target_brake || !validActuatorConversion(cfg) ||
  //     !std::isfinite(c.target_steer_rad) || !std::isfinite(c.target_speed_mps))
  //   return {0.0F, 0.0F, 1.0F};
  // preserve the angle resolved by StopSteeringHold during braking.
  if (!validActuatorConversion(cfg) || !std::isfinite(c.target_steer_rad))
    return {0.0F, 0.0F, 1.0F};
  const double pi = std::acos(-1.0);
  const double steering = -c.target_steer_rad * 180.0 / pi / cfg.steering_full_scale_deg;
  const auto steering_raw = static_cast<float>(std::clamp(steering, -1.0, 1.0));
  if (!c.valid || c.target_brake || !std::isfinite(c.target_speed_mps))
    return {steering_raw, 0.0F, 1.0F};
  const double motor_rpm = c.target_speed_mps * 60.0 / (pi * cfg.wheel_diameter_m) *
                           cfg.motor_to_wheel_ratio;
  const double speed = motor_rpm / cfg.motor_full_scale_rpm;
  // if (!std::isfinite(steering) || !std::isfinite(speed)) return {0.0F, 0.0F, 1.0F};
  // conversion overflow also brakes without recentering the held angle.
  if (!std::isfinite(steering) || !std::isfinite(speed)) return {steering_raw, 0.0F, 1.0F};
  return {static_cast<float>(std::clamp(steering, -1.0, 1.0)),
          static_cast<float>(std::clamp(speed, -1.0, 1.0)), 0.0F};
}
VehicleFsmState fsmState(bool has_final, bool is_fresh, const Command &c, ControlMode mode) {
  if (!has_final) return VehicleFsmState::INIT;
  if (!is_fresh || !c.valid) return VehicleFsmState::FAULT;
  if (mode == ControlMode::IDLE) return VehicleFsmState::READY;
  return c.target_brake || c.target_speed_mps == 0.0 ? VehicleFsmState::STOPPED
                                                 : VehicleFsmState::DRIVING;
}
}  // namespace control_manager::core
