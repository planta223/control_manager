#pragma once
#include <cmath>
#include <cstdint>
#include <string>
#include "control_manager/core/types.hxx"
#include "control_manager/msg/lane_pose.hpp"

namespace control_manager::ingress {
inline bool validStampFormat(const builtin_interfaces::msg::Time &stamp) {
  return stamp.sec >= 0 && stamp.nanosec < 1000000000U;
}
// Check wire timestamps without constructing rclcpp::Time from malformed input.
// No clock conversion or replacement with receipt time is performed here.
inline bool freshStamp(const builtin_interfaces::msg::Time &stamp, std::int64_t now_ns,
                       double timeout_s) {
  return validStampFormat(stamp) &&
    core::fresh(static_cast<std::int64_t>(stamp.sec)*1000000000LL + stamp.nanosec,
                now_ns, timeout_s);
}
inline bool admissibleLane(const msg::LanePose &sample, std::int64_t now_ns, double timeout_s) {
  return freshStamp(sample.header.stamp, now_ns, timeout_s) &&
    std::isfinite(sample.lateral_error_m) && std::isfinite(sample.heading_error_rad) &&
    std::isfinite(sample.confidence) && sample.confidence >= 0.0 && sample.confidence <= 1.0;
}
}  // namespace control_manager::ingress
