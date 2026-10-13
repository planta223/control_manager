#pragma once
#include <optional>
#include <sensor_msgs/msg/image.hpp>
namespace control_manager::perception {
struct SpeedLimitObservation { std_msgs::msg::Header header; double speed_limit_mps; };
class TrafficSignDetector {
 public:
  virtual ~TrafficSignDetector() = default;
  virtual std::optional<SpeedLimitObservation> detect(const sensor_msgs::msg::Image &) = 0;
};
}
