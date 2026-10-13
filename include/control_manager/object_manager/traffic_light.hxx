#pragma once
#include <optional>
#include <sensor_msgs/msg/image.hpp>
namespace control_manager::perception {
// No implementation yet. Input RGB; result time must be the sensor acquisition time.
struct TrafficLightObservation { std_msgs::msg::Header header; std::uint8_t state; };
class TrafficLightDetector {
 public:
  virtual ~TrafficLightDetector() = default;
  virtual std::optional<TrafficLightObservation> detect(const sensor_msgs::msg::Image &) = 0;
};
}
