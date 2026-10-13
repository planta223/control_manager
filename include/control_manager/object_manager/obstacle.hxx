#pragma once
#include <optional>
#include <sensor_msgs/msg/point_cloud2.hpp>
namespace control_manager::perception {
struct ObstacleObservation {
  std_msgs::msg::Header header;
  bool detected;
  double nearest_distance_m;
};
class ObstacleDetector {
 public:
  virtual ~ObstacleDetector() = default;
  // LiDAR PRIMARY. TODO: define distance from vehicle envelope, extrinsics, ROI,
  // freshness and confidence; optional depth validation is a future extension.
  virtual std::optional<ObstacleObservation> detect(const sensor_msgs::msg::PointCloud2 &) = 0;
};
}
