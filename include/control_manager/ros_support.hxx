#pragma once
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "control_manager/core/types.hxx"
#include "control_manager/msg/global_pose.hpp"
#include "control_manager/msg/motion_state.hpp"
#include "control_manager/msg/lane_pose.hpp"
#include "control_manager/msg/object_state.hpp"
#include "control_manager/msg/mission_limit.hpp"
#include "control_manager/msg/mission_map.hpp"
#include "control_manager/msg/motion_plan.hpp"
#include "control_manager/msg/vehicle_command.hpp"
#include "control_manager/msg/control_status.hpp"
#include "control_manager/msg/pause_request.hpp"

namespace control_manager {
// Central defaults; all topic names may be overridden with topics.<key> parameters.
inline const std::map<std::string, std::string> kTopics{
  {"gnss", "/gnss/odom"}, {"imu", "/imu/data"},
  {"depth", "/camera_dep/depth/image_raw"},
  {"detect_cone", "/sensors/detectCone"}, {"detect_obstacle", "/sensors/detectObstacle"},
  {"detect_traffic", "/sensors/detectTraffic"},
  {"sensor_gnss", "/control/sensors/gnss_odom"}, {"sensor_imu", "/control/sensors/imu"},
  {"sensor_depth", "/control/sensors/depth"},
  {"sensor_cone", "/control/sensors/cone"}, {"sensor_obstacle", "/control/sensors/obstacle"},
  {"sensor_traffic", "/control/sensors/traffic"},
  {"external_lane", ""},  // No agreed external Lane topic; explicitly configure the adapter.
  {"external_motor_feedback", "/motor_feedback"},
  {"external_steering_feedback", "/steering_feedback"},
  {"sensor_motor_feedback", "/control/sensors/motor_feedback"},
  {"sensor_steering_feedback", "/control/sensors/steering_feedback"},
  {"lane_observation", "/control/lane_observation"},
  {"global_pose", "/control/global_pose"}, {"motion_state", "/control/motion_state"}, {"lane_pose", "/control/lane_pose"},
  {"object_state", "/control/object_state"}, {"global_path", "/control/global_path"},
  {"reference_path", "/control/reference_path"}, {"mission_limit", "/control/mission_limit"},
  {"mission_map", "/control/mission_map"},
  {"motion_plan", "/control/motion_plan"}, {"auto_cmd", "/control/auto_cmd"},
  {"manual_cmd", "/control/manual_cmd"}, {"selected_cmd", "/control/selected_cmd"},
  {"actuator_raw", "/control/actuator_raw"},
  {"status", "/control/status"},
  {"stop_status", "/control/stop_status"},  // 고규범 수정 (2026-10-21): measured state.
  {"pause_request", "/control/pause_request"},
};
template<class T> struct Input {
  typename T::ConstSharedPtr value;
  std::chrono::steady_clock::time_point received{};
};
class NodeBase : public rclcpp::Node {
 public:
  explicit NodeBase(const std::string &name) : Node(name) {}
 protected:
  template<class T> T param(const std::string &key, const T &value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(key, value, descriptor);
  }
  double positive(const std::string &key, double value) {
    value = param<double>(key, value);
    if (!std::isfinite(value) || value <= 0.0) throw std::invalid_argument(key + " must be positive");
    return value;
  }
  bool enabled(const std::string &key, bool default_value) {
    return has_parameter(key) ? get_parameter(key).as_bool() : param<bool>(key, default_value);
  }
  double nonnegative(const std::string &key, double value) {
    value = param<double>(key, value);
    if (!std::isfinite(value) || value < 0.0) throw std::invalid_argument(key + " must be nonnegative");
    return value;
  }
  core::VehicleLimits vehicleLimits() {
    const double rad_per_deg = std::acos(-1.0) / 180.0;
    core::VehicleLimits limits{
      param<double>("vehicle.limits.min_steering_deg", 0.0) * rad_per_deg,
      param<double>("vehicle.limits.max_steering_deg", 0.0) * rad_per_deg,
      param<double>("vehicle.limits.min_speed_kph", 0.0) / 3.6,
      param<double>("vehicle.limits.max_speed_kph", 0.0) / 3.6};
    if (!core::validVehicleLimits(limits))
      throw std::invalid_argument("vehicle.limits must be finite with min_steering_deg < 0 < max_steering_deg and min_speed_kph <= 0 < max_speed_kph; load vehicle.yaml");
    return limits;
  }
  std::string topic(const std::string &key) {
    const auto name = "topics." + key;
    return has_parameter(name) ? get_parameter(name).as_string() : param(name, kTopics.at(key));
  }
  std::string perceptionType(const std::string &key) {
    auto type = param<std::string>("perception_types." + key,
      key == "traffic" ? "control_manager/msg/TrafficLight" : "");
    if (type.empty()) throw std::invalid_argument(key + "_enabled requires nonempty perception_types." + key);
    return type;
  }
  template<class T> std::shared_ptr<Input<T>> input(const std::string &key,
                                                    rclcpp::QoS qos = rclcpp::QoS(10)) {
    auto result = std::make_shared<Input<T>>();
    subscriptions_.push_back(create_subscription<T>(topic(key), qos,
      [result](typename T::ConstSharedPtr value) {
        result->value = value;
        result->received = std::chrono::steady_clock::now();
      }));
    return result;
  }
  template<class T> typename rclcpp::Publisher<T>::SharedPtr output(
      const std::string &key, rclcpp::QoS qos = rclcpp::QoS(10)) {
    return create_publisher<T>(topic(key), qos);
  }
  template<class T> bool current(const std::shared_ptr<Input<T>> &in, double timeout) {
    return in && in->value &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - in->received).count() <= timeout &&
      core::fresh(rclcpp::Time(in->value->header.stamp).nanoseconds(), now().nanoseconds(), timeout);
  }
  void tick(const std::function<void()> &callback) {
    const auto hz = positive("rate_hz", 20.0);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / hz)), callback);
  }
  std_msgs::msg::Header header(const std::string &frame) {
    std_msgs::msg::Header h; h.stamp = now(); h.frame_id = frame; return h;
  }
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr timer_;
};
inline core::GlobalPose toCore(const msg::GlobalPose &s) {
  return {s.x_m, s.y_m, s.yaw_rad, s.position_variance_x_m2, s.position_variance_y_m2,
          s.yaw_variance_rad2, s.valid, rclcpp::Time(s.header.stamp).nanoseconds()};
}
inline core::MotionState toCore(const msg::MotionState &s) {
  return {s.speed_mps, s.yaw_rate_radps, s.speed_valid, s.yaw_rate_valid,
          rclcpp::Time(s.header.stamp).nanoseconds()};
}
inline core::TrackingState trackingInput(const msg::GlobalPose &p, const msg::MotionState &m) {
  return {p.x_m, p.y_m, p.yaw_rad, m.speed_mps, m.yaw_rate_radps,
          p.valid, m.speed_valid, m.yaw_rate_valid, rclcpp::Time(p.header.stamp).nanoseconds()};
}
inline core::Command toCore(const msg::VehicleCommand &c) {
  return {c.target_steer_rad, c.target_speed_mps, c.target_brake, c.valid,
          rclcpp::Time(c.header.stamp).nanoseconds(), c.reason};
}
inline msg::VehicleCommand toRos(const core::Command &c, const std::string &frame) {
  msg::VehicleCommand out;
  out.header.stamp = rclcpp::Time(c.timestamp_ns); out.header.frame_id = frame;
  out.target_steer_rad = c.target_steer_rad; out.target_speed_mps = c.target_speed_mps;
  out.target_brake = c.target_brake; out.valid = c.valid; out.reason = c.reason;
  return out;
}
inline bool validPath(const nav_msgs::msg::Path &path, const std::string &frame) {
  if (path.header.frame_id != frame || path.poses.size() < 2) return false;
  bool nondegenerate = false;
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    const auto &p = path.poses[i];
    if ((!p.header.frame_id.empty() && p.header.frame_id != frame) ||
        !std::isfinite(p.pose.position.x) || !std::isfinite(p.pose.position.y)) return false;
    if (i > 0 && std::hypot(p.pose.position.x - path.poses[i-1].pose.position.x,
                           p.pose.position.y - path.poses[i-1].pose.position.y) > 0.0)
      nondegenerate = true;
  }
  return nondegenerate;
}
}  // namespace control_manager
