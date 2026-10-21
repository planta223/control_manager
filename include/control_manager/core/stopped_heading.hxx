#pragma once
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>

namespace control_manager::core {
struct StoppedHeadingConfig {
  bool enabled{true};
  double stop_speed_mps{0.1}, reacquire_speed_mps{0.7};
  double max_distance_m{1.0}, max_reacquire_time_s{3.0};
};
struct StoppedHeadingResult {
  std::optional<double> yaw_rad;
  std::string reason;
};
// Control-local memory, never a new GNSS observation or a replacement timestamp.
class StoppedHeadingMemory {
 public:
  explicit StoppedHeadingMemory(StoppedHeadingConfig config) : config_(config) {
    if (!std::isfinite(config.stop_speed_mps) || config.stop_speed_mps < 0.0 ||
        !std::isfinite(config.reacquire_speed_mps) || config.reacquire_speed_mps <= config.stop_speed_mps ||
        !std::isfinite(config.max_distance_m) || config.max_distance_m <= 0.0 ||
        !std::isfinite(config.max_reacquire_time_s) || config.max_reacquire_time_s <= 0.0)
      throw std::invalid_argument("invalid stopped_heading configuration");
  }
  StoppedHeadingResult update(double now_s, bool observations_valid, bool intentional_stop,
                             double x, double y, double speed, std::optional<double> observed_yaw) {
    if (!config_.enabled || !observations_valid || !std::isfinite(now_s) ||
        !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(speed) || speed < 0.0 ||
        (observed_yaw && !std::isfinite(*observed_yaw))) {
      reset();
      return {};
    }
    if (observed_yaw) {
      last_yaw_ = observed_yaw;
      failure_.clear();
      if (intentional_stop) stop_armed_ = true;
      // Below the COG threshold a valid pose may still carry a decaying held yaw.
      if (!intentional_stop && speed >= config_.reacquire_speed_mps) {
        held_yaw_.reset(); moving_since_.reset(); stop_armed_ = false;
      }
    }
    if (!intentional_stop && !held_yaw_) stop_armed_ = false;
    if (intentional_stop && stop_armed_ && last_yaw_ &&
        speed <= config_.stop_speed_mps && !held_yaw_) {
      held_yaw_ = last_yaw_;
      anchor_x_ = x; anchor_y_ = y;
      moving_since_.reset();
    }
    if (held_yaw_) {
      if (std::hypot(x - anchor_x_, y - anchor_y_) > config_.max_distance_m) {
        expire("HEADING_HOLD_DISTANCE_EXCEEDED");
      } else {
        if (speed > config_.stop_speed_mps && !moving_since_) moving_since_ = now_s;
        if (moving_since_ && now_s - *moving_since_ >= config_.max_reacquire_time_s)
          expire("HEADING_HOLD_REACQUIRE_TIMEOUT");
      }
    }
    // A currently valid observation always takes precedence over the remembered yaw.
    if (observed_yaw) return {};
    return {held_yaw_, failure_};
  }
 private:
  void reset() {
    last_yaw_.reset(); held_yaw_.reset(); moving_since_.reset();
    stop_armed_ = false; failure_.clear();
  }
  void expire(const std::string &reason) {
    held_yaw_.reset(); moving_since_.reset(); stop_armed_ = false;
    failure_ = reason;
  }
  StoppedHeadingConfig config_;
  std::optional<double> last_yaw_, held_yaw_, moving_since_;
  bool stop_armed_{false};
  double anchor_x_{0.0}, anchor_y_{0.0};
  std::string failure_;
};
}  // namespace control_manager::core
