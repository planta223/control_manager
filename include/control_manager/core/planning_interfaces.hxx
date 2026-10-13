#pragma once
#include <optional>
#include "control_manager/core/types.hxx"

namespace control_manager::core {
// TODO: select map format, validate ENU datum and rear-axle reference before loading.
class MapLoader {
 public:
  virtual ~MapLoader() = default;
  virtual std::optional<std::vector<Point>> load(const std::string &file) const = 0;
};
// TODO: define confidence, sign conventions and bounded correction before enabling.
class LaneCorrection {
 public:
  virtual ~LaneCorrection() = default;
  virtual std::vector<Point> correct(const std::vector<Point> &, const LaneState &) const = 0;
};
}  // namespace control_manager::core
