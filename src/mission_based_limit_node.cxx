#include "control_manager/nodes.hxx"
#include "control_manager/ingress.hxx"
#include <algorithm>
#include <set>
namespace control_manager {
MissionBasedLimitNode::MissionBasedLimitNode() : NodeBase("mission_based_limit") {
  if (!enabled("mission_limit_enabled", true)) return;  // No input, output or timer.
  frame_ = param<std::string>("map_frame", "map");
  const double cruise = nonnegative("cruise_speed_mps", nonnegative("cruise_speed_kph", 0.0) / 3.6);
  max_speed_mps_ = nonnegative("mission_max_speed_mps", cruise);
  timeout_s_ = positive("object_timeout_s", 0.5);
  baseline_enabled_ = param<bool>("mission_baseline_enabled", false);
  sections_enabled_ = enabled("mission_sections_enabled", false);
  if (sections_enabled_) {
    pose_timeout_s_ = positive("state_timeout_s", 0.5);
    mission_config_ = {max_speed_mps_, positive("max_path_distance_m", 1.0),
      nonnegative("association_tolerance_m", 0.2), positive("position_sigma_multiplier", 2.0)};
    std::set<std::string> ids, names;
    for (const auto &id : param<std::vector<std::string>>("section_ids", {})) {
      if (id.empty() || !ids.insert(id).second)
        throw std::invalid_argument("section_ids must be unique and nonempty");
      const auto prefix = "sections." + id + ".";
      const auto name = param<std::string>(prefix + "map_section", "");
      if (name.empty() || !names.insert(name).second)
        throw std::invalid_argument("map_section must be unique and nonempty");
      section_policies_.push_back({name,
        nonnegative(prefix + "max_speed_kph", max_speed_mps_ * 3.6) / 3.6,
        nonnegative(prefix + "margin_before_m", 0.0), nonnegative(prefix + "margin_after_m", 0.0),
        param<bool>(prefix + "stop_required", false)});
    }
    pose_ = input<msg::GlobalPose>("global_pose");
    map_ = input<msg::MissionMap>("mission_map", rclcpp::QoS(1).transient_local());
  }
  traffic_light_applies_ = param<bool>("traffic_light_applies", false);
  if (enabled("object_state_enabled", true)) objects_ = input<msg::ObjectState>("object_state");
  if (traffic_light_applies_ && !objects_)
    throw std::invalid_argument("traffic_light_applies requires object_state_enabled=true");
  publisher_ = output<msg::MissionLimit>("mission_limit");
  tick([this] { update(); });
}
core::MissionDecision MissionBasedLimitNode::sectionLimit() {
  auto invalid = [](const std::string &reason) { core::MissionDecision d; d.reason = reason; return d; };
  if (!pose_->value || !ingress::validStampFormat(pose_->value->header.stamp) ||
      !current(pose_, pose_timeout_s_) || pose_->value->header.frame_id != frame_)
    return invalid("MISSION_POSE_UNAVAILABLE");
  if (!map_->value) return invalid("MISSION_MAP_UNAVAILABLE");
  const auto &map = *map_->value;
  if (map.header.frame_id != frame_ || !validPath(map.path, frame_) ||
      map.path.poses.size() != map.stations_m.size() ||
      map.section_names.size() != map.section_start_s_m.size() ||
      map.section_names.size() != map.section_end_s_m.size()) return invalid("INVALID_MISSION_MAP");
  std::set<std::string> names;
  for (std::size_t i = 0; i < map.section_names.size(); ++i) {
    if (map.section_names[i].empty() || !names.insert(map.section_names[i]).second ||
        !std::isfinite(map.section_start_s_m[i]) || !std::isfinite(map.section_end_s_m[i]) ||
        map.section_start_s_m[i] < map.stations_m.front() ||
        map.section_end_s_m[i] > map.stations_m.back() ||
        map.section_end_s_m[i] <= map.section_start_s_m[i]) return invalid("INVALID_MISSION_MAP");
  }
  std::vector<core::MissionSection> sections;
  for (const auto &policy : section_policies_) {
    const auto found = std::find(map.section_names.begin(), map.section_names.end(), policy.map_section);
    if (found == map.section_names.end()) return invalid("MISSION_SECTION_NOT_IN_MAP:" + policy.map_section);
    const auto i = std::distance(map.section_names.begin(), found);
    sections.push_back({policy.map_section, map.section_start_s_m[i], map.section_end_s_m[i],
      policy.max_speed_mps, policy.margin_before_m, policy.margin_after_m, policy.stop_required});
  }
  std::vector<core::Point> path;
  path.reserve(map.path.poses.size());
  for (const auto &p : map.path.poses) path.push_back({p.pose.position.x, p.pose.position.y});
  return core::evaluateMission(toCore(*pose_->value), path, map.stations_m, sections, mission_config_);
}
void MissionBasedLimitNode::update() {
  msg::MissionLimit limit;
  limit.header = header(frame_);
  limit.max_speed_mps = max_speed_mps_;
  limit.valid = baseline_enabled_;
  limit.stop_required = !baseline_enabled_;
  limit.reason = baseline_enabled_ ? "CONFIGURED_BASELINE" : "MISSION_POLICY_NOT_CONFIGURED";
  if (baseline_enabled_ && sections_enabled_) {
    const auto decision = sectionLimit();
    limit.max_speed_mps = decision.max_speed_mps;
    limit.valid = decision.valid;
    limit.stop_required = decision.stop_required;
    limit.reason = decision.reason;
  }
  const bool available = current(objects_, timeout_s_) && objects_->value->valid;
  if (available && objects_->value->speed_limit_valid) {
    const double speed = objects_->value->speed_limit_mps;
    if (!std::isfinite(speed) || speed < 0.0) {
      limit.valid = false; limit.stop_required = true; limit.reason = "INVALID_SPEED_LIMIT";
    } else {
      limit.max_speed_mps = std::min(limit.max_speed_mps, speed);
    }
  }
  if (traffic_light_applies_) {
    if (!available || !objects_->value->traffic_light_valid ||
        objects_->value->traffic_light_state == msg::ObjectState::UNKNOWN ||
        objects_->value->traffic_light_state > msg::ObjectState::GREEN) {
      limit.valid = false; limit.stop_required = true; limit.reason = "TRAFFIC_LIGHT_UNAVAILABLE";
    } else if (objects_->value->traffic_light_state != msg::ObjectState::GREEN) {
      limit.stop_required = true; limit.reason = "TRAFFIC_LIGHT_STOP";
    }
  }
  // Only Motion Planning consumes these restrictions. No sensor/mode switching.
  publisher_->publish(limit);
}
}  // namespace control_manager
