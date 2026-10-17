#include "control_manager/core/route_progress.hxx"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace control_manager::core {
RouteProgressTracker::RouteProgressTracker(RouteProgressConfig config) : config_(config) {
  if (config.expected_points < 4 || config.phase_end_indices[0] == 0 ||
      config.phase_end_indices[0] >= config.phase_end_indices[1] ||
      config.phase_end_indices[1] >= config.expected_points-1 ||
      !std::isfinite(config.initial_search_m) || config.initial_search_m <= 0 ||
      !std::isfinite(config.search_back_m) || config.search_back_m <= 0 ||
      !std::isfinite(config.search_forward_m) || config.search_forward_m <= 0)
    throw std::invalid_argument("invalid route progress profile/search bounds");
}
RouteProgressResult RouteProgressTracker::update(double x, double y, std::optional<double> yaw,
                                                const std::vector<Point> &path, double max_distance) {
  RouteProgressResult out; out.progress_m = progress_m_; out.phase = phase_;
  const auto fail = [&out](const char *reason) { out.match.valid = false; out.reason = reason; return out; };
  if (!std::isfinite(x) || !std::isfinite(y) || (yaw && !std::isfinite(*yaw)) ||
      !std::isfinite(max_distance) || max_distance <= 0)
    return fail("INVALID_ROUTE_PROGRESS_INPUT");
  if (path.size() != config_.expected_points) return fail("ROUTE_PROFILE_POINT_COUNT_MISMATCH");
  for (const auto &p : path)
    if (!std::isfinite(p.x_m) || !std::isfinite(p.y_m)) return fail("INVALID_TRACKING_GEOMETRY");
  if (!path_.empty() && !std::equal(path.begin(),path.end(),path_.begin(),
      [](const Point &a,const Point &b) { return a.x_m==b.x_m && a.y_m==b.y_m; }))
    return fail("ROUTE_CHANGED_RESTART_REQUIRED");
  if (path_.empty()) {
    std::vector<double> stations(path.size(),0.0);
    for (std::size_t i=1;i<path.size();++i) {
      const double length = std::hypot(path[i].x_m-path[i-1].x_m,path[i].y_m-path[i-1].y_m);
      stations[i]=stations[i-1]+length;
      if (!std::isfinite(stations[i])) return fail("INVALID_TRACKING_GEOMETRY");
    }
    if (stations[config_.phase_end_indices[0]] <= 0 ||
        stations[config_.phase_end_indices[1]] <= stations[config_.phase_end_indices[0]] ||
        stations.back() <= stations[config_.phase_end_indices[1]])
      return fail("INVALID_ROUTE_PHASE_GEOMETRY");
    stations_=std::move(stations); path_=path;
  }
  const double phase_end = phase_<2 ? stations_[config_.phase_end_indices[phase_]] : stations_.back();
  const double low = initialized_ ? std::max(0.0,progress_m_-config_.search_back_m) : 0.0;
  const double high = initialized_
    ? std::min({stations_.back(),progress_m_+config_.search_forward_m,phase_end+config_.search_back_m})
    : std::min(stations_[config_.phase_end_indices[0]],config_.initial_search_m);
  double best=std::numeric_limits<double>::infinity(), selected_s=0;
  bool heading_rejected=false;
  const auto begin=std::upper_bound(stations_.begin(),stations_.end(),low);
  const std::size_t first=begin==stations_.begin() ? 0 : static_cast<std::size_t>(begin-stations_.begin()-1);
  for (std::size_t i=first;i+1<path.size() && stations_[i]<=high;++i) {
    const double length=stations_[i+1]-stations_[i];
    if (length<=1e-9 || stations_[i+1]<low) continue;
    const double dx=path[i+1].x_m-path[i].x_m, dy=path[i+1].y_m-path[i].y_m;
    const double t0=std::clamp((low-stations_[i])/length,0.0,1.0);
    const double t1=std::clamp((high-stations_[i])/length,0.0,1.0);
    const double t=std::clamp(((x-path[i].x_m)*dx+(y-path[i].y_m)*dy)/(length*length),t0,t1);
    const double distance=std::hypot(path[i].x_m+t*dx-x,path[i].y_m+t*dy-y);
    // Forward-only tracking: do not associate an opposite-facing branch.
    if (yaw && dx*std::cos(*yaw)+dy*std::sin(*yaw)<=0) {
      if (distance<=max_distance) heading_rejected=true;
      continue;
    }
    if (distance<best) {
      best=distance; selected_s=stations_[i]+t*length;
      out.match={true,i,t,distance};
    }
  }
  if (!out.match.valid || best>max_distance)
    return fail(heading_rejected ? "ROUTE_HEADING_MISMATCH" : "PATH_DEVIATION_EXCEEDED");
  progress_m_=std::max(progress_m_,selected_s); initialized_=true;
  while (phase_<2 && progress_m_>=stations_[config_.phase_end_indices[phase_]]) ++phase_;
  out.progress_m=progress_m_; out.phase=phase_; out.reason="ROUTE_PROGRESS_VALID";
  return out;
}
}  // namespace control_manager::core
