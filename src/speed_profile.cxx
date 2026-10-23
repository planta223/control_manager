#include "control_manager/core/speed_profile.hxx"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace control_manager::core {
PathSpeedProfile::PathSpeedProfile(SpeedProfileConfig config) : config_(config) {
  for (double v : {config.max_speed_mps, config.max_lateral_accel_mps2,
      config.planned_decel_mps2, config.wheelbase_m, config.max_steering_rad,
      config.curvature_sample_m, config.max_accel_mps2})
    if (!std::isfinite(v) || v <= 0.0) throw std::invalid_argument("speed profile limits must be finite and positive");
  if (config.max_steering_rad >= std::acos(-1.0)/2 ||
      !std::isfinite(config.response_time_s) || config.response_time_s < 0.0 ||
      !std::isfinite(config.stop_before_end_m) || config.stop_before_end_m < 0.0)
    throw std::invalid_argument("invalid speed profile geometry/timing");
}
Point PathSpeedProfile::pointAt(double station) const {
  if (station >= stations_.back()) return path_.back();
  const auto next = std::upper_bound(stations_.begin(), stations_.end(), station);
  const auto i = static_cast<std::size_t>(next-stations_.begin()-1);
  const double t = (station-stations_[i])/(stations_[i+1]-stations_[i]);
  return {path_[i].x_m+t*(path_[i+1].x_m-path_[i].x_m),
          path_[i].y_m+t*(path_[i+1].y_m-path_[i].y_m)};
}
std::string PathSpeedProfile::setPath(const std::vector<Point> &path) {
  if (!path_.empty()) {
    if (path.size() == path_.size() && std::equal(path.begin(),path.end(),path_.begin(),
        [](const Point &a,const Point &b) { return a.x_m==b.x_m && a.y_m==b.y_m; })) return "";
    return "SPEED_PROFILE_ROUTE_CHANGED_RESTART_REQUIRED";
  }
  if (path.size()<2) return "SPEED_PROFILE_INVALID_PATH";
  std::vector<double> stations(path.size(),0.0);
  for (std::size_t i=0;i<path.size();++i) {
    if (!std::isfinite(path[i].x_m) || !std::isfinite(path[i].y_m)) return "SPEED_PROFILE_INVALID_PATH";
    if (i) stations[i]=stations[i-1]+std::hypot(path[i].x_m-path[i-1].x_m,path[i].y_m-path[i-1].y_m);
    if (!std::isfinite(stations[i])) return "SPEED_PROFILE_INVALID_PATH";
  }
  if (stations.back() <= config_.stop_before_end_m || stations.back() <= 1e-6)
    return "SPEED_PROFILE_PATH_TOO_SHORT";
  path_=path; stations_=std::move(stations);
  speeds_.assign(path.size(),config_.max_speed_mps);
  const double max_curvature=std::tan(config_.max_steering_rad)/config_.wheelbase_m;
  for (std::size_t i=0;i<path.size();++i) {
    const double left=std::max(0.0,stations_[i]-config_.curvature_sample_m);
    const double right=std::min(stations_.back(),stations_[i]+config_.curvature_sample_m);
    const double middle=std::clamp(stations_[i],left+(right-left)*0.001,right-(right-left)*0.001);
    // At either endpoint use a balanced triangle rather than a zero-length side.
    const auto a=pointAt(left), b=pointAt(i==0 || i+1==path.size() ? (left+right)/2 : middle), c=pointAt(right);
    const double ab=std::hypot(b.x_m-a.x_m,b.y_m-a.y_m),bc=std::hypot(c.x_m-b.x_m,c.y_m-b.y_m),
                 ac=std::hypot(c.x_m-a.x_m,c.y_m-a.y_m);
    const double cross=(b.x_m-a.x_m)*(c.y_m-a.y_m)-(b.y_m-a.y_m)*(c.x_m-a.x_m);
    const double curvature=std::min({ab,bc,ac})>1e-9 ? 2*std::abs(cross)/(ab*bc*ac) : 0.0;
    if (!std::isfinite(curvature) || curvature>max_curvature+1e-6) {
      path_.clear(); stations_.clear(); speeds_.clear();
      return "SPEED_PROFILE_STEERING_GEOMETRY_EXCEEDED";
    }
    if (curvature>1e-9) speeds_[i]=std::min(speeds_[i],std::sqrt(config_.max_lateral_accel_mps2/curvature));
    if (stations_.back()-stations_[i] <= config_.stop_before_end_m) speeds_[i]=0.0;
  }
  const auto local_caps=speeds_;
  // v_before^2 <= v_after^2 + 2*b*distance. Future bends/end affect upstream speeds.
  for (std::size_t i=path.size()-1;i>0;--i)
    speeds_[i-1]=std::min(speeds_[i-1],std::sqrt(speeds_[i]*speeds_[i]+
        2*config_.planned_decel_mps2*(stations_[i]-stations_[i-1])));
  // For a lower future cap w at distance d, reserve response time t:
  // v*t + (v*v-w*w)/(2*b) <= d. This is a planning assumption, not measured braking.
  const double b=config_.planned_decel_mps2, bt=b*config_.response_time_s;
  const double horizon=config_.max_speed_mps*config_.response_time_s+
      config_.max_speed_mps*config_.max_speed_mps/(2*b);
  if (bt>0.0) for (std::size_t i=0;i<path.size();++i)
    for (std::size_t j=i+1;j<path.size() && stations_[j]-stations_[i]<=horizon;++j)
      if (local_caps[j]+1e-9<local_caps[i])
        // Already travelling at w needs no deceleration or response-distance reserve.
        speeds_[i]=std::min(speeds_[i],std::max(local_caps[j],std::sqrt(bt*bt+local_caps[j]*local_caps[j]+
            2*b*(stations_[j]-stations_[i]))-bt));
  // Re-enforce pairwise braking after response-distance reservations, then bound
  // acceleration out of every low-speed curve by the available path distance.
  for (std::size_t i=path.size()-1;i>0;--i)
    speeds_[i-1]=std::min(speeds_[i-1],std::sqrt(speeds_[i]*speeds_[i]+
        2*config_.planned_decel_mps2*(stations_[i]-stations_[i-1])));
  for (std::size_t i=1;i<path.size();++i)
    speeds_[i]=std::min(speeds_[i],std::sqrt(speeds_[i-1]*speeds_[i-1]+
        2*config_.max_accel_mps2*(stations_[i]-stations_[i-1])));
  return "";
}
double PathSpeedProfile::speedAt(double station) const {
  if (station >= stations_.back()) return speeds_.back();
  const auto next=std::upper_bound(stations_.begin(),stations_.end(),station);
  const auto i=static_cast<std::size_t>(next-stations_.begin()-1);
  const double t=(station-stations_[i])/(stations_[i+1]-stations_[i]);
  return std::sqrt((1-t)*speeds_[i]*speeds_[i]+t*speeds_[i+1]*speeds_[i+1]);
}
SpeedProfileSample PathSpeedProfile::sample(const PathMatch &match,double measured_speed_mps) const {
  SpeedProfileSample out;
  if (path_.empty() || !match.valid || match.segment>=path_.size()-1 ||
      !std::isfinite(match.fraction) || match.fraction<0 || match.fraction>1 ||
      !std::isfinite(measured_speed_mps) || measured_speed_mps<0) {
    out.reason="SPEED_PROFILE_INVALID_STATE"; return out;
  }
  out.station_m=stations_[match.segment]+match.fraction*(stations_[match.segment+1]-stations_[match.segment]);
  out.target_speed_mps=speedAt(out.station_m);
  out.speed_limit_mps=out.target_speed_mps;
  out.measured_overspeed=measured_speed_mps>out.speed_limit_mps+1e-6;
  out.valid=true;
  out.reason=out.target_speed_mps==0.0 ? "SPEED_PROFILE_END_STOP" : "CURVATURE_SPEED_PROFILE";
  return out;
}

SpeedReferencePlanner::SpeedReferencePlanner(double max_accel_mps2, double max_decel_mps2, double max_speed_mps)
    : max_accel_mps2_(max_accel_mps2), max_decel_mps2_(max_decel_mps2), max_speed_mps_(max_speed_mps) {
  if (!std::isfinite(max_accel_mps2) || max_accel_mps2<=0.0 ||
      !std::isfinite(max_decel_mps2) || max_decel_mps2<=0.0 ||
      !std::isfinite(max_speed_mps) || max_speed_mps<=0.0)
    throw std::invalid_argument("speed reference acceleration/deceleration must be finite and positive");
}
SpeedProfileSample SpeedReferencePlanner::update(SpeedProfileSample sample,
                                                 double measured_speed_mps, double dt_s) {
  if (!sample.valid || !std::isfinite(sample.target_speed_mps) || sample.target_speed_mps<0.0 ||
      !std::isfinite(measured_speed_mps) || measured_speed_mps<0.0 ||
      !std::isfinite(dt_s) || dt_s<=0.0) {
    reset(); sample.valid=false; sample.target_speed_mps=0.0;
    sample.reason="SPEED_PLAN_INVALID_STATE"; return sample;
  }
  // Endpoint/mission/protection stops remain immediate, independent of rate limits.
  if (sample.target_speed_mps==0.0) { reset(); return sample; }
  if (!previous_speed_) previous_speed_=std::min(measured_speed_mps,max_speed_mps_);
  const double dt=std::min(dt_s,0.1), requested=std::min(sample.target_speed_mps,max_speed_mps_);
  sample.speed_limit_mps=requested;
  sample.measured_overspeed=measured_speed_mps>requested+1e-6;
  double desired=requested;
  // Do not increase a low RPM reference while measured speed already exceeds
  // the route limit. A persistent bias still requires vehicle calibration/control.
  if (sample.measured_overspeed) {
    desired=std::min(desired,*previous_speed_);
    sample.reason+=":MEASURED_OVERSPEED";
  }
  const double next=std::clamp(desired,
      std::max(0.0,*previous_speed_-max_decel_mps2_*dt),
      *previous_speed_+max_accel_mps2_*dt);
  if (next<requested-1e-9) sample.reason+=":PLANNED_ACCELERATION_LIMIT";
  if (next>requested+1e-9) sample.reason+=":PLANNED_DECELERATION_LIMIT";
  sample.target_speed_mps=next;
  previous_speed_=next;
  return sample;
}
}  // namespace control_manager::core
