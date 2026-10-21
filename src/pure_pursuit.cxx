#include "control_manager/core/tracking.hxx"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace control_manager::core {
namespace {
// sample forward by arc length on the ALREADY matched branch.
Point forwardPoint(const std::vector<Point> &path, const PathMatch &match, double distance) {
  Point start{path[match.segment].x_m + match.fraction*(path[match.segment+1].x_m-path[match.segment].x_m),
              path[match.segment].y_m + match.fraction*(path[match.segment+1].y_m-path[match.segment].y_m)};
  for (std::size_t i=match.segment+1;i<path.size();++i) {
    const double length=std::hypot(path[i].x_m-start.x_m,path[i].y_m-start.y_m);
    if (length>0 && distance<=length) {
      const double t=distance/length;
      return {start.x_m+t*(path[i].x_m-start.x_m),start.y_m+t*(path[i].y_m-start.y_m)};
    }
    distance-=length; start=path[i];
  }
  return path.back();
}
double previewCurvature(const std::vector<Point> &path, const PathError &error,
                        const PurePursuitConfig &config) {
  // Resampling in metres makes the estimate insensitive to export point density/duplicates.
  const double preview=std::min(config.curvature_preview_m,error.remaining_path_m);
  const auto count=static_cast<std::size_t>(std::floor(preview/config.curvature_sample_m));
  if (count<2) return 0.0;
  Point a=forwardPoint(path,error.match,0.0), b=forwardPoint(path,error.match,config.curvature_sample_m);
  double maximum=0.0;
  for (std::size_t i=2;i<=count;++i) {
    const auto c=forwardPoint(path,error.match,i*config.curvature_sample_m);
    const double ab=std::hypot(b.x_m-a.x_m,b.y_m-a.y_m),
                 bc=std::hypot(c.x_m-b.x_m,c.y_m-b.y_m),
                 ac=std::hypot(c.x_m-a.x_m,c.y_m-a.y_m);
    if (std::min({ab,bc,ac})>1e-9) {
      const double cross=(b.x_m-a.x_m)*(c.y_m-a.y_m)-(b.y_m-a.y_m)*(c.x_m-a.x_m);
      maximum=std::max(maximum,2.0*std::abs(cross)/(ab*bc*ac));
    }
    a=b; b=c;
  }
  return maximum;
}
}  // namespace
AdaptivePurePursuit::AdaptivePurePursuit(PurePursuitConfig config) : config_(config) {
  if (!std::isfinite(config.wheelbase_m) || config.wheelbase_m <= 0.0 ||
      !std::isfinite(config.lookahead_min_m) || config.lookahead_min_m <= 0.0 ||
      !std::isfinite(config.lookahead_max_m) || config.lookahead_max_m < config.lookahead_min_m ||
      !std::isfinite(config.lookahead_gain_s) || config.lookahead_gain_s < 0.0 ||
      !std::isfinite(config.goal_tolerance_m) || config.goal_tolerance_m < 0.0 ||
      // bound preview work and reject unusable curvature settings at startup.
      !std::isfinite(config.curvature_error_m) || config.curvature_error_m<=0.0 ||
      !std::isfinite(config.curvature_preview_m) || !std::isfinite(config.curvature_sample_m) ||
      config.curvature_sample_m<=0.0 || config.curvature_preview_m<2*config.curvature_sample_m ||
      config.curvature_preview_m/config.curvature_sample_m>128 ||
      !std::isfinite(config.minimum_lookahead_time_s) || config.minimum_lookahead_time_s<0.0)
    throw std::invalid_argument("invalid Pure Pursuit geometry/tuning");
}
LateralResult AdaptivePurePursuit::compute(const TrackingState &s,
                                          const std::vector<Point> &path) const {
  return computeImpl(s,path,nullptr);
}
LateralResult AdaptivePurePursuit::compute(const TrackingState &s,
                                          const std::vector<Point> &path, const PathMatch &match) const {
  return computeImpl(s,path,&match);
}
LateralResult AdaptivePurePursuit::computeImpl(const TrackingState &s,
                                             const std::vector<Point> &path, const PathMatch *match) const {
  const auto error=pathError(s,path,match);
  if (!error.valid) return {};
  if (goalReached(error,config_.goal_tolerance_m)) return {0.0,true,true};
  // original speed-only target selection preserved for review.
  // const auto segment=error.match.segment;
  // const auto fraction=error.match.fraction;
  // const Point projected{path[segment].x_m+fraction*(path[segment+1].x_m-path[segment].x_m),
  //                       path[segment].y_m+fraction*(path[segment+1].y_m-path[segment].y_m)};
  // double remaining = std::clamp(config_.lookahead_min_m + config_.lookahead_gain_s * s.speed_mps,
  //                               config_.lookahead_min_m, config_.lookahead_max_m);
  // Point target = path.back(), start = projected;
  // for (std::size_t i = segment + 1; i < path.size(); ++i) {
  //   const double length = std::hypot(path[i].x_m - start.x_m, path[i].y_m - start.y_m);
  //   if (length > 0.0 && remaining <= length) {
  //     const double t = remaining / length;
  //     target = {start.x_m + t * (path[i].x_m - start.x_m),start.y_m + t * (path[i].y_m - start.y_m)};
  //     break;
  //   }
  //   remaining -= length; start = path[i];
  // }
  // shorten the target horizon at bends while keeping straight-road stability.
  const double speed_lookahead=std::clamp(config_.lookahead_min_m+config_.lookahead_gain_s*s.speed_mps,
                                         config_.lookahead_min_m,config_.lookahead_max_m);
  const double curvature=config_.curvature_lookahead_enabled ? previewCurvature(path,error,config_) : 0.0;
  // Circular-arc sagitta approximation e ~= |kappa| * L^2 / 8; a tuning budget, not a lane guarantee.
  const double curve_lookahead=curvature>1e-9 ? std::sqrt(8.0*config_.curvature_error_m/curvature)
                                            : config_.lookahead_max_m;
  // at speed, shortening below a response horizon amplifies actuator lag.
  // Never extend the old speed horizon or bypass its maximum; speed planning must respect this limit.
  const double minimum_lookahead=config_.curvature_lookahead_enabled ?
    std::min(speed_lookahead,std::max(config_.lookahead_min_m,s.speed_mps*config_.minimum_lookahead_time_s)) :
    config_.lookahead_min_m;
  const double lookahead=std::clamp(std::min(speed_lookahead,curve_lookahead),
                                    minimum_lookahead,config_.lookahead_max_m);
  const Point target=forwardPoint(path,error.match,lookahead);
  const double dx = target.x_m - s.x_m, dy = target.y_m - s.y_m;
  const double forward = std::cos(s.yaw_rad) * dx + std::sin(s.yaw_rad) * dy;
  const double left = -std::sin(s.yaw_rad) * dx + std::cos(s.yaw_rad) * dy;
  const double chord2 = dx * dx + dy * dy;
  if (forward <= 0.0 || chord2 <= std::numeric_limits<double>::epsilon()) return {};
  const double angle = std::atan2(2.0 * config_.wheelbase_m * left, chord2);
  // return {angle, std::isfinite(angle), false};
  // expose the exact match/target used by this computation.
  LateralResult out{angle,std::isfinite(angle),false,{}};
  out.pp={out.valid,target,error.match,speed_lookahead,std::min(lookahead,error.remaining_path_m),curvature,
          minimum_lookahead};
  return out;
}
}  // namespace control_manager::core
