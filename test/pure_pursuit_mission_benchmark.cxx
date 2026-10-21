// deterministic lateral-only bicycle simulation.
// No ROS or sockets. Steering lag is a test assumption, not measured vehicle data.
#include "control_manager/core/tracking.hxx"
#include "control_manager/core/route_progress.hxx"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
namespace cm = control_manager::core;
const double pi=std::acos(-1.0);

std::vector<cm::Point> bend(double radius, bool s_turn) {
  std::vector<cm::Point> p;
  for (int i=0;i<=100;++i) p.push_back({.1*i,0});
  for (int i=1;i<=160;++i) {
    double a=(s_turn ? pi/2 : pi)*i/160;
    p.push_back({10+radius*std::sin(a),radius*(1-std::cos(a))});
  }
  if (s_turn) {
    for (int i=1;i<=160;++i) {
      double a=pi/2*i/160;
      p.push_back({10+2*radius-radius*std::cos(a),radius+radius*std::sin(a)});
    }
    for (int i=1;i<=150;++i) p.push_back({10+2*radius+.1*i,2*radius});
  } else {
    for (int i=1;i<=150;++i) p.push_back({10-.1*i,2*radius});
  }
  return p;
}

void simulate(const std::string &name,const std::vector<cm::Point> &path,double speed,
              double lag,double offset,const std::string &directory,double curve_error,bool use_progress,
              double minimum_time) {
  cm::PurePursuitConfig cfg;
#ifdef MISSION_PP
  cfg.curvature_lookahead_enabled=curve_error>0;
  if (curve_error>0) cfg.curvature_error_m=curve_error;
  cfg.minimum_lookahead_time_s=minimum_time;
#else
  (void)curve_error;
  (void)minimum_time;
#endif
  cm::AdaptivePurePursuit pp(cfg);
  const double yaw=std::atan2(path[1].y_m-path[0].y_m,path[1].x_m-path[0].x_m);
  cm::TrackingState s;
  s.x_m=path[0].x_m-offset*std::sin(yaw); s.y_m=path[0].y_m+offset*std::cos(yaw);
  s.yaw_rad=yaw; s.speed_mps=speed; s.localization_valid=s.speed_valid=true;
  std::vector<double> errors;
  double length=0,actual=0,previous=0,travel=0,steering_change=0;
  double compute_us=0,max_compute_us=0;
  int saturated=0,steps=0; bool reached=false,failed=false;
  for (std::size_t i=1;i<path.size();++i)
    length+=std::hypot(path[i].x_m-path[i-1].x_m,path[i].y_m-path[i-1].y_m);
  cm::RouteProgressConfig progress_config;
  progress_config.expected_points=path.size();
  // Same distance-third gate construction as the preliminary mode profile.
  double station=0; std::size_t front=0,rear=0;
  for (std::size_t i=1;i<path.size();++i) {
    station+=std::hypot(path[i].x_m-path[i-1].x_m,path[i].y_m-path[i-1].y_m);
    if (!front && station>=length/3) front=i;
    if (!rear && station>=2*length/3) rear=i;
  }
  progress_config.phase_end_indices={front,rear};
  cm::RouteProgressTracker progress(progress_config);
  std::string label=name+"_v"+std::to_string(int(speed*3.6+.5))+"_lag"+std::to_string(int(lag*1000))+
                    "_offset"+std::to_string(int(offset*100));
  std::ofstream trace(directory+"/"+label+".csv");
  trace<<"time_s,x_m,y_m,yaw_rad,error_m,command_rad,actual_rad\n"<<std::setprecision(12);
  // 24 deg is ONLY the provisional simulation bound, not a measured vehicle limit.
  // The user expects the real maximum to be larger. The production PP core has no 24-deg constant.
  const double dt=.02,limit=24*pi/180;
  while (travel<length*1.6+20) {
    cm::PathMatch match;
    if (use_progress) {
      auto associated=progress.update(s.x_m,s.y_m,s.yaw_rad,path,3.0);
      if (!associated.match.valid) {failed=true;break;}
      match=associated.match;
    }
    auto e=cm::pathError(s,path,use_progress ? &match : nullptr);
    if (!e.valid) {failed=true;break;}
    auto start=std::chrono::steady_clock::now();
    auto result=use_progress ? pp.compute(s,path,match) : pp.compute(s,path);
    const double duration=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
    compute_us+=duration; max_compute_us=std::max(max_compute_us,duration);
    if (!result.valid) {failed=true;break;}
    if (result.goal_reached) {reached=true;break;}
    errors.push_back(e.match.distance_m);
    if (std::abs(result.steering_rad)>limit) ++saturated;
    const double command=std::round(std::clamp(result.steering_rad,-limit,limit)*180/pi)*pi/180;
    actual+= (lag>0 ? 1-std::exp(-dt/lag) : 1)*(command-actual);
    steering_change+=std::abs(command-previous); previous=command;
    trace<<steps*dt<<','<<s.x_m<<','<<s.y_m<<','<<s.yaw_rad<<','<<e.match.distance_m<<','<<command<<','<<actual<<'\n';
    const double rate=speed/std::max(.001,cfg.wheelbase_m)*std::tan(actual);
    if (std::abs(rate)>1e-10) {
      s.x_m+=speed/rate*(std::sin(s.yaw_rad+rate*dt)-std::sin(s.yaw_rad));
      s.y_m+=speed/rate*(std::cos(s.yaw_rad)-std::cos(s.yaw_rad+rate*dt));
    } else {s.x_m+=speed*dt*std::cos(s.yaw_rad);s.y_m+=speed*dt*std::sin(s.yaw_rad);}
    s.yaw_rad=std::remainder(s.yaw_rad+rate*dt,2*pi);
    travel+=speed*dt; ++steps;
    if (e.match.distance_m>3.0) {failed=true;break;}
  }
  double squares=0,peak=0;
  for (double x:errors) {squares+=x*x;peak=std::max(peak,x);}
  std::sort(errors.begin(),errors.end());
  const double n=std::max<std::size_t>(1,errors.size());
  const double p95=errors.empty()?0:errors[std::size_t((errors.size()-1)*.95)];
  std::cout<<name<<','<<speed*3.6<<','<<lag<<','<<offset<<','<<reached<<','<<failed<<','
           <<std::sqrt(squares/n)<<','<<p95<<','<<peak<<','<<saturated/n<<','
           <<steering_change/std::max(.02,steps*dt)<<','<<compute_us/n<<','<<max_compute_us<<'\n';
}
int main(int argc,char **argv) {
  if (argc<3) return 2;
  std::ifstream f(argv[1]); std::vector<cm::Point> real;
  cm::Point p; while (f>>p.x_m>>p.y_m) real.push_back(p);
  if (real.size()<2) return 3;
  const double error=argc>3?std::stod(argv[3]):.15;
  const bool use_progress=argc>4 && std::string(argv[4])=="progress";
  const double minimum_time=argc>5 ? std::stod(argv[5]) : .6;
  std::cout<<std::setprecision(9)<<"path,speed_kph,lag_s,offset_m,reached,failed,rms_m,p95_m,max_m,saturation_ratio,steering_variation_radps,mean_compute_us,max_compute_us\n";
  for (const auto &item:std::vector<std::pair<std::string,std::vector<cm::Point>>>{{"preliminary",real},{"s_curve",bend(8,true)},{"u_turn",bend(3.5,false)}})
    for (double speed:{10/3.6,20/3.6,30/3.6})
      for (double lag:{0.0,.15,.30})
        for (double offset:{-.3,.3})
          simulate(item.first,item.second,speed,lag,offset,argv[2],error,use_progress,minimum_time);
}
