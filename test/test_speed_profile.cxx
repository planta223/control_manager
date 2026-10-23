#include "control_manager/core/speed_profile.hxx"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace cm=control_manager::core;
namespace {
cm::SpeedProfileConfig config() { return {20.0/3.6,1.5,1.0,1.328,17.01*std::acos(-1.0)/180,1.0,0.0,0.3}; }
std::vector<cm::Point> straight() {
  std::vector<cm::Point> p;
  for (int i=0;i<=1000;++i) p.push_back({i*0.1,0});
  return p;
}
cm::PathMatch at(std::size_t segment,double fraction=0.0) { return {true,segment,fraction,0.0}; }
}
TEST(SpeedProfile, StraightCeilingAndAdvanceEndDeceleration) {
  auto c=config(); cm::PathSpeedProfile p(c);
  ASSERT_EQ(p.setPath(straight()),"");
  EXPECT_DOUBLE_EQ(p.sample(at(0),0).target_speed_mps,c.max_speed_mps);
  EXPECT_NEAR(p.sample(at(800),5).target_speed_mps,c.max_speed_mps,1e-9);
  const auto slow=p.sample(at(950),5);
  EXPECT_GT(slow.target_speed_mps,0.0);
  EXPECT_LT(slow.target_speed_mps,c.max_speed_mps);
  EXPECT_LE(slow.target_speed_mps,std::sqrt(2*c.planned_decel_mps2*5));
  EXPECT_DOUBLE_EQ(p.sample(at(999,1.0),0).target_speed_mps,0.0);
}
TEST(SpeedProfile, CurveCapIsPropagatedBackToStraight) {
  std::vector<cm::Point> path;
  for(int i=0;i<=500;++i) path.push_back({i*0.1,0});
  for(int i=1;i<=200;++i) {
    const double a=i*std::acos(-1.0)/400;
    path.push_back({50+10*std::sin(a),10*(1-std::cos(a))});
  }
  for(int i=1;i<=500;++i) path.push_back({60,10+i*0.1});
  auto c=config(); c.response_time_s=0.5; cm::PathSpeedProfile p(c);
  ASSERT_EQ(p.setPath(path),"");
  EXPECT_DOUBLE_EQ(p.sample(at(0),0).target_speed_mps,c.max_speed_mps);
  EXPECT_LT(p.sample(at(480),5).target_speed_mps,c.max_speed_mps);
  const auto bend=p.sample(at(600),4);
  EXPECT_NEAR(bend.target_speed_mps,std::sqrt(c.max_lateral_accel_mps2*10),0.02);
  EXPECT_GT(p.sample(at(900),4).target_speed_mps,bend.target_speed_mps);
}
TEST(SpeedProfile, ResponseAssumptionSlowsEarlierWithoutPrematureZeroTarget) {
  auto c=config(); cm::PathSpeedProfile fast(c); c.response_time_s=0.5; cm::PathSpeedProfile slow(c);
  ASSERT_EQ(fast.setPath(straight()),""); ASSERT_EQ(slow.setPath(straight()),"");
  EXPECT_LT(slow.sample(at(900),4).target_speed_mps,fast.sample(at(900),4).target_speed_mps);
  EXPECT_GT(slow.sample(at(990),1).target_speed_mps,0.0);
}
TEST(SpeedProfile, InvalidGeometryStateAndChangedRouteFailClosed) {
  auto c=config(); cm::PathSpeedProfile p(c);
  EXPECT_FALSE(p.sample(at(0),0).valid);
  EXPECT_NE(p.setPath({{0,0},{0,0}}),"");
  EXPECT_NE(p.setPath({{0,0},{1,0},{1,1}}),"");
  ASSERT_EQ(p.setPath(straight()),"");
  EXPECT_EQ(p.setPath(straight()),"");
  auto changed=straight(); changed.back().x_m+=1;
  EXPECT_NE(p.setPath(changed),"");
  EXPECT_FALSE(p.sample(at(1001),0).valid);
  EXPECT_FALSE(p.sample(at(0),-1).valid);
  c.max_lateral_accel_mps2=0;
  EXPECT_THROW(cm::PathSpeedProfile{c},std::invalid_argument);
}

TEST(SpeedProfile, CurveExitAccelerationAndBrakingRespectSpatialLimits) {
  std::vector<cm::Point> path;
  for(int i=0;i<=300;++i) {
    const double a=i*std::acos(-1.0)/600;
    path.push_back({10*std::sin(a),10*(1-std::cos(a))});
  }
  for(int i=1;i<=700;++i) path.push_back({10,10+i*.1});
  auto c=config(); c.max_accel_mps2=.25; c.response_time_s=.5;
  cm::PathSpeedProfile p(c);
  ASSERT_EQ(p.setPath(path),"");
  double previous=p.sample(at(0),0).target_speed_mps;
  for(std::size_t i=1;i<path.size();++i) {
    const auto sample=i+1==path.size() ? p.sample(at(i-1,1.),0) : p.sample(at(i),0);
    const double speed=sample.target_speed_mps;
    const double distance=std::hypot(path[i].x_m-path[i-1].x_m,path[i].y_m-path[i-1].y_m);
    EXPECT_LE(speed*speed-previous*previous,2*c.max_accel_mps2*distance+1e-9);
    EXPECT_LE(previous*previous-speed*speed,2*c.planned_decel_mps2*distance+1e-9);
    previous=speed;
  }
  const auto bend=p.sample(at(290),0), exit=p.sample(at(310),0);
  EXPECT_LT(exit.target_speed_mps,c.max_speed_mps);
  EXPECT_GT(exit.target_speed_mps,bend.target_speed_mps);
  EXPECT_TRUE(p.sample(at(310),10).measured_overspeed);
}

namespace {
cm::SpeedProfileSample cap(double speed) {
  cm::SpeedProfileSample sample;
  sample.valid=true; sample.target_speed_mps=sample.speed_limit_mps=speed;
  sample.reason="CURVATURE_SPEED_PROFILE";
  return sample;
}
}
TEST(SpeedReferencePlanner, StartsFromMeasuredSpeedAndContinuesWithSensorLag) {
  cm::SpeedReferencePlanner planner(1.,2.,10.);
  for(int i=1;i<=40;++i) {
    const auto sample=planner.update(cap(10.),0.,.05);
    ASSERT_TRUE(sample.valid);
    EXPECT_NEAR(sample.target_speed_mps,i*.05,1e-9);
  }
  planner.reset();
  EXPECT_NEAR(planner.update(cap(10.),3.,.05).target_speed_mps,3.05,1e-9);
}
TEST(SpeedReferencePlanner, DeceleratesAtConfiguredRateAndReportsUnreachableLimit) {
  cm::SpeedReferencePlanner planner(1.,2.,10.);
  const auto first=planner.update(cap(2.),5.,.05);
  EXPECT_TRUE(first.measured_overspeed);
  EXPECT_NEAR(first.target_speed_mps,4.9,1e-9);
  EXPECT_NE(first.reason.find("MEASURED_OVERSPEED"),std::string::npos);
  EXPECT_NE(first.reason.find("PLANNED_DECELERATION_LIMIT"),std::string::npos);
  for(int i=0;i<40;++i) planner.update(cap(2.),2.,.05);
  EXPECT_DOUBLE_EQ(planner.update(cap(2.),2.,.05).target_speed_mps,2.);
  // A measured overspeed must not increase an already lower RPM reference.
  EXPECT_DOUBLE_EQ(planner.update(cap(3.),4.,.05).target_speed_mps,2.);
  EXPECT_NEAR(planner.update(cap(3.),2.,.05).target_speed_mps,2.05,1e-9);
}
TEST(SpeedReferencePlanner, StopsInvalidInputsAndResetsWithoutDelayedTimerCatchup) {
  cm::SpeedReferencePlanner planner(1.,2.,10.);
  EXPECT_DOUBLE_EQ(planner.update(cap(10.),0.,5.).target_speed_mps,.1);
  EXPECT_DOUBLE_EQ(planner.update(cap(0.),5.,.05).target_speed_mps,0.);
  EXPECT_NEAR(planner.update(cap(10.),0.,.05).target_speed_mps,.05,1e-9);
  for(double bad : {-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_FALSE(planner.update(cap(10.),bad,.05).valid);
    EXPECT_FALSE(planner.update(cap(10.),0.,bad).valid);
  }
  EXPECT_FALSE(planner.update(cap(10.),0.,0.).valid);
  EXPECT_FALSE(planner.update(cm::SpeedProfileSample{},0.,.05).valid);
  EXPECT_THROW(cm::SpeedReferencePlanner(0.,1.,10.),std::invalid_argument);
  planner.reset();
  EXPECT_LE(planner.update(cap(100.),100.,.05).target_speed_mps,10.);
  auto c=config(); c.max_accel_mps2=0.;
  EXPECT_THROW(cm::PathSpeedProfile{c},std::invalid_argument);
}
