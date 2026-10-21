// target geometry, curvature preview and response horizon.
#include "control_manager/core/tracking.hxx"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace cm=control_manager::core;
namespace {
cm::TrackingState state(double speed=10/3.6) {return {0,0,0,speed,0,true,true,false,10000000000LL};}
std::vector<cm::Point> bend(double spacing=0.1) {
  std::vector<cm::Point> p;
  for (double x=0;x<2-1e-8;x+=spacing) p.push_back({x,0});
  p.push_back({2,0});
  for (double a=spacing/4;a<1.57;a+=spacing/4) p.push_back({2+4*std::sin(a),4*(1-std::cos(a))});
  p.push_back({6,4});p.push_back({6,10});return p;
}
}
TEST(MissionPurePursuit,StraightRetainsSpeedHorizonAndCorrectSteeringGeometry) {
  cm::AdaptivePurePursuit pp({});
  auto s=state();s.y_m=0.3;
  const auto out=pp.compute(s,{{0,0},{20,0}});
  ASSERT_TRUE(out.valid);ASSERT_TRUE(out.pp.valid);
  EXPECT_NEAR(out.pp.preview_curvature_per_m,0,1e-12);
  EXPECT_NEAR(out.pp.effective_lookahead_m,2+s.speed_mps,1e-12);
  EXPECT_NEAR(out.steering_rad,std::atan2(-2*1.328*.3,out.pp.target.x_m*out.pp.target.x_m+.09),1e-12);
  EXPECT_EQ(out.pp.match.segment,0U);EXPECT_LT(out.steering_rad,0);
}
TEST(MissionPurePursuit,ShortensAtBendAndKeepsTheActualChordDenominator) {
  cm::PurePursuitConfig config;
  cm::AdaptivePurePursuit pp(config);
  const auto out=pp.compute(state(),bend());
  ASSERT_TRUE(out.valid);ASSERT_TRUE(out.pp.valid);
  EXPECT_GT(out.pp.preview_curvature_per_m,.2);
  EXPECT_LT(out.pp.effective_lookahead_m,out.pp.speed_lookahead_m);
  const double chord2=out.pp.target.x_m*out.pp.target.x_m+out.pp.target.y_m*out.pp.target.y_m;
  EXPECT_NEAR(out.steering_rad,std::atan2(2*config.wheelbase_m*out.pp.target.y_m,chord2),1e-12);
  EXPECT_GT(out.steering_rad,0);
}
TEST(MissionPurePursuit,ResponseFloorPreventsTinyHighSpeedHorizon) {
  cm::PurePursuitConfig c;c.curvature_error_m=.01;
  const auto out=cm::AdaptivePurePursuit(c).compute(state(20/3.6),bend());
  ASSERT_TRUE(out.valid);
  EXPECT_NEAR(out.pp.minimum_lookahead_m,(20/3.6)*.6,1e-12);
  EXPECT_GE(out.pp.effective_lookahead_m,out.pp.minimum_lookahead_m);
  EXPECT_LE(out.pp.effective_lookahead_m,out.pp.speed_lookahead_m);
}
TEST(MissionPurePursuit,FixedCurvaturePreviewIndependentOfSpeedAndPointDensity) {
  cm::AdaptivePurePursuit pp({});
  const auto slow=pp.compute(state(1),bend()),fast=pp.compute(state(6),bend());
  ASSERT_TRUE(slow.valid);ASSERT_TRUE(fast.valid);
  EXPECT_NEAR(slow.pp.preview_curvature_per_m,fast.pp.preview_curvature_per_m,1e-12);
  const auto sparse=pp.compute(state(1),bend(.2));
  EXPECT_NEAR(slow.pp.preview_curvature_per_m,sparse.pp.preview_curvature_per_m,.003);
  auto duplicates=bend();duplicates.insert(duplicates.begin()+1,duplicates.front());
  const auto duplicate=pp.compute(state(1),duplicates);
  EXPECT_NEAR(slow.steering_rad,duplicate.steering_rad,1e-12);
}
TEST(MissionPurePursuit,PreservesExplicitRouteMatchAtCrossings) {
  const std::vector<cm::Point> p{{0,0},{20,0},{20,20},{0,20},{0,.1},{20,.1}};
  auto s=state();s.x_m=2;s.y_m=.08;
  const cm::PathMatch first{true,0,.1,.08};
  const auto out=cm::AdaptivePurePursuit({}).compute(s,p,first);
  ASSERT_TRUE(out.valid);EXPECT_EQ(out.pp.match.segment,0U);
  EXPECT_NEAR(out.pp.target.y_m,0,1e-12);EXPECT_LT(out.steering_rad,0);
}
TEST(MissionPurePursuit,DisableRestoresPreviousSpeedOnlyTarget) {
  cm::PurePursuitConfig c;c.curvature_lookahead_enabled=false;
  const auto out=cm::AdaptivePurePursuit(c).compute(state(),bend());
  ASSERT_TRUE(out.valid);
  EXPECT_NEAR(out.pp.speed_lookahead_m,out.pp.effective_lookahead_m,1e-12);
  EXPECT_DOUBLE_EQ(out.pp.preview_curvature_per_m,0);
}
TEST(MissionPurePursuit,MirrorRotationTranslationAndShortEndpoint) {
  cm::AdaptivePurePursuit pp({});auto path=bend();
  auto left=pp.compute(state(),path);
  for (auto &p:path) p.y_m=-p.y_m;
  EXPECT_NEAR(pp.compute(state(),path).steering_rad,-left.steering_rad,1e-12);
  path=bend();
  for (auto &p:path) {const double x=p.x_m;p.x_m=10-p.y_m;p.y_m=20+x;}
  auto s=state();s.x_m=10;s.y_m=20;s.yaw_rad=std::acos(-1.0)/2;
  EXPECT_NEAR(pp.compute(s,path).steering_rad,left.steering_rad,1e-12);
  auto end=pp.compute(state(),{{0,0},{1,1}});
  ASSERT_TRUE(end.valid);EXPECT_NEAR(end.pp.effective_lookahead_m,std::sqrt(2.),1e-12);
  EXPECT_NEAR(end.steering_rad,std::atan2(2*1.328,2),1e-12);
}
TEST(MissionPurePursuit,RejectUnboundedOrInvalidPreviewConfiguration) {
  for (double bad:{0.,-1.,std::numeric_limits<double>::quiet_NaN()}) {
    cm::PurePursuitConfig c;c.curvature_error_m=bad;EXPECT_THROW(cm::AdaptivePurePursuit{c},std::invalid_argument);
    c={};c.curvature_sample_m=bad;EXPECT_THROW(cm::AdaptivePurePursuit{c},std::invalid_argument);
  }
  cm::PurePursuitConfig c;c.curvature_sample_m=.001;
  EXPECT_THROW(cm::AdaptivePurePursuit{c},std::invalid_argument);
  c={};c.minimum_lookahead_time_s=-1;
  EXPECT_THROW(cm::AdaptivePurePursuit{c},std::invalid_argument);
}
