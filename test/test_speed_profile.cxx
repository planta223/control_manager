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
