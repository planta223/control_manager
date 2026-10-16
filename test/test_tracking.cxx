#include <cmath>
#include <limits>
#include <gtest/gtest.h>
#include "control_manager/core/tracking.hxx"
namespace cm = control_manager::core;
namespace {
const std::vector<cm::Point> straight{{0,0},{20,0}};
cm::TrackingState state(double x = 1, double y = 0, double yaw = 0) {
  return {x,y,yaw,1,0,true,true,false,10000000000LL};
}
cm::PidConfig config() {
  cm::PidConfig c; c.enabled = true; c.kp_lateral = 0.2; c.kp_heading = 0.4;
  c.integral_limit_m_s = 0.5; c.max_steering_rad = 0.6; return c;
}
}
TEST(PidGeometry, OnPathAndLeftRightSigns) {
  cm::PidLateralController pid(config());
  auto out = pid.compute(state(),straight,0.1);
  ASSERT_TRUE(out.valid); EXPECT_NEAR(out.steering_rad,0,1e-12);
  EXPECT_NEAR(out.lateral_error_m,0,1e-12); EXPECT_NEAR(out.heading_error_rad,0,1e-12);
  out = pid.compute(state(1,1),straight,0.1);
  ASSERT_TRUE(out.valid); EXPECT_NEAR(out.lateral_error_m,-1,1e-12);
  EXPECT_LT(out.steering_rad,0);  // Vehicle left of eastbound path -> steer right.
  out = pid.compute(state(1,-1),straight,0.1);
  ASSERT_TRUE(out.valid); EXPECT_NEAR(out.lateral_error_m,1,1e-12); EXPECT_GT(out.steering_rad,0);
}
TEST(PidGeometry, HeadingCorrectionAndWrap) {
  cm::PidLateralController pid(config());
  auto out = pid.compute(state(1,0,0.2),straight,0.1);
  ASSERT_TRUE(out.valid); EXPECT_NEAR(out.heading_error_rad,-0.2,1e-12);
  EXPECT_NEAR(out.steering_rad,-0.08,1e-12);
  out = pid.compute(state(1,0,-0.2),straight,0.1);
  EXPECT_NEAR(out.steering_rad,0.08,1e-12);
  const double pi = std::acos(-1.0);
  out = pid.compute(state(-1,0,-pi+0.1),{{0,0},{-20,0}},0.1);
  ASSERT_TRUE(out.valid); EXPECT_NEAR(out.heading_error_rad,-0.1,1e-12);
}
TEST(PidGeometry, RotationTranslationAndDuplicatePoints) {
  cm::PidLateralController pid(config());
  auto out = pid.compute(state(9,21,std::acos(-1.0)/2),{{10,20},{10,20},{10,40}},0.1);
  ASSERT_TRUE(out.valid); EXPECT_NEAR(out.lateral_error_m,-1,1e-12);
  EXPECT_NEAR(out.heading_error_rad,0,1e-12); EXPECT_NEAR(out.steering_rad,-0.2,1e-12);
}
TEST(PidGeometry, RejectInsufficientDegenerateAndNonfinite) {
  cm::PidLateralController pid(config());
  EXPECT_FALSE(pid.compute(state(),{},0.1).valid);
  EXPECT_FALSE(pid.compute(state(),{{0,0}},0.1).valid);
  EXPECT_FALSE(pid.compute(state(),{{0,0},{0,0}},0.1).valid);
  for (double bad : {std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}) {
    EXPECT_FALSE(pid.compute(state(),{{0,0},{bad,0}},0.1).valid);
    EXPECT_FALSE(pid.compute(state(1,bad),straight,0.1).valid);
    EXPECT_FALSE(pid.compute(state(1,0,bad),straight,0.1).valid);
    auto s = state(); s.speed_mps = bad; EXPECT_FALSE(pid.compute(s,straight,0.1).valid);
  }
  EXPECT_FALSE(pid.compute(state(),{{0,1e200},{20,1e200}},0.1).valid);
  auto s = state(); s.localization_valid = false; EXPECT_FALSE(pid.compute(s,straight,0.1).valid);
  s = state(); s.speed_valid = false; EXPECT_FALSE(pid.compute(s,straight,0.1).valid);
  s = state(); s.speed_mps = -1; EXPECT_FALSE(pid.compute(s,straight,0.1).valid);
}
TEST(PidGeometry, GoalAndEndBehindAndReverseHeading) {
  cm::PidLateralController pid(config());
  auto out = pid.compute(state(20,0),straight,0.1);
  ASSERT_TRUE(out.valid); EXPECT_TRUE(out.goal_reached); EXPECT_EQ(out.steering_rad,0);
  EXPECT_FALSE(pid.compute(state(21,1),straight,0.1).valid);
  EXPECT_FALSE(pid.compute(state(1,0,std::acos(-1.0)),straight,0.1).valid);
}
TEST(PidState, IntegralAccumulatesAndClampsBothSigns) {
  auto c = config(); c.kp_lateral = c.kp_heading = 0; c.ki_lateral = 1;
  c.integral_limit_m_s = 0.25;
  cm::PidLateralController pid(c);
  EXPECT_EQ(pid.compute(state(1,-1),straight,0.1).steering_rad,0);  // No hidden first dt.
  EXPECT_NEAR(pid.compute(state(1,-1),straight,0.1).steering_rad,0.1,1e-12);
  for (int i=0;i<20;++i) EXPECT_LE(pid.compute(state(1,-1),straight,0.1).steering_rad,0.25);
  EXPECT_NEAR(pid.compute(state(1,-1),straight,0.1).steering_rad,0.25,1e-12);
  for (int i=0;i<20;++i) pid.compute(state(1,1),straight,0.1);
  EXPECT_NEAR(pid.compute(state(1,1),straight,0.1).steering_rad,-0.25,1e-12);
}
TEST(PidState, DerivativeUsesDtAndNoKickAfterReset) {
  auto c = config(); c.kp_lateral = c.kp_heading = 0; c.kd_lateral = 0.1;
  cm::PidLateralController pid(c);
  EXPECT_EQ(pid.compute(state(1,-1),straight,0.1).steering_rad,0);
  EXPECT_NEAR(pid.compute(state(1,-1.1),straight,0.2).steering_rad,0.05,1e-12);
  pid.reset(); EXPECT_EQ(pid.compute(state(1,-5),straight,0.1).steering_rad,0);
}
TEST(PidState, SteeringClampAndFormula) {
  auto c = config(); c.ki_lateral = 0.1; c.kd_lateral = 0.02;
  cm::PidLateralController pid(c);
  pid.compute(state(1,-0.5),straight,0.1);
  auto out = pid.compute(state(1,-0.6,-0.1),straight,0.1);
  ASSERT_TRUE(out.valid);
  EXPECT_NEAR(out.steering_rad,0.2*0.6+0.1*0.06+0.02*1.0+0.4*0.1,1e-12);
  EXPECT_DOUBLE_EQ(pid.compute(state(1,-10),straight,0.1).steering_rad,0.6);
  EXPECT_DOUBLE_EQ(pid.compute(state(1,10),straight,0.1).steering_rad,-0.6);
}
TEST(PidState, InvalidInputAndTimeoutResetHistory) {
  auto c = config(); c.kp_lateral = c.kp_heading = 0; c.ki_lateral = 1;
  cm::PidLateralController pid(c);
  for (double dt : {0.0,-0.1,0.6,std::numeric_limits<double>::quiet_NaN()}) {
    pid.compute(state(1,-1),straight,0.1); pid.compute(state(1,-1),straight,0.1);
    EXPECT_FALSE(pid.compute(state(1,-1),straight,dt).valid);
    EXPECT_EQ(pid.compute(state(1,-1),straight,0.1).steering_rad,0);
  }
  pid.compute(state(1,-1),straight,0.1);
  EXPECT_FALSE(pid.compute(state(),{},0.1).valid);
  EXPECT_EQ(pid.compute(state(1,-1),straight,0.1).steering_rad,0);
}
TEST(PidState, DefaultsDisabledAndInvalidConfigurationRejected) {
  cm::PidLateralController pid({});
  EXPECT_EQ(pid.compute(state(),straight,0.1).reason,"PID_NOT_CONFIGURED");
  auto c = config(); c.kp_lateral = -1;
  EXPECT_THROW(cm::PidLateralController{c},std::invalid_argument);
  c = config(); c.max_dt_s = 0;
  EXPECT_THROW(cm::PidLateralController{c},std::invalid_argument);
  c = config(); c.kd_lateral = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(cm::PidLateralController{c},std::invalid_argument);
}
TEST(Tracking, DefaultPidAndEveryInvalidEnum) {
  cm::TrackingController tracker(config(),{});
  EXPECT_EQ(tracker.requested(),0);
  ASSERT_TRUE(tracker.compute(state(),straight,0.1).valid);
  for (int value=2;value<=255;++value) {
    tracker.request(value);
    EXPECT_EQ(tracker.requested(),value);
    auto out = tracker.compute(state(),straight,0.1);
    EXPECT_FALSE(out.valid); EXPECT_EQ(out.reason,"INVALID_TRACKING_MODE");
  }
}
TEST(Tracking, ModeSwitchAndInactiveAutoResetPid) {
  auto c = config(); c.kp_lateral = c.kp_heading = 0; c.ki_lateral = 1;
  cm::TrackingController tracker(c,{});
  tracker.compute(state(1,-1),straight,0.1);
  EXPECT_GT(tracker.compute(state(1,-1),straight,0.1).steering_rad,0);
  tracker.request(0);  // Repeated same-mode request must not erase I.
  EXPECT_GT(tracker.compute(state(1,-1),straight,0.1).steering_rad,0.1);
  tracker.request(1); tracker.request(0);
  EXPECT_EQ(tracker.compute(state(1,-1),straight,0.1).steering_rad,0);
  EXPECT_GT(tracker.compute(state(1,-1),straight,0.1).steering_rad,0);
  EXPECT_EQ(tracker.compute(state(1,-1),straight,0.1,false).steering_rad,0);
  EXPECT_EQ(tracker.compute(state(1,-1),straight,0.1,true).steering_rad,0);
  tracker.reset(); EXPECT_EQ(tracker.compute(state(1,-1),straight,0.1).steering_rad,0);
}
TEST(Tracking, PurePursuitRegressionUsesUnchangedLegacyAlgorithm) {
  cm::PurePursuitConfig c;
  cm::AdaptivePurePursuit legacy(c);
  cm::TrackingController tracker(config(),c); tracker.request(1);
  for (const auto &path : {straight, std::vector<cm::Point>{{0,1},{20,1}},
                          std::vector<cm::Point>{{0,-1},{1,-1}},
                          std::vector<cm::Point>{{0,0},{5,0},{5,10}}}) {
    for (double speed : {0.0,1.0,4.0}) {
      auto s = state(0,0); s.speed_mps = speed;
      const auto a = legacy.compute(s,path);
      const auto b = tracker.compute(s,path,0.1);
      EXPECT_EQ(a.valid,b.valid); EXPECT_EQ(a.goal_reached,b.goal_reached);
      EXPECT_DOUBLE_EQ(a.steering_rad,b.steering_rad);
    }
  }
  EXPECT_NEAR(tracker.compute(state(0,0),{{0,1},{1,1}},0.1).steering_rad,
              std::atan2(2*1.328,2.0),1e-12);
}
TEST(PathDistance, IndependentOfHeadingAndUsesClampedSegments) {
  EXPECT_DOUBLE_EQ(cm::distanceToPath(5,3,straight),3.0);
  EXPECT_DOUBLE_EQ(cm::distanceToPath(5,-3,straight),3.0);
  EXPECT_DOUBLE_EQ(cm::distanceToPath(-3,4,straight),5.0);
  EXPECT_DOUBLE_EQ(cm::distanceToPath(3,5,{{0,0},{0,0},{0,20}}),3.0);
  EXPECT_FALSE(std::isfinite(cm::distanceToPath(0,0,{})));
  EXPECT_FALSE(std::isfinite(cm::distanceToPath(0,0,{{0,0},{0,0}})));
  EXPECT_FALSE(std::isfinite(cm::distanceToPath(0,0,{{0,0},{1,std::numeric_limits<double>::quiet_NaN()}})));
}
