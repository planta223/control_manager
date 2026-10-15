#include <cmath>
#include <gtest/gtest.h>
#include <limits>
#include "control_manager/core/lateral_controller.hxx"
namespace cm = control_manager::core;
namespace {
constexpr std::int64_t now_ns = 10000000000LL;
cm::TrackingState state() { return {0, 0, 0, 1, 0, true, true, false, now_ns}; }
cm::Command command() { return {0.1, 1.0, false, true, now_ns, "TEST_FIXTURE"}; }
cm::ActuatorProtectionConfig config() { return {true, 0.4, 2.0, 0.5}; }
}
TEST(Freshness, RejectZeroFutureAndExpired) {
  EXPECT_FALSE(cm::fresh(0, now_ns, 0.5));
  EXPECT_FALSE(cm::fresh(now_ns+1, now_ns, 0.5));
  EXPECT_FALSE(cm::fresh(now_ns-600000000, now_ns, 0.5));
  EXPECT_TRUE(cm::fresh(now_ns-500000000, now_ns, 0.5));
}
TEST(Arbitration, ModeIsolationAndTimestampPreserved) {
  auto m = command(), a = command(); m.target_speed_mps = 0.4; a.target_speed_mps = 0.8;
  EXPECT_DOUBLE_EQ(cm::selectCommand(cm::ControlMode::MANUAL,m,a,now_ns).target_speed_mps,0.4);
  EXPECT_DOUBLE_EQ(cm::selectCommand(cm::ControlMode::AUTO,m,a,now_ns).target_speed_mps,0.8);
  m.valid = false;
  EXPECT_FALSE(cm::selectCommand(cm::ControlMode::MANUAL,m,a,now_ns).valid);
  EXPECT_TRUE(cm::selectCommand(cm::ControlMode::IDLE,m,a,now_ns).target_brake);
  a.timestamp_ns -= 100;
  EXPECT_EQ(cm::selectCommand(cm::ControlMode::AUTO,m,a,now_ns).timestamp_ns,a.timestamp_ns);
}
TEST(ActuatorProtection, EstopOverridesModesAndInvalidCommand) {
  for (auto mode : {cm::ControlMode::IDLE, cm::ControlMode::AUTO, cm::ControlMode::MANUAL}) {
    const auto c = cm::selectCommand(mode,command(),command(),now_ns);
    const auto out = cm::protectActuatorCommand(c,true,now_ns,config());
    EXPECT_EQ(out.reason,"E_STOP"); EXPECT_FALSE(out.valid);
    EXPECT_TRUE(out.target_brake); EXPECT_DOUBLE_EQ(out.target_speed_mps,0);
    EXPECT_DOUBLE_EQ(out.target_steer_rad,0);
  }
  EXPECT_EQ(cm::protectActuatorCommand({},true,now_ns,{}).reason,"E_STOP");
}
TEST(ActuatorProtection, InvalidLimitsFailClosed) {
  EXPECT_EQ(cm::protectActuatorCommand(command(),false,now_ns,{}).reason,"OPERATING_LIMITS_NOT_VERIFIED");
  for (double bad : {0., -1., std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::infinity()}) {
    auto cfg=config(); cfg.max_speed_mps=bad;
    EXPECT_FALSE(cm::protectActuatorCommand(command(),false,now_ns,cfg).valid);
    cfg=config(); cfg.max_steer_rad=bad;
    EXPECT_FALSE(cm::protectActuatorCommand(command(),false,now_ns,cfg).valid);
  }
}
TEST(ActuatorProtection, InvalidAndStaleSourceStop) {
  auto c=command(); c.valid=false;
  EXPECT_EQ(cm::protectActuatorCommand(c,false,now_ns,config()).reason,"INVALID_OR_STALE_COMMAND");
  for (auto stamp : {0LL, now_ns+1LL, now_ns-600000000LL}) {
    c=command(); c.timestamp_ns=stamp;
    const auto out=cm::protectActuatorCommand(c,false,now_ns,config());
    EXPECT_FALSE(out.valid); EXPECT_TRUE(out.target_brake);
    EXPECT_DOUBLE_EQ(out.target_speed_mps,0); EXPECT_DOUBLE_EQ(out.target_steer_rad,0);
  }
}
TEST(ActuatorProtection, FiniteRangeAndContradictoryBrake) {
  auto c=command();
  for (double bad : {std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::infinity(),
                    -std::numeric_limits<double>::infinity()}) {
    c=command(); c.target_steer_rad=bad;
    EXPECT_EQ(cm::protectActuatorCommand(c,false,now_ns,config()).reason,"INVALID_COMMAND_VALUES");
    c=command(); c.target_speed_mps=bad;
    EXPECT_EQ(cm::protectActuatorCommand(c,false,now_ns,config()).reason,"INVALID_COMMAND_VALUES");
  }
  c=command(); c.target_speed_mps=-.1;
  EXPECT_FALSE(cm::protectActuatorCommand(c,false,now_ns,config()).valid);
  c=command(); c.target_brake=true;
  EXPECT_FALSE(cm::protectActuatorCommand(c,false,now_ns,config()).valid);
  for (double steer : {-.41,.41}) {
    c=command(); c.target_steer_rad=steer;
    EXPECT_EQ(cm::protectActuatorCommand(c,false,now_ns,config()).reason,"COMMAND_OUT_OF_RANGE");
  }
  c=command(); c.target_speed_mps=2.01;
  EXPECT_EQ(cm::protectActuatorCommand(c,false,now_ns,config()).reason,"COMMAND_OUT_OF_RANGE");
}
TEST(ActuatorProtection, BoundaryAcceptedAndSourceTimePreserved) {
  auto c=command(); c.target_speed_mps=2.; c.target_steer_rad=-.4; c.timestamp_ns-=100;
  const auto out=cm::protectActuatorCommand(c,false,now_ns,config());
  EXPECT_TRUE(out.valid); EXPECT_EQ(out.timestamp_ns,c.timestamp_ns);
  EXPECT_DOUBLE_EQ(out.target_speed_mps,2.); EXPECT_DOUBLE_EQ(out.target_steer_rad,-.4);
}
TEST(Fsm, ModeAndFsmAreSeparate) {
  EXPECT_EQ(cm::fsmState(false,false,{},cm::ControlMode::AUTO),cm::VehicleFsmState::INIT);
  EXPECT_EQ(cm::fsmState(true,false,command(),cm::ControlMode::AUTO),cm::VehicleFsmState::FAULT);
  EXPECT_EQ(cm::fsmState(true,true,command(),cm::ControlMode::IDLE),cm::VehicleFsmState::READY);
  EXPECT_EQ(cm::fsmState(true,true,command(),cm::ControlMode::MANUAL),cm::VehicleFsmState::DRIVING);
  EXPECT_EQ(cm::fsmState(true,true,cm::stop(now_ns,"stop",true),cm::ControlMode::AUTO),cm::VehicleFsmState::STOPPED);
}
TEST(PurePursuit, StraightLeftRightAndRotation) {
  cm::AdaptivePurePursuit pp({});
  auto s = state();
  EXPECT_NEAR(pp.compute(s,{{0,0},{20,0}}).steering_rad,0,1e-12);
  EXPECT_GT(pp.compute(s,{{0,1},{20,1}}).steering_rad,0);
  EXPECT_LT(pp.compute(s,{{0,-1},{20,-1}}).steering_rad,0);
  s.yaw_rad = std::acos(-1.0)/2;
  EXPECT_NEAR(pp.compute(s,{{0,0},{0,20}}).steering_rad,0,1e-12);
}
TEST(PurePursuit, AdaptiveLookaheadAndActualChordNearEnd) {
  cm::AdaptivePurePursuit pp({}); auto slow = state(), fast = state(); fast.speed_mps = 4;
  EXPECT_GT(pp.compute(slow,{{0,1},{20,1}}).steering_rad,
            pp.compute(fast,{{0,1},{20,1}}).steering_rad);
  const auto near_end = pp.compute(slow,{{0,1},{1,1}});
  ASSERT_TRUE(near_end.valid);
  EXPECT_NEAR(near_end.steering_rad,std::atan2(2*1.328,2.0),1e-12);
}
TEST(PurePursuit, RejectDegenerateBehindNonfiniteAndDetectGoal) {
  cm::AdaptivePurePursuit pp({}); auto s = state();
  EXPECT_FALSE(pp.compute(s,{}).valid);
  EXPECT_FALSE(pp.compute(s,{{0,0},{0,0}}).valid);
  EXPECT_FALSE(pp.compute(s,{{-10,0},{-1,0}}).valid);
  EXPECT_FALSE(pp.compute(s,{{0,0},{std::numeric_limits<double>::quiet_NaN(),0}}).valid);
  s.x_m = 10; EXPECT_TRUE(pp.compute(s,{{0,0},{10,0}}).goal_reached);
  s.localization_valid = false; EXPECT_FALSE(pp.compute(s,{{0,0},{20,0}}).valid);
}
