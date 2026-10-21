#include <gtest/gtest.h>
#include "control_manager/core/stopped_heading.hxx"
using control_manager::core::StoppedHeadingMemory;
using control_manager::core::StoppedHeadingConfig;

TEST(StoppedHeading, NoMemoryForStartupOrUncommandedLoss) {
  StoppedHeadingMemory h({});
  EXPECT_FALSE(h.update(0,true,true,0,0,0,std::nullopt).yaw_rad);
  h.update(1,true,false,0,0,1,1.2);
  EXPECT_FALSE(h.update(2,true,false,0,0,0,std::nullopt).yaw_rad);
  // A pause after an unrelated heading failure must not legitimize old data.
  EXPECT_FALSE(h.update(3,true,true,0,0,0,std::nullopt).yaw_rad);
}
TEST(StoppedHeading, KeepsLatestYawAcrossLongStopAndUsesFreshYawFirst) {
  StoppedHeadingMemory h({});
  h.update(0,true,true,0,0,0,1.1);
  // Validity-degraded quaternion is never treated as a new yaw.
  EXPECT_DOUBLE_EQ(*h.update(100,true,true,0,0,0,std::nullopt).yaw_rad,1.1);
  EXPECT_DOUBLE_EQ(*h.update(101,true,false,0,0,0,std::nullopt).yaw_rad,1.1);
  EXPECT_DOUBLE_EQ(*h.update(102,true,false,.1,0,.3,std::nullopt).yaw_rad,1.1);
  EXPECT_FALSE(h.update(102.1,true,false,.2,0,.8,1.3).yaw_rad);
  EXPECT_FALSE(h.update(102.2,true,false,.3,0,.8,std::nullopt).yaw_rad);
}
TEST(StoppedHeading, ArmsDuringDecelerationAndFreezesLastAcceptedYaw) {
  StoppedHeadingMemory h({});
  h.update(0,true,true,0,0,2,1.0);
  h.update(.1,true,true,.1,0,1,1.2);
  EXPECT_FALSE(h.update(.2,true,true,.2,0,.5,std::nullopt).yaw_rad);
  EXPECT_DOUBLE_EQ(*h.update(.3,true,true,.3,0,0,std::nullopt).yaw_rad,1.2);
}
TEST(StoppedHeading, DoesNotAverageAcrossTurnOrAngleWrap) {
  StoppedHeadingMemory h({});
  h.update(0,true,true,0,0,.5,3.13);
  h.update(.1,true,true,0,0,.5,-3.12);
  h.update(.2,true,true,0,0,0,-3.10);
  EXPECT_DOUBLE_EQ(*h.update(.3,true,false,0,0,0,std::nullopt).yaw_rad,-3.10);
}
TEST(StoppedHeading, InvalidObservationsDiscardMemory) {
  StoppedHeadingMemory h({});
  h.update(0,true,true,0,0,0,1.0);
  EXPECT_FALSE(h.update(1,false,true,0,0,0,std::nullopt).yaw_rad);
  EXPECT_FALSE(h.update(2,true,false,0,0,.1,std::nullopt).yaw_rad);
}
TEST(StoppedHeading, BoundedDistanceAndMovementTimeCannotBeResetByRequests) {
  StoppedHeadingMemory distance({});
  distance.update(0,true,true,0,0,0,1.0);
  EXPECT_EQ(distance.update(1,true,false,1.1,0,.2,std::nullopt).reason,"HEADING_HOLD_DISTANCE_EXCEEDED");
  EXPECT_FALSE(distance.update(2,true,true,0,0,0,std::nullopt).yaw_rad);
  StoppedHeadingMemory timed({});
  timed.update(0,true,true,0,0,0,1.0);
  EXPECT_TRUE(timed.update(100,true,false,0,0,.2,std::nullopt).yaw_rad);
  EXPECT_TRUE(timed.update(102,true,true,0,0,0,std::nullopt).yaw_rad);
  EXPECT_EQ(timed.update(103,true,false,0,0,.2,std::nullopt).reason,"HEADING_HOLD_REACQUIRE_TIMEOUT");
}
TEST(StoppedHeading, DisabledAndInvalidConfiguration) {
  StoppedHeadingConfig c; c.enabled=false;
  StoppedHeadingMemory h(c);
  h.update(0,true,true,0,0,0,1.0);
  EXPECT_FALSE(h.update(1,true,false,0,0,0,std::nullopt).yaw_rad);
  c.reacquire_speed_mps=c.stop_speed_mps;
  EXPECT_THROW(StoppedHeadingMemory{c},std::invalid_argument);
}
