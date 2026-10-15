#include <gtest/gtest.h>
#include <limits>
#include "control_manager/ingress.hxx"
namespace cm = control_manager;
namespace {
constexpr std::int64_t now_ns = 10000000000LL;
cm::msg::LanePose lane() {
  cm::msg::LanePose s;
  s.header.stamp.sec = 10; s.header.frame_id = "external_lane_reference";
  s.lateral_error_m = -0.2; s.heading_error_rad = 0.4; s.confidence = 0.8;
  return s;
}
cm::msg::VehicleFeedback feedback() {
  cm::msg::VehicleFeedback s;
  s.header.stamp.sec = 10; s.header.frame_id = "base_link";
  s.speed_mps = 1.0; s.steering_angle_rad = -0.2;
  s.speed_valid = true; s.steering_valid = true;
  return s;
}
}
TEST(IngressLane, FourFieldsAndLowConfidenceRemainObservations) {
  auto s = lane(); EXPECT_TRUE(cm::ingress::admissibleLane(s,now_ns,.5));
  s.confidence = 0; EXPECT_TRUE(cm::ingress::admissibleLane(s,now_ns,.5));
  s.confidence = 1; EXPECT_TRUE(cm::ingress::admissibleLane(s,now_ns,.5));
}
TEST(IngressLane, RejectMalformedValues) {
  for (double bad : {std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::infinity()}) {
    auto s = lane(); s.lateral_error_m = bad;
    EXPECT_FALSE(cm::ingress::admissibleLane(s,now_ns,.5));
    s = lane(); s.heading_error_rad = bad;
    EXPECT_FALSE(cm::ingress::admissibleLane(s,now_ns,.5));
    s = lane(); s.confidence = bad;
    EXPECT_FALSE(cm::ingress::admissibleLane(s,now_ns,.5));
  }
  for (double bad : {-0.1,1.1}) {
    auto s = lane(); s.confidence = bad;
    EXPECT_FALSE(cm::ingress::admissibleLane(s,now_ns,.5));
  }
}
TEST(IngressLane, RejectZeroStaleFutureNegativeAndNoncanonicalTime) {
  for (int sec : {0,9,11,-1}) {
    auto s = lane(); s.header.stamp.sec = sec;
    EXPECT_FALSE(cm::ingress::admissibleLane(s,now_ns,.5));
  }
  auto s = lane(); s.header.stamp.nanosec = 1000000000U;
  EXPECT_FALSE(cm::ingress::admissibleLane(s,now_ns,.5));
}
TEST(IngressFeedback, PreserveActualValuesAndTimeNeverPromoteInvalid) {
  const auto s = feedback();
  auto out = cm::ingress::checkedFeedback(s,now_ns,.5,"base_link");
  EXPECT_TRUE(out.speed_valid); EXPECT_TRUE(out.steering_valid);
  EXPECT_EQ(out.header,s.header); EXPECT_DOUBLE_EQ(out.speed_mps,s.speed_mps);
  EXPECT_DOUBLE_EQ(out.steering_angle_rad,s.steering_angle_rad);
  auto invalid = s; invalid.speed_valid = false; invalid.steering_valid = false;
  out = cm::ingress::checkedFeedback(invalid,now_ns,.5,"base_link");
  EXPECT_FALSE(out.speed_valid); EXPECT_FALSE(out.steering_valid);
}
TEST(IngressFeedback, ValidateEachFieldAndRejectBadFrameOrTime) {
  auto s = feedback(); s.speed_mps = std::numeric_limits<double>::quiet_NaN();
  auto out = cm::ingress::checkedFeedback(s,now_ns,.5,"base_link");
  EXPECT_FALSE(out.speed_valid); EXPECT_TRUE(out.steering_valid);
  s = feedback(); s.steering_angle_rad = std::numeric_limits<double>::infinity();
  out = cm::ingress::checkedFeedback(s,now_ns,.5,"base_link");
  EXPECT_TRUE(out.speed_valid); EXPECT_FALSE(out.steering_valid);
  for (int sec : {0,9,11,-1}) {
    s = feedback(); s.header.stamp.sec = sec;
    out = cm::ingress::checkedFeedback(s,now_ns,.5,"base_link");
    EXPECT_FALSE(out.speed_valid); EXPECT_FALSE(out.steering_valid);
  }
  out = cm::ingress::checkedFeedback(feedback(),now_ns,.5,"wrong_frame");
  EXPECT_FALSE(out.speed_valid); EXPECT_FALSE(out.steering_valid);
}
