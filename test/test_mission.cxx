#include <gtest/gtest.h>
#include <limits>
#include "control_manager/core/mission.hxx"
namespace cm = control_manager::core;
namespace {
const std::vector<cm::Point> path{{0, 0}, {10, 0}, {20, 0}, {30, 0}};
const std::vector<double> stations{0, 10, 20, 30};
const cm::MissionConfig config{10.0 / 3.6, 1.0, 0.2, 2.0};
cm::GlobalPose pose(double x, double y = 0) {
  cm::GlobalPose p; p.x_m = x; p.y_m = y; p.valid = true; return p;
}
cm::MissionSection section(bool stop = false) { return {"turn", 10, 20, 1.0, 3.0, 4.0, stop}; }
}
TEST(Mission, DefaultAndAsymmetricMargins) {
  for (double x : {0.0, 6.9, 24.1, 30.0}) {
    auto d = cm::evaluateMission(pose(x), path, stations, {section()}, config);
    ASSERT_TRUE(d.valid); EXPECT_FALSE(d.stop_required); EXPECT_DOUBLE_EQ(d.max_speed_mps, 10.0 / 3.6);
  }
  for (double x : {7.0, 9.0, 10.0, 20.0, 24.0}) {
    auto d = cm::evaluateMission(pose(x), path, stations, {section()}, config);
    ASSERT_TRUE(d.valid); EXPECT_FALSE(d.stop_required); EXPECT_DOUBLE_EQ(d.max_speed_mps, 1.0);
  }
}
TEST(Mission, StopUsesOriginalSectionAndOverlapsUseMostRestrictive) {
  auto d = cm::evaluateMission(pose(9), path, stations, {section(true)}, config);
  ASSERT_TRUE(d.valid); EXPECT_FALSE(d.stop_required); EXPECT_DOUBLE_EQ(d.max_speed_mps, 1.0);
  d = cm::evaluateMission(pose(15), path, stations,
    {section(true), {"slow", 14, 25, 0.5, 0, 0, false}}, config);
  ASSERT_TRUE(d.valid); EXPECT_TRUE(d.stop_required); EXPECT_DOUBLE_EQ(d.max_speed_mps, 0.0);
  EXPECT_EQ(d.reason, "MISSION_SECTION_STOP:turn");
  d = cm::evaluateMission(pose(15), path, stations,
    {section(false), {"slow", 14, 25, 0.5, 0, 0, false}}, config);
  ASSERT_TRUE(d.valid); EXPECT_DOUBLE_EQ(d.max_speed_mps, 0.5);
}
TEST(Mission, PositionUncertaintyAtBoundaryAppliesConservativePolicy) {
  auto p = pose(9.5); p.position_variance_x_m2 = 0.25;
  auto s = section(true); s.margin_before_m = s.margin_after_m = 0;
  auto d = cm::evaluateMission(p, path, stations, {s}, config);
  ASSERT_TRUE(d.valid); EXPECT_TRUE(d.stop_required);
  d = cm::evaluateMission(pose(9.5), path, stations, {s}, config);
  ASSERT_TRUE(d.valid); EXPECT_FALSE(d.stop_required);
}
TEST(Mission, CrossingIsUncertainAndNeverSelectsByPointOrder) {
  const std::vector<cm::Point> crossing{{-10, 0}, {10, 0}, {10, 10}, {0, 10}, {0, -10}};
  auto d = cm::evaluateMission(pose(0), crossing, {0, 20, 30, 40, 60}, {}, config);
  EXPECT_FALSE(d.valid); EXPECT_TRUE(d.stop_required); EXPECT_EQ(d.reason, "MISSION_SECTION_UNCERTAIN");
  d = cm::evaluateMission(pose(-5), crossing, {0, 20, 30, 40, 60}, {}, config);
  EXPECT_TRUE(d.valid);
}
TEST(Mission, InvalidPoseMapAndOffPathStop) {
  auto d = cm::evaluateMission(pose(5, 2), path, stations, {}, config);
  EXPECT_FALSE(d.valid); EXPECT_TRUE(d.stop_required); EXPECT_EQ(d.reason, "MISSION_OFF_PATH");
  auto p = pose(5); p.position_variance_x_m2 = -1;
  EXPECT_FALSE(cm::evaluateMission(p, path, stations, {}, config).valid);
  p = pose(5); p.x_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(cm::evaluateMission(p, path, stations, {}, config).valid);
  EXPECT_FALSE(cm::evaluateMission(pose(5), path, {0, 10}, {}, config).valid);
  EXPECT_FALSE(cm::evaluateMission(pose(5), path, {0, 10, 10, 30}, {}, config).valid);
  auto s = section(); s.end_s_m = 31;
  EXPECT_FALSE(cm::evaluateMission(pose(5), path, stations, {s}, config).valid);
}
