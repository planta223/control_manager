#include <gtest/gtest.h>
#include "control_manager/core/guidance.hxx"
#include "control_manager/core/types.hxx"
namespace cm = control_manager::core;
TEST(Guidance, GpsRequiresBothInputs) {
  for (bool localization : {false, true}) {
    for (bool path : {false, true}) {
      const auto result = cm::evaluateGuidance(0, localization, path, "");
      EXPECT_EQ(result.valid, localization && path);
      EXPECT_EQ(result.actual_mode.has_value(), result.valid);
      EXPECT_FALSE(result.degraded);
      if (result.valid) {
        EXPECT_EQ(*result.actual_mode, cm::GuidanceMode::GPS);
        EXPECT_EQ(result.active_source, "GLOBAL_PATH");
      } else EXPECT_EQ(result.active_source, "NONE");
    }
  }
}
TEST(Guidance, UnsupportedNeverFallsBackAndRetainsRequest) {
  for (auto requested : {1, 2}) {
    for (bool localization : {false, true}) {
      for (bool path : {false, true}) {
        const auto result = cm::evaluateGuidance(requested, localization, path, "");
        EXPECT_EQ(result.requested_mode, requested);
        EXPECT_FALSE(result.valid);
        EXPECT_FALSE(result.actual_mode.has_value());
        EXPECT_FALSE(result.degraded);
        EXPECT_EQ(result.active_source, "NONE");
        EXPECT_EQ(result.reason, requested == 1 ? "LANE_NOT_IMPLEMENTED" :
          !localization ? "GLOBAL_POSE_UNAVAILABLE" : !path ? "GLOBAL_PATH_UNAVAILABLE" :
          "HYBRID_NOT_IMPLEMENTED");
      }
    }
  }
}
TEST(Guidance, RejectEveryInvalidWireValue) {
  for (int value = 3; value <= 255; ++value) {
    const auto result = cm::evaluateGuidance(value, true, true, "");
    EXPECT_EQ(result.requested_mode, value);
    EXPECT_FALSE(result.valid);
    EXPECT_FALSE(result.actual_mode.has_value());
    EXPECT_EQ(result.reason, "INVALID_GUIDANCE_MODE");
  }
}
TEST(Guidance, ControlModeSelectionRemainsIndependent) {
  const cm::Command manual{0.1, 0.4, false, true, 1, "MANUAL_FIXTURE"};
  for (int request : {0, 1, 2, 255}) {
    const auto guidance = cm::evaluateGuidance(request, true, true, "");
    const auto automatic = guidance.valid ? cm::Command{0.2, 1, false, true, 1, "AUTO_FIXTURE"}
                                         : cm::stop(1, guidance.reason);
    const auto chosen = cm::selectCommand(cm::ControlMode::MANUAL, manual, automatic, 1);
    EXPECT_EQ(chosen.target_speed_mps, manual.target_speed_mps);
    EXPECT_TRUE(chosen.valid);
    EXPECT_TRUE(cm::selectCommand(cm::ControlMode::IDLE, manual, automatic, 1).target_brake);
    EXPECT_EQ(cm::selectCommand(cm::ControlMode::AUTO, manual, automatic, 1).valid, guidance.valid);
  }
}

TEST(Guidance, LaneValidationDoesNotDependOnPoseOrMap) {
  for (bool pose : {false, true}) for (bool path : {false, true}) {
    for (const auto &error : {"LANE_POSE_UNAVAILABLE", "LANE_CONFIDENCE_LOW", "LANE_VALUES_INVALID"}) {
      EXPECT_EQ(cm::evaluateGuidance(1, pose, path, error).reason, error);
      EXPECT_EQ(cm::evaluateGuidance(0, pose, path, error).valid, pose && path);
    }
    EXPECT_EQ(cm::evaluateGuidance(1, pose, path, "").reason, "LANE_NOT_IMPLEMENTED");
  }
}
TEST(Guidance, HybridRequiresEveryInputAndStillDoesNotClaimSuccess) {
  EXPECT_EQ(cm::evaluateGuidance(2, false, true, "").reason, "GLOBAL_POSE_UNAVAILABLE");
  EXPECT_EQ(cm::evaluateGuidance(2, true, false, "").reason, "GLOBAL_PATH_UNAVAILABLE");
  EXPECT_EQ(cm::evaluateGuidance(2, true, true, "LANE_CONFIDENCE_LOW").reason, "LANE_CONFIDENCE_LOW");
  const auto result = cm::evaluateGuidance(2, true, true, "");
  EXPECT_EQ(result.reason, "HYBRID_NOT_IMPLEMENTED");
  EXPECT_FALSE(result.valid); EXPECT_FALSE(result.actual_mode.has_value());
}
