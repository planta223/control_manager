#include <cmath>
#include <limits>
#include <gtest/gtest.h>
#include "control_manager/core/route_progress.hxx"
#include "control_manager/core/tracking.hxx"
namespace cm=control_manager::core;
namespace {
const std::vector<cm::Point> route{{0,-10},{0,-5},{0,0},{0,5},{-5,5},{-10,5},
  {-10,15},{10,15},{10,5},{5,5},{0,0},{-5,-5},{-10,-5}};
cm::RouteProgressConfig profile() { return {route.size(),{4,9},6.0,1.0,8.0}; }
cm::PidConfig pidConfig() {
  cm::PidConfig c; c.enabled=true; c.kp_lateral=0.2; c.kp_heading=0.4;
  c.max_steering_rad=0.6; return c;
}
}
TEST(RouteProgress, BothControllersFollowTwoVisitsAndOnlyTheRealEndpointIsGoal) {
  for (std::uint8_t mode : {0,1}) {
    cm::RouteProgressTracker progress(profile());
    cm::TrackingController controller(pidConfig(),{}); controller.request(mode);
    auto initial=progress.update(0,-10,std::acos(-1.0)/2,route,3.0);
    ASSERT_TRUE(initial.match.valid);
    double station=0.0,previous=0.0;
    bool first_cross=false,second_cross=false;
    for (std::size_t i=0;i+1<route.size();++i) {
      const double dx=route[i+1].x_m-route[i].x_m,dy=route[i+1].y_m-route[i].y_m;
      const double length=std::hypot(dx,dy);
      for (int n=1;n<=20;++n) {
        const double fraction=n/20.0;
        double yaw=std::atan2(dy,dx);
        if (n==20 && i+2<route.size()) {
          const double nx=route[i+2].x_m-route[i+1].x_m,ny=route[i+2].y_m-route[i+1].y_m;
          const double next_length=std::hypot(nx,ny);
          yaw=std::atan2(dy/length+ny/next_length,dx/length+nx/next_length);
        }
        cm::TrackingState state;
        state.x_m=route[i].x_m+fraction*dx; state.y_m=route[i].y_m+fraction*dy;
        state.yaw_rad=yaw; state.speed_mps=1.0; state.localization_valid=state.speed_valid=true;
        const auto matched=progress.update(state.x_m,state.y_m,yaw,route,3.0);
        ASSERT_TRUE(matched.match.valid)<<matched.reason<<" segment="<<i;
        EXPECT_GE(matched.progress_m,previous);
        EXPECT_NEAR(matched.progress_m,station+fraction*length,1e-8);
        previous=matched.progress_m;
        const auto control=controller.compute(state,route,0.05,true,&matched.match);
        ASSERT_TRUE(control.valid)<<control.reason<<" mode="<<int(mode)<<" segment="<<i;
        if (i==1 && n==20) { first_cross=true; EXPECT_LT(matched.match.segment,4U); EXPECT_EQ(matched.phase,0U); }
        if (i==9 && n==20) { second_cross=true; EXPECT_GE(matched.match.segment,9U); EXPECT_EQ(matched.phase,2U); }
        if ((i==3 || i==8) && n==20) { EXPECT_FALSE(control.goal_reached); }
        if (i+2==route.size() && n==20) { EXPECT_TRUE(control.goal_reached); }
      }
      station+=length;
    }
    EXPECT_TRUE(first_cross && second_cross);
  }
}
TEST(RouteProgress, JitterAndFailedInputsDoNotEraseProgress) {
  cm::RouteProgressTracker progress(profile());
  ASSERT_TRUE(progress.update(0,-10,std::nullopt,route,3.0).match.valid);
  const auto ahead=progress.update(0,-4,std::nullopt,route,3.0);
  ASSERT_TRUE(ahead.match.valid);
  const auto jitter=progress.update(0,-4.5,std::nullopt,route,3.0);
  EXPECT_DOUBLE_EQ(jitter.progress_m,ahead.progress_m);
  EXPECT_FALSE(progress.update(100,100,std::nullopt,route,3.0).match.valid);
  EXPECT_FALSE(progress.update(0,-4,std::nullopt,{},3.0).match.valid);
  auto changed=route; changed.back().x_m-=1;
  EXPECT_EQ(progress.update(0,-4,std::nullopt,changed,3.0).reason,"ROUTE_CHANGED_RESTART_REQUIRED");
  const auto recovered=progress.update(0,-4,std::nullopt,route,3.0);
  EXPECT_TRUE(recovered.match.valid); EXPECT_DOUBLE_EQ(recovered.progress_m,ahead.progress_m);
  EXPECT_EQ(progress.update(0,-4,-std::acos(-1.0)/2,route,3.0).reason,"ROUTE_HEADING_MISMATCH");
}
TEST(RouteProgress, CannotInitializeAtTheLaterCrossingOrSkipToADistantSection) {
  cm::RouteProgressTracker progress(profile());
  EXPECT_FALSE(progress.update(0,0,std::nullopt,route,3.0).match.valid);  // 10 m from start, outside 6+3.
  ASSERT_TRUE(progress.update(0,-10,std::nullopt,route,3.0).match.valid);
  EXPECT_FALSE(progress.update(10,15,std::nullopt,route,3.0).match.valid);
  EXPECT_DOUBLE_EQ(progress.update(0,-10,std::nullopt,route,3.0).progress_m,0.0);
}
TEST(RouteProgress, RejectsInvalidProfileAndExternalMatches) {
  EXPECT_THROW(cm::RouteProgressTracker({}),std::invalid_argument);
  auto c=profile(); c.phase_end_indices={9,4}; EXPECT_THROW(cm::RouteProgressTracker{c},std::invalid_argument);
  c=profile(); c.search_forward_m=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(cm::RouteProgressTracker{c},std::invalid_argument);
  cm::TrackingState state; state.localization_valid=state.speed_valid=true;
  cm::PathMatch bad{true,999,0.0,0.0};
  EXPECT_FALSE(cm::pathError(state,route,&bad).valid);
  EXPECT_FALSE(cm::AdaptivePurePursuit({}).compute(state,route,bad).valid);
  const std::vector<cm::Point> degenerate{{0,0},{0,0},{1,0}};
  const cm::PathMatch repeated{true,0,0.0,0.0};
  EXPECT_FALSE(cm::AdaptivePurePursuit({}).compute(state,degenerate,repeated).valid);
}
