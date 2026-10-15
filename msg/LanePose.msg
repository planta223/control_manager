# Vehicle pose relative to the lane, supplied by External Lane AI.
# The same four-field wire shape is used for observation and validated pose.
# sensor_interface -> lane_observation -> lane_state_estimator -> lane_pose -> planning.
# The optional external LanePose adapter preserves these fields unchanged.
# header.stamp: observation time. Confidence must be finite in [0, 1].
# Error sign/reference semantics require agreement with Lane AI before path generation.
# lateral_error_m: lane-relative lateral error in metres.
# heading_error_rad: lane-relative heading error in radians, supplied unchanged.
# No curvature, polyline or lane geometry is supplied by this contract.
std_msgs/Header header
float64 lateral_error_m
float64 heading_error_rad
float64 confidence
