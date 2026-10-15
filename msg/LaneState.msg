# External Lane AI result, consumed directly by path_planning (no raw-sensor relay).
# header.stamp: observation time. Confidence must be finite in [0, 1].
# Error sign/reference semantics require agreement with Lane AI before path generation.
# No heading/curvature/polyline is supplied by this contract.
std_msgs/Header header
float64 lateral_error_m
float64 longitudinal_error_m
float64 confidence
