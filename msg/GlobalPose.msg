# Global/map pose at source measurement time; valid means position AND yaw are usable.
# Marginal position/yaw variances preserve the GNSS quality gate (not a full EKF covariance).
std_msgs/Header header
float64 x_m
float64 y_m
float64 yaw_rad
float64 position_variance_x_m2
float64 position_variance_y_m2
float64 yaw_variance_rad2
# Position-only admission for bootstrap / control-local stop yaw memory; never makes yaw valid.
bool position_valid
# GNSS has never supplied an observed heading and currently marks yaw unobserved.
bool initial_heading_unavailable
# Fresh, well-formed GNSS orientation whose yaw variance exceeds the quality limit.
# May be true after a long stop. Does not make yaw valid or authorize straight driving.
bool heading_unavailable
bool valid
string reason
