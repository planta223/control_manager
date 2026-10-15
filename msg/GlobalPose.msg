# Global/map pose at source measurement time; valid=false means unusable.
# Marginal position/yaw variances preserve the GNSS quality gate (not a full EKF covariance).
std_msgs/Header header
float64 x_m
float64 y_m
float64 yaw_rad
float64 position_variance_x_m2
float64 position_variance_y_m2
float64 yaw_variance_rad2
bool valid
string reason
