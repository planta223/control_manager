# Body-frame longitudinal speed, independent of GlobalPose/Guidance mode.
# Source measurement time, not publish time. Invalid yaw rate must not be consumed.
std_msgs/Header header
float64 speed_mps
float64 yaw_rate_radps
bool speed_valid
bool yaw_rate_valid
string reason
