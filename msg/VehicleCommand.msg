# Physical front road-wheel angle: radians, left positive. NEVER actuator raw.
# header.stamp is source command time; arbitration MUST preserve it.
std_msgs/Header header
float64 target_steer_rad
float64 target_speed_mps
bool target_brake
bool valid
string reason
