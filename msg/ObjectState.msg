# Unknown is not clear. Per-field validity is mandatory for consumers.
std_msgs/Header header
uint8 UNKNOWN=0
uint8 RED=1
uint8 YELLOW=2
uint8 GREEN=3
uint8 traffic_light_state
bool traffic_light_valid
float64 speed_limit_mps
bool speed_limit_valid
bool obstacle_detected
float64 nearest_obstacle_distance_m
bool obstacle_valid
bool valid
