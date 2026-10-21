# observation only; exact geometry used by Pure Pursuit.
# header.frame_id is the path/map frame. Invalid/paused/bootstrap states clear valid.
std_msgs/Header header
builtin_interfaces/Time pose_stamp
builtin_interfaces/Time motion_stamp
bool valid
string reason
float64 speed_mps
float64 used_yaw_rad
bool heading_held
uint32 matched_segment
float64 matched_fraction
float64 lateral_error_m
float64 heading_error_rad
float64 target_x_m
float64 target_y_m
float64 speed_lookahead_m
float64 effective_lookahead_m
float64 preview_curvature_per_m
float64 minimum_lookahead_m
# Raw PP solution, before vehicle limits / stop steering hold; compare auto_cmd for applied command.
float64 steering_rad
