# INTERNAL normalized actual feedback; NOT a declared external hardware wire contract.
# header.stamp is a common acquisition time for the valid fields, never publication time.
# A future adapter must agree how to time-align separate motor/steering observations;
# do not relabel an older field with a newer stamp. Unsynchronized fields stay invalid.
# header.frame_id: configured base_frame. Signed forward speed, left-positive road-wheel angle.
std_msgs/Header header
float64 speed_mps
float64 steering_angle_rad
bool speed_valid
bool steering_valid
