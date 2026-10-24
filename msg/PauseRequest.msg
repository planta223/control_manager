# One-shot AUTO pause/resume request. Old/future/malformed requests are rejected.
# Empty frame_id or base_frame is accepted. The accepted state remains latched.
std_msgs/Header header
bool pause
# pause=false, motor_zero=true: latch motor 0/brake OFF while tracking steering.
# pause=true always requires motor_zero=false; contradictory requests are rejected.
bool motor_zero
