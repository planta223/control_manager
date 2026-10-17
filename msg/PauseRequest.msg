# One-shot AUTO pause/resume request. Old/future/malformed requests are rejected.
# Empty frame_id or base_frame is accepted. The accepted state remains latched.
std_msgs/Header header
bool pause
