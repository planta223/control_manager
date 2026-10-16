# Selection: operation.yaml tracking_mode at startup. Independent of Guidance/ControlMode.
uint8 PID=0
uint8 PURE_PURSUIT=1
uint8 ADAPTIVE_PURE_PURSUIT=2
uint8 NONE=255
std_msgs/Header header
# Configured startup mode; unsupported algorithms never silently fall back.
uint8 requested_mode
# NONE when steering was not computed (also PLAN_STOP / invalid inputs).
uint8 actual_mode
# Valid computation, not permission to drive or measured steering feedback.
bool valid
string reason
