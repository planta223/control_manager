# Configured startup guidance_mode: GPS / LANE / HYBRID (0, 1, 2).
uint8 GPS=0
uint8 LANE=1
uint8 HYBRID=2
uint8 NONE=255
# Planning report time; same stamp as the associated ReferencePath.
std_msgs/Header header
uint8 requested_mode
# NONE means no valid guidance is active; it is not a selectable startup mode.
uint8 actual_mode
# Current values: NONE, GLOBAL_PATH. Future: LANE, GPS_LANE.
string active_source
bool valid
# True only for an implemented, valid fallback. Always false in this version.
bool degraded
string reason
