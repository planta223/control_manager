# Request: std_msgs/UInt8 on guidance_mode_request (only 0, 1, 2).
uint8 GPS=0
uint8 LANE=1
uint8 HYBRID=2
uint8 NONE=255
# Planning report time; same stamp as the associated ReferencePath.
std_msgs/Header header
uint8 requested_mode
# NONE means no valid guidance is active. Invalid requests are preserved above.
uint8 actual_mode
# Current values: NONE, GLOBAL_PATH. Future: LANE, GPS_LANE.
string active_source
bool valid
# True only for an implemented, valid fallback. Always false in this version.
bool degraded
string reason
