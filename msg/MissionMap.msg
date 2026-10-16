# Static, atomic route + task_sections snapshot; same projection as global_path.
# Empty path withdraws the map. Parallel section arrays must have equal lengths.
std_msgs/Header header
nav_msgs/Path path
float64[] stations_m
string[] section_names
float64[] section_start_s_m
float64[] section_end_s_m
