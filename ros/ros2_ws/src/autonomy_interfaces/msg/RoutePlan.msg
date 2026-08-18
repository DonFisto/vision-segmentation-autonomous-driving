uint8 STATUS_INVALID=0
uint8 STATUS_VALID=1
uint8 STATUS_PARTIAL=2
uint8 STATUS_BLOCKED=3

std_msgs/Header header

uint64 route_id
uint64 map_revision

geometry_msgs/PoseStamped start
geometry_msgs/PoseStamped goal

uint64[] lane_segment_ids
nav_msgs/Path coarse_reference_path

uint8 status
float32 confidence
