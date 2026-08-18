uint8 TOPOLOGY_UNKNOWN=0
uint8 TOPOLOGY_PARTIAL=1
uint8 TOPOLOGY_CONFIRMED=2

uint8 TURN_UNKNOWN=0
uint8 TURN_STRAIGHT=1
uint8 TURN_LEFT=2
uint8 TURN_RIGHT=3
uint8 TURN_UTURN=4

uint64 id

autonomy_interfaces/LaneSample[] centerline
autonomy_interfaces/LaneBoundary left_boundary
autonomy_interfaces/LaneBoundary right_boundary

uint64[] predecessor_ids
uint64[] successor_ids

uint64 left_neighbor_id
uint64 right_neighbor_id

bool lane_change_left_allowed
bool lane_change_right_allowed

float32 nominal_width_m
bool speed_limit_known
float32 speed_limit_mps
float32 confidence

uint8 topology_state
uint8 turn_direction

bool drivable
bool is_intersection
