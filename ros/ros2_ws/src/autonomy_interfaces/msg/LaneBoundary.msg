uint8 SIDE_UNKNOWN=0
uint8 SIDE_LEFT=1
uint8 SIDE_RIGHT=2

uint8 SOURCE_DIRECT=0
uint8 SOURCE_INFERRED=1
uint8 SOURCE_MEMORY=2

uint64 id
uint8 side
uint8 source

geometry_msgs/Point32[] points
float32[] point_confidence

float32 mean_confidence
bool complete
