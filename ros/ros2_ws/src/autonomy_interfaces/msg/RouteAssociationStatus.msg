# Observability only; never an authorization to drive.
# Intervals use global RoutePlan arc length in metres.

std_msgs/Header header

uint64 mission_id
uint64 route_id
uint64 route_map_revision

uint64 map_session_id
uint64 lane_map_revision
uint64 evidence_revision

bool selected_segment_valid
uint64 selected_segment_id

string decision_status
string decision_reason
string temporal_status
string temporal_reason
string selection_reason
string provenance_status
string evidence_time_status

bool metrics_valid
float32 median_abs_lateral_m
float32 median_heading_error_deg
float32 coverage_ratio_ahead
float32 supported_ahead_m
float32 continuous_from_ego_m

float32 evidence_age_s
float32 temporal_observation_age_s

uint32 consecutive_supported
uint32 candidate_count
uint32 supported_candidate_count

float32 ego_s_m
float32 window_start_s_m
float32 window_end_s_m

float32[] supported_interval_start_s_m
float32[] supported_interval_end_s_m
