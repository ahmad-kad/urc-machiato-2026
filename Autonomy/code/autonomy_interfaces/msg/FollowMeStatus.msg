# FollowMeStatus.msg
# Status information for follow me mode

std_msgs/Header header

# Follow me state
bool is_following  # Whether currently following
int32 target_tag_id  # ID of tag being followed
float32 target_distance  # Current distance to target
float32 target_angle  # Current angle to target (radians)

# Safety information
float32 safety_distance  # Configured safety distance
bool safety_violation  # Whether safety distance is violated
float32 current_speed  # Current following speed

# Target tracking
geometry_msgs/Point target_position  # 3D position of target
bool target_visible  # Whether target is currently visible
float32 last_detection_time  # Time since last detection (seconds)

# Control parameters
float32 max_speed  # Maximum allowed speed
string operator_id  # ID of controlling operator
