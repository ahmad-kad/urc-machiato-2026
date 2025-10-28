# ArmAlignmentCommand.msg
# Command for arm alignment based on ArUco tag detection

std_msgs/Header header

# Mission information
string mission_type  # Mission type: "AUTONOMOUS_TYPING", "USB_CONNECTION", "PANEL_OPERATIONS"
string alignment_id  # Unique identifier for this alignment operation

# Target information
geometry_msgs/Point target_position  # Target position for arm end-effector
geometry_msgs/Quaternion target_orientation  # Target orientation for arm end-effector
float32 approach_distance  # Distance to maintain from target during approach
float32 final_distance  # Final distance from target for operation

# Alignment parameters
float32 alignment_quality  # Quality score of alignment (0.0-1.0)
float32 max_position_error  # Maximum allowed position error (meters)
float32 max_orientation_error  # Maximum allowed orientation error (radians)
float32 alignment_timeout  # Timeout for alignment operation (seconds)

# Safety parameters
float32 max_approach_speed  # Maximum approach speed (m/s)
float32 max_rotation_speed  # Maximum rotation speed (rad/s)
bool enable_collision_avoidance  # Whether to enable collision avoidance
string[] safety_zones  # Safety zones to avoid during movement

# Feedback requirements
bool require_position_feedback  # Whether to require position feedback during alignment
bool require_force_feedback  # Whether to require force feedback during alignment
float32 feedback_rate  # Rate for feedback updates (Hz)

# Mission-specific parameters
string[] required_aruco_tags  # ArUco tags that must remain visible during operation
float32 tag_visibility_timeout  # Timeout for tag visibility loss (seconds)
bool allow_realignment  # Whether to allow realignment if tags are lost
