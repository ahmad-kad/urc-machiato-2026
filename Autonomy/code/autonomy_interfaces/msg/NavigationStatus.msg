# Navigation subsystem status
# Provides current navigation state and progress

std_msgs/Header header

# Navigation state machine
string state  # "idle", "planning", "navigating", "avoiding_obstacle", "precision_approach", "arrived", "error"

# Progress indicators
float32 mission_progress  # 0.0 to 1.0, overall mission completion
int32 current_waypoint    # Current waypoint index (0-based)
int32 total_waypoints     # Total waypoints in mission

# Current position and goal
geometry_msgs/PoseStamped current_pose
geometry_msgs/PoseStamped goal_pose

# Performance metrics
float32 distance_to_goal  # meters, negative if no active goal
float32 speed             # m/s, current linear speed
float32 heading_error     # radians, difference between desired and actual heading

# Status message for operators
string status_message
