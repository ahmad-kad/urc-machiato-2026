# SLAM subsystem status and pose estimate

std_msgs/Header header

# SLAM state
string state  # "initializing", "tracking", "lost", "relocalizing", "mapping"

# Pose estimate with covariance
geometry_msgs/PoseWithCovarianceStamped pose

# Map information
nav_msgs/OccupancyGrid local_map
int32 map_width         # Map dimensions
int32 map_height
float32 map_resolution  # meters per pixel

# Performance metrics
float32 loop_closure_confidence  # 0.0 to 1.0
int32 keyframes_tracked         # Number of keyframes in map
int32 landmarks_tracked         # Number of landmarks/features tracked

# Quality indicators
float32 tracking_quality        # 0.0 (lost) to 1.0 (excellent tracking)
bool loop_closure_detected      # True if recent loop closure
float32 drift_estimate          # Estimated position drift in meters
