# Single object detection result
# Used in DetectionArray for multiple detections

std_msgs/Header header

# Detection metadata
string class_name        # Object class ("aruco_marker", "keyboard", "target", etc.)
int32 class_id          # Class identifier
float32 confidence      # Detection confidence (0.0 to 1.0)

# Object pose and size
geometry_msgs/PoseStamped pose  # Object position and orientation
geometry_msgs/Vector3 size      # Object dimensions (width, height, depth)

# Vision-specific data
float32[] keypoints     # Feature points (x,y coordinates)
string detector_type    # "aruco", "yolo", "template_matching", etc.

# Tracking information
int32 track_id         # Object track ID for multi-frame tracking
float32 age            # Seconds since first detection
