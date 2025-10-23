# Goal specification for autonomous typing task

std_msgs/Header header

# Target specification
string target_text      # Text to type on keyboard
geometry_msgs/PoseStamped keyboard_pose  # Location of keyboard in world frame

# Performance requirements
float32 accuracy_requirement  # Minimum accuracy (0.0 to 1.0)
float32 speed_requirement     # Minimum typing speed (chars/min)
float32 timeout               # Maximum time allowed (seconds)

# Keyboard layout information
string keyboard_layout   # "qwerty", "azerty", etc.
float32 key_spacing_m    # Distance between key centers (meters)
geometry_msgs/Vector3 key_dimensions  # Width, height, depth of keys

# Approach parameters
float32 standoff_distance # Distance to approach keyboard (meters)
float32 contact_force     # Desired contact force (Newtons)
